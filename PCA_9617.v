// =============================================================================
// PCA9617A Level-Shifting I2C Bus Repeater — RTL Model
// Submodules: power_monitor, power_on_reset_delay, port_A_low_detect,
//             port_B_low_detect, direction_propagation_logic,
//             propagation_delay_model, enable_controller, PCA9617A_TOP
// All voltages represented as Q20 fixed-point integers (1V = 2^20 = 1048576).
// =============================================================================


// =============================================================================
// Module: power_monitor
// Debounces the combinational power_good_raw signal over DEBOUNCE_CYCLES clock
// cycles to suppress glitches before feeding downstream logic.
// =============================================================================
module power_monitor #(
    parameter integer DEBOUNCE_CYCLES = 20000   // # cycles input must be stable before output changes
)(
    input  wire clk,
    input  wire rst_n,
    input  wire power_good_raw,  // combinational power-good from voltage comparators
    output reg  power_good       // debounced, synchronised output
);

    // Elaboration-time guard: counter width would be 0 if DEBOUNCE_CYCLES < 1
    // synthesis translate_off
    initial begin
        if (DEBOUNCE_CYCLES < 1)
            $fatal(1, "power_monitor: DEBOUNCE_CYCLES must be >= 1");
    end
    // synthesis translate_on

    reg [1:0] sync_ff;  // 2-FF synchroniser to cross power_good_raw into clk domain
    // Stage 1: double-flop synchroniser — eliminates metastability on async input
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sync_ff <= 2'b00;
        else
            sync_ff <= {sync_ff[0], power_good_raw};  // shift: [1] is stable output
    end

    wire raw_sync = sync_ff[1];  // synchronised (but unfiltered) power_good
    localparam integer CNT_WIDTH = $clog2(DEBOUNCE_CYCLES + 1);  // minimum bits for counter
    reg [CNT_WIDTH-1:0] debounce_cnt;

    // Stage 2: debounce counter — output changes only after DEBOUNCE_CYCLES consecutive
    // stable cycles; any disagreement between raw_sync and current output resets the counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_cnt <= {CNT_WIDTH{1'b0}};
            power_good   <= 1'b0;
        end else begin
            if (raw_sync == power_good) begin
                // Input matches output: no transition pending, reset counter
                debounce_cnt <= {CNT_WIDTH{1'b0}};
            end else if (debounce_cnt == DEBOUNCE_CYCLES[CNT_WIDTH-1:0] - 1'b1) begin
                // Counter reached threshold: commit the new output value
                power_good   <= raw_sync;
                debounce_cnt <= {CNT_WIDTH{1'b0}};
            end else begin
                // Counting towards threshold
                debounce_cnt <= debounce_cnt + 1'b1;
            end
        end
    end

endmodule


// =============================================================================
// Module: power_on_reset_delay
// Holds por_done low for at least POR_DELAY_US microseconds after both supply
// rails (VCCA_ok, VCCB_ok) become valid.  Any supply drop resets the counter.
// =============================================================================
module power_on_reset_delay #(
    parameter integer CLK_FREQ_HZ  = 50_000_000,  // system clock frequency
    parameter integer POR_DELAY_US = 400  )(      // minimum POR hold time in µs
    input  wire clk,
    input  wire rst_n,
    input  wire VCCA_ok,    // VCCA within valid range (from top-level comparators)
    input  wire VCCB_ok,    // VCCB within valid range
    output reg  por_done ); // asserts when POR delay has elapsed with supplies valid

    // synthesis translate_off
    initial begin
        if (CLK_FREQ_HZ < 1)
            $fatal(1, "power_on_reset_delay: CLK_FREQ_HZ must be >= 1");
        if (POR_DELAY_US < 1)
            $fatal(1, "power_on_reset_delay: POR_DELAY_US must be >= 1");
    end
    // synthesis translate_on

    // Convert POR_DELAY_US to clock cycles using integer picosecond arithmetic
    // (avoids tool-dependent real-number handling in synthesis)
    localparam integer CLK_PERIOD_PS  = (64'd1_000_000_000_000 / CLK_FREQ_HZ);
    localparam integer POR_CYCLES_RAW = (POR_DELAY_US * 1_000_000 + CLK_PERIOD_PS - 1)/ CLK_PERIOD_PS;  // ceiling division → meet-or-exceed spec
    localparam integer POR_CYCLES     = (POR_CYCLES_RAW > 1) ? POR_CYCLES_RAW : 1;  // clamp to ≥1
    localparam integer CNT_WIDTH      = $clog2(POR_CYCLES + 1);

    reg [1:0] vcca_sync;  // 2-FF synchroniser for VCCA_ok
    reg [1:0] vccb_sync;  // 2-FF synchroniser for VCCB_ok
    // Synchronise both supply-OK signals into the clock domain
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vcca_sync <= 2'b00;
            vccb_sync <= 2'b00;
        end else begin
            vcca_sync <= {vcca_sync[0], VCCA_ok};
            vccb_sync <= {vccb_sync[0], VCCB_ok};
        end
    end

    reg [CNT_WIDTH-1:0] counter;
    // POR counter: counts up while both supplies are valid; resets on any dropout
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter  <= {CNT_WIDTH{1'b0}};
            por_done <= 1'b0;
        end else begin
            if (!vcca_sync[1] || !vccb_sync[1]) begin
                // Either supply has dropped — restart the POR delay
                counter  <= {CNT_WIDTH{1'b0}};
                por_done <= 1'b0;
            end else if (!por_done) begin
                // Both supplies valid and POR not yet complete — keep counting
                if (counter == POR_CYCLES[CNT_WIDTH-1:0] - 1'b1)
                    por_done <= 1'b1;       // delay elapsed: release system
                else
                    counter  <= counter + 1'b1;
            end
            // por_done stays high once set (until reset or supply dropout above)
        end
    end
endmodule


// =============================================================================
// Module: port_A_low_detect
// Detects when the Port-A signal voltage falls below VIL(A) = VCCA/4,
// using a hysteretic comparator to prevent chatter around the threshold.
// Threshold and hysteresis bands are in Q20 fixed-point.
// =============================================================================
module port_A_low_detect #(
    parameter SIGNAL_NAME   = "SDA",       // used in simulation warnings only
    parameter integer HYSTERESIS_Q = 131072  // ±hysteresis in Q20 (≈ ±0.125 V)
)(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] signal_voltage,  // Q20 voltage to monitor
    input  wire [31:0] VCCA_analog,     // Q20 VCCA supply for computing threshold
    input  wire        VCCA_ok,         // gates low_out: no false drive if rail is bad
    output reg         low_out,         // synchronised low detect, gated by VCCA_ok
    output reg         low_detected     // synchronised low detect, ungated (for status)
);

    reg        last_state;  // hysteretic comparator state: 1 = signal is low
    reg [1:0]  sync_ff;     // 2-FF pipeline to align comparator output with clock

    // VIL threshold = VCCA/4 (right-shift by 2 in Q20 domain)
    wire [31:0] threshold      = VCCA_analog >> 2;
    wire [31:0] threshold_low  = threshold - HYSTERESIS_Q[31:0];  // falling edge trip point
    wire [31:0] threshold_high = threshold + HYSTERESIS_Q[31:0];  // rising edge trip point

    wire comparator_out = last_state;  // readable alias

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_state   <= 1'b0;
            sync_ff      <= 2'b00;
            low_detected <= 1'b0;
            low_out      <= 1'b0;
        end else begin
            // Hysteretic comparator: only changes state when crossing either band edge
            // (avoids glitching at the decision boundary)
            if      (signal_voltage > threshold_high)  last_state <= 1'b0;  // signal high → clear
            else if (signal_voltage < threshold_low)   last_state <= 1'b1;  // signal low  → set
            // else: inside hysteresis band → retain last_state

            // 2-cycle pipeline register: reduces combinational path from comparator to output
            sync_ff <= {sync_ff[0], comparator_out};

            low_detected <= sync_ff[1];               // registered output, ungated
            low_out      <= VCCA_ok & sync_ff[1];     // gated: inactive if rail is invalid

            // Warn in simulation when signal is in the marginal region (threshold < v ≤ threshold_high)
            // synthesis translate_off
            if (VCCA_ok &&
                signal_voltage >  threshold      &&
                signal_voltage <= threshold_high) begin
                $display("WARNING [%0t] %s_A voltage in marginal region",
                         $time, SIGNAL_NAME);
            end
            // synthesis translate_on
        end
    end

endmodule


// =============================================================================
// Module: port_B_low_detect
// Detects when the Port-B signal falls below a fixed VIL(B) threshold
// (not ratiometric: B-side uses a supply-independent comparator level).
// Hysteresis prevents chatter.  Same 2-FF pipeline as port_A_low_detect.
// =============================================================================
module port_B_low_detect #(
    parameter SIGNAL_NAME    = "SDA",
    parameter integer THRESHOLD_Q  = 419430,  // VIL(B) centre in Q20 (≈ 0.40 V)
    parameter integer HYSTERESIS_Q = 209715   // ±hysteresis in Q20 (≈ ±0.20 V)
)(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] signal_voltage,
    input  wire        VCCB_ok,       // gates low_out
    output reg         low_out,
    output reg         low_detected
);

    // Fixed (non-ratiometric) trip bands derived from parameters
    wire [31:0] threshold_low  = THRESHOLD_Q[31:0]  - HYSTERESIS_Q[31:0];  // ≈ 0.20 V Q20
    wire [31:0] threshold_high = THRESHOLD_Q[31:0]  + HYSTERESIS_Q[31:0];  // ≈ 0.60 V Q20

    reg        last_state;          // hysteretic comparator state
    wire       comparator_out = last_state;
    reg [1:0]  sync_ff;             // 2-FF pipeline

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_state   <= 1'b0;
            sync_ff      <= 2'b00;
            low_detected <= 1'b0;
            low_out      <= 1'b0;
        end else begin
            // Hysteretic comparator (strict inequalities: boundary = no change)
            if      (signal_voltage > threshold_high)  last_state <= 1'b0;
            else if (signal_voltage < threshold_low)   last_state <= 1'b1;

            sync_ff <= {sync_ff[0], comparator_out};

            low_detected <= sync_ff[1];
            low_out      <= VCCB_ok & sync_ff[1];  // suppress drive output if VCCB is invalid
        end
    end

endmodule


// =============================================================================
// Module: direction_propagation_logic
// Determines which bus side should be driven low based on which side pulled
// the bus down first.  Detects and latches bus arbitration loss when both
// sides are pulled low simultaneously or in rapid succession.
// =============================================================================
module direction_propagation_logic #(
    parameter SIGNAL_NAME = "SDA" )(
    input  wire clk,
    input  wire rst_n,
    input  wire low_A,           // port-A detected low (from port_A_low_detect)
    input  wire low_B,           // port-B detected low (from port_B_low_detect)
    input  wire repeater_active, // master enable: all outputs forced 0 when inactive
    output reg  drive_A_intent,  // request to pull port-A low (B→A direction)
    output reg  drive_B_intent,  // request to pull port-B low (A→B direction)
    output reg  arbitration_lost ); // latched: both sides low simultaneously (multi-master conflict)


    reg low_A_prev;  // previous-cycle sample of low_A for edge detection
    reg low_B_prev;  // previous-cycle sample of low_B

    // Rising-edge detectors: asserted for exactly one cycle on the transition 0→1
    wire a_falling = low_A  & ~low_A_prev;  // "falling" = bus just went low on A-side
    wire b_falling = low_B  & ~low_B_prev;  // "falling" = bus just went low on B-side

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            low_A_prev       <= 1'b0;
            low_B_prev       <= 1'b0;
            drive_A_intent   <= 1'b0;
            drive_B_intent   <= 1'b0;
            arbitration_lost <= 1'b0;
        end else begin
            // Capture previous state for next cycle's edge detection
            low_A_prev <= low_A;
            low_B_prev <= low_B;

            if (!repeater_active) begin
                // Repeater disabled: clear all drive and fault signals immediately
                drive_A_intent   <= 1'b0;
                drive_B_intent   <= 1'b0;
                arbitration_lost <= 1'b0;
            end else begin
                // Transparent level copy: activity on one side drives the other
                drive_B_intent <= low_A;  // A pulled low → drive B low (A→B propagation)
                drive_A_intent <= low_B;  // B pulled low → drive A low (B→A propagation)

                // Arbitration-lost detection and latch
                // Set if: new A-edge while B already low, new B-edge while A already low,
                //         or both edges occur in the same cycle (true simultaneous pull).
                // Clear only when both sides have fully released the bus.
                if (!arbitration_lost) begin
                    if ((a_falling && low_B) ||
                        (b_falling && low_A) ||
                        (a_falling && b_falling)) begin
                        arbitration_lost <= 1'b1;  // latch conflict
                    end
                end else begin
                    if (!low_A && !low_B)
                        arbitration_lost <= 1'b0;  // release latch when bus fully idle
                end
            end
        end
    end

endmodule


// =============================================================================
// Module: propagation_delay_model
// Introduces the datasheet-specified tPHL and tPLH propagation delays between
// the drive_X_intent signals and the actual drive_X_delayed outputs.
//
// Pipeline overhead (7 cycles at 50 MHz = 140 ns) accounts for:
//   port_X_low_detect : comparator reg (1) + sync_ff[0] (1) + sync_ff[1] (1) = 3 cycles
//   direction_prop_logic: low_X_prev reg (1) + output reg (1)                 = 2 cycles
//   this module        : drive_X_prev edge-detect reg (1)                     = 1 cycle
//   Total                                                                      = 7 cycles (not used directly;
//                                                                                subtracted from raw delay target)
//
// Each side has independent pull (PHL) and release (PLH) countdown timers.
// A new pull cancels a pending release, and vice versa, ensuring output
// faithfully tracks rapid toggling.
// =============================================================================
module propagation_delay_model #(
    parameter SIGNAL_NAME          = "SDA",
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer PHL_A2B_NS   = 173,  // A→B propagation High-to-Low (pull) spec in ns
    parameter integer PLH_A2B_NS   = 102,  // A→B propagation Low-to-High (release) spec in ns
    parameter integer PHL_B2A_NS   = 152,  // B→A pull spec in ns
    parameter integer PLH_B2A_NS   = 103   // B→A release spec in ns
)(
    input  wire clk,
    input  wire rst_n,
    input  wire drive_A_intent,   // from direction_propagation_logic
    input  wire drive_B_intent,
    input  wire repeater_active,
    output reg  drive_A_delayed,  // delayed drive request for A-side output driver
    output reg  drive_B_delayed   // delayed drive request for B-side output driver
);

    // Clock period in picoseconds (integer, avoids real-number issues in synthesis)
    localparam integer CLK_PS = (64'd1_000_000_000_000 / CLK_FREQ_HZ);

    localparam integer PIPELINE_OVERHEAD_CYC = 7;  // see module header for derivation

    // Ceiling-divide each spec delay (ns→ps) by CLK_PS to get raw cycle counts
    localparam integer CYC_PHL_A2B_RAW = (PHL_A2B_NS * 1000 + CLK_PS - 1) / CLK_PS;
    localparam integer CYC_PLH_A2B_RAW = (PLH_A2B_NS * 1000 + CLK_PS - 1) / CLK_PS;
    localparam integer CYC_PHL_B2A_RAW = (PHL_B2A_NS * 1000 + CLK_PS - 1) / CLK_PS;
    localparam integer CYC_PLH_B2A_RAW = (PLH_B2A_NS * 1000 + CLK_PS - 1) / CLK_PS;

    // Subtract pipeline overhead already present in upstream stages; clamp to ≥1
    localparam integer CYC_PHL_A2B = (CYC_PHL_A2B_RAW > PIPELINE_OVERHEAD_CYC + 1)
                                      ? (CYC_PHL_A2B_RAW - PIPELINE_OVERHEAD_CYC) : 1;
    localparam integer CYC_PLH_A2B = (CYC_PLH_A2B_RAW > PIPELINE_OVERHEAD_CYC + 1)
                                      ? (CYC_PLH_A2B_RAW - PIPELINE_OVERHEAD_CYC) : 1;
    localparam integer CYC_PHL_B2A = (CYC_PHL_B2A_RAW > PIPELINE_OVERHEAD_CYC + 1)
                                      ? (CYC_PHL_B2A_RAW - PIPELINE_OVERHEAD_CYC) : 1;
    localparam integer CYC_PLH_B2A = (CYC_PLH_B2A_RAW > PIPELINE_OVERHEAD_CYC + 1)
                                      ? (CYC_PLH_B2A_RAW - PIPELINE_OVERHEAD_CYC) : 1;

    // Counter width: wide enough for the largest single delay target
    localparam integer MAX_A2B = (CYC_PHL_A2B > CYC_PLH_A2B) ? CYC_PHL_A2B : CYC_PLH_A2B;
    localparam integer MAX_B2A = (CYC_PHL_B2A > CYC_PLH_B2A) ? CYC_PHL_B2A : CYC_PLH_B2A;
    localparam integer MAX_CYC = (MAX_A2B > MAX_B2A) ? MAX_A2B : MAX_B2A;
    localparam integer CNT_W   = (MAX_CYC > 1) ? $clog2(MAX_CYC + 1) : 1;

    // Previous-cycle registers for rising/falling edge detection on intent signals
    reg drive_A_prev;
    reg drive_B_prev;

    // Edge signals: one-cycle-wide pulses
    wire rising_A  = ~drive_A_prev &  drive_A_intent;  // intent just went high (pull requested)
    wire falling_A =  drive_A_prev & ~drive_A_intent;  // intent just went low  (release requested)
    wire rising_B  = ~drive_B_prev &  drive_B_intent;
    wire falling_B =  drive_B_prev & ~drive_B_intent;

    // B-side delay state machine registers
    reg [CNT_W-1:0] cnt_B_pull;    // countdown for PHL_A2B (pull delay)
    reg [CNT_W-1:0] cnt_B_rel;     // countdown for PLH_A2B (release delay)
    reg             pending_B_pull; // pull timer is running
    reg             pending_B_rel;  // release timer is running

    // A-side delay state machine registers (symmetric to B-side)
    reg [CNT_W-1:0] cnt_A_pull;
    reg [CNT_W-1:0] cnt_A_rel;
    reg             pending_A_pull;
    reg             pending_A_rel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drive_A_prev    <= 1'b0;
            drive_B_prev    <= 1'b0;
            drive_A_delayed <= 1'b0;
            drive_B_delayed <= 1'b0;
            cnt_B_pull      <= {CNT_W{1'b0}};
            cnt_B_rel       <= {CNT_W{1'b0}};
            cnt_A_pull      <= {CNT_W{1'b0}};
            cnt_A_rel       <= {CNT_W{1'b0}};
            pending_B_pull  <= 1'b0;
            pending_B_rel   <= 1'b0;
            pending_A_pull  <= 1'b0;
            pending_A_rel   <= 1'b0;
        end else begin
            // Capture intent edges for next cycle
            drive_A_prev <= drive_A_intent;
            drive_B_prev <= drive_B_intent;

            if (!repeater_active) begin
                // Hard reset all delay state when repeater is disabled
                drive_A_delayed <= 1'b0;
                drive_B_delayed <= 1'b0;
                pending_B_pull  <= 1'b0;
                pending_B_rel   <= 1'b0;
                pending_A_pull  <= 1'b0;
                pending_A_rel   <= 1'b0;
                cnt_B_pull      <= {CNT_W{1'b0}};
                cnt_B_rel       <= {CNT_W{1'b0}};
                cnt_A_pull      <= {CNT_W{1'b0}};
                cnt_A_rel       <= {CNT_W{1'b0}};
            end else begin

                // ==============================================================
                // B-side state machine  (A→B direction)
                // drive_B_intent rising  → start PHL_A2B countdown → assert drive_B_delayed
                // drive_B_intent falling → start PLH_A2B countdown → deassert drive_B_delayed
                // Rising edge takes priority over a pending release (pre-empts it).
                // Falling edge takes priority over a pending pull   (pre-empts it).
                // ==============================================================

                // B pull phase: intent went high — cancel any pending release, start pull timer
                if (rising_B) begin
                    pending_B_rel  <= 1'b0;
                    pending_B_pull <= 1'b1;
                    cnt_B_pull     <= CYC_PHL_A2B[CNT_W-1:0] - 1'b1;
                end else if (pending_B_pull) begin
                    if (cnt_B_pull == {CNT_W{1'b0}}) begin
                        drive_B_delayed <= 1'b1;  // pull timer expired: assert output
                        pending_B_pull  <= 1'b0;
                    end else begin
                        cnt_B_pull <= cnt_B_pull - 1'b1;
                    end
                end

                // B release phase: intent went low — cancel pending pull, start release timer
                // Guard: only start release if output is or was about to be driven
                if (falling_B) begin
                    if (drive_B_delayed || pending_B_pull) begin
                        pending_B_pull <= 1'b0;
                        pending_B_rel  <= 1'b1;
                        cnt_B_rel      <= CYC_PLH_A2B[CNT_W-1:0] - 1'b1;
                    end
                end else if (pending_B_rel) begin
                    if (cnt_B_rel == {CNT_W{1'b0}}) begin
                        drive_B_delayed <= 1'b0;  // release timer expired: deassert output
                        pending_B_rel   <= 1'b0;
                    end else begin
                        cnt_B_rel <= cnt_B_rel - 1'b1;
                    end
                end

                // ==============================================================
                // A-side state machine  (B→A direction) — symmetric to B-side
                // Uses PHL_B2A / PLH_B2A timing parameters.
                // ==============================================================

                // A pull phase
                if (rising_A) begin
                    pending_A_rel  <= 1'b0;
                    pending_A_pull <= 1'b1;
                    cnt_A_pull     <= CYC_PHL_B2A[CNT_W-1:0] - 1'b1;
                end else if (pending_A_pull) begin
                    if (cnt_A_pull == {CNT_W{1'b0}}) begin
                        drive_A_delayed <= 1'b1;
                        pending_A_pull  <= 1'b0;
                    end else begin
                        cnt_A_pull <= cnt_A_pull - 1'b1;
                    end
                end

                // A release phase
                if (falling_A) begin
                    if (drive_A_delayed || pending_A_pull) begin
                        pending_A_pull <= 1'b0;
                        pending_A_rel  <= 1'b1;
                        cnt_A_rel      <= CYC_PLH_B2A[CNT_W-1:0] - 1'b1;
                    end
                end else if (pending_A_rel) begin
                    if (cnt_A_rel == {CNT_W{1'b0}}) begin
                        drive_A_delayed <= 1'b0;
                        pending_A_rel   <= 1'b0;
                    end else begin
                        cnt_A_rel <= cnt_A_rel - 1'b1;
                    end
                end

            end // repeater_active
        end // rst_n
    end

endmodule


// =============================================================================
// Module: enable_controller
// Controls the repeater_active signal with configurable turn-on and turn-off
// delays.  Enable is gated by: EN input, power_good, por_done, and bus idle.
// Disable waits for bus idle before de-asserting.  Power loss bypasses delays.
// =============================================================================
module enable_controller #(
    parameter integer CLK_FREQ_HZ      = 50_000_000,
    parameter integer ENABLE_DELAY_NS  = 100,  // turn-on  delay in ns
    parameter integer DISABLE_DELAY_NS = 100   // turn-off delay in ns
)(
    input  wire clk,
    input  wire rst_n,
    input  wire EN,             // asynchronous enable request (synchronised internally)
    input  wire power_good,     // from power_monitor (debounced)
    input  wire por_done,       // from power_on_reset_delay
    input  wire sda_a_idle,     // SDA_A bus-idle indicator (combinational from top)
    input  wire scl_a_idle,     // SCL_A bus-idle indicator
    output reg  repeater_active,// main enable output to all datapath submodules
    output wire enable_ready    // status: no pending transition, stable state
);

    // synthesis translate_off
    initial begin
        if (CLK_FREQ_HZ < 1)
            $fatal(1, "enable_controller: CLK_FREQ_HZ must be >= 1");
    end
    // synthesis translate_on

    localparam integer CLK_PERIOD_PS  = (64'd1_000_000_000_000 / CLK_FREQ_HZ);
    // Ceiling-divide delay specs to cycle counts so hardware meets-or-exceeds spec
    localparam integer EN_CYCLES_RAW  = (ENABLE_DELAY_NS  * 1000 + CLK_PERIOD_PS - 1)/ CLK_PERIOD_PS;
    localparam integer DIS_CYCLES_RAW = (DISABLE_DELAY_NS * 1000 + CLK_PERIOD_PS - 1)/ CLK_PERIOD_PS;
    localparam integer EN_CYCLES      = (EN_CYCLES_RAW  > 1) ? EN_CYCLES_RAW  : 1;
    localparam integer DIS_CYCLES     = (DIS_CYCLES_RAW > 1) ? DIS_CYCLES_RAW : 1;
    localparam integer MAX_CYCLES = (EN_CYCLES > DIS_CYCLES) ? EN_CYCLES : DIS_CYCLES;
    localparam integer CNT_WIDTH  = $clog2(MAX_CYCLES + 1);

    reg [1:0] en_sync;    // 2-FF synchroniser for the EN input
    reg       bus_idle_r; // registered AND of sda_a_idle & scl_a_idle (1 cycle latency)
    reg       en_req;     // stable synchronised enable request
    reg                 pending_enable;   // enable delay countdown in progress
    reg                 pending_disable;  // disable delay countdown in progress
    reg [CNT_WIDTH-1:0] en_cnt;
    reg [CNT_WIDTH-1:0] dis_cnt;

    // Register bus-idle to avoid combinational glitches gating the state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bus_idle_r <= 1'b1;  // assume idle at reset
        else
            bus_idle_r <= sda_a_idle & scl_a_idle;
    end

    // Synchronise EN into the clock domain
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_sync <= 2'b00;
            en_req  <= 1'b0;
        end else begin
            en_sync <= {en_sync[0], EN};
            en_req  <= en_sync[1];  // stable, metastability-free enable request
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_enable  <= 1'b0;
            pending_disable <= 1'b0;
            en_cnt          <= {CNT_WIDTH{1'b0}};
            dis_cnt         <= {CNT_WIDTH{1'b0}};
            repeater_active <= 1'b0;
        end else begin

            // Trigger enable sequence: EN asserted, bus idle, power valid, repeater currently off
            if ( en_req && bus_idle_r && power_good && por_done &&
                !repeater_active && !pending_disable && !pending_enable) begin
                pending_enable <= 1'b1;
                en_cnt         <= EN_CYCLES[CNT_WIDTH-1:0] - 1'b1;
            end

            // Trigger disable sequence: EN de-asserted, bus idle, repeater currently on
            if (!en_req && bus_idle_r && repeater_active &&
                !pending_enable && !pending_disable) begin
                pending_disable <= 1'b1;
                dis_cnt         <= DIS_CYCLES[CNT_WIDTH-1:0] - 1'b1;
            end

            // Enable countdown: abort if bus becomes busy during the delay
            if (pending_enable) begin
                if (!bus_idle_r) begin
                    // Bus went busy mid-delay: cancel enable, wait for next idle window
                    pending_enable <= 1'b0;
                    en_cnt         <= {CNT_WIDTH{1'b0}};
                end else if (en_cnt == {CNT_WIDTH{1'b0}}) begin
                    // Delay elapsed: assert repeater only if conditions still met
                    repeater_active <= en_req & power_good & por_done;
                    pending_enable  <= 1'b0;
                end else begin
                    en_cnt <= en_cnt - 1'b1;
                end
            end

            // Disable countdown: abort if bus becomes busy during the delay
            if (pending_disable) begin
                if (!bus_idle_r) begin
                    // Bus went busy: cancel disable, wait for next idle window
                    pending_disable <= 1'b0;
                    dis_cnt         <= {CNT_WIDTH{1'b0}};
                end else if (dis_cnt == {CNT_WIDTH{1'b0}}) begin
                    repeater_active <= 1'b0;
                    pending_disable <= 1'b0;
                end else begin
                    dis_cnt <= dis_cnt - 1'b1;
                end
            end

            // Emergency disable: power dropped while repeater was active — bypass all delays
            if (repeater_active && (!power_good || !por_done)) begin
                repeater_active <= 1'b0;
                pending_enable  <= 1'b0;
                pending_disable <= 1'b0;
                en_cnt          <= {CNT_WIDTH{1'b0}};
                dis_cnt         <= {CNT_WIDTH{1'b0}};
            end

        end
    end

    // enable_ready: combinational status flag — true when no transition is pending
    // and the current state matches the request (useful for external sequencing)
    assign enable_ready = bus_idle_r & ~pending_enable & ~pending_disable &(en_req == repeater_active);
endmodule


// =============================================================================
// Module: PCA9617A_TOP
// Top-level wrapper instantiating the full repeater signal chain for both
// SDA and SCL.  Voltage comparators are implemented as combinational integer
// expressions in Q20 fixed-point; all pipeline logic is in submodules.
//
// Bus-idle definition: signal > 0.7 × VCCA
//   Integer form: signal × 10 > VCCA × 7  (avoids fractional constants)
//
// Power validity:
//   VCCA_ok : 0.8 V ≤ VCCA ≤ 5.5 V  (Q20: 838861 … 5767168)
//   VCCB_ok : 2.2 V ≤ VCCB ≤ 5.5 V  (Q20: 2306867 … 5767168)
//   voltage_condition (VCCB ≥ 0.4·VCCA + 0.8):
//     Multiplied through by 5 to keep integers: 2·VCCA + 4·2^20 ≤ 5·VCCB
// =============================================================================
module PCA9617A_TOP #(
    parameter integer CLK_FREQ_HZ  = 50_000_000,
    parameter integer POR_DELAY_US = 400,
    parameter integer PHL_A2B_NS   = 173,
    parameter integer PLH_A2B_NS   = 102,
    parameter integer PHL_B2A_NS   = 152,
    parameter integer PLH_B2A_NS   = 103
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] VCCA_analog,       // Q20: A-side supply voltage
    input  wire [31:0] VCCB_analog,       // Q20: B-side supply voltage
    input  wire        EN,                // active-high enable request
    input  wire [31:0] SDA_A,            // Q20: SDA voltage on A-side
    input  wire [31:0] SCL_A,            // Q20: SCL voltage on A-side
    output wire [31:0] SDA_A_driver,     // Q20 pull-low value, or 0xFFFFFFFF (Hi-Z)
    output wire [31:0] SCL_A_driver,
    input  wire [31:0] SDA_B,
    input  wire [31:0] SCL_B,
    output wire [31:0] SDA_B_driver,
    output wire [31:0] SCL_B_driver,
    output wire        sda_a_low_detected,
    output wire        sda_b_low_detected,
    output wire        scl_a_low_detected,
    output wire        scl_b_low_detected,
    output wire        repeater_active,
    output wire        arbitration_lost_sda,
    output wire        arbitration_lost_scl
);

    // -------------------------------------------------------------------------
    // Supply validity — all purely combinational (no registers)
    // -------------------------------------------------------------------------
    localparam [31:0] VCCA_MIN_Q = 32'd838861;   // 0.8 V in Q20
    localparam [31:0] VCCA_MAX_Q = 32'd5767168;  // 5.5 V in Q20
    localparam [31:0] VCCB_MIN_Q = 32'd2306867;  // 2.2 V in Q20
    localparam [31:0] VCCB_MAX_Q = 32'd5767168;  // 5.5 V in Q20
    localparam [31:0] VC_OFFSET  = 32'd4194304;  // 4.0 × 2^20 (= 0.8 V × 5 in Q20)

    wire VCCA_ok = (VCCA_analog >= VCCA_MIN_Q) && (VCCA_analog <= VCCA_MAX_Q);
    wire VCCB_ok = (VCCB_analog >= VCCB_MIN_Q) && (VCCB_analog <= VCCB_MAX_Q);
    // Voltage condition: VCCB ≥ 0.4·VCCA + 0.8, scaled ×5: 2·VCCA + 4·2^20 ≤ 5·VCCB
    wire voltage_condition_ok = ((2 * VCCA_analog + VC_OFFSET) <= (5 * VCCB_analog));

    wire power_good_raw = VCCA_ok & VCCB_ok & voltage_condition_ok;  // all conditions met

    wire power_good;  // debounced output of power_monitor
    wire por_done;    // POR delay complete signal

    // Debounce the combinational power_good_raw over 20000 cycles (400 µs at 50 MHz)
    power_monitor #(
        .DEBOUNCE_CYCLES (20000)
    ) u_power_monitor (
        .clk            (clk),
        .rst_n          (rst_n),
        .power_good_raw (power_good_raw),
        .power_good     (power_good)
    );

    // POR delay: hold por_done low for 400 µs after both supplies become valid
    power_on_reset_delay #(
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .POR_DELAY_US (POR_DELAY_US)
    ) u_por (
        .clk      (clk),
        .rst_n    (rst_n),
        .VCCA_ok  (VCCA_ok),
        .VCCB_ok  (VCCB_ok),
        .por_done (por_done)
    );

    // Bus-idle: signal high (> 70% of VCCA) means no master is pulling the line low
    // Multiplied form avoids fixed-point division: 10·signal > 7·VCCA
    wire sda_a_idle = (SDA_A * 32'd10 > VCCA_analog * 32'd7);
    wire scl_a_idle = (SCL_A * 32'd10 > VCCA_analog * 32'd7);

    // Enable controller: manages repeater_active with turn-on/off delays and bus-idle gating
    enable_controller #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .ENABLE_DELAY_NS  (100),
        .DISABLE_DELAY_NS (100)
    ) u_enable_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .EN             (EN),
        .power_good     (power_good),
        .por_done       (por_done),
        .sda_a_idle     (sda_a_idle),
        .scl_a_idle     (scl_a_idle),
        .repeater_active(repeater_active),
        .enable_ready   ()               // unused at top level
    );

    // -------------------------------------------------------------------------
    // SDA signal chain
    // -------------------------------------------------------------------------
    wire sda_low_A, sda_low_B;               // low-detected outputs from detectors
    wire sda_drive_A_intent, sda_drive_B_intent;  // direction logic requests
    wire sda_drive_A_delayed, sda_drive_B_delayed; // delay-modelled drive enables

    // A-side low detector: ratiometric threshold (VCCA/4) with hysteresis
    port_A_low_detect #(
        .SIGNAL_NAME   ("SDA"),
        .HYSTERESIS_Q  (131072)       // ±0.125 V hysteresis band in Q20
    ) u_sda_low_A (
        .clk           (clk),
        .rst_n         (rst_n),
        .signal_voltage(SDA_A),
        .VCCA_analog   (VCCA_analog),
        .VCCA_ok       (VCCA_ok),
        .low_out       (sda_low_A),
        .low_detected  (sda_a_low_detected)
    );

    // B-side low detector: fixed threshold (≈0.40 V) with hysteresis (±0.20 V)
    port_B_low_detect #(
        .SIGNAL_NAME   ("SDA"),
        .THRESHOLD_Q   (419430),      // 0.40 V centre in Q20
        .HYSTERESIS_Q  (209715)       // ±0.20 V in Q20
    ) u_sda_low_B (
        .clk           (clk),
        .rst_n         (rst_n),
        .signal_voltage(SDA_B),
        .VCCB_ok       (VCCB_ok),
        .low_out       (sda_low_B),
        .low_detected  (sda_b_low_detected)
    );

    // Direction logic: routes low signal to opposite side; detects arbitration conflicts
    direction_propagation_logic #(.SIGNAL_NAME("SDA")) u_sda_dir (
        .clk             (clk),
        .rst_n           (rst_n),
        .low_A           (sda_low_A),
        .low_B           (sda_low_B),
        .repeater_active (repeater_active),
        .drive_A_intent  (sda_drive_A_intent),
        .drive_B_intent  (sda_drive_B_intent),
        .arbitration_lost(arbitration_lost_sda)
    );

    // Delay model: introduces datasheet tPHL/tPLH between intent and actual drive
    propagation_delay_model #(
        .SIGNAL_NAME  ("SDA"),
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .PHL_A2B_NS   (PHL_A2B_NS),
        .PLH_A2B_NS   (PLH_A2B_NS),
        .PHL_B2A_NS   (PHL_B2A_NS),
        .PLH_B2A_NS   (PLH_B2A_NS)
    ) u_sda_delay (
        .clk            (clk),
        .rst_n          (rst_n),
        .drive_A_intent (sda_drive_A_intent),
        .drive_B_intent (sda_drive_B_intent),
        .repeater_active(repeater_active),
        .drive_A_delayed(sda_drive_A_delayed),
        .drive_B_delayed(sda_drive_B_delayed)
    );

    // Output drivers: active → pull-low voltage; inactive → Hi-Z sentinel (0xFFFFFFFF)
    // A-side pulls SDA to ≈0.10 V (Q20: 104858); B-side pulls to ≈0.55 V (Q20: 576717)
    assign SDA_A_driver = (sda_drive_A_delayed && repeater_active && VCCA_ok)
                           ? 32'd104858 : 32'hFFFF_FFFF;
    assign SDA_B_driver = (sda_drive_B_delayed && repeater_active && VCCB_ok)
                           ? 32'd576717 : 32'hFFFF_FFFF;

    // -------------------------------------------------------------------------
    // SCL signal chain — identical structure to SDA; see SDA comments above
    // -------------------------------------------------------------------------
    wire scl_low_A, scl_low_B;
    wire scl_drive_A_intent, scl_drive_B_intent;
    wire scl_drive_A_delayed, scl_drive_B_delayed;

    port_A_low_detect #(
        .SIGNAL_NAME   ("SCL"),
        .HYSTERESIS_Q  (131072)
    ) u_scl_low_A (
        .clk           (clk),
        .rst_n         (rst_n),
        .signal_voltage(SCL_A),
        .VCCA_analog   (VCCA_analog),
        .VCCA_ok       (VCCA_ok),
        .low_out       (scl_low_A),
        .low_detected  (scl_a_low_detected)
    );

    port_B_low_detect #(
        .SIGNAL_NAME   ("SCL"),
        .THRESHOLD_Q   (419430),
        .HYSTERESIS_Q  (209715)
    ) u_scl_low_B (
        .clk           (clk),
        .rst_n         (rst_n),
        .signal_voltage(SCL_B),
        .VCCB_ok       (VCCB_ok),
        .low_out       (scl_low_B),
        .low_detected  (scl_b_low_detected)
    );

    direction_propagation_logic #(.SIGNAL_NAME("SCL")) u_scl_dir (
        .clk             (clk),
        .rst_n           (rst_n),
        .low_A           (scl_low_A),
        .low_B           (scl_low_B),
        .repeater_active (repeater_active),
        .drive_A_intent  (scl_drive_A_intent),
        .drive_B_intent  (scl_drive_B_intent),
        .arbitration_lost(arbitration_lost_scl)
    );

    propagation_delay_model #(
        .SIGNAL_NAME  ("SCL"),
        .CLK_FREQ_HZ  (CLK_FREQ_HZ),
        .PHL_A2B_NS   (PHL_A2B_NS),
        .PLH_A2B_NS   (PLH_A2B_NS),
        .PHL_B2A_NS   (PHL_B2A_NS),
        .PLH_B2A_NS   (PLH_B2A_NS)
    ) u_scl_delay (
        .clk            (clk),
        .rst_n          (rst_n),
        .drive_A_intent (scl_drive_A_intent),
        .drive_B_intent (scl_drive_B_intent),
        .repeater_active(repeater_active),
        .drive_A_delayed(scl_drive_A_delayed),
        .drive_B_delayed(scl_drive_B_delayed)
    );

    // SCL A-side pulls to ≈0.10 V (Q20: 104858); B-side pulls to ≈0.40 V (Q20: 419430)
    assign SCL_A_driver = (scl_drive_A_delayed && repeater_active && VCCA_ok)
                           ? 32'd104858 : 32'hFFFF_FFFF;
    assign SCL_B_driver = (scl_drive_B_delayed && repeater_active && VCCB_ok)
                           ? 32'd419430 : 32'hFFFF_FFFF;

endmodule 

