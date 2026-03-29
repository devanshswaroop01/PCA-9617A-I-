// =============================================================================
// Testbench: PCA9617A_tb
// Signoff verification for PCA9617A_TOP.
// 20 test cases covering: debounce, POR, enable/disable sequencing, bus-busy
// gating, propagation delay (tPHL/tPLH), level-shifting, arbitration detection,
// VIL boundary conditions, sub-minimum supply rejection, and full I2C byte.
//
// All voltages are Q20 fixed-point integers (1 V = 2^20 = 1048576).
// Clock: 50 MHz → 20 ns period.  Tolerance: ±5 clock cycles (±100 ns).
// Driver-active classification: driver output < 1.0 V Q20 (= 1048576).
// Hi-Z sentinel: 0xFFFFFFFF.
// =============================================================================
`timescale 1ns / 1ps

module PCA9617A_tb;

    // -------------------------------------------------------------------------
    // Simulation parameters
    // -------------------------------------------------------------------------
    localparam integer CLK_FREQ_HZ  = 50_000_000;
    localparam integer POR_DELAY_US = 400;
    localparam integer PHL_A2B_NS   = 173;
    localparam integer PLH_A2B_NS   = 102;
    localparam integer PHL_B2A_NS   = 152;
    localparam integer PLH_B2A_NS   = 103;

    localparam integer CLK_PERIOD_NS = 20;           // 50 MHz → 20 ns
    localparam integer TOL_NS        = 5 * CLK_PERIOD_NS;  // ±100 ns measurement tolerance

    // -------------------------------------------------------------------------
    // Q20 voltage constants  (voltage_V × 2^20, rounded to nearest integer)
    // -------------------------------------------------------------------------
    localparam [31:0] V0_0   = 32'd0;
    localparam [31:0] V0_1   = 32'd104858;   // 0.10 V
    localparam [31:0] V0_2   = 32'd209715;   // 0.20 V — equals port_B threshold_low exactly
    localparam [31:0] V0_6   = 32'd629146;   // 0.60 V — just above port_B threshold_high
    localparam [31:0] V0_7   = 32'd734003;   // 0.70 V — equals port_A threshold_low at VCCA=3.3V
    localparam [31:0] V0_95  = 32'd996147;   // 0.95 V — equals port_A threshold_high at VCCA=3.3V
    localparam [31:0] V1_8   = 32'd1887437;  // 1.80 V
    localparam [31:0] V2_0   = 32'd2097152;  // 2.00 V — below VCCB minimum (2.2V)
    localparam [31:0] V2_2   = 32'd2306867;  // 2.20 V — VCCB minimum
    localparam [31:0] V3_3   = 32'd3460301;  // 3.30 V — nominal rail
    localparam [31:0] V5_5   = 32'd5767168;  // 5.50 V — maximum rail

    localparam [31:0] HIZ       = 32'hFFFF_FFFF;  // sentinel value: driver in Hi-Z state
    localparam [31:0] DRIVE_THR = 32'd1048576;    // 1.0 V Q20: threshold for "actively driven"

    // -------------------------------------------------------------------------
    // DUT port declarations
    // -------------------------------------------------------------------------
    reg  clk   = 1'b0;
    reg  rst_n = 1'b0;

    always #(CLK_PERIOD_NS / 2) clk = ~clk;  // 50 MHz free-running clock

    reg  [31:0] VCCA_analog = V0_0;
    reg  [31:0] VCCB_analog = V0_0;
    reg         EN          = 1'b0;

    // Bus stimulus signals; idle default = high (no master pulling low)
    reg  [31:0] SDA_A = V3_3;
    reg  [31:0] SCL_A = V3_3;
    reg  [31:0] SDA_B = V3_3;
    reg  [31:0] SCL_B = V3_3;

    wire [31:0] SDA_A_driver;
    wire [31:0] SCL_A_driver;
    wire [31:0] SDA_B_driver;
    wire [31:0] SCL_B_driver;

    wire sda_a_low_detected;
    wire sda_b_low_detected;
    wire scl_a_low_detected;
    wire scl_b_low_detected;
    wire repeater_active;
    wire arbitration_lost_sda;
    wire arbitration_lost_scl;

    PCA9617A_TOP #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .POR_DELAY_US(POR_DELAY_US),
        .PHL_A2B_NS  (PHL_A2B_NS),
        .PLH_A2B_NS  (PLH_A2B_NS),
        .PHL_B2A_NS  (PHL_B2A_NS),
        .PLH_B2A_NS  (PLH_B2A_NS)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .VCCA_analog         (VCCA_analog),
        .VCCB_analog         (VCCB_analog),
        .EN                  (EN),
        .SDA_A               (SDA_A),
        .SCL_A               (SCL_A),
        .SDA_B               (SDA_B),
        .SCL_B               (SCL_B),
        .SDA_A_driver        (SDA_A_driver),
        .SCL_A_driver        (SCL_A_driver),
        .SDA_B_driver        (SDA_B_driver),
        .SCL_B_driver        (SCL_B_driver),
        .sda_a_low_detected  (sda_a_low_detected),
        .sda_b_low_detected  (sda_b_low_detected),
        .scl_a_low_detected  (scl_a_low_detected),
        .scl_b_low_detected  (scl_b_low_detected),
        .repeater_active     (repeater_active),
        .arbitration_lost_sda(arbitration_lost_sda),
        .arbitration_lost_scl(arbitration_lost_scl)
    );

    // Dump all signals for waveform viewing
    initial begin
        $dumpfile("PCA9617A_tb.vcd");
        $dumpvars(0, PCA9617A_tb);
    end

    // -------------------------------------------------------------------------
    // Pass / fail counters and assertion task
    // -------------------------------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    // chk: print PASS/FAIL with label and current simulation time on failure
    task automatic chk;
      input [511:0] label;  // 40-char ASCII label (padded)
        input         cond;
        begin
            if (cond) begin
                $display("  [PASS] %s", label);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [FAIL] *** %s ***  at %0t ns", label, $time / 1000);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // clk_n: advance simulation by exactly n rising clock edges
    task automatic clk_n;
        input integer n;
        integer i;
        for (i = 0; i < n; i = i + 1)
            @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Polling tasks — each samples a DUT signal fresh on every rising edge to
    // avoid the "frozen argument" bug (task inputs are evaluated once at call site).
    // -------------------------------------------------------------------------

    // Wait up to timeout_ns for por_done to assert
    task automatic wait_por_done;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (dut.por_done === 1'b1) begin
                    success = 1;
                    disable wait_por_done;
                end
            end
        end
    endtask

    // Wait up to timeout_ns for repeater_active to assert
    task automatic wait_rep_active;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (repeater_active === 1'b1) begin
                    success = 1;
                    disable wait_rep_active;
                end
            end
        end
    endtask

    // Wait up to timeout_ns for repeater_active to deassert
    task automatic wait_rep_inactive;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (repeater_active === 1'b0) begin
                    success = 1;
                    disable wait_rep_inactive;
                end
            end
        end
    endtask

    // Wait for SDA_B_driver to show active-low (< DRIVE_THR)
    task automatic wait_driving_B_SDA;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SDA_B_driver < DRIVE_THR) begin
                    success = 1;
                    disable wait_driving_B_SDA;
                end
            end
        end
    endtask

    // Wait for SDA_B_driver to return to Hi-Z sentinel
    task automatic wait_hiZ_B_SDA;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SDA_B_driver === HIZ) begin
                    success = 1;
                    disable wait_hiZ_B_SDA;
                end
            end
        end
    endtask

    // Wait for SDA_A_driver to show active-low
    task automatic wait_driving_A_SDA;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SDA_A_driver < DRIVE_THR) begin
                    success = 1;
                    disable wait_driving_A_SDA;
                end
            end
        end
    endtask

    // Wait for SDA_A_driver to return to Hi-Z
    task automatic wait_hiZ_A_SDA;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SDA_A_driver === HIZ) begin
                    success = 1;
                    disable wait_hiZ_A_SDA;
                end
            end
        end
    endtask

    // Wait for SCL_B_driver to show active-low
    task automatic wait_driving_B_SCL;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SCL_B_driver < DRIVE_THR) begin
                    success = 1;
                    disable wait_driving_B_SCL;
                end
            end
        end
    endtask

    // Wait for SCL_A_driver to show active-low
    task automatic wait_driving_A_SCL;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (SCL_A_driver < DRIVE_THR) begin
                    success = 1;
                    disable wait_driving_A_SCL;
                end
            end
        end
    endtask

    // Wait for arbitration_lost_sda to assert
    task automatic wait_arb_sda;
        input  integer timeout_ns;
        output integer success;
        integer start_time;
        begin
            success    = 0;
            start_time = $time;
            while (($time - start_time) < timeout_ns) begin
                @(posedge clk);
                if (arbitration_lost_sda === 1'b1) begin
                    success = 1;
                    disable wait_arb_sda;
                end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper functions: classify a [31:0] driver value sampled at the call site.
    // Used only in chk() expressions — never passed as task arguments (which
    // would freeze the value at call time instead of re-evaluating each cycle).
    // -------------------------------------------------------------------------
    function automatic integer is_driving;
        input [31:0] driver_val;
        begin
            is_driving = (driver_val < DRIVE_THR) ? 1 : 0;
        end
    endfunction

    function automatic integer is_hiZ;
        input [31:0] driver_val;
        begin
            is_hiZ = (driver_val === HIZ) ? 1 : 0;
        end
    endfunction

    // -------------------------------------------------------------------------
    // Composite helper tasks: common setup sequences reused across test cases
    // -------------------------------------------------------------------------

    // do_reset: assert reset, clear all stimulus, then release reset
    task automatic do_reset;
        begin
            rst_n       = 1'b0;
            EN          = 1'b0;
            VCCA_analog = V0_0;
            VCCB_analog = V0_0;
            SDA_A       = V3_3;
            SCL_A       = V3_3;
            SDA_B       = V3_3;
            SCL_B       = V3_3;
            clk_n(4);
            rst_n = 1'b1;  // release reset
            clk_n(2);
        end
    endtask

    // power_up: apply supply voltages and wait for POR to complete (≤ 440 µs)
    task automatic power_up;
        input  [31:0] vcca_q;
        input  [31:0] vccb_q;
        output integer ok;
        begin
            ok          = 0;
            VCCA_analog = vcca_q;
            VCCB_analog = vccb_q;
            wait_por_done(440000, ok);  // 440 µs timeout (400 µs spec + margin)
        end
    endtask

    // enable_rep: assert EN and wait for repeater_active (≤ 10 µs)
    task automatic enable_rep;
        output integer ok;
        begin
            ok = 0;
            EN = 1'b1;
            wait_rep_active(10000, ok);
        end
    endtask

    // =========================================================================
    // MAIN TEST SEQUENCE
    // =========================================================================
    integer     t0_int, t1_int, meas_ns;  // timing measurement registers
    integer     ok;
    integer     wait_ok;
    integer     i;
    reg   [7:0] byte_val;

    initial begin
        $display("");
        $display("============================================================");
        $display("  PCA9617A_TOP  Signoff Verification  (fixed TB)");
        $display("============================================================");

        // ----------------------------------------------------------------
        // TC01 – Power-good debounce filter
        // Verifies the 20000-cycle debounce counter in power_monitor.
        // A short glitch (50 cycles) during stable operation must not
        // change the debounced output.
        // ----------------------------------------------------------------
        $display("\n[TC01] Power-good debounce filter");
        do_reset;
        VCCA_analog = V3_3;
        VCCB_analog = V3_3;
        clk_n(100);   // 2 µs: well before 20000-cycle (400 µs) debounce threshold
        chk("TC01a: power_good=0 before debounce completes",
            dut.u_power_monitor.power_good === 1'b0);
        clk_n(20100); // now past the 20000-cycle debounce window
        chk("TC01b: power_good=1 after debounce completes",
            dut.u_power_monitor.power_good === 1'b1);
        VCCA_analog = V0_0;   // inject a 50-cycle glitch (< debounce threshold)
        clk_n(50);
        VCCA_analog = V3_3;
        clk_n(200);
        chk("TC01c: power_good stable through short glitch",
            dut.u_power_monitor.power_good === 1'b1);

        // ----------------------------------------------------------------
        // TC02 – POR 400 µs delay
        // Verifies that por_done remains low during the delay and asserts
        // after 400 µs with both supplies valid.
        // ----------------------------------------------------------------
        $display("\n[TC02] POR 400 us delay");
        do_reset;
        VCCA_analog = V3_3;
        VCCB_analog = V3_3;
        clk_n(100);  // sample well before POR timer expires
        chk("TC02a: por_done=0 before 400 us",
            dut.por_done === 1'b0);
        wait_por_done(450000, wait_ok);  // wait up to 450 µs
        chk("TC02b: por_done=1 after 400 us", wait_ok === 1);

        // ----------------------------------------------------------------
        // TC03 – POR counter resets on supply dropout
        // Drops VCCB mid-count to verify the POR counter restarts from zero.
        // ----------------------------------------------------------------
        $display("\n[TC03] POR counter reset on supply dropout");
        do_reset;
        VCCA_analog = V3_3;
        VCCB_analog = V3_3;
        clk_n(10000);  // 200 µs: halfway through POR delay
        chk("TC03a: por_done=0 at 200 us", dut.por_done === 1'b0);
        VCCB_analog = V0_0;  // glitch VCCB to reset the POR counter
        clk_n(5);
        VCCB_analog = V3_3;
        clk_n(5000);   // only 100 µs since restart — should not have completed
        chk("TC03b: por_done=0 after interrupted POR",
            dut.por_done === 1'b0);
        clk_n(16000);  // additional ~320 µs: now past 400 µs from restart
        chk("TC03c: por_done=1 after clean 400 us",
            dut.por_done === 1'b1);

        // ----------------------------------------------------------------
        // TC04 – EN=0 keeps repeater disabled
        // Ensures repeater_active stays low when EN is never asserted.
        // ----------------------------------------------------------------
        $display("\n[TC04] EN=0 keeps repeater disabled");
        do_reset;
        power_up(V3_3, V3_3, ok);
        EN = 1'b0;
        clk_n(200);
        chk("TC04: repeater_active=0 with EN=0",
            repeater_active === 1'b0);

        // ----------------------------------------------------------------
        // TC05 – Normal enable on idle bus after POR
        // Full enable and disable cycle on an idle bus.
        // ----------------------------------------------------------------
        $display("\n[TC05] Normal enable sequence");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        chk("TC05a: repeater_active asserts after EN", ok === 1);
        EN = 1'b0;
        wait_rep_inactive(10000, wait_ok);
        chk("TC05b: repeater_active de-asserts on EN=0", wait_ok === 1);

        // ----------------------------------------------------------------
        // TC06 – Enable blocked while bus is busy
        // SDA_A pulled low before EN asserts; enable must wait for release.
        // ----------------------------------------------------------------
        $display("\n[TC06] Enable blocked on busy bus");
        do_reset;
        power_up(V3_3, V3_3, ok);
        SDA_A = V0_1;   // pull A-side SDA low to simulate active bus
        clk_n(10);
        EN = 1'b1;
        clk_n(300);     // 6 µs — longer than enable delay; must still be blocked
        chk("TC06a: repeater_active=0 while bus busy",
            repeater_active === 1'b0);
        SDA_A = V3_3;   // release bus → enable sequence can now proceed
        wait_rep_active(10000, wait_ok);
        chk("TC06b: repeater_active=1 after bus release", wait_ok === 1);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC07 – Disable on EN de-assert
        // Repeater must de-assert within the configured disable delay.
        // ----------------------------------------------------------------
        $display("\n[TC07] Disable on EN de-assert");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(10);
        EN = 1'b0;
        wait_rep_inactive(10000, wait_ok);
        chk("TC07: repeater de-asserts on EN=0", wait_ok === 1);

        // ----------------------------------------------------------------
        // TC08 – Power loss forces repeater off
        // Removing VCCA while active must instantly override the enable
        // controller and pull all drivers to Hi-Z (bypasses disable delay).
        // ----------------------------------------------------------------
        $display("\n[TC08] Power loss forces repeater off");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        chk("TC08a: repeater_active before power loss", ok === 1);
        VCCA_analog = V0_0;  // drop VCCA — power_good falls after debounce
        clk_n(20500);        // wait for debounce counter to expire (20000 cycles)
        chk("TC08b: repeater_active=0 after VCCA loss",
            repeater_active === 1'b0);
        chk("TC08c: SDA_A_driver hi-Z after power loss",
            is_hiZ(SDA_A_driver));
        chk("TC08d: SDA_B_driver hi-Z after power loss",
            is_hiZ(SDA_B_driver));
        VCCA_analog = V3_3;
        clk_n(100);

        // ----------------------------------------------------------------
        // TC09 – SDA A→B tPHL propagation  (spec 173 ns)
        // Pipeline analysis at 50 MHz (CLK_PS = 20000 ps):
        //   raw_cycles = ceil(173000 / 20000) = 9
        //   net_cycles = 9 − 7 (overhead) = 2
        //   total observable delay = 9 cycles × 20 ns = 180 ns
        //   Window: [173−100, 173+100] = [73, 273] ns → 180 ns passes.
        // ----------------------------------------------------------------
        $display("\n[TC09] SDA A->B tPHL (spec 173 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        @(posedge clk);
        t0_int = $time;
        SDA_A = V0_1;                    // pull A-side low: triggers A→B propagation
        wait_driving_B_SDA(PHL_A2B_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC09a: SDA_B_driver responds to SDA_A falling", wait_ok === 1);
        chk("TC09b: tPHL(A->B) within tolerance",
            (meas_ns >= (PHL_A2B_NS - TOL_NS)) && (meas_ns <= (PHL_A2B_NS + TOL_NS)));
        $display("       Measured tPHL(A->B) = %0d ns  (expect ~180 ns, window [%0d,%0d])",
                 meas_ns, PHL_A2B_NS - TOL_NS, PHL_A2B_NS + TOL_NS);
        SDA_A = V3_3;
        clk_n(30);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC10 – SDA A→B tPLH propagation  (spec 102 ns)
        // Pipeline analysis:
        //   raw_cycles = ceil(102000 / 20000) = 6
        //   net_cycles = 6 − 7 → clamped to 1 (minimum)
        //   total = (1 + 7) cycles × 20 ns = 160 ns
        //   Window: [102−100, 102+100] = [2, 202] ns → 160 ns passes.
        // ----------------------------------------------------------------
        $display("\n[TC10] SDA A->B tPLH (spec 102 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        SDA_A = V0_1;
        wait_driving_B_SDA(PHL_A2B_NS * 3, wait_ok);  // pre-condition: B-side driven low
        clk_n(5);
        @(posedge clk);
        t0_int = $time;
        SDA_A = V3_3;                    // release A-side: triggers A→B release propagation
        wait_hiZ_B_SDA(PLH_A2B_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC10a: SDA_B_driver releases after SDA_A rising", wait_ok === 1);
        chk("TC10b: tPLH(A->B) within tolerance",
            (meas_ns >= (PLH_A2B_NS - TOL_NS)) && (meas_ns <= (PLH_A2B_NS + TOL_NS)));
        $display("       Measured tPLH(A->B) = %0d ns  (expect ~160 ns, window [%0d,%0d])",
                 meas_ns, PLH_A2B_NS - TOL_NS, PLH_A2B_NS + TOL_NS);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC11 – SDA B→A tPHL propagation  (spec 152 ns)
        // Pipeline analysis:
        //   raw_cycles = ceil(152000 / 20000) = 8
        //   net_cycles = 8 − 7 = 1
        //   total = 8 cycles × 20 ns = 160 ns
        //   Window: [152−100, 152+100] = [52, 252] ns → 160 ns passes.
        //
        // Stimulus = V0_1 (not V0_2):
        //   port_B_low_detect threshold_low = 419430 − 209715 = 209715 = V0_2.
        //   Comparator uses strict less-than: V0_2 < V0_2 → false (not detected).
        //   V0_1 = 104858 < 209715 → true (detected correctly).
        // ----------------------------------------------------------------
        $display("\n[TC11] SDA B->A tPHL (spec 152 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        @(posedge clk);
        t0_int = $time;
        SDA_B = V0_1;   // V0_1 is strictly below port_B threshold_low (V0_2)
        wait_driving_A_SDA(PHL_B2A_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC11a: SDA_A_driver responds to SDA_B falling", wait_ok === 1);
        chk("TC11b: tPHL(B->A) within tolerance",
            (meas_ns >= (PHL_B2A_NS - TOL_NS)) && (meas_ns <= (PHL_B2A_NS + TOL_NS)));
        $display("       Measured tPHL(B->A) = %0d ns  (expect ~160 ns, window [%0d,%0d])",
                 meas_ns, PHL_B2A_NS - TOL_NS, PHL_B2A_NS + TOL_NS);
        SDA_B = V3_3;
        clk_n(30);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC12 – SDA B→A tPLH propagation  (spec 103 ns)
        // Same threshold rationale as TC11 (V0_1 used, not V0_2).
        // Pipeline: raw=6, net=1, total=8 cycles=160 ns; window [3,203] ns.
        // ----------------------------------------------------------------
        $display("\n[TC12] SDA B->A tPLH (spec 103 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        SDA_B = V0_1;
        wait_driving_A_SDA(PHL_B2A_NS * 3, wait_ok);  // pre-condition: A-side driven low
        clk_n(5);
        @(posedge clk);
        t0_int = $time;
        SDA_B = V3_3;                    // release B-side
        wait_hiZ_A_SDA(PLH_B2A_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC12a: SDA_A_driver releases after SDA_B rising", wait_ok === 1);
        chk("TC12b: tPLH(B->A) within tolerance",
            (meas_ns >= (PLH_B2A_NS - TOL_NS)) && (meas_ns <= (PLH_B2A_NS + TOL_NS)));
        $display("       Measured tPLH(B->A) = %0d ns  (expect ~160 ns, window [%0d,%0d])",
                 meas_ns, PLH_B2A_NS - TOL_NS, PLH_B2A_NS + TOL_NS);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC13 – SCL A→B tPHL  (spec 173 ns, same parameters as TC09)
        // Repeats TC09 on the SCL path to verify the SCL datapath instance.
        // ----------------------------------------------------------------
        $display("\n[TC13] SCL A->B tPHL (spec 173 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        @(posedge clk);
        t0_int = $time;
        SCL_A = V0_1;
        wait_driving_B_SCL(PHL_A2B_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC13a: SCL_B_driver responds to SCL_A falling", wait_ok === 1);
        chk("TC13b: tPHL_SCL(A->B) within tolerance",
            (meas_ns >= (PHL_A2B_NS - TOL_NS)) && (meas_ns <= (PHL_A2B_NS + TOL_NS)));
        $display("       Measured tPHL_SCL(A->B) = %0d ns  (expect ~180 ns, window [%0d,%0d])",
                 meas_ns, PHL_A2B_NS - TOL_NS, PHL_A2B_NS + TOL_NS);
        SCL_A = V3_3;
        clk_n(30);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC14 – SCL B→A tPHL  (spec 152 ns)
        // Stimulus V0_1 required: V0_2 equals port_B threshold_low exactly,
        // which fails the strict-less-than comparator condition (no detect).
        // ----------------------------------------------------------------
        $display("\n[TC14] SCL B->A tPHL (spec 152 ns)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        @(posedge clk);
        t0_int = $time;
        SCL_B = V0_1;   // strictly below threshold_low (V0_2) → detected
        wait_driving_A_SCL(PHL_B2A_NS * 3, wait_ok);
        t1_int = $time;
        meas_ns = t1_int - t0_int;
        chk("TC14a: SCL_A_driver responds to SCL_B falling", wait_ok === 1);
        chk("TC14b: tPHL_SCL(B->A) within tolerance",
            (meas_ns >= (PHL_B2A_NS - TOL_NS)) && (meas_ns <= (PHL_B2A_NS + TOL_NS)));
        $display("       Measured tPHL_SCL(B->A) = %0d ns  (expect ~160 ns, window [%0d,%0d])",
                 meas_ns, PHL_B2A_NS - TOL_NS, PHL_B2A_NS + TOL_NS);
        SCL_B = V3_3;
        clk_n(30);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC15 – Arbitration: simultaneous SDA fall on both sides
        // Both sides driven low in the same stimulus cycle; after 3 sync
        // cycles both low_A and low_B assert together, causing direction_
        // propagation_logic to see a_falling & b_falling simultaneously.
        // arbitration_lost must latch high and clear only after full release.
        // Arbitration pipeline latency ≈ 4 cycles (80 ns); timeout = 400 ns.
        // After release: 200 ns >> 80 ns deassert latency → expect clear.
        // ----------------------------------------------------------------
        $display("\n[TC15] Arbitration - simultaneous SDA fall");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        SDA_A = V0_1;  // both sides driven low simultaneously
        SDA_B = V0_1;
        wait_arb_sda(400, wait_ok);
        chk("TC15a: arbitration_lost_sda asserts", wait_ok === 1);
        SDA_A = V3_3;
        SDA_B = V3_3;
        clk_n(10);   // 200 ns: well past the ~4-cycle deassert latency
        chk("TC15b: arbitration_lost_sda de-asserts after release",
            arbitration_lost_sda === 1'b0);
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC16 – Level-shift: VCCA=1.8 V / VCCB=3.3 V
        // Verifies the ratiometric A-side threshold scales with VCCA:
        //   threshold     = q(1.8) >> 2 = 1887437 >> 2 = 471859
        //   threshold_low = 471859 − 131072 = 340787 ≈ 0.325 V
        //   V0_2 = 209715 < 340787 → detected (low_A asserts). PASS.
        // ----------------------------------------------------------------
        $display("\n[TC16] Level-shift 1.8V / 3.3V");
        do_reset;
        SDA_A = V1_8;  // initialise A-side to 1.8 V (new VCCA rail level)
        SCL_A = V1_8;
        SDA_B = V3_3;
        SCL_B = V3_3;
        power_up(V1_8, V3_3, ok);
        enable_rep(ok);
        chk("TC16a: repeater_active with 1.8V/3.3V supplies", ok === 1);
        SDA_A = V0_2;  // pull to 0.20 V — below threshold_low for VCCA=1.8 V
        clk_n(10);
        chk("TC16b: sda_a_low_detected at VIL(1.8V)",
            sda_a_low_detected === 1'b1);
        wait_driving_B_SDA(PHL_A2B_NS * 3, wait_ok);
        chk("TC16c: SDA_B_driver goes low (level-shifted)", wait_ok === 1);
        SDA_A = V1_8;
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC17 – VIL(A) threshold boundary  (VCCA=3.3 V)
        // Derived thresholds at VCCA = Q20(3.3V) = 3460301:
        //   threshold     = 3460301 >> 2 = 865075
        //   threshold_low = 865075 − 131072 = 734003 (= V0_7 exactly)
        //   threshold_high= 865075 + 131072 = 996147 (= V0_95 exactly)
        // TC17a: V0_7 == threshold_low → strict-less-than is false → no detect.
        // TC17b: V0_95 == threshold_high → strict-greater-than is false → no clear.
        //        Comparator retains its state from TC17a (still 0) → low_detected=0.
        // ----------------------------------------------------------------
        $display("\n[TC17] VIL(A) threshold boundary");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        SDA_A = V0_7;   // exactly at threshold_low: strict < fails → no low detect
        clk_n(10);
        chk("TC17a: sda_a_low_detected NOT asserted at threshold_low (0.70V)",
            sda_a_low_detected !== 1'b1);
        SDA_A = V0_95;  // exactly at threshold_high: strict > fails → no state change
        clk_n(10);
        chk("TC17b: sda_a_low_detected remains 0 at threshold_high (0.95V)",
            sda_a_low_detected === 1'b0);
        SDA_A = V3_3;
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC18 – VIL(B) fixed threshold
        // Fixed (non-ratiometric) thresholds:
        //   threshold_low  = 419430 − 209715 = 209715 (= V0_2 exactly)
        //   threshold_high = 419430 + 209715 = 629145
        // TC18a: V0_2 == threshold_low → strict < fails → not detected.
        // TC18b: V0_6 = 629146 > threshold_high=629145 → comparator clears → 0.
        // ----------------------------------------------------------------
        $display("\n[TC18] VIL(B) fixed threshold");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);
        SDA_B = V0_2;   // exactly at threshold_low: strict < fails → not detected
        clk_n(10);
        chk("TC18a: sda_b_low_detected NOT asserted at threshold_low (0.20V)",
            sda_b_low_detected !== 1'b1);
        SDA_B = V0_6;   // 629146 > 629145 = threshold_high → comparator cleared
        clk_n(10);
        chk("TC18b: sda_b_low_detected cleared at 0.60V (above threshold_high)",
            sda_b_low_detected === 1'b0);
        SDA_B = V3_3;
        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // TC19 – Supply below minimum keeps repeater off
        // TC19a: VCCA=0.7V < 0.8V min → VCCA_ok=0 → power_good=0.
        // TC19b: VCCB=2.0V < 2.2V min → VCCB_ok=0 → power_good=0.
        // TC19c: VCCA=5.5V, VCCB=2.2V → voltage_condition check:
        //   2×q(5.5) + 4×2^20 = 15728640 > 5×q(2.2) = 11534335 → fails.
        // ----------------------------------------------------------------
        $display("\n[TC19] Sub-minimum supply keeps repeater off");
        do_reset;
        VCCA_analog = V0_7;   // 0.70 V < 0.80 V minimum → VCCA_ok=0
        VCCB_analog = V3_3;
        clk_n(25000);
        EN = 1'b1;
        clk_n(200);
        chk("TC19a: repeater off with VCCA=0.7V", repeater_active === 1'b0);
        VCCA_analog = V3_3;
        VCCB_analog = V2_0;   // 2.00 V < 2.20 V minimum → VCCB_ok=0
        clk_n(25000);
        chk("TC19b: repeater off with VCCB=2.0V", repeater_active === 1'b0);
        VCCA_analog = V5_5;
        VCCB_analog = V2_2;   // VCCB at minimum but voltage_condition fails: 2×5.5+0.8 > 5×2.2
        clk_n(25000);
        chk("TC19c: repeater off with voltage condition violated",
            repeater_active === 1'b0);
        EN = 1'b0;
        clk_n(100);

        // ----------------------------------------------------------------
        // TC20 – Full I2C byte transaction (byte 0xA5)
        // Simulates START, 8-bit data with clock, and STOP on the A-side.
        // Verifies A→B propagation at both START and STOP, and that the
        // repeater remains active throughout the transaction.
        //
        // Note: wait_hiZ_B_SDA() is used for the STOP check (not a function
        // passed as a task argument) because task input arguments in Verilog-2001
        // are evaluated once at the call site — passing is_hiZ(SDA_B_driver)
        // would freeze the value at call time and never re-evaluate the live wire.
        // ----------------------------------------------------------------
        $display("\n[TC20] Full I2C byte transaction (0xA5)");
        do_reset;
        power_up(V3_3, V3_3, ok);
        enable_rep(ok);
        clk_n(4);

        begin : tc20_block
            byte_val = 8'hA5;  // test data byte: 1010_0101

            $display("       START condition");
            SDA_A = V0_1;   // START: pull SDA low (SCL still high)
            wait_driving_B_SDA(PHL_A2B_NS * 3, wait_ok);
            chk("TC20a: START propagated to B-side", wait_ok === 1);

            SCL_A = V0_1;   // pull SCL low to begin clocking data
            clk_n(5);

            // Shift out 8 bits MSB-first; toggle SCL around each data bit
            for (i = 7; i >= 0; i = i - 1) begin
                if (byte_val[i])
                    SDA_A = V3_3;  // data bit = 1
                else
                    SDA_A = V0_1;  // data bit = 0
                clk_n(3);
                SCL_A = V3_3;   // SCL rising edge (data sampled by receiver)
                clk_n(5);
                SCL_A = V0_1;   // SCL falling edge
                clk_n(3);
            end

            // STOP: release SCL then SDA (SCL high → SDA rising = I2C STOP)
            SDA_A = V3_3;
            SCL_A = V3_3;
            clk_n(5);

            $display("       STOP condition");
            // Re-evaluates SDA_B_driver live on every clock edge (not a frozen value)
            wait_hiZ_B_SDA(PLH_A2B_NS * 3, wait_ok);
            chk("TC20b: STOP propagated - SDA_B_driver Hi-Z", wait_ok === 1);
            chk("TC20c: repeater still active after transaction",
                repeater_active === 1'b1);
        end

        EN = 1'b0;
        clk_n(200);

        // ----------------------------------------------------------------
        // Final summary
        // ----------------------------------------------------------------
        $display("");
        $display("============================================================");
        $display("  FINAL RESULTS:  PASS = %0d   FAIL = %0d   TOTAL = %0d",
                  pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        if (fail_cnt == 0)
            $display("  STATUS: ALL TESTS PASSED");
        else
            $display("  STATUS: *** %0d TEST(S) FAILED ***", fail_cnt);
        $display("============================================================");

        $finish;
    end

    // Watchdog: kill simulation if it exceeds 50 ms (prevents infinite loops)
    initial begin
        #50_000_000;
        $display("[WATCHDOG] Simulation exceeded 50 ms – forcing $finish");
        $finish;
    end

endmodule

