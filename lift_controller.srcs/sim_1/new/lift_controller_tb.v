// lift_controller_tb.v
// Saket Sankhla | DRDO Jodhpur Internship | June 2026
// Testbench for lift_controller.v
// Simulating 7 test cases to verify all major scenarios

`timescale 1ns / 1ps

module lift_controller_tb;

    // simulation parameters
    parameter NUM_FLOORS   = 4;
    parameter SIM_CLK      = 100;    // using 100 Hz instead of 50 MHz to speed up simulation
    parameter DWELL_S      = 3;      // door stays open for 3 seconds
    parameter CLK_PERIOD   = 20;     // 20ns clock period

    // inputs to DUT
    reg                  clk;
    reg                  rst;
    reg [NUM_FLOORS-1:0] hall_req_up;
    reg [NUM_FLOORS-1:0] hall_req_dn;
    reg [NUM_FLOORS-1:0] cabin_req;
    reg [NUM_FLOORS-1:0] floor_sensor;
    reg                  door_obstruction;
    reg                  cabin_overload;

    // outputs from DUT
    wire motor_up;
    wire motor_dn;
    wire brake_release;
    wire door_open;
    wire door_close;

    // instantiate the design under test
    // overriding CLK_FREQ_HZ so dwell = 300 cycles instead of 150 million
    lift_controller #(
        .NUM_FLOORS  (NUM_FLOORS),
        .CLK_FREQ_HZ (SIM_CLK),
        .DOOR_DWELL_S(DWELL_S)
    ) DUT (
        .clk             (clk),
        .rst             (rst),
        .hall_req_up     (hall_req_up),
        .hall_req_dn     (hall_req_dn),
        .cabin_req       (cabin_req),
        .floor_sensor    (floor_sensor),
        .door_obstruction(door_obstruction),
        .cabin_overload  (cabin_overload),
        .motor_up        (motor_up),
        .motor_dn        (motor_dn),
        .brake_release   (brake_release),
        .door_open       (door_open),
        .door_close      (door_close)
    );

    // clock
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // waveform dump for GTKWave
    initial begin
        $dumpfile("lift_tb.vcd");
        $dumpvars(0, lift_controller_tb);
    end

    // ---------------------------------------------------------------
    // task: apply reset and clear all inputs
    // ---------------------------------------------------------------
    task do_reset;
        begin
            rst              = 1;
            hall_req_up      = 0;
            hall_req_dn      = 0;
            cabin_req        = 0;
            floor_sensor     = 4'b0001;   // car starts at floor 0
            door_obstruction = 0;
            cabin_overload   = 0;
            repeat(4) @(posedge clk);
            rst = 0;
            @(posedge clk);
        end
    endtask

    // ---------------------------------------------------------------
    // task: simulate car arriving at a floor
    // ---------------------------------------------------------------
    task arrive;
        input integer floor_num;
        begin
            floor_sensor = (1 << floor_num);
            @(posedge clk);
        end
    endtask

    // ---------------------------------------------------------------
    // task: wait until motor starts
    // ---------------------------------------------------------------
    task wait_motor;
        input dir;   // 1 = expect UP, 0 = expect DOWN
        integer t;
        begin
            // wait 3 cycles for latch + scheduler + FSM to settle
            repeat(3) @(posedge clk);
            t = 0;
            while (!(motor_up | motor_dn) && t < 50) begin
                @(posedge clk);
                t = t + 1;
            end
            if (t >= 50)
                $display("  [TIMEOUT] motor did not start");
            else if (dir && motor_up)
                $display("  [PASS] motor_up=1, brake_release=%b", brake_release);
            else if (!dir && motor_dn)
                $display("  [PASS] motor_dn=1, brake_release=%b", brake_release);
            else
                $display("  [FAIL] wrong direction : motor_up=%b motor_dn=%b", motor_up, motor_dn);
        end
    endtask

    // ---------------------------------------------------------------
    // task: wait until door opens
    // ---------------------------------------------------------------
    task wait_door_open;
        integer t;
        begin
            t = 0;
            while (!door_open && t < 50) begin
                @(posedge clk);
                t = t + 1;
            end
            if (t >= 50)
                $display("  [TIMEOUT] door did not open");
            else
                $display("  [PASS] door_open=1");
        end
    endtask

    // ---------------------------------------------------------------
    // task: wait until system is back to idle
    // ---------------------------------------------------------------
    task wait_idle;
        integer t;
        begin
            t = 0;
            while ((motor_up | motor_dn | door_open) && t < 1000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (t >= 1000)
                $display("  [TIMEOUT] system did not go to IDLE");
            else
                $display("  [PASS] back to IDLE at t=%0t", $time);
        end
    endtask

    // ---------------------------------------------------------------
    // main stimulus block
    // ---------------------------------------------------------------
    initial begin
        $display("\n=== LIFT CONTROLLER TESTBENCH ===");
        $display("Sim clock: %0d Hz, Door dwell: %0d sec = %0d cycles\n",
                  SIM_CLK, DWELL_S, DWELL_S * SIM_CLK);

        // ---- TC1: Reset check ----
        $display("\n--- TC1: Reset ---");
        do_reset;
        if (!motor_up && !motor_dn && !brake_release && !door_open && door_close)
            $display("  [PASS] all outputs safe after reset");
        else
            $display("  [FAIL] unexpected output after reset");
        repeat(5) @(posedge clk);

        // ---- TC2: Single UP call, floor 0 to floor 3 ----
        $display("\n--- TC2: UP call floor 0 -> floor 3 ---");
        do_reset;
        floor_sensor = 4'b0001;
        cabin_req    = 4'b1000;
        repeat(2) @(posedge clk);   // hold 2 cycles so latch captures it
        cabin_req = 0;
        wait_motor(1);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(3);
        wait_door_open;
        wait_idle;

        // ---- TC3: Single DOWN call, floor 3 to floor 0 ----
        $display("\n--- TC3: DOWN call floor 3 -> floor 0 ---");
        do_reset;
        floor_sensor = 4'b1000;
        hall_req_dn  = 4'b0001;
        repeat(2) @(posedge clk);
        hall_req_dn = 0;
        wait_motor(0);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(0);
        wait_door_open;
        wait_idle;

        // ---- TC4: LOOK algorithm direction reversal ----
        $display("\n--- TC4: LOOK reversal - cabin[3] + hall_dn[0], car at floor 1 ---");
        do_reset;
        floor_sensor = 4'b0010;
        cabin_req    = 4'b1000;
        hall_req_dn  = 4'b0001;
        repeat(2) @(posedge clk);
        cabin_req   = 0;
        hall_req_dn = 0;
        $display("  expecting MOVE_UP first...");
        wait_motor(1);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(3);
        wait_door_open;
        repeat(DWELL_S * SIM_CLK + 10) @(posedge clk);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(0);
        if (motor_dn)
            $display("  [PASS] direction reversed to DOWN");
        else
            $display("  [FAIL] motor_up=%b motor_dn=%b", motor_up, motor_dn);
        wait_door_open;
        wait_idle;

        // ---- TC5: Door obstruction extends dwell timer ----
        $display("\n--- TC5: Door obstruction ---");
        do_reset;
        floor_sensor = 4'b0001;
        cabin_req    = 4'b0100;
        repeat(2) @(posedge clk);
        cabin_req = 0;
        wait_motor(1);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(2);
        wait_door_open;
        repeat(DWELL_S * SIM_CLK / 2) @(posedge clk);
        $display("  applying door_obstruction at t=%0t", $time);
        door_obstruction = 1;
        repeat(50) @(posedge clk);
        door_obstruction = 0;
        $display("  released at t=%0t", $time);
        if (door_open)
            $display("  [PASS] door still open - timer extended");
        else
            $display("  [FAIL] door closed too early");
        wait_idle;

        // ---- TC6: Cabin overload blocks departure ----
        $display("\n--- TC6: Cabin overload ---");
        do_reset;
        floor_sensor   = 4'b0001;
        cabin_overload = 1;
        cabin_req      = 4'b1000;
        repeat(2) @(posedge clk);
        cabin_req = 0;
        repeat(10) @(posedge clk);
        if (!motor_up && !motor_dn)
            $display("  [PASS] motor blocked due to overload");
        else
            $display("  [FAIL] motor started despite overload");
        cabin_overload = 0;
        $display("  overload cleared, expecting departure...");
        wait_motor(1);
        floor_sensor = 0;
        repeat(3) @(posedge clk);
        arrive(3);
        wait_door_open;
        wait_idle;

        // ---- TC7: Request at current floor, door opens immediately ----
        $display("\n--- TC7: Same floor request ---");
        do_reset;
        floor_sensor = 4'b0100;
        cabin_req    = 4'b0100;
        repeat(2) @(posedge clk);
        cabin_req = 0;
        repeat(3) @(posedge clk);
        if (door_open && !motor_up && !motor_dn)
            $display("  [PASS] door opened without motor movement");
        else
            $display("  [FAIL] motor_up=%b motor_dn=%b door_open=%b", motor_up, motor_dn, door_open);
        wait_idle;

        $display("\n=== ALL TEST CASES DONE ===\n");
        $finish;
    end

    // watchdog - kills sim if it hangs
    initial begin
        #(CLK_PERIOD * 200_000);
        $display("[WATCHDOG] timeout - force quit");
        $finish;
    end

endmodule
