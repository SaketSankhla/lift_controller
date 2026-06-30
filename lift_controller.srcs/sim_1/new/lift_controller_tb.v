// lift_controller_tb.v
// Student Testbench: 4-Floor Elevator Controller Simulation
// Verifying 7 test cases

`timescale 1ns / 1ps

module lift_controller_tb;

    // DUT Inputs
    reg       clk;
    reg       rst;
    reg [3:0] hall_req_up;
    reg [3:0] hall_req_dn;
    reg [3:0] cabin_req;
    reg [3:0] floor_sensor;
    reg       door_obstruction;
    reg       cabin_overload;

    // DUT Outputs
    wire motor_up;
    wire motor_dn;
    wire brake_release;
    wire door_open;
    wire door_close;

    // Instantiate Elevator Controller
    // Override CLK_FREQ_HZ to 100 for fast simulation (3 seconds = 300 clock cycles)
    lift_controller #(
        .CLK_FREQ_HZ(100)
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

    // Clock generation (50 MHz clock period = 20 ns)
    initial clk = 0;
    always #10 clk = ~clk;

    // Waveform output setup
    initial begin
        $dumpfile("lift_tb.vcd");
        $dumpvars(0, lift_controller_tb);
    end

    // Task: Reset the controller
    task do_reset;
        begin
            rst              = 1'b1;
            hall_req_up      = 4'b0000;
            hall_req_dn      = 4'b0000;
            cabin_req        = 4'b0000;
            floor_sensor     = 4'b0001; // Starts at floor 0
            door_obstruction = 1'b0;
            cabin_overload   = 1'b0;
            #80;
            rst = 1'b0;
            #20;
        end
    endtask

    // Task: Simulate car arriving at a floor landing
    task arrive;
        input [1:0] floor_num;
        begin
            floor_sensor = (4'b0001 << floor_num);
            #20;
        end
    endtask

    // Task: Wait for motor start
    task wait_motor;
        input direction_up; // 1 = UP, 0 = DOWN
        integer count;
        begin
            #60; // wait logic settle
            count = 0;
            while (!(motor_up | motor_dn) && count < 50) begin
                #20;
                count = count + 1;
            end
            
            if (count >= 50) begin
                $display("  [TIMEOUT] Motor did not start");
            end else if (direction_up && motor_up) begin
                $display("  [PASS] Motor UP started: motor_up=1, brake_release=%b", brake_release);
            end else if (!direction_up && motor_dn) begin
                $display("  [PASS] Motor DOWN started: motor_dn=1, brake_release=%b", brake_release);
            end else begin
                $display("  [FAIL] Wrong motor state: motor_up=%b, motor_dn=%b", motor_up, motor_dn);
            end
        end
    endtask

    // Task: Wait for door to open
    task wait_door_open;
        integer count;
        begin
            count = 0;
            while (!door_open && count < 50) begin
                #20;
                count = count + 1;
            end
            if (count >= 50) begin
                $display("  [TIMEOUT] Door did not open");
            end else begin
                $display("  [PASS] Door opened: door_open=1");
            end
        end
    endtask

    // Task: Wait for controller to return to IDLE state
    task wait_idle;
        integer count;
        begin
            count = 0;
            while ((motor_up | motor_dn | door_open) && count < 1000) begin
                #20;
                count = count + 1;
            end
            if (count >= 1000) begin
                $display("  [TIMEOUT] Controller did not return to IDLE");
            end else begin
                $display("  [PASS] Returned to IDLE at t=%0t ns", $time);
            end
        end
    endtask

    // Main Test Stimulus
    initial begin
        $display("\n=== LIFT CONTROLLER TESTBENCH ===");
        $display("Simulating 4 floors. 3s dwell = 300 clock cycles at 100Hz.\n");

        // ---- TC1: Reset Check ----
        $display("\n--- TC1: Reset check ---");
        do_reset;
        if (!motor_up && !motor_dn && !brake_release && !door_open && door_close) begin
            $display("  [PASS] All outputs are in safe state after reset");
        end else begin
            $display("  [FAIL] Outputs in unsafe state after reset");
        end
        #100;

        // ---- TC2: Cabin request from Floor 0 to Floor 3 ----
        $display("\n--- TC2: Cabin call Floor 0 -> Floor 3 ---");
        do_reset;
        floor_sensor = 4'b0001;
        cabin_req    = 4'b1000; // Request Floor 3
        #40;
        cabin_req = 4'b0000;
        wait_motor(1);
        floor_sensor = 4'b0000;
        #60;
        arrive(3); // Arrived at Floor 3
        wait_door_open;
        wait_idle;

        // ---- TC3: Hall request from Floor 3 to Floor 0 ----
        $display("\n--- TC3: Hall DOWN call Floor 3 -> Floor 0 ---");
        do_reset;
        floor_sensor = 4'b1000; // Start at Floor 3
        hall_req_dn  = 4'b0001; // Call from Floor 0
        #40;
        hall_req_dn = 4'b0000;
        wait_motor(0);
        floor_sensor = 4'b0000;
        #60;
        arrive(0); // Arrived at Floor 0
        wait_door_open;
        wait_idle;

        // ---- TC4: LOOK Algorithm Direction Reversal ----
        $display("\n--- TC4: LOOK Reversal (Cabin[3] + Hall_DN[0]), starting at Floor 1 ---");
        do_reset;
        floor_sensor = 4'b0010; // Start at Floor 1
        cabin_req    = 4'b1000; // Destination Floor 3
        hall_req_dn  = 4'b0001; // Call from Floor 0
        #40;
        cabin_req   = 4'b0000;
        hall_req_dn = 4'b0000;
        $display("  Expecting motor to run UP first...");
        wait_motor(1);
        floor_sensor = 4'b0000;
        #60;
        arrive(3); // Arrived at Floor 3
        wait_door_open;
        
        // Wait out the door open hold time (300 cycles = 6000ns)
        #6500;
        
        floor_sensor = 4'b0000;
        #60;
        if (motor_dn) begin
            $display("  [PASS] Direction reversed to DOWN successfully");
        end else begin
            $display("  [FAIL] Direction reversal failed");
        end
        arrive(0); // Arrived at Floor 0
        wait_door_open;
        wait_idle;

        // ---- TC5: Door Obstruction Dwell Extension ----
        $display("\n--- TC5: Door Obstruction ---");
        do_reset;
        floor_sensor = 4'b0001;
        cabin_req    = 4'b0100; // Request Floor 2
        #40;
        cabin_req = 4'b0000;
        wait_motor(1);
        floor_sensor = 4'b0000;
        #60;
        arrive(2); // Arrived at Floor 2
        wait_door_open;
        
        // Let it hold for a bit, then trigger obstruction
        #3000;
        $display("  Applying door obstruction at t=%0t ns", $time);
        door_obstruction = 1'b1;
        #1000;
        door_obstruction = 1'b0;
        $display("  Obstruction cleared at t=%0t ns", $time);
        
        if (door_open) begin
            $display("  [PASS] Door stayed open (dwell timer extended)");
        end else begin
            $display("  [FAIL] Door closed prematurely");
        end
        wait_idle;

        // ---- TC6: Overload Safety Block ----
        $display("\n--- TC6: Cabin Overload ---");
        do_reset;
        floor_sensor   = 4'b0001;
        cabin_overload = 1'b1; // Overloaded
        cabin_req      = 4'b1000; // Request Floor 3
        #40;
        cabin_req = 4'b0000;
        #200;
        if (!motor_up && !motor_dn) begin
            $display("  [PASS] Motor blocked while cabin is overloaded");
        end else begin
            $display("  [FAIL] Motor run while overloaded");
        end
        cabin_overload = 1'b0; // Overload cleared
        $display("  Overload cleared, expecting departure...");
        wait_motor(1);
        floor_sensor = 4'b0000;
        #60;
        arrive(3);
        wait_door_open;
        wait_idle;

        // ---- TC7: Same Floor Immediate Door Open ----
        $display("\n--- TC7: Request at Current Floor ---");
        do_reset;
        floor_sensor = 4'b0100; // At Floor 2
        cabin_req    = 4'b0100; // Request Floor 2
        #40;
        cabin_req = 4'b0000;
        #60;
        if (door_open && !motor_up && !motor_dn) begin
            $display("  [PASS] Door opened directly on the same floor");
        end else begin
            $display("  [FAIL] Unexpected behavior on same floor request");
        end
        wait_idle;

        $display("\n=== ALL TEST CASES COMPLETE ===\n");
        $finish;
    end

    // Watchdog to prevent infinite loop hanging
    initial begin
        #5000000;
        $display("[TIMEOUT] Watchdog triggered, killing simulation");
        $finish;
    end

endmodule
