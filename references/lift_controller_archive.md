# Archive: Parameterized Lift Controller Project Session
**Date**: 03 June 2026  
**Context**: DRDO Internship Design & Verification (LOOK Algorithm Lift Controller)  
**Status**: Verilog simulation verified (100% pass), files organized, and directory cleaned up.

---

## 1. Technical Issues Identified & Resolved

### Bug 1: Testbench Input Race Conditions
- **Issue**: The testbench used blocking assignments (`=`) directly on positive clock edges, causing the simulator to evaluate inputs and clock edges in the same active region. This led to missed input pulses.
- **Fix**: Converted all stimulus assignments in [lift_controller_tb.v](file:///home/tyler/workspace/PROJECTS/lift_controller/tb/lift_controller_tb.v) to non-blocking assignments (`<=`).

### Bug 2: 1-Clock Cycle Target Update Delay
- **Issue**: `target_floor` was calculated sequentially inside the clock-edge block. Since `state` also transitions sequentially, the next-state combinational logic evaluated FSM changes using the *old* target floor value, triggering an incorrect `DOOR_OPEN` transition at the start floor.
- **Fix**: Implemented combinational target detection `target_floor_comb` inside a continuous evaluation block (`always @(*)`) and updated FSM next-state checks to evaluate this immediate value. The FSM now departs immediately to `MOVE_UP` or `MOVE_DOWN`.

### Bug 3: Current Floor Call Handling
- **Issue**: Calls made at the floor the lift was currently idling on were ignored or caused moving FSM glitches.
- **Fix**: Added `all_requests[curr_floor]` check to the top of the combinational scheduler to bypass direction sweep logic and open doors immediately.

---

## 2. Codebase Reference (Archived)

### RTL Controller Code (`lift_controller.v`)
```verilog
module lift_controller #(
    parameter NUM_FLOORS    = 4,
    parameter CLK_FREQ_HZ   = 50_000_000,
    parameter DOOR_DWELL_S  = 3
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire [NUM_FLOORS-1:0]  hall_req_up,
    input  wire [NUM_FLOORS-1:0]  hall_req_dn,
    input  wire [NUM_FLOORS-1:0]  cabin_req,
    input  wire [NUM_FLOORS-1:0]  floor_sensor,
    input  wire                   door_obstruction,
    input  wire                   cabin_overload,
    output reg                    motor_up,
    output reg                    motor_dn,
    output reg                    brake_release,
    output reg                    door_open,
    output reg                    door_close
);

    localparam FLOOR_WIDTH = $clog2(NUM_FLOORS);
    localparam DWELL_COUNT = DOOR_DWELL_S * CLK_FREQ_HZ;
    localparam DWELL_BITS  = $clog2(DWELL_COUNT + 1);

    localparam [2:0] IDLE       = 3'b000;
    localparam [2:0] MOVE_UP    = 3'b001;
    localparam [2:0] MOVE_DOWN  = 3'b011;
    localparam [2:0] DOOR_OPEN  = 3'b010;
    localparam [2:0] DOOR_HOLD  = 3'b110;

    reg [2:0]             state;
    reg [2:0]             next_state;
    reg [FLOOR_WIDTH-1:0] curr_floor;
    reg [FLOOR_WIDTH-1:0] target_floor;
    reg                   direction; // 1 = UP, 0 = DOWN

    reg [NUM_FLOORS-1:0]  req_up_reg;
    reg [NUM_FLOORS-1:0]  req_dn_reg;
    reg [NUM_FLOORS-1:0]  req_cabin_reg;
    reg [DWELL_BITS-1:0]  dwell_timer;

    wire [NUM_FLOORS-1:0] all_requests;
    wire                  any_request;
    wire                  at_target;
    wire                  dwell_done;

    reg                   request_above;
    reg                   request_below;
    reg [FLOOR_WIDTH-1:0] target_floor_comb;

    assign all_requests = req_up_reg | req_dn_reg | req_cabin_reg;
    assign any_request  = |all_requests;
    assign at_target = (target_floor < NUM_FLOORS) ? floor_sensor[target_floor] : 1'b0;
    assign dwell_done = (dwell_timer == {DWELL_BITS{1'b0}});

    integer k;
    always @(*) begin
        request_above = 1'b0;
        request_below = 1'b0;
        for (k = 0; k < NUM_FLOORS; k = k + 1) begin
            if (k > curr_floor && all_requests[k]) request_above = 1'b1;
            if (k < curr_floor && all_requests[k]) request_below = 1'b1;
        end
    end

    integer i;
    always @(posedge clk) begin
        if (rst) curr_floor <= {FLOOR_WIDTH{1'b0}};
        else begin
            for (i = 0; i < NUM_FLOORS; i = i + 1) begin
                if (floor_sensor[i]) curr_floor <= i[FLOOR_WIDTH-1:0];
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            req_up_reg    <= {NUM_FLOORS{1'b0}};
            req_dn_reg    <= {NUM_FLOORS{1'b0}};
            req_cabin_reg <= {NUM_FLOORS{1'b0}};
        end else begin
            req_up_reg    <= req_up_reg    | hall_req_up;
            req_dn_reg    <= req_dn_reg    | hall_req_dn;
            req_cabin_reg <= req_cabin_reg | cabin_req;
            if (state == DOOR_OPEN) begin
                req_up_reg[curr_floor]    <= 1'b0;
                req_dn_reg[curr_floor]    <= 1'b0;
                req_cabin_reg[curr_floor] <= 1'b0;
            end
        end
    end

    integer j;
    always @(*) begin : look_scheduler_comb
        target_floor_comb = target_floor;
        if (all_requests[curr_floor]) begin
            target_floor_comb = curr_floor;
        end else if (direction == 1'b1) begin
            if (request_above) begin
                for (j = NUM_FLOORS-1; j >= 0; j = j - 1) begin
                    if (j > curr_floor && all_requests[j]) target_floor_comb = j[FLOOR_WIDTH-1:0];
                end
            end else if (request_below) begin
                for (j = 0; j < NUM_FLOORS; j = j + 1) begin
                    if (j < curr_floor && all_requests[j]) target_floor_comb = j[FLOOR_WIDTH-1:0];
                end
            end
        end else begin
            if (request_below) begin
                for (j = 0; j < NUM_FLOORS; j = j + 1) begin
                    if (j < curr_floor && all_requests[j]) target_floor_comb = j[FLOOR_WIDTH-1:0];
                end
            end else if (request_above) begin
                for (j = NUM_FLOORS-1; j >= 0; j = j - 1) begin
                    if (j > curr_floor && all_requests[j]) target_floor_comb = j[FLOOR_WIDTH-1:0];
                end
            end
        end
    end

    always @(posedge clk) begin : look_scheduler_seq
        if (rst) begin
            target_floor <= {FLOOR_WIDTH{1'b0}};
            direction    <= 1'b1;
        end else if (state == IDLE || state == DOOR_HOLD) begin
            target_floor <= target_floor_comb;
            if (direction == 1'b1) begin
                if (!request_above && request_below) direction <= 1'b0;
            end else begin
                if (!request_below && request_above) direction <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) dwell_timer <= {DWELL_BITS{1'b0}};
        else if (state == DOOR_OPEN) dwell_timer <= DWELL_COUNT;
        else if (state == DOOR_HOLD) begin
            if (door_obstruction) dwell_timer <= DWELL_COUNT;
            else if (!dwell_done) dwell_timer <= dwell_timer - 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) state <= IDLE;
        else state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (any_request && !cabin_overload) begin
                    if (target_floor_comb == curr_floor) next_state = DOOR_OPEN;
                    else if (target_floor_comb > curr_floor) next_state = MOVE_UP;
                    else next_state = MOVE_DOWN;
                end
            end
            MOVE_UP: begin
                if (cabin_overload) next_state = IDLE;
                else if (at_target) next_state = DOOR_OPEN;
            end
            MOVE_DOWN: begin
                if (cabin_overload) next_state = IDLE;
                else if (at_target) next_state = DOOR_OPEN;
            end
            DOOR_OPEN: next_state = DOOR_HOLD;
            DOOR_HOLD: begin
                if (dwell_done && !door_obstruction && !cabin_overload) begin
                    if (any_request) begin
                        if (target_floor_comb > curr_floor) next_state = MOVE_UP;
                        else if (target_floor_comb < curr_floor) next_state = MOVE_DOWN;
                        else next_state = IDLE;
                    end else next_state = IDLE;
                end
            end
            default: next_state = IDLE;
        endcase
    end

    always @(*) begin
        motor_up      = 1'b0;
        motor_dn      = 1'b0;
        brake_release = 1'b0;
        door_open     = 1'b0;
        door_close    = 1'b0;
        case (state)
            IDLE: door_close = 1'b1;
            MOVE_UP: begin
                motor_up = 1'b1;
                brake_release = 1'b1;
                door_close = 1'b1;
            end
            MOVE_DOWN: begin
                motor_dn = 1'b1;
                brake_release = 1'b1;
                door_close = 1'b1;
            end
            DOOR_OPEN: door_open = 1'b1;
            DOOR_HOLD: door_open = 1'b1;
            default: ;
        endcase
    end
endmodule
```

### Xilinx Basys 3 Constraints (`lift_controller.xdc`)
```xdc
set_property PACKAGE_PIN W5 [get_ports clk]							
	set_property IOSTANDARD LVCMOS33 [get_ports clk]
	create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

set_property PACKAGE_PIN U18 [get_ports rst]						
	set_property IOSTANDARD LVCMOS33 [get_ports rst]

set_property PACKAGE_PIN V17 [get_ports {hall_req_up[0]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[0]}]
set_property PACKAGE_PIN V16 [get_ports {hall_req_up[1]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[1]}]
set_property PACKAGE_PIN W16 [get_ports {hall_req_up[2]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[2]}]
set_property PACKAGE_PIN W17 [get_ports {hall_req_up[3]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_up[3]}]

set_property PACKAGE_PIN W15 [get_ports {hall_req_dn[0]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[0]}]
set_property PACKAGE_PIN V15 [get_ports {hall_req_dn[1]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[1]}]
set_property PACKAGE_PIN W14 [get_ports {hall_req_dn[2]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[2]}]
set_property PACKAGE_PIN W13 [get_ports {hall_req_dn[3]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {hall_req_dn[3]}]

set_property PACKAGE_PIN V2 [get_ports {cabin_req[0]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[0]}]
set_property PACKAGE_PIN T3 [get_ports {cabin_req[1]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[1]}]
set_property PACKAGE_PIN T2 [get_ports {cabin_req[2]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[2]}]
set_property PACKAGE_PIN R3 [get_ports {cabin_req[3]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {cabin_req[3]}]

set_property PACKAGE_PIN W2 [get_ports {floor_sensor[0]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[0]}]
set_property PACKAGE_PIN U1 [get_ports {floor_sensor[1]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[1]}]
set_property PACKAGE_PIN T1 [get_ports {floor_sensor[2]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[2]}]
set_property PACKAGE_PIN R2 [get_ports {floor_sensor[3]}]					
	set_property IOSTANDARD LVCMOS33 [get_ports {floor_sensor[3]}]

set_property PACKAGE_PIN W19 [get_ports door_obstruction]						
	set_property IOSTANDARD LVCMOS33 [get_ports door_obstruction]
set_property PACKAGE_PIN T17 [get_ports cabin_overload]						
	set_property IOSTANDARD LVCMOS33 [get_ports cabin_overload]

set_property PACKAGE_PIN U16 [get_ports motor_up]					
	set_property IOSTANDARD LVCMOS33 [get_ports motor_up]
set_property PACKAGE_PIN E19 [get_ports motor_dn]					
	set_property IOSTANDARD LVCMOS33 [get_ports motor_dn]
set_property PACKAGE_PIN U19 [get_ports brake_release]					
	set_property IOSTANDARD LVCMOS33 [get_ports brake_release]
set_property PACKAGE_PIN V19 [get_ports door_open]					
	set_property IOSTANDARD LVCMOS33 [get_ports door_open]
set_property PACKAGE_PIN W18 [get_ports door_close]					
	set_property IOSTANDARD LVCMOS33 [get_ports door_close]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
```

---

## 3. Active Roadmaps for Next Sessions

### Arduino JARVIS Desk Robot
- Decide between an I2C adapter backpack and standard parallel wiring for 1602 LCD module connection to Arduino UNO.
- Wire peripheral controls: passive buzzer (`D9`), HC-SR04 ultrasonic rangefinder (`D2`/`D3`), DHT11 temperature/humidity (`D4`), IR Receiver (`D5`), and navigation buttons (`D6`/`D7`).

### Electricity Theft Detection
- Code ESP32 telemetry capture loops polling ZMPT101B and SCT-013 analog lines.
- Build training pipeline for the CNN-LSTM neural network to run local classification of stealing activities.

### Career Roadmap (GATE 2027)
- Review Digital Electronics and Computer Architecture basics for VLSI IIT Bombay admission (Target Score: 720-750).
- Master sequential logic patterns (D-FF metastabilities, sequence detectors) before implementing custom AMBA APB-to-AHB bus protocols.
