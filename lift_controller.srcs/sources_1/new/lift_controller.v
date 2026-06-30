// lift_controller.v
// Saket Sankhla | DRDO Jodhpur Internship | June 2026
// Parameterized lift controller using LOOK scheduling algorithm
// Target : Xilinx Basys 3 (Artix-7) | Tool : Vivado 2020.1
// Standard : Verilog-2001

module lift_controller #(
    parameter NUM_FLOORS   = 4,
    parameter CLK_FREQ_HZ  = 50_000_000,
    parameter DOOR_DWELL_S = 3
)(
    input  wire                  clk,
    input  wire                  rst,

    // hall buttons and cabin buttons
    input  wire [NUM_FLOORS-1:0] hall_req_up,
    input  wire [NUM_FLOORS-1:0] hall_req_dn,
    input  wire [NUM_FLOORS-1:0] cabin_req,

    // sensors
    input  wire [NUM_FLOORS-1:0] floor_sensor,    // one-hot, 1 = car at that floor
    input  wire                  door_obstruction,
    input  wire                  cabin_overload,

    // motor and door outputs
    output reg                   motor_up,
    output reg                   motor_dn,
    output reg                   brake_release,
    output reg                   door_open,
    output reg                   door_close
);

// ---------------------------------------------------------------
// local parameters
// ---------------------------------------------------------------
localparam FLOOR_BITS  = $clog2(NUM_FLOORS);
localparam DWELL_COUNT = DOOR_DWELL_S * CLK_FREQ_HZ;
localparam TIMER_BITS  = $clog2(DWELL_COUNT + 1);

// FSM states - gray coded to avoid glitches during transitions
localparam [2:0] IDLE      = 3'b000;
localparam [2:0] MOVE_UP   = 3'b001;
localparam [2:0] MOVE_DOWN = 3'b011;
localparam [2:0] DOOR_OPEN = 3'b010;
localparam [2:0] DOOR_HOLD = 3'b110;

// ---------------------------------------------------------------
// internal registers
// ---------------------------------------------------------------
reg [2:0]            state, next_state;
reg [FLOOR_BITS-1:0] curr_floor;
reg [FLOOR_BITS-1:0] target_floor;
reg                  direction;        // 1 = going up, 0 = going down

// latched request registers - set by button press, cleared on service
reg [NUM_FLOORS-1:0] req_up;
reg [NUM_FLOORS-1:0] req_dn;
reg [NUM_FLOORS-1:0] req_cab;

reg [TIMER_BITS-1:0] dwell_timer;

// ---------------------------------------------------------------
// derived signals
// ---------------------------------------------------------------
wire [NUM_FLOORS-1:0] all_req = req_up | req_dn | req_cab;
wire any_req    = |all_req;
wire at_floor   = floor_sensor[target_floor];
wire timer_done = (dwell_timer == 0);

// combinational target - FSM reads this immediately (no 1-cycle delay)
reg [FLOOR_BITS-1:0] target_floor_comb;

// request_above and request_below - combinational
reg req_above, req_below;
integer k;
always @(*) begin
    req_above = 0;
    req_below = 0;
    for (k = 0; k < NUM_FLOORS; k = k + 1) begin
        if (k > curr_floor && all_req[k]) req_above = 1;
        if (k < curr_floor && all_req[k]) req_below = 1;
    end
end

// ---------------------------------------------------------------
// floor position decoder
// converts one-hot floor_sensor to binary curr_floor
// ---------------------------------------------------------------
integer i;
always @(posedge clk) begin
    if (rst) begin
        curr_floor <= 0;
    end else begin
        for (i = 0; i < NUM_FLOORS; i = i + 1) begin
            if (floor_sensor[i])
                curr_floor <= i[FLOOR_BITS-1:0];
        end
    end
end

// ---------------------------------------------------------------
// request latches
// buttons are latched until the floor is served
// ---------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        req_up  <= 0;
        req_dn  <= 0;
        req_cab <= 0;
    end else begin
        req_up  <= req_up  | hall_req_up;
        req_dn  <= req_dn  | hall_req_dn;
        req_cab <= req_cab | cabin_req;

        // clear requests for current floor when door opens
        if (state == DOOR_OPEN) begin
            req_up [curr_floor] <= 0;
            req_dn [curr_floor] <= 0;
            req_cab[curr_floor] <= 0;
        end
    end
end

// ---------------------------------------------------------------
// LOOK scheduler - combinational part
// computes target_floor_comb immediately so FSM can use it this cycle
// ---------------------------------------------------------------
integer j;
always @(*) begin : look_comb
    target_floor_comb = target_floor;   // default: hold current target

    // if request is at current floor, open door immediately
    if (all_req[curr_floor]) begin
        target_floor_comb = curr_floor;
    end else if (direction == 1) begin
        if (req_above) begin
            for (j = 0; j < NUM_FLOORS; j = j + 1)
                if (j > curr_floor && all_req[j])
                    target_floor_comb = j[FLOOR_BITS-1:0];
        end else if (req_below) begin
            for (j = NUM_FLOORS-1; j >= 0; j = j - 1)
                if (j < curr_floor && all_req[j])
                    target_floor_comb = j[FLOOR_BITS-1:0];
        end
    end else begin
        if (req_below) begin
            for (j = NUM_FLOORS-1; j >= 0; j = j - 1)
                if (j < curr_floor && all_req[j])
                    target_floor_comb = j[FLOOR_BITS-1:0];
        end else if (req_above) begin
            for (j = 0; j < NUM_FLOORS; j = j + 1)
                if (j > curr_floor && all_req[j])
                    target_floor_comb = j[FLOOR_BITS-1:0];
        end
    end
end

// LOOK scheduler - sequential part
// latches the combinational result and updates direction
always @(posedge clk) begin : look_seq
    if (rst) begin
        target_floor <= 0;
        direction    <= 1;
    end else if (state == IDLE || state == DOOR_HOLD) begin
        target_floor <= target_floor_comb;
        if (direction == 1) begin
            if (!req_above && req_below) direction <= 0;
        end else begin
            if (!req_below && req_above) direction <= 1;
        end
    end
end

// ---------------------------------------------------------------
// door dwell timer
// loads on DOOR_OPEN, counts down in DOOR_HOLD
// resets back to full if obstacle detected
// ---------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        dwell_timer <= 0;
    end else if (state == DOOR_OPEN) begin
        dwell_timer <= DWELL_COUNT;
    end else if (state == DOOR_HOLD) begin
        if (door_obstruction)
            dwell_timer <= DWELL_COUNT;   // restart timer if someone is in the door
        else if (!timer_done)
            dwell_timer <= dwell_timer - 1;
    end
end

// ---------------------------------------------------------------
// FSM state register
// ---------------------------------------------------------------
always @(posedge clk) begin
    if (rst)
        state <= IDLE;
    else
        state <= next_state;
end

// ---------------------------------------------------------------
// FSM next state logic (combinational)
// ---------------------------------------------------------------
always @(*) begin
    next_state = state;

    case (state)
        IDLE: begin
            if (any_req && !cabin_overload) begin
                // use target_floor_comb so FSM reacts immediately
                if (target_floor_comb == curr_floor)
                    next_state = DOOR_OPEN;
                else if (target_floor_comb > curr_floor)
                    next_state = MOVE_UP;
                else
                    next_state = MOVE_DOWN;
            end
        end

        MOVE_UP: begin
            if (cabin_overload)
                next_state = IDLE;
            else if (at_floor)
                next_state = DOOR_OPEN;
        end

        MOVE_DOWN: begin
            if (cabin_overload)
                next_state = IDLE;
            else if (at_floor)
                next_state = DOOR_OPEN;
        end

        DOOR_OPEN: begin
            // one clock pulse to actuate door, then hold
            next_state = DOOR_HOLD;
        end

        DOOR_HOLD: begin
            if (timer_done && !door_obstruction && !cabin_overload) begin
                if (any_req) begin
                    if (target_floor_comb > curr_floor)
                        next_state = MOVE_UP;
                    else if (target_floor_comb < curr_floor)
                        next_state = MOVE_DOWN;
                    else
                        next_state = IDLE;
                end else begin
                    next_state = IDLE;
                end
            end
        end

        default: next_state = IDLE;
    endcase
end

// ---------------------------------------------------------------
// output logic (Moore - outputs only depend on state)
// ---------------------------------------------------------------
always @(*) begin
    // safe defaults
    motor_up      = 0;
    motor_dn      = 0;
    brake_release = 0;
    door_open     = 0;
    door_close    = 0;

    case (state)
        IDLE: begin
            door_close = 1;
        end

        MOVE_UP: begin
            motor_up      = 1;
            brake_release = 1;
            door_close    = 1;
        end

        MOVE_DOWN: begin
            motor_dn      = 1;
            brake_release = 1;
            door_close    = 1;
        end

        DOOR_OPEN: begin
            door_open = 1;
        end

        DOOR_HOLD: begin
            door_open = 1;
        end

        default: ;
    endcase
end

// ---------------------------------------------------------------
// simulation safety checks (removed during synthesis)
// synthesis translate_off
// ---------------------------------------------------------------
always @(posedge clk) begin
    if (motor_up && motor_dn)
        $display("[ERROR] t=%0t : motor_up and motor_dn both HIGH", $time);

    if (motor_up && !brake_release)
        $display("[ERROR] t=%0t : motor running without brake release", $time);

    if (motor_dn && !brake_release)
        $display("[ERROR] t=%0t : motor running without brake release", $time);

    if (door_open && door_close)
        $display("[ERROR] t=%0t : door_open and door_close both HIGH", $time);

    if (cabin_overload && (motor_up || motor_dn))
        $display("[ERROR] t=%0t : motor running while overloaded", $time);
end
// synthesis translate_on

endmodule
