// lift_controller.v
// Student Project: 4-Floor Elevator Controller
// Board: Xilinx Basys 3 (Artix-7)
// Course: B.Tech ECE (7th Sem)

module lift_controller #(
    parameter CLK_FREQ_HZ = 50_000_000  // 50 MHz board clock
)(
    input  wire       clk,
    input  wire       rst,
    
    // Inputs from buttons
    input  wire [3:0] hall_req_up,      // UP requests from floor landings (0 to 3)
    input  wire [3:0] hall_req_dn,      // DOWN requests from floor landings (0 to 3)
    input  wire [3:0] cabin_req,        // Destination buttons inside the cabin
    
    // Proximity/Sensor inputs
    input  wire [3:0] floor_sensor,     // One-hot sensor input (e.g. 4'b0001 = Floor 0)
    input  wire       door_obstruction, // 1 if door sensor blocked
    input  wire       cabin_overload,   // 1 if cabin weight limit exceeded
    
    // Actuator outputs
    output reg        motor_up,         // Drive motor UP
    output reg        motor_dn,         // Drive motor DOWN
    output reg        brake_release,    // Release mechanical brake (active high)
    output reg        door_open,        // Command door to open
    output reg        door_close        // Command door to close
);

    // States for the Finite State Machine (FSM)
    parameter IDLE      = 3'b000;
    parameter MOVE_UP   = 3'b001;
    parameter MOVE_DOWN = 3'b010;
    parameter DOOR_OPEN = 3'b011;
    parameter DOOR_HOLD = 3'b100;
    
    reg [2:0] state;
    reg [2:0] next_state;
    
    // Position tracking variables
    reg [1:0] curr_floor;
    reg [1:0] target_floor;
    reg       direction; // 1 = UP, 0 = DOWN
    
    // Latched request registers
    reg [3:0] req_up;
    reg [3:0] req_dn;
    reg [3:0] req_cab;
    
    // Combine all requests into a single 4-bit bus
    wire [3:0] all_req = req_up | req_dn | req_cab;
    wire       any_request = |all_req;
    
    // Timer register for door open dwell (3 seconds at CLK_FREQ_HZ clock cycles)
    localparam DWELL_COUNT = 3 * CLK_FREQ_HZ;
    reg [27:0] dwell_timer;
    
    // Binary floor decoding from one-hot floor_sensor
    always @(posedge clk) begin
        if (rst) begin
            curr_floor <= 2'b00;
        end else begin
            if (floor_sensor[0]) curr_floor <= 2'd0;
            else if (floor_sensor[1]) curr_floor <= 2'd1;
            else if (floor_sensor[2]) curr_floor <= 2'd2;
            else if (floor_sensor[3]) curr_floor <= 2'd3;
        end
    end
    
    // Latch calls until serviced
    always @(posedge clk) begin
        if (rst) begin
            req_up  <= 4'b0000;
            req_dn  <= 4'b0000;
            req_cab <= 4'b0000;
        end else begin
            // Latch button inputs
            req_up  <= req_up | hall_req_up;
            req_dn  <= req_dn | hall_req_dn;
            req_cab <= req_cab | cabin_req;
            
            // Clear current floor requests when doors are open
            if (state == DOOR_OPEN) begin
                req_up[curr_floor]  <= 1'b0;
                req_dn[curr_floor]  <= 1'b0;
                req_cab[curr_floor] <= 1'b0;
            end
        end
    end
    
    // LOOK Algorithm Scheduler: Check pending calls above/below position
    reg req_above;
    reg req_below;
    
    always @(*) begin
        req_above = 1'b0;
        req_below = 1'b0;
        
        // Find if there are any requests on floors higher than curr_floor
        if (curr_floor == 2'd0) begin
            if (all_req[1] || all_req[2] || all_req[3]) req_above = 1'b1;
        end else if (curr_floor == 2'd1) begin
            if (all_req[2] || all_req[3]) req_above = 1'b1;
        end else if (curr_floor == 2'd2) begin
            if (all_req[3]) req_above = 1'b1;
        end
        
        // Find if there are any requests on floors lower than curr_floor
        if (curr_floor == 2'd3) begin
            if (all_req[2] || all_req[1] || all_req[0]) req_below = 1'b1;
        end else if (curr_floor == 2'd2) begin
            if (all_req[1] || all_req[0]) req_below = 1'b1;
        end else if (curr_floor == 2'd1) begin
            if (all_req[0]) req_below = 1'b1;
        end
    end
    
    // Determine next target floor based on LOOK algorithm
    always @(posedge clk) begin
        if (rst) begin
            target_floor <= 2'b00;
            direction    <= 1'b1; // Default is UP
        end else if (state == IDLE || state == DOOR_HOLD) begin
            // 1. Request is on the current floor, stay and serve it
            if (all_req[curr_floor]) begin
                target_floor <= curr_floor;
            end
            // 2. If moving UP, continue checking upwards
            else if (direction == 1'b1) begin
                if (req_above) begin
                    if (curr_floor < 2'd1 && all_req[1]) target_floor <= 2'd1;
                    else if (curr_floor < 2'd2 && all_req[2]) target_floor <= 2'd2;
                    else if (curr_floor < 2'd3 && all_req[3]) target_floor <= 2'd3;
                end else if (req_below) begin
                    direction <= 1'b0; // Reverse direction
                    if (curr_floor > 2'd2 && all_req[2]) target_floor <= 2'd2;
                    else if (curr_floor > 2'd1 && all_req[1]) target_floor <= 2'd1;
                    else if (curr_floor > 2'd0 && all_req[0]) target_floor <= 2'd0;
                end
            end
            // 3. If moving DOWN, continue checking downwards
            else begin
                if (req_below) begin
                    if (curr_floor > 2'd2 && all_req[2]) target_floor <= 2'd2;
                    else if (curr_floor > 2'd1 && all_req[1]) target_floor <= 2'd1;
                    else if (curr_floor > 2'd0 && all_req[0]) target_floor <= 2'd0;
                end else if (req_above) begin
                    direction <= 1'b1; // Reverse direction
                    if (curr_floor < 2'd1 && all_req[1]) target_floor <= 2'd1;
                    else if (curr_floor < 2'd2 && all_req[2]) target_floor <= 2'd2;
                    else if (curr_floor < 2'd3 && all_req[3]) target_floor <= 2'd3;
                end
            end
        end
    end
    
    // Door open timer
    always @(posedge clk) begin
        if (rst) begin
            dwell_timer <= 28'd0;
        end else if (state == DOOR_OPEN) begin
            dwell_timer <= DWELL_COUNT;
        end else if (state == DOOR_HOLD) begin
            if (door_obstruction) begin
                dwell_timer <= DWELL_COUNT; // Reload timer if door is blocked
            end else if (dwell_timer > 28'd0) begin
                dwell_timer <= dwell_timer - 28'd1;
            end
        end
    end
    
    // FSM state register sequential block
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    // FSM next-state combinational block
    always @(*) begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (any_request && !cabin_overload) begin
                    // If request on current floor, open doors immediately
                    if (all_req[curr_floor]) begin
                        next_state = DOOR_OPEN;
                    end
                    // Travel up if calls are above and we are oriented UP
                    else if (req_above && direction == 1'b1) begin
                        next_state = MOVE_UP;
                    end
                    // Travel down if calls are below and we are oriented DOWN
                    else if (req_below && direction == 1'b0) begin
                        next_state = MOVE_DOWN;
                    end
                    // Reversal cases
                    else if (req_above) begin
                        next_state = MOVE_UP;
                    end else if (req_below) begin
                        next_state = MOVE_DOWN;
                    end
                end
            end
            
            MOVE_UP: begin
                if (cabin_overload) begin
                    next_state = IDLE;
                end else if (floor_sensor[target_floor]) begin
                    next_state = DOOR_OPEN;
                end
            end
            
            MOVE_DOWN: begin
                if (cabin_overload) begin
                    next_state = IDLE;
                end else if (floor_sensor[target_floor]) begin
                    next_state = DOOR_OPEN;
                end
            end
            
            DOOR_OPEN: begin
                next_state = DOOR_HOLD;
            end
            
            DOOR_HOLD: begin
                if (dwell_timer == 28'd0 && !door_obstruction && !cabin_overload) begin
                    if (any_request) begin
                        // If call on current floor, open doors again
                        if (all_req[curr_floor]) begin
                            next_state = DOOR_OPEN;
                        end
                        else if (req_above && direction == 1'b1) next_state = MOVE_UP;
                        else if (req_below && direction == 1'b0) next_state = MOVE_DOWN;
                        else if (req_above) next_state = MOVE_UP;
                        else if (req_below) next_state = MOVE_DOWN;
                        else next_state = IDLE;
                    end else begin
                        next_state = IDLE;
                    end
                end
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Actuator outputs combinational block
    always @(*) begin
        // Safe default state (stopped, doors closed)
        motor_up      = 1'b0;
        motor_dn      = 1'b0;
        brake_release = 1'b0;
        door_open     = 1'b0;
        door_close    = 1'b0;
        
        case (state)
            IDLE: begin
                door_close = 1'b1;
            end
            
            MOVE_UP: begin
                motor_up      = 1'b1;
                brake_release = 1'b1;
                door_close    = 1'b1;
            end
            
            MOVE_DOWN: begin
                motor_dn      = 1'b1;
                brake_release = 1'b1;
                door_close    = 1'b1;
            end
            
            DOOR_OPEN: begin
                door_open = 1'b1;
            end
            
            DOOR_HOLD: begin
                door_open = 1'b1;
            end
            
            default: ;
        endcase
    end

endmodule
