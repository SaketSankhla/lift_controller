// lift_controller.v
// Simple 4-Floor Elevator Controller (Textbook Style)
// Student Lab Project | B.Tech ECE (7th Sem)

module lift_controller #(
    parameter CLK_FREQ_HZ = 50_000_000  // Default board clock frequency
)(
    input  wire       clk,
    input  wire       rst,
    
    // Inputs from buttons
    input  wire [3:0] hall_req_up,      // UP requests from floors 0, 1, 2, 3
    input  wire [3:0] hall_req_dn,      // DOWN requests from floors 0, 1, 2, 3
    input  wire [3:0] cabin_req,        // Destination calls inside the cabin
    
    // Inputs from sensors
    input  wire [3:0] floor_sensor,     // One-hot sensor input (e.g. 4'b0010 = Floor 1)
    input  wire       door_obstruction, // 1 if door is blocked
    input  wire       cabin_overload,   // 1 if cabin is overloaded
    
    // Actuator outputs
    output reg        motor_up,         // Turn motor on to go UP
    output reg        motor_dn,         // Turn motor on to go DOWN
    output reg        brake_release,    // Release mechanical brake (active high)
    output reg        door_open,        // Command door to open
    output reg        door_close        // Command door to close
);

    // FSM States
    parameter IDLE = 2'b00;
    parameter UP   = 2'b01;
    parameter DN   = 2'b10;
    parameter OPEN = 2'b11;
    
    reg [1:0]  state;
    reg [1:0]  next_state;
    
    // Floor tracking registers
    reg [1:0]  curr_floor;
    reg [1:0]  target_floor;
    reg        direction; // 1 = UP, 0 = DOWN
    
    // Call latch registers
    reg [3:0]  req_up;
    reg [3:0]  req_dn;
    reg [3:0]  req_cab;
    
    // Combined calls
    wire [3:0] all_req = req_up | req_dn | req_cab;
    wire       any_req = |all_req;
    
    // Dwell timer register (3 seconds)
    localparam DWELL_COUNT = 3 * CLK_FREQ_HZ;
    reg [27:0] timer;

    // 1. Position decoder (One-Hot to Binary conversion)
    always @(posedge clk) begin
        if (rst) begin
            curr_floor <= 2'b00;
        end else begin
            if (floor_sensor[0])      curr_floor <= 2'd0;
            else if (floor_sensor[1]) curr_floor <= 2'd1;
            else if (floor_sensor[2]) curr_floor <= 2'd2;
            else if (floor_sensor[3]) curr_floor <= 2'd3;
        end
    end

    // 2. Request latching and clears
    always @(posedge clk) begin
        if (rst) begin
            req_up  <= 4'b0000;
            req_dn  <= 4'b0000;
            req_cab <= 4'b0000;
        end else begin
            // Latch button clicks
            req_up  <= req_up | hall_req_up;
            req_dn  <= req_dn | hall_req_dn;
            req_cab <= req_cab | cabin_req;
            
            // Clear current floor requests when doors are open
            if (state == OPEN) begin
                req_up[curr_floor]  <= 1'b0;
                req_dn[curr_floor]  <= 1'b0;
                req_cab[curr_floor] <= 1'b0;
            end
        end
    end

    // 3. Combinational target determination for LOOK scheduler
    reg [1:0] target_comb;
    always @(*) begin
        target_comb = target_floor; // Default
        if (all_req[curr_floor]) begin
            target_comb = curr_floor;
        end else begin
            case (curr_floor)
                2'd0: begin
                    if (all_req[1])      target_comb = 2'd1;
                    else if (all_req[2]) target_comb = 2'd2;
                    else if (all_req[3]) target_comb = 2'd3;
                end
                
                2'd1: begin
                    if (direction == 1'b1) begin
                        if (all_req[2])      target_comb = 2'd2;
                        else if (all_req[3]) target_comb = 2'd3;
                        else if (all_req[0]) target_comb = 2'd0;
                    end else begin
                        if (all_req[0])      target_comb = 2'd0;
                        else if (all_req[2]) target_comb = 2'd2;
                        else if (all_req[3]) target_comb = 2'd3;
                    end
                end

                2'd2: begin
                    if (direction == 1'b1) begin
                        if (all_req[3])      target_comb = 2'd3;
                        else if (all_req[1]) target_comb = 2'd1;
                        else if (all_req[0]) target_comb = 2'd0;
                    end else begin
                        if (all_req[1])      target_comb = 2'd1;
                        else if (all_req[0])      target_comb = 2'd0;
                        else if (all_req[3]) target_comb = 2'd3;
                    end
                end

                2'd3: begin
                    if (all_req[2])      target_comb = 2'd2;
                    else if (all_req[1]) target_comb = 2'd1;
                    else if (all_req[0]) target_comb = 2'd0;
                end
            endcase
        end
    end

    // 4. Sequential target and direction update
    always @(posedge clk) begin
        if (rst) begin
            target_floor <= 2'b00;
            direction    <= 1'b1; // Default to UP
        end else if (state == IDLE || state == OPEN) begin
            target_floor <= target_comb;
            // Update direction flag
            if (curr_floor == 2'd0) direction <= 1'b1;
            else if (curr_floor == 2'd3) direction <= 1'b0;
            else if (target_comb > curr_floor) direction <= 1'b1;
            else if (target_comb < curr_floor) direction <= 1'b0;
        end
    end

    // 5. Door hold timer control
    always @(posedge clk) begin
        if (rst) begin
            timer <= 28'd0;
        end else if (state == IDLE && any_req && all_req[curr_floor]) begin
            timer <= DWELL_COUNT; // Load timer on same-floor request
        end else if (state == UP || state == DN) begin
            if (floor_sensor[target_floor]) begin
                timer <= DWELL_COUNT; // Load timer upon arrival
            end
        end else if (state == OPEN) begin
            if (door_obstruction) begin
                timer <= DWELL_COUNT; // Reload timer if door is blocked
            end else if (timer > 28'd0) begin
                timer <= timer - 28'd1;
            end
        end
    end

    // 6. FSM Sequential state register
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // 7. FSM Combinational next-state decoder
    always @(*) begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (any_req && !cabin_overload) begin
                    if (target_comb == curr_floor)
                        next_state = OPEN;
                    else if (target_comb > curr_floor)
                        next_state = UP;
                    else
                        next_state = DN;
                end
            end
            
            UP: begin
                if (cabin_overload) begin
                    next_state = IDLE;
                end else if (floor_sensor[target_floor]) begin
                    next_state = OPEN;
                end
            end
            
            DN: begin
                if (cabin_overload) begin
                    next_state = IDLE;
                end else if (floor_sensor[target_floor]) begin
                    next_state = OPEN;
                end
            end
            
            OPEN: begin
                if (timer == 28'd0 && !door_obstruction && !cabin_overload) begin
                    next_state = IDLE;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end

    // 8. Moore Outputs Combinational Decoder
    always @(*) begin
        motor_up      = (state == UP);
        motor_dn      = (state == DN);
        brake_release = (state == UP || state == DN);
        door_open     = (state == OPEN);
        door_close    = (state == IDLE || state == UP || state == DN);
    end

endmodule
