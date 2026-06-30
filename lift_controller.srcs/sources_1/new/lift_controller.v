// lift_controller.v
// Condensed 4-Floor Elevator Controller (FPGA-Generic)
// Student Project | B.Tech ECE

module lift_controller #(
    parameter CLK_FREQ_HZ = 50_000_000
)(
    input  wire       clk, rst, door_obstruction, cabin_overload,
    input  wire [3:0] hall_req_up, hall_req_dn, cabin_req, floor_sensor,
    output reg        motor_up, motor_dn, brake_release, door_open, door_close
);
    // States
    localparam IDLE = 2'b00, MOVE = 2'b01, OPEN = 2'b10;
    
    reg [1:0]  state;
    reg [1:0]  curr, target;
    reg        dir; // 1 = UP, 0 = DOWN
    reg [3:0]  reqs;
    reg [27:0] timer;

    // Decode current floor position
    always @(posedge clk) begin
        if (floor_sensor[0])      curr <= 2'd0;
        else if (floor_sensor[1]) curr <= 2'd1;
        else if (floor_sensor[2]) curr <= 2'd2;
        else if (floor_sensor[3]) curr <= 2'd3;
    end

    // Latch button requests and clear serviced floor requests
    always @(posedge clk) begin
        if (rst) begin
            reqs <= 4'b0000;
        end else begin
            reqs <= (reqs | hall_req_up | hall_req_dn | cabin_req);
            if (state == OPEN) begin
                reqs[curr] <= 1'b0;
            end
        end
    end

    // LOOK Algorithm: Check pending requests above and below
    wire req_above = (curr == 0 && (reqs[1] || reqs[2] || reqs[3])) ||
                     (curr == 1 && (reqs[2] || reqs[3])) ||
                     (curr == 2 && reqs[3]);
                     
    wire req_below = (curr == 3 && (reqs[2] || reqs[1] || reqs[0])) ||
                     (curr == 2 && (reqs[1] || reqs[0])) ||
                     (curr == 1 && reqs[0]);

    // FSM State Register & Controller Logic
    always @(posedge clk) begin
        if (rst) begin
            state  <= IDLE;
            target <= 2'b00;
            dir    <= 1'b1; // Default: UP
            timer  <= 28'd0;
        end else begin
            case (state)
                IDLE: begin
                    if (reqs != 4'b0000 && !cabin_overload) begin
                        if (reqs[curr]) begin
                            state <= OPEN;
                            timer <= 3 * CLK_FREQ_HZ; // Load 3-second timer for same floor request
                        end else begin
                            state <= MOVE;
                            // Set target floor based on LOOK logic
                            if (dir && req_above)
                                target <= reqs[3] ? 2'd3 : (reqs[2] ? 2'd2 : 2'd1);
                            else if (!dir && req_below)
                                target <= reqs[0] ? 2'd0 : (reqs[1] ? 2'd1 : 2'd2);
                            else begin
                                dir    <= ~dir; // Change direction
                                target <= (~dir) ? (reqs[3] ? 2'd3 : (reqs[2] ? 2'd2 : 2'd1))
                                                 : (reqs[0] ? 2'd0 : (reqs[1] ? 2'd1 : 2'd2));
                            end
                        end
                    end
                end

                MOVE: begin
                    if (cabin_overload) begin
                        state <= IDLE;
                    end else if (floor_sensor[target]) begin
                        state <= OPEN;
                        timer <= 3 * CLK_FREQ_HZ; // Load 3-second timer
                    end
                end

                OPEN: begin
                    if (door_obstruction) begin
                        timer <= 3 * CLK_FREQ_HZ; // Reset timer if door blocked
                    end else if (timer > 28'd0) begin
                        timer <= timer - 28'd1;
                    end else begin
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    // Moore Output Decoders
    always @(*) begin
        motor_up      = (state == MOVE) && (target > curr);
        motor_dn      = (state == MOVE) && (target < curr);
        brake_release = (state == MOVE);
        door_open     = (state == OPEN);
        door_close    = (state == IDLE || state == MOVE);
    end

endmodule
