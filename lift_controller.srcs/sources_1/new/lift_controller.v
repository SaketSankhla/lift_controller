 module lift_controller #(
        parameter CLK_FREQ_HZ = 50_000_000
    )(
        input  wire       clk, rst, door_obstruction, cabin_overload,
        input  wire [3:0] hall_req_up, hall_req_dn, cabin_req, floor_sensor,
        output reg        motor_up, motor_dn, brake_release, door_open, door_close
    );
        // FSM States
        parameter IDLE    = 2'b00;
        parameter MOVE_UP = 2'b01;
        parameter MOVE_DN = 2'b10;
        parameter DOOR    = 2'b11;

        reg [1:0]  state;
        reg [1:0]  curr;
        reg [1:0]  target;
        reg [3:0]  reqs;
        reg [27:0] timer;

        // Temporary variable for immediate target evaluation
        reg [1:0]  next_target;

        always @(posedge clk) begin
            if (rst) begin
                state         <= IDLE;
                curr          <= 2'b00;
                target        <= 2'b00;
                reqs          <= 4'b0000;
                timer         <= 28'd0;
                motor_up      <= 1'b0;
                motor_dn      <= 1'b0;
                brake_release <= 1'b0;
                door_open     <= 1'b0;
                door_close    <= 1'b1;
            end else begin
                // 1. Decode one-hot floor sensor to binary current floor
                if (floor_sensor[0])      curr <= 2'd0;
                else if (floor_sensor[1]) curr <= 2'd1;
                else if (floor_sensor[2]) curr <= 2'd2;
                else if (floor_sensor[3]) curr <= 2'd3;

                // 2. Latch incoming button requests
                reqs <= reqs | hall_req_up | hall_req_dn | cabin_req;

                // Default temporary target
                next_target = target;

                // 3. FSM Logic
                case (state)
                    IDLE: begin
                        // Idle state outputs
                        motor_up      <= 1'b0;
                        motor_dn      <= 1'b0;
                        brake_release <= 1'b0;
                        door_open     <= 1'b0;
                        door_close    <= 1'b1;

                        if (reqs != 4'b0000 && !cabin_overload) begin
                            // If call is on the current floor, open doors immediately
                            if (reqs[curr]) begin
                                state      <= DOOR;
                                timer      <= 3 * CLK_FREQ_HZ;
                                reqs[curr] <= 1'b0; // Clear serviced request
                            end
                            // Otherwise, scan requests to set target floor
                            else begin
                                if (reqs[3])      next_target = 2'd3;
                                else if (reqs[2]) next_target = 2'd2;
                                else if (reqs[1]) next_target = 2'd1;
                                else              next_target = 2'd0;

                                target <= next_target; // Save to target register

                                // Determine direction based on target floor
                                if (next_target > curr) begin
                                    state <= MOVE_UP;
                                end else if (next_target < curr) begin
                                    state <= MOVE_DN;
                                end
                            end
                        end
                    end

                    MOVE_UP: begin
                        // Move UP outputs
                        motor_up      <= 1'b1;
                        motor_dn      <= 1'b0;
                        brake_release <= 1'b1;
                        door_open     <= 1'b0;
                        door_close    <= 1'b1;

                        if (cabin_overload) begin
                            state <= IDLE;
                        end else if (floor_sensor[target]) begin
                            state        <= DOOR;
                            timer        <= 3 * CLK_FREQ_HZ;
                            reqs[target] <= 1'b0; // Clear serviced request
                        end
                    end

                    MOVE_DN: begin
                        // Move DOWN outputs
                        motor_up      <= 1'b0;
                        motor_dn      <= 1'b1;
                        brake_release <= 1'b1;
                        door_open     <= 1'b0;
                        door_close    <= 1'b1;

                        if (cabin_overload) begin
                            state <= IDLE;
                        end else if (floor_sensor[target]) begin
                            state        <= DOOR;
                            timer        <= 3 * CLK_FREQ_HZ;
                            reqs[target] <= 1'b0; // Clear serviced request
                        end
                    end

                    DOOR: begin
                        // Door open hold outputs
                        motor_up      <= 1'b0;
                        motor_dn      <= 1'b0;
                        brake_release <= 1'b0;
                        door_open     <= 1'b1;
                        door_close    <= 1'b0;

                        if (door_obstruction) begin
                            timer <= 3 * CLK_FREQ_HZ; // Reload timer if door is blocked
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

    endmodule