module car_cmd(
    input  wire clk,
    input  wire reset_n,

    input  wire [7:0] rx_data,
    input  wire       rx_done,

    output reg signed [12:0] motor_speed
);

    reg direction;

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            direction   <= 1'b1;
            motor_speed <= 13'sd0;
        end
        else if(rx_done) begin

            case(rx_data)

                8'h46: begin           // F
                    direction <= 1'b1;
                    motor_speed <= 13'sd1000;
                end

                8'h42: begin           // B
                    direction <= 1'b0;
                    motor_speed <= -13'sd1000;
                end

                8'h53: begin           // S
                    motor_speed <= 13'sd0;
                end

                8'h31: begin           // 1 = 20%
                    if(direction)
                        motor_speed <= 13'sd500;
                    else
                        motor_speed <= -13'sd500;
                end

                8'h32: begin           // 2 = 40%
                    if(direction)
                        motor_speed <= 13'sd1000;
                    else
                        motor_speed <= -13'sd1000;
                end

                8'h33: begin           // 3 = 60%
                    if(direction)
                        motor_speed <= 13'sd1500;
                    else
                        motor_speed <= -13'sd1500;
                end

                8'h34: begin           // 4 = 80%
                    if(direction)
                        motor_speed <= 13'sd2000;
                    else
                        motor_speed <= -13'sd2000;
                end

            endcase
        end
    end

endmodule