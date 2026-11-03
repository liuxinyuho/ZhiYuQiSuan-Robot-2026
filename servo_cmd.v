//////////////////////////////////////////////////////////////////////////////////
// File Name : servo_cmd.v
//
// UART Hex Commands:
//
//E1右
//E2中
//E3左
//////////////////////////////////////////////////////////////////////////////////

module servo_cmd(

    input wire clk,
    input wire reset_n,

    input wire [7:0] rx_data,
    input wire       rx_done,

    output reg [1:0] servo_position

);


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            // 上电默认居中
            servo_position <= 2'd1;

        end

        else begin

            if(rx_done) begin

                case(rx_data)

                    // 右
                    8'hE1:
                        servo_position <= 2'd0;

                    // 中
                    8'hE2:
                        servo_position <= 2'd1;

                    // 左
                    8'hE3:
                        servo_position <= 2'd2;

                    8'hE4: servo_position <= 2'd1;
                    // 自动避障启动
                    8'hA5:
                        servo_position <= 2'd1;
                    default:
                        servo_position <= servo_position;

                endcase

            end

        end

    end


endmodule