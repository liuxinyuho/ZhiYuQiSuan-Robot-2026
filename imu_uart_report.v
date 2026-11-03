//////////////////////////////////////////////////////////////////////////////////
// File Name   : imu_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU9250调试信息UART输出模块。
//
//   每隔约500ms通过UART输出一次：
//
//       MPU_ID=71 ACK=0
//
//   若I2C通信失败，则可能显示：
//
//       MPU_ID=-- ACK=1
//
// UART :
//   115200bps
//   8 Data bits
//   No parity
//   1 Stop bit
//////////////////////////////////////////////////////////////////////////////////

module imu_uart_report(
    input wire clk,
    input wire reset_n,

    input wire [7:0] imu_id,
    input wire       imu_valid,
    input wire       imu_ack_error,

    output wire uart_tx
);


    //================================================
    // UART TX接口
    //================================================

    reg       tx_send;
    reg [7:0] tx_data;

    wire tx_done;
    wire tx_busy;


    uart_byte_tx u_uart_tx(
        .clk       (clk),
        .reset_n   (reset_n),

        .data_byte (tx_data),
        .send_en   (tx_send),

        .uart_tx   (uart_tx),

        .tx_done   (tx_done),
        .tx_busy   (tx_busy)
    );


    //================================================
    // HEX -> ASCII
    //================================================

    function [7:0] hex_to_ascii;

        input [3:0] hex;

        begin

            if(hex < 4'd10)

                hex_to_ascii =
                    8'h30 + hex;

            else

                hex_to_ascii =
                    8'h41 + (hex - 4'd10);

        end

    endfunction


    //================================================
    // 每500ms发送一次
    //
    // 50MHz × 0.5 = 25,000,000
    //================================================

    reg [24:0] report_cnt;


    //================================================
    // UART字符串发送状态
    //================================================

    reg [4:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 根据char_index产生当前ASCII字符
    //
    // MPU_ID=71 ACK=0\r\n
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        case(char_index)


            5'd0:
                tx_data = "M";


            5'd1:
                tx_data = "P";


            5'd2:
                tx_data = "U";


            5'd3:
                tx_data = "_";


            5'd4:
                tx_data = "I";


            5'd5:
                tx_data = "D";


            5'd6:
                tx_data = "=";


            // WHO_AM_I高4bit
            5'd7: begin

                if(imu_valid)
                    tx_data =
                        hex_to_ascii(imu_id[7:4]);
                else
                    tx_data = "-";

            end


            // WHO_AM_I低4bit
            5'd8: begin

                if(imu_valid)
                    tx_data =
                        hex_to_ascii(imu_id[3:0]);
                else
                    tx_data = "-";

            end


            5'd9:
                tx_data = " ";


            5'd10:
                tx_data = "A";


            5'd11:
                tx_data = "C";


            5'd12:
                tx_data = "K";


            5'd13:
                tx_data = "=";


            5'd14: begin

                if(imu_ack_error)
                    tx_data = "1";
                else
                    tx_data = "0";

            end


            // 回车
            5'd15:
                tx_data = 8'h0D;


            // 换行
            5'd16:
                tx_data = 8'h0A;


            default:
                tx_data = 8'h20;

        endcase

    end


    //================================================
    // UART字符串发送控制
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            report_cnt <= 25'd0;

            char_index <= 5'd0;

            sending <= 1'b0;

            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

        end

        else begin

            // 默认send只保持一个clk
            tx_send <= 1'b0;


            //========================================
            // 当前没有发送
            //========================================

            if(!sending) begin

                if(report_cnt >= 25'd24999999) begin

                    report_cnt <= 25'd0;

                    char_index <= 5'd0;

                    sending <= 1'b1;

                    waiting_tx <= 1'b0;

                end
                else begin

                    report_cnt <=
                        report_cnt + 1'b1;

                end

            end


            //========================================
            // 正在发送一行
            //========================================

            else begin


                // UART空闲
                // 开始发送当前字符
                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                // 当前字符发送完成
                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    // 最后一个字符发送完成
                    if(char_index >= 5'd16) begin

                        sending <= 1'b0;

                    end
                    else begin

                        char_index <=
                            char_index + 1'b1;

                    end

                end

            end

        end

    end


endmodule