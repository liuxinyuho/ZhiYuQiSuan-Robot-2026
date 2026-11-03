//////////////////////////////////////////////////////////////////////////////////
// File Name   : imu_init_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500初始化状态UART调试输出。
//
// Display:
//
//   IMU_INIT=WAIT
//
//   或
//
//   IMU_INIT=OK
//
//   或
//
//   IMU_INIT=ERR
//
// UART:
//   115200 / 8N1
//////////////////////////////////////////////////////////////////////////////////

module imu_init_uart_report(
    input wire clk,
    input wire reset_n,

    input wire init_done,
    input wire init_error,

    output wire uart_tx
);


    //================================================
    // UART
    //================================================

    reg       tx_send;
    reg [7:0] tx_data;

    wire tx_busy;
    wire tx_done;


    uart_byte_tx u_uart_byte_tx(

        .clk       (clk),
        .reset_n   (reset_n),

        .data_byte (tx_data),
        .send_en   (tx_send),

        .uart_tx   (uart_tx),

        .tx_done   (tx_done),
        .tx_busy   (tx_busy)

    );


    //================================================
    // 每500ms输出一次
    //================================================

    reg [24:0] report_cnt;

    reg [4:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 固定输出15个字符
    //
    // IMU_INIT=WAIT\r\n
    //
    // IMU_INIT=OK  \r\n
    //
    // IMU_INIT=ERR \r\n
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        case(char_index)

            5'd0 : tx_data = "I";
            5'd1 : tx_data = "M";
            5'd2 : tx_data = "U";
            5'd3 : tx_data = "_";
            5'd4 : tx_data = "I";
            5'd5 : tx_data = "N";
            5'd6 : tx_data = "I";
            5'd7 : tx_data = "T";
            5'd8 : tx_data = "=";


            //========================================
            // 状态字符 9~12
            //========================================

            5'd9: begin

                if(init_error)
                    tx_data = "E";

                else if(init_done)
                    tx_data = "O";

                else
                    tx_data = "W";

            end


            5'd10: begin

                if(init_error)
                    tx_data = "R";

                else if(init_done)
                    tx_data = "K";

                else
                    tx_data = "A";

            end


            5'd11: begin

                if(init_error)
                    tx_data = "R";

                else if(init_done)
                    tx_data = " ";

                else
                    tx_data = "I";

            end


            5'd12: begin

                if(init_error)
                    tx_data = " ";

                else if(init_done)
                    tx_data = " ";

                else
                    tx_data = "T";

            end


            // CR
            5'd13:
                tx_data = 8'h0D;


            // LF
            5'd14:
                tx_data = 8'h0A;


            default:
                tx_data = 8'h20;

        endcase

    end


    //================================================
    // UART发送状态机
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

            tx_send <= 1'b0;


            //========================================
            // 等待500ms
            //========================================

            if(!sending) begin

                if(report_cnt >= 25'd24999999) begin

                    report_cnt <= 25'd0;

                    char_index <= 5'd0;

                    sending <= 1'b1;

                    waiting_tx <= 1'b0;

                end

                else begin

                    report_cnt <= report_cnt + 1'b1;

                end

            end


            //========================================
            // 发送一行
            //========================================

            else begin


                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    if(char_index >= 5'd14) begin

                        sending <= 1'b0;

                    end

                    else begin

                        char_index <= char_index + 1'b1;

                    end

                end

            end

        end

    end

endmodule