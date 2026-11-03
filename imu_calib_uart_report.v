//////////////////////////////////////////////////////////////////////////////////
// File Name   : imu_calib_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500陀螺仪零偏校准调试输出模块。
//
//   串口周期性显示：
//
//   CAL=WAIT
//
//   或校准完成后：
//
//   CAL=OK
//   BX=xxxx BY=xxxx BZ=xxxx
//   GXC=xxxxxxxx GYC=xxxxxxxx GZC=xxxxxxxx
//
// Note :
//   BX/BY/BZ：16bit陀螺仪零偏
//
//   GXC/GYC/GZC：
//   已经执行 Raw - Bias 后的32bit有符号结果。
//
// UART :
//   115200 / 8N1
//////////////////////////////////////////////////////////////////////////////////

module imu_calib_uart_report(
    input wire clk,
    input wire reset_n,

    //================================================
    // 校准状态
    //================================================
    input wire calib_done,

    //================================================
    // 零偏
    //================================================
    input wire signed [15:0] bias_x,
    input wire signed [15:0] bias_y,
    input wire signed [15:0] bias_z,

    //================================================
    // 校准后的陀螺仪
    //================================================
    input wire signed [31:0] gyro_x_corrected,
    input wire signed [31:0] gyro_y_corrected,
    input wire signed [31:0] gyro_z_corrected,

    output wire uart_tx
);


    //================================================
    // UART接口
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
    // HEX转ASCII
    //================================================

    function [7:0] hex_ascii;

        input [3:0] hex;

        begin

            if(hex < 4'd10)

                hex_ascii = 8'h30 + hex;

            else

                hex_ascii =
                    8'h41 + hex - 4'd10;

        end

    endfunction


    //================================================
    // 将当前数据锁存
    // 防止发送过程中数值变化
    //================================================

    reg        calib_latch;

    reg [15:0] bx_latch;
    reg [15:0] by_latch;
    reg [15:0] bz_latch;

    reg [31:0] gxc_latch;
    reg [31:0] gyc_latch;
    reg [31:0] gzc_latch;


    //================================================
    // 每200ms打印一次
    //
    // 50MHz × 0.2s = 10,000,000
    //================================================

    reg [23:0] report_cnt;


    reg [6:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 字符生成
    //
    // 输出格式：
    //
    // CAL=OK
    // BX=xxxx BY=xxxx BZ=xxxx
    // GXC=xxxxxxxx GYC=xxxxxxxx GZC=xxxxxxxx
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        case(char_index)

            //========================================
            // CAL=OK / WAIT
            //========================================

            7'd0  : tx_data = "C";
            7'd1  : tx_data = "A";
            7'd2  : tx_data = "L";
            7'd3  : tx_data = "=";


            7'd4: begin

                if(calib_latch)
                    tx_data = "O";
                else
                    tx_data = "W";

            end


            7'd5: begin

                if(calib_latch)
                    tx_data = "K";
                else
                    tx_data = "A";

            end


            7'd6: begin

                if(calib_latch)
                    tx_data = " ";
                else
                    tx_data = "I";

            end


            7'd7: begin

                if(calib_latch)
                    tx_data = " ";
                else
                    tx_data = "T";

            end


            7'd8  : tx_data = 8'h0D;
            7'd9  : tx_data = 8'h0A;


            //========================================
            // BX=xxxx
            //========================================

            7'd10 : tx_data = "B";
            7'd11 : tx_data = "X";
            7'd12 : tx_data = "=";

            7'd13 : tx_data = hex_ascii(bx_latch[15:12]);
            7'd14 : tx_data = hex_ascii(bx_latch[11:8]);
            7'd15 : tx_data = hex_ascii(bx_latch[7:4]);
            7'd16 : tx_data = hex_ascii(bx_latch[3:0]);

            7'd17 : tx_data = " ";


            //========================================
            // BY=xxxx
            //========================================

            7'd18 : tx_data = "B";
            7'd19 : tx_data = "Y";
            7'd20 : tx_data = "=";

            7'd21 : tx_data = hex_ascii(by_latch[15:12]);
            7'd22 : tx_data = hex_ascii(by_latch[11:8]);
            7'd23 : tx_data = hex_ascii(by_latch[7:4]);
            7'd24 : tx_data = hex_ascii(by_latch[3:0]);

            7'd25 : tx_data = " ";


            //========================================
            // BZ=xxxx
            //========================================

            7'd26 : tx_data = "B";
            7'd27 : tx_data = "Z";
            7'd28 : tx_data = "=";

            7'd29 : tx_data = hex_ascii(bz_latch[15:12]);
            7'd30 : tx_data = hex_ascii(bz_latch[11:8]);
            7'd31 : tx_data = hex_ascii(bz_latch[7:4]);
            7'd32 : tx_data = hex_ascii(bz_latch[3:0]);

            7'd33 : tx_data = 8'h0D;
            7'd34 : tx_data = 8'h0A;


            //========================================
            // GXC=xxxxxxxx
            //========================================

            7'd35 : tx_data = "G";
            7'd36 : tx_data = "X";
            7'd37 : tx_data = "C";
            7'd38 : tx_data = "=";

            7'd39 : tx_data = hex_ascii(gxc_latch[31:28]);
            7'd40 : tx_data = hex_ascii(gxc_latch[27:24]);
            7'd41 : tx_data = hex_ascii(gxc_latch[23:20]);
            7'd42 : tx_data = hex_ascii(gxc_latch[19:16]);
            7'd43 : tx_data = hex_ascii(gxc_latch[15:12]);
            7'd44 : tx_data = hex_ascii(gxc_latch[11:8]);
            7'd45 : tx_data = hex_ascii(gxc_latch[7:4]);
            7'd46 : tx_data = hex_ascii(gxc_latch[3:0]);

            7'd47 : tx_data = " ";


            //========================================
            // GYC=xxxxxxxx
            //========================================

            7'd48 : tx_data = "G";
            7'd49 : tx_data = "Y";
            7'd50 : tx_data = "C";
            7'd51 : tx_data = "=";

            7'd52 : tx_data = hex_ascii(gyc_latch[31:28]);
            7'd53 : tx_data = hex_ascii(gyc_latch[27:24]);
            7'd54 : tx_data = hex_ascii(gyc_latch[23:20]);
            7'd55 : tx_data = hex_ascii(gyc_latch[19:16]);
            7'd56 : tx_data = hex_ascii(gyc_latch[15:12]);
            7'd57 : tx_data = hex_ascii(gyc_latch[11:8]);
            7'd58 : tx_data = hex_ascii(gyc_latch[7:4]);
            7'd59 : tx_data = hex_ascii(gyc_latch[3:0]);

            7'd60 : tx_data = " ";


            //========================================
            // GZC=xxxxxxxx
            //========================================

            7'd61 : tx_data = "G";
            7'd62 : tx_data = "Z";
            7'd63 : tx_data = "C";
            7'd64 : tx_data = "=";

            7'd65 : tx_data = hex_ascii(gzc_latch[31:28]);
            7'd66 : tx_data = hex_ascii(gzc_latch[27:24]);
            7'd67 : tx_data = hex_ascii(gzc_latch[23:20]);
            7'd68 : tx_data = hex_ascii(gzc_latch[19:16]);
            7'd69 : tx_data = hex_ascii(gzc_latch[15:12]);
            7'd70 : tx_data = hex_ascii(gzc_latch[11:8]);
            7'd71 : tx_data = hex_ascii(gzc_latch[7:4]);
            7'd72 : tx_data = hex_ascii(gzc_latch[3:0]);

            7'd73 : tx_data = 8'h0D;
            7'd74 : tx_data = 8'h0A;


            default:
                tx_data = 8'h20;

        endcase

    end


    //================================================
    // UART发送控制
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            report_cnt <= 24'd0;

            char_index <= 7'd0;

            sending <= 1'b0;

            waiting_tx <= 1'b0;

            tx_send <= 1'b0;


            calib_latch <= 1'b0;

            bx_latch <= 16'd0;
            by_latch <= 16'd0;
            bz_latch <= 16'd0;

            gxc_latch <= 32'd0;
            gyc_latch <= 32'd0;
            gzc_latch <= 32'd0;

        end

        else begin

            tx_send <= 1'b0;


            if(!sending) begin

                if(report_cnt >= 24'd9999999) begin

                    report_cnt <= 24'd0;


                    // 锁存当前结果
                    calib_latch <= calib_done;

                    bx_latch <= bias_x;
                    by_latch <= bias_y;
                    bz_latch <= bias_z;

                    gxc_latch <= gyro_x_corrected;
                    gyc_latch <= gyro_y_corrected;
                    gzc_latch <= gyro_z_corrected;


                    char_index <= 7'd0;

                    sending <= 1'b1;

                    waiting_tx <= 1'b0;

                end

                else begin

                    report_cnt <=
                        report_cnt + 1'b1;

                end

            end


            else begin

                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    if(char_index >= 7'd74) begin

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