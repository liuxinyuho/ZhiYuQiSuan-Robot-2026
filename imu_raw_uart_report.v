//////////////////////////////////////////////////////////////////////////////////
// File Name   : imu_raw_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500六轴原始数据UART显示模块。
//
// Display:
//
//   AX=xxxx AY=xxxx AZ=xxxx
//   GX=xxxx GY=xxxx GZ=xxxx
//
// 数据均为16bit HEX。
//////////////////////////////////////////////////////////////////////////////////

module imu_raw_uart_report(
    input wire clk,
    input wire reset_n,

    input wire signed [15:0] accel_x,
    input wire signed [15:0] accel_y,
    input wire signed [15:0] accel_z,

    input wire signed [15:0] gyro_x,
    input wire signed [15:0] gyro_y,
    input wire signed [15:0] gyro_z,

    input wire data_valid,

    output wire uart_tx
);


    reg tx_send;
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
    // HEX -> ASCII
    //================================================

    function [7:0] hex_ascii;

        input [3:0] hex;

        begin

            if(hex < 4'd10)

                hex_ascii =
                    8'h30 + hex;

            else

                hex_ascii =
                    8'h41 + hex - 4'd10;

        end

    endfunction


    //================================================
    // 锁存数据
    //================================================

    reg [15:0] ax_latch;
    reg [15:0] ay_latch;
    reg [15:0] az_latch;

    reg [15:0] gx_latch;
    reg [15:0] gy_latch;
    reg [15:0] gz_latch;

    reg have_data;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            ax_latch <= 16'd0;
            ay_latch <= 16'd0;
            az_latch <= 16'd0;

            gx_latch <= 16'd0;
            gy_latch <= 16'd0;
            gz_latch <= 16'd0;

            have_data <= 1'b0;

        end

        else if(data_valid) begin

            ax_latch <= accel_x;
            ay_latch <= accel_y;
            az_latch <= accel_z;

            gx_latch <= gyro_x;
            gy_latch <= gyro_y;
            gz_latch <= gyro_z;

            have_data <= 1'b1;

        end

    end


    //================================================
    // 每200ms打印一次
    //
    // 50MHz × 0.2 = 10,000,000
    //================================================

    reg [23:0] report_cnt;

    reg [5:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 当前字符
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        case(char_index)

            //========================================
            // AX=xxxx
            //========================================

            6'd0 : tx_data = "A";
            6'd1 : tx_data = "X";
            6'd2 : tx_data = "=";

            6'd3 : tx_data = hex_ascii(ax_latch[15:12]);
            6'd4 : tx_data = hex_ascii(ax_latch[11:8]);
            6'd5 : tx_data = hex_ascii(ax_latch[7:4]);
            6'd6 : tx_data = hex_ascii(ax_latch[3:0]);

            6'd7 : tx_data = " ";


            //========================================
            // AY
            //========================================

            6'd8  : tx_data = "A";
            6'd9  : tx_data = "Y";
            6'd10 : tx_data = "=";

            6'd11 : tx_data = hex_ascii(ay_latch[15:12]);
            6'd12 : tx_data = hex_ascii(ay_latch[11:8]);
            6'd13 : tx_data = hex_ascii(ay_latch[7:4]);
            6'd14 : tx_data = hex_ascii(ay_latch[3:0]);

            6'd15 : tx_data = " ";


            //========================================
            // AZ
            //========================================

            6'd16 : tx_data = "A";
            6'd17 : tx_data = "Z";
            6'd18 : tx_data = "=";

            6'd19 : tx_data = hex_ascii(az_latch[15:12]);
            6'd20 : tx_data = hex_ascii(az_latch[11:8]);
            6'd21 : tx_data = hex_ascii(az_latch[7:4]);
            6'd22 : tx_data = hex_ascii(az_latch[3:0]);

            6'd23 : tx_data = 8'h0D;
            6'd24 : tx_data = 8'h0A;


            //========================================
            // GX
            //========================================

            6'd25 : tx_data = "G";
            6'd26 : tx_data = "X";
            6'd27 : tx_data = "=";

            6'd28 : tx_data = hex_ascii(gx_latch[15:12]);
            6'd29 : tx_data = hex_ascii(gx_latch[11:8]);
            6'd30 : tx_data = hex_ascii(gx_latch[7:4]);
            6'd31 : tx_data = hex_ascii(gx_latch[3:0]);

            6'd32 : tx_data = " ";


            //========================================
            // GY
            //========================================

            6'd33 : tx_data = "G";
            6'd34 : tx_data = "Y";
            6'd35 : tx_data = "=";

            6'd36 : tx_data = hex_ascii(gy_latch[15:12]);
            6'd37 : tx_data = hex_ascii(gy_latch[11:8]);
            6'd38 : tx_data = hex_ascii(gy_latch[7:4]);
            6'd39 : tx_data = hex_ascii(gy_latch[3:0]);

            6'd40 : tx_data = " ";


            //========================================
            // GZ
            //========================================

            6'd41 : tx_data = "G";
            6'd42 : tx_data = "Z";
            6'd43 : tx_data = "=";

            6'd44 : tx_data = hex_ascii(gz_latch[15:12]);
            6'd45 : tx_data = hex_ascii(gz_latch[11:8]);
            6'd46 : tx_data = hex_ascii(gz_latch[7:4]);
            6'd47 : tx_data = hex_ascii(gz_latch[3:0]);

            6'd48 : tx_data = 8'h0D;
            6'd49 : tx_data = 8'h0A;


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

            char_index <= 6'd0;

            sending <= 1'b0;

            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

        end

        else begin

            tx_send <= 1'b0;


            if(!sending) begin

                if(
                    have_data
                    &&
                    report_cnt >= 24'd9999999
                ) begin

                    report_cnt <= 24'd0;

                    char_index <= 6'd0;

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


                    if(char_index >= 6'd49) begin

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