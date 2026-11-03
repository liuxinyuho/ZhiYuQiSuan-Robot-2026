//////////////////////////////////////////////////////////////////////////////////
// File Name   : imu_debug_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500通信诊断串口输出
//
// Output Example:
//
//   CAL=1 YAW=+012.34 VALID=00100 ERR=00000
//
// Function:
//
//   YAW   : 当前航向角，单位 degree
//   VALID : 最近1秒内 imu_data_valid 次数
//   ERR   : 最近1秒内 read_error 上升沿次数
//
// Clock : 50MHz
// UART  : 115200
//////////////////////////////////////////////////////////////////////////////////

module imu_debug_uart_report #(

    parameter integer CLK_FREQ  = 50000000,
    parameter integer BAUD_RATE = 115200

)(
    input wire clk,
    input wire reset_n,

    input wire calib_done,

    input wire data_valid,

    input wire read_error,

    // 单位0.01°
    input wire signed [31:0] yaw_cdeg,

    output reg uart_tx
);


    //================================================
    // UART parameter
    //================================================

    localparam integer CLKS_PER_BIT =
        CLK_FREQ / BAUD_RATE;


    //================================================
    // 1 second counter
    //================================================

    reg [31:0] one_sec_counter;


    //================================================
    // IMU counters
    //================================================

    reg [15:0] valid_counter;
    reg [15:0] error_counter;

    reg [15:0] valid_latched;
    reg [15:0] error_latched;


    //================================================
    // read_error rising edge
    //================================================

    reg read_error_d;

    wire read_error_rise;


    assign read_error_rise =

        read_error

        &&

        !read_error_d;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n)

            read_error_d <= 1'b0;

        else

            read_error_d <= read_error;

    end


    //================================================
    // Report request
    //================================================

    reg report_request;


    //================================================
    // Count VALID / ERROR every second
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            one_sec_counter <= 32'd0;

            valid_counter <= 16'd0;
            error_counter <= 16'd0;

            valid_latched <= 16'd0;
            error_latched <= 16'd0;

            report_request <= 1'b0;

        end

        else begin

            report_request <= 1'b0;


            //========================================
            // data_valid一般为一个CLK脉冲
            //========================================

            if(data_valid)

                valid_counter <=
                    valid_counter + 1'b1;


            //========================================
            // error只统计上升沿
            // 防止error保持高电平时疯狂累加
            //========================================

            if(read_error_rise)

                error_counter <=
                    error_counter + 1'b1;


            //========================================
            // 每1秒锁存一次
            //========================================

            if(
                one_sec_counter
                >=
                CLK_FREQ - 1
            ) begin

                one_sec_counter <= 32'd0;

                valid_latched <=
                    valid_counter;

                error_latched <=
                    error_counter;

                valid_counter <= 16'd0;

                error_counter <= 16'd0;

                report_request <= 1'b1;

            end

            else begin

                one_sec_counter <=
                    one_sec_counter + 1'b1;

            end

        end

    end


    //================================================
    // Yaw latch
    //================================================

    reg [31:0] yaw_mag;
    reg        yaw_negative;

    reg        calib_latched;


    //================================================
    // Decimal helper
    // 5 digits
    //================================================

    function [7:0] decimal_digit5;

        input [31:0] value;
        input [2:0]  pos;

        reg [31:0] digit;

        begin

            case(pos)

                3'd0:
                    digit =
                        (value / 32'd10000) % 10;

                3'd1:
                    digit =
                        (value / 32'd1000) % 10;

                3'd2:
                    digit =
                        (value / 32'd100) % 10;

                3'd3:
                    digit =
                        (value / 32'd10) % 10;

                default:
                    digit =
                        value % 10;

            endcase


            decimal_digit5 =
                8'd48 + digit[7:0];

        end

    endfunction


    //================================================
    // Yaw digit
    //
    // yaw_cdeg = 1234
    // display 012.34
    //================================================

    function [7:0] yaw_digit;

        input [31:0] value;
        input [2:0]  pos;

        reg [31:0] digit;

        begin

            case(pos)

                // 百度
                3'd0:
                    digit =
                        (value / 32'd10000) % 10;

                // 十度
                3'd1:
                    digit =
                        (value / 32'd1000) % 10;

                // 个度
                3'd2:
                    digit =
                        (value / 32'd100) % 10;

                // 0.1°
                3'd3:
                    digit =
                        (value / 32'd10) % 10;

                // 0.01°
                default:
                    digit =
                        value % 10;

            endcase


            yaw_digit =
                8'd48 + digit[7:0];

        end

    endfunction


    //================================================
    // Message
    //
    // CAL=1 YAW=+012.34 VALID=00100 ERR=00000\r\n
    //
    // 41 bytes
    //================================================

    reg [5:0] tx_index;

    reg [7:0] msg_char;


    always @(*) begin

        msg_char = 8'h20;


        case(tx_index)

            // CAL=
            6'd0:  msg_char = "C";
            6'd1:  msg_char = "A";
            6'd2:  msg_char = "L";
            6'd3:  msg_char = "=";

            6'd4:
                msg_char =
                    calib_latched
                    ?
                    "1"
                    :
                    "0";

            6'd5:  msg_char = " ";


            // YAW=
            6'd6:  msg_char = "Y";
            6'd7:  msg_char = "A";
            6'd8:  msg_char = "W";
            6'd9:  msg_char = "=";


            // Yaw sign
            6'd10:
                msg_char =
                    yaw_negative
                    ?
                    "-"
                    :
                    "+";


            // 012.34
            6'd11:
                msg_char =
                    yaw_digit(yaw_mag, 3'd0);

            6'd12:
                msg_char =
                    yaw_digit(yaw_mag, 3'd1);

            6'd13:
                msg_char =
                    yaw_digit(yaw_mag, 3'd2);

            6'd14:
                msg_char = ".";

            6'd15:
                msg_char =
                    yaw_digit(yaw_mag, 3'd3);

            6'd16:
                msg_char =
                    yaw_digit(yaw_mag, 3'd4);


            6'd17:
                msg_char = " ";


            // VALID=
            6'd18: msg_char = "V";
            6'd19: msg_char = "A";
            6'd20: msg_char = "L";
            6'd21: msg_char = "I";
            6'd22: msg_char = "D";
            6'd23: msg_char = "=";


            6'd24:
                msg_char =
                    decimal_digit5(
                        valid_latched,
                        3'd0
                    );

            6'd25:
                msg_char =
                    decimal_digit5(
                        valid_latched,
                        3'd1
                    );

            6'd26:
                msg_char =
                    decimal_digit5(
                        valid_latched,
                        3'd2
                    );

            6'd27:
                msg_char =
                    decimal_digit5(
                        valid_latched,
                        3'd3
                    );

            6'd28:
                msg_char =
                    decimal_digit5(
                        valid_latched,
                        3'd4
                    );


            6'd29:
                msg_char = " ";


            // ERR=
            6'd30: msg_char = "E";
            6'd31: msg_char = "R";
            6'd32: msg_char = "R";
            6'd33: msg_char = "=";


            6'd34:
                msg_char =
                    decimal_digit5(
                        error_latched,
                        3'd0
                    );

            6'd35:
                msg_char =
                    decimal_digit5(
                        error_latched,
                        3'd1
                    );

            6'd36:
                msg_char =
                    decimal_digit5(
                        error_latched,
                        3'd2
                    );

            6'd37:
                msg_char =
                    decimal_digit5(
                        error_latched,
                        3'd3
                    );

            6'd38:
                msg_char =
                    decimal_digit5(
                        error_latched,
                        3'd4
                    );


            // CR LF
            6'd39:
                msg_char = 8'h0D;

            6'd40:
                msg_char = 8'h0A;


            default:
                msg_char = 8'h20;

        endcase

    end


    //================================================
    // Message sender
    //================================================

    reg sending;

    reg [7:0] tx_data;
    reg       tx_start;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            sending <= 1'b0;

            tx_index <= 6'd0;

            tx_data <= 8'd0;

            tx_start <= 1'b0;

            yaw_mag <= 32'd0;

            yaw_negative <= 1'b0;

            calib_latched <= 1'b0;

        end

        else begin

            tx_start <= 1'b0;


            //========================================
            // Start new report
            //========================================

            if(
                report_request
                &&
                !sending
            ) begin

                sending <= 1'b1;

                tx_index <= 6'd0;

                calib_latched <=
                    calib_done;


                if(yaw_cdeg < 0) begin

                    yaw_negative <= 1'b1;

                    yaw_mag <=
                        -yaw_cdeg;

                end

                else begin

                    yaw_negative <= 1'b0;

                    yaw_mag <=
                        yaw_cdeg;

                end

            end


            //========================================
            // Send next byte
            //========================================

            else if(
                sending
                &&
                !uart_busy
                &&
                !tx_start
            ) begin

                tx_data <=
                    msg_char;

                tx_start <=
                    1'b1;


                if(tx_index >= 6'd40) begin

                    sending <= 1'b0;

                    tx_index <= 6'd0;

                end

                else begin

                    tx_index <=
                        tx_index + 1'b1;

                end

            end

        end

    end


    //================================================
    // UART TX
    //================================================

    reg uart_busy;

    reg [15:0] baud_counter;

    reg [3:0] bit_index;

    reg [7:0] tx_shift;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            uart_tx <= 1'b1;

            uart_busy <= 1'b0;

            baud_counter <= 16'd0;

            bit_index <= 4'd0;

            tx_shift <= 8'd0;

        end

        else begin


            //========================================
            // Start byte
            //========================================

            if(
                tx_start
                &&
                !uart_busy
            ) begin

                uart_busy <=
                    1'b1;

                baud_counter <=
                    16'd0;

                bit_index <=
                    4'd0;

                tx_shift <=
                    tx_data;

                // start bit
                uart_tx <=
                    1'b0;

            end


            else if(uart_busy) begin


                if(
                    baud_counter
                    >=
                    CLKS_PER_BIT - 1
                ) begin

                    baud_counter <=
                        16'd0;


                    case(bit_index)

                        // data bit0
                        4'd0: begin
                            uart_tx <= tx_shift[0];
                            bit_index <= 4'd1;
                        end

                        4'd1: begin
                            uart_tx <= tx_shift[1];
                            bit_index <= 4'd2;
                        end

                        4'd2: begin
                            uart_tx <= tx_shift[2];
                            bit_index <= 4'd3;
                        end

                        4'd3: begin
                            uart_tx <= tx_shift[3];
                            bit_index <= 4'd4;
                        end

                        4'd4: begin
                            uart_tx <= tx_shift[4];
                            bit_index <= 4'd5;
                        end

                        4'd5: begin
                            uart_tx <= tx_shift[5];
                            bit_index <= 4'd6;
                        end

                        4'd6: begin
                            uart_tx <= tx_shift[6];
                            bit_index <= 4'd7;
                        end

                        4'd7: begin
                            uart_tx <= tx_shift[7];
                            bit_index <= 4'd8;
                        end

                        // stop bit
                        4'd8: begin
                            uart_tx <= 1'b1;
                            bit_index <= 4'd9;
                        end

                        // finish
                        default: begin

                            uart_tx <= 1'b1;

                            uart_busy <= 1'b0;

                            bit_index <= 4'd0;

                        end

                    endcase

                end

                else begin

                    baud_counter <=
                        baud_counter + 1'b1;

                end

            end

            else begin

                uart_tx <= 1'b1;

            end

        end

    end


endmodule