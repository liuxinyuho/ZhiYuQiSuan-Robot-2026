//////////////////////////////////////////////////////////////////////////////////
// File Name   : odom_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   周期性通过UART输出底盘里程计信息。
//
// Output:
//   CAL=1 X=+0000500 Y=-0000019 YAW=-006.65
//
// UART:
//   115200, 8N1
//
// Clock:
//   50MHz
//
// 优化：
//   原版本在组合逻辑中进行大量 /10、/100、%10 等运算，
//   导致严重 Setup Timing 违例。
//   新版本使用逐周期 Binary -> BCD 转换，
//   UART发送阶段只进行简单MUX，不再使用硬件除法器。
//////////////////////////////////////////////////////////////////////////////////

module odom_uart_report #(

    parameter integer CLK_FREQ  = 50000000,
    parameter integer BAUD_RATE = 115200,
    parameter integer REPORT_MS = 200

)(
    input wire clk,
    input wire reset_n,

    //================================================
    // IMU校准
    //================================================
    input wire calib_done,

    //================================================
    // 里程计
    //================================================
    input wire signed [31:0] pos_x_count,
    input wire signed [31:0] pos_y_count,

    // 单位：0.01°
    input wire signed [31:0] yaw_cdeg,

    //================================================
    // UART TX
    //================================================
    output reg uart_tx
);


    //================================================
    // UART参数
    //================================================

    localparam integer CLKS_PER_BIT =
        CLK_FREQ / BAUD_RATE;

    localparam integer REPORT_CYCLES =
        (CLK_FREQ / 1000) * REPORT_MS;


    //================================================
    // 状态
    //================================================

    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_PREP_X     = 4'd1,
        ST_CONV_X     = 4'd2,
        ST_PREP_Y     = 4'd3,
        ST_CONV_Y     = 4'd4,
        ST_PREP_YAW   = 4'd5,
        ST_CONV_YAW   = 4'd6,
        ST_SEND       = 4'd7,
        ST_WAIT_LAST  = 4'd8;


    reg [3:0] state;


    //================================================
    // 报告定时
    //================================================

    reg [31:0] report_counter;


    //================================================
    // 锁存原始数据
    //================================================

    reg [31:0] x_mag_latched;
    reg [31:0] y_mag_latched;
    reg [31:0] yaw_mag_latched;

    reg x_negative;
    reg y_negative;
    reg yaw_negative;

    reg calib_latched;


    //================================================
    // BCD结果
    //
    // 40bit = 10个十进制数字
    //
    // [3:0]   个位
    // [7:4]   十位
    // ...
    //================================================

    reg [39:0] x_bcd;
    reg [39:0] y_bcd;
    reg [39:0] yaw_bcd;


    //================================================
    // Binary -> BCD 公共转换器
    //================================================

    reg [31:0] binary_shift;
    reg [39:0] bcd_shift;

    reg [5:0] convert_count;


    //================================================
    // Double Dabble:
    // 每个BCD数字 >=5 时先 +3
    //================================================

    reg [39:0] bcd_adjusted;

    integer i;

    always @(*) begin

        bcd_adjusted = bcd_shift;

        for(i = 0; i < 10; i = i + 1) begin

            if(
                bcd_adjusted[
                    i*4 +: 4
                ]
                >=
                4'd5
            ) begin

                bcd_adjusted[
                    i*4 +: 4
                ]
                =
                bcd_adjusted[
                    i*4 +: 4
                ]
                +
                4'd3;

            end

        end

    end


    //================================================
    // 本周期移位后的BCD结果
    //================================================

    wire [39:0] bcd_shift_next;

    assign bcd_shift_next = {
        bcd_adjusted[38:0],
        binary_shift[31]
    };


    //================================================
    // UART消息
    //================================================

    reg [5:0] tx_index;

    reg [7:0] tx_data;
    reg       tx_start;

    reg [7:0] msg_char;


    //================================================
    // 一个BCD数字 -> ASCII
    //================================================

    function [7:0] bcd_to_ascii;

        input [3:0] digit;

        begin

            bcd_to_ascii =
                8'h30 + {4'd0, digit};

        end

    endfunction


    //================================================
    // UART字符串
    //
    // CAL=1 X=+0000500 Y=-0000019 YAW=-006.65
    //
    // 共41字节
    //================================================

    always @(*) begin

        msg_char = 8'h20;


        case(tx_index)

            //----------------------------------------
            // CAL=
            //----------------------------------------

            6'd0:
                msg_char = "C";

            6'd1:
                msg_char = "A";

            6'd2:
                msg_char = "L";

            6'd3:
                msg_char = "=";

            6'd4:
                msg_char =
                    calib_latched
                    ?
                    "1"
                    :
                    "0";

            6'd5:
                msg_char = " ";


            //----------------------------------------
            // X=
            //----------------------------------------

            6'd6:
                msg_char = "X";

            6'd7:
                msg_char = "=";

            6'd8:
                msg_char =
                    x_negative
                    ?
                    "-"
                    :
                    "+";


            //----------------------------------------
            // X: 7位
            //
            // 百万 ~ 个位
            //----------------------------------------

            6'd9:
                msg_char =
                    bcd_to_ascii(x_bcd[27:24]);

            6'd10:
                msg_char =
                    bcd_to_ascii(x_bcd[23:20]);

            6'd11:
                msg_char =
                    bcd_to_ascii(x_bcd[19:16]);

            6'd12:
                msg_char =
                    bcd_to_ascii(x_bcd[15:12]);

            6'd13:
                msg_char =
                    bcd_to_ascii(x_bcd[11:8]);

            6'd14:
                msg_char =
                    bcd_to_ascii(x_bcd[7:4]);

            6'd15:
                msg_char =
                    bcd_to_ascii(x_bcd[3:0]);


            6'd16:
                msg_char = " ";


            //----------------------------------------
            // Y=
            //----------------------------------------

            6'd17:
                msg_char = "Y";

            6'd18:
                msg_char = "=";

            6'd19:
                msg_char =
                    y_negative
                    ?
                    "-"
                    :
                    "+";


            //----------------------------------------
            // Y: 7位
            //----------------------------------------

            6'd20:
                msg_char =
                    bcd_to_ascii(y_bcd[27:24]);

            6'd21:
                msg_char =
                    bcd_to_ascii(y_bcd[23:20]);

            6'd22:
                msg_char =
                    bcd_to_ascii(y_bcd[19:16]);

            6'd23:
                msg_char =
                    bcd_to_ascii(y_bcd[15:12]);

            6'd24:
                msg_char =
                    bcd_to_ascii(y_bcd[11:8]);

            6'd25:
                msg_char =
                    bcd_to_ascii(y_bcd[7:4]);

            6'd26:
                msg_char =
                    bcd_to_ascii(y_bcd[3:0]);


            6'd27:
                msg_char = " ";


            //----------------------------------------
            // YAW=
            //----------------------------------------

            6'd28:
                msg_char = "Y";

            6'd29:
                msg_char = "A";

            6'd30:
                msg_char = "W";

            6'd31:
                msg_char = "=";

            6'd32:
                msg_char =
                    yaw_negative
                    ?
                    "-"
                    :
                    "+";


            //----------------------------------------
            // YAW:
            //
            // yaw单位0.01°
            //
            // 1234 -> 012.34
            //----------------------------------------

            // 百度
            6'd33:
                msg_char =
                    bcd_to_ascii(yaw_bcd[19:16]);

            // 十度
            6'd34:
                msg_char =
                    bcd_to_ascii(yaw_bcd[15:12]);

            // 个度
            6'd35:
                msg_char =
                    bcd_to_ascii(yaw_bcd[11:8]);

            // 小数点
            6'd36:
                msg_char = ".";

            // 0.1°
            6'd37:
                msg_char =
                    bcd_to_ascii(yaw_bcd[7:4]);

            // 0.01°
            6'd38:
                msg_char =
                    bcd_to_ascii(yaw_bcd[3:0]);


            //----------------------------------------
            // CR LF
            //----------------------------------------

            6'd39:
                msg_char = 8'h0D;

            6'd40:
                msg_char = 8'h0A;


            default:
                msg_char = 8'h20;

        endcase

    end


    //================================================
    // UART底层
    //================================================

    reg uart_busy;

    reg [15:0] baud_counter;
    reg [3:0]  bit_index;

    reg [7:0] tx_shift;


    //================================================
    // 高层状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            report_counter <= 32'd0;

            x_mag_latched <= 32'd0;
            y_mag_latched <= 32'd0;
            yaw_mag_latched <= 32'd0;

            x_negative <= 1'b0;
            y_negative <= 1'b0;
            yaw_negative <= 1'b0;

            calib_latched <= 1'b0;

            x_bcd <= 40'd0;
            y_bcd <= 40'd0;
            yaw_bcd <= 40'd0;

            binary_shift <= 32'd0;
            bcd_shift <= 40'd0;

            convert_count <= 6'd0;

            tx_index <= 6'd0;

            tx_data <= 8'd0;
            tx_start <= 1'b0;

        end

        else begin

            //================================================
            // tx_start永远只保持一个clk
            //================================================

            tx_start <= 1'b0;


            case(state)


                //================================================
                // 等待200ms
                //================================================

                ST_IDLE: begin

                    if(
                        report_counter
                        >=
                        REPORT_CYCLES - 1
                    ) begin

                        report_counter <= 32'd0;


                        //========================================
                        // 锁存校准状态
                        //========================================

                        calib_latched <=
                            calib_done;


                        //========================================
                        // X绝对值
                        //========================================

                        x_negative <=
                            pos_x_count[31];

                        if(pos_x_count[31])

                            x_mag_latched <=
                                (~pos_x_count) + 32'd1;

                        else

                            x_mag_latched <=
                                pos_x_count;


                        //========================================
                        // Y绝对值
                        //========================================

                        y_negative <=
                            pos_y_count[31];

                        if(pos_y_count[31])

                            y_mag_latched <=
                                (~pos_y_count) + 32'd1;

                        else

                            y_mag_latched <=
                                pos_y_count;


                        //========================================
                        // YAW绝对值
                        //========================================

                        yaw_negative <=
                            yaw_cdeg[31];

                        if(yaw_cdeg[31])

                            yaw_mag_latched <=
                                (~yaw_cdeg) + 32'd1;

                        else

                            yaw_mag_latched <=
                                yaw_cdeg;


                        // 下一状态
                        state <= ST_PREP_X;

                    end

                    else begin

                        report_counter <=
                            report_counter + 1'b1;

                    end

                end


                //================================================
                // 准备转换X
                //================================================

                ST_PREP_X: begin

                    binary_shift <=
                        x_mag_latched;

                    bcd_shift <= 40'd0;

                    convert_count <= 6'd0;

                    state <= ST_CONV_X;

                end


                //================================================
                // X Binary -> BCD
                //================================================

                ST_CONV_X: begin

                    binary_shift <= {
                        binary_shift[30:0],
                        1'b0
                    };

                    bcd_shift <=
                        bcd_shift_next;


                    if(convert_count == 6'd31) begin

                        x_bcd <=
                            bcd_shift_next;

                        convert_count <= 6'd0;

                        state <= ST_PREP_Y;

                    end

                    else begin

                        convert_count <=
                            convert_count + 1'b1;

                    end

                end


                //================================================
                // 准备转换Y
                //================================================

                ST_PREP_Y: begin

                    binary_shift <=
                        y_mag_latched;

                    bcd_shift <= 40'd0;

                    convert_count <= 6'd0;

                    state <= ST_CONV_Y;

                end


                //================================================
                // Y Binary -> BCD
                //================================================

                ST_CONV_Y: begin

                    binary_shift <= {
                        binary_shift[30:0],
                        1'b0
                    };

                    bcd_shift <=
                        bcd_shift_next;


                    if(convert_count == 6'd31) begin

                        y_bcd <=
                            bcd_shift_next;

                        convert_count <= 6'd0;

                        state <= ST_PREP_YAW;

                    end

                    else begin

                        convert_count <=
                            convert_count + 1'b1;

                    end

                end


                //================================================
                // 准备转换YAW
                //================================================

                ST_PREP_YAW: begin

                    binary_shift <=
                        yaw_mag_latched;

                    bcd_shift <= 40'd0;

                    convert_count <= 6'd0;

                    state <= ST_CONV_YAW;

                end


                //================================================
                // YAW Binary -> BCD
                //================================================

                ST_CONV_YAW: begin

                    binary_shift <= {
                        binary_shift[30:0],
                        1'b0
                    };

                    bcd_shift <=
                        bcd_shift_next;


                    if(convert_count == 6'd31) begin

                        yaw_bcd <=
                            bcd_shift_next;

                        convert_count <= 6'd0;

                        tx_index <= 6'd0;

                        state <= ST_SEND;

                    end

                    else begin

                        convert_count <=
                            convert_count + 1'b1;

                    end

                end


                //================================================
                // 发送41个字符
                //================================================

                ST_SEND: begin

                    if(
                        !uart_busy
                        &&
                        !tx_start
                    ) begin

                        tx_data <=
                            msg_char;

                        tx_start <=
                            1'b1;


                        //========================================
                        // 最后一个字符
                        //========================================

                        if(tx_index == 6'd40) begin

                            state <=
                                ST_WAIT_LAST;

                        end

                        else begin

                            tx_index <=
                                tx_index + 1'b1;

                        end

                    end

                end


                //================================================
                // 等最后一个字节彻底发完
                //================================================

                ST_WAIT_LAST: begin

                    if(
                        !uart_busy
                        &&
                        !tx_start
                    ) begin

                        state <= ST_IDLE;

                        report_counter <= 32'd0;

                    end

                end


                default: begin

                    state <= ST_IDLE;

                    report_counter <= 32'd0;

                end

            endcase

        end

    end


    //================================================
    // UART发送器
    //
    // 115200 / 8N1
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            uart_tx <= 1'b1;

            uart_busy <= 1'b0;

            baud_counter <= 16'd0;

            bit_index <= 4'd0;

            tx_shift <= 8'd0;

        end

        else begin


            //================================================
            // 空闲
            //================================================

            if(!uart_busy) begin

                uart_tx <= 1'b1;

                baud_counter <= 16'd0;

                bit_index <= 4'd0;


                //============================================
                // 开始发送一个字节
                //============================================

                if(tx_start) begin

                    tx_shift <=
                        tx_data;

                    uart_busy <=
                        1'b1;

                    // 起始位
                    uart_tx <=
                        1'b0;

                end

            end


            //================================================
            // 正在发送
            //================================================

            else begin

                if(
                    baud_counter
                    >=
                    CLKS_PER_BIT - 1
                ) begin

                    baud_counter <= 16'd0;


                    //========================================
                    // 数据位 D0 ~ D7
                    //========================================

                    if(bit_index <= 4'd7) begin

                        uart_tx <=
                            tx_shift[bit_index];

                        bit_index <=
                            bit_index + 1'b1;

                    end


                    //========================================
                    // 停止位
                    //========================================

                    else if(bit_index == 4'd8) begin

                        uart_tx <= 1'b1;

                        bit_index <= 4'd9;

                    end


                    //========================================
                    // 一个字节结束
                    //========================================

                    else begin

                        uart_tx <= 1'b1;

                        uart_busy <= 1'b0;

                        bit_index <= 4'd0;

                    end

                end

                else begin

                    baud_counter <=
                        baud_counter + 1'b1;

                end

            end

        end

    end


endmodule