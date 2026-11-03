`timescale 1ns / 1ps

//============================================================
// ACG720-GW5AT60K
//
// 3 x VL53L0X 独立I2C实时测距
//
// TOF1 = LEFT
// TOF2 = RIGHT
// TOF3 = CENTER / FRONT
//
// UART:
// 115200 8N1
//
// 输出:
// L=0123 C=0456 R=0789 E=000
//
// E=000:
// bit位置依次表示 LEFT / CENTER / RIGHT 是否异常
// 0 = 正常
// 1 = 尚未得到有效距离或传感器错误
//============================================================

module tof3_distance_test_top
(
    input wire clk,
    input wire reset_n,

    //========================================================
    // TOF1 LEFT
    //========================================================

    inout wire tof1_sda,
    inout wire tof1_scl,

    //========================================================
    // TOF2 RIGHT
    //========================================================

    inout wire tof2_sda,
    inout wire tof2_scl,

    //========================================================
    // TOF3 CENTER / FRONT
    //========================================================

    inout wire tof3_sda,
    inout wire tof3_scl,

    //========================================================
    // UART
    //========================================================

    output wire uart_tx
);


//============================================================
// 三路距离
//============================================================

wire [15:0] tof_left_raw_mm;
wire [15:0] tof_right_raw_mm;
wire [15:0] tof_center_raw_mm;

wire [15:0] tof_left_cal_mm;
wire [15:0] tof_right_cal_mm;
wire [15:0] tof_center_cal_mm;


//============================================================
// Valid
//============================================================

wire tof_left_valid;
wire tof_right_valid;
wire tof_center_valid;


//============================================================
// Error
//============================================================

wire tof_left_error;
wire tof_right_error;
wire tof_center_error;


//============================================================
// 每个core内部原UART不用
//============================================================

wire dummy_uart_left;
wire dummy_uart_right;
wire dummy_uart_center;


//============================================================
// TOF1 = LEFT
//============================================================

vl53l0x_sensor_core
u_tof_left
(
    .clk                    (clk),
    .reset_n                (reset_n),

    .imu_sda                (tof1_sda),
    .imu_scl                (tof1_scl),

    .uart_tx                (dummy_uart_left),

    .distance_raw_mm_out    (tof_left_raw_mm),
    .distance_cal_mm_out    (tof_left_cal_mm),

    .distance_valid_out     (tof_left_valid),

    .sensor_error           (tof_left_error)
);


//============================================================
// TOF2 = RIGHT
//============================================================

vl53l0x_sensor_core
u_tof_right
(
    .clk                    (clk),
    .reset_n                (reset_n),

    .imu_sda                (tof2_sda),
    .imu_scl                (tof2_scl),

    .uart_tx                (dummy_uart_right),

    .distance_raw_mm_out    (tof_right_raw_mm),
    .distance_cal_mm_out    (tof_right_cal_mm),

    .distance_valid_out     (tof_right_valid),

    .sensor_error           (tof_right_error)
);


//============================================================
// TOF3 = CENTER / FRONT
//============================================================

vl53l0x_sensor_core
u_tof_center
(
    .clk                    (clk),
    .reset_n                (reset_n),

    .imu_sda                (tof3_sda),
    .imu_scl                (tof3_scl),

    .uart_tx                (dummy_uart_center),

    .distance_raw_mm_out    (tof_center_raw_mm),
    .distance_cal_mm_out    (tof_center_cal_mm),

    .distance_valid_out     (tof_center_valid),

    .sensor_error           (tof_center_error)
);


//============================================================
// 是否已经至少收到一次有效距离
//============================================================

reg left_seen;
reg right_seen;
reg center_seen;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        left_seen   <= 1'b0;
        right_seen  <= 1'b0;
        center_seen <= 1'b0;

    end

    else
    begin

        if(tof_left_valid)
            left_seen <= 1'b1;

        if(tof_right_valid)
            right_seen <= 1'b1;

        if(tof_center_valid)
            center_seen <= 1'b1;

    end

end


//============================================================
// UART显示的错误
//
// 顺序：
// E = L C R
//============================================================

wire report_left_error;
wire report_center_error;
wire report_right_error;


assign report_left_error =
       tof_left_error
       ||
       !left_seen;


assign report_center_error =
       tof_center_error
       ||
       !center_seen;


assign report_right_error =
       tof_right_error
       ||
       !right_seen;


//============================================================
// 200ms产生一次UART报告
//
// 50MHz * 0.2s = 10,000,000
//============================================================

reg [23:0] report_timer;

reg report_start;

wire report_busy;
wire report_done;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        report_timer <= 24'd0;

        report_start <= 1'b0;

    end

    else
    begin

        report_start <= 1'b0;


        if(report_timer >= 24'd9_999_999)
        begin

            report_timer <= 24'd0;


            if(!report_busy)
            begin

                report_start <= 1'b1;

            end

        end

        else
        begin

            report_timer <=
                report_timer + 1'b1;

        end

    end

end


//============================================================
// UART报告
//
// 注意：
// 这里第一阶段使用 RAW 原始距离。
// 不要先给三个TOF共用同一组标定参数。
//============================================================

tof3_distance_uart_report
u_tof3_distance_uart_report
(
    .clk        (clk),

    .reset_n    (reset_n),

    .start      (report_start),


    // TOF1
    .left_mm
    (
        tof_left_raw_mm
    ),


    // TOF3
    .center_mm
    (
        tof_center_raw_mm
    ),


    // TOF2
    .right_mm
    (
        tof_right_raw_mm
    ),


    .err_left
    (
        report_left_error
    ),

    .err_center
    (
        report_center_error
    ),

    .err_right
    (
        report_right_error
    ),


    .busy
    (
        report_busy
    ),

    .done
    (
        report_done
    ),

    .uart_tx
    (
        uart_tx
    )
);


endmodule



//============================================================
// 三路距离UART报告
//
// L=0123 C=0456 R=0789 E=000
//============================================================

module tof3_distance_uart_report
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire [15:0] left_mm,
    input wire [15:0] center_mm,
    input wire [15:0] right_mm,

    input wire err_left,
    input wire err_center,
    input wire err_right,

    output reg busy,
    output reg done,

    output wire uart_tx
);


//============================================================
// 锁存数据
//============================================================

reg [15:0] left_reg;
reg [15:0] center_reg;
reg [15:0] right_reg;

reg err_left_reg;
reg err_center_reg;
reg err_right_reg;


//============================================================
// UART TX
//============================================================

reg [7:0] tx_data;
reg tx_start;

wire tx_busy;
wire tx_done;


tof3_distance_uart_tx
u_uart_tx
(
    .clk
    (
        clk
    ),

    .reset_n
    (
        reset_n
    ),

    .data
    (
        tx_data
    ),

    .start
    (
        tx_start
    ),

    .tx
    (
        uart_tx
    ),

    .busy
    (
        tx_busy
    ),

    .done
    (
        tx_done
    )
);


//============================================================
// 十进制数字
//============================================================

wire [3:0] l_thousand;
wire [3:0] l_hundred;
wire [3:0] l_ten;
wire [3:0] l_one;

wire [3:0] c_thousand;
wire [3:0] c_hundred;
wire [3:0] c_ten;
wire [3:0] c_one;

wire [3:0] r_thousand;
wire [3:0] r_hundred;
wire [3:0] r_ten;
wire [3:0] r_one;


assign l_thousand =
       (left_reg / 1000) % 10;

assign l_hundred =
       (left_reg / 100) % 10;

assign l_ten =
       (left_reg / 10) % 10;

assign l_one =
       left_reg % 10;


assign c_thousand =
       (center_reg / 1000) % 10;

assign c_hundred =
       (center_reg / 100) % 10;

assign c_ten =
       (center_reg / 10) % 10;

assign c_one =
       center_reg % 10;


assign r_thousand =
       (right_reg / 1000) % 10;

assign r_hundred =
       (right_reg / 100) % 10;

assign r_ten =
       (right_reg / 10) % 10;

assign r_one =
       right_reg % 10;


//============================================================
// 数字转ASCII
//============================================================

function [7:0] digit_ascii;

    input [3:0] digit;

    begin

        digit_ascii =
            8'h30 + digit;

    end

endfunction


//============================================================
// Reporter FSM
//============================================================

reg [5:0] index;

reg phase;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        busy <= 1'b0;

        done <= 1'b0;

        tx_data <= 8'h00;

        tx_start <= 1'b0;

        index <= 6'd0;

        phase <= 1'b0;

        left_reg <= 16'd0;
        center_reg <= 16'd0;
        right_reg <= 16'd0;

        err_left_reg <= 1'b1;
        err_center_reg <= 1'b1;
        err_right_reg <= 1'b1;

    end

    else
    begin

        done <= 1'b0;

        tx_start <= 1'b0;


        //----------------------------------------------------
        // 启动一帧
        //----------------------------------------------------

        if(!busy)
        begin

            if(start)
            begin

                left_reg <= left_mm;

                center_reg <= center_mm;

                right_reg <= right_mm;


                err_left_reg <=
                    err_left;

                err_center_reg <=
                    err_center;

                err_right_reg <=
                    err_right;


                index <= 6'd0;

                phase <= 1'b0;

                busy <= 1'b1;

            end

        end


        //----------------------------------------------------
        // 正在发送
        //----------------------------------------------------

        else
        begin

            //------------------------------------------------
            // 准备一个字符
            //------------------------------------------------

            if(!phase)
            begin

                if(!tx_busy)
                begin

                    case(index)

                    //----------------------------------------
                    // L=0123
                    //----------------------------------------

                    6'd0:
                        tx_data <= "L";

                    6'd1:
                        tx_data <= "=";

                    6'd2:
                        tx_data <=
                            digit_ascii(l_thousand);

                    6'd3:
                        tx_data <=
                            digit_ascii(l_hundred);

                    6'd4:
                        tx_data <=
                            digit_ascii(l_ten);

                    6'd5:
                        tx_data <=
                            digit_ascii(l_one);


                    //----------------------------------------
                    // space
                    //----------------------------------------

                    6'd6:
                        tx_data <= " ";


                    //----------------------------------------
                    // C=0123
                    //----------------------------------------

                    6'd7:
                        tx_data <= "C";

                    6'd8:
                        tx_data <= "=";

                    6'd9:
                        tx_data <=
                            digit_ascii(c_thousand);

                    6'd10:
                        tx_data <=
                            digit_ascii(c_hundred);

                    6'd11:
                        tx_data <=
                            digit_ascii(c_ten);

                    6'd12:
                        tx_data <=
                            digit_ascii(c_one);


                    //----------------------------------------
                    // space
                    //----------------------------------------

                    6'd13:
                        tx_data <= " ";


                    //----------------------------------------
                    // R=0123
                    //----------------------------------------

                    6'd14:
                        tx_data <= "R";

                    6'd15:
                        tx_data <= "=";

                    6'd16:
                        tx_data <=
                            digit_ascii(r_thousand);

                    6'd17:
                        tx_data <=
                            digit_ascii(r_hundred);

                    6'd18:
                        tx_data <=
                            digit_ascii(r_ten);

                    6'd19:
                        tx_data <=
                            digit_ascii(r_one);


                    //----------------------------------------
                    // space
                    //----------------------------------------

                    6'd20:
                        tx_data <= " ";


                    //----------------------------------------
                    // E=000
                    //----------------------------------------

                    6'd21:
                        tx_data <= "E";

                    6'd22:
                        tx_data <= "=";

                    6'd23:
                        tx_data <=
                            err_left_reg
                            ?
                            "1"
                            :
                            "0";

                    6'd24:
                        tx_data <=
                            err_center_reg
                            ?
                            "1"
                            :
                            "0";

                    6'd25:
                        tx_data <=
                            err_right_reg
                            ?
                            "1"
                            :
                            "0";


                    //----------------------------------------
                    // CR LF
                    //----------------------------------------

                    6'd26:
                        tx_data <= 8'h0D;

                    6'd27:
                        tx_data <= 8'h0A;


                    default:
                        tx_data <= 8'h20;

                    endcase


                    tx_start <= 1'b1;

                    phase <= 1'b1;

                end

            end


            //------------------------------------------------
            // 等字符发送完成
            //------------------------------------------------

            else
            begin

                if(tx_done)
                begin

                    if(index == 6'd27)
                    begin

                        busy <= 1'b0;

                        done <= 1'b1;

                        index <= 6'd0;

                        phase <= 1'b0;

                    end

                    else
                    begin

                        index <=
                            index + 1'b1;

                        phase <= 1'b0;

                    end

                end

            end

        end

    end

end


endmodule



//============================================================
// UART TX
//
// CLK = 50MHz
// UART = 115200 8N1
//============================================================

module tof3_distance_uart_tx
(
    input wire clk,
    input wire reset_n,

    input wire [7:0] data,

    input wire start,

    output reg tx,

    output reg busy,

    output reg done
);


//============================================================
// 50MHz / 115200 ≈ 434
//============================================================

localparam integer CLKS_PER_BIT = 434;


reg [8:0] baud_cnt;

reg [3:0] bit_index;

reg [9:0] tx_shift;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        tx <= 1'b1;

        busy <= 1'b0;

        done <= 1'b0;

        baud_cnt <= 9'd0;

        bit_index <= 4'd0;

        tx_shift <= 10'h3FF;

    end

    else
    begin

        done <= 1'b0;


        //----------------------------------------------------
        // IDLE
        //----------------------------------------------------

        if(!busy)
        begin

            tx <= 1'b1;

            baud_cnt <= 9'd0;

            bit_index <= 4'd0;


            if(start)
            begin

                // stop + data + start
                tx_shift <=
                {
                    1'b1,
                    data,
                    1'b0
                };

                tx <= 1'b0;

                busy <= 1'b1;

            end

        end


        //----------------------------------------------------
        // SEND
        //----------------------------------------------------

        else
        begin

            if
            (
                baud_cnt
                >=
                CLKS_PER_BIT - 1
            )
            begin

                baud_cnt <= 9'd0;


                if(bit_index == 4'd9)
                begin

                    tx <= 1'b1;

                    busy <= 1'b0;

                    done <= 1'b1;

                    bit_index <= 4'd0;

                end

                else
                begin

                    bit_index <=
                        bit_index + 1'b1;


                    tx_shift <=
                    {
                        1'b1,
                        tx_shift[9:1]
                    };


                    tx <=
                        tx_shift[1];

                end

            end

            else
            begin

                baud_cnt <=
                    baud_cnt + 1'b1;

            end

        end

    end

end


endmodule