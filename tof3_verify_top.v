`timescale 1ns / 1ps

//============================================================
// ACG720-GW5AT60K
// 3 x VL53L0X 独立 I2C 总线硬件验证
//
// CLK      = 50MHz
// UART TX  = 115200 8N1
//
// TOF1:
//   SDA = C20
//   SCL = D20
//
// TOF2:
//   SDA = D19
//   SCL = E19
//
// TOF3:
//   SDA = E18
//   SCL = F18
//
// VL53L0X:
//   7-bit address = 0x29
//   WRITE         = 0x52
//   READ          = 0x53
//   ID register   = 0xC0
//   expected ID   = 0xEE
//
// 串口正常示例:
// T1=EE T2=EE T3=EE E=000
//
// E=000:
//   第1位 TOF1 I2C错误
//   第2位 TOF2 I2C错误
//   第3位 TOF3 I2C错误
//   0=正常，1=ACK错误
//============================================================

module tof3_verify_top
(
    input  wire clk,
    input  wire reset_n,

    inout  wire tof1_sda,
    inout  wire tof1_scl,

    inout  wire tof2_sda,
    inout  wire tof2_scl,

    inout  wire tof3_sda,
    inout  wire tof3_scl,

    output wire uart_tx
);


//============================================================
// 三路 I2C
//============================================================

reg tof_start;


//---------------- TOF1 ----------------
wire [7:0] tof1_id;
wire tof1_busy;
wire tof1_done;
wire tof1_error;

tof3_verify_i2c
u_tof1_i2c
(
    .clk       (clk),
    .reset_n   (reset_n),

    .start     (tof_start),

    .reg_addr  (8'hC0),

    .read_data (tof1_id),

    .busy      (tof1_busy),
    .done      (tof1_done),
    .ack_error (tof1_error),

    .i2c_sda   (tof1_sda),
    .i2c_scl   (tof1_scl)
);


//---------------- TOF2 ----------------
wire [7:0] tof2_id;
wire tof2_busy;
wire tof2_done;
wire tof2_error;

tof3_verify_i2c
u_tof2_i2c
(
    .clk       (clk),
    .reset_n   (reset_n),

    .start     (tof_start),

    .reg_addr  (8'hC0),

    .read_data (tof2_id),

    .busy      (tof2_busy),
    .done      (tof2_done),
    .ack_error (tof2_error),

    .i2c_sda   (tof2_sda),
    .i2c_scl   (tof2_scl)
);


//---------------- TOF3 ----------------
wire [7:0] tof3_id;
wire tof3_busy;
wire tof3_done;
wire tof3_error;

tof3_verify_i2c
u_tof3_i2c
(
    .clk       (clk),
    .reset_n   (reset_n),

    .start     (tof_start),

    .reg_addr  (8'hC0),

    .read_data (tof3_id),

    .busy      (tof3_busy),
    .done      (tof3_done),
    .ack_error (tof3_error),

    .i2c_sda   (tof3_sda),
    .i2c_scl   (tof3_scl)
);


//============================================================
// 三路完成标志
//============================================================

reg tof1_done_flag;
reg tof2_done_flag;
reg tof3_done_flag;

reg [7:0] tof1_id_latched;
reg [7:0] tof2_id_latched;
reg [7:0] tof3_id_latched;

reg tof1_error_latched;
reg tof2_error_latched;
reg tof3_error_latched;


//============================================================
// UART Reporter
//============================================================

reg report_start;

wire report_busy;
wire report_done;

tof3_verify_uart_report
u_uart_report
(
    .clk       (clk),
    .reset_n   (reset_n),

    .start     (report_start),

    .id1       (tof1_id_latched),
    .id2       (tof2_id_latched),
    .id3       (tof3_id_latched),

    .err1      (tof1_error_latched),
    .err2      (tof2_error_latched),
    .err3      (tof3_error_latched),

    .busy      (report_busy),
    .done      (report_done),

    .uart_tx   (uart_tx)
);


//============================================================
// 主状态机
//============================================================

localparam
    MAIN_BOOT       = 3'd0,
    MAIN_START      = 3'd1,
    MAIN_WAIT       = 3'd2,
    MAIN_REPORT     = 3'd3,
    MAIN_WAIT_UART  = 3'd4,
    MAIN_DELAY      = 3'd5;

reg [2:0] main_state;

reg [25:0] delay_cnt;


//============================================================
// 主控制
//============================================================

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        main_state <= MAIN_BOOT;

        delay_cnt <= 26'd0;

        tof_start <= 1'b0;
        report_start <= 1'b0;

        tof1_done_flag <= 1'b0;
        tof2_done_flag <= 1'b0;
        tof3_done_flag <= 1'b0;

        tof1_id_latched <= 8'h00;
        tof2_id_latched <= 8'h00;
        tof3_id_latched <= 8'h00;

        tof1_error_latched <= 1'b0;
        tof2_error_latched <= 1'b0;
        tof3_error_latched <= 1'b0;

    end

    else
    begin

        // start信号只维持一个50MHz周期
        tof_start    <= 1'b0;
        report_start <= 1'b0;


        //----------------------------------------------------
        // 捕获每一路完成结果
        //----------------------------------------------------

        if(tof1_done)
        begin
            tof1_done_flag <= 1'b1;
            tof1_id_latched <= tof1_id;
            tof1_error_latched <= tof1_error;
        end

        if(tof2_done)
        begin
            tof2_done_flag <= 1'b1;
            tof2_id_latched <= tof2_id;
            tof2_error_latched <= tof2_error;
        end

        if(tof3_done)
        begin
            tof3_done_flag <= 1'b1;
            tof3_id_latched <= tof3_id;
            tof3_error_latched <= tof3_error;
        end


        case(main_state)

        //----------------------------------------------------
        // 上电等待100ms
        //----------------------------------------------------
        MAIN_BOOT:
        begin

            if(delay_cnt >= 26'd4_999_999)
            begin
                delay_cnt <= 26'd0;
                main_state <= MAIN_START;
            end
            else
            begin
                delay_cnt <= delay_cnt + 1'b1;
            end

        end


        //----------------------------------------------------
        // 同时启动三路读取
        //----------------------------------------------------
        MAIN_START:
        begin

            tof1_done_flag <= 1'b0;
            tof2_done_flag <= 1'b0;
            tof3_done_flag <= 1'b0;

            tof_start <= 1'b1;

            main_state <= MAIN_WAIT;

        end


        //----------------------------------------------------
        // 等待三路完成
        //----------------------------------------------------
        MAIN_WAIT:
        begin

            if
            (
                (tof1_done_flag || tof1_done)
                &&
                (tof2_done_flag || tof2_done)
                &&
                (tof3_done_flag || tof3_done)
            )
            begin

                main_state <= MAIN_REPORT;

            end

        end


        //----------------------------------------------------
        // 启动UART报告
        //----------------------------------------------------
        MAIN_REPORT:
        begin

            if(!report_busy)
            begin

                report_start <= 1'b1;

                main_state <= MAIN_WAIT_UART;

            end

        end


        //----------------------------------------------------
        // 等待UART发送完成
        //----------------------------------------------------
        MAIN_WAIT_UART:
        begin

            if(report_done)
            begin

                delay_cnt <= 26'd0;

                main_state <= MAIN_DELAY;

            end

        end


        //----------------------------------------------------
        // 等待约1秒，再检测一次
        //----------------------------------------------------
        MAIN_DELAY:
        begin

            if(delay_cnt >= 26'd49_999_999)
            begin

                delay_cnt <= 26'd0;

                main_state <= MAIN_START;

            end

            else
            begin

                delay_cnt <= delay_cnt + 1'b1;

            end

        end


        default:
        begin
            main_state <= MAIN_BOOT;
        end

        endcase

    end

end

endmodule



//==========================================================================
// VL53L0X 单寄存器读取 I2C
//
// 固定设备:
// WRITE = 0x52
// READ  = 0x53
//
// I2C约100kHz
//==========================================================================
module tof3_verify_i2c
(
    input  wire clk,
    input  wire reset_n,

    input  wire start,

    input  wire [7:0] reg_addr,

    output reg  [7:0] read_data,

    output reg  busy,
    output reg  done,
    output reg  ack_error,

    inout  wire i2c_sda,
    inout  wire i2c_scl
);


//============================================================
// Open Drain
//============================================================

reg sda_low;
reg scl_low;

assign i2c_sda = sda_low ? 1'b0 : 1'bz;
assign i2c_scl = scl_low ? 1'b0 : 1'bz;

wire sda_in;

assign sda_in = i2c_sda;


//============================================================
// 2.5us Tick
//
// 50MHz:
// 125 clocks = 2.5us
//
// 4相位 / bit
// => 10us/bit
// => 100kHz
//============================================================

reg [7:0] tick_cnt;

wire tick;

assign tick = (tick_cnt == 8'd124);

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin
        tick_cnt <= 8'd0;
    end

    else
    begin

        if(tick)
            tick_cnt <= 8'd0;
        else
            tick_cnt <= tick_cnt + 1'b1;

    end

end


//============================================================
// State
//============================================================

localparam
    ST_IDLE      = 4'd0,
    ST_START     = 4'd1,
    ST_SEND_W    = 4'd2,
    ST_ACK_W     = 4'd3,
    ST_SEND_REG  = 4'd4,
    ST_ACK_REG   = 4'd5,
    ST_RSTART    = 4'd6,
    ST_SEND_R    = 4'd7,
    ST_ACK_R     = 4'd8,
    ST_READ      = 4'd9,
    ST_NACK      = 4'd10,
    ST_STOP      = 4'd11;

reg [3:0] state;

reg [1:0] phase;

reg [2:0] bit_index;

reg [7:0] tx_byte;


//============================================================
// I2C FSM
//============================================================

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        state <= ST_IDLE;
        phase <= 2'd0;

        bit_index <= 3'd7;

        tx_byte <= 8'h00;

        read_data <= 8'h00;

        busy <= 1'b0;
        done <= 1'b0;
        ack_error <= 1'b0;

        sda_low <= 1'b0;
        scl_low <= 1'b0;

    end

    else
    begin

        done <= 1'b0;


        //----------------------------------------------------
        // 启动
        //----------------------------------------------------
        if(state == ST_IDLE)
        begin

            busy <= 1'b0;

            sda_low <= 1'b0;
            scl_low <= 1'b0;

            if(start)
            begin

                busy <= 1'b1;

                ack_error <= 1'b0;

                read_data <= 8'h00;

                phase <= 2'd0;

                state <= ST_START;

            end

        end


        //----------------------------------------------------
        // I2C时序
        //----------------------------------------------------
        else if(tick)
        begin

            case(state)


            //------------------------------------------------
            // START
            //------------------------------------------------
            ST_START:
            begin

                case(phase)

                2'd0:
                begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    phase <= 2'd1;
                end

                2'd1:
                begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    phase <= 2'd2;
                end

                2'd2:
                begin
                    // SDA下降，SCL保持高
                    sda_low <= 1'b1;
                    scl_low <= 1'b0;
                    phase <= 2'd3;
                end

                2'd3:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b1;

                    tx_byte <= 8'h52;

                    bit_index <= 3'd7;

                    phase <= 2'd0;

                    state <= ST_SEND_W;

                end

                endcase

            end


            //------------------------------------------------
            // SEND WRITE ADDRESS = 0x52
            //------------------------------------------------
            ST_SEND_W:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;

                    sda_low <= ~tx_byte[bit_index];

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    if(bit_index == 3'd0)
                    begin

                        sda_low <= 1'b0;

                        phase <= 2'd0;

                        state <= ST_ACK_W;

                    end

                    else
                    begin

                        bit_index <= bit_index - 1'b1;

                        phase <= 2'd0;

                    end

                end

                endcase

            end


            //------------------------------------------------
            // ACK WRITE ADDRESS
            //------------------------------------------------
            ST_ACK_W:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    if(sda_in)
                        ack_error <= 1'b1;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    tx_byte <= reg_addr;

                    bit_index <= 3'd7;

                    phase <= 2'd0;

                    state <= ST_SEND_REG;

                end

                endcase

            end


            //------------------------------------------------
            // SEND REGISTER
            //------------------------------------------------
            ST_SEND_REG:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;

                    sda_low <= ~tx_byte[bit_index];

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    if(bit_index == 3'd0)
                    begin

                        sda_low <= 1'b0;

                        phase <= 2'd0;

                        state <= ST_ACK_REG;

                    end

                    else
                    begin

                        bit_index <= bit_index - 1'b1;

                        phase <= 2'd0;

                    end

                end

                endcase

            end


            //------------------------------------------------
            // ACK REGISTER
            //------------------------------------------------
            ST_ACK_REG:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    if(sda_in)
                        ack_error <= 1'b1;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    phase <= 2'd0;

                    state <= ST_RSTART;

                end

                endcase

            end


            //------------------------------------------------
            // REPEATED START
            //------------------------------------------------
            ST_RSTART:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    // repeated START
                    scl_low <= 1'b0;
                    sda_low <= 1'b1;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    tx_byte <= 8'h53;

                    bit_index <= 3'd7;

                    phase <= 2'd0;

                    state <= ST_SEND_R;

                end

                endcase

            end


            //------------------------------------------------
            // SEND READ ADDRESS = 0x53
            //------------------------------------------------
            ST_SEND_R:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;

                    sda_low <= ~tx_byte[bit_index];

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    if(bit_index == 3'd0)
                    begin

                        sda_low <= 1'b0;

                        phase <= 2'd0;

                        state <= ST_ACK_R;

                    end

                    else
                    begin

                        bit_index <= bit_index - 1'b1;

                        phase <= 2'd0;

                    end

                end

                endcase

            end


            //------------------------------------------------
            // ACK READ ADDRESS
            //------------------------------------------------
            ST_ACK_R:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    if(sda_in)
                        ack_error <= 1'b1;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    bit_index <= 3'd7;

                    phase <= 2'd0;

                    state <= ST_READ;

                end

                endcase

            end


            //------------------------------------------------
            // READ 8 BITS
            //------------------------------------------------
            ST_READ:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;

                    // 主机释放SDA
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    read_data[bit_index] <= sda_in;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    if(bit_index == 3'd0)
                    begin

                        phase <= 2'd0;

                        state <= ST_NACK;

                    end

                    else
                    begin

                        bit_index <= bit_index - 1'b1;

                        phase <= 2'd0;

                    end

                end

                endcase

            end


            //------------------------------------------------
            // MASTER NACK
            //------------------------------------------------
            ST_NACK:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;

                    // NACK = SDA释放为高
                    sda_low <= 1'b0;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    scl_low <= 1'b0;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b1;

                    phase <= 2'd0;

                    state <= ST_STOP;

                end

                endcase

            end


            //------------------------------------------------
            // STOP
            //------------------------------------------------
            ST_STOP:
            begin

                case(phase)

                2'd0:
                begin

                    scl_low <= 1'b1;
                    sda_low <= 1'b1;

                    phase <= 2'd1;

                end

                2'd1:
                begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b1;

                    phase <= 2'd2;

                end

                2'd2:
                begin

                    // SDA由低到高 while SCL high
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    phase <= 2'd3;

                end

                2'd3:
                begin

                    scl_low <= 1'b0;
                    sda_low <= 1'b0;

                    busy <= 1'b0;
                    done <= 1'b1;

                    phase <= 2'd0;

                    state <= ST_IDLE;

                end

                endcase

            end


            default:
            begin

                state <= ST_IDLE;

                busy <= 1'b0;

                sda_low <= 1'b0;
                scl_low <= 1'b0;

            end

            endcase

        end

    end

end

endmodule



//==========================================================================
// UART报告模块
//
// 输出:
// T1=EE T2=EE T3=EE E=000\r\n
//==========================================================================
module tof3_verify_uart_report
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire [7:0] id1,
    input wire [7:0] id2,
    input wire [7:0] id3,

    input wire err1,
    input wire err2,
    input wire err3,

    output reg busy,
    output reg done,

    output wire uart_tx
);


//============================================================
// 锁存数据
//============================================================

reg [7:0] id1_reg;
reg [7:0] id2_reg;
reg [7:0] id3_reg;

reg err1_reg;
reg err2_reg;
reg err3_reg;


//============================================================
// UART TX
//============================================================

reg [7:0] tx_data;
reg tx_start;

wire tx_busy;
wire tx_done;


tof3_verify_uart_tx
u_uart_tx
(
    .clk      (clk),
    .reset_n  (reset_n),

    .data     (tx_data),
    .start    (tx_start),

    .tx       (uart_tx),

    .busy     (tx_busy),
    .done     (tx_done)
);


//============================================================
// Hex -> ASCII
//============================================================

function [7:0] hex_ascii;

    input [3:0] hex;

    begin

        if(hex < 4'd10)
            hex_ascii = 8'h30 + hex;
        else
            hex_ascii = 8'h41 + (hex - 4'd10);

    end

endfunction


//============================================================
// Report FSM
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

        id1_reg <= 8'h00;
        id2_reg <= 8'h00;
        id3_reg <= 8'h00;

        err1_reg <= 1'b0;
        err2_reg <= 1'b0;
        err3_reg <= 1'b0;

    end

    else
    begin

        done <= 1'b0;
        tx_start <= 1'b0;


        //----------------------------------------------------
        // Start report
        //----------------------------------------------------
        if(!busy)
        begin

            if(start)
            begin

                id1_reg <= id1;
                id2_reg <= id2;
                id3_reg <= id3;

                err1_reg <= err1;
                err2_reg <= err2;
                err3_reg <= err3;

                index <= 6'd0;
                phase <= 1'b0;

                busy <= 1'b1;

            end

        end


        //----------------------------------------------------
        // Send report
        //----------------------------------------------------
        else
        begin

            //------------------------------------------------
            // 准备字符
            //------------------------------------------------
            if(!phase)
            begin

                if(!tx_busy)
                begin

                    case(index)

                    6'd0 : tx_data <= "T";
                    6'd1 : tx_data <= "1";
                    6'd2 : tx_data <= "=";

                    6'd3 : tx_data <= hex_ascii(id1_reg[7:4]);
                    6'd4 : tx_data <= hex_ascii(id1_reg[3:0]);

                    6'd5 : tx_data <= " ";

                    6'd6 : tx_data <= "T";
                    6'd7 : tx_data <= "2";
                    6'd8 : tx_data <= "=";

                    6'd9 : tx_data <= hex_ascii(id2_reg[7:4]);
                    6'd10: tx_data <= hex_ascii(id2_reg[3:0]);

                    6'd11: tx_data <= " ";

                    6'd12: tx_data <= "T";
                    6'd13: tx_data <= "3";
                    6'd14: tx_data <= "=";

                    6'd15: tx_data <= hex_ascii(id3_reg[7:4]);
                    6'd16: tx_data <= hex_ascii(id3_reg[3:0]);

                    6'd17: tx_data <= " ";

                    6'd18: tx_data <= "E";
                    6'd19: tx_data <= "=";

                    6'd20:
                        tx_data <= err1_reg ? "1" : "0";

                    6'd21:
                        tx_data <= err2_reg ? "1" : "0";

                    6'd22:
                        tx_data <= err3_reg ? "1" : "0";

                    6'd23:
                        tx_data <= 8'h0D;

                    6'd24:
                        tx_data <= 8'h0A;

                    default:
                        tx_data <= 8'h20;

                    endcase

                    tx_start <= 1'b1;

                    phase <= 1'b1;

                end

            end


            //------------------------------------------------
            // 等待字符发送完
            //------------------------------------------------
            else
            begin

                if(tx_done)
                begin

                    if(index == 6'd24)
                    begin

                        busy <= 1'b0;
                        done <= 1'b1;

                        index <= 6'd0;
                        phase <= 1'b0;

                    end

                    else
                    begin

                        index <= index + 1'b1;

                        phase <= 1'b0;

                    end

                end

            end

        end

    end

end

endmodule



//==========================================================================
// UART TX
//
// 50MHz
// 115200
// 8N1
//==========================================================================
module tof3_verify_uart_tx
(
    input wire clk,
    input wire reset_n,

    input wire [7:0] data,
    input wire start,

    output reg tx,

    output reg busy,
    output reg done
);


// 50MHz / 115200 ≈ 434
localparam integer CLKS_PER_BIT = 434;


reg [8:0] baud_cnt;

reg [3:0] bit_index;

reg [9:0] tx_shift;


//============================================================
// UART TX
//============================================================

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
        // Idle
        //----------------------------------------------------
        if(!busy)
        begin

            tx <= 1'b1;

            baud_cnt <= 9'd0;

            bit_index <= 4'd0;

            if(start)
            begin

                // {stop, data[7:0], start}
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
        // Sending
        //----------------------------------------------------
        else
        begin

            if(baud_cnt >= CLKS_PER_BIT - 1)
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

                    bit_index <= bit_index + 1'b1;

                    tx_shift <=
                    {
                        1'b1,
                        tx_shift[9:1]
                    };

                    tx <= tx_shift[1];

                end

            end

            else
            begin

                baud_cnt <= baud_cnt + 1'b1;

            end

        end

    end

end

endmodule