//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_reg_read.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   I2C单寄存器读取模块。
//   用于从指定I2C从设备中读取一个8位寄存器的数据。
//
//   当前系统时钟：50MHz
//   I2C速度：约100kHz
//
// I2C Read Sequence :
//
//   START
//     ↓
//   Slave Address + W
//     ↓
//   ACK
//     ↓
//   Register Address
//     ↓
//   ACK
//     ↓
//   Repeated START
//     ↓
//   Slave Address + R
//     ↓
//   ACK
//     ↓
//   Read Data
//     ↓
//   NACK
//     ↓
//   STOP
//
// Note :
//   I2C使用开漏方式：
//   输出0 -> FPGA主动拉低
//   输出1 -> FPGA释放总线，由外部上拉电阻拉高
//////////////////////////////////////////////////////////////////////////////////

module i2c_reg_read #(
    // 7位I2C从机地址
    parameter [6:0] DEV_ADDR = 7'h68,

    // 要读取的寄存器地址
    parameter [7:0] REG_ADDR = 8'h75
)(
    input  wire clk,
    input  wire reset_n,

    // 启动一次读取
    // 可以只保持1个50MHz时钟周期
    input  wire start,

    // I2C总线
    inout  wire imu_scl,
    inout  wire imu_sda,

    // 读取到的数据
    output reg  [7:0] read_data,

    // 当前正在通信
    output reg        busy,

    // 一次读取完成脉冲
    output reg        done,

    // ACK错误
    // 0 = 正常
    // 1 = 从机没有应答
    output reg        ack_error
);


    //================================================
    // I2C 开漏控制
    //================================================

    reg scl_drive_low;
    reg sda_drive_low;

    // drive_low = 1 -> 输出0
    // drive_low = 0 -> 高阻态Z，让上拉电阻拉到3.3V

    assign imu_scl =
        scl_drive_low ? 1'b0 : 1'bz;

    assign imu_sda =
        sda_drive_low ? 1'b0 : 1'bz;


    // SDA输入值
    wire sda_in;

    assign sda_in = imu_sda;


    //================================================
    // I2C时序分频
    //
    // FPGA时钟 = 50MHz
    //
    // 50MHz周期 = 20ns
    //
    // 每125个CLK产生一个tick：
    //
    // 125 × 20ns = 2.5us
    //
    // 一个I2C bit使用4个tick：
    //
    // 4 × 2.5us = 10us
    //
    // I2C ≈ 100kHz
    //================================================

    reg [6:0] div_cnt;
    reg       tick;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            div_cnt <= 7'd0;
            tick    <= 1'b0;

        end
        else begin

            tick <= 1'b0;

            if(div_cnt >= 7'd124) begin

                div_cnt <= 7'd0;
                tick    <= 1'b1;

            end
            else begin

                div_cnt <= div_cnt + 1'b1;

            end

        end

    end


    //================================================
    // 防止start只有一个系统时钟周期而被I2C tick错过
    //================================================

    reg start_pending;


    //================================================
    // 状态定义
    //================================================

    localparam ST_IDLE        = 5'd0;

    localparam ST_START_1     = 5'd1;
    localparam ST_START_2     = 5'd2;
    localparam ST_START_3     = 5'd3;

    localparam ST_SEND_SETUP  = 5'd4;
    localparam ST_SEND_RAISE  = 5'd5;
    localparam ST_SEND_HIGH   = 5'd6;
    localparam ST_SEND_FALL   = 5'd7;

    localparam ST_ACK_SETUP   = 5'd8;
    localparam ST_ACK_RAISE   = 5'd9;
    localparam ST_ACK_SAMPLE  = 5'd10;
    localparam ST_ACK_FALL    = 5'd11;

    localparam ST_REP_1       = 5'd12;
    localparam ST_REP_2       = 5'd13;
    localparam ST_REP_3       = 5'd14;
    localparam ST_REP_4       = 5'd15;

    localparam ST_READ_SETUP  = 5'd16;
    localparam ST_READ_RAISE  = 5'd17;
    localparam ST_READ_SAMPLE = 5'd18;
    localparam ST_READ_FALL   = 5'd19;

    localparam ST_NACK_SETUP  = 5'd20;
    localparam ST_NACK_RAISE  = 5'd21;
    localparam ST_NACK_HIGH   = 5'd22;
    localparam ST_NACK_FALL   = 5'd23;

    localparam ST_STOP_1      = 5'd24;
    localparam ST_STOP_2      = 5'd25;
    localparam ST_STOP_3      = 5'd26;

    localparam ST_DONE        = 5'd27;


    reg [4:0] state;


    //================================================
    // 发送相关变量
    //================================================

    reg [7:0] tx_byte;

    reg [2:0] bit_index;


    // phase含义：
    //
    // 0 = Device Address + Write
    // 1 = Register Address
    // 2 = Device Address + Read

    reg [1:0] phase;


    // 保存本次ACK结果
    reg ack_fail;


    //================================================
    // I2C主状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            scl_drive_low <= 1'b0;
            sda_drive_low <= 1'b0;

            tx_byte   <= 8'd0;
            bit_index <= 3'd7;
            phase     <= 2'd0;

            read_data <= 8'd0;

            busy      <= 1'b0;
            done      <= 1'b0;

            ack_error <= 1'b0;
            ack_fail  <= 1'b0;

            start_pending <= 1'b0;

        end

        else begin

            // done只保持一个50MHz时钟
            done <= 1'b0;


            //========================================
            // 捕获start请求
            //========================================

            if(start && (state == ST_IDLE))
                start_pending <= 1'b1;


            //========================================
            // I2C状态机只在tick时运行
            //========================================

            if(tick) begin

                case(state)


                    //================================
                    // 空闲状态
                    //================================

                    ST_IDLE: begin

                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;

                        busy <= 1'b0;


                        if(start_pending || start) begin

                            start_pending <= 1'b0;

                            busy <= 1'b1;

                            ack_error <= 1'b0;
                            ack_fail  <= 1'b0;

                            read_data <= 8'd0;

                            state <= ST_START_1;

                        end

                    end


                    //================================
                    // START
                    //
                    // SCL = 1 时
                    // SDA从1变为0
                    //================================

                    ST_START_1: begin

                        // 两根线都释放
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;

                        state <= ST_START_2;

                    end


                    ST_START_2: begin

                        // SCL保持高
                        // SDA拉低 -> START
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;

                        state <= ST_START_3;

                    end


                    ST_START_3: begin

                        // SCL拉低
                        scl_drive_low <= 1'b1;

                        // MPU地址 + Write
                        tx_byte <= {
                            DEV_ADDR,
                            1'b0
                        };

                        bit_index <= 3'd7;

                        phase <= 2'd0;

                        state <= ST_SEND_SETUP;

                    end


                    //================================
                    // 发送一个bit
                    //================================

                    ST_SEND_SETUP: begin

                        // SCL低电平时修改SDA
                        scl_drive_low <= 1'b1;


                        // bit=1 -> 释放SDA
                        // bit=0 -> 拉低SDA

                        if(tx_byte[bit_index])
                            sda_drive_low <= 1'b0;
                        else
                            sda_drive_low <= 1'b1;


                        state <= ST_SEND_RAISE;

                    end


                    ST_SEND_RAISE: begin

                        // SCL释放 -> 被上拉为高
                        scl_drive_low <= 1'b0;

                        state <= ST_SEND_HIGH;

                    end


                    ST_SEND_HIGH: begin

                        // 保持高电平一段时间
                        state <= ST_SEND_FALL;

                    end


                    ST_SEND_FALL: begin

                        // 拉低SCL
                        scl_drive_low <= 1'b1;


                        if(bit_index == 3'd0) begin

                            // 一个Byte已经发完
                            state <= ST_ACK_SETUP;

                        end
                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_SEND_SETUP;

                        end

                    end


                    //================================
                    // 等待从机ACK
                    //================================

                    ST_ACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // 主机释放SDA
                        sda_drive_low <= 1'b0;

                        ack_fail <= 1'b0;

                        state <= ST_ACK_RAISE;

                    end


                    ST_ACK_RAISE: begin

                        // SCL高
                        scl_drive_low <= 1'b0;

                        state <= ST_ACK_SAMPLE;

                    end


                    ST_ACK_SAMPLE: begin

                        // ACK应该是0
                        if(sda_in == 1'b1) begin

                            ack_fail  <= 1'b1;
                            ack_error <= 1'b1;

                        end
                        else begin

                            ack_fail <= 1'b0;

                        end


                        state <= ST_ACK_FALL;

                    end


                    ST_ACK_FALL: begin

                        scl_drive_low <= 1'b1;


                        // ACK失败，直接STOP
                        if(ack_fail) begin

                            state <= ST_STOP_1;

                        end

                        else begin

                            case(phase)


                                //====================
                                // 已发送：
                                // Device Address + W
                                //
                                // 下一步发送寄存器地址
                                //====================

                                2'd0: begin

                                    tx_byte <= REG_ADDR;

                                    bit_index <= 3'd7;

                                    phase <= 2'd1;

                                    state <= ST_SEND_SETUP;

                                end


                                //====================
                                // 已发送寄存器地址
                                //
                                // 下一步Repeated START
                                //====================

                                2'd1: begin

                                    state <= ST_REP_1;

                                end


                                //====================
                                // 已发送：
                                // Device Address + R
                                //
                                // 下一步读数据
                                //====================

                                2'd2: begin

                                    bit_index <= 3'd7;

                                    state <= ST_READ_SETUP;

                                end


                                default: begin

                                    state <= ST_STOP_1;

                                end

                            endcase

                        end

                    end


                    //================================
                    // Repeated START
                    //================================

                    ST_REP_1: begin

                        // SCL保持低
                        // SDA释放
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b0;

                        state <= ST_REP_2;

                    end


                    ST_REP_2: begin

                        // SCL释放为高
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;

                        state <= ST_REP_3;

                    end


                    ST_REP_3: begin

                        // SCL高时SDA拉低
                        // 形成Repeated START
                        sda_drive_low <= 1'b1;

                        state <= ST_REP_4;

                    end


                    ST_REP_4: begin

                        scl_drive_low <= 1'b1;


                        // MPU地址 + Read
                        tx_byte <= {
                            DEV_ADDR,
                            1'b1
                        };

                        bit_index <= 3'd7;

                        phase <= 2'd2;

                        state <= ST_SEND_SETUP;

                    end


                    //================================
                    // 读取1Byte数据
                    //================================

                    ST_READ_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // 释放SDA，由MPU输出
                        sda_drive_low <= 1'b0;

                        state <= ST_READ_RAISE;

                    end


                    ST_READ_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_READ_SAMPLE;

                    end


                    ST_READ_SAMPLE: begin

                        // 在SCL高电平期间采样
                        read_data[bit_index] <= sda_in;

                        state <= ST_READ_FALL;

                    end


                    ST_READ_FALL: begin

                        scl_drive_low <= 1'b1;


                        if(bit_index == 3'd0) begin

                            state <= ST_NACK_SETUP;

                        end

                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_READ_SETUP;

                        end

                    end


                    //================================
                    // 读取最后一个Byte后发送NACK
                    //
                    // NACK = SDA保持高
                    //================================

                    ST_NACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA释放 = 高
                        sda_drive_low <= 1'b0;

                        state <= ST_NACK_RAISE;

                    end


                    ST_NACK_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_NACK_HIGH;

                    end


                    ST_NACK_HIGH: begin

                        state <= ST_NACK_FALL;

                    end


                    ST_NACK_FALL: begin

                        scl_drive_low <= 1'b1;

                        state <= ST_STOP_1;

                    end


                    //================================
                    // STOP
                    //
                    // SCL高时
                    // SDA从0变成1
                    //================================

                    ST_STOP_1: begin

                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_2;

                    end


                    ST_STOP_2: begin

                        // SDA保持低
                        // 释放SCL
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_3;

                    end


                    ST_STOP_3: begin

                        // SCL高时释放SDA
                        sda_drive_low <= 1'b0;

                        state <= ST_DONE;

                    end


                    //================================
                    // 完成
                    //================================

                    ST_DONE: begin

                        busy <= 1'b0;

                        done <= 1'b1;

                        state <= ST_IDLE;

                    end


                    default: begin

                        state <= ST_IDLE;

                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;

                    end

                endcase

            end

        end

    end


endmodule