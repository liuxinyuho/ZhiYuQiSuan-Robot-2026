//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_addr_probe.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   I2C地址探测模块。
//   对指定7位I2C地址发送一次Address + Write，检测从机是否ACK。
//
// Test Sequence:
//
//      START
//        ↓
//      Address + W
//        ↓
//      ACK?
//        ↓
//      STOP
//
// Output:
//
//   ack_ok = 1 : 该地址存在I2C设备
//   ack_ok = 0 : 该地址没有设备回应
//
// Clock :
//   50MHz
//
// I2C Speed :
//   约100kHz
//
// Note :
//   SDA/SCL采用开漏方式。
//   输出0：FPGA主动拉低
//   输出1：FPGA释放，由上拉电阻拉高
//////////////////////////////////////////////////////////////////////////////////

module i2c_addr_probe(
    input wire clk,
    input wire reset_n,

    // 启动一次地址检测
    input wire start,

    // 要检测的7位I2C地址
    input wire [6:0] dev_addr,

    // I2C总线
    inout wire imu_sda,
    inout wire imu_scl,

    // 状态输出
    output reg busy,
    output reg done,
    output reg ack_ok
);


    //================================================
    // I2C开漏控制
    //================================================

    reg sda_drive_low;
    reg scl_drive_low;

    assign imu_sda =
        sda_drive_low ? 1'b0 : 1'bz;

    assign imu_scl =
        scl_drive_low ? 1'b0 : 1'bz;


    wire sda_in;

    assign sda_in = imu_sda;


    //================================================
    // 100kHz左右的I2C时序
    //
    // 50MHz：
    // 125 CLK = 2.5us
    //
    // 4个阶段约10us
    // => 约100kHz
    //================================================

    reg [6:0] div_cnt;
    reg tick;


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
    // start请求锁存
    //
    // 防止start只有一个50MHz CLK，
    // 而恰好没有遇到I2C tick。
    //================================================

    reg start_pending;


    //================================================
    // 状态定义
    //================================================

    localparam ST_IDLE       = 4'd0;

    localparam ST_START_1    = 4'd1;
    localparam ST_START_2    = 4'd2;
    localparam ST_START_3    = 4'd3;

    localparam ST_BIT_SETUP  = 4'd4;
    localparam ST_BIT_RAISE  = 4'd5;
    localparam ST_BIT_HIGH   = 4'd6;
    localparam ST_BIT_FALL   = 4'd7;

    localparam ST_ACK_SETUP  = 4'd8;
    localparam ST_ACK_RAISE  = 4'd9;
    localparam ST_ACK_SAMPLE = 4'd10;
    localparam ST_ACK_FALL   = 4'd11;

    localparam ST_STOP_1     = 4'd12;
    localparam ST_STOP_2     = 4'd13;
    localparam ST_STOP_3     = 4'd14;

    localparam ST_DONE       = 4'd15;


    reg [3:0] state;


    // 要发送的地址字节：
    //
    // [7:1] = 7位设备地址
    // [0]   = 0，Write
    reg [7:0] tx_byte;

    reg [2:0] bit_index;


    //================================================
    // I2C状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            sda_drive_low <= 1'b0;
            scl_drive_low <= 1'b0;

            busy   <= 1'b0;
            done   <= 1'b0;
            ack_ok <= 1'b0;

            tx_byte   <= 8'd0;
            bit_index <= 3'd7;

            start_pending <= 1'b0;

        end

        else begin

            // done默认只持续一个系统时钟
            done <= 1'b0;


            //================================================
            // 捕获start
            //================================================

            if(start && !busy)
                start_pending <= 1'b1;


            //================================================
            // I2C状态机
            //================================================

            if(tick) begin

                case(state)


                    //========================================
                    // IDLE
                    //========================================

                    ST_IDLE: begin

                        // 释放总线
                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        busy <= 1'b0;

                        if(start_pending || start) begin

                            start_pending <= 1'b0;

                            busy   <= 1'b1;
                            ack_ok <= 1'b0;

                            // 7位地址 + Write(0)
                            tx_byte <= {
                                dev_addr,
                                1'b0
                            };

                            bit_index <= 3'd7;

                            state <= ST_START_1;

                        end

                    end


                    //========================================
                    // START
                    //
                    // SCL高电平时：
                    // SDA 1 → 0
                    //========================================

                    ST_START_1: begin

                        // SDA/SCL都释放
                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_2;

                    end


                    ST_START_2: begin

                        // SCL高
                        // SDA拉低
                        sda_drive_low <= 1'b1;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_3;

                    end


                    ST_START_3: begin

                        // 拉低SCL
                        scl_drive_low <= 1'b1;

                        state <= ST_BIT_SETUP;

                    end


                    //========================================
                    // 发送8位地址数据
                    //========================================

                    ST_BIT_SETUP: begin

                        // SCL低的时候改变SDA
                        scl_drive_low <= 1'b1;

                        if(tx_byte[bit_index] == 1'b0)
                            sda_drive_low <= 1'b1;
                        else
                            sda_drive_low <= 1'b0;

                        state <= ST_BIT_RAISE;

                    end


                    ST_BIT_RAISE: begin

                        // SCL释放为高
                        scl_drive_low <= 1'b0;

                        state <= ST_BIT_HIGH;

                    end


                    ST_BIT_HIGH: begin

                        // 保持高电平
                        state <= ST_BIT_FALL;

                    end


                    ST_BIT_FALL: begin

                        // 拉低SCL
                        scl_drive_low <= 1'b1;

                        if(bit_index == 3'd0) begin

                            state <= ST_ACK_SETUP;

                        end

                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_BIT_SETUP;

                        end

                    end


                    //========================================
                    // ACK
                    //========================================

                    ST_ACK_SETUP: begin

                        // SCL低
                        scl_drive_low <= 1'b1;

                        // 主机释放SDA
                        sda_drive_low <= 1'b0;

                        state <= ST_ACK_RAISE;

                    end


                    ST_ACK_RAISE: begin

                        // SCL高
                        scl_drive_low <= 1'b0;

                        state <= ST_ACK_SAMPLE;

                    end


                    ST_ACK_SAMPLE: begin

                        // ACK = SDA被从机拉低
                        if(sda_in == 1'b0)
                            ack_ok <= 1'b1;
                        else
                            ack_ok <= 1'b0;

                        state <= ST_ACK_FALL;

                    end


                    ST_ACK_FALL: begin

                        scl_drive_low <= 1'b1;

                        state <= ST_STOP_1;

                    end


                    //========================================
                    // STOP
                    //========================================

                    ST_STOP_1: begin

                        // SDA先保持低
                        sda_drive_low <= 1'b1;
                        scl_drive_low <= 1'b1;

                        state <= ST_STOP_2;

                    end


                    ST_STOP_2: begin

                        // 释放SCL
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_3;

                    end


                    ST_STOP_3: begin

                        // SCL高时释放SDA
                        // SDA 0 → 1 = STOP
                        sda_drive_low <= 1'b0;

                        state <= ST_DONE;

                    end


                    //========================================
                    // 完成
                    //========================================

                    ST_DONE: begin

                        busy <= 1'b0;

                        done <= 1'b1;

                        state <= ST_IDLE;

                    end


                    default: begin

                        state <= ST_IDLE;

                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                    end

                endcase

            end

        end

    end


endmodule