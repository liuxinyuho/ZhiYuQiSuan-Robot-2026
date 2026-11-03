//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_reg_write.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   I2C单寄存器写入模块。
//   用于向指定I2C从设备的指定8位寄存器写入1Byte数据。
//
// Clock :
//   50MHz
//
// I2C Speed :
//   约100kHz
//
// Write Sequence :
//
//      START
//        ↓
//      Slave Address + W
//        ↓
//      ACK
//        ↓
//      Register Address
//        ↓
//      ACK
//        ↓
//      Write Data
//        ↓
//      ACK
//        ↓
//      STOP
//
// Note :
//   SDA、SCL采用开漏输出。
//   drive_low = 1 -> 主动拉低
//   drive_low = 0 -> 高阻态，由上拉拉高
//////////////////////////////////////////////////////////////////////////////////

module i2c_reg_write(
    input  wire clk,
    input  wire reset_n,

    //================================================
    // 写操作控制
    //================================================
    input  wire       start,

    input  wire [6:0] dev_addr,
    input  wire [7:0] reg_addr,
    input  wire [7:0] write_data,

    //================================================
    // I2C
    //================================================
    inout  wire imu_sda,
    inout  wire imu_scl,

    //================================================
    // 状态
    //================================================
    output reg busy,
    output reg done,

    // 0 = 正常
    // 1 = 某一步没有收到ACK
    output reg ack_error
);


    //================================================
    // 开漏控制
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
    // 产生约100kHz I2C节拍
    //
    // 50MHz时钟周期 = 20ns
    //
    // 125 × 20ns = 2.5us
    //
    // 一个I2C bit使用约4个阶段
    // 约10us，即约100kHz
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
    // 锁存一次写操作参数
    //================================================

    reg [6:0] dev_addr_latch;
    reg [7:0] reg_addr_latch;
    reg [7:0] data_latch;

    reg start_pending;


    //================================================
    // 状态定义
    //================================================

    localparam ST_IDLE        = 4'd0;

    localparam ST_START_1     = 4'd1;
    localparam ST_START_2     = 4'd2;
    localparam ST_START_3     = 4'd3;

    localparam ST_SEND_SETUP  = 4'd4;
    localparam ST_SEND_RAISE  = 4'd5;
    localparam ST_SEND_HIGH   = 4'd6;
    localparam ST_SEND_FALL   = 4'd7;

    localparam ST_ACK_SETUP   = 4'd8;
    localparam ST_ACK_RAISE   = 4'd9;
    localparam ST_ACK_SAMPLE  = 4'd10;
    localparam ST_ACK_FALL    = 4'd11;

    localparam ST_STOP_1      = 4'd12;
    localparam ST_STOP_2      = 4'd13;
    localparam ST_STOP_3      = 4'd14;

    localparam ST_DONE        = 4'd15;


    reg [3:0] state;


    //================================================
    // phase
    //
    // 0 : Device Address + W
    // 1 : Register Address
    // 2 : Write Data
    //================================================

    reg [1:0] phase;

    reg [7:0] tx_byte;
    reg [2:0] bit_index;

    reg ack_fail;


    //================================================
    // 主状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            sda_drive_low <= 1'b0;
            scl_drive_low <= 1'b0;

            busy      <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;

            phase     <= 2'd0;
            tx_byte   <= 8'd0;
            bit_index <= 3'd7;

            ack_fail <= 1'b0;

            start_pending <= 1'b0;

            dev_addr_latch <= 7'd0;
            reg_addr_latch <= 8'd0;
            data_latch     <= 8'd0;

        end

        else begin

            // done仅保持一个系统时钟
            done <= 1'b0;


            //================================================
            // 捕获start
            //================================================

            if(start && !busy) begin

                start_pending <= 1'b1;

                dev_addr_latch <= dev_addr;
                reg_addr_latch <= reg_addr;
                data_latch     <= write_data;

            end


            //================================================
            // I2C状态机
            //================================================

            if(tick) begin

                case(state)


                    //========================================
                    // IDLE
                    //========================================

                    ST_IDLE: begin

                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        busy <= 1'b0;


                        if(start_pending || start) begin

                            start_pending <= 1'b0;

                            busy      <= 1'b1;
                            ack_error <= 1'b0;
                            ack_fail  <= 1'b0;

                            // 第一Byte：
                            // Device Address + Write
                            tx_byte <= {
                                dev_addr_latch,
                                1'b0
                            };

                            bit_index <= 3'd7;

                            phase <= 2'd0;

                            state <= ST_START_1;

                        end

                    end


                    //========================================
                    // START
                    //========================================

                    ST_START_1: begin

                        // SDA、SCL释放
                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_2;

                    end


                    ST_START_2: begin

                        // SCL高时 SDA从1变0
                        // 产生START
                        sda_drive_low <= 1'b1;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_3;

                    end


                    ST_START_3: begin

                        // SCL拉低
                        scl_drive_low <= 1'b1;

                        state <= ST_SEND_SETUP;

                    end


                    //========================================
                    // SEND BYTE
                    //========================================

                    ST_SEND_SETUP: begin

                        // SCL低时改变SDA
                        scl_drive_low <= 1'b1;


                        if(tx_byte[bit_index] == 1'b0)

                            sda_drive_low <= 1'b1;

                        else

                            sda_drive_low <= 1'b0;


                        state <= ST_SEND_RAISE;

                    end


                    ST_SEND_RAISE: begin

                        // SCL释放为高
                        scl_drive_low <= 1'b0;

                        state <= ST_SEND_HIGH;

                    end


                    ST_SEND_HIGH: begin

                        // 保持一段高电平
                        state <= ST_SEND_FALL;

                    end


                    ST_SEND_FALL: begin

                        // SCL拉低
                        scl_drive_low <= 1'b1;


                        if(bit_index == 3'd0) begin

                            state <= ST_ACK_SETUP;

                        end

                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_SEND_SETUP;

                        end

                    end


                    //========================================
                    // ACK
                    //========================================

                    ST_ACK_SETUP: begin

                        // SCL保持低
                        scl_drive_low <= 1'b1;

                        // 主机释放SDA
                        sda_drive_low <= 1'b0;

                        ack_fail <= 1'b0;

                        state <= ST_ACK_RAISE;

                    end


                    ST_ACK_RAISE: begin

                        // SCL变高
                        scl_drive_low <= 1'b0;

                        state <= ST_ACK_SAMPLE;

                    end


                    ST_ACK_SAMPLE: begin

                        // ACK = SDA低
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


                        // 没有ACK，直接结束
                        if(ack_fail) begin

                            state <= ST_STOP_1;

                        end

                        else begin

                            case(phase)


                                //============================
                                // 地址发送完成
                                // 下一步发寄存器地址
                                //============================

                                2'd0: begin

                                    tx_byte <= reg_addr_latch;

                                    bit_index <= 3'd7;

                                    phase <= 2'd1;

                                    state <= ST_SEND_SETUP;

                                end


                                //============================
                                // 寄存器地址完成
                                // 下一步发数据
                                //============================

                                2'd1: begin

                                    tx_byte <= data_latch;

                                    bit_index <= 3'd7;

                                    phase <= 2'd2;

                                    state <= ST_SEND_SETUP;

                                end


                                //============================
                                // 数据发送完成
                                // STOP
                                //============================

                                2'd2: begin

                                    state <= ST_STOP_1;

                                end


                                default: begin

                                    state <= ST_STOP_1;

                                end

                            endcase

                        end

                    end


                    //========================================
                    // STOP
                    //========================================

                    ST_STOP_1: begin

                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_2;

                    end


                    ST_STOP_2: begin

                        // SDA保持低，SCL释放
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_3;

                    end


                    ST_STOP_3: begin

                        // SCL高时释放SDA
                        // SDA 0 -> 1
                        sda_drive_low <= 1'b0;

                        state <= ST_DONE;

                    end


                    //========================================
                    // DONE
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