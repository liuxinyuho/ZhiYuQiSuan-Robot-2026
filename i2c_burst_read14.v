//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_burst_read14.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   I2C连续14Byte读取模块。
//   用于MPU6500从指定起始寄存器连续读取14Byte数据。
//
// Clock       : 50MHz
// I2C Speed   : 约100kHz
//
// Sequence:
//
//   START
//   Device Address + W
//   ACK
//   Start Register
//   ACK
//   Repeated START
//   Device Address + R
//   ACK
//
//   Read Byte0  -> ACK
//   Read Byte1  -> ACK
//   ...
//   Read Byte12 -> ACK
//   Read Byte13 -> NACK
//
//   STOP
//
// data_out:
//
//   [111:104] Byte0
//   [103:96]  Byte1
//   ...
//   [7:0]     Byte13
//
//////////////////////////////////////////////////////////////////////////////////

module i2c_burst_read14(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire [6:0] dev_addr,
    input wire [7:0] start_reg,

    inout wire imu_sda,
    inout wire imu_scl,

    output reg [111:0] data_out,

    output reg busy,
    output reg done,
    output reg ack_error
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
    // I2C约100kHz节拍
    //
    // 50MHz:
    // 125个CLK = 2.5us
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
    // 请求锁存
    //================================================

    reg start_pending;

    reg [6:0] dev_addr_latch;
    reg [7:0] start_reg_latch;


    //================================================
    // 状态
    //================================================

    localparam ST_IDLE         = 6'd0;

    localparam ST_START_1      = 6'd1;
    localparam ST_START_2      = 6'd2;
    localparam ST_START_3      = 6'd3;

    localparam ST_SEND_SETUP   = 6'd4;
    localparam ST_SEND_RAISE   = 6'd5;
    localparam ST_SEND_HIGH    = 6'd6;
    localparam ST_SEND_FALL    = 6'd7;

    localparam ST_ACK_SETUP    = 6'd8;
    localparam ST_ACK_RAISE    = 6'd9;
    localparam ST_ACK_SAMPLE   = 6'd10;
    localparam ST_ACK_FALL     = 6'd11;

    localparam ST_REP_1        = 6'd12;
    localparam ST_REP_2        = 6'd13;
    localparam ST_REP_3        = 6'd14;
    localparam ST_REP_4        = 6'd15;

    localparam ST_READ_SETUP   = 6'd16;
    localparam ST_READ_RAISE   = 6'd17;
    localparam ST_READ_SAMPLE  = 6'd18;
    localparam ST_READ_FALL    = 6'd19;

    localparam ST_MACK_SETUP   = 6'd20;
    localparam ST_MACK_RAISE   = 6'd21;
    localparam ST_MACK_HIGH    = 6'd22;
    localparam ST_MACK_FALL    = 6'd23;

    localparam ST_NACK_SETUP   = 6'd24;
    localparam ST_NACK_RAISE   = 6'd25;
    localparam ST_NACK_HIGH    = 6'd26;
    localparam ST_NACK_FALL    = 6'd27;

    localparam ST_STOP_1       = 6'd28;
    localparam ST_STOP_2       = 6'd29;
    localparam ST_STOP_3       = 6'd30;

    localparam ST_DONE         = 6'd31;


    reg [5:0] state;


    //================================================
    // 发送
    //================================================

    reg [7:0] tx_byte;
    reg [2:0] bit_index;

    // 0 : Address + W
    // 1 : Register Address
    // 2 : Address + R

    reg [1:0] phase;

    reg ack_fail;


    //================================================
    // 接收
    //================================================

    reg [7:0] rx_byte;

    reg [3:0] byte_index;


    //================================================
    // 状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            sda_drive_low <= 1'b0;
            scl_drive_low <= 1'b0;

            busy      <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;

            start_pending <= 1'b0;

            dev_addr_latch <= 7'd0;
            start_reg_latch <= 8'd0;

            tx_byte <= 8'd0;
            bit_index <= 3'd7;

            phase <= 2'd0;

            ack_fail <= 1'b0;

            rx_byte <= 8'd0;
            byte_index <= 4'd0;

            data_out <= 112'd0;

        end

        else begin

            done <= 1'b0;


            //========================================
            // 捕获start
            //========================================

            if(start && !busy && !start_pending) begin

                start_pending <= 1'b1;

                dev_addr_latch <= dev_addr;
                start_reg_latch <= start_reg;

            end


            if(tick) begin

                case(state)


                    //================================
                    // IDLE
                    //================================

                    ST_IDLE: begin

                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        busy <= 1'b0;


                        if(start_pending) begin

                            start_pending <= 1'b0;

                            busy <= 1'b1;

                            ack_error <= 1'b0;
                            ack_fail  <= 1'b0;

                            byte_index <= 4'd0;

                            data_out <= 112'd0;


                            // Address + Write
                            tx_byte <= {
                                dev_addr_latch,
                                1'b0
                            };

                            bit_index <= 3'd7;

                            phase <= 2'd0;

                            state <= ST_START_1;

                        end

                    end


                    //================================
                    // START
                    //================================

                    ST_START_1: begin

                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_2;

                    end


                    ST_START_2: begin

                        // SCL高时，SDA 1→0
                        sda_drive_low <= 1'b1;

                        state <= ST_START_3;

                    end


                    ST_START_3: begin

                        scl_drive_low <= 1'b1;

                        state <= ST_SEND_SETUP;

                    end


                    //================================
                    // SEND
                    //================================

                    ST_SEND_SETUP: begin

                        scl_drive_low <= 1'b1;


                        if(tx_byte[bit_index])

                            sda_drive_low <= 1'b0;

                        else

                            sda_drive_low <= 1'b1;


                        state <= ST_SEND_RAISE;

                    end


                    ST_SEND_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_SEND_HIGH;

                    end


                    ST_SEND_HIGH: begin

                        state <= ST_SEND_FALL;

                    end


                    ST_SEND_FALL: begin

                        scl_drive_low <= 1'b1;


                        if(bit_index == 3'd0) begin

                            state <= ST_ACK_SETUP;

                        end

                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_SEND_SETUP;

                        end

                    end


                    //================================
                    // SLAVE ACK
                    //================================

                    ST_ACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA释放
                        sda_drive_low <= 1'b0;

                        ack_fail <= 1'b0;

                        state <= ST_ACK_RAISE;

                    end


                    ST_ACK_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_ACK_SAMPLE;

                    end


                    ST_ACK_SAMPLE: begin

                        if(sda_in) begin

                            ack_fail <= 1'b1;
                            ack_error <= 1'b1;

                        end

                        else begin

                            ack_fail <= 1'b0;

                        end

                        state <= ST_ACK_FALL;

                    end


                    ST_ACK_FALL: begin

                        scl_drive_low <= 1'b1;


                        if(ack_fail) begin

                            state <= ST_STOP_1;

                        end

                        else begin

                            case(phase)

                                // Address+W完成
                                2'd0: begin

                                    tx_byte <= start_reg_latch;

                                    bit_index <= 3'd7;

                                    phase <= 2'd1;

                                    state <= ST_SEND_SETUP;

                                end


                                // Register完成
                                2'd1: begin

                                    state <= ST_REP_1;

                                end


                                // Address+R完成
                                2'd2: begin

                                    byte_index <= 4'd0;

                                    bit_index <= 3'd7;

                                    rx_byte <= 8'd0;

                                    state <= ST_READ_SETUP;

                                end


                                default: begin

                                    state <= ST_STOP_1;

                                end

                            endcase

                        end

                    end


                    //================================
                    // REPEATED START
                    //================================

                    ST_REP_1: begin

                        scl_drive_low <= 1'b1;

                        sda_drive_low <= 1'b0;

                        state <= ST_REP_2;

                    end


                    ST_REP_2: begin

                        scl_drive_low <= 1'b0;

                        sda_drive_low <= 1'b0;

                        state <= ST_REP_3;

                    end


                    ST_REP_3: begin

                        // SCL高时SDA 1→0
                        sda_drive_low <= 1'b1;

                        state <= ST_REP_4;

                    end


                    ST_REP_4: begin

                        scl_drive_low <= 1'b1;


                        tx_byte <= {
                            dev_addr_latch,
                            1'b1
                        };

                        bit_index <= 3'd7;

                        phase <= 2'd2;

                        state <= ST_SEND_SETUP;

                    end


                    //================================
                    // READ BYTE
                    //================================

                    ST_READ_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA释放，由MPU输出
                        sda_drive_low <= 1'b0;

                        state <= ST_READ_RAISE;

                    end


                    ST_READ_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_READ_SAMPLE;

                    end


                    ST_READ_SAMPLE: begin

                        rx_byte[bit_index] <= sda_in;

                        state <= ST_READ_FALL;

                    end


                    ST_READ_FALL: begin

                        scl_drive_low <= 1'b1;


                        if(bit_index == 3'd0) begin

                            // 保存当前Byte
                            case(byte_index)

                                4'd0:
                                    data_out[111:104] <= rx_byte;

                                4'd1:
                                    data_out[103:96] <= rx_byte;

                                4'd2:
                                    data_out[95:88] <= rx_byte;

                                4'd3:
                                    data_out[87:80] <= rx_byte;

                                4'd4:
                                    data_out[79:72] <= rx_byte;

                                4'd5:
                                    data_out[71:64] <= rx_byte;

                                4'd6:
                                    data_out[63:56] <= rx_byte;

                                4'd7:
                                    data_out[55:48] <= rx_byte;

                                4'd8:
                                    data_out[47:40] <= rx_byte;

                                4'd9:
                                    data_out[39:32] <= rx_byte;

                                4'd10:
                                    data_out[31:24] <= rx_byte;

                                4'd11:
                                    data_out[23:16] <= rx_byte;

                                4'd12:
                                    data_out[15:8] <= rx_byte;

                                4'd13:
                                    data_out[7:0] <= rx_byte;

                            endcase


                            // 前13Byte发送ACK
                            if(byte_index < 4'd13)

                                state <= ST_MACK_SETUP;

                            else

                                state <= ST_NACK_SETUP;

                        end

                        else begin

                            bit_index <= bit_index - 1'b1;

                            state <= ST_READ_SETUP;

                        end

                    end


                    //================================
                    // MASTER ACK
                    //
                    // ACK = SDA拉低
                    //================================

                    ST_MACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        sda_drive_low <= 1'b1;

                        state <= ST_MACK_RAISE;

                    end


                    ST_MACK_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_MACK_HIGH;

                    end


                    ST_MACK_HIGH: begin

                        state <= ST_MACK_FALL;

                    end


                    ST_MACK_FALL: begin

                        scl_drive_low <= 1'b1;

                        // 释放SDA
                        sda_drive_low <= 1'b0;

                        byte_index <= byte_index + 1'b1;

                        bit_index <= 3'd7;

                        rx_byte <= 8'd0;

                        state <= ST_READ_SETUP;

                    end


                    //================================
                    // MASTER NACK
                    //================================

                    ST_NACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA释放为高
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
                    //================================

                    ST_STOP_1: begin

                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_2;

                    end


                    ST_STOP_2: begin

                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;

                        state <= ST_STOP_3;

                    end


                    ST_STOP_3: begin

                        sda_drive_low <= 1'b0;

                        state <= ST_DONE;

                    end


                    //================================
                    // DONE
                    //================================

                    ST_DONE: begin

                        busy <= 1'b0;

                        done <= 1'b1;

                        state <= ST_IDLE;

                    end


                    default: begin

                        state <= ST_IDLE;

                    end

                endcase

            end

        end

    end

endmodule