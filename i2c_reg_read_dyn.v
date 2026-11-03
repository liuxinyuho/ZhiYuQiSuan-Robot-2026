//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_reg_read_dyn.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   动态I2C单寄存器读取模块。
//   与之前固定地址的i2c_reg_read不同，
//   本模块可以在每次start时指定设备地址和寄存器地址。
//
// Clock       : 50MHz
// I2C Speed   : 约100kHz
//
// Read Sequence:
//
//   START
//     ↓
//   Device Address + W
//     ↓
//   ACK
//     ↓
//   Register Address
//     ↓
//   ACK
//     ↓
//   Repeated START
//     ↓
//   Device Address + R
//     ↓
//   ACK
//     ↓
//   Read 1 Byte
//     ↓
//   NACK
//     ↓
//   STOP
//////////////////////////////////////////////////////////////////////////////////

module i2c_reg_read_dyn(
    input  wire       clk,
    input  wire       reset_n,

    input  wire       start,

    input  wire [6:0] dev_addr,
    input  wire [7:0] reg_addr,

    inout  wire       imu_sda,
    inout  wire       imu_scl,

    output reg  [7:0] read_data,

    output reg        busy,
    output reg        done,
    output reg        ack_error
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
    // 100kHz左右I2C节拍
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
    // 请求参数锁存
    //================================================

    reg [6:0] dev_addr_latch;
    reg [7:0] reg_addr_latch;

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

    reg [7:0] tx_byte;
    reg [2:0] bit_index;

    // 0 = Address + W
    // 1 = Register
    // 2 = Address + R
    reg [1:0] phase;

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

            tx_byte   <= 8'd0;
            bit_index <= 3'd7;

            phase <= 2'd0;

            ack_fail <= 1'b0;

            start_pending <= 1'b0;

            dev_addr_latch <= 7'd0;
            reg_addr_latch <= 8'd0;

            read_data <= 8'd0;

        end

        else begin

            done <= 1'b0;


            //========================================
            // 捕获读取请求
            //========================================

            if(start && !busy && !start_pending) begin

                dev_addr_latch <= dev_addr;
                reg_addr_latch <= reg_addr;

                start_pending <= 1'b1;

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

                            busy      <= 1'b1;
                            ack_error <= 1'b0;
                            ack_fail  <= 1'b0;

                            read_data <= 8'd0;

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

                        // SCL高时SDA拉低
                        sda_drive_low <= 1'b1;
                        scl_drive_low <= 1'b0;

                        state <= ST_START_3;

                    end


                    ST_START_3: begin

                        scl_drive_low <= 1'b1;

                        state <= ST_SEND_SETUP;

                    end


                    //================================
                    // 发送Byte
                    //================================

                    ST_SEND_SETUP: begin

                        scl_drive_low <= 1'b1;


                        if(tx_byte[bit_index] == 1'b0)

                            sda_drive_low <= 1'b1;

                        else

                            sda_drive_low <= 1'b0;


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
                    // ACK
                    //================================

                    ST_ACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // 主机释放SDA
                        sda_drive_low <= 1'b0;

                        ack_fail <= 1'b0;

                        state <= ST_ACK_RAISE;

                    end


                    ST_ACK_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_ACK_SAMPLE;

                    end


                    ST_ACK_SAMPLE: begin

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


                        if(ack_fail) begin

                            state <= ST_STOP_1;

                        end

                        else begin

                            case(phase)


                                // Address + W 已发送
                                2'd0: begin

                                    tx_byte <= reg_addr_latch;

                                    bit_index <= 3'd7;

                                    phase <= 2'd1;

                                    state <= ST_SEND_SETUP;

                                end


                                // Register已发送
                                2'd1: begin

                                    state <= ST_REP_1;

                                end


                                // Address + R 已发送
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

                        scl_drive_low <= 1'b1;

                        // SDA释放
                        sda_drive_low <= 1'b0;

                        state <= ST_REP_2;

                    end


                    ST_REP_2: begin

                        // SCL释放到高
                        scl_drive_low <= 1'b0;

                        state <= ST_REP_3;

                    end


                    ST_REP_3: begin

                        // SCL高时SDA拉低
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
                    // 读取1Byte
                    //================================

                    ST_READ_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA交给从机
                        sda_drive_low <= 1'b0;

                        state <= ST_READ_RAISE;

                    end


                    ST_READ_RAISE: begin

                        scl_drive_low <= 1'b0;

                        state <= ST_READ_SAMPLE;

                    end


                    ST_READ_SAMPLE: begin

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
                    // NACK
                    //================================

                    ST_NACK_SETUP: begin

                        scl_drive_low <= 1'b1;

                        // SDA释放，高电平表示NACK
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

                        // SCL高时SDA释放
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

                        sda_drive_low <= 1'b0;
                        scl_drive_low <= 1'b0;

                    end

                endcase

            end

        end

    end

endmodule