//////////////////////////////////////////////////////////////////////////////////
// File Name   : mpu6500_data_reader.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500六轴原始数据读取模块。
//
//   初始化完成以后依次读取：
//
//   0x3B~0x40 : Accel X/Y/Z
//   0x41~0x42 : Temperature
//   0x43~0x48 : Gyro X/Y/Z
//
// Output :
//   accel_x/y/z : 16bit有符号原始加速度
//   temp_raw    : 16bit原始温度
//   gyro_x/y/z  : 16bit有符号原始角速度
//
//   data_valid  : 一整组14Byte读取完成脉冲
//////////////////////////////////////////////////////////////////////////////////

module mpu6500_data_reader(
    input wire clk,
    input wire reset_n,

    input wire init_done,

    inout wire imu_sda,
    inout wire imu_scl,

    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z,

    output reg signed [15:0] temp_raw,

    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,

    output reg data_valid,
    output reg read_error
);


    //================================================
    // I2C动态读取接口
    //================================================

    reg read_start;

    reg [7:0] current_reg;


    wire [7:0] read_data;

    wire read_busy;
    wire read_done;
    wire read_ack_error;


    i2c_reg_read_dyn u_i2c_reg_read_dyn(

        .clk       (clk),
        .reset_n   (reset_n),

        .start     (read_start),

        .dev_addr  (7'h68),

        .reg_addr  (current_reg),

        .imu_sda   (imu_sda),
        .imu_scl   (imu_scl),

        .read_data (read_data),

        .busy      (read_busy),
        .done      (read_done),

        .ack_error (read_ack_error)

    );


    //================================================
    // 14Byte缓存
    //================================================

    reg [7:0] data_buf [0:13];

    reg [3:0] byte_index;


    // 当前地址：
    //
    // 0 → 0x3B
    // 1 → 0x3C
    // ...
    // 13 → 0x48

    always @(*) begin

        current_reg =
            8'h3B + byte_index;

    end


    //================================================
    // 状态定义
    //================================================

    localparam ST_WAIT_INIT = 3'd0;
    localparam ST_START     = 3'd1;
    localparam ST_WAIT      = 3'd2;
    localparam ST_ASSEMBLE  = 3'd3;
    localparam ST_DELAY     = 3'd4;


    reg [2:0] state;


    //================================================
    // 两次完整数据之间等待10ms
    //================================================

    reg [18:0] delay_cnt;


    integer i;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_WAIT_INIT;

            read_start <= 1'b0;

            byte_index <= 4'd0;

            delay_cnt <= 19'd0;

            accel_x <= 16'sd0;
            accel_y <= 16'sd0;
            accel_z <= 16'sd0;

            temp_raw <= 16'sd0;

            gyro_x <= 16'sd0;
            gyro_y <= 16'sd0;
            gyro_z <= 16'sd0;

            data_valid <= 1'b0;
            read_error <= 1'b0;


            for(i = 0; i < 14; i = i + 1)
                data_buf[i] <= 8'd0;

        end

        else begin

            read_start <= 1'b0;

            data_valid <= 1'b0;


            case(state)


                //====================================
                // 等待初始化完成
                //====================================

                ST_WAIT_INIT: begin

                    if(init_done) begin

                        byte_index <= 4'd0;

                        read_error <= 1'b0;

                        state <= ST_START;

                    end

                end


                //====================================
                // 开始读当前寄存器
                //====================================

                ST_START: begin

                    if(!read_busy) begin

                        read_start <= 1'b1;

                        state <= ST_WAIT;

                    end

                end


                //====================================
                // 等待读取完成
                //====================================

                ST_WAIT: begin

                    if(read_done) begin


                        if(read_ack_error) begin

                            read_error <= 1'b1;

                            delay_cnt <= 19'd0;

                            state <= ST_DELAY;

                        end


                        else begin

                            data_buf[byte_index] <= read_data;


                            if(byte_index >= 4'd13) begin

                                state <= ST_ASSEMBLE;

                            end

                            else begin

                                byte_index <= byte_index + 1'b1;

                                state <= ST_START;

                            end

                        end

                    end

                end


                //====================================
                // 将14Byte组合成16bit数据
                //====================================

                ST_ASSEMBLE: begin

                    accel_x <=
                        $signed({data_buf[0], data_buf[1]});

                    accel_y <=
                        $signed({data_buf[2], data_buf[3]});

                    accel_z <=
                        $signed({data_buf[4], data_buf[5]});


                    temp_raw <=
                        $signed({data_buf[6], data_buf[7]});


                    gyro_x <=
                        $signed({data_buf[8], data_buf[9]});

                    gyro_y <=
                        $signed({data_buf[10], data_buf[11]});

                    gyro_z <=
                        $signed({data_buf[12],read_data});


                    data_valid <= 1'b1;

                    delay_cnt <= 19'd0;

                    state <= ST_DELAY;

                end


                //====================================
                // 等待10ms
                //====================================

                ST_DELAY: begin

                    if(delay_cnt >= 19'd499999) begin

                        delay_cnt <= 19'd0;

                        byte_index <= 4'd0;

                        state <= ST_START;

                    end

                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                default: begin

                    state <= ST_WAIT_INIT;

                end

            endcase

        end

    end

endmodule