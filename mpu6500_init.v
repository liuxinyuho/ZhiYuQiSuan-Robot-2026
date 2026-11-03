//////////////////////////////////////////////////////////////////////////////////
// File Name   : mpu6500_init.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500初始化控制模块。
//
// Function :
//   上电后依次配置MPU6500寄存器。
//
// Init Sequence :
//
//   1. PWR_MGMT_1    0x6B = 0x80
//      软件复位
//
//   2. PWR_MGMT_1    0x6B = 0x01
//      唤醒设备，并选择PLL时钟
//
//   3. PWR_MGMT_2    0x6C = 0x00
//      使能全部Accel/Gyro轴
//
//   4. CONFIG        0x1A = 0x03
//      配置陀螺仪数字低通滤波
//
//   5. SMPLRT_DIV    0x19 = 0x09
//      配置采样分频
//
//   6. GYRO_CONFIG   0x1B = 0x00
//      Gyro量程 ±250 dps
//
//   7. ACCEL_CONFIG  0x1C = 0x00
//      Accel量程 ±2g
//
//   8. ACCEL_CONFIG2 0x1D = 0x03
//      Accel数字低通滤波
//
// Output :
//
//   init_done  = 1 : 初始化完成
//   init_error = 1 : I2C初始化过程出现NACK
//////////////////////////////////////////////////////////////////////////////////

module mpu6500_init(
    input wire clk,
    input wire reset_n,

    inout wire imu_sda,
    inout wire imu_scl,

    output reg       init_done,
    output reg       init_error,

    // 用于调试：
    // 当前正在配置第几步
    output reg [3:0] init_step
);


    //================================================
    // I2C write接口
    //================================================

    reg write_start;

    reg [7:0] reg_addr;
    reg [7:0] reg_data;


    wire write_busy;
    wire write_done;
    wire write_ack_error;


    i2c_reg_write u_i2c_reg_write(

        .clk        (clk),
        .reset_n    (reset_n),

        .start      (write_start),

        .dev_addr   (7'h68),

        .reg_addr   (reg_addr),

        .write_data (reg_data),

        .imu_sda    (imu_sda),
        .imu_scl    (imu_scl),

        .busy       (write_busy),
        .done       (write_done),

        .ack_error  (write_ack_error)

    );


    //================================================
    // 根据当前step产生对应寄存器配置
    //================================================

    always @(*) begin

        reg_addr = 8'h00;
        reg_data = 8'h00;


        case(init_step)


            // 软件复位
            4'd0: begin

                reg_addr = 8'h6B;
                reg_data = 8'h80;

            end


            // 唤醒 + PLL
            4'd1: begin

                reg_addr = 8'h6B;
                reg_data = 8'h01;

            end


            // 所有传感器轴打开
            4'd2: begin

                reg_addr = 8'h6C;
                reg_data = 8'h00;

            end


            // Gyro DLPF
            4'd3: begin

                reg_addr = 8'h1A;
                reg_data = 8'h03;

            end


            // Sample Rate Divider
            4'd4: begin

                reg_addr = 8'h19;
                reg_data = 8'h09;

            end


            // Gyro ±250dps
            4'd5: begin

                reg_addr = 8'h1B;
                reg_data = 8'h00;

            end


            // Accel ±2g
            4'd6: begin

                reg_addr = 8'h1C;
                reg_data = 8'h00;

            end


            // Accel DLPF
            4'd7: begin

                reg_addr = 8'h1D;
                reg_data = 8'h03;

            end


            default: begin

                reg_addr = 8'h00;
                reg_data = 8'h00;

            end

        endcase

    end


    //================================================
    // 初始化控制状态
    //================================================

    localparam ST_POWER_WAIT = 3'd0;
    localparam ST_START      = 3'd1;
    localparam ST_WAIT_WRITE = 3'd2;
    localparam ST_RESET_WAIT = 3'd3;
    localparam ST_GAP_WAIT   = 3'd4;
    localparam ST_DONE       = 3'd5;
    localparam ST_ERROR      = 3'd6;


    reg [2:0] state;


    //================================================
    // 延时
    //================================================

    reg [22:0] delay_cnt;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_POWER_WAIT;

            delay_cnt <= 23'd0;

            init_step <= 4'd0;

            write_start <= 1'b0;

            init_done  <= 1'b0;
            init_error <= 1'b0;

        end

        else begin

            // write_start默认一个CLK
            write_start <= 1'b0;


            case(state)


                //====================================
                // 上电等待约100ms
                //
                // 50MHz × 100ms = 5,000,000
                //====================================

                ST_POWER_WAIT: begin

                    init_done  <= 1'b0;
                    init_error <= 1'b0;

                    if(delay_cnt >= 23'd4999999) begin

                        delay_cnt <= 23'd0;

                        init_step <= 4'd0;

                        state <= ST_START;

                    end

                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                //====================================
                // 开始写一个寄存器
                //====================================

                ST_START: begin

                    if(!write_busy) begin

                        write_start <= 1'b1;

                        state <= ST_WAIT_WRITE;

                    end

                end


                //====================================
                // 等待I2C写完成
                //====================================

                ST_WAIT_WRITE: begin

                    if(write_done) begin


                        // 写失败
                        if(write_ack_error) begin

                            init_error <= 1'b1;

                            state <= ST_ERROR;

                        end


                        // 第0步是芯片reset
                        // reset后必须等一段时间
                        else if(init_step == 4'd0) begin

                            delay_cnt <= 23'd0;

                            init_step <= 4'd1;

                            state <= ST_RESET_WAIT;

                        end


                        // 最后一条初始化完成
                        else if(init_step >= 4'd7) begin

                            init_done <= 1'b1;

                            state <= ST_DONE;

                        end


                        // 继续下一寄存器
                        else begin

                            init_step <= init_step + 1'b1;

                            delay_cnt <= 23'd0;

                            state <= ST_GAP_WAIT;

                        end

                    end

                end


                //====================================
                // 软件Reset后等待100ms
                //====================================

                ST_RESET_WAIT: begin

                    if(delay_cnt >= 23'd4999999) begin

                        delay_cnt <= 23'd0;

                        state <= ST_START;

                    end

                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                //====================================
                // 普通寄存器之间等待约1ms
                //
                // 50MHz × 1ms = 50000
                //====================================

                ST_GAP_WAIT: begin

                    if(delay_cnt >= 23'd49999) begin

                        delay_cnt <= 23'd0;

                        state <= ST_START;

                    end

                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                //====================================
                // 初始化完成
                //====================================

                ST_DONE: begin

                    init_done <= 1'b1;

                    state <= ST_DONE;

                end


                //====================================
                // 初始化失败
                //====================================

                ST_ERROR: begin

                    init_error <= 1'b1;

                    state <= ST_ERROR;

                end


                default: begin

                    state <= ST_POWER_WAIT;

                end

            endcase

        end

    end

endmodule