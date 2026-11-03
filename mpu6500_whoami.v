//////////////////////////////////////////////////////////////////////////////////
// File Name   : mpu6500_whoami.v
//
// Description :
//   MPU6500身份识别测试模块。
//   设备I2C地址：0x68（AD0=0）
//   WHO_AM_I寄存器：0x75
//   MPU6500正常返回：0x70
//////////////////////////////////////////////////////////////////////////////////
// Function :
//   1. FPGA上电后等待约100ms
//   2. 自动读取WHO_AM_I
//   3. 保存读取结果
//   4. 每隔约500ms重新读取一次
//
// Output :
//   imu_id        - WHO_AM_I读取值
//   imu_valid     - 本次读取有效
//   imu_ack_error - I2C ACK错误
//////////////////////////////////////////////////////////////////////////////////

module mpu9250_whoami(
    input  wire clk,
    input  wire reset_n,

    inout  wire imu_scl,
    inout  wire imu_sda,

    output reg  [7:0] imu_id,
    output reg        imu_valid,
    output reg        imu_ack_error
);


    //================================================
    // I2C控制
    //================================================

    reg i2c_start;

    wire [7:0] i2c_data;

    wire i2c_busy;
    wire i2c_done;
    wire i2c_ack_error;


    //================================================
    // MPU9250 WHO_AM_I读取
    //
    // Address = 0x68
    // Register = 0x75
    //================================================

    i2c_reg_read #(
        .DEV_ADDR (7'h68),
        .REG_ADDR (8'h75)
    )
    u_i2c_whoami(
        .clk       (clk),
        .reset_n   (reset_n),

        .start     (i2c_start),

        .imu_scl   (imu_scl),
        .imu_sda   (imu_sda),

        .read_data (i2c_data),

        .busy      (i2c_busy),
        .done      (i2c_done),

        .ack_error (i2c_ack_error)
    );


    //================================================
    // 状态机
    //================================================

    localparam ST_POWER_WAIT = 2'd0;
    localparam ST_START      = 2'd1;
    localparam ST_WAIT_DONE  = 2'd2;
    localparam ST_RETRY_WAIT = 2'd3;

    reg [1:0] state;


    //================================================
    // 延时计数器
    //
    // 50MHz:
    //
    // 100ms = 5,000,000 CLK
    //
    // 500ms = 25,000,000 CLK
    //================================================

    reg [24:0] delay_cnt;


    //================================================
    // 主控制状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_POWER_WAIT;

            delay_cnt <= 25'd0;

            i2c_start <= 1'b0;

            imu_id <= 8'h00;

            imu_valid <= 1'b0;

            imu_ack_error <= 1'b0;

        end

        else begin

            // 默认start只有一个clk周期
            i2c_start <= 1'b0;


            case(state)


                //====================================
                // 上电等待100ms
                //====================================

                ST_POWER_WAIT: begin

                    imu_valid <= 1'b0;

                    if(delay_cnt >= 25'd4999999) begin

                        delay_cnt <= 25'd0;

                        state <= ST_START;

                    end
                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                //====================================
                // 启动一次I2C读取
                //====================================

                ST_START: begin

                    if(!i2c_busy) begin

                        i2c_start <= 1'b1;

                        state <= ST_WAIT_DONE;

                    end

                end


                //====================================
                // 等待读取完成
                //====================================

                ST_WAIT_DONE: begin

                    if(i2c_done) begin

                        imu_id <= i2c_data;

                        imu_ack_error <= i2c_ack_error;


                        // 没有ACK错误才认为数据有效
                        if(!i2c_ack_error)
                            imu_valid <= 1'b1;
                        else
                            imu_valid <= 1'b0;


                        delay_cnt <= 25'd0;

                        state <= ST_RETRY_WAIT;

                    end

                end


                //====================================
                // 等待500ms，再读一次
                //====================================

                ST_RETRY_WAIT: begin

                    if(delay_cnt >= 25'd24999999) begin

                        delay_cnt <= 25'd0;

                        state <= ST_START;

                    end
                    else begin

                        delay_cnt <= delay_cnt + 1'b1;

                    end

                end


                default: begin

                    state <= ST_POWER_WAIT;

                end

            endcase

        end

    end


endmodule