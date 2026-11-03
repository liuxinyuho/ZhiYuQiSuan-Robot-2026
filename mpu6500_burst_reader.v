//////////////////////////////////////////////////////////////////////////////////
// File Name   : mpu6500_burst_reader.v
//
// Description :
//   MPU6500 100Hz六轴连续读取模块。
//
//   每10ms启动一次I2C Burst Read。
//   一次从0x3B连续读取14Byte。
//
// Output:
//   AX AY AZ
//   TEMP
//   GX GY GZ
//
// data_valid:
//   每获得一组完整新数据产生一个CLK脉冲。
//////////////////////////////////////////////////////////////////////////////////

module mpu6500_burst_reader(
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
    // Burst Read
    //================================================

    reg burst_start;

    wire [111:0] burst_data;

    wire burst_busy;
    wire burst_done;
    wire burst_ack_error;


    i2c_burst_read14 u_i2c_burst_read14(

        .clk       (clk),
        .reset_n   (reset_n),

        .start     (burst_start),

        .dev_addr  (7'h68),

        .start_reg (8'h3B),

        .imu_sda   (imu_sda),
        .imu_scl   (imu_scl),

        .data_out  (burst_data),

        .busy      (burst_busy),
        .done      (burst_done),

        .ack_error (burst_ack_error)

    );


    //================================================
    // 100Hz定时器
    //
    // 50MHz / 100Hz = 500000
    //================================================

    reg [18:0] sample_cnt;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            sample_cnt <= 19'd0;

            burst_start <= 1'b0;

            data_valid <= 1'b0;

            read_error <= 1'b0;


            accel_x <= 16'sd0;
            accel_y <= 16'sd0;
            accel_z <= 16'sd0;

            temp_raw <= 16'sd0;

            gyro_x <= 16'sd0;
            gyro_y <= 16'sd0;
            gyro_z <= 16'sd0;

        end


        else begin

            burst_start <= 1'b0;

            data_valid <= 1'b0;


            //========================================
            // 初始化完成以后开始100Hz采样
            //========================================

            if(init_done) begin

                if(sample_cnt >= 19'd499999) begin

                    sample_cnt <= 19'd0;


                    if(!burst_busy)

                        burst_start <= 1'b1;

                end

                else begin

                    sample_cnt <=
                        sample_cnt + 1'b1;

                end

            end

            else begin

                sample_cnt <= 19'd0;

            end


            //========================================
            // 一组14Byte读取完成
            //========================================

            if(burst_done) begin

                read_error <= burst_ack_error;


                if(!burst_ack_error) begin

                    accel_x <=
                        $signed({
                            burst_data[111:104],
                            burst_data[103:96]
                        });

                    accel_y <=
                        $signed({
                            burst_data[95:88],
                            burst_data[87:80]
                        });

                    accel_z <=
                        $signed({
                            burst_data[79:72],
                            burst_data[71:64]
                        });


                    temp_raw <=
                        $signed({
                            burst_data[63:56],
                            burst_data[55:48]
                        });


                    gyro_x <=
                        $signed({
                            burst_data[47:40],
                            burst_data[39:32]
                        });

                    gyro_y <=
                        $signed({
                            burst_data[31:24],
                            burst_data[23:16]
                        });

                    gyro_z <=
                        $signed({
                            burst_data[15:8],
                            burst_data[7:0]
                        });


                    data_valid <= 1'b1;

                end

            end

        end

    end

endmodule