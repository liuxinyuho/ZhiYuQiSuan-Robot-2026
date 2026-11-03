//////////////////////////////////////////////////////////////////////////////////
// File Name   : gyro_calib.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   MPU6500陀螺仪零偏自动校准模块。
//
// Function :
//   MPU初始化完成后保持机器人静止。
//   模块自动采集1024组GX/GY/GZ原始数据并求平均。
//   平均值作为陀螺仪零偏Bias.
//
// Important :
//   校准期间机器人必须保持完全静止。
//////////////////////////////////////////////////////////////////////////////////

module gyro_calib
(
    input wire clk,
    input wire reset_n,

    // MPU一组新数据产生脉冲
    input wire data_valid,

    // 原始陀螺仪
    input wire signed [15:0] gyro_x_raw,
    input wire signed [15:0] gyro_y_raw,
    input wire signed [15:0] gyro_z_raw,

    // 计算出的零偏
    output reg signed [15:0] bias_x,
    output reg signed [15:0] bias_y,
    output reg signed [15:0] bias_z,

    // 校准后的陀螺仪数据
    output wire signed [31:0] gyro_x_corrected,
    output wire signed [31:0] gyro_y_corrected,
    output wire signed [31:0] gyro_z_corrected,

    // 1 = 校准完成
    output reg calib_done
);


    //================================================
    // Sample Counter
    //
    // 1024 = 2^10
    //================================================

    reg [10:0] sample_count;


    //================================================
    // 累加器
    //================================================

    reg signed [31:0] sum_x;
    reg signed [31:0] sum_y;
    reg signed [31:0] sum_z;


    //================================================
    // 16bit原始数据 -> 32bit符号扩展
    //================================================

    wire signed [31:0] gx_ext;
    wire signed [31:0] gy_ext;
    wire signed [31:0] gz_ext;


    assign gx_ext =
    {
        {16{gyro_x_raw[15]}},
        gyro_x_raw
    };


    assign gy_ext =
    {
        {16{gyro_y_raw[15]}},
        gyro_y_raw
    };


    assign gz_ext =
    {
        {16{gyro_z_raw[15]}},
        gyro_z_raw
    };


    //================================================
    // 最后一个采样加入后的总和
    //================================================

    wire signed [31:0] final_sum_x;
    wire signed [31:0] final_sum_y;
    wire signed [31:0] final_sum_z;


    assign final_sum_x =
        sum_x + gx_ext;

    assign final_sum_y =
        sum_y + gy_ext;

    assign final_sum_z =
        sum_z + gz_ext;


    //================================================
    // 1024点平均值
    //
    // signed arithmetic shift
    //================================================

    wire signed [31:0] average_x;
    wire signed [31:0] average_y;
    wire signed [31:0] average_z;


    assign average_x =
        final_sum_x >>> 10;

    assign average_y =
        final_sum_y >>> 10;

    assign average_z =
        final_sum_z >>> 10;


    //================================================
    // Bias符号扩展
    //================================================

    wire signed [31:0] bias_x_ext;
    wire signed [31:0] bias_y_ext;
    wire signed [31:0] bias_z_ext;


    assign bias_x_ext =
    {
        {16{bias_x[15]}},
        bias_x
    };


    assign bias_y_ext =
    {
        {16{bias_y[15]}},
        bias_y
    };


    assign bias_z_ext =
    {
        {16{bias_z[15]}},
        bias_z
    };


    //================================================
    // Corrected Gyro
    //================================================

    assign gyro_x_corrected =
        gx_ext - bias_x_ext;


    assign gyro_y_corrected =
        gy_ext - bias_y_ext;


    assign gyro_z_corrected =
        gz_ext - bias_z_ext;


    //================================================
    // Calibration FSM
    //================================================

    always @(posedge clk or negedge reset_n)
    begin

        if(!reset_n)
        begin

            sample_count <=
                11'd0;


            sum_x <=
                32'sd0;

            sum_y <=
                32'sd0;

            sum_z <=
                32'sd0;


            bias_x <=
                16'sd0;

            bias_y <=
                16'sd0;

            bias_z <=
                16'sd0;


            calib_done <=
                1'b0;

        end


        else if(!calib_done && data_valid)
        begin

            //========================================
            // 前1023个样本
            //========================================

            if(sample_count < 11'd1023)
            begin

                sum_x <=
                    sum_x + gx_ext;

                sum_y <=
                    sum_y + gy_ext;

                sum_z <=
                    sum_z + gz_ext;


                sample_count <=
                    sample_count + 1'b1;

            end


            //========================================
            // 第1024个样本
            //
            // average_x/y/z已经包含：
            //
            // sum + 当前样本
            //
            // 并完成 >>> 10
            //========================================

            else
            begin

                // 平均值一定处于16bit陀螺仪范围内，
                // 因此明确取低16bit。
                //
                // 这样不会再产生位宽截断warning。

                bias_x <=
                    average_x[15:0];

                bias_y <=
                    average_y[15:0];

                bias_z <=
                    average_z[15:0];


                calib_done <=
                    1'b1;

            end

        end

    end


endmodule