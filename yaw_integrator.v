
`timescale 1ns / 1ps

//====================================================
// ACG720 GW5AT-60K
// MPU6500 YAW INTEGRATOR
//
// Timing optimized pipelined version
//
// System clock : 50 MHz
// IMU update   : nominal 100 Hz
// YAW unit     : 0.01 degree
//
// Pipeline:
//  1. Capture corrected gyro Z
//  2. Deadband and accumulate
//  3. Wrap to +/-180 degrees
//  4. Compute 126 * yaw accumulator
//  5. Compute 125 * accumulator and scale
//
// Original interfaces and scale are preserved.
//====================================================

module yaw_integrator #(
    parameter signed [31:0] DEADBAND = 32'sd30
)(
    input wire clk,
    input wire reset_n,

    input wire data_valid,
    input wire calib_done,
    input wire read_error,

    input wire signed [31:0] gyro_z_corrected,

    input wire zero_yaw,

    output reg signed [31:0] yaw_cdeg
);

//====================================================
// 1. Constants
//====================================================

localparam signed [31:0] YAW_180_RAW =
    32'sd2358000;

localparam signed [31:0] YAW_360_RAW =
    32'sd4716000;

//====================================================
// 2. Pipeline registers
//====================================================

// Stage 1: capture gyro Z
reg signed [31:0] gyro_z_s1;
reg valid_s1;

// Stage 2: accumulate
reg signed [31:0] yaw_sum_s2;
reg valid_s2;

// Stage 3: wrap
reg signed [31:0] yaw_accum;
reg valid_s3;

// Stage 4: partial scale
reg signed [31:0] scaled_126_s4;
reg signed [31:0] yaw_raw_s4;
reg valid_s4;

//====================================================
// 3. Deadband
//====================================================

wire signed [31:0] gyro_z_used;

assign gyro_z_used =
    (
        (gyro_z_s1 <= DEADBAND) &&
        (gyro_z_s1 >= -DEADBAND)
    )
    ? 32'sd0
    : gyro_z_s1;

//====================================================
// 4. Accumulator addition
//====================================================

wire signed [31:0] yaw_sum_next;

assign yaw_sum_next =
    yaw_accum + gyro_z_used;

//====================================================
// 5. Wrap to -180 ... +180 degrees
//====================================================

reg signed [31:0] yaw_wrapped_next;

always @(*) begin

    if (yaw_sum_s2 > YAW_180_RAW) begin

        yaw_wrapped_next =
            yaw_sum_s2 - YAW_360_RAW;

    end else if (yaw_sum_s2 < -YAW_180_RAW) begin

        yaw_wrapped_next =
            yaw_sum_s2 + YAW_360_RAW;

    end else begin

        yaw_wrapped_next = yaw_sum_s2;

    end

end

//====================================================
// 6. Angle scaling
//
// Original approximation:
//
// yaw_cdeg = yaw_accum * 125 / 16384
//
// 125 = 126 - 1
// 126 = 128 - 2
//
// Split into separate clock stages:
// Stage 4: raw * 126
// Stage 5: raw * 126 - raw
//====================================================

wire signed [31:0] scaled_126_next;

assign scaled_126_next =
    (yaw_accum <<< 7) -
    (yaw_accum <<< 1);

wire signed [31:0] scaled_125_next;

assign scaled_125_next =
    scaled_126_s4 - yaw_raw_s4;

wire signed [31:0] yaw_cdeg_next;

assign yaw_cdeg_next =
    scaled_125_next >>> 14;

//====================================================
// 7. Pipelined sequential logic
//====================================================

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        gyro_z_s1 <= 32'sd0;
        valid_s1 <= 1'b0;

        yaw_sum_s2 <= 32'sd0;
        valid_s2 <= 1'b0;

        yaw_accum <= 32'sd0;
        valid_s3 <= 1'b0;

        scaled_126_s4 <= 32'sd0;
        yaw_raw_s4 <= 32'sd0;
        valid_s4 <= 1'b0;

        yaw_cdeg <= 32'sd0;

    end else if (zero_yaw) begin

        // Reset complete pipeline to prevent an
        // old in-flight sample after zeroing.

        gyro_z_s1 <= 32'sd0;
        valid_s1 <= 1'b0;

        yaw_sum_s2 <= 32'sd0;
        valid_s2 <= 1'b0;

        yaw_accum <= 32'sd0;
        valid_s3 <= 1'b0;

        scaled_126_s4 <= 32'sd0;
        yaw_raw_s4 <= 32'sd0;
        valid_s4 <= 1'b0;

        yaw_cdeg <= 32'sd0;

    end else begin

        //========================================
        // Stage 1:
        // Capture valid gyro sample
        //========================================

        valid_s1 <=
            calib_done &&
            data_valid &&
            !read_error;

        if (calib_done &&
            data_valid &&
            !read_error) begin

            gyro_z_s1 <= gyro_z_corrected;

        end

        //========================================
        // Stage 2:
        // Deadband and addition
        //========================================

        valid_s2 <= valid_s1;

        if (valid_s1) begin

            yaw_sum_s2 <= yaw_sum_next;

        end

        //========================================
        // Stage 3:
        // +/-180 degree wrapping
        //========================================

        valid_s3 <= valid_s2;

        if (valid_s2) begin

            yaw_accum <= yaw_wrapped_next;

        end

        //========================================
        // Stage 4:
        // First scaling operation
        //========================================

        valid_s4 <= valid_s3;

        if (valid_s3) begin

            scaled_126_s4 <= scaled_126_next;

            yaw_raw_s4 <= yaw_accum;

        end

        //========================================
        // Stage 5:
        // Final scaling and registered output
        //========================================

        if (valid_s4) begin

            yaw_cdeg <= yaw_cdeg_next;

        end

    end

end

endmodule
