
`timescale 1ns / 1ps

// ================================================================
// motor_driver.v
// ACG720 - 50MHz Timing Optimization
//
// Function:
//   1. Signed 13-bit motor speed input
//   2. Motor direction control
//   3. PWM magnitude limiting
//   4. Pipeline registers for timing optimization
//
// Original motor direction mapping is preserved.
//
// Pipeline:
//   Stage 1: Register speed command
//   Stage 2: Register PWM duty and direction
//   Stage 3: PWM generator
//
// The top-level emergency stop protection is preserved.
// ================================================================

module motor_driver #(
    parameter REVERSE = 1'b0
)(
    input  wire               clk,
    input  wire               reset_n,
    input  wire signed [12:0] speed_cmd,

    output wire               motor_pwm,
    output reg                motor_in1,
    output reg                motor_in2
);

    // ============================================================
    // Stage 1: Register input speed command
    // ============================================================

    reg signed [12:0] speed_cmd_q;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            speed_cmd_q <= 13'sd0;
        else
            speed_cmd_q <= speed_cmd;
    end

    // ============================================================
    // Stage 2: Calculate motor PWM magnitude
    // ============================================================

    // Use 13 bits to handle the full signed input range.

    wire [12:0] speed_magnitude =
        speed_cmd_q[12]
        ? (~speed_cmd_q[12:0] + 13'd1)
        : speed_cmd_q[12:0];

    // PWM period = 2500 counts
    // Duty limit = 2499

    wire [11:0] duty_limited =
        (speed_magnitude > 13'd2499)
        ? 12'd2499
        : speed_magnitude[11:0];

    // Registered duty

    reg [11:0] duty_q;

    // ============================================================
    // Stage 2: Register direction and duty together
    // ============================================================

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin

            duty_q    <= 12'd0;
            motor_in1 <= 1'b0;
            motor_in2 <= 1'b0;

        end

        else if (speed_cmd_q == 13'sd0) begin

            // Stop motor when commanded speed is zero

            duty_q    <= 12'd0;
            motor_in1 <= 1'b0;
            motor_in2 <= 1'b0;

        end

        else begin

            duty_q <= duty_limited;

            // Positive speed
            // Preserve original REVERSE semantics

            if (speed_cmd_q > 13'sd0) begin

                motor_in1 <=  REVERSE;
                motor_in2 <= ~REVERSE;

            end

            // Negative speed

            else begin

                motor_in1 <= ~REVERSE;
                motor_in2 <=  REVERSE;

            end

        end

    end

    // ============================================================
    // Stage 3: PWM generator
    // ============================================================

    pwm_gen u_pwm_gen (

        .clk     (clk),
        .reset_n (reset_n),

        .duty    (duty_q),

        .pwm_out (motor_pwm)

    );

endmodule
