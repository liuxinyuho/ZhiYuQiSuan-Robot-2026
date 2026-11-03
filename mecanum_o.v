
`timescale 1ns / 1ps

// ============================================================================
// mecanum_o.v -- ACG720 / GW5AT, 50 MHz timing pipeline (Verilog-2001)
//
// Four-wheel inverse kinematics, matching the prior direction convention:
//   A (left-front)  = Vx + Vy + Wz
//   B (left-rear)   = Vx - Vy + Wz
//   C (right-front) = Vx - Vy - Wz
//   D (right-rear)  = Vx + Vy - Wz
//   +Vx forward; +Vy right; +Wz right rotation.
//
// Critical path being repaired:
//   u_obstacle_avoidance/state -> top safety mux -> 13-bit +/-
//     -> target_speed_[a-d]_reg/D
//
// Pipeline:
//   Stage 1 (register):   latch final safety-filtered Vx/Vy/Wz
//   Stage 2 (register):   compute Vx+Vy and Vx-Vy; delay Wz in parallel
//   Stage 3 (downstream): add/subtract Wz, consumed by the existing
//                          target_speed_*_reg registers in top.v
//
// Adds two 50-MHz clock cycles (~40 ns) against prior combinational unit.
// The existing top-level motor emergency stop gate is NOT removed.
// Arithmetic is intentionally 13-bit signed, so it has the SAME modulo-8192
// behavior as the original 13-bit outputs. Do not replace with unsigned math.
// ============================================================================
module mecanum_o (
    input  wire               clk,
    input  wire               reset_n,
    input  wire signed [12:0] vx,
    input  wire signed [12:0] vy,
    input  wire signed [12:0] wz,
    output wire signed [12:0] speed_a,
    output wire signed [12:0] speed_b,
    output wire signed [12:0] speed_c,
    output wire signed [12:0] speed_d
);

    // First boundary: cut the obstacle-state and sensor-safety logic
    // away from all four arithmetic wheel paths.
    reg signed [12:0] vx_q;
    reg signed [12:0] vy_q;
    reg signed [12:0] wz_q;

    // Second boundary: do at most one 13-bit add/subtract in this stage.
    reg signed [12:0] vx_plus_vy_q;
    reg signed [12:0] vx_minus_vy_q;
    reg signed [12:0] wz_delayed_q;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            vx_q          <= 13'sd0;
            vy_q          <= 13'sd0;
            wz_q          <= 13'sd0;
            vx_plus_vy_q  <= 13'sd0;
            vx_minus_vy_q <= 13'sd0;
            wz_delayed_q  <= 13'sd0;
        end else begin
            vx_q          <= vx;
            vy_q          <= vy;
            wz_q          <= wz;
            vx_plus_vy_q  <= vx_q + vy_q;
            vx_minus_vy_q <= vx_q - vy_q;
            wz_delayed_q  <= wz_q;
        end
    end

    // Last stage: only one 13-bit adder/subtractor before top's
    // already-present target_speed_*_reg registers.
    assign speed_a = vx_plus_vy_q  + wz_delayed_q;
    assign speed_b = vx_minus_vy_q + wz_delayed_q;
    assign speed_c = vx_minus_vy_q - wz_delayed_q;
    assign speed_d = vx_plus_vy_q  - wz_delayed_q;

endmodule
