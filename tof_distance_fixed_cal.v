
`timescale 1ns / 1ps

// ============================================================================
// ACG720 2026 Task-3: four independently calibrated VL53L0X channels.
// Navigation effective distance: 0..1200 mm (saturated, NOT rejected).
//
// IMPORTANT:
// 1. RAW_MAX_MM is a raw-sample plausibility bound, NOT navigation range.
//    Do not set RAW_MAX_MM to 1200: raw data is not calibrated distance;
//    doing so would turn genuine far readings into invalid data.
// 2. Values above NAV_MAX_MM become 1200 with calibrated_valid=1.
//    They mean "at least 1200 mm measured" rather than an exact 1200 mm.
// 3. A raw zero is UNKNOWN: it might be out-of-range, near-field blind,
//    no return, or a measurement failure. Do NOT invent clearance on zero.
// 4. Four existing calibration coefficients are unchanged.
// 5. All inputs/outputs remain identical to the prior module.
// ============================================================================
module tof_distance_fixed_cal #(
    parameter integer CAL_MODE       = 1,
    parameter integer OFFSET_MM      = 0,
    parameter integer GAIN_Q12       = 4096,
    parameter [15:0] RAW_MIN_MM      = 16'd30,
    parameter [15:0] RAW_MAX_MM      = 16'd2000,
    parameter [15:0] NAV_MAX_MM      = 16'd1200
)(
    input  wire        clk,
    input  wire        reset_n,
    input  wire [15:0] raw_distance_mm,
    input  wire        raw_valid,
    output reg  [15:0] calibrated_mm,
    output reg         calibrated_valid
);

    wire signed [31:0] raw_signed =
        $signed({16'd0, raw_distance_mm});

    reg signed [31:0] centered;
    reg signed [31:0] multiplied;
    reg signed [31:0] calculated;
    reg [15:0] bounded_mm;

    wire raw_in_plausible_range =
        raw_valid &&
        (raw_distance_mm >= RAW_MIN_MM) &&
        (raw_distance_mm <= RAW_MAX_MM);

    always @(*) begin
        centered   = 32'sd0;
        multiplied = 32'sd0;
        calculated = raw_signed;

        if (CAL_MODE == 2) begin
            // Rear sensor (B): original two-segment calibration, unchanged.
            if (raw_distance_mm <= 16'd300) begin
                centered   = raw_signed - 32'sd170;
                multiplied = centered * 32'sd3151;
                calculated = 32'sd100 +
                    ((multiplied + 32'sd2048) >>> 12);
            end else begin
                centered   = raw_signed - 32'sd97;
                multiplied = centered * 32'sd4043;
                calculated = (multiplied + 32'sd2048) >>> 12;
            end
        end else if (CAL_MODE == 1) begin
            // L/R/F: original independent linear calibrations, unchanged.
            centered   = raw_signed - OFFSET_MM;
            multiplied = centered * GAIN_Q12;
            calculated = (multiplied + 32'sd2048) >>> 12;
        end

        // Saturate the CALIBRATED distance for navigation.
        // A nonzero raw measurement above the cap stays valid.
        if (calculated < 32'sd0)
            bounded_mm = 16'd0;
        else if (calculated > $signed({16'd0, NAV_MAX_MM}))
            bounded_mm = NAV_MAX_MM;
        else
            bounded_mm = calculated[15:0];
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            calibrated_mm    <= 16'd0;
            calibrated_valid <= 1'b0;
        end else begin
            calibrated_valid <= raw_in_plausible_range;
            if (raw_in_plausible_range)
                calibrated_mm <= bounded_mm;
        end
    end
endmodule
