`timescale 1ns / 1ps
// ============================================================================
// ACG720 / HC-SR04 + SG90, robust 3-direction scan (Verilog-2001)
// Direction: 0=RIGHT, 1=CENTER, 2=LEFT.
// Route: CENTER -> RIGHT -> LEFT -> CENTER.
//
// Each direction collects three fresh, in-range echoes AFTER servo settling.
// A pair must agree within PAIR_TOL_MM for the result to be accepted.
// If only two agreed samples arrive by SAMPLE_TIMEOUT_MS, their midpoint is used.
// Otherwise this direction is UNKNOWN (distance=0, never "clear").
// A watchdog per direction guarantees scan_done even with disconnected HC-SR04.
// scan_done means SCAN ENDED, not "all three directions were valid".
// UART SV is set separately by the reporter only if all three are nonzero.
// ============================================================================
module servo_ultrasonic_scan #(
    parameter integer CLK_FREQ = 50000000,
    parameter integer SETTLE_MS = 250,
    parameter integer SAMPLE_TIMEOUT_MS = 650,
    parameter [15:0] MIN_RANGE_MM = 16'd80,
    parameter [15:0] MAX_RANGE_MM = 16'd3000,
    parameter [15:0] PAIR_TOL_MM = 16'd120
)(
    input wire clk,
    input wire reset_n,
    input wire [7:0] rx_data,
    input wire rx_done,
    input wire [15:0] distance_mm,
    input wire distance_valid,
    input wire echo_timeout,
    output reg [1:0] servo_position,
    output reg [15:0] left_distance_mm,
    output reg [15:0] center_distance_mm,
    output reg [15:0] right_distance_mm,
    output reg scan_active,
    output reg scan_done
);
    localparam [1:0] POS_RIGHT=2'd0, POS_CENTER=2'd1, POS_LEFT=2'd2;
    localparam [2:0] ST_IDLE=3'd0, ST_C_SETTLE=3'd1, ST_C_SAMPLE=3'd2,
                     ST_R_SETTLE=3'd3, ST_R_SAMPLE=3'd4,
                     ST_L_SETTLE=3'd5, ST_L_SAMPLE=3'd6,
                     ST_HOME=3'd7;

    localparam integer CYCLES_PER_MS = CLK_FREQ / 1000;
    localparam integer SETTLE_CYCLES = CYCLES_PER_MS * SETTLE_MS;
    localparam integer SAMPLE_CYCLES = CYCLES_PER_MS * SAMPLE_TIMEOUT_MS;

    reg [2:0] state;
    reg [31:0] timer;
    reg [1:0] good_count;
    reg [15:0] first_mm, second_mm;

    // Input valid is a one-clock pulse. Invalid ultrasonic returns and timeout
    // are UNKNOWN, not infinite free space. The direction watchdog keeps
    // scanning regardless of whether timeout arrives as a pulse or a level.
    wire new_sample = distance_valid && !echo_timeout &&
                      (distance_mm >= MIN_RANGE_MM) &&
                      (distance_mm <= MAX_RANGE_MM);

    function [15:0] absdiff16;
        input [15:0] a, b;
        begin
            absdiff16 = (a >= b) ? (a - b) : (b - a);
        end
    endfunction

    function [15:0] median3;
        input [15:0] a, b, c;
        begin
            if (a > b)
                median3 = (b > c) ? b : ((a > c) ? c : a);
            else
                median3 = (a > c) ? a : ((b > c) ? c : b);
        end
    endfunction

    // Reject isolated HC-SR04 reflections. At least two echoes must agree.
    function [15:0] consistent3;
        input [15:0] a, b, c;
        input [15:0] tol;
        begin
            if ((absdiff16(a,b) <= tol) ||
                (absdiff16(a,c) <= tol) ||
                (absdiff16(b,c) <= tol))
                consistent3 = median3(a,b,c);
            else
                consistent3 = 16'd0;
        end
    endfunction

    wire [15:0] third_result =
        consistent3(first_mm, second_mm, distance_mm, PAIR_TOL_MM);
    wire [15:0] timeout_result =
        (good_count == 2'd2 &&
         absdiff16(first_mm,second_mm) <= PAIR_TOL_MM) ?
          (({1'b0,first_mm} + {1'b0,second_mm}) >> 1) : 16'd0;


    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_IDLE;
            timer <= 32'd0;
            good_count <= 2'd0;
            first_mm <= 16'd0;
            second_mm <= 16'd0;
            servo_position <= POS_CENTER;
            left_distance_mm <= 16'd0;
            center_distance_mm <= 16'd0;
            right_distance_mm <= 16'd0;
            scan_active <= 1'b0;
            scan_done <= 1'b0;
        end else begin
            scan_done <= 1'b0;
            case (state)
                ST_IDLE: begin
                    scan_active <= 1'b0;
                    servo_position <= POS_CENTER;
                    timer <= 32'd0;
                    good_count <= 2'd0;
                    if (rx_done && rx_data == 8'hE4) begin
                        // Old measurements are never reused as new scan data.
                        left_distance_mm <= 16'd0;
                        center_distance_mm <= 16'd0;
                        right_distance_mm <= 16'd0;
                        scan_active <= 1'b1;
                        state <= ST_C_SETTLE;
                    end
                end
                ST_C_SETTLE: begin
                    servo_position <= POS_CENTER;
                    if (timer >= SETTLE_CYCLES-1) begin
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_C_SAMPLE;
                    end else timer <= timer + 1'b1;
                end
                ST_C_SAMPLE: begin
                    // Third valid sample wins over the watchdog on the same edge.
                    if (new_sample && good_count == 2'd2) begin
                        center_distance_mm <= third_result;
                        servo_position <= POS_RIGHT;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_R_SETTLE;
                    end else if (timer >= SAMPLE_CYCLES-1) begin
                        // No three stable echoes: try two agreeing readings;
                        // otherwise store 0 (UNKNOWN) and advance.
                        center_distance_mm <= timeout_result;
                        servo_position <= POS_RIGHT;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_R_SETTLE;
                    end else begin
                        timer <= timer + 1'b1;
                        if (new_sample) begin
                            if (good_count == 2'd0) begin
                                first_mm <= distance_mm;
                                good_count <= 2'd1;
                            end else if (good_count == 2'd1) begin
                                second_mm <= distance_mm;
                                good_count <= 2'd2;
                            end
                        end
                    end
                end
                ST_R_SETTLE: begin
                    if (timer >= SETTLE_CYCLES-1) begin
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_R_SAMPLE;
                    end else timer <= timer + 1'b1;
                end
                ST_R_SAMPLE: begin
                    // Third valid sample wins over the watchdog on the same edge.
                    if (new_sample && good_count == 2'd2) begin
                        right_distance_mm <= third_result;
                        servo_position <= POS_LEFT;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_L_SETTLE;
                    end else if (timer >= SAMPLE_CYCLES-1) begin
                        // No three stable echoes: try two agreeing readings;
                        // otherwise store 0 (UNKNOWN) and advance.
                        right_distance_mm <= timeout_result;
                        servo_position <= POS_LEFT;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_L_SETTLE;
                    end else begin
                        timer <= timer + 1'b1;
                        if (new_sample) begin
                            if (good_count == 2'd0) begin
                                first_mm <= distance_mm;
                                good_count <= 2'd1;
                            end else if (good_count == 2'd1) begin
                                second_mm <= distance_mm;
                                good_count <= 2'd2;
                            end
                        end
                    end
                end
                ST_L_SETTLE: begin
                    if (timer >= SETTLE_CYCLES-1) begin
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_L_SAMPLE;
                    end else timer <= timer + 1'b1;
                end
                ST_L_SAMPLE: begin
                    // Third valid sample wins over the watchdog on the same edge.
                    if (new_sample && good_count == 2'd2) begin
                        left_distance_mm <= third_result;
                        servo_position <= POS_CENTER;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_HOME;
                    end else if (timer >= SAMPLE_CYCLES-1) begin
                        // No three stable echoes: try two agreeing readings;
                        // otherwise store 0 (UNKNOWN) and advance.
                        left_distance_mm <= timeout_result;
                        servo_position <= POS_CENTER;
                        timer <= 32'd0;
                        good_count <= 2'd0;
                        state <= ST_HOME;
                    end else begin
                        timer <= timer + 1'b1;
                        if (new_sample) begin
                            if (good_count == 2'd0) begin
                                first_mm <= distance_mm;
                                good_count <= 2'd1;
                            end else if (good_count == 2'd1) begin
                                second_mm <= distance_mm;
                                good_count <= 2'd2;
                            end
                        end
                    end
                end
                ST_HOME: begin
                    if (timer >= SETTLE_CYCLES-1) begin
                        timer <= 32'd0;
                        scan_active <= 1'b0;
                        scan_done <= 1'b1;  // completion, even if some ranges UNKNOWN
                        state <= ST_IDLE;
                    end else timer <= timer + 1'b1;
                end
                default: begin
                    state <= ST_IDLE;
                    scan_active <= 1'b0;
                    servo_position <= POS_CENTER;
                    timer <= 32'd0;
                end
            endcase
        end
    end
endmodule
