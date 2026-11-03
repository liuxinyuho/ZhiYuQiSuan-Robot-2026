
`timescale 1ns / 1ps

module d6_tof_fusion_guard #(
    parameter [15:0] FRONT_STOP_MM = 16'd250,
    parameter [15:0] FRONT_CLEAR_MM = 16'd500,
    parameter [15:0] SIDE_SAFE_MM = 16'd450,
    parameter [15:0] SCAN_SIDE_SAFE_MM = 16'd450,
    parameter [7:0] ULTRA_MAX_AGE = 8'd50,
    parameter [9:0] SCAN_MAX_AGE = 10'd300
)(
    input wire clk,
    input wire reset_n,
    input wire sample_tick,

    input wire [15:0] tof_left_mm,
    input wire [15:0] tof_front_mm,
    input wire [15:0] tof_right_mm,

    input wire tof_left_ok,
    input wire tof_front_ok,
    input wire tof_right_ok,

    input wire [15:0] ultra_distance_mm,
    input wire ultra_distance_valid,
    input wire ultra_echo_timeout,

    input wire scan_done,

    input wire [15:0] scan_left_mm,
    input wire [15:0] scan_center_mm,
    input wire [15:0] scan_right_mm,

    output wire all_tof_ok,
    output wire ultra_ok,
    output wire scan_ok,
    output wire auto_ready,

    output wire front_stop,
    output wire front_slow,

    output wire left_clear,
    output wire right_clear,

    output reg [1:0] suggested_direction
);

    localparam [1:0]
        DIR_HOLD    = 2'd0,
        DIR_FORWARD = 2'd1,
        DIR_LEFT    = 2'd2,
        DIR_RIGHT   = 2'd3;

    // ========================================
    // 超声波距离缓存
    // ========================================
    reg [15:0] ultra_last_mm;
    reg [7:0] ultra_age;
    reg ultra_seen;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ultra_last_mm <= 16'd0;
            ultra_age <= 8'hFF;
            ultra_seen <= 1'b0;
        end else if (
            ultra_distance_valid &&
            ultra_distance_mm >= 16'd80 &&
            ultra_distance_mm <= 16'd3000
        ) begin
            ultra_last_mm <= ultra_distance_mm;
            ultra_age <= 8'd0;
            ultra_seen <= 1'b1;
        end else if (
            sample_tick &&
            ultra_age != 8'hFF
        ) begin
            ultra_age <= ultra_age + 1'b1;
        end
    end

    // ========================================
    // 左右扫描结果缓存
    // ========================================
    reg [15:0] scan_left_last_mm;
    reg [15:0] scan_right_last_mm;
    reg [9:0] scan_age;
    reg scan_seen;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            scan_left_last_mm <= 16'd0;
            scan_right_last_mm <= 16'd0;
            scan_age <= 10'h3FF;
            scan_seen <= 1'b0;
        end else if (scan_done) begin
            scan_left_last_mm <= scan_left_mm;
            scan_right_last_mm <= scan_right_mm;
            scan_age <= 10'd0;
            scan_seen <= 1'b1;
        end else if (
            sample_tick &&
            scan_age != 10'h3FF
        ) begin
            scan_age <= scan_age + 1'b1;
        end
    end

    // ========================================
    // 传感器有效性
    // ========================================
    assign all_tof_ok =
        tof_left_ok &&
        tof_front_ok &&
        tof_right_ok;

    assign ultra_ok =
        ultra_seen &&
        (ultra_age <= ULTRA_MAX_AGE);

    assign scan_ok =
        scan_seen &&
        (scan_age <= SCAN_MAX_AGE);

    // ========================================
    // A5/A6 启动许可
    //
    // 不要求启动前五个传感器全部有效。
    // 这不改变任何一个传感器的真实有效位。
    // ========================================
    assign auto_ready = 1'b1;

    // ========================================
    // 前方近障判断
    //
    // 有效且近距离 -> 障碍
    // 无效、超量程、无回波 -> UNKNOWN
    // UNKNOWN 本身不触发停车
    // ========================================
    assign front_stop =
        (
            ultra_ok &&
            ultra_last_mm > 16'd0 &&
            ultra_last_mm <= FRONT_STOP_MM
        )
        ||
        (
            tof_front_ok &&
            tof_front_mm > 16'd0 &&
            tof_front_mm <= FRONT_STOP_MM
        );

    // ========================================
    // 前方减速判断
    // ========================================
    assign front_slow =
        (ultra_ok || tof_front_ok) &&
        !front_stop &&
        (
            (
                ultra_ok &&
                ultra_last_mm < FRONT_CLEAR_MM
            )
            ||
            (
                tof_front_ok &&
                tof_front_mm < FRONT_CLEAR_MM
            )
        );

    // ========================================
    // 左右侧可通行判断
    //
    // 无效不当作近障。
    // 但此输出不代表物理空间已被证实安全。
    // 实际运动还有 top.v 的最终保护。
    // ========================================
    assign left_clear =
        (
            !tof_left_ok ||
            tof_left_mm >= SIDE_SAFE_MM
        )
        &&
        (
            !scan_ok ||
            scan_left_last_mm == 16'd0 ||
            scan_left_last_mm >= SCAN_SIDE_SAFE_MM
        );

    assign right_clear =
        (
            !tof_right_ok ||
            tof_right_mm >= SIDE_SAFE_MM
        )
        &&
        (
            !scan_ok ||
            scan_right_last_mm == 16'd0 ||
            scan_right_last_mm >= SCAN_SIDE_SAFE_MM
        );

    // ========================================
    // 推荐运动方向
    // ========================================
    always @(*) begin
        suggested_direction = DIR_HOLD;

        if (auto_ready) begin

            if (
                (!ultra_ok ||
                 ultra_last_mm >= FRONT_CLEAR_MM)
                &&
                (!tof_front_ok ||
                 tof_front_mm >= FRONT_CLEAR_MM)
            ) begin
                suggested_direction = DIR_FORWARD;
            end

            else if (left_clear && right_clear) begin
                if (
                    scan_left_last_mm >=
                    scan_right_last_mm
                )
                    suggested_direction = DIR_LEFT;
                else
                    suggested_direction = DIR_RIGHT;
            end

            else if (left_clear) begin
                suggested_direction = DIR_LEFT;
            end

            else if (right_clear) begin
                suggested_direction = DIR_RIGHT;
            end

        end
    end

    wire unused_scan_center = ^scan_center_mm;

endmodule
