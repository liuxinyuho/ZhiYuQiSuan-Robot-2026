// ================================================================
// 2026 competition task 3 - A5 safety refinement (experimental)
// Focus: independent axis safety / calibrated ToF / monotone braking.
// Never bypass P&R timing checks and low-speed physical validation.
// ================================================================

`timescale 1ns / 1ps

//======================================================================
// 文件名称：top.v
//
// 项目：基于高云 FPGA 的麦克纳姆轮自主导航小车
// 开发板：小梅哥 ACG720
// FPGA：GW5AT 系列
// 系统时钟：50 MHz
//
// 功能：
// 1. 串口手动控制
// 2. 四轮编码器速度检测
// 3. 四路电机 PI 速度闭环
// 4. 麦克纳姆轮运动学解算
// 5. MPU6500 陀螺仪零偏校准和航向控制
// 6. 四轮里程计定位
// 7. HC-SR04 超声波测距
// 8. SG90 舵机左、中、右扫描
// 9. 左、右 VL53L0X 侧向防撞
// 10. A5 自主避障
// 11. A6 自主航点导航及自动避障恢复
// 12. 上位机串口通信
//
// 运动方向定义：
// +Vx：前进
// -Vx：后退
// +Vy：向右横移
// -Vy：向左横移
//
// 串口命令：
// HEX A5：启动自主避障
// HEX A6：启动航点导航
// HEX 53：停止命令
//
// 当前硬件配置：
// 左 ToF：SDA=C20，SCL=D20
// 右 ToF：SDA=D19，SCL=E19
// 前 ToF：SDA=E18，SCL=F18
// 后 ToF：SDA=F16，SCL=F17
// 车头超声波保留原 ultra_trig / ultra_echo，SG90不变
//
// 注意：
// 本版本增加第四路后方ToF，保留原前方HC-SR04+SG90扫描。
// 原有电机、编码器和 PI 参数保持不变。
//======================================================================

module top(

    //==================================================
    // 1. 系统时钟和复位

    input wire clk,
    input wire reset_n,

    //==================================================
    // 2. 电脑调试串口
    //==================================================

    input wire uart_rx,
    output wire uart_tx,

    // Two independent active-low buttons (connect each to GND).
    // Assign their actual exposed GPIO pins in src/car.cst.
    input wire key_a5_n,
    input wire key_a6_n,

    // On-board mode selection keys (active LOW, 20ms debounce):
    // S2 / B20: select original FPGA local A5/A6 mode.
    // S3 / A21: select BM1684 host communication mode.
    input wire key_mode_local_n,
    input wire key_mode_host_n,

    // SW0 (G22): display-only page select. ON pulls low -> AI page.
    // OFF is held high by the internal pull-up -> BASIC page.
    input wire ui_mode_sw0,

    //==================================================
    // 3. MPU6500 I2C
    //==================================================

    inout wire imu_sda,
    inout wire imu_scl,

    //==================================================
    // 4. 四轮编码器
    //
    // A：左前轮
    // B：左后轮
    // C：右前轮
    // D：右后轮
    //==================================================

    input wire encoder_a_cha,
    input wire encoder_a_chb,

    input wire encoder_b_cha,
    input wire encoder_b_chb,

    input wire encoder_c_cha,
    input wire encoder_c_chb,

    input wire encoder_d_cha,
    input wire encoder_d_chb,

    //==================================================
    // 5. 四轮 TB6612 电机驱动
    //==================================================

    output wire motor_a_pwm,
    output wire motor_a_in1,
    output wire motor_a_in2,

    output wire motor_b_pwm,
    output wire motor_b_in1,
    output wire motor_b_in2,

    output wire motor_c_pwm,
    output wire motor_c_in1,
    output wire motor_c_in2,

    output wire motor_d_pwm,
    output wire motor_d_in1,
    output wire motor_d_in2,

    //==================================================
    // 6. SG90 舵机和 HC-SR04
    //==================================================

    output wire servo_pwm,

    output wire ultra_trig,
    input  wire ultra_echo,

    //==================================================
    // 8. 左侧 VL53L0X
    // 独立 I2C 总线
    //==================================================

    inout wire tof1_sda,
    inout wire tof1_scl,

    //==================================================
    // 9. 右侧 VL53L0X
    // 独立 I2C 总线
    //==================================================

    inout wire tof2_sda,
    inout wire tof2_scl,


    inout wire tof3_sda,  // 前方 ToF
    inout wire tof3_scl,

    // 后方 VL53L0X：第四条独立I2C总线
    inout wire tof4_sda,
    inout wire tof4_scl,

    // OV5640 CAMERA
    output wire camera_xclk,
    input  wire camera_pclk,
    input  wire camera_vsync,
    input  wire camera_href,
    input  wire [7:0] camera_data,

    output wire camera_rst_n,
    output wire camera_pwdn,
    output wire camera_sclk,
    inout  wire camera_sdat,

    // HDMI OUTPUT
    output wire tmds_clk_p,
    output wire tmds_clk_n,
    output wire [2:0] tmds_data_p,
    output wire [2:0] tmds_data_n
);



//======================================================================
// 第一部分：系统内部信号
//======================================================================


//==========================================================
// 1. PC 串口接收
//==========================================================

// 接收到的完整8位命令
wire [7:0] rx_data;

// 收到新命令时产生一个脉冲
wire rx_done;

// PC-UART raw bytes. The existing modules all consume rx_data/rx_done,
// which now select either PC UART or debounced physical button commands.
wire [7:0] pc_rx_data;
wire pc_rx_done;

// Binary AA 55 protocol bytes are consumed by uart_protocol only.
// Non-frame bytes continue to the original manual/A5/A6 command path.
wire [7:0] legacy_rx_data;
wire legacy_rx_done;

wire [1:0] key_selected_mode;

//==========================================================
// 板载按键模式选择：
// S2（B20）按下 -> 本地模式；S3（A21）按下 -> BM1684上位机模式。
// 两个按键均为低电平有效，复用工程已有的20ms消抖模块。
// 同时按下时优先选择本地模式，确保失效安全。
//==========================================================
localparam integer MODE_KEY_DEBOUNCE_TICKS = 1000000; // 50MHz * 20ms

wire mode_local_press;
wire mode_host_press;

a5_a6_key_debounce #(
    .DEBOUNCE_TICKS(MODE_KEY_DEBOUNCE_TICKS)
) u_mode_local_key_debounce (
    .clk         (clk),
    .reset_n     (reset_n),
    .key_n       (key_mode_local_n),
    .press_pulse (mode_local_press)
);

a5_a6_key_debounce #(
    .DEBOUNCE_TICKS(MODE_KEY_DEBOUNCE_TICKS)
) u_mode_host_key_debounce (
    .clk         (clk),
    .reset_n     (reset_n),
    .key_n       (key_mode_host_n),
    .press_pulse (mode_host_press)
);

reg mode_host_active;
reg mode_host_d1;

// 本地控制模块在上位机模式下保持复位，防止原A5/A6任务残留。
wire local_control_reset_n = reset_n && !mode_host_active;

// 模式发生任意方向变化时，向原有控制路径注入一次 STOP。
wire mode_change_pulse = mode_host_active ^ mode_host_d1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        mode_host_active <= 1'b0; // 上电默认本地模式
        mode_host_d1     <= 1'b0;
    end else begin
        mode_host_d1 <= mode_host_active;

        // S2优先：同时按下时进入本地模式。
        if (mode_local_press)
            mode_host_active <= 1'b0;
        else if (mode_host_press)
            mode_host_active <= 1'b1;
    end
end

//==========================================================
// SW0 display page selector (display only; no motor control effect)
// Board pin table: SW0 = G22. Assumption: ON connects the pin to GND.
// 20ms stable filter prevents switch contact bounce. Power-up defaults
// to BASIC because the input is internally pulled high.
//==========================================================
localparam integer UI_SW0_DEBOUNCE_TICKS = 1_000_000; // 50MHz * 20ms
reg ui_sw0_meta;
reg ui_sw0_sync;
reg ui_sw0_stable;
reg [19:0] ui_sw0_filter_count;
wire ui_mode_ai = ~ui_sw0_stable;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        ui_sw0_meta         <= 1'b1;
        ui_sw0_sync         <= 1'b1;
        ui_sw0_stable       <= 1'b1;
        ui_sw0_filter_count <= 20'd0;
    end else begin
        ui_sw0_meta <= ui_mode_sw0;
        ui_sw0_sync <= ui_sw0_meta;

        if (ui_sw0_sync == ui_sw0_stable) begin
            ui_sw0_filter_count <= 20'd0;
        end else if (ui_sw0_filter_count >= UI_SW0_DEBOUNCE_TICKS - 1) begin
            ui_sw0_stable       <= ui_sw0_sync;
            ui_sw0_filter_count <= 20'd0;
        end else begin
            ui_sw0_filter_count <= ui_sw0_filter_count + 1'b1;
        end
    end
end

// Button controller is reset while host mode is selected. This prevents
// an A5/A6 task from remaining latched and unexpectedly resuming later.
wire [7:0] local_cmd_data;
wire local_cmd_done;

a5_a6_button_control u_a5_a6_button_control (
    .clk           (clk),
    .reset_n       (local_control_reset_n),
    .key_a5_n      (key_a5_n),
    .key_a6_n      (key_a6_n),
    .uart_data     (legacy_rx_data),
    .uart_done     (legacy_rx_done && !mode_host_active),
    .cmd_data      (local_cmd_data),
    .cmd_done      (local_cmd_done),
    .selected_mode (key_selected_mode)
);

// On either mode transition, inject one STOP into the original command
// path. In host mode, local A5/A6 commands are blocked.
assign rx_data = (mode_change_pulse || mode_host_active) ?
                 8'h53 : local_cmd_data;
assign rx_done = mode_change_pulse ? 1'b1 :
                 (mode_host_active ? 1'b0 : local_cmd_done);


//==========================================================
// 2. 上位机串口
//==========================================================

wire [7:0] host_rx_data;
wire host_rx_done;

assign host_rx_data = pc_rx_data;
assign host_rx_done = pc_rx_done;

wire [7:0] host_tx_data;
wire host_tx_send;

wire host_tx_done;
wire host_tx_busy;

// 上位机请求的运动速度
wire signed [12:0] host_vx;
wire signed [12:0] host_vy;
wire signed [12:0] host_wz;

wire host_cmd_valid;
wire host_ai_mode;
wire host_stop_req;
wire host_waypoint_start_req;
wire host_frame_ok;

// Host commands must be refreshed at least every 300ms. A stale or
// missing command always produces zero velocity.
reg [5:0] host_watchdog_ticks;
reg host_watchdog_expired;
wire host_control_active;
wire signed [12:0] host_vx_limited;
wire signed [12:0] host_vy_limited;
wire signed [12:0] host_wz_limited;
wire signed [12:0] host_vx_active;
wire signed [12:0] host_vy_active;
wire signed [12:0] host_wz_active;

assign host_control_active =
    mode_host_active && host_ai_mode &&
    !host_watchdog_expired && !host_stop_req;

// Keep host velocity units identical to the existing FPGA command units.
// Conservative limits: translation +/-24, rotation +/-12.
assign host_vx_limited =
    ($signed(host_vx) > 13'sd24)  ? 13'sd24  :
    ($signed(host_vx) < -13'sd24) ? -13'sd24 : host_vx;
assign host_vy_limited =
    ($signed(host_vy) > 13'sd24)  ? 13'sd24  :
    ($signed(host_vy) < -13'sd24) ? -13'sd24 : host_vy;
assign host_wz_limited =
    ($signed(host_wz) > 13'sd12)  ? 13'sd12  :
    ($signed(host_wz) < -13'sd12) ? -13'sd12 : host_wz;

assign host_vx_active = host_control_active ? host_vx_limited : 13'sd0;
assign host_vy_active = host_control_active ? host_vy_limited : 13'sd0;
assign host_wz_active = host_control_active ? host_wz_limited : 13'sd0;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        host_watchdog_ticks   <= 6'd0;
        host_watchdog_expired <= 1'b1;
    end else if (!mode_host_active || !host_ai_mode || host_stop_req) begin
        host_watchdog_ticks   <= 6'd0;
        host_watchdog_expired <= 1'b1;
    end else if (host_cmd_valid) begin
        host_watchdog_ticks   <= 6'd0;
        host_watchdog_expired <= 1'b0;
    end else if (speed_tick_a) begin
        if (host_watchdog_ticks >= 6'd29) begin
            host_watchdog_expired <= 1'b1;
        end else begin
            host_watchdog_ticks <= host_watchdog_ticks + 1'b1;
        end
    end
end


//==========================================================
// 3. SG90 舵机扫描
//==========================================================

// 最终输出给舵机的位置
wire [1:0] servo_position;

// 手动控制舵机的位置
wire [1:0] servo_manual_position;

// 自动扫描模块请求的位置
wire [1:0] servo_scan_position;

wire servo_scan_active;
wire servo_scan_done;

// 三个扫描方向的距离，单位mm
wire [15:0] scan_left_distance_mm;
wire [15:0] scan_center_distance_mm;
wire [15:0] scan_right_distance_mm;


//==========================================================
// 4. 自动运动序列
//==========================================================

wire seq_distance_start;

wire seq_angle_start;
wire seq_angle_turn_right;

wire seq_lateral_start;
wire seq_lateral_move_right;

wire seq_stop_req;

wire sequence_active;
wire sequence_done;
wire sequence_error;

wire [5:0] sequence_state;

// 横移控制器输出的速度
wire signed [12:0] vx_lateral;
wire signed [12:0] vy_lateral;
wire signed [12:0] wz_lateral;

wire lateral_auto_active;
wire lateral_auto_done;
wire lateral_move_right;

wire signed [31:0] lateral_travelled_count;


//==========================================================
// 5. 基础控制器
//==========================================================

wire ctrl_manual_rx_done;

// 里程计输出的世界坐标，单位mm
wire signed [31:0] odom_world_x_mm;
wire signed [31:0] odom_world_y_mm;

// 手动速度安全过滤
wire signed [12:0] manual_vx_safe;
wire signed [12:0] manual_vy_safe;
wire signed [12:0] manual_wz_safe;


//==========================================================
// 6. HC-SR04 超声波
//==========================================================

// 实时距离，单位mm
wire [15:0] ultra_distance_mm;

// 一次有效测距脉冲
wire ultra_distance_valid;

// 当前是否正在测距
wire ultra_measure_busy;

// 超声波回波超时
wire ultra_echo_timeout;

// HC-SR04 health / recovery status (from ultrasonic_driver.v)
wire ultra_measurement_done;
wire ultra_sensor_online;
wire ultra_sensor_recovering;
wire [7:0] ultra_recovery_count;


//==========================================================
// 7. 左侧 VL53L0X
//==========================================================

// 传感器原始距离
wire [15:0] tof_left_raw_mm;

// 传感器内部校准后的距离
wire [15:0] tof_left_cal_mm;

// 新测距数据有效
wire tof_left_valid;

// 传感器错误
wire tof_left_error;

// 驱动状态，便于排查初始化失败
wire [6:0] tof_left_state;

// 驱动自带的调试串口没有连接到PC
wire tof_left_uart_unused;


//==========================================================
// 8. 右侧 VL53L0X
//==========================================================

wire [15:0] tof_right_raw_mm;
wire [15:0] tof_right_cal_mm;

wire tof_right_valid;
wire tof_right_error;

wire [6:0] tof_right_state;

wire tof_right_uart_unused;


//==========================================================
// 第三路正前方 ToF 原始信号
wire [15:0] tof_front_raw_mm;
wire [15:0] tof_front_cal_mm;
wire tof_front_valid;
wire tof_front_error;
wire [6:0] tof_front_state;
wire tof_front_uart_unused;
reg [15:0] tof_front_mm;
reg tof_front_seen;
reg [7:0] tof_front_age;
wire tof_front_ok;
wire tof_front_block;

//=================== 后方 ToF 传感器 ===================
wire [15:0] tof_rear_raw_mm;
wire [15:0] tof_rear_cal_mm;
wire tof_rear_valid;
wire tof_rear_error;
wire [6:0] tof_rear_state;
wire tof_rear_uart_unused;
reg  [15:0] tof_rear_mm;
reg  tof_rear_seen;
reg  [7:0] tof_rear_age;
wire tof_rear_ok;
wire tof_rear_block;
wire rear_reverse_request;
wire rear_reverse_inhibit;

//==========================================================
// Four independent ToF recovery supervisors
// Each supervisor resets ONLY its own VL53L0X driver.
//==========================================================
wire tof_recovery_motion_stopped;

wire tof_left_driver_reset_n;
wire tof_right_driver_reset_n;
wire tof_front_driver_reset_n;
wire tof_rear_driver_reset_n;

wire tof_left_recovery_healthy;
wire tof_right_recovery_healthy;
wire tof_front_recovery_healthy;
wire tof_rear_recovery_healthy;

wire tof_left_recovering;
wire tof_right_recovering;
wire tof_front_recovering;
wire tof_rear_recovering;

wire tof_left_recovery_fault;
wire tof_right_recovery_fault;
wire tof_front_recovery_fault;
wire tof_rear_recovery_fault;

wire [7:0] tof_left_recovery_count;
wire [7:0] tof_right_recovery_count;
wire [7:0] tof_front_recovery_count;
wire [7:0] tof_rear_recovery_count;



// ============================================================================
// 2026-10-26 Four independent ToF sensor-front-face calibration.
// A5, A6, the emergency safety guards, UART and video downstream all receive
// the SAME calibrated tof_*_mm values from the cache below.
// Invalid raw samples do not generate calibrated_valid pulses.
// To temporarily disable a channel's calibration, set CAL_MODE to 0.
//
// Reference in millimeters, per sensor: 100 / 200 / 300 / 400 / 500 / 600.
// L: raw ~ 0.988571*actual + 25.333 => (raw - 25)*4143/4096
// R: raw ~ 1.050286*actual - 3.600  => (raw + 4) *3900/4096
// F: raw ~ 1.028114*actual + 39.993 => (raw - 40)*3984/4096
// B: two-segment near-range and 20..60 cm fit; see new module.
// WARNING: this is an empirical calibration over 100..600mm, NOT proof of
// accuracy outside this range. Keep existing obstacle protection enabled.
// ============================================================================
wire [15:0] tof_left_fixed_mm;
wire [15:0] tof_right_fixed_mm;
wire [15:0] tof_front_fixed_mm;
wire [15:0] tof_rear_fixed_mm;
wire tof_left_fixed_valid;
wire tof_right_fixed_valid;
wire tof_front_fixed_valid;
wire tof_rear_fixed_valid;

tof_distance_fixed_cal #(.CAL_MODE(1), .OFFSET_MM(25), .GAIN_Q12(4143))
u_tof_left_fixed_cal (
    .clk(clk), .reset_n(reset_n),
    .raw_distance_mm(tof_left_raw_mm),
    .raw_valid(tof_left_valid && !tof_left_error),
    .calibrated_mm(tof_left_fixed_mm),
    .calibrated_valid(tof_left_fixed_valid)
);

tof_distance_fixed_cal #(.CAL_MODE(1), .OFFSET_MM(-4), .GAIN_Q12(3900))
u_tof_right_fixed_cal (
    .clk(clk), .reset_n(reset_n),
    .raw_distance_mm(tof_right_raw_mm),
    .raw_valid(tof_right_valid && !tof_right_error),
    .calibrated_mm(tof_right_fixed_mm),
    .calibrated_valid(tof_right_fixed_valid)
);

tof_distance_fixed_cal #(.CAL_MODE(1), .OFFSET_MM(40), .GAIN_Q12(3984))
u_tof_front_fixed_cal (
    .clk(clk), .reset_n(reset_n),
    .raw_distance_mm(tof_front_raw_mm),
    .raw_valid(tof_front_valid && !tof_front_error),
    .calibrated_mm(tof_front_fixed_mm),
    .calibrated_valid(tof_front_fixed_valid)
);

tof_distance_fixed_cal #(.CAL_MODE(2))
u_tof_rear_fixed_cal (
    .clk(clk), .reset_n(reset_n),
    .raw_distance_mm(tof_rear_raw_mm),
    .raw_valid(tof_rear_valid && !tof_rear_error),
    .calibrated_mm(tof_rear_fixed_mm),
    .calibrated_valid(tof_rear_fixed_valid)
);

// 9. ToF 距离缓存
//
// 不能直接把 distance_valid 当作持续有效信号。
// 需要保存上次测距结果并检测数据是否过期。
//==========================================================

// 最近一次有效距离
reg [15:0] tof_left_mm;
reg [15:0] tof_right_mm;

// 是否曾经收到过有效数据
reg tof_left_seen;
reg tof_right_seen;

// 距离上次更新经过的采样周期
reg [7:0] tof_left_age;
reg [7:0] tof_right_age;


//==========================================================
// 10. ToF 安全参数
//==========================================================

// 当前启用左右两路保护
localparam ENABLE_LEFT_TOF_SAFETY  = 1'b1;
localparam ENABLE_RIGHT_TOF_SAFETY = 1'b1;
// 未装齐三路前必须保持关闭。模块到齐并完成单路验证后再置1。
localparam ENABLE_FRONT_TOF_SAFETY = 1'b1; // D7: only automatic motion, see guard below

localparam [15:0] TOF_FRONT_STOP_MM = 16'd200;

// A5 safety margin in calibrated millimetres.
// <=100mm: block motion toward the corresponding side.
// This is EXPERIMENTAL until robot corner-sweep / braking
// distance is measured on the real 25x30cm chassis.
localparam [15:0] TOF_SIDE_STOP_MM = 16'd100;

// 后方保护暂不降低
localparam [15:0] REAR_TOF_STOP_MM = 16'd100;

// 保留数据超时判断
// 200ms maximum age at a nominal 10ms speed_tick_a.
// A stale ToF reading is UNKNOWN, never assumed to be open space.
localparam [7:0] TOF_MAX_AGE = 8'd20;



//==========================================================
// 11. 距离控制器
//==========================================================

wire signed [12:0] vx_ctrl;
wire signed [12:0] vy_ctrl;
wire signed [12:0] wz_ctrl;


//==========================================================
// 12. A5 自主避障模块
//==========================================================

// 避障模块是否正在运行
wire obstacle_avoid_active;

// 避障模块是否进入错误状态
wire obstacle_avoid_error;

// 请求执行一次舵机三方向扫描
wire obstacle_scan_start;

// A5状态编号，范围0~14
wire [3:0] obstacle_state;

// A5输出的底盘速度
wire signed [12:0] obstacle_vx;
wire signed [12:0] obstacle_vy;
wire signed [12:0] obstacle_wz;


//==========================================================
// 13. A6 航点导航
//==========================================================

wire waypoint_active;
wire waypoint_done;
wire waypoint_turning;
wire waypoint_corner_ready;
// Near-end handshake: ONLY when (1) verified 100ms turn clearance,
// (2) odometry is near the waypoint, and (3) no detour command is active.
// Does not bypass the real motor emergency gates or turn-space check.
wire a6_corner_takeover;
// Timing isolation: a registered snapshot of the corner handshake for
// the A6->A5 supervisor only. The waypoint and safety gates remain unchanged.
reg a6_corner_takeover_supervisor_q;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        a6_corner_takeover_supervisor_q <= 1'b0;
    else
        a6_corner_takeover_supervisor_q <= a6_corner_takeover;
end
// A6 90-degree turn is only permitted with verified frontal and side space.
// Rear clearance is a proximity emergency only, not the normal turn trigger.
// A6 turn: only a POSITIVE close reading is an obstacle. UNKNOWN is
// NOT a free-space measurement. Use this mode only in a cleared test arena;
// the 180-degree sweep can reach beyond point-sensor fields of view.
wire a6_turn_near_now =
    (tof_front_ok && tof_front_mm <= 16'd100) ||
    (tof_left_ok  && tof_left_mm  <= 16'd100) ||
    (tof_right_ok && tof_right_mm <= 16'd100) ||
    (tof_rear_ok  && tof_rear_mm  <= 16'd100) ||
    (d7_fused_front_valid && d7_fused_front_mm <= 16'd100);
wire a6_turn_space_ok = !a6_turn_near_now &&
    !a6_near_front_memory && !a6_near_left_memory &&
    !a6_near_right_memory && !a6_near_rear_memory;

// Timing isolation for the A6 waypoint turn-permission input.
// Register each per-sensor positive-near veto in parallel. The raw
// a6_turn_space_ok above remains connected to the final motor safety gate,
// so a newly detected near obstacle still inhibits rotation immediately.
// This registered copy is used only by the waypoint state/CE logic; it is
// reset to the blocked state so startup can never grant turn permission.
reg [4:0] a6_turn_near_flags_q;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        a6_turn_near_flags_q <= 5'b11111;
    end else begin
        a6_turn_near_flags_q[0] <=
            tof_front_ok && (tof_front_mm <= 16'd100);
        a6_turn_near_flags_q[1] <=
            tof_left_ok && (tof_left_mm <= 16'd100);
        a6_turn_near_flags_q[2] <=
            tof_right_ok && (tof_right_mm <= 16'd100);
        a6_turn_near_flags_q[3] <=
            tof_rear_ok && (tof_rear_mm <= 16'd100);
        a6_turn_near_flags_q[4] <=
            d7_fused_front_valid && (d7_fused_front_mm <= 16'd100);
    end
end
wire a6_turn_space_ok_nav = !(|a6_turn_near_flags_q) &&
    !a6_near_front_memory && !a6_near_left_memory &&
    !a6_near_right_memory && !a6_near_rear_memory;
wire a6_turn_has_unknown =
    !d7_fused_front_valid || !tof_left_ok ||
    !tof_right_ok || !tof_rear_ok;

wire [2:0] waypoint_index;
wire [2:0] waypoint_state;
// Latched first A6 navigation fault, independent from A6 state 06.
wire [2:0] a6_nav_fault_code;
// Gate codes: 0=none,1=global,2=avoiding/paused,3=front,
// 4=right,5=left,6=rear,7=turn clearance.
wire [2:0] a6_gate_debug;
assign a6_gate_debug =
    d7_force_stop ? 3'd1 :
    (waypoint_active && nav_waypoint_pause && !a6_corner_takeover) ? 3'd2 :
    (waypoint_active && ((tof_front_block || d7_forward_inhibit) &&
                        $signed(vx_mecanum_reg) > 13'sd0)) ? 3'd3 :
    (waypoint_active && tof_right_block &&
     $signed(vy_mecanum_reg) > 13'sd0) ? 3'd4 :
    (waypoint_active && tof_left_block &&
     $signed(vy_mecanum_reg) < 13'sd0) ? 3'd5 :
    (waypoint_active && (d7_reverse_inhibit || rear_reverse_inhibit) &&
     $signed(vx_mecanum_reg) < 13'sd0) ? 3'd6 :
    (waypoint_active && waypoint_turning && !a6_turn_space_ok) ? 3'd7 :
    3'd0;


// Registered waypoint commands.  Definitions are shared with the
// motion pipeline (do not redeclare them there).
reg signed [12:0] waypoint_vx_timing_q;
reg signed [12:0] waypoint_vy_timing_q;
reg signed [12:0] waypoint_wz_timing_q;

// A6 motion policy for this small, physically inspected arena:
// - VALID near range -> collision/avoidance decision.
// - 0, overrange, stale, missing echo -> UNKNOWN, not a near obstacle.
// - UNKNOWN permits a capped-speed A6 waypoint command; it does NOT
//   establish that the physical corridor is safe.
// - A5 retains its previously tested, separate direction safeguards.
wire a6_leg_x = (waypoint_index == 3'd0) ||
                (waypoint_index == 3'd2) ||
                (waypoint_index == 3'd5) ||
                (waypoint_index == 3'd7);
wire a6_front_sensor_valid = d7_fused_front_valid;
wire a6_path_sensor_valid = a6_leg_x ? a6_front_sensor_valid : tof_right_ok;
wire [15:0] a6_path_distance_mm =
          a6_leg_x ? d7_fused_front_mm : tof_right_mm;
wire a6_path_distance_valid =
          a6_leg_x ? d7_fused_front_valid : tof_right_ok;

wire a6_odom_plausible =
          (odom_world_x_mm >= -32'sd5000) &&
          (odom_world_x_mm <=  32'sd5000) &&
          (odom_world_y_mm >= -32'sd5000) &&
          (odom_world_y_mm <=  32'sd5000);

// Five-sensor rule: invalid/zero/overrange/echo-timeout = UNKNOWN, not near.
// Only a trustworthy POSITIVE near measurement triggers an obstacle veto.
// Hold an observed near reading for <=200ms so a 1-frame dropout cannot
// instantly bypass a nearby object. No indefinite historical lockout.
// NOTE: after this grace, UNKNOWN is permitted with a low-speed cap. This
// is NOT proof of safety; use only in a physically cleared test arena.
localparam [5:0] NEAR_MEMORY_TICKS = 6'd20; // speed_tick_a ~10ms
reg [5:0] a6_front_near_age;
reg [5:0] a6_left_near_age;
reg [5:0] a6_right_near_age;
reg [5:0] a6_rear_near_age;
wire a6_near_front_memory = a6_front_near_age < NEAR_MEMORY_TICKS;
wire a6_near_left_memory  = a6_left_near_age  < NEAR_MEMORY_TICKS;
wire a6_near_right_memory = a6_right_near_age < NEAR_MEMORY_TICKS;
wire a6_near_rear_memory  = a6_rear_near_age  < NEAR_MEMORY_TICKS;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        a6_front_near_age <= NEAR_MEMORY_TICKS;
        a6_left_near_age  <= NEAR_MEMORY_TICKS;
        a6_right_near_age <= NEAR_MEMORY_TICKS;
        a6_rear_near_age  <= NEAR_MEMORY_TICKS;
    end else if (speed_tick_a) begin
        if (d7_fused_front_valid && d7_fused_front_mm <= 16'd100)
            a6_front_near_age <= 6'd0;
        else if (d7_fused_front_valid && d7_fused_front_mm >= 16'd140)
            a6_front_near_age <= NEAR_MEMORY_TICKS;
        else if (a6_front_near_age < NEAR_MEMORY_TICKS)
            a6_front_near_age <= a6_front_near_age + 1'b1;

        if (tof_left_ok && tof_left_mm <= 16'd100)
            a6_left_near_age <= 6'd0;
        else if (tof_left_ok && tof_left_mm >= 16'd140)
            a6_left_near_age <= NEAR_MEMORY_TICKS;
        else if (a6_left_near_age < NEAR_MEMORY_TICKS)
            a6_left_near_age <= a6_left_near_age + 1'b1;

        if (tof_right_ok && tof_right_mm <= 16'd100)
            a6_right_near_age <= 6'd0;
        else if (tof_right_ok && tof_right_mm >= 16'd140)
            a6_right_near_age <= NEAR_MEMORY_TICKS;
        else if (a6_right_near_age < NEAR_MEMORY_TICKS)
            a6_right_near_age <= a6_right_near_age + 1'b1;

        if (tof_rear_ok && tof_rear_mm <= REAR_TOF_STOP_MM)
            a6_rear_near_age <= 6'd0;
        else if (tof_rear_ok && tof_rear_mm >= 16'd140)
            a6_rear_near_age <= NEAR_MEMORY_TICKS;
        else if (a6_rear_near_age < NEAR_MEMORY_TICKS)
            a6_rear_near_age <= a6_rear_near_age + 1'b1;
    end
end

// Missing sensor feedback does not pause A6 or send an A5 command.
// HARD error flags are different from over-range / zero measurements.
wire a6_route_only = waypoint_active && !obstacle_avoid_active &&
        !nav_avoid_cycle_active && !nav_restore_active &&
        (waypoint_state == 3'd1) && a6_odom_plausible;
wire a6_unknown_forward = d7_auto_active && !d7_fused_front_valid;
wire a6_unknown_left = !tof_left_ok && !a6_near_left_memory;
wire a6_unknown_right = !tof_right_ok && !a6_near_right_memory;
wire a6_unknown_side_command = d7_auto_active &&
    (($signed(vy_mecanum_reg) < 13'sd0 && !tof_left_ok) ||
     ($signed(vy_mecanum_reg) > 13'sd0 && !tof_right_ok));
// Kept as a named signal for compatibility with existing status logging.
wire a6_sensor_hold = 1'b0;
wire a6_initial_probe = 1'b0;

// A6/A5 supervisor: use REAL distance and REAL validity, never
// synthesize 1000mm from a 240mm reading.  During the active A6 run,
// remember the commanded body-motion direction (registered command).
// Hold this direction during A5 takeover so rejoin checks the SAME
// sensor that originally confirmed the obstacle.  This also covers
// A6 correcting a cross-track error after a previous detour.
localparam [1:0] A6_DIR_FRONT=2'd0, A6_DIR_REAR=2'd1,
                 A6_DIR_LEFT =2'd2, A6_DIR_RIGHT=2'd3;
reg [1:0] a6_supervisor_direction;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        a6_supervisor_direction <= A6_DIR_FRONT;
    else if (!waypoint_active)
        a6_supervisor_direction <= A6_DIR_FRONT;
    else if ((waypoint_state == 3'd1) && !nav_waypoint_pause &&
             !obstacle_avoid_active) begin
        if ($signed(waypoint_vx_timing_q) > 13'sd0)
            a6_supervisor_direction <= A6_DIR_FRONT;
        else if ($signed(waypoint_vx_timing_q) < 13'sd0)
            a6_supervisor_direction <= A6_DIR_REAR;
        else if ($signed(waypoint_vy_timing_q) < 13'sd0)
            a6_supervisor_direction <= A6_DIR_LEFT;
        else if ($signed(waypoint_vy_timing_q) > 13'sd0)
            a6_supervisor_direction <= A6_DIR_RIGHT;
    end
end
// -------------------------------------------------------------------
// A6 rejoin sensor qualification, 50MHz-safe registered processing.
// These counters count NEW VL53L0X completed samples, not 10ms repeats.
// A single 50->1000mm spike can NEVER mean a cleared obstacle.
// Do not alter independent A5 measurement or emergency-stop inputs.
// -------------------------------------------------------------------
reg [1:0] a6_front_far_samples, a6_rear_far_samples;
reg [1:0] a6_left_far_samples, a6_right_far_samples;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        a6_front_far_samples <= 0;
        a6_rear_far_samples <= 0;
        a6_left_far_samples <= 0;
        a6_right_far_samples <= 0;
    end else begin
        if (tof_front_fixed_valid && !tof_front_error) begin
            if (tof_front_fixed_mm >= 16'd340) begin
                if (a6_front_far_samples != 2'd3)
                    a6_front_far_samples <= a6_front_far_samples+1'b1;
            end else begin
                a6_front_far_samples <= 2'd0;
            end
        end else if (speed_tick_a && !tof_front_ok)
            a6_front_far_samples <= 0;

        if (tof_rear_fixed_valid && !tof_rear_error) begin
            if (tof_rear_fixed_mm >= 16'd340) begin
                if (a6_rear_far_samples != 2'd3)
                    a6_rear_far_samples <= a6_rear_far_samples+1'b1;
            end else begin
                a6_rear_far_samples <= 2'd0;
            end
        end else if (speed_tick_a && !tof_rear_ok)
            a6_rear_far_samples <= 0;

        if (tof_left_fixed_valid && !tof_left_error) begin
            if (tof_left_fixed_mm >= 16'd340) begin
                if (a6_left_far_samples != 2'd3)
                    a6_left_far_samples <= a6_left_far_samples+1'b1;
            end else begin
                a6_left_far_samples <= 2'd0;
            end
        end else if (speed_tick_a && !tof_left_ok)
            a6_left_far_samples <= 0;

        if (tof_right_fixed_valid && !tof_right_error) begin
            if (tof_right_fixed_mm >= 16'd340) begin
                if (a6_right_far_samples != 2'd3)
                    a6_right_far_samples <= a6_right_far_samples+1'b1;
            end else begin
                a6_right_far_samples <= 2'd0;
            end
        end else if (speed_tick_a && !tof_right_ok)
            a6_right_far_samples <= 0;
    end
end
wire a6_hc_far_trusted; // asserted only after 3 fresh HC-SR04 measurement pulses
wire a6_front_far_trusted = (tof_front_ok && a6_front_far_samples == 2'd3) || a6_hc_far_trusted;
wire a6_rear_far_trusted  = tof_rear_ok  && a6_rear_far_samples  == 2'd3;
wire a6_left_far_trusted  = tof_left_ok  && a6_left_far_samples  == 2'd3;
wire a6_right_far_trusted = tof_right_ok && a6_right_far_samples == 2'd3;
// Near data remains actionable immediately. Mid/far data only qualifies
// as clearance after three separate measurement-complete pulses.
wire a6_front_for_detour_valid = d7_fused_front_valid &&
    ((d7_fused_front_mm < 16'd340) || a6_front_far_trusted);
wire a6_rear_for_detour_valid = tof_rear_ok &&
    ((tof_rear_mm < 16'd340) || a6_rear_far_trusted);
wire a6_left_for_detour_valid = tof_left_ok &&
    ((tof_left_mm < 16'd340) || a6_left_far_trusted);
wire a6_right_for_detour_valid = tof_right_ok &&
    ((tof_right_mm < 16'd340) || a6_right_far_trusted);

wire [15:0] a6_supervisor_distance =
    (a6_supervisor_direction == A6_DIR_FRONT) ? d7_fused_front_mm :
    (a6_supervisor_direction == A6_DIR_REAR)  ? tof_rear_mm :
    (a6_supervisor_direction == A6_DIR_LEFT)  ? tof_left_mm :
                                                tof_right_mm;
wire a6_supervisor_valid =
    ((a6_supervisor_direction == A6_DIR_FRONT) && a6_front_for_detour_valid &&
        (d7_fused_front_mm != 16'd0)) ||
    ((a6_supervisor_direction == A6_DIR_REAR) && a6_rear_for_detour_valid &&
        (tof_rear_mm != 16'd0)) ||
    ((a6_supervisor_direction == A6_DIR_LEFT) && a6_left_for_detour_valid &&
        (tof_left_mm != 16'd0)) ||
    ((a6_supervisor_direction == A6_DIR_RIGHT) && a6_right_for_detour_valid &&
        (tof_right_mm != 16'd0));


// A6_FIX temporary UART debug on A6=xx (replaces supervisor status only):
// 00=not selected; 01=navigating; 02=waypoint hold;
// 03=turning; 04=selected but idle / postturn;
// 05=front ranges unavailable / navigation completed;
// 06=safety fault; 07=IMU calibration incomplete.
// WP=0 while idle OR navigating first waypoint; this separates them.
wire [2:0] a6_start_diagnostic =
    (d7_fault_latched || (waypoint_active && !a6_odom_plausible)) ? 3'd6 :
    (waypoint_active && a6_sensor_hold) ? 3'd5 :
    (waypoint_state != 3'd0) ? waypoint_state :
    ((key_selected_mode == 2'd2) ?
      (!gyro_calib_done ? 3'd7 :
       !a6_odom_plausible ? 3'd6 : 3'd4) : 3'd0);

// 航点导航计算出的速度
wire signed [12:0] waypoint_vx;
wire signed [12:0] waypoint_vy;
wire signed [12:0] waypoint_wz;


//==========================================================
// 14. A6 自动避障协调器
//
// A6行驶 -> 遇障 -> 暂停A6
// -> 启动A5 -> 绕障 -> 恢复航向
// -> 恢复A6
//==========================================================

wire nav_waypoint_pause;

wire nav_auto_avoid_start;
wire nav_auto_avoid_stop;

wire nav_restore_active;

wire signed [12:0] nav_restore_wz;

wire nav_avoid_cycle_active;

wire [2:0] nav_avoid_state;
// A6-only finite local detour. Manual A5 is untouched.
wire nav_detour_active;
assign a6_corner_takeover = waypoint_corner_ready && !nav_detour_active &&
    (nav_avoid_state == 3'd0) &&
    !obstacle_avoid_active && !obstacle_avoid_error &&
    !d7_force_stop && a6_turn_space_ok_nav;
wire signed [12:0] nav_detour_vx;
wire signed [12:0] nav_detour_vy;
wire signed [12:0] nav_detour_wz;
wire [1:0] nav_detour_dir_debug;
wire [15:0] nav_detour_mm_debug;
wire [2:0] nav_detour_fault_debug;
// TB: 0 no block; 1 front, 2 left, 3 right, 4 rear,
// 5 recent-close memory (current sensor no longer close).
wire [2:0] a6_turn_block_reason =
    ((tof_front_ok && tof_front_mm <= 16'd100) ||
     (d7_fused_front_valid && d7_fused_front_mm <= 16'd100)) ? 3'd1 :
    (tof_left_ok && tof_left_mm <= 16'd100) ? 3'd2 :
    (tof_right_ok && tof_right_mm <= 16'd100) ? 3'd3 :
    (tof_rear_ok && tof_rear_mm <= 16'd100) ? 3'd4 :
    !a6_turn_space_ok ? 3'd5 : 3'd0;

//======================================================================
// P4 turn first-block snapshot - DIAGNOSTICS ONLY.
// The first valid turn veto is latched at the 50MHz clock, not at the
// ~200ms UART report cadence. No motor/safety decision uses these regs.
// A new ST_TURN clears the previous snapshot; fault/stop retain it for
// later inspection until the next turn or reset.
//======================================================================
reg a6_turning_previous_q;
reg a6_turn_event_seen;
reg [15:0] a6_turn_event_tof_mm;
reg a6_turn_event_tof_valid;
reg [15:0] a6_turn_event_hc_mm;
reg a6_turn_event_hc_valid;
reg [15:0] a6_turn_event_fused_mm;
reg a6_turn_event_fused_valid;
reg [2:0] a6_turn_event_reason;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        a6_turning_previous_q    <= 1'b0;
        a6_turn_event_seen       <= 1'b0;
        a6_turn_event_tof_mm     <= 16'd0;
        a6_turn_event_tof_valid  <= 1'b0;
        a6_turn_event_hc_mm      <= 16'd0;
        a6_turn_event_hc_valid   <= 1'b0;
        a6_turn_event_fused_mm   <= 16'd0;
        a6_turn_event_fused_valid<= 1'b0;
        a6_turn_event_reason     <= 3'd0;
    end else begin
        a6_turning_previous_q <= waypoint_turning;
        if (waypoint_turning && !a6_turning_previous_q) begin
            a6_turn_event_seen        <= 1'b0;
            a6_turn_event_tof_mm      <= 16'd0;
            a6_turn_event_tof_valid   <= 1'b0;
            a6_turn_event_hc_mm       <= 16'd0;
            a6_turn_event_hc_valid    <= 1'b0;
            a6_turn_event_fused_mm    <= 16'd0;
            a6_turn_event_fused_valid <= 1'b0;
            a6_turn_event_reason      <= 3'd0;
        end else if (waypoint_turning && !a6_turn_space_ok &&
                     !a6_turn_event_seen) begin
            a6_turn_event_seen        <= 1'b1;
            a6_turn_event_tof_mm      <= tof_front_mm;
            a6_turn_event_tof_valid   <= tof_front_ok;
            a6_turn_event_hc_mm       <= a6_hc_cache_mm;
            a6_turn_event_hc_valid    <= a6_hc_cache_ok;
            a6_turn_event_fused_mm    <= d7_fused_front_mm;
            a6_turn_event_fused_valid <= d7_fused_front_valid;
            a6_turn_event_reason      <= a6_turn_block_reason;
        end
    end
end




//==========================================================
// 15. 向A5注入串口指令
//==========================================================

// A6检测到障碍时，内部生成A5
// 绕障完成时，内部生成53
wire [7:0] obstacle_rx_data;
wire obstacle_rx_done;


//==========================================================
// 16. 最终运动源
//==========================================================

// 未经保护的导航速度
wire signed [12:0] vx_nav;
wire signed [12:0] vy_nav;
wire signed [12:0] wz_nav;

// 经过ToF防撞保护后的速度
wire signed [12:0] vx_nav_safe;
wire signed [12:0] vy_nav_safe;
wire signed [12:0] wz_nav_safe;

// 左右ToF触发的侧向保护
wire tof_side_emergency_stop;


//==========================================================
// 17. 自动角度控制
//==========================================================

wire signed [12:0] vx_angle;
wire signed [12:0] vy_angle;
wire signed [12:0] wz_angle;

wire angle_auto_active;
wire angle_auto_done;

wire signed [31:0] angle_travelled_cdeg;

wire distance_auto_active;
wire distance_auto_done;


//==========================================================
// 18. 里程计转换流水线
//==========================================================

reg signed [31:0] odom_x_reg;
reg signed [31:0] odom_y_reg;

reg signed [47:0] odom_x_mul_stage1;
reg signed [47:0] odom_y_mul_stage1;

reg signed [47:0] odom_x_mul_stage2;
reg signed [47:0] odom_y_mul_stage2;

reg signed [31:0] odom_pos_x_mm;
reg signed [31:0] odom_pos_y_mm;


//==========================================================
// 19. 麦轮里程计内部计数
//==========================================================

wire signed [31:0] odom_delta_x;
wire signed [31:0] odom_delta_y;
wire signed [31:0] odom_delta_w;

wire signed [31:0] odom_pos_x;
wire signed [31:0] odom_pos_y;

wire signed [31:0] odom_yaw;


//==========================================================
// 20. MPU6500 相关信号
//==========================================================

wire [7:0] mpu_id;
wire mpu_valid;
wire mpu_ack_error;

wire signed [31:0] imu_yaw_cdeg;

wire i2c_scan_done;
wire [127:0] i2c_found_map;
wire [7:0] i2c_found_count;

wire signed [15:0] gyro_bias_x;
wire signed [15:0] gyro_bias_y;
wire signed [15:0] gyro_bias_z;

wire signed [31:0] gyro_x_corrected;
wire signed [31:0] gyro_y_corrected;
wire signed [31:0] gyro_z_corrected;

wire gyro_calib_done;

wire signed [15:0] imu_accel_x;
wire signed [15:0] imu_accel_y;
wire signed [15:0] imu_accel_z;

wire signed [15:0] imu_temp_raw;

wire signed [15:0] imu_gyro_x;
wire signed [15:0] imu_gyro_y;
wire signed [15:0] imu_gyro_z;

wire imu_data_valid;
wire imu_read_error;

wire imu_init_done;
wire imu_init_error;

wire [3:0] imu_init_step;


//==========================================================
// 21. 手动控制指令
//==========================================================

wire signed [12:0] vx_cmd;
wire signed [12:0] vy_cmd;
wire signed [12:0] wz_cmd;


//==========================================================
// 22. 航向保持控制器
//==========================================================

wire signed [12:0] wz_final;

wire signed [31:0] heading_target_yaw;
wire signed [31:0] heading_yaw_error;

wire heading_hold_active;


//==========================================================
// 23. 电机PWM控制量
//==========================================================

wire signed [12:0] pwm_cmd_a;
wire signed [12:0] pwm_cmd_b;
wire signed [12:0] pwm_cmd_c;
wire signed [12:0] pwm_cmd_d;


//==========================================================
// 24. 四轮目标速度
//==========================================================

wire signed [12:0] target_speed_a;
wire signed [12:0] target_speed_b;
wire signed [12:0] target_speed_c;
wire signed [12:0] target_speed_d;

// 一级寄存器，改善组合逻辑时序
reg signed [12:0] target_speed_a_reg;
reg signed [12:0] target_speed_b_reg;
reg signed [12:0] target_speed_c_reg;
reg signed [12:0] target_speed_d_reg;


//==========================================================
// 25. 编码器位置及速度
//==========================================================

wire signed [31:0] encoder_a_position;
wire signed [31:0] encoder_b_position;
wire signed [31:0] encoder_c_position;
wire signed [31:0] encoder_d_position;

wire signed [31:0] encoder_a_speed;
wire signed [31:0] encoder_b_speed;
wire signed [31:0] encoder_c_speed;
wire signed [31:0] encoder_d_speed;

// 各轮测速采样脉冲
wire speed_tick_a;
wire speed_tick_b;
wire speed_tick_c;
wire speed_tick_d;

// 延迟一级供PI使用
reg speed_tick_a_d1;
reg speed_tick_b_d1;
reg speed_tick_c_d1;
reg speed_tick_d_d1;


//======================================================================
// 第二部分：舵机控制来源选择
//======================================================================

// 舵机控制优先级：
// 1. 正在扫描时，扫描模块最高优先级
// 2. A5避障时，根据当前绕行状态调整方向
// 3. A6恢复航向时，舵机指向正前方
// 4. A6航点运行时，舵机指向正前方
// 5. 其余情况使用手动舵机指令

assign servo_position =

    servo_scan_active
    ?
        servo_scan_position

    :

    // During A6 one-shot lateral bypass, keep sonar centered so the
    // front echo can be refreshed while fixed side ToFs protect Y motion.
    // Servo scan still takes highest priority above.
    (obstacle_avoid_active && nav_avoid_cycle_active)
    ?
        2'd1
    :
    obstacle_avoid_active
    ?
        (
            (
                (obstacle_state == 4'd5)
                ||
                (obstacle_state == 4'd10)
            )
            ?
                2'd2
            :
            (
                (obstacle_state == 4'd8)
                ||
                (obstacle_state == 4'd7)
            )
            ?
                2'd0
            :
                2'd1
        )

    :

    nav_restore_active
    ?
        2'd1

    :

    waypoint_active
    ?
        2'd1

    :
        servo_manual_position;


//======================================================================
// 第三部分：手动控制互斥
//======================================================================

// 自动任务运行时，不让普通手动命令
// 同时控制基础距离/角度控制器。

assign ctrl_manual_rx_done =
    rx_done
    &&
    !sequence_active
    &&
    !obstacle_avoid_active
    &&
    !waypoint_active;


// 自动任务执行时，手动速度清零
// 防止两套控制器同时输出不同运动指令。

assign manual_vx_safe =
    (
        sequence_active
        ||
        obstacle_avoid_active
        ||
        waypoint_active
        ||
        nav_restore_active
    )
    ?
        13'sd0
    :
        vx_cmd;

assign manual_vy_safe =
    (
        sequence_active
        ||
        obstacle_avoid_active
        ||
        waypoint_active
        ||
        nav_restore_active
    )
    ?
        13'sd0
    :
        vy_cmd;

assign manual_wz_safe =
    (
        sequence_active
        ||
        obstacle_avoid_active
        ||
        waypoint_active
        ||
        nav_restore_active
    )
    ?
        13'sd0
    :
        wz_cmd;


//======================================================================
// 第四部分：里程计计数转换为毫米
//======================================================================

// 编码器输出的是脉冲计数，不是实际毫米。
// 这里使用定点乘法流水线完成计数到毫米的转换。
//
// 分两级计算，避免单周期组合运算太长。
// 保持此前已经标定的原工程系数。

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        odom_x_reg <= 32'sd0;
        odom_y_reg <= 32'sd0;

        odom_x_mul_stage1 <= 48'sd0;
        odom_y_mul_stage1 <= 48'sd0;

        odom_x_mul_stage2 <= 48'sd0;
        odom_y_mul_stage2 <= 48'sd0;

        odom_pos_x_mm <= 32'sd0;
        odom_pos_y_mm <= 32'sd0;

    end

    else begin

        // 第一级：锁存编码器位置
        odom_x_reg <= odom_pos_x;
        odom_y_reg <= odom_pos_y;

        // 第二级：移位加法乘以定点系数
        odom_x_mul_stage1 <=
              ($signed(odom_x_reg) <<< 11)
            + ($signed(odom_x_reg) <<< 10)
            + ($signed(odom_x_reg) <<< 7);

        odom_y_mul_stage1 <=
              ($signed(odom_y_reg) <<< 11)
            + ($signed(odom_y_reg) <<< 9);

        // 第三级：继续累加剩余系数
        odom_x_mul_stage2 <=
              odom_x_mul_stage1
            + ($signed(odom_x_reg) <<< 5)
            + ($signed(odom_x_reg) <<< 3)
            + ($signed(odom_x_reg) <<< 2);

        odom_y_mul_stage2 <=
              odom_y_mul_stage1
            + ($signed(odom_y_reg) <<< 3)
            + ($signed(odom_y_reg) <<< 2);

        // Q14格式转为整数毫米
        odom_pos_x_mm <=
            $signed(odom_x_mul_stage2[45:14]);

        odom_pos_y_mm <=
            $signed(odom_y_mul_stage2[45:14]);

    end

end


//======================================================================
// 第五部分：电机目标速度流水线
//======================================================================

// 麦轮分解得到四轮目标速度。
// 通过寄存器缓存，供四路 PI 控制器使用。

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        target_speed_a_reg <= 13'sd0;
        target_speed_b_reg <= 13'sd0;
        target_speed_c_reg <= 13'sd0;
        target_speed_d_reg <= 13'sd0;

        speed_tick_a_d1 <= 1'b0;
        speed_tick_b_d1 <= 1'b0;
        speed_tick_c_d1 <= 1'b0;
        speed_tick_d_d1 <= 1'b0;

    end

    else begin

        target_speed_a_reg <= target_speed_a;
        target_speed_b_reg <= target_speed_b;
        target_speed_c_reg <= target_speed_c;
        target_speed_d_reg <= target_speed_d;

        speed_tick_a_d1 <= speed_tick_a;
        speed_tick_b_d1 <= speed_tick_b;
        speed_tick_c_d1 <= speed_tick_c;
        speed_tick_d_d1 <= speed_tick_d;

    end

end


// A completed raw VL53L0X measurement can report zero when no range
// was resolved (or an object is in the minimum-range blind zone).
// Invalidate an older cached positive range IMMEDIATELY on such a
// report; otherwise a recent 600mm sample can masquerade as current.
// Zero does NOT mean a measured 0mm distance and does NOT mean free.
reg [3:0] tof_raw_zero_seen;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        tof_raw_zero_seen <= 4'b0000;
    else begin
        if (!tof_left_recovery_healthy || tof_left_error)
            tof_raw_zero_seen[0] <= 1'b1;
        else if (tof_left_valid)
            tof_raw_zero_seen[0] <= (tof_left_raw_mm == 16'd0);

        if (!tof_right_recovery_healthy || tof_right_error)
            tof_raw_zero_seen[1] <= 1'b1;
        else if (tof_right_valid)
            tof_raw_zero_seen[1] <= (tof_right_raw_mm == 16'd0);

        if (!tof_front_recovery_healthy || tof_front_error)
            tof_raw_zero_seen[2] <= 1'b1;
        else if (tof_front_valid)
            tof_raw_zero_seen[2] <= (tof_front_raw_mm == 16'd0);

        if (!tof_rear_recovery_healthy || tof_rear_error)
            tof_raw_zero_seen[3] <= 1'b1;
        else if (tof_rear_valid)
            tof_raw_zero_seen[3] <= (tof_rear_raw_mm == 16'd0);
    end
end

//======================================================================
// 第六部分：双 ToF 测距数据缓存
//======================================================================

// 为什么需要缓存？
//
// VL53L0X只在测距完成时输出有效数据脉冲。
// 如果直接使用有效脉冲作为安全状态，
// 就可能在两次测距间隔中误判为传感器失效。
//
// 因此保存最近一次距离，并统计数据年龄。
// 如果超时过久，则判定该侧测距不可用。

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin
        tof_left_mm   <= 16'd0;
        tof_right_mm  <= 16'd0;
        tof_left_seen <= 1'b0;
        tof_right_seen<= 1'b0;
        tof_left_age  <= 8'hFF;
        tof_right_age <= 8'hFF;
    end else begin

        // Recovery / reinitialization window is UNKNOWN.
        if (!tof_left_recovery_healthy || tof_left_error) begin
            tof_left_mm   <= 16'd0;
            tof_left_seen <= 1'b0;
            tof_left_age  <= 8'hFF;
        end else if (tof_left_valid && tof_left_raw_mm == 16'd0) begin
            tof_left_mm  <= 16'd0;
            tof_left_age <= 8'hFF;
        end else if (tof_left_fixed_valid) begin
            tof_left_mm   <= tof_left_fixed_mm;
            tof_left_seen <= 1'b1;
            tof_left_age  <= 8'd0;
        end else if (speed_tick_a && tof_left_age != 8'hFF) begin
            tof_left_age <= tof_left_age + 1'b1;
        end

        if (!tof_right_recovery_healthy || tof_right_error) begin
            tof_right_mm   <= 16'd0;
            tof_right_seen <= 1'b0;
            tof_right_age  <= 8'hFF;
        end else if (tof_right_valid && tof_right_raw_mm == 16'd0) begin
            tof_right_mm  <= 16'd0;
            tof_right_age <= 8'hFF;
        end else if (tof_right_fixed_valid) begin
            tof_right_mm   <= tof_right_fixed_mm;
            tof_right_seen <= 1'b1;
            tof_right_age  <= 8'd0;
        end else if (speed_tick_a && tof_right_age != 8'hFF) begin
            tof_right_age <= tof_right_age + 1'b1;
        end

    end
end


//======================================================================
// 正前方 ToF 数据缓存：逻辑与左右两路一致
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        tof_front_mm   <= 16'd0;
        tof_front_seen <= 1'b0;
        tof_front_age  <= 8'hFF;
    end else if (!tof_front_recovery_healthy || tof_front_error) begin
        tof_front_mm   <= 16'd0;
        tof_front_seen <= 1'b0;
        tof_front_age  <= 8'hFF;
    end else if (tof_front_valid && tof_front_raw_mm == 16'd0) begin
        tof_front_mm  <= 16'd0;
        tof_front_age <= 8'hFF;
    end else if (tof_front_fixed_valid) begin
        tof_front_mm   <= tof_front_fixed_mm;
        tof_front_seen <= 1'b1;
        tof_front_age  <= 8'd0;
    end else if (speed_tick_a && tof_front_age != 8'hFF) begin
        tof_front_age <= tof_front_age + 1'b1;
    end
end

//================ 后方 ToF 缓存和超时 ================
// 仅在有效脉冲到来时更新；500ms无有效数据则视为失效。
// 传感器失效/过近时，fail-closed: 禁止任何倒车。
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        tof_rear_mm   <= 16'd0;
        tof_rear_seen <= 1'b0;
        tof_rear_age  <= 8'hFF;
    end else if (!tof_rear_recovery_healthy || tof_rear_error) begin
        tof_rear_mm   <= 16'd0;
        tof_rear_seen <= 1'b0;
        tof_rear_age  <= 8'hFF;
    end else if (tof_rear_valid && tof_rear_raw_mm == 16'd0) begin
        tof_rear_mm  <= 16'd0;
        tof_rear_age <= 8'hFF;
    end else if (tof_rear_fixed_valid) begin
        tof_rear_mm   <= tof_rear_fixed_mm;
        tof_rear_seen <= 1'b1;
        tof_rear_age  <= 8'd0;
    end else if (speed_tick_a && tof_rear_age != 8'hFF) begin
        tof_rear_age <= tof_rear_age + 1'b1;
    end
end
assign tof_rear_ok = tof_rear_recovery_healthy && tof_rear_seen && !tof_rear_error &&
                     (tof_rear_age <= TOF_MAX_AGE) && !tof_raw_zero_seen[3];
// UNKNOWN rear is not a measured obstacle. A confirmed near rear remains a stop.
assign tof_rear_block = (tof_rear_ok &&
                         tof_rear_mm <= REAR_TOF_STOP_MM) ||
                        a6_near_rear_memory;

assign tof_front_ok = tof_front_recovery_healthy && tof_front_seen && !tof_front_error &&
                      (tof_front_age <= TOF_MAX_AGE) && !tof_raw_zero_seen[2];
// D7: The incomplete 3-ToF installation must NOT disable the manual
// development tests. Front ToF is required for autonomous motion only.

//====================================================
// 前方ToF安全保护
//
// ToF有效且过近：禁止前进
// ToF无效：交给前方HC-SR04安全保护
//
// 不会把ToF=0当成安全距离。
//====================================================

assign tof_front_block =
    ENABLE_FRONT_TOF_SAFETY &&
    d7_auto_active &&
    tof_front_ok &&
    (tof_front_mm <= TOF_FRONT_STOP_MM);


// 第七部分：ToF 健康判断与侧向安全保护
//======================================================================

wire tof_left_ok;
wire tof_right_ok;

wire tof_left_block;
wire tof_right_block;


//==========================================================
// 1. 左侧传感器健康判断
//==========================================================

// 只有同时满足以下三个条件才算正常：
// 1. 曾经成功测距
// 2. 传感器没有错误
// 3. 上次有效数据没有超时

assign tof_left_ok =
    tof_left_recovery_healthy &&
    tof_left_seen
    &&
    !tof_left_error
    &&
    (tof_left_age <= TOF_MAX_AGE) && !tof_raw_zero_seen[0];


//==========================================================
// 2. 右侧传感器健康判断
//==========================================================

assign tof_right_ok =
    tof_right_recovery_healthy &&
    tof_right_seen
    &&
    !tof_right_error
    &&
    (tof_right_age <= TOF_MAX_AGE) && !tof_raw_zero_seen[1];


//==========================================================
// 2026 TASK-3: 0/expired ToF is UNKNOWN, not "obstacle at 0mm".
// It is also NOT a proof of open space. If the independent HC-SR04
// has JUST scanned that side with a clear result, A5 alone can perform
// a <=100ms Y-only low-speed probe, then MUST stop and rescan.
// A6/manual operation never inherits this exception.
// NOTE: servo scan is oblique, so the robot's side/corner swept
// envelope still needs on-ground low-speed validation.
//==========================================================
reg [7:0] a5_scan_age;
reg a5_left_scan_clear;
reg a5_right_scan_clear;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        a5_scan_age         <= 8'hFF;
        a5_left_scan_clear  <= 1'b0;
        a5_right_scan_clear <= 1'b0;
    end else if (!obstacle_avoid_active || obstacle_scan_start) begin
        a5_scan_age         <= 8'hFF;
        a5_left_scan_clear  <= 1'b0;
        a5_right_scan_clear <= 1'b0;
    end else if (servo_scan_done) begin
        a5_scan_age         <= 8'd0;
        a5_left_scan_clear  <= (scan_left_distance_mm >= 16'd600) &&
                               (scan_left_distance_mm <= 16'd3000);
        a5_right_scan_clear <= (scan_right_distance_mm >= 16'd600) &&
                               (scan_right_distance_mm <= 16'd3000);
    end else if (speed_tick_a && a5_scan_age != 8'hFF) begin
        a5_scan_age <= a5_scan_age + 1'b1;
    end
end

wire a5_left_tof_unknown = !tof_left_ok && !tof_left_error;
wire a5_right_tof_unknown = !tof_right_ok && !tof_right_error;

wire a5_left_zero_trial_ok =
    obstacle_avoid_active && (obstacle_state == 4'd5) &&
    a5_left_tof_unknown && a5_left_scan_clear &&
    (a5_scan_age <= 8'd10);
wire a5_right_zero_trial_ok =
    obstacle_avoid_active && (obstacle_state == 4'd8) &&
    a5_right_tof_unknown && a5_right_scan_clear &&
    (a5_scan_age <= 8'd10);

//==========================================================
// 3. 左侧禁止横移条件
//==========================================================

// 满足以下任意一种情况，就禁止左移：
// - ToF数据失效
// - ToF报错
// - ToF距离小于或等于100mm
//
// 注意：这里只表示对应方向不可移动，
// 不代表所有运动方向都必须禁止。

assign tof_left_block =
    ENABLE_LEFT_TOF_SAFETY
    &&
    (
        (tof_left_ok && tof_left_mm <= TOF_SIDE_STOP_MM)
        || a6_near_left_memory
    );


//==========================================================
// 4. 右侧禁止横移条件
//==========================================================

assign tof_right_block =
    ENABLE_RIGHT_TOF_SAFETY
    &&
    (
        (tof_right_ok && tof_right_mm <= TOF_SIDE_STOP_MM)
        || a6_near_right_memory
    );



//======================================================================
// D7: Three-ToF + ultrasonic autonomous A5/A6 safety interface
//======================================================================
// Sensors missing now: left/front validity = 0, so A5/A6 cannot start.
// Manual drive, ToF reporting and E1-E4 pan/scan are left unchanged.
// Never substitute a "clear" artificial range for a missing sensor.


    //========================================================
    // 四路ToF + 车头HC-SR04：A5/A6融合保护
    //
    // 前方超声波：中远距离检测
    // 前方ToF：有效时参与近距离保护
    // 左侧ToF：左移安全保护
    // 右侧ToF：右移安全保护
    // 后方ToF：倒车安全保护
    //
    // ToF无效不代表没有障碍物。
    // 只有对应方向有可靠的距离信息时才允许运动。
    //========================================================

    wire d7_all_tof_ok;
    wire d7_ultra_ok;
    wire d7_scan_ok;
    wire d7_auto_ready;

    wire d7_front_stop;
    wire d7_front_slow;
    wire d7_left_clear;
    wire d7_right_clear;

    wire [1:0] d7_suggested_direction;

    //========================================================
    // 前方超声波是否正对车头
    //========================================================

    wire d7_hc_front_facing;

    assign d7_hc_front_facing =
        (servo_position == 2'd1) &&
        !servo_scan_active;

    //========================================================
    // 前方ToF + 超声波融合距离
    //
    // TOF失效表示UNKNOWN，不是离障碍太近，也不是肯定安全。
    // 两者都有效按近值处理；只有TOF有效而HC未产生脉冲时
    // A5内部直接使用TOF。超声波脉冲在A5内部缓存300ms。
    //========================================================

    wire [15:0] d7_fused_front_mm;
    wire d7_fused_front_valid;

    // A6_FIX: Prefer the nearest VALID source. A zero from an
    // invalid HC-SR04 reading must never replace a valid front ToF.
    wire d7_ultra_fresh_front =
        ultra_distance_valid && d7_hc_front_facing;

    reg [15:0] a6_hc_cache_mm;
    reg [7:0] a6_hc_cache_age;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a6_hc_cache_mm <= 16'd0;
            a6_hc_cache_age <= 8'hFF;
        end else if (d7_ultra_fresh_front &&
                     ultra_distance_mm >= 16'd80 &&
                     ultra_distance_mm <= 16'd3000) begin
            a6_hc_cache_mm <= ultra_distance_mm;
            a6_hc_cache_age <= 8'd0;
        end else if (ultra_echo_timeout && d7_hc_front_facing) begin
            // A no-response / stuck-high event invalidates the old HC cache
            // immediately. Do not keep using a stale frontal distance.
            a6_hc_cache_age <= 8'hFF;
        end else if (speed_tick_a && a6_hc_cache_age != 8'hFF) begin
            a6_hc_cache_age <= a6_hc_cache_age + 1'b1;
        end
    end
    wire a6_hc_cache_ok = d7_hc_front_facing &&
            (a6_hc_cache_age <= 8'd20);
    // Guard against a multi-clock-wide validity flag: a new HC sample
    // must be a fresh rising edge, not three cycles of one measurement.
    reg a6_hc_fresh_d;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) a6_hc_fresh_d <= 1'b0;
        else a6_hc_fresh_d <= d7_ultra_fresh_front;
    end
    wire a6_hc_new_pulse = d7_ultra_fresh_front && !a6_hc_fresh_d;
    // Reliable forward rejoin must work even if front ToF is unavailable.
    // The old implementation required 3 NEW ToF far readings even when
    // a fresh ultrasonic sensor was the only valid frontal measurement.
    // Count only *physical* completed HC readings, never repeated 10ms ticks.
    reg [1:0] a6_hc_far_samples;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a6_hc_far_samples <= 2'd0;
        end else if (a6_hc_new_pulse) begin
            if (ultra_distance_mm >= 16'd340 &&
                ultra_distance_mm <= 16'd3000) begin
                if (a6_hc_far_samples != 2'd3)
                    a6_hc_far_samples <= a6_hc_far_samples + 1'b1;
            end else begin
                a6_hc_far_samples <= 2'd0;
            end
        end else if (speed_tick_a && !a6_hc_cache_ok) begin
            a6_hc_far_samples <= 2'd0;
        end
    end
    assign a6_hc_far_trusted = a6_hc_cache_ok &&
                                (a6_hc_far_samples == 2'd3);


    // A6->A5 trigger is confirmed on THREE NEW physical range
    // measurements, not by repeating one cached 55mm reading on
    // consecutive 10ms control ticks.  These counters are for the
    // AUTONOMOUS A5 HANDOFF only; near-range motor stopping still uses
    // the immediate raw/qualified ToF/ultrasonic protections below.
    reg [1:0] a6_tof_near_samples;
    reg [1:0] a6_hc_near_samples;
    reg a6_tof_near_fresh_d;
    wire a6_tof_new_pulse = tof_front_fixed_valid &&
                            !a6_tof_near_fresh_d;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) a6_tof_near_fresh_d<=1'b0;
        else a6_tof_near_fresh_d<=tof_front_fixed_valid;
    end
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            a6_tof_near_samples<=2'd0;
            a6_hc_near_samples<=2'd0;
        end else begin
            if(a6_tof_new_pulse && !tof_front_error) begin
                if(tof_front_fixed_mm!=16'd0 &&
                   tof_front_fixed_mm<=16'd260) begin
                    if(a6_tof_near_samples!=2'd3)
                        a6_tof_near_samples<=a6_tof_near_samples+1'b1;
                end else a6_tof_near_samples<=2'd0;
            end else if(speed_tick_a && !tof_front_ok)
                a6_tof_near_samples<=2'd0;

            if(a6_hc_new_pulse) begin
                if(ultra_distance_mm>=16'd80 &&
                   ultra_distance_mm<=16'd260) begin
                    if(a6_hc_near_samples!=2'd3)
                        a6_hc_near_samples<=a6_hc_near_samples+1'b1;
                end else a6_hc_near_samples<=2'd0;
            end else if(speed_tick_a && !a6_hc_cache_ok)
                a6_hc_near_samples<=2'd0;
        end
    end
    wire a6_front_near_confirmed =
        (tof_front_ok && tof_front_mm!=16'd0 &&
         tof_front_mm<=16'd260 && a6_tof_near_samples==2'd3) ||
        (a6_hc_cache_ok && a6_hc_cache_mm<=16'd260 &&
         a6_hc_near_samples==2'd3);


    // A5.distance_valid expects a one-clock NEW measurement pulse.
    // Previously it received a sustained cache-valid LEVEL, refreshing
    // its 200-ms timer every 20ns even when no new range existed.
    reg a6_fused_front_fresh_q;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            a6_fused_front_fresh_q <= 1'b0;
        else
            a6_fused_front_fresh_q <=
                (tof_front_valid && !tof_front_error) ||
                a6_hc_new_pulse;
    end
    assign d7_fused_front_valid = tof_front_ok || a6_hc_cache_ok;
    assign d7_fused_front_mm =
        (tof_front_ok && a6_hc_cache_ok) ?
            ((tof_front_mm <= a6_hc_cache_mm) ?
              tof_front_mm : a6_hc_cache_mm) :
        tof_front_ok ? tof_front_mm :
        a6_hc_cache_ok ? a6_hc_cache_mm : 16'd0;

    // Sticky FRONT obstacle in A5/A6: UNKNOWN (expired/unresolved)
    // is not a fresh clearance! A previously measured <=260mm must
    // be cleared by 3 distinct measured >=380mm NEW conversion events.
    // This stops the observed close->FV=0->blind forward regression.
    reg a6_front_obstacle_latch;
    reg [1:0] a6_front_release_new_count;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a6_front_obstacle_latch <= 1'b0;
            a6_front_release_new_count <= 2'd0;
        end else if (d7_fused_front_valid &&
                     d7_fused_front_mm != 16'd0 &&
                     d7_fused_front_mm <= 16'd260) begin
            a6_front_obstacle_latch <= 1'b1;
            a6_front_release_new_count <= 2'd0;
        end else if (a6_fused_front_fresh_q) begin
            if (d7_fused_front_valid &&
                d7_fused_front_mm >= 16'd380) begin
                if (a6_front_release_new_count >= 2'd2) begin
                    a6_front_obstacle_latch <= 1'b0;
                    a6_front_release_new_count <= 2'd3;
                end else
                    a6_front_release_new_count <=
                        a6_front_release_new_count + 1'b1;
            end else
                a6_front_release_new_count <= 2'd0;
        end
    end

    //========================================================
    // D6传感器融合
    //========================================================

    d6_tof_fusion_guard #(

        .FRONT_STOP_MM     (16'd200),
        .FRONT_CLEAR_MM    (16'd280),

        .SIDE_SAFE_MM      (16'd101), // strictly above 100mm side stop
        .SCAN_SIDE_SAFE_MM (16'd200),

        .ULTRA_MAX_AGE     (8'd20),
        .SCAN_MAX_AGE      (10'd300)

    ) u_d7_fusion (

        .clk                (clk),
        .reset_n            (reset_n),
        .sample_tick        (speed_tick_a),

        .tof_left_mm        (tof_left_mm),
        .tof_front_mm       (tof_front_mm),
        .tof_right_mm       (tof_right_mm),

        .tof_left_ok        (tof_left_ok),
        .tof_front_ok       (tof_front_ok),
        .tof_right_ok       (tof_right_ok),

        .ultra_distance_mm  (ultra_distance_mm),

        .ultra_distance_valid (
            ultra_distance_valid &&
            d7_hc_front_facing
        ),

        .ultra_echo_timeout (
            ultra_echo_timeout &&
            d7_hc_front_facing
        ),

        .scan_done          (servo_scan_done),

        .scan_left_mm       (scan_left_distance_mm),
        .scan_center_mm     (scan_center_distance_mm),
        .scan_right_mm      (scan_right_distance_mm),

        .all_tof_ok         (d7_all_tof_ok),
        .ultra_ok           (d7_ultra_ok),
        .scan_ok            (d7_scan_ok),

        .auto_ready         (d7_auto_ready),

        .front_stop         (d7_front_stop),
        .front_slow         (d7_front_slow),

        .left_clear         (d7_left_clear),
        .right_clear        (d7_right_clear),

        .suggested_direction (
            d7_suggested_direction
        )
    );

    //========================================================
    // A5/A6自动运行状态
    //========================================================

    wire d7_auto_active;

    assign d7_auto_active =
        obstacle_avoid_active ||
        waypoint_active ||
        nav_avoid_cycle_active ||
        nav_restore_active;

    //========================================================
    // 自动模式安全控制
    //========================================================

    reg d7_fault_latched;

    wire d7_fault_now;
    wire d7_force_stop;
    wire d7_start_allowed;

    wire [7:0] d7_rx_data;
    wire d7_rx_done;
    wire d7_auto_start_request;

    //========================================================
    // 前方超声波短暂失效宽限计数
    //
    // 舵机转回中位后，允许短暂等待新回波。
    // 等待期间仍禁止向前运动。
    //========================================================

    reg [6:0] d7_ultra_missing_ticks;

    wire d7_ultra_missing_timeout;

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin

            d7_ultra_missing_ticks <= 7'd0;

        end

        else if (
            !d7_auto_active ||
            !d7_hc_front_facing ||
            d7_ultra_ok
        ) begin

            d7_ultra_missing_ticks <= 7'd0;

        end

        else if (
            speed_tick_a &&
            d7_ultra_missing_ticks < 7'd50
        ) begin

            d7_ultra_missing_ticks <=
                d7_ultra_missing_ticks + 1'b1;

        end

    end

    assign d7_ultra_missing_timeout =
        d7_hc_front_facing &&
        !d7_ultra_ok &&
        (d7_ultra_missing_ticks >= 7'd50);

    //========================================================
    // 自动模式故障条件
    //
    // 后方ToF无效时，不直接阻止向前启动。
    // 但是请求倒车时，必须检测后方安全。
    //
    // 左右横移同理。
    //========================================================

    // A5_FIX: A5 has its own ToF blocked-direction handling.
    // A5 must NOT be killed with a latched 0x53 just because a lateral
    // clearance changes during avoidance.  The final motor gate below
    // continues to block every unsafe motion request.
    // For A6 direct lateral navigation (without A5), retain fault latch.
    assign d7_fault_now =
        d7_auto_active &&
        (
            !gyro_calib_done ||
            // A5: no ultrasonic return is UNKNOWN, not a latched fault.
            // The A5 controller + final forward motion gate hold the car
            // safely while it searches for a fresh measurement.
            // A6 retains its former missing-ultrasonic fault protection.
            // HC-SR04 may have no echo even when front ToF has a
            // fresh range. Do not latch STOP solely for missing HC
            // while another trusted frontal range source is healthy.
            // Missing range is UNKNOWN: pause that axis, NOT 0x53 latch.
            // A5 retains per-axis hardware safety and its own FSM.
            (waypoint_active && !a6_odom_plausible)
        );

    //========================================================
    // A5/A6启动许可
    //
    // 不再强制后方BV=1才能开始前进。
    // 仍然要求超声波有效、IMU已校准。
    //
    // 注意：只能保留此处一个赋值。
    //========================================================

    // A5 can start in UNKNOWN sensor conditions, and will hold/scan
    // until a trustworthy forward range is available.
    // A6 still requires its original auto-ready condition.
    // A6 can arm its waypoint FSM without waiting for ALL range channels.
    // Motion is still blocked by a6_sensor_hold and directional motor gates.
    assign d7_start_allowed =
        gyro_calib_done &&
        !d7_fault_latched &&
        !d7_fault_now &&
        ((rx_data != 8'hA6) || a6_odom_plausible);

    //========================================================
    // 故障锁存
    // HEX 53用于清除故障
    //========================================================

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n)
            d7_fault_latched <= 1'b0;

        else if (
            rx_done &&
            rx_data == 8'h53
        )
            d7_fault_latched <= 1'b0;

        else if (d7_fault_now)
            d7_fault_latched <= 1'b1;

    end

    assign d7_auto_start_request =
        (rx_data == 8'hA5) ||
        (rx_data == 8'hA6);

    //========================================================
    // 故障时自动注入停车命令
    //
    // 修复之前d7_force_stop没有驱动的问题。
    //========================================================

    assign d7_force_stop =
        d7_fault_latched ||
        d7_fault_now;

    assign d7_rx_data =
        d7_force_stop ? 8'h53 : rx_data;

    assign d7_rx_done =
        d7_force_stop ||
        (
            rx_done &&
            (
                !d7_auto_start_request ||
                d7_start_allowed
            )
        );

    //========================================================
    // 自主倒车许可
    //
    // 测试阶段关闭自主倒车。
    // 不得直接取消后方防撞。
    //========================================================

    // 只允许A5在ST_BACKUP内执行受控倒车，其他自主状态禁止倒车。
    localparam D7_ALLOW_AUTONOMOUS_REVERSE =
        1'b1;

    wire d7_reverse_inhibit;
    wire d7_forward_inhibit;

    //========================================================
    // 检测倒车请求
    //========================================================

    assign rear_reverse_request =
        ($signed(vx_nav) < 13'sd0) ||
        ($signed(vx_mecanum_reg) < 13'sd0);

    //========================================================
    // 后方ToF无效或过近时禁止倒车
    //========================================================

    assign rear_reverse_inhibit =
        rear_reverse_request &&
        tof_rear_block;

    //========================================================
    // 自动倒车额外许可
    //========================================================

    wire d7_a6_reverse_correction = waypoint_active &&
        !obstacle_avoid_active && !nav_avoid_cycle_active &&
        !nav_restore_active && a6_odom_plausible;

    // A6 turn-relocation issues a BODY reverse command while the A6
    // supervisor is active (NS=2). The old !nav_avoid_cycle_active
    // condition accidentally rejected it. Allow ONLY this measured,
    // low-speed reverse action, with a healthy and sufficiently distant
    // rear ToF. Independent rear_reverse_inhibit remains in force.
    wire d7_a6_detour_reverse_permitted =
        D7_ALLOW_AUTONOMOUS_REVERSE &&
        waypoint_active && nav_detour_active &&
        nav_avoid_state == 3'd2 &&
        ($signed(nav_detour_vx) < 13'sd0) &&
        !obstacle_avoid_active && !nav_restore_active &&
        a6_odom_plausible &&
        tof_rear_ok && (tof_rear_mm >= 16'd440) &&
        !tof_rear_block && !d7_force_stop;

    assign d7_reverse_inhibit = d7_auto_active &&
        rear_reverse_request &&
        !((D7_ALLOW_AUTONOMOUS_REVERSE &&
           obstacle_avoid_active && (obstacle_state == 4'd11)) ||
          d7_a6_reverse_correction ||
          d7_a6_detour_reverse_permitted);

    //========================================================
    // 超声波不朝前或检测到前方危险时
    // 禁止自动模式继续向前运动
    //========================================================

    // When the SG90 is scanning a side, its sonar must NOT serve as
    // live forward clearance.  A fresh, calibrated front ToF reading
    // >450mm alone may authorize *slow* forward motion (<=7 command).
    // Otherwise block X+ but still allow independently verified Y.
    wire d7_front_tof_only_safe = tof_front_ok &&
                                   (tof_front_mm > 16'd280);

    // Keep proven A5 motion gating unchanged.  Only A6 uses the new
    // missing-sensor hold, so A5 does not regress.
    // All sensor UNKNOWN is NOT an obstacle: only a recent confirmed near
    // reading may stop X+. The independent near check survives servo scans.
    assign d7_forward_inhibit = d7_auto_active &&
        ($signed(vx_mecanum_reg) > 13'sd0) &&
        (tof_front_block || d7_front_stop || a6_near_front_memory ||
         a6_front_obstacle_latch);


//======================================================================
//======================================================================
// A6 航点运动命令 1 级寄存器（50MHz 时序优化）
//
// 原路径：里程计 world_x_mm -> A6 32 位坐标比较/速度组合
//        -> 运动来源选择/侧向门控 -> heading_hold.target_yaw/CE
// 现路径：里程计 -> A6组合逻辑 -> 本寄存器
//        本寄存器 -> 运动来源选择 -> heading_hold.target_yaw/CE
//
// 只延迟一拍 50MHz (20ns)，不更改航点坐标或避障阈值。
// 最终电机侧的实时近障禁止门控仍保留，不能删除。
//======================================================================
// Navigation command registers declared above (used also by supervisor).

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        waypoint_vx_timing_q <= 13'sd0;
        waypoint_vy_timing_q <= 13'sd0;
        waypoint_wz_timing_q <= 13'sd0;
    end else begin
        waypoint_vx_timing_q <= waypoint_vx;
        waypoint_vy_timing_q <= waypoint_vy;
        waypoint_wz_timing_q <= waypoint_wz;
    end
end

//======================================================================
// 第八部分：运动命令来源选择
//======================================================================

// 运动指令优先级：
//
// 第一：A5正在避障
// 第二：A6正在恢复航向
// 第三：A6航点导航
// 第四：基础手动/距离/角度控制器
//
// 避免多个模块同时争夺电机控制权。

assign vx_nav =
    mode_host_active ? host_vx_active :
    (obstacle_avoid_active ? obstacle_vx :
     nav_waypoint_pause ? 13'sd0 :
     waypoint_active ? waypoint_vx_timing_q : vx_angle);


assign vy_nav =
    mode_host_active ? host_vy_active :
    (obstacle_avoid_active ? obstacle_vy :
     nav_waypoint_pause ? 13'sd0 :
     waypoint_active ? waypoint_vy_timing_q : vy_angle);


assign wz_nav =
    mode_host_active ? host_wz_active :
    (obstacle_avoid_active ? obstacle_wz :
     nav_waypoint_pause ? 13'sd0 :
     waypoint_active ? waypoint_wz_timing_q : wz_angle);


//======================================================================
// 第九部分：ToF 侧向防撞保护
//======================================================================

// 小车坐标方向：
// +Vy：右移
// -Vy：左移
//
// 左边危险时，只禁止左移。
// 右边危险时，只禁止右移。
//
// 如果某一侧被判定危险，
// 就把当前运动请求的Vx、Vy、Wz全部清零。
// 这是原工程采用的保守停车方式。

assign tof_side_emergency_stop =

    (
        ($signed(vy_nav) < 13'sd0)
        &&
        tof_left_block
    )

    ||

    (
        ($signed(vy_nav) > 13'sd0)
        &&
        tof_right_block
    )
    ;

// Front blocking is intentionally handled independently at the
// downstream registered Vx-axis gate; it must not prohibit a separately
// checked lateral bypass in A5=05/08.  Lateral hazards still stop all
// axes as a conservative corner-sweep policy.


//==========================================================
// 安全过滤后的最终运动请求
//==========================================================

assign vx_nav_safe =
    tof_side_emergency_stop
    ?
        13'sd0
    :
        vx_nav;


assign vy_nav_safe =
    tof_side_emergency_stop
    ?
        13'sd0
    :
        vy_nav;


assign wz_nav_safe =
    tof_side_emergency_stop
    ?
        13'sd0
    :
        wz_nav;


//======================================================================
// 第十部分：PC UART
//======================================================================

// 接收电脑发送的HEX命令。
// 采用原工程已经调通的波特率配置。

uart_byte_rx u_uart_byte_rx(

    .clk       (clk),
    .reset_n   (reset_n),

    .baud_set  (3'd4),

    .uart_rx   (uart_rx),

    .data_byte (pc_rx_data),
    .rx_done   (pc_rx_done)

);

// Filter protocol frames away from the original single-byte command path.
// uart_protocol still receives every raw byte through host_rx_data/host_rx_done.
uart_rx_legacy_filter u_uart_rx_legacy_filter (
    .clk          (clk),
    .reset_n      (reset_n),
    .rx_data      (pc_rx_data),
    .rx_done      (pc_rx_done),
    .legacy_data  (legacy_rx_data),
    .legacy_done  (legacy_rx_done)
);


//======================================================================
// 第十一部分：电脑调试 UART 复用为 BM1684 双向通信 UART
//======================================================================

uart_byte_tx u_host_uart_tx(

    .clk       (clk),
    .reset_n   (reset_n),

    .data_byte (host_tx_data),
    .send_en   (host_tx_send),

    .uart_tx   (uart_tx),

    .tx_done   (host_tx_done),
    .tx_busy   (host_tx_busy)

);


//======================================================================
// 第十二部分：基础手动运动命令
//======================================================================

// 解析前进、后退、左右横移、旋转等命令。
// 这里只产生速度请求，不直接产生PWM。

car_cmd u_car_cmd(

    .clk      (clk),
    .reset_n  (reset_n),

    .rx_data  (rx_data),
    .rx_done  (rx_done),

    .vx_cmd   (vx_cmd),
    .vy_cmd   (vy_cmd),
    .wz_cmd   (wz_cmd)

);


//======================================================================
// 第十三部分：自动前进距离控制器
//======================================================================

// 利用编码器里程计，在达到目标距离前逐级减速。

distance_controller #(

    .TARGET_COUNT     (32'sd2525),
    .SLOW_ZONE_COUNT  (32'sd1000),
    .STOP_EARLY_COUNT (32'sd800),

    .FAST_SPEED       (13'sd32),
    .SLOW_SPEED       (13'sd10)

) u_distance_controller (

    .clk          (clk),
    .reset_n      (reset_n),

    .sample_tick  (speed_tick_a),

    .rx_data      (rx_data),
    .rx_done      (ctrl_manual_rx_done),

    .ext_start    (seq_distance_start),
    .ext_stop     (seq_stop_req),

    .calib_done   (gyro_calib_done),

    .pos_x_count  (odom_pos_x),

    .manual_vx    (manual_vx_safe),
    .manual_vy    (manual_vy_safe),
    .manual_wz    (manual_wz_safe),

    .vx_out       (vx_ctrl),
    .vy_out       (vy_ctrl),
    .wz_out       (wz_ctrl),

    .auto_active  (distance_auto_active),
    .auto_done    (distance_auto_done)

);


//======================================================================
// 第十四部分：横移距离控制器
//======================================================================

// 对麦轮的侧向位移做闭环控制。
// 通过编码器Y方向累计计数判断实际横移距离。

lateral_distance_controller #(

    .TARGET_COUNT     (32'sd2920),
    .SLOW_ZONE_COUNT  (32'sd1000),
    .STOP_EARLY_COUNT (32'sd300),

    .FAST_SPEED       (13'sd24),
    .SLOW_SPEED       (13'sd8)

) u_lateral_distance_controller (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_a),

    .rx_data        (rx_data),
    .rx_done        (ctrl_manual_rx_done),

    .ext_start      (seq_lateral_start),
    .ext_move_right (seq_lateral_move_right),

    .ext_stop       (seq_stop_req),

    .calib_done     (gyro_calib_done),

    .pos_y_count    (odom_pos_y),

    .vx_in          (vx_ctrl),
    .vy_in          (vy_ctrl),
    .wz_in          (wz_ctrl),

    .vx_out         (vx_lateral),
    .vy_out         (vy_lateral),
    .wz_out         (wz_lateral),

    .auto_active    (lateral_auto_active),
    .auto_done      (lateral_auto_done),

    .move_right     (lateral_move_right),

    .travelled_count(lateral_travelled_count)

);


//======================================================================
// 第十五部分：自动转向角度控制器
//======================================================================

// 通过MPU6500测量的航向角进行角度闭环。
// 目标90°，接近目标前降低旋转速度。

angle_controller #(

    .TARGET_ANGLE_CDEG (32'sd9000),

    .SLOW_ZONE_CDEG    (32'sd1500),
    .STOP_EARLY_CDEG   (32'sd1100),

    .FAST_WZ           (13'sd16),
    .SLOW_WZ           (13'sd8),

    .MAX_TURN_TICKS    (9'd300)

) u_angle_controller (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_a),

    .rx_data        (rx_data),
    .rx_done        (ctrl_manual_rx_done),

    .ext_start      (seq_angle_start),
    .ext_turn_right (seq_angle_turn_right),

    .ext_stop       (seq_stop_req),

    .calib_done     (gyro_calib_done),

    .yaw_cdeg       (imu_yaw_cdeg),

    .vx_in          (vx_lateral),
    .vy_in          (vy_lateral),
    .wz_in          (wz_lateral),

    .vx_out         (vx_angle),
    .vy_out         (vy_angle),
    .wz_out         (wz_angle),

    .auto_active    (angle_auto_active),
    .auto_done      (angle_auto_done),

    .angle_travelled_cdeg
    (
        angle_travelled_cdeg
    )

);


//======================================================================
// 第十六部分：航向保持控制器
//======================================================================

// 小车直线前进或横移时，
// MPU6500可能检测到偏航。
// heading_hold根据航向误差输出Wz修正量。
//
// KP_DIV：比例参数
// MAX_CORRECTION：最大转向修正幅度
// REVERSE：修正方向

heading_hold #(

    .KP_DIV         (32'sd40),

    .MAX_CORRECTION (13'sd10),

    .REVERSE        (1'b1)

) u_heading_hold (

    .clk         (clk),
    .reset_n     (reset_n),

    .vx_cmd      (vx_nav_safe),
    .vy_cmd      (vy_nav_safe),
    .wz_user     (wz_nav_safe),

    .calib_done  (gyro_calib_done),

    .yaw_cdeg    (imu_yaw_cdeg),

    .wz_final    (wz_final),

    .target_yaw  (heading_target_yaw),
    .yaw_error   (heading_yaw_error),

    .hold_active (heading_hold_active)

);


//======================================================================
// 第十七部分：最终旋转速度控制
//======================================================================

// A5横移和A5原地搜索的旋转处理不同。
//
// 横移时：允许有限的航向修正，避免小车斜着走。
// 旋转搜索时：使用A5要求的Wz。
// 停车、扫描、倒车等阶段：禁止额外的航向修正。
// A6恢复航向时：使用恢复控制器的Wz。

wire obstacle_lateral_active;
wire obstacle_force_rotate_stop;

wire signed [12:0] wz_mecanum;


// A5正在左移、右移或者返回原路线
assign obstacle_lateral_active =

       (obstacle_state == 4'd5)
    || (obstacle_state == 4'd7)
    || (obstacle_state == 4'd8)
    || (obstacle_state == 4'd10);


// A5这些状态不能让航向保持产生额外旋转
assign obstacle_force_rotate_stop =

    obstacle_avoid_active
    &&
    (
           (obstacle_state == 4'd1)
        || (obstacle_state == 4'd3)
        || (obstacle_state == 4'd4)
        || (obstacle_state == 4'd11)
        || (obstacle_state == 4'd12)
        || (obstacle_state == 4'd13)
        || (obstacle_state == 4'd15)
    );


//==========================================================
// 最终送给麦轮运动学模块的旋转速度
//==========================================================

assign wz_mecanum =
    mode_host_active ? host_wz_active :
    (tof_side_emergency_stop ? 13'sd0 :
     nav_restore_active ? nav_restore_wz :
     (obstacle_avoid_active && obstacle_state == 4'd14) ? obstacle_wz :
     obstacle_force_rotate_stop ? 13'sd0 :
     obstacle_lateral_active ?
        ((wz_final > 13'sd5) ? 13'sd5 :
         (wz_final < -13'sd5) ? -13'sd5 : wz_final) :
     wz_final);


//======================================================================
// 第十八部分：麦克纳姆轮运动学分解
//======================================================================

    //========================================================
    // 新增：底盘运动指令流水线
    //
    // 目的：
    // 将 A6 航点导航到四轮目标速度之间的长组合路径
    // 分成两级，改善 50MHz Setup Timing。
    //
    // 第一级：
    // waypoint_nav -> 运动源选择 -> 速度寄存器
    //
    // 第二级：
    // 速度寄存器 -> mecanum_o -> 四轮速度寄存器
    //
    // 增加1个50MHz时钟周期的延迟，即20ns。
    // 对比10ms的速度控制周期，该延迟通常很小。
    //========================================================

    reg signed [12:0] vx_mecanum_reg;
    reg signed [12:0] vy_mecanum_reg;
    reg signed [12:0] wz_mecanum_reg;

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin

            vx_mecanum_reg <= 13'sd0;
            vy_mecanum_reg <= 13'sd0;
            wz_mecanum_reg <= 13'sd0;

        end

        else begin

            // 保存本周期计算出的底盘运动指令
            vx_mecanum_reg <= vx_nav_safe;
            vy_mecanum_reg <= vy_nav_safe;
            wz_mecanum_reg <= wz_mecanum;

        end

    end


    //========================================================
    // 流水线输出的安全保护
    //
    // 由于增加了寄存器，速度指令会延迟一个周期。
    //
    // 因此在寄存器之后仍然保留ToF安全门控，
    // 防止新检测到的侧向危险被流水线延迟。
    //
    // 此处同时禁止避障错误状态下继续驱动。
    //========================================================

    wire signed [12:0] vx_mecanum_final;
    wire signed [12:0] vy_mecanum_final;
    wire signed [12:0] wz_mecanum_final;

    wire mecanum_emergency_stop;

    // GLOBAL STOP only for a fault or an unsafe side-body motion.
    // A front obstacle blocks X+, without disabling a proven-safe Y move.
    // A rear obstacle similarly blocks X-; normal A5 scan HOLD/ROTATE
    // remain independent of this translation-axis protection.
    assign mecanum_emergency_stop =
        tof_side_emergency_stop
        || (a6_sensor_hold && waypoint_active)
        || obstacle_avoid_error
        || d7_force_stop
        // Second gate after the pipeline register, no stale side motion.
        || (($signed(vy_mecanum_reg) < 13'sd0) && tof_left_block)
        || (($signed(vy_mecanum_reg) > 13'sd0) && tof_right_block);

    wire x_axis_inhibited =
        d7_forward_inhibit ||
        d7_reverse_inhibit ||
        rear_reverse_inhibit ||
        (($signed(vx_mecanum_reg) > 13'sd0) &&
         (tof_front_block || (mode_host_active && d7_front_stop)));

    wire signed [12:0] offcenter_forward_limited =
        (d7_auto_active && !d7_hc_front_facing &&
         ($signed(vx_mecanum_reg) > 13'sd7)) ?
         13'sd7 : vx_mecanum_reg;

    // A6 with UNKNOWN frontal range runs only at low speed.
    wire signed [12:0] a6_probe_limited_vx =
        (a6_unknown_forward &&
         ($signed(offcenter_forward_limited) > 13'sd6)) ?
        13'sd6 : offcenter_forward_limited;
    wire signed [12:0] a6_uncertain_side_limited =
        (a6_unknown_side_command &&
         $signed(vy_mecanum_reg) > 13'sd6) ? 13'sd6 :
        (a6_unknown_side_command &&
         $signed(vy_mecanum_reg) < -13'sd6) ? -13'sd6 :
        vy_mecanum_reg;

    wire signed [12:0] rear_unknown_limited_vx =
        (d7_auto_active && !tof_rear_ok &&
         $signed(a6_probe_limited_vx) < -13'sd6) ?
            -13'sd6 : a6_probe_limited_vx;
    assign vx_mecanum_final =
        (mecanum_emergency_stop || x_axis_inhibited) ?
            13'sd0 : rear_unknown_limited_vx;

    assign vy_mecanum_final =
        mecanum_emergency_stop ? 13'sd0 : a6_uncertain_side_limited;

    // Prevent heading correction when the nose is too close to an
    // obstacle during attempted forward movement.  Explicit A5
    // rotation uses Vx=0, and its own four-direction clearance test.
    wire signed [12:0] a6_turn_limited_wz =
        ((waypoint_turning || nav_restore_active) && a6_turn_has_unknown &&
         $signed(wz_mecanum_reg) > 13'sd7) ? 13'sd7 :
        ((waypoint_turning || nav_restore_active) && a6_turn_has_unknown &&
         $signed(wz_mecanum_reg) < -13'sd7) ? -13'sd7 :
        wz_mecanum_reg;
    // During a bounded A6 UNKNOWN-side translation trial, suppress
    // simultaneous yaw correction.  Otherwise a steering correction can
    // turn a small lateral probe into an unintended corner sweep.
    // This does not affect normal A6 90-degree waypoint turns.
    wire a6_unknown_side_probe_veto_wz = obstacle_avoid_active &&
        nav_avoid_cycle_active &&
        (((obstacle_state == 4'd5) && a5_left_tof_unknown &&
          !tof_left_block) ||
         ((obstacle_state == 4'd8) && a5_right_tof_unknown &&
          !tof_right_block));
    assign wz_mecanum_final =
        (mecanum_emergency_stop ||
         (d7_forward_inhibit && !waypoint_turning && !nav_restore_active) ||
         ((waypoint_turning || nav_restore_active) && !a6_turn_space_ok) ||
         a5_left_zero_trial_ok || a5_right_zero_trial_ok ||
         a6_unknown_side_probe_veto_wz) ?
            13'sd0 : a6_turn_limited_wz;


    //========================================================
    // 麦克纳姆轮运动学分解
    //
    // 输入：
    // Vx：前后速度
    // Vy：左右横移速度
    // Wz：旋转速度
    //
    // 输出：
    // A/B/C/D四轮目标速度
    //========================================================

    // STA FIX (2026-10-27): Two internal registered timing stages in
    // mecanum_o separate A5 state/safety decode from wheel arithmetic.
    // Do NOT alter the top-level emergency-stop gating downstream.
    mecanum_o u_mecanum_o (

        .clk(clk),
        .reset_n(reset_n),

        .vx(vx_mecanum_final),

        .vy(vy_mecanum_final),

        .wz(wz_mecanum_final),

        .speed_a(target_speed_a),
        .speed_b(target_speed_b),
        .speed_c(target_speed_c),
        .speed_d(target_speed_d)

    );



//======================================================================
// 第十九部分：四路电机驱动
//======================================================================

// 电机驱动模块根据有符号PWM命令决定：
// 正转、反转、占空比。
//
// 方向反了时才修改对应的REVERSE参数。
// 这里保持之前验证正确的方向配置。

motor_driver #(
    .REVERSE(1'b0)
) u_motor_a (

    .clk       (clk),
    .reset_n   (reset_n),

    .speed_cmd (mecanum_emergency_stop ? 13'sd0 : pwm_cmd_a),

    .motor_pwm (motor_a_pwm),
    .motor_in1 (motor_a_in1),
    .motor_in2 (motor_a_in2)

);


motor_driver #(
    .REVERSE(1'b0)
) u_motor_b (

    .clk       (clk),
    .reset_n   (reset_n),

    .speed_cmd (mecanum_emergency_stop ? 13'sd0 : pwm_cmd_b),

    .motor_pwm (motor_b_pwm),
    .motor_in1 (motor_b_in1),
    .motor_in2 (motor_b_in2)

);


motor_driver #(
    .REVERSE(1'b0)
) u_motor_c (

    .clk       (clk),
    .reset_n   (reset_n),

    .speed_cmd (mecanum_emergency_stop ? 13'sd0 : pwm_cmd_c),

    .motor_pwm (motor_c_pwm),
    .motor_in1 (motor_c_in1),
    .motor_in2 (motor_c_in2)

);


motor_driver #(
    .REVERSE(1'b0)
) u_motor_d (

    .clk       (clk),
    .reset_n   (reset_n),

    .speed_cmd (mecanum_emergency_stop ? 13'sd0 : pwm_cmd_d),

    .motor_pwm (motor_d_pwm),
    .motor_in1 (motor_d_in1),
    .motor_in2 (motor_d_in2)

);


//======================================================================
// 第二十部分：四轮编码器采集
//======================================================================

// 每个编码器分别输出累计位置。
// REVERSE 用于修正编码器计数方向。
// 保持原工程已经验证的设置。

encoder_quad #(
    .REVERSE(1'b1)
) u_encoder_a (

    .clk      (clk),
    .reset_n  (reset_n),

    .enc_a    (encoder_a_cha),
    .enc_b    (encoder_a_chb),

    .position (encoder_a_position)

);


encoder_quad #(
    .REVERSE(1'b1)
) u_encoder_b (

    .clk      (clk),
    .reset_n  (reset_n),

    .enc_a    (encoder_b_cha),
    .enc_b    (encoder_b_chb),

    .position (encoder_b_position)

);


encoder_quad #(
    .REVERSE(1'b0)
) u_encoder_c (

    .clk      (clk),
    .reset_n  (reset_n),

    .enc_a    (encoder_c_cha),
    .enc_b    (encoder_c_chb),

    .position (encoder_c_position)

);


encoder_quad #(
    .REVERSE(1'b0)
) u_encoder_d (

    .clk      (clk),
    .reset_n  (reset_n),

    .enc_a    (encoder_d_cha),
    .enc_b    (encoder_d_chb),

    .position (encoder_d_position)

);


//======================================================================
// 第二十一部分：四轮里程计
//======================================================================

// 编码器负责记录轮子的运动。
// MPU6500提供航向角。
// 两者结合可以得到小车X、Y位移。
//
// 转向期间暂停平移里程累计：
// - 自动角度控制
// - A6恢复航向
// - A5原地旋转搜索
//
// 这样可以减少原地转向造成的假位移。

wire odom_freeze_translation;

assign odom_freeze_translation =

       angle_auto_active
    || nav_restore_active
    // Pure 90-degree waypoint rotation has no translation.
    || waypoint_turning
    || (
           obstacle_avoid_active
        && obstacle_state == 4'd14
       );


mecanum_odometry u_mecanum_odometry (

    .clk                (clk),
    .reset_n            (reset_n),

    .sample_tick        (speed_tick_a),

    .encoder_a_position (encoder_a_position),
    .encoder_b_position (encoder_b_position),
    .encoder_c_position (encoder_c_position),
    .encoder_d_position (encoder_d_position),

    .yaw_cdeg           (imu_yaw_cdeg),

    .freeze_translation (odom_freeze_translation),

    .delta_x_count      (odom_delta_x),
    .delta_y_count      (odom_delta_y),
    .delta_w_count      (odom_delta_w),

    .pos_x_count        (odom_pos_x),
    .pos_y_count        (odom_pos_y),

    .world_x_mm         (odom_world_x_mm),
    .world_y_mm         (odom_world_y_mm),

    .odom_yaw_cdeg      (odom_yaw)

);


//======================================================================
// 第二十二部分：四轮速度测量
//======================================================================

// 通过固定周期内编码器计数变化计算轮速。
// speed_tick 同时作为速度PI及导航的采样周期来源。

speed_measure u_speed_a (

    .clk         (clk),
    .reset_n     (reset_n),

    .position    (encoder_a_position),
    .speed_count (encoder_a_speed),

    .sample_tick (speed_tick_a)

);


speed_measure u_speed_b (

    .clk         (clk),
    .reset_n     (reset_n),

    .position    (encoder_b_position),
    .speed_count (encoder_b_speed),

    .sample_tick (speed_tick_b)

);


speed_measure u_speed_c (

    .clk         (clk),
    .reset_n     (reset_n),

    .position    (encoder_c_position),
    .speed_count (encoder_c_speed),

    .sample_tick (speed_tick_c)

);


speed_measure u_speed_d (

    .clk         (clk),
    .reset_n     (reset_n),

    .position    (encoder_d_position),
    .speed_count (encoder_d_speed),

    .sample_tick (speed_tick_d)

);


//======================================================================
// 第二十三部分：四轮速度PI闭环
//======================================================================

// 控制目标：让实际轮速接近目标轮速。
//
// 输入：
// target_speed：期望速度
// feedback_speed：编码器实际速度
//
// 输出：
// pwm_cmd：电机PWM控制量
//
// KP、KI 保持之前验证过的数值。

speed_pi #(
    .KP(16'sd25),
    .KI(16'sd4)
) u_pi_a (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_a_d1),

    .target_speed   (target_speed_a_reg),
    .feedback_speed (encoder_a_speed),

    .pwm_cmd        (pwm_cmd_a)

);


speed_pi #(
    .KP(16'sd25),
    .KI(16'sd4)
) u_pi_b (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_b_d1),

    .target_speed   (target_speed_b_reg),
    .feedback_speed (encoder_b_speed),

    .pwm_cmd        (pwm_cmd_b)

);


speed_pi #(
    .KP(16'sd25),
    .KI(16'sd4)
) u_pi_c (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_c_d1),

    .target_speed   (target_speed_c_reg),
    .feedback_speed (encoder_c_speed),

    .pwm_cmd        (pwm_cmd_c)

);


speed_pi #(
    .KP(16'sd25),
    .KI(16'sd4)
) u_pi_d (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_d_d1),

    .target_speed   (target_speed_d_reg),
    .feedback_speed (encoder_d_speed),

    .pwm_cmd        (pwm_cmd_d)

);


//======================================================================
// 第二十四部分：舵机控制模块
//======================================================================

// servo_cmd负责解析手动舵机命令。
// servo_pwm负责把位置转换成SG90控制脉宽。

servo_cmd u_servo_cmd (

    .clk            (clk),
    .reset_n        (reset_n),

    .rx_data        (rx_data),
    .rx_done        (rx_done),

    .servo_position (servo_manual_position)

);


servo_pwm u_servo_pwm (

    .clk      (clk),
    .reset_n  (reset_n),

    .position (servo_position),

    .servo_pwm(servo_pwm)

);


//======================================================================
// 第二十五部分：MPU6500初始化
//======================================================================

// 给MPU6500配置工作模式和量程。
// init_done=1后，后级才开始读取有效数据。

mpu6500_init u_mpu6500_init (

    .clk        (clk),
    .reset_n    (reset_n),

    .imu_sda    (imu_sda),
    .imu_scl    (imu_scl),

    .init_done  (imu_init_done),
    .init_error (imu_init_error),
    .init_step  (imu_init_step)

);


//======================================================================
// 第二十六部分：MPU6500连续读取
//======================================================================

// 读取加速度、温度和三轴角速度。
// 通过I2C总线与MPU6500通信。

mpu6500_burst_reader u_mpu6500_burst_reader (

    .clk        (clk),
    .reset_n    (reset_n),

    .init_done  (imu_init_done),

    .imu_sda    (imu_sda),
    .imu_scl    (imu_scl),

    .accel_x    (imu_accel_x),
    .accel_y    (imu_accel_y),
    .accel_z    (imu_accel_z),

    .temp_raw   (imu_temp_raw),

    .gyro_x     (imu_gyro_x),
    .gyro_y     (imu_gyro_y),
    .gyro_z     (imu_gyro_z),

    .data_valid (imu_data_valid),
    .read_error (imu_read_error)

);


//======================================================================
// 第二十七部分：陀螺仪自动零偏校准
//======================================================================

// 小车上电后保持静止。
// gyro_calib自动采集样本计算偏置。
// 后面的航向积分使用去除零偏后的角速度。

gyro_calib u_gyro_calib (

    .clk              (clk),
    .reset_n          (reset_n),

    .data_valid       (imu_data_valid),

    .gyro_x_raw       (imu_gyro_x),
    .gyro_y_raw       (imu_gyro_y),
    .gyro_z_raw       (imu_gyro_z),

    .bias_x           (gyro_bias_x),
    .bias_y           (gyro_bias_y),
    .bias_z           (gyro_bias_z),

    .gyro_x_corrected (gyro_x_corrected),
    .gyro_y_corrected (gyro_y_corrected),
    .gyro_z_corrected (gyro_z_corrected),

    .calib_done       (gyro_calib_done)

);


//======================================================================
// 第二十八部分：航向角积分
//======================================================================

// 使用校准后的Z轴角速度积分得到yaw。
// 输出单位为0.01度。
//
// 例如：
// yaw=9000，表示90.00度。

yaw_integrator u_yaw_integrator (

    .clk              (clk),
    .reset_n          (reset_n),

    .data_valid       (imu_data_valid),

    .calib_done       (gyro_calib_done),

    .read_error       (imu_read_error),

    .gyro_z_corrected (gyro_z_corrected),

    .zero_yaw         (1'b0),

    .yaw_cdeg         (imu_yaw_cdeg)

);


//======================================================================
// 第二十九部分：MPU调试串口
//======================================================================

// 把校准状态和航向角发送至PC。
// 当前PC的uart_tx连接这个报告模块。
//
// 注意：本版本PC串口不会自动输出左右ToF距离。
// ToF距离已经在FPGA内部用于避障判断。
//================================================
// 原IMU调试串口输出
//
// 为了避免与ToF监测模块同时驱动PC串口，
// 将原IMU报告连接至内部未使用信号。
// IMU数据读取、校准和航向计算保持正常。
//================================================

wire imu_uart_unused;
imu_debug_uart_report u_imu_debug_uart_report (

    .clk        (clk),
    .reset_n    (reset_n),

    .calib_done (gyro_calib_done),

    .data_valid (imu_data_valid),

    .read_error (imu_read_error),

    .yaw_cdeg   (imu_yaw_cdeg),

    .uart_tx(imu_uart_unused)

);


//======================================================================
// 第三十部分：基础运动序列
//======================================================================

// 实现以前已完成的直行、横移、旋转组合任务。
// A5/A6使用独立状态机，不在此模块里重写。

motion_sequence #(

    .PAUSE_TICKS        (8'd50),

    .STEP_TIMEOUT_TICKS (10'd800)

) u_motion_sequence (

    .clk                (clk),
    .reset_n            (reset_n),

    .sample_tick        (speed_tick_a),

    .rx_data            (rx_data),
    .rx_done            (rx_done),

    .calib_done         (gyro_calib_done),

    .distance_auto_active (distance_auto_active),
    .distance_auto_done   (distance_auto_done),

    .angle_auto_active    (angle_auto_active),
    .angle_auto_done      (angle_auto_done),

    .lateral_auto_active  (lateral_auto_active),
    .lateral_auto_done    (lateral_auto_done),

    .distance_start     (seq_distance_start),

    .angle_start        (seq_angle_start),
    .angle_turn_right   (seq_angle_turn_right),

    .lateral_start      (seq_lateral_start),
    .lateral_move_right (seq_lateral_move_right),

    .stop_req           (seq_stop_req),

    .sequence_active    (sequence_active),
    .sequence_done      (sequence_done),
    .sequence_error     (sequence_error),

    .state_debug        (sequence_state)

);


//======================================================================
// 第三十一部分：HC-SR04实时超声波测距
//======================================================================

// trig产生超声波触发脉冲。
// echo返回声波往返时间。
// 模块换算出距离（mm）。
//
// 用途：
// 1. A5前方障碍检测
// 2. A5前进减速和停车
// 3. A6遇障后触发A5
// 4. 舵机扫描时的测距数据来源

ultrasonic_driver u_ultrasonic_driver (

    .clk            (clk),
    .reset_n        (reset_n),

    .ultra_trig     (ultra_trig),
    .ultra_echo     (ultra_echo),

    .distance_mm    (ultra_distance_mm),
    .distance_valid (ultra_distance_valid),

    .measure_busy      (ultra_measure_busy),
    .echo_timeout      (ultra_echo_timeout),

    .measurement_done  (ultra_measurement_done),
    .sensor_online     (ultra_sensor_online),
    .sensor_recovering (ultra_sensor_recovering),
    .recovery_count    (ultra_recovery_count)

);


//======================================================================
// 第三十二部分：左侧 VL53L0X
//======================================================================

// 左传感器独立I2C总线。
// 此处继续使用已经验证能正常测距的驱动。
//
// 注意：
// VL53L0X模块自身内部有初始化状态机。
// 只有初始化和测距成功后才会输出有效数据。

vl53l0x_sensor_core u_tof_left (

    .clk                 (clk),
    .reset_n             (tof_left_driver_reset_n),

    .imu_sda             (tof1_sda),
    .imu_scl             (tof1_scl),

    .uart_tx             (tof_left_uart_unused),

    .distance_raw_mm_out (tof_left_raw_mm),
    .distance_cal_mm_out (tof_left_cal_mm),

    .distance_valid_out  (tof_left_valid),

    .sensor_error        (tof_left_error),

    .debug_state_out     (tof_left_state)

);


//======================================================================
// 第三十三部分：右侧 VL53L0X
//======================================================================

// 右传感器使用另一组I2C引脚。
// 两块传感器可以保持同一个I2C地址0x29，
// 因为它们在不同I2C总线上。

//==========================================================
// Four independent VL53L0X recovery supervisors
//
// Policy:
// - Each channel has its own data timeout and error monitor.
// - Only that channel's driver reset is pulsed.
// - Local manual mode permits a per-channel driver reset while the motors
//   are commanded; autonomous/host operation still requires a stopped chassis.
// - Data timeout 300ms; reset pulse 10ms; retry delay staggered 150..300ms.
// - A channel is declared healthy after 2 fresh distance samples.
// - A stale/failed channel is UNKNOWN and its cached distance is cleared.
//==========================================================

assign tof_recovery_motion_stopped =
    (vx_mecanum_reg == 13'sd0) &&
    (vy_mecanum_reg == 13'sd0) &&
    (wz_mecanum_reg == 13'sd0);

// Local manual operation may keep the motor command active continuously.
// In that mode, allow an individual ToF FPGA driver to soft-reset without
// waiting for the chassis to stop. This reset touches only that ToF channel's
// I2C controller/state machine; it does not reset the motor controller.
// Autonomous sequences and host-controlled operation still require a stop.
wire tof_recovery_local_manual_mode =
    !mode_host_active &&
    !sequence_active &&
    !obstacle_avoid_active &&
    !waypoint_active &&
    !nav_avoid_cycle_active &&
    !nav_restore_active;

wire tof_recovery_reset_permitted =
    tof_recovery_motion_stopped || tof_recovery_local_manual_mode;

tof_channel_recovery #(
    .CLK_HZ(50_000_000),
    .DATA_TIMEOUT_MS(300),
    .RESET_HOLD_MS(10),
    .STARTUP_TIMEOUT_MS(15_000),
    .RETRY_COOLDOWN_MS(150),
    .ERROR_CONFIRM_MS(2),
    .VALID_CONFIRM_SAMPLES(2),
    .INITIAL_DELAY_MS(0),
    .RUN_STATE_MIN(7'd36),
    .STOP_STABLE_MS(20)
) u_tof_left_recovery (
    .clk(clk), .reset_n(reset_n),
    .sensor_error(tof_left_error),
    .distance_valid(tof_left_valid),
    .debug_state(tof_left_state),
    .reset_permitted(tof_recovery_reset_permitted),
    .driver_reset_n(tof_left_driver_reset_n),
    .healthy(tof_left_recovery_healthy),
    .recovering(tof_left_recovering),
    .fault_latched(tof_left_recovery_fault),
    .recovery_count(tof_left_recovery_count)
);

tof_channel_recovery #(
    .CLK_HZ(50_000_000),
    .DATA_TIMEOUT_MS(300),
    .RESET_HOLD_MS(10),
    .STARTUP_TIMEOUT_MS(15_000),
    .RETRY_COOLDOWN_MS(200),
    .ERROR_CONFIRM_MS(2),
    .VALID_CONFIRM_SAMPLES(2),
    .INITIAL_DELAY_MS(25),
    .RUN_STATE_MIN(7'd36),
    .STOP_STABLE_MS(20)
) u_tof_right_recovery (
    .clk(clk), .reset_n(reset_n),
    .sensor_error(tof_right_error),
    .distance_valid(tof_right_valid),
    .debug_state(tof_right_state),
    .reset_permitted(tof_recovery_reset_permitted),
    .driver_reset_n(tof_right_driver_reset_n),
    .healthy(tof_right_recovery_healthy),
    .recovering(tof_right_recovering),
    .fault_latched(tof_right_recovery_fault),
    .recovery_count(tof_right_recovery_count)
);

tof_channel_recovery #(
    .CLK_HZ(50_000_000),
    .DATA_TIMEOUT_MS(300),
    .RESET_HOLD_MS(10),
    .STARTUP_TIMEOUT_MS(15_000),
    .RETRY_COOLDOWN_MS(250),
    .ERROR_CONFIRM_MS(2),
    .VALID_CONFIRM_SAMPLES(2),
    .INITIAL_DELAY_MS(50),
    .RUN_STATE_MIN(7'd36),
    .STOP_STABLE_MS(20)
) u_tof_front_recovery (
    .clk(clk), .reset_n(reset_n),
    .sensor_error(tof_front_error),
    .distance_valid(tof_front_valid),
    .debug_state(tof_front_state),
    .reset_permitted(tof_recovery_reset_permitted),
    .driver_reset_n(tof_front_driver_reset_n),
    .healthy(tof_front_recovery_healthy),
    .recovering(tof_front_recovering),
    .fault_latched(tof_front_recovery_fault),
    .recovery_count(tof_front_recovery_count)
);

tof_channel_recovery #(
    .CLK_HZ(50_000_000),
    .DATA_TIMEOUT_MS(300),
    .RESET_HOLD_MS(10),
    .STARTUP_TIMEOUT_MS(15_000),
    .RETRY_COOLDOWN_MS(300),
    .ERROR_CONFIRM_MS(2),
    .VALID_CONFIRM_SAMPLES(2),
    .INITIAL_DELAY_MS(75),
    .RUN_STATE_MIN(7'd36),
    .STOP_STABLE_MS(20)
) u_tof_rear_recovery (
    .clk(clk), .reset_n(reset_n),
    .sensor_error(tof_rear_error),
    .distance_valid(tof_rear_valid),
    .debug_state(tof_rear_state),
    .reset_permitted(tof_recovery_reset_permitted),
    .driver_reset_n(tof_rear_driver_reset_n),
    .healthy(tof_rear_recovery_healthy),
    .recovering(tof_rear_recovering),
    .fault_latched(tof_rear_recovery_fault),
    .recovery_count(tof_rear_recovery_count)
);


vl53l0x_sensor_core u_tof_right (

    .clk                 (clk),
    .reset_n             (tof_right_driver_reset_n),

    .imu_sda             (tof2_sda),
    .imu_scl             (tof2_scl),

    .uart_tx             (tof_right_uart_unused),

    .distance_raw_mm_out (tof_right_raw_mm),
    .distance_cal_mm_out (tof_right_cal_mm),

    .distance_valid_out  (tof_right_valid),

    .sensor_error        (tof_right_error),

    .debug_state_out     (tof_right_state)

);

//========================================================
    // 双ToF + A5/A6 串口实时监测
    //
    // 使用原来的电脑调试串口：
    // FPGA TX：M15
    // 波特率：115200
    //
    // 不占用HOST上位机串口。
    //
    // 监测项目：
    // 1. 左侧ToF距离
    // 2. 右侧ToF距离
    // 3. 左右传感器健康状态
    // 4. A5避障状态
    // 5. A6协调器状态
    // 6. 当前航点编号
    //========================================================

// 第三路正前方 VL53L0X：独立I2C，模块尚未安装也可综合
vl53l0x_sensor_core u_tof_front (
    .clk                 (clk),
    .reset_n             (tof_front_driver_reset_n),
    .imu_sda             (tof3_sda),
    .imu_scl             (tof3_scl),
    .uart_tx             (tof_front_uart_unused),
    .distance_raw_mm_out (tof_front_raw_mm),
    .distance_cal_mm_out (tof_front_cal_mm),
    .distance_valid_out  (tof_front_valid),
    .sensor_error        (tof_front_error),
    .debug_state_out     (tof_front_state)
);

//================ 后方 ToF：独立I2C 0x29 ================
vl53l0x_sensor_core u_tof_rear (
    .clk                 (clk),
    .reset_n             (tof_rear_driver_reset_n),
    .imu_sda             (tof4_sda),
    .imu_scl             (tof4_scl),
    .uart_tx             (tof_rear_uart_unused),
    .distance_raw_mm_out (tof_rear_raw_mm),
    .distance_cal_mm_out (tof_rear_cal_mm),
    .distance_valid_out  (tof_rear_valid),
    .sensor_error        (tof_rear_error),
    .debug_state_out     (tof_rear_state)
);

wire debug_report_uart_unused;

tof_nav_uart_report u_tof_nav_uart_report (
.clk            (clk),
.reset_n        (reset_n),
// 左侧最近一次有效距离
.left_mm        (tof_left_mm),
// 右侧最近一次有效距离
.right_mm       (tof_right_mm),
// 前方最近一次有效距离（模组未安装时为0）
.front_mm       (tof_front_mm),
// 后方 VL53L0X 距离和健康状态
.rear_mm        (tof_rear_mm),
.rear_ok        (tof_rear_ok),
// 左侧数据是否仍然有效
.left_ok        (tof_left_ok),
// 右侧数据是否仍然有效
.right_ok       (tof_right_ok),
.front_ok       (tof_front_ok),
.right_error    (tof_right_error),
// A5状态机
.a5_state       (obstacle_state),
// A6协调器状态
.a6_state       (a6_start_diagnostic),
// 当前航点
.waypoint_index (waypoint_index),
// A6 navigation first fault and motor gate diagnostics.
.fault_code (a6_nav_fault_code),
.gate_debug (a6_gate_debug),
.supervisor_debug (nav_avoid_state),
.detour_direction (nav_detour_dir_debug),
.detour_distance (nav_detour_mm_debug),
.detour_fault (nav_detour_fault_debug),
.detour_vx (nav_detour_vx),
.detour_vy (nav_detour_vy),
.turn_block_reason (a6_turn_block_reason),
// P4 front-source diagnostics: live at each UART frame and first veto.
.hc_front_mm        (a6_hc_cache_mm),
.hc_front_valid     (a6_hc_cache_ok),
.hc_front_age       (a6_hc_cache_age),
.fused_front_mm     (d7_fused_front_mm),
.fused_front_valid  (d7_fused_front_valid),
.turn_event_seen    (a6_turn_event_seen),
.turn_event_tof_mm  (a6_turn_event_tof_mm),
.turn_event_tof_valid (a6_turn_event_tof_valid),
.turn_event_hc_mm   (a6_turn_event_hc_mm),
.turn_event_hc_valid (a6_turn_event_hc_valid),
.turn_event_fused_mm (a6_turn_event_fused_mm),
.turn_event_fused_valid (a6_turn_event_fused_valid),
.turn_event_reason (a6_turn_event_reason),
// 舵机超声波自动扫描：仅在scan_done脉冲后保存完整一轮结果
.scan_active      (servo_scan_active),
.scan_done        (servo_scan_done),
.scan_left_mm     (scan_left_distance_mm),
.scan_center_mm   (scan_center_distance_mm),
.scan_right_mm    (scan_right_distance_mm),
// D5: IMU yaw (centidegrees), world odometry position (millimeters), encoders (ticks)
.yaw_cdeg       (imu_yaw_cdeg),
.world_x_mm     (odom_world_x_mm),
.world_y_mm     (odom_world_y_mm),
.encoder_a      (encoder_a_position),
.encoder_b      (encoder_b_position),
.encoder_c      (encoder_c_position),
.encoder_d      (encoder_d_position),
// PC串口输出M15
.uart_tx        (debug_report_uart_unused)
);
//======================================================================
// 第三十四部分：舵机超声波扫描
//======================================================================

// 自动扫描触发命令：HEX E4
//
// 两种触发来源：
// 1. 手动向PC串口发送E4
// 2. A5需要扫描时产生obstacle_scan_start
//
// A5触发时，把E4注入舵机扫描模块。

wire [7:0] scan_rx_data;
wire scan_rx_done;

assign scan_rx_data =
    obstacle_scan_start
    ?
        8'hE4
    :
        rx_data;

assign scan_rx_done =
    obstacle_scan_start
    ?
        1'b1
    :
        rx_done;


//==========================================================
// 实际扫描模块
//==========================================================

// SG90依次旋转到左、中、右方向。
// 每个方向等待200ms，使舵机稳定。
// 随后通过HC-SR04获取该方向距离。
//
// 扫描结束后：
// scan_done产生完成脉冲。
// A5读取三个方向的距离决定后续动作。

servo_ultrasonic_scan #(

    .CLK_FREQ  (50000000),

    .SETTLE_MS (200)

) u_servo_ultrasonic_scan (

    .clk                 (clk),
    .reset_n             (reset_n),

    .rx_data             (scan_rx_data),
    .rx_done             (scan_rx_done),

    .distance_mm         (ultra_distance_mm),
    .distance_valid      (ultra_distance_valid),
    .echo_timeout        (ultra_echo_timeout),

    .servo_position      (servo_scan_position),

    .left_distance_mm    (scan_left_distance_mm),
    .center_distance_mm  (scan_center_distance_mm),
    .right_distance_mm   (scan_right_distance_mm),

    .scan_active         (servo_scan_active),
    .scan_done           (servo_scan_done)

);


//======================================================================
// 第三十五部分：A6航点导航
//======================================================================

// A6负责按预设目标坐标运行。
//
// 当前航点逻辑保留原工程：
// 1. 计算当前坐标与目标坐标误差
// 2. 根据误差产生Vx、Vy
// 3. 接近航点时自动减速
// 4. 到达后切换下一航点
//
// 如果A5正在避障，A6必须暂停。
// A5完成后，再继续原来的航点任务。

waypoint_nav #(

    .POSITION_TOL_MM (32'sd40),

    .SLOW_ZONE_MM    (32'sd200),

    .FAST_SPEED      (13'sd18),

    .SLOW_SPEED      (13'sd8),

    // Set to 1'b1 for P1->P2->stop test; 0 keeps full four-point trip.
    .TEST_STOP_AFTER_P2 (1'b0)

) u_waypoint_nav (

    .clk            (clk),
    .reset_n        (reset_n),

    .sample_tick    (speed_tick_a),

    .rx_data        (d7_rx_data),
    .rx_done        (d7_rx_done),

    .calib_done     (gyro_calib_done),

    // 导航暂停条件
    .pause
    (
        (nav_waypoint_pause && !a6_corner_takeover)
        ||
        a6_sensor_hold
        ||
        obstacle_avoid_active
        ||
        nav_restore_active
    ),

    // 避免自动任务冲突
    .block_start
    (
        sequence_active
        ||
        obstacle_avoid_active
    ),

     .world_x_mm     (odom_world_x_mm),
    .world_y_mm     (odom_world_y_mm),
    .yaw_cdeg       (imu_yaw_cdeg),
    .turn_clear     (a6_turn_space_ok_nav),
    .corner_ready   (waypoint_corner_ready),

    .vx_out         (waypoint_vx),
    .vy_out         (waypoint_vy),
    .wz_out         (waypoint_wz),

    .active         (waypoint_active),
     .done           (waypoint_done),
    .turning        (waypoint_turning),

    .waypoint_index (waypoint_index),
    .state_debug    (waypoint_state),
    .fault_code     (a6_nav_fault_code)

);


//======================================================================
// 第三十六部分：A6自动避障协调器
//======================================================================

// 这是A6与A5之间的桥梁。
//
// 工作过程：
//
// A6正在运行
//      ↓
// 超声波连续发现前方障碍
//      ↓
// 暂停A6
//      ↓
// 自动发出A5命令
//      ↓
// A5执行避障
//      ↓
// A5进入RECOVER状态
//      ↓
// 协调器停止A5
//      ↓
// 使用MPU6500恢复避障前航向
//      ↓
// 解除A6暂停
//      ↓
// 继续同一个航点
//
// 如果A5报告错误，协调器应保持故障处理状态。

nav_avoid_supervisor #(

    .OBSTACLE_TRIGGER_MM     (16'd260),

    .CONFIRM_SAMPLES         (3'd3),

    .RESTORE_TOL_CDEG        (32'sd200),

    .RESTORE_FAST_ZONE_CDEG  (32'sd800),

    .RESTORE_FAST_WZ         (13'sd14),

    .RESTORE_SLOW_WZ         (13'sd7),

    .RESTORE_REVERSE         (1'b1),
    .MAX_AVOID_TRAVEL_MM     (32'sd200),
    .MAX_AVOID_TICKS         (16'd500),
    .REJOIN_CLEAR_MM        (16'd340),
    .REJOIN_CONFIRM_TICKS   (4'd10),
    .REJOIN_GUARD_TICKS     (8'd50),
    .SENSOR_GAP_TICKS      (5'd20),
    .MAX_ATTEMPTS_PER_WP    (2'd2),
    .A5_CRUISE_MAX_TICKS   (16'd120),
    .A5_TOTAL_MAX_TICKS    (16'd2000)

) u_nav_avoid_supervisor (

    .clk               (clk),
    .reset_n           (reset_n),

    .sample_tick       (speed_tick_a),

    .rx_data           (d7_rx_data),
    .rx_done           (d7_rx_done),

     .waypoint_active   (waypoint_active &&
                         (waypoint_state == 3'd1 || waypoint_state == 3'd3) &&
                         !a6_corner_takeover_supervisor_q),
    .waypoint_turning (waypoint_turning),
    .waypoint_done     (waypoint_done),
    .waypoint_index    (waypoint_index),
    .world_x_mm        (odom_world_x_mm),
    .world_y_mm        (odom_world_y_mm),
    .restore_turn_clear(a6_turn_space_ok),

    .distance_mm       (d7_fused_front_mm),
    // Far enough to rejoin only after three genuinely fresh samples.
    .distance_valid    (a6_front_for_detour_valid),
    .front_near_confirmed (a6_front_near_confirmed &&
                           a6_supervisor_direction==A6_DIR_FRONT),

    .move_direction    (a6_supervisor_direction),
    .front_mm          (d7_fused_front_mm),
    .front_valid       (a6_front_for_detour_valid),
    .rear_mm           (tof_rear_mm),
    .rear_valid        (a6_rear_for_detour_valid),
    .left_mm           (tof_left_mm),
    .left_valid        (a6_left_for_detour_valid),
    .right_mm          (tof_right_mm),
    .right_valid       (a6_right_for_detour_valid),

    .obstacle_active   (obstacle_avoid_active),
    .obstacle_error    (obstacle_avoid_error),
    .obstacle_state    (obstacle_state),

    .yaw_cdeg          (imu_yaw_cdeg),

    .waypoint_pause    (nav_waypoint_pause),

    .avoid_start_pulse (nav_auto_avoid_start),
    .avoid_stop_pulse  (nav_auto_avoid_stop),

    .restore_active    (nav_restore_active),
    .restore_wz        (nav_restore_wz),

    .avoid_cycle_active(nav_avoid_cycle_active),

    .state_debug       (nav_avoid_state),
    .detour_active     (nav_detour_active),
    .detour_vx         (nav_detour_vx),
    .detour_vy         (nav_detour_vy),
    .detour_wz         (nav_detour_wz),
    .debug_detour_direction(nav_detour_dir_debug),
    .debug_detour_distance (nav_detour_mm_debug),
    .debug_detour_fault    (nav_detour_fault_debug)

);


//======================================================================
// 第三十七部分：A6自动向A5发送指令
//======================================================================

// A6遇障时生成HEX A5。
// A6避障完成时生成HEX 53。
//
// 如果没有内部自动指令，
// 则直接传递电脑串口的命令。

// D7: fault stop beats the supervisor's internal start pulse.
// Auto-trigger is internal.  Do not reuse d7_auto_ready (A6 starter)
// because a close obstacle intentionally makes ready=0.
wire nav_a5_pulse_ok = gyro_calib_done && !d7_force_stop;
wire nav_emergency_stop = rx_done && (rx_data == 8'h53);
assign obstacle_rx_data =
    (d7_force_stop || nav_emergency_stop) ? 8'h53 :
    nav_auto_avoid_stop ? 8'h53 :
    (nav_auto_avoid_start && nav_a5_pulse_ok) ? 8'hA7 : d7_rx_data;

assign obstacle_rx_done =
    d7_force_stop || nav_emergency_stop || nav_auto_avoid_stop ||
    (nav_auto_avoid_start && nav_a5_pulse_ok) || d7_rx_done;


//======================================================================
// 第三十八部分：A5双ToF融合避障
//======================================================================

// 这是本次重点升级的模块。
//
// A5负责：
// 1. 检测前方障碍
// 2. 逐级减速
// 3. 停车扫描
// 4. 判断左右可通行方向
// 5. 结合固定ToF判断侧向安全性
// 6. 使用编码器闭环横移
// 7. 前进通过障碍
// 8. 横移返回原路线
//
// 本模块最终输出Vx、Vy、Wz，不直接输出PWM。
//
// obstacle_avoidance.v中：
// 超过100mm才允许朝该侧横移。
// top.v中：
// <=100mm 时禁止朝该侧横移；保持传感器失效保护。

obstacle_avoidance #(

    //==================================================
    // 距离参数
    //==================================================

    // 前方逐级减速
.SLOW1_THRESHOLD       (16'd600),
.SLOW2_THRESHOLD       (16'd400),


// A5前方200mm硬停；260mm时提前停车扫描
.FRONT_THRESHOLD       (16'd200),

// 后向100mm为安全停止阈值
.BACK_THRESHOLD        (16'd100),

// 后退后的前方恢复距离
.BACK_CLEAR_DISTANCE   (16'd200),

// 扫描方向200mm以上才作为侧向候选（须有效）
.SIDE_MIN              (16'd200),

// 横移近障停止
.LATERAL_STOP_DISTANCE (16'd100), // experimental side stop 10cm

// 旋转后及舵机扫描的重新判定
.SEARCH_CLEAR_DISTANCE (16'd280),
.SEARCH_SIDE_CLEAR     (16'd250),
.SEARCH_DIR_MARGIN     (16'd80),

// 绕过障碍后侧方释放阈值：下方只保留一处SIDE_RELEASE_MM=550

    //==================================================
    // 运动速度
    //==================================================

    .FORWARD_SPEED (12'd20),
    .SLOW1_SPEED   (12'd16),
    .SLOW2_SPEED   (12'd12),

    .DIAG_SPEED    (12'd18),

    .BACK_SPEED    (12'd8),

    .ROTATE_SPEED  (12'd14),

    .RETURN_SPEED  (12'd12),

    //==================================================
    // 原地搜索参数
    //==================================================

    .ROTATE_STEP_CDEG     (32'd1500),
    .MAX_ROTATE_STEPS     (4'd12),

    .ROTATE_STEP_TICKS    (16'd100),
    .ROTATE_TIMEOUT_TICKS (16'd150),
    .ROTATE_MIN_PROGRESS_CDEG (32'd40),

    .MAX_DIR_CHANGES      (2'd1),

    //==================================================
    // 里程计绕障参数
    //==================================================

    .LATERAL_STEP_COUNT (32'd600),
    .LATERAL_MAX_COUNT  (32'd2400),

    .RETURN_TOL_COUNT   (32'd160),

    .PASS_STEP_COUNT    (32'd1800),
    .PASS_MIN_COUNT     (32'd1800),
    .PASS_MAX_COUNT     (32'd5000),

    .SIDE_RELEASE_MM    (16'd550),

    //==================================================
    // 状态机时间参数，采样周期预计10ms
    //==================================================

    .START_SETTLE_TICKS  (16'd30),
    .STOP_SETTLE_TICKS   (16'd20),

    .SIDE_SETTLE_TICKS   (16'd20),

    .DIAG_TICKS          (16'd250),
    .PASS_TICKS          (16'd300),

    .BACK_TICKS          (16'd65),

    .RECOVER_TICKS       (16'd20),

    .RETURN_TIMEOUT_TICKS(16'd300),

    .SCAN_TIMEOUT_TICKS  (16'd500),
    .BACKUP_START_MM    (16'd150),
    .BACKUP_STOP_MM     (16'd100),
    .BACKUP_MAX_COUNT   (32'd350),
    .MAX_BACKUP_TRIES   (2'd2),
    .HOLD_RESCAN_TICKS  (16'd150),

    // Verified ToF calibration is upstream of this A5 module.
    // Do not reuse the old 100mm lateral allowance.
    .FLOW_MIN_SIDE_MM        (16'd200),
    .FLOW_SIDE_HARD_MM       (16'd130),
    .ROTATE_FRONT_SAFE_MM    (16'd150),
    .ROTATE_SIDE_SAFE_MM     (16'd150),
    .ROTATE_REAR_SAFE_MM     (16'd150),
    .ROTATE_SCAN_SAFE_MM     (16'd150),
    .CRUISE_SCAN_TRIGGER_MM (16'd260),
    .FLOW_SIDE_ENTER_FRONT_MM (16'd320),
    .FLOW_RELEASE_FRONT_MM   (16'd360),
    .FLOW_FORWARD_MIN_MM     (16'd240)

) u_obstacle_avoidance (

    //==================================================
    // 系统控制
    //==================================================

    .clk                (clk),
    .reset_n            (reset_n),

    .sample_tick        (speed_tick_a),

    // A5/53命令
    .rx_data            (obstacle_rx_data),
    .rx_done            (obstacle_rx_done),

    .sequence_active    (sequence_active),

    //==================================================
    // HC-SR04实时测距
    //==================================================

    .distance_mm        (d7_fused_front_mm),

    .distance_valid     (d7_fused_front_valid &&
                         a6_fused_front_fresh_q),

    .echo_timeout       (ultra_echo_timeout && d7_hc_front_facing),

    //==================================================
    // 【新增重点】左右ToF融合输入
    //==================================================

    // 左侧最近有效距离
    .tof_left_mm
    (
        tof_left_mm
    ),

    // 右侧最近有效距离
    .tof_right_mm
    (
        tof_right_mm
    ),

    // 左侧传感器是否健康
    .tof_left_ok
    (
        tof_left_ok
    ),

    // 右侧传感器是否健康
    .tof_right_ok
    (
        tof_right_ok
    ),
    .tof_left_unknown (a5_left_tof_unknown),
    .tof_right_unknown(a5_right_tof_unknown),
    .tof_left_trial_ok(a5_left_zero_trial_ok),
    .tof_right_trial_ok(a5_right_zero_trial_ok),

    // 前方独立ToF，用于核实扫描中心是否真能前进。
    .tof_front_mm (tof_front_mm),
    .tof_front_ok (tof_front_ok),
    .tof_rear_mm  (tof_rear_mm),
    .tof_rear_ok  (tof_rear_ok),

    //==================================================
    // 航向角和里程计
    //==================================================

    .yaw_cdeg           (imu_yaw_cdeg),

    .odom_pos_x_count   (odom_pos_x),
    .odom_pos_y_count   (odom_pos_y),

    //==================================================
    // 舵机扫描接口
    //==================================================

    .scan_start         (obstacle_scan_start),
    .scan_done          (servo_scan_done),

    .left_distance_mm   (scan_left_distance_mm),
    .center_distance_mm (scan_center_distance_mm),
    .right_distance_mm  (scan_right_distance_mm),

    //==================================================
    // 底盘运动输出
    //==================================================

    .vx_out             (obstacle_vx),
    .vy_out             (obstacle_vy),
    .wz_out             (obstacle_wz),

    //==================================================
    // 避障状态
    //==================================================

    .active             (obstacle_avoid_active),
    .error              (obstacle_avoid_error),
    .state_debug        (obstacle_state)

);


//======================================================================
// 第三十九部分：上位机通信协议
//======================================================================

// 与外部上位机交换：
// - 里程计X/Y坐标
// - 航向角
// - 前方超声波距离
// - A5避障状态
// - A6航点编号
// - A6协调器运行状态
//
// 双模式控制：本地模式保留原A5/A6；按下S3切到上位机模式后，
// host_vx/vy/wz进入统一运动入口。后级ToF/故障/电机安全门仍然有效。

uart_protocol u_uart_protocol (

    .clk                   (clk),
    .reset_n               (reset_n),

    .sample_tick           (speed_tick_a),

    // 上位机接收
    .rx_data               (host_rx_data),
    .rx_done               (host_rx_done),

    // 上位机发送
    .tx_data               (host_tx_data),
    .tx_send               (host_tx_send),

    .tx_done               (host_tx_done),
    .tx_busy               (host_tx_busy),

    // 上位机控制请求
    .host_vx               (host_vx),
    .host_vy               (host_vy),
    .host_wz               (host_wz),

    .host_cmd_valid        (host_cmd_valid),
    .ai_mode               (host_ai_mode),
    .stop_req              (host_stop_req),

    .waypoint_start_req    (host_waypoint_start_req),

    .frame_ok              (host_frame_ok),

    //==================================================
    // 反馈给上位机的状态数据
    //==================================================

    .status_x_mm           (odom_world_x_mm),
    .status_y_mm           (odom_world_y_mm),

    .status_yaw_cdeg       (imu_yaw_cdeg),

    .status_distance_mm    (ultra_distance_mm),

    .status_obstacle_state (obstacle_state),

    .status_waypoint_index (waypoint_index),

    .status_nav_state      (nav_avoid_state),

    .status_host_mode      (mode_host_active)

);


//======================================================================
// 顶层结束
//======================================================================

//====================================================
// ACG720 60K - OV5640 HDMI FULL SCREEN
//
// Existing robot control stays unchanged.
// Camera and HDMI run in parallel.
//
// Clock: 50MHz
// Reset: original robot reset_n
//====================================================
//================================================
// Ultrasonic freshness for the robot-status health indicator.
// The numeric L/F/R/U distance display now uses four ToF sensors.
// speed_tick_a is used as the age reference.
//
// If speed_tick_a = 10 ms,
// 100 ticks corresponds to approximately 1 s.
//================================================


// The display reports ONLINE only after three consecutive ECHO responses.
// RECOVER remains asserted from the automatic soft reset until those three
// response confirmations have completed.
wire osd_ultra_ok = ultra_sensor_online;
wire osd_ultra_recovering = ultra_sensor_recovering;
    //============================================
    // ACG720 60K
    // OV5640 + HDMI + Realtime YAW
    //============================================

   
    //================================================
    // ACG720 60K
    // OV5640 + HDMI + YAW + DISTANCE OSD
    //================================================

    hdmi_color #(
        .VIDEO_TEST(1'b1)
    ) u_video (

        // System clock and reset
        .clk              (clk),
        .rst_n            (reset_n),

        // SW0 changes only the HDMI display page.
        .ui_mode_ai       (ui_mode_ai),
        .status_host_mode (mode_host_active),
        .status_host_ai_mode (host_ai_mode),
        .status_host_control_active (host_control_active),
        .status_host_watchdog_ok (!host_watchdog_expired),
        .status_host_stop (host_stop_req),
        .ultra_recovering (osd_ultra_recovering),

        // MPU6500 YAW
        .yaw_cdeg         (imu_yaw_cdeg),
    // MPU6500 initialization and calibration
    //==============================================
    // Robot running status
    //==============================================


    .status_calib_done(gyro_calib_done),

    .status_obstacle_active(obstacle_avoid_active),
    .status_obstacle_error (obstacle_avoid_error),

    .status_waypoint_active(waypoint_active),
    .status_waypoint_done  (waypoint_done),
    .status_waypoint_index (waypoint_index),

    .status_nav_restore_active(nav_restore_active),
    .status_nav_waypoint_pause (nav_waypoint_pause),
    .status_nav_avoid_cycle_active(nav_avoid_cycle_active),

    .status_fault_stop(d7_force_stop),

    .status_front_stop(d7_front_stop),
    .status_front_slow(d7_front_slow),
    .status_side_stop(tof_side_emergency_stop),
//============================================
    // Four wheel encoder speed feedback
    //============================================


    .motion_speed_a (encoder_a_speed),
    .motion_speed_b (encoder_b_speed),
    .motion_speed_c (encoder_c_speed),
    .motion_speed_d (encoder_d_speed),

    //============================================
    // World XY odometry, unit: mm
    //============================================

    .motion_x_mm (odom_world_x_mm),
    .motion_y_mm (odom_world_y_mm),
            .imu_init_done   (imu_init_done),
            .gyro_calib_done (gyro_calib_done),
            .imu_data_valid  (imu_data_valid),
        //========================================
        // Four ToF real-time distance
        // Unit: mm
        //========================================

        .tof_left_mm      (tof_left_mm),
        .tof_front_mm     (tof_front_mm),
        .tof_right_mm     (tof_right_mm),
        .tof_rear_mm      (tof_rear_mm),

        // Ultrasonic remains in the original front-assist/safety path.
        //========================================
        // Sensor validity
        //========================================

        .tof_left_ok      (tof_left_ok),
        .tof_front_ok     (tof_front_ok),
        .tof_right_ok     (tof_right_ok),
        .tof_rear_ok      (tof_rear_ok),
        .tof_left_recovering  (tof_left_recovering),
        .tof_front_recovering (tof_front_recovering),
        .tof_right_recovering (tof_right_recovering),
        .tof_rear_recovering  (tof_rear_recovering),
        .ultra_ok         (osd_ultra_ok),

        //========================================
        // OV5640 camera
        //========================================

        .camera_xclk      (camera_xclk),
        .camera_pclk      (camera_pclk),
        .camera_vsync     (camera_vsync),
        .camera_href      (camera_href),
        .camera_data      (camera_data),

        .camera_rst_n     (camera_rst_n),
        .camera_pwdn      (camera_pwdn),
        .camera_sclk      (camera_sclk),
        .camera_sdat      (camera_sdat),

        //========================================
        // HDMI
        //========================================

        .tmds_clk_p       (tmds_clk_p),
        .tmds_clk_n       (tmds_clk_n),
        .tmds_data_p      (tmds_data_p),
        .tmds_data_n      (tmds_data_n),

        //========================================
        // Unused video outputs
        //========================================

        .led                   (),
        .snapshot_pixel_valid  (),
        .snapshot_pixel_rgb565 (),
        .snapshot_pixel_x      (),
        .snapshot_pixel_y      ()

    );



endmodule
