`timescale 1ns / 1ps
// A6 -> A5 handoff supervisor.
// 0=MON, 1=WAIT_START, 2=A5_RUN, 3=WAIT_STOP,
// 4=HEADING, 5=FAULT_RECOVERY, 6=AUTO_RETRY_HOLD.
// A5 remains the only obstacle/detour planner.
// Recoverable timeouts/errors never latch navigation permanently.

module nav_avoid_supervisor #(
    parameter [15:0] OBSTACLE_TRIGGER_MM=16'd260,
    parameter [2:0] CONFIRM_SAMPLES=3'd3,
    parameter [15:0] REJOIN_CLEAR_MM=16'd340,
    parameter [3:0] REJOIN_CONFIRM_TICKS=4'd10,
    parameter [7:0] REJOIN_GUARD_TICKS=8'd50,
    parameter [4:0] SENSOR_GAP_TICKS=5'd20,
    parameter signed [31:0] MAX_AVOID_TRAVEL_MM=32'sd200,
    parameter [15:0] MAX_AVOID_TICKS=16'd500,
    parameter [15:0] MAX_RESTORE_TICKS=16'd500,
    parameter [15:0] A5_CRUISE_MAX_TICKS=16'd120,
    parameter [15:0] A5_TOTAL_MAX_TICKS=16'd2000,
    parameter [7:0] START_TIMEOUT_TICKS=8'd200,
    parameter [1:0] MAX_ATTEMPTS_PER_WP=2'd2,
    parameter [15:0] RETRY_WAIT_TICKS=16'd100,
    parameter [15:0] STOP_REPEAT_TICKS=16'd50,
    parameter signed [31:0] RESTORE_TOL_CDEG=32'sd200,
    parameter signed [31:0] RESTORE_FAST_ZONE_CDEG=32'sd800,
    parameter signed [12:0] RESTORE_FAST_WZ=13'sd14,
    parameter signed [12:0] RESTORE_SLOW_WZ=13'sd7,
    parameter RESTORE_REVERSE=1'b1
)(
    input wire clk, reset_n, sample_tick,
    input wire [7:0] rx_data,
    input wire rx_done,
    input wire waypoint_active, waypoint_done, waypoint_turning,
    input wire [2:0] waypoint_index,
    input wire signed [31:0] world_x_mm, world_y_mm,
    input wire [15:0] distance_mm,
    input wire distance_valid,
    input wire front_near_confirmed,
    input wire obstacle_active, obstacle_error,
    input wire [3:0] obstacle_state,
    input wire signed [31:0] yaw_cdeg,
    input wire restore_turn_clear,
    input wire [1:0] move_direction,
    input wire [15:0] front_mm,
    input wire front_valid,
    input wire [15:0] rear_mm,
    input wire rear_valid,
    input wire [15:0] left_mm,
    input wire left_valid,
    input wire [15:0] right_mm,
    input wire right_valid,

    output wire waypoint_pause,
    output reg avoid_start_pulse,
    output reg avoid_stop_pulse,
    output wire restore_active,
    output reg signed [12:0] restore_wz,
    output wire avoid_cycle_active,
    output wire [2:0] state_debug,
    output wire detour_active,
    output wire signed [12:0] detour_vx, detour_vy, detour_wz,
    output wire [1:0] debug_detour_direction,
    output wire [15:0] debug_detour_distance,
    output reg [2:0] debug_detour_fault
);

    localparam [2:0]
        MON       = 3'd0,
        WAIT_START= 3'd1,
        A5_RUN    = 3'd2,
        WAIT_STOP = 3'd3,
        HEADING   = 3'd4,
        FAULT     = 3'd5,
        HOLD      = 3'd6;

    reg [2:0] state;
    reg [2:0] near_count;
    reg [3:0] clear_count;

    reg [7:0] start_ticks;
    reg [7:0] guard_ticks;

    reg [15:0] operation_ticks;
    reg [15:0] a5_cruise_ticks;

    //========================================================
    // Timing optimization 1: retry counter width
    // Default RETRY_WAIT_TICKS = 100, only 7 bits required.
    //========================================================

    localparam integer RETRY_TICK_WIDTH =
        (RETRY_WAIT_TICKS <= 1) ? 1 : $clog2(RETRY_WAIT_TICKS);

    reg [RETRY_TICK_WIDTH-1:0] retry_ticks;

    //========================================================
    // Timing optimization 2: HOLD condition predecode
    //========================================================

    reg hold_resume_enable_q;
    reg hold_retry_enable_q;
    reg hold_stop_needed_q;

    reg [1:0] attempts;
    reg [2:0] previous_wp;

    reg signed [31:0] saved_yaw;

    reg a5_bypass_verified;

    assign waypoint_pause = (state != MON);
    assign avoid_cycle_active = (state != MON);
    assign restore_active = (state == HEADING);
    assign state_debug = state;

    // A5 is the only detour planner.
    assign detour_active = 1'b0;
    assign detour_vx = 13'sd0;
    assign detour_vy = 13'sd0;
    assign detour_wz = 13'sd0;

    assign debug_detour_direction = move_direction;
    assign debug_detour_distance = 16'd0;

    // Planner-only predicates. The complete validity/range expression is
    // sampled together every clk, never on sample_tick. Final motor safety
    // continues to use the original live signals in top.v.
    reg near_front_q, clear_front_q, blocked_front_q;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            near_front_q <= 1'b0;
            clear_front_q <= 1'b0;
            blocked_front_q <= 1'b0;
        end else begin
            near_front_q <= front_near_confirmed && distance_valid &&
                (distance_mm != 16'd0) &&
                (distance_mm <= OBSTACLE_TRIGGER_MM);
            clear_front_q <= distance_valid &&
                (distance_mm >= REJOIN_CLEAR_MM);
            blocked_front_q <= distance_valid &&
                (distance_mm < REJOIN_CLEAR_MM);
        end
    end
    wire near_front = near_front_q;
    wire clear_front = clear_front_q;

    wire a5_finished =
        obstacle_active &&
        (obstacle_state == 4'd12);

    //========================================================
    // YAW restore comparison pipeline (50 MHz)
    //
    // yaw_integrator wraps yaw_cdeg to approximately +/-18000
    // centidegrees.  The difference of two valid headings is
    // therefore in [-36000, +36000] and fits in signed 17 bits.
    // Narrowing the arithmetic avoids a 32-bit subtract/wrap/abs/
    // compare chain feeding the navigation state-machine counters.
    //
    // Pipeline stages:
    //   1) 17-bit heading difference
    //   2) shortest-angle wrap + circular tolerance predecode
    //   3) absolute error and direction flags
    //
    // The supervisor state machine advances on sample_tick (much
    // slower than clk), so the 3-cycle / 60 ns status latency does
    // not change the 10 ms navigation decision cadence.
    //========================================================

    localparam signed [16:0] YAW_HALF_TURN_CDEG = 17'sd18000;
    localparam signed [16:0] YAW_FULL_TURN_CDEG = 17'sd36000;
    localparam signed [16:0] RESTORE_TOL_CDEG_17 =
        RESTORE_TOL_CDEG[16:0];
    localparam [16:0] RESTORE_FAST_ZONE_CDEG_17 =
        RESTORE_FAST_ZONE_CDEG[16:0];

    reg signed [16:0] yaw_delta_raw_q;
    reg signed [16:0] yaw_delta_short_q;
    reg [16:0] yaw_abs_delta_q;
    reg yaw_delta_positive_q;
    reg heading_ok_stage2_q;
    reg heading_ok_q;

    // Stage 1: narrow signed subtraction.
    // Both source values originate from the +/-180 degree yaw integrator.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            yaw_delta_raw_q <= 17'sd0;
        end else begin
            yaw_delta_raw_q <=
                $signed(saved_yaw[16:0]) - $signed(yaw_cdeg[16:0]);
        end
    end

    // Stage 2: shortest-angle wrap and circular tolerance check.
    // The three intervals represent |shortest_delta| <= tolerance:
    //   [-tol, +tol], [36000-tol, 36000], [-36000, -36000+tol].
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            yaw_delta_short_q <= 17'sd0;
            heading_ok_stage2_q <= 1'b1;
        end else begin
            if (yaw_delta_raw_q > YAW_HALF_TURN_CDEG)
                yaw_delta_short_q <=
                    yaw_delta_raw_q - YAW_FULL_TURN_CDEG;
            else if (yaw_delta_raw_q < -YAW_HALF_TURN_CDEG)
                yaw_delta_short_q <=
                    yaw_delta_raw_q + YAW_FULL_TURN_CDEG;
            else
                yaw_delta_short_q <= yaw_delta_raw_q;

            heading_ok_stage2_q <=
                ((yaw_delta_raw_q >= -RESTORE_TOL_CDEG_17) &&
                 (yaw_delta_raw_q <=  RESTORE_TOL_CDEG_17)) ||
                ((yaw_delta_raw_q >=
                    (YAW_FULL_TURN_CDEG - RESTORE_TOL_CDEG_17)) &&
                 (yaw_delta_raw_q <= YAW_FULL_TURN_CDEG)) ||
                ((yaw_delta_raw_q <=
                    (-YAW_FULL_TURN_CDEG + RESTORE_TOL_CDEG_17)) &&
                 (yaw_delta_raw_q >= -YAW_FULL_TURN_CDEG));
        end
    end

    // Stage 3: absolute heading error and direction, aligned with
    // the registered heading_ok flag.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            yaw_abs_delta_q <= 17'd0;
            yaw_delta_positive_q <= 1'b1;
            heading_ok_q <= 1'b1;
        end else begin
            if (yaw_delta_short_q[16]) begin
                yaw_abs_delta_q <= -yaw_delta_short_q;
                yaw_delta_positive_q <= 1'b0;
            end else begin
                yaw_abs_delta_q <= yaw_delta_short_q;
                yaw_delta_positive_q <= 1'b1;
            end
            heading_ok_q <= heading_ok_stage2_q;
        end
    end

    wire [16:0] abs_delta = yaw_abs_delta_q;
    wire heading_ok = heading_ok_q;

    //========================================================
    // HOLD decision predecode (50MHz).
    //========================================================

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin

            hold_resume_enable_q <= 1'b0;
            hold_retry_enable_q  <= 1'b0;
            hold_stop_needed_q   <= 1'b0;

        end else begin

            // Successful bypass + heading + front clearance.
            hold_resume_enable_q <=
                (state == HOLD) &&
                a5_bypass_verified &&
                clear_front &&
                heading_ok &&
                !obstacle_active;

            // Waypoint active and A5 has stopped.
            hold_retry_enable_q <=
                (state == HOLD) &&
                waypoint_active &&
                !waypoint_done &&
                !waypoint_turning &&
                !obstacle_active;

            // A5 is still active: keep issuing STOP.
            hold_stop_needed_q <=
                (state == HOLD) &&
                waypoint_active &&
                !waypoint_done &&
                !waypoint_turning &&
                obstacle_active;

        end

    end

    //========================================================
    // 航向恢复控制
    //========================================================

    always @(*) begin

        restore_wz = 13'sd0;

        if(state == HEADING &&
           restore_turn_clear &&
           !heading_ok) begin

            if(yaw_delta_positive_q)

                restore_wz = RESTORE_REVERSE ?
                    ((abs_delta > RESTORE_FAST_ZONE_CDEG_17) ?
                        -RESTORE_FAST_WZ :
                        -RESTORE_SLOW_WZ) :
                    ((abs_delta > RESTORE_FAST_ZONE_CDEG_17) ?
                        RESTORE_FAST_WZ :
                        RESTORE_SLOW_WZ);

            else

                restore_wz = RESTORE_REVERSE ?
                    ((abs_delta > RESTORE_FAST_ZONE_CDEG_17) ?
                        RESTORE_FAST_WZ :
                        RESTORE_SLOW_WZ) :
                    ((abs_delta > RESTORE_FAST_ZONE_CDEG_17) ?
                        -RESTORE_FAST_WZ :
                        -RESTORE_SLOW_WZ);

        end

    end

    //========================================================
    // 主状态机
    //========================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= MON;

            near_count <= 0;
            clear_count <= 0;

            start_ticks <= 0;
            guard_ticks <= 0;

            operation_ticks <= 0;
            a5_cruise_ticks <= 0;
            retry_ticks <= 0;

            attempts <= 0;
            previous_wp <= 0;

            saved_yaw <= 0;

            a5_bypass_verified <= 1'b0;

            avoid_start_pulse <= 0;
            avoid_stop_pulse <= 0;

            debug_detour_fault <= 0;

        end else begin

            avoid_start_pulse <= 0;
            avoid_stop_pulse <= 0;

            //================================================
            // 外部停车复位
            //================================================

            if(rx_done && rx_data == 8'h53) begin

                state <= MON;

                near_count <= 0;
                clear_count <= 0;

                start_ticks <= 0;
                guard_ticks <= 0;

                operation_ticks <= 0;
                a5_cruise_ticks <= 0;
                retry_ticks <= 0;

                attempts <= 0;

                debug_detour_fault <= 0;

                a5_bypass_verified <= 1'b0;

            end

            else if(sample_tick) begin

                case(state)

                //============================================
                // 0. 正常导航监测
                //============================================

                MON: begin

                    operation_ticks <= 0;
                    retry_ticks <= 0;

                    if(previous_wp != waypoint_index) begin

                        previous_wp <= waypoint_index;

                        attempts <= 0;
                        guard_ticks <= 0;
                        near_count <= 0;

                    end else begin

                        if(guard_ticks != 0)
                            guard_ticks <= guard_ticks - 1'b1;

                        if(!waypoint_active ||
                           waypoint_done ||
                           waypoint_turning ||
                           obstacle_active) begin

                            near_count <= 0;

                        end

                        else if(near_front && guard_ticks == 0) begin

                            if(near_count >= CONFIRM_SAMPLES-1'b1) begin

                                near_count <= 0;

                                if(attempts >= MAX_ATTEMPTS_PER_WP) begin

                                    state <= HOLD;

                                    retry_ticks <= 0;
                                    clear_count <= 0;

                                    debug_detour_fault <= 3'd5;

                                end else begin

                                    saved_yaw <= yaw_cdeg;

                                    a5_bypass_verified <= 1'b0;

                                    avoid_start_pulse <= 1'b1;

                                    attempts <= attempts + 1'b1;

                                    start_ticks <= 0;
                                    operation_ticks <= 0;

                                    state <= WAIT_START;

                                    debug_detour_fault <= 0;

                                end

                            end else begin

                                near_count <= near_count + 1'b1;

                            end

                        end else begin

                            near_count <= 0;

                        end

                    end

                end

                //============================================
                // 1. 等待A5启动
                //============================================

                WAIT_START: begin

                    if(obstacle_active) begin

                        operation_ticks <= 0;
                        a5_cruise_ticks <= 0;

                        state <= A5_RUN;

                    end

                    else if(start_ticks >= START_TIMEOUT_TICKS) begin

                        debug_detour_fault <= 3'd4;

                        a5_bypass_verified <= 1'b0;

                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;
                        retry_ticks <= 0;

                        state <= WAIT_STOP;

                    end

                    else begin

                        start_ticks <= start_ticks + 1'b1;

                        if(start_ticks == 8'd50 ||
                           start_ticks == 8'd100)

                            avoid_start_pulse <= 1'b1;

                    end

                end

                //============================================
                // 2. A5执行避障
                //============================================

                A5_RUN: begin

                    if(obstacle_state == 4'd2 &&
                       obstacle_active) begin

                        if(a5_cruise_ticks < 16'hffff)
                            a5_cruise_ticks <=
                                a5_cruise_ticks + 1'b1;

                    end else begin

                        a5_cruise_ticks <= 0;

                    end

                    // A5异常：停止并进入恢复流程。
                    if(obstacle_error) begin

                        debug_detour_fault <= 3'd4;

                        a5_bypass_verified <= 1'b0;

                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;
                        retry_ticks <= 0;

                        state <= WAIT_STOP;

                    end

                    // A5完成绕障。
                    else if(a5_finished) begin

                        a5_bypass_verified <= 1'b1;

                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;
                        a5_cruise_ticks <= 0;

                        state <= WAIT_STOP;

                    end

                    // A5运行超时。
                    else if(obstacle_active &&
                        ((obstacle_state == 4'd2 &&
                          a5_cruise_ticks >= A5_CRUISE_MAX_TICKS) ||
                         operation_ticks >= A5_TOTAL_MAX_TICKS)) begin

                        a5_bypass_verified <= 1'b0;

                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;
                        a5_cruise_ticks <= 0;
                        retry_ticks <= 0;

                        debug_detour_fault <= 3'd3;

                        state <= WAIT_STOP;

                    end

                    // A5意外退出。
                    else if(!obstacle_active) begin

                        debug_detour_fault <= 3'd2;

                        a5_bypass_verified <= 1'b0;

                        operation_ticks <= 0;
                        retry_ticks <= 0;
                        clear_count <= 0;

                        state <= HOLD;

                    end

                    else begin

                        operation_ticks <=
                            operation_ticks + 1'b1;

                    end

                end

                //============================================
                // 3. 等待A5真正停止
                //============================================

                WAIT_STOP: begin

                    if(!obstacle_active) begin

                        operation_ticks <= 0;
                        retry_ticks <= 0;
                        clear_count <= 0;

                        if(a5_bypass_verified)
                            state <= HEADING;
                        else
                            state <= HOLD;

                        if(!a5_bypass_verified)
                            debug_detour_fault <= 3'd3;

                    end

                    else if(operation_ticks >=
                            STOP_REPEAT_TICKS-1'b1) begin

                        // 每500ms重新发出STOP。
                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;

                        debug_detour_fault <= 3'd2;

                    end

                    else begin

                        operation_ticks <=
                            operation_ticks + 1'b1;

                    end

                end

                //============================================
                // 4. 恢复原航向
                //============================================

                HEADING: begin

                    if(operation_ticks >= MAX_RESTORE_TICKS) begin

                        debug_detour_fault <=
                            heading_ok ? 3'd4 : 3'd2;

                        state <= HOLD;

                        operation_ticks <= 0;
                        retry_ticks <= 0;
                        clear_count <= 0;

                    end

                    else begin

                        operation_ticks <=
                            operation_ticks + 1'b1;

                        if(heading_ok && clear_front) begin

                            if(clear_count >=
                               REJOIN_CONFIRM_TICKS-1'b1) begin

                                state <= MON;

                                guard_ticks <= REJOIN_GUARD_TICKS;

                                near_count <= 0;
                                clear_count <= 0;

                                retry_ticks <= 0;
                                operation_ticks <= 0;

                                debug_detour_fault <= 0;

                            end

                            else begin

                                clear_count <=
                                    clear_count + 1'b1;

                            end

                        end

                        else if(heading_ok &&
                                blocked_front_q &&
                                operation_ticks >= 16'd50) begin

                            debug_detour_fault <= 3'd4;

                            clear_count <= 0;

                            state <= HOLD;

                            operation_ticks <= 0;
                            retry_ticks <= 0;

                        end

                        else begin

                            clear_count <= 0;

                        end

                    end

                end

                //============================================
                // 6. 自动恢复，不永久锁死
                //============================================

                HOLD: begin

                    // 成功绕障并确认安全，恢复A6。
                    if(hold_resume_enable_q) begin

                        retry_ticks <= 0;

                        if(clear_count >=
                           REJOIN_CONFIRM_TICKS-1'b1) begin

                            state <= MON;

                            guard_ticks <= REJOIN_GUARD_TICKS;

                            clear_count <= 0;
                            near_count <= 0;

                            operation_ticks <= 0;

                            debug_detour_fault <= 0;

                        end else begin

                            clear_count <= clear_count + 1'b1;

                        end

                    end else begin

                        clear_count <= 0;

                        // 重试计数器只由预译码寄存器控制。
                        if(!hold_retry_enable_q) begin

                            retry_ticks <= 0;

                            // A5仍在运行时，继续发送STOP。
                            if(hold_stop_needed_q) begin

                                if(operation_ticks >=
                                   STOP_REPEAT_TICKS-1'b1) begin

                                    avoid_stop_pulse <= 1'b1;
                                    operation_ticks <= 0;

                                end else begin

                                    operation_ticks <=
                                        operation_ticks + 1'b1;

                                end

                            end

                        end

                        // 冷却结束，自动重新执行A5。
                        else if(retry_ticks >=
                                RETRY_WAIT_TICKS-1'b1) begin

                            // 保留最初遇障时的saved_yaw。
                            a5_bypass_verified <= 1'b0;

                            attempts <= 0;

                            start_ticks <= 0;

                            operation_ticks <= 0;
                            a5_cruise_ticks <= 0;
                            retry_ticks <= 0;

                            avoid_start_pulse <= 1'b1;

                            state <= WAIT_START;

                        end else begin

                            retry_ticks <= retry_ticks + 1'b1;

                        end

                    end

                end

                //============================================
                // 5. 旧FAULT状态改为自动恢复状态
                //============================================

                FAULT: begin

                    if(!obstacle_active) begin

                        state <= HOLD;

                        operation_ticks <= 0;
                        retry_ticks <= 0;
                        clear_count <= 0;

                    end

                    else if(operation_ticks >=
                            STOP_REPEAT_TICKS-1'b1) begin

                        avoid_stop_pulse <= 1'b1;

                        operation_ticks <= 0;

                    end

                    else begin

                        operation_ticks <=
                            operation_ticks + 1'b1;

                    end

                end

                default: begin

                    state <= HOLD;

                    operation_ticks <= 0;
                    retry_ticks <= 0;
                    clear_count <= 0;

                    debug_detour_fault <= 3'd4;

                    if(obstacle_active)
                        avoid_stop_pulse <= 1'b1;

                end

                endcase

            end

        end

    end

    wire unused=^{world_x_mm,world_y_mm,waypoint_index,
        front_mm,front_valid,rear_mm,rear_valid,left_mm,left_valid,
        right_mm,right_valid,SENSOR_GAP_TICKS,MAX_AVOID_TRAVEL_MM,
        MAX_AVOID_TICKS};

endmodule