// ================================================================
// 2026 competition task 3 - A5 safety refinement (experimental)
// Focus: independent axis safety / calibrated ToF / monotone braking.
// Never bypass P&R timing checks and low-speed physical validation.
// ================================================================
`timescale 1ns / 1ps
// UNKNOWN-SENSOR UPDATE (2026-10-26):
// Four ToF validity flags are independent of obstacle detection.
// Missing ToF does not stop forward cruise if fresh center HC-SR04
// confirms space.  HC echo cached for 200ms, with stale-data speed cap.
// If BOTH forward sources are unknown, A5 stays armed and retries; no
// blind motion is commanded. Sideways/backward motion requires reliable
// clearance in that movement direction.
// ============================================================
// A5: continuous cruise hysteresis fix (ACG720 / Verilog-2001)
// Preserves original top.v ports / parameters / state debug codes.
// IMPORTANT: Reactive navigation only, NOT SLAM or a global map.
// Direction: vx+ forward, vx- backward, vy+ right, vy- left.
// sample_tick must be approximately 10 ms.
// ============================================================
module obstacle_avoidance #(
    parameter [15:0] SLOW1_THRESHOLD = 16'd800,
    parameter [15:0] SLOW2_THRESHOLD = 16'd600,
    parameter [15:0] FRONT_THRESHOLD = 16'd200,
    parameter [15:0] BACK_THRESHOLD = 16'd100,
    parameter [15:0] BACK_CLEAR_DISTANCE = 16'd200,
    parameter [15:0] SIDE_MIN = 16'd200,
    parameter [15:0] LATERAL_STOP_DISTANCE = 16'd100,
    parameter [15:0] SEARCH_CLEAR_DISTANCE = 16'd280,
    parameter [15:0] SEARCH_SIDE_CLEAR = 16'd250,
    parameter [15:0] SEARCH_DIR_MARGIN = 16'd120,
    parameter [11:0] FORWARD_SPEED = 12'd20,
    parameter [11:0] SLOW1_SPEED = 12'd16,
    parameter [11:0] SLOW2_SPEED = 12'd12,
    parameter [11:0] DIAG_SPEED = 12'd18,
    parameter [11:0] BACK_SPEED = 12'd8,
    parameter [11:0] ROTATE_SPEED = 12'd14,
    parameter [11:0] RETURN_SPEED = 12'd12,
    parameter [31:0] ROTATE_STEP_CDEG = 32'd1500,
    parameter [3:0] MAX_ROTATE_STEPS = 4'd12,
    parameter [15:0] ROTATE_STEP_TICKS = 16'd100,
    parameter [15:0] ROTATE_TIMEOUT_TICKS = 16'd150,
    parameter [31:0] ROTATE_MIN_PROGRESS_CDEG = 32'd40,
    parameter [1:0] MAX_DIR_CHANGES = 2'd1,
    parameter [31:0] DEADEND_MIN_EXIT_CDEG = 32'd6000,
    parameter [31:0] DEADEND_TURN_LIMIT_CDEG = 32'd17500,
    parameter [31:0] LATERAL_STEP_COUNT = 32'd600,
    parameter [31:0] LATERAL_MAX_COUNT = 32'd2400,
    parameter [31:0] RETURN_TOL_COUNT = 32'd160,
    parameter [31:0] PASS_STEP_COUNT = 32'd1800,
    parameter [31:0] PASS_MIN_COUNT = 32'd1800,
    parameter [31:0] PASS_MAX_COUNT = 32'd5000,
    parameter [15:0] SIDE_RELEASE_MM = 16'd550,
    parameter [15:0] START_SETTLE_TICKS = 16'd30,
    parameter [15:0] STOP_SETTLE_TICKS = 16'd20,
    parameter [15:0] SIDE_SETTLE_TICKS = 16'd20,
    parameter [15:0] DIAG_TICKS = 16'd250,
    parameter [15:0] PASS_TICKS = 16'd300,
    parameter [15:0] BACK_TICKS = 16'd50,
    parameter [15:0] RECOVER_TICKS = 16'd20,
    parameter [15:0] RETURN_TIMEOUT_TICKS = 16'd300,
    parameter [15:0] SCAN_TIMEOUT_TICKS = 16'd500,
    parameter [15:0] BACKUP_START_MM = 16'd150,
    parameter [15:0] BACKUP_STOP_MM = 16'd100,
    parameter [31:0] BACKUP_MAX_COUNT = 32'd350,
    parameter [1:0] MAX_BACKUP_TRIES = 2'd2,
    parameter [15:0] HOLD_RESCAN_TICKS = 16'd300,
    parameter [15:0] ROTATE_FRONT_SAFE_MM = 16'd150,
    parameter [15:0] ROTATE_SIDE_SAFE_MM = 16'd150,
    // Kept for top.v parameter compatibility; NOT a mandatory 150-mm
    // rotation-entry condition. Reverse driving still uses BACKUP_* guards.
    parameter [15:0] ROTATE_REAR_SAFE_MM = 16'd150,
    // Emergency-only rear veto during in-place turn: if the rear TOF
    // actually reports <=100 mm, rotation stops; invalid rear samples
    // do NOT prevent choosing a turn direction.
    parameter [15:0] ROTATE_REAR_VETO_MM = 16'd100,
    parameter [15:0] ROTATE_SCAN_SAFE_MM = 16'd150,
    // Rotation-only: tolerate at most 12 missing 10-ms samples AFTER a
    // fully verified rotation start. Any newly detected near object vetoes
    // rotation immediately. This parameter does not authorize entering a
    // rotation with any unknown/unsafe sensor.
    parameter [4:0] ROTATE_UNKNOWN_GRACE_TICKS = 5'd12,
    // Experimental close-range tunables: top-level side stop is 100mm.
    parameter [15:0] FLOW_START_FRONT_MM = 16'd360,
    parameter [15:0] FLOW_MIN_SIDE_MM = 16'd200,
    parameter [15:0] FLOW_SIDE_HARD_MM = 16'd130,
    parameter [15:0] FLOW_FORWARD_MIN_MM = 16'd240,
    // A5 stays in low-speed cruise above this distance, even when
    // lateral ToFs are UNKNOWN. Fixes repeated 420..580mm scan loop.
    parameter [15:0] CRUISE_SCAN_TRIGGER_MM = 16'd260,
    // Enter proactive lateral bypass only when obstacle is near AND
    // that side has a valid clearance measurement.
    parameter [15:0] FLOW_SIDE_ENTER_FRONT_MM = 16'd320,
    parameter [15:0] FLOW_RELEASE_FRONT_MM = 16'd360,
    parameter [31:0] FLOW_MAX_SIDE_COUNT = 32'd1500,
    parameter [12:0] SLEW_PER_TICK = 13'd2
)(
    input wire clk,
    input wire reset_n,
    input wire sample_tick,
    input wire [7:0] rx_data,
    input wire rx_done,
    input wire sequence_active,
    input wire [15:0] distance_mm,
    input wire distance_valid,
    input wire echo_timeout,
    input wire [15:0] tof_left_mm,
    input wire [15:0] tof_right_mm,
    input wire tof_left_ok,
    input wire tof_right_ok,
    // A raw ToF zero / expired sample means UNKNOWN, never a near obstacle.
    // An UNKNOWN side may be probed ONLY after a new HC-SR04 side scan
    // confirms a clear oblique direction. The top-level supervisor limits
    // the trial to 100ms and a low lateral speed.
    input wire tof_left_unknown,
    input wire tof_right_unknown,
    input wire tof_left_trial_ok,
    input wire tof_right_trial_ok,
    input wire [15:0] tof_front_mm,
    input wire tof_front_ok,
    input wire [15:0] tof_rear_mm,
    input wire tof_rear_ok,
    input wire signed [31:0] yaw_cdeg,
    input wire signed [31:0] odom_pos_x_count,
    input wire signed [31:0] odom_pos_y_count,
    output reg scan_start,
    input wire scan_done,
    input wire [15:0] left_distance_mm,
    input wire [15:0] center_distance_mm,
    input wire [15:0] right_distance_mm,
    output reg signed [12:0] vx_out,
    output reg signed [12:0] vy_out,
    output reg signed [12:0] wz_out,
    output wire active,
    output wire error,
    output wire [3:0] state_debug
);

    localparam [3:0] ST_IDLE=4'd0, ST_START=4'd1,
                     ST_CRUISE=4'd2, ST_SCAN_SETTLE=4'd3,
                     ST_SCAN_WAIT=4'd4, ST_FLOW_LEFT=4'd5,
                     ST_VERIFY_FRONT=4'd6, ST_FLOW_RIGHT=4'd8, ST_BACKUP=4'd11,
                     ST_RECOVER=4'd12, ST_ERROR=4'd13,
                     ST_ROTATE=4'd14, ST_HOLD=4'd15;

    reg [3:0] state;
    reg [15:0] ticks;
    reg [15:0] hold_ticks;
    reg [3:0] rotate_steps;
    reg [2:0] backup_steps;
    reg rotate_right;
    reg [3:0] front_close_count;
    reg [3:0] front_open_count;
    reg signed [31:0] flow_start_y;
    reg signed [31:0] reverse_start_x;
    reg signed [31:0] rotate_start_yaw;
    reg signed [31:0] explore_start_yaw;
    reg [1:0] flow_attempts;
    reg exploring_turn;
    reg signed [12:0] want_vx, want_vy, want_wz;

    assign active = state != ST_IDLE;
    assign error = state == ST_ERROR;
    assign state_debug = state;

    function [31:0] abs_count;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [32:0] diff;
        begin
            diff = $signed({a[31],a})-$signed({b[31],b});
            if (diff < 0) diff = -diff;
            abs_count = diff[31:0];
        end
    endfunction

    function [31:0] yaw_abs;
        input signed [31:0] now_angle;
        input signed [31:0] start_angle;
        reg signed [31:0] d;
        begin
            d = now_angle - start_angle;
            if (d > 32'sd18000) d = d-32'sd36000;
            else if (d < -32'sd18000) d = d+32'sd36000;
            if (d < 0) d = -d;
            yaw_abs = d[31:0];
        end
    endfunction

    function signed [12:0] ramp13;
        input signed [12:0] last_value;
        input signed [12:0] target_value;
        input signed [12:0] max_step;
        reg signed [13:0] delta;
        begin
            delta = $signed(target_value) - $signed(last_value);
            if (delta > $signed(max_step)) ramp13 = last_value + max_step;
            else if (delta < -$signed(max_step)) ramp13 = last_value-max_step;
            else ramp13 = target_value;
        end
    endfunction

    // ========================================================
    // FIX: distance_valid is a single-clk strobe, not a persistent
    // validity level. Cache the last measurement when the servo is
    // facing forward (top.v already qualifies that condition).
    // A 10-ms controller must never sample a 20-ns validity pulse
    // directly. Expire the cache after 200 ms without updates.
    // The cached range is NOT allowed for side/rear movement.
    // ========================================================
    reg [15:0] ultra_front_last_mm;
    reg [6:0] ultra_front_age;
    reg ultra_front_seen;
    reg fresh_since_scan;

    wire ultra_update = distance_valid && !echo_timeout &&
                         distance_mm >= 16'd80 &&
                         distance_mm <= 16'd3000;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ultra_front_last_mm <= 16'd0;
            ultra_front_age <= 7'd127;
            ultra_front_seen <= 1'b0;
            fresh_since_scan <= 1'b0;
        end else if (scan_done && state == ST_SCAN_WAIT) begin
            // The servo has just finished and returned to center.
            // Require a NEW post-scan forward echo before driving.
            fresh_since_scan <= 1'b0;
            ultra_front_age <= 7'd127;
            ultra_front_seen <= 1'b0;
        end else if (ultra_update) begin
            ultra_front_last_mm <= distance_mm;
            ultra_front_age <= 7'd0;
            ultra_front_seen <= 1'b1;
            fresh_since_scan <= 1'b1;
        end else if (sample_tick && ultra_front_age != 7'd127) begin
            ultra_front_age <= ultra_front_age + 1'b1;
        end
    end

    wire ultra_front_fresh = ultra_front_seen &&
                             (ultra_front_age <= 7'd20);
    wire front_valid_raw = ultra_front_fresh || tof_front_ok;
    wire [15:0] front_mm_raw =
        (ultra_front_fresh && tof_front_ok) ?
            ((ultra_front_last_mm < tof_front_mm) ?
              ultra_front_last_mm : tof_front_mm) :
        ultra_front_fresh ? ultra_front_last_mm :
        tof_front_ok ? tof_front_mm : 16'd0;

    // A pipeline stage keeps the original 50-MHz timing relief.
    reg front_valid;
    reg [15:0] front_mm;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            front_valid <= 1'b0;
            front_mm <= 16'd0;
        end else begin
            front_valid <= front_valid_raw;
            front_mm <= front_mm_raw;
        end
    end

    // Confirmed nonzero ranges are preferred.  A 0/expired value
    // permits a SHORT probe only after the independent servo scan.
    wire left_flow_ok = (tof_left_ok && tof_left_mm >= FLOW_MIN_SIDE_MM)
                        || tof_left_trial_ok;
    wire right_flow_ok = (tof_right_ok && tof_right_mm >= FLOW_MIN_SIDE_MM)
                         || tof_right_trial_ok;
    wire left_continue_ok = (tof_left_ok && tof_left_mm > FLOW_SIDE_HARD_MM)
                            || tof_left_trial_ok;
    wire right_continue_ok = (tof_right_ok && tof_right_mm > FLOW_SIDE_HARD_MM)
                             || tof_right_trial_ok;
    wire left_scan_good = (left_distance_mm >= SIDE_MIN) &&
                          (left_distance_mm <= 16'd3000);
    wire right_scan_good = (right_distance_mm >= SIDE_MIN) &&
                           (right_distance_mm <= 16'd3000);
    wire center_scan_good = (center_distance_mm >= SEARCH_CLEAR_DISTANCE) &&
                            (center_distance_mm <= 16'd3000);
    wire [31:0] side_travel = abs_count(odom_pos_y_count, flow_start_y);
    wire [31:0] reverse_travel = abs_count(odom_pos_x_count, reverse_start_x);
    wire [31:0] rotated_step_angle = yaw_abs(yaw_cdeg, rotate_start_yaw);
    wire [31:0] rotated_total_angle = yaw_abs(yaw_cdeg, explore_start_yaw);
    wire back_start_ok = tof_rear_ok && tof_rear_mm >= BACKUP_START_MM;
    wire back_cont_ok = tof_rear_ok && tof_rear_mm > BACKUP_STOP_MM;

    // DEAD-END TURN POLICY (A5 only): inability to TRANSLATE forward/left/
    // right triggers searching for a TURN, not an automatic reverse.
    // TOF rear >=150 is NOT required to choose or enter a rotation.
    // Rotation still needs confirmed front+left+right clearance for the
    // chassis swept envelope. A measured extremely close rear obstacle
    // is an emergency veto, not a normal turn-entry requirement.
    // IMPORTANT: three point sensors cannot certify 360-degree sweep;
    // experimental low-speed testing and an emergency stop are required.
    // No ToF/sonar 0 is silently re-labelled as 'clear'.
    wire rotation_front_clear =
        tof_front_ok ? (tof_front_mm >= ROTATE_FRONT_SAFE_MM) :
        (ultra_front_fresh && ultra_front_last_mm >= ROTATE_FRONT_SAFE_MM);
    wire rotation_left_clear =
        tof_left_ok ?
        ((tof_left_mm >= ROTATE_SIDE_SAFE_MM) &&
         (left_distance_mm == 16'd0 ||
          left_distance_mm >= ROTATE_SCAN_SAFE_MM)) :
        ((left_distance_mm >= ROTATE_SCAN_SAFE_MM) &&
         (left_distance_mm <= 16'd3000));
    wire rotation_right_clear =
        tof_right_ok ?
        ((tof_right_mm >= ROTATE_SIDE_SAFE_MM) &&
         (right_distance_mm == 16'd0 ||
          right_distance_mm >= ROTATE_SCAN_SAFE_MM)) :
        ((right_distance_mm >= ROTATE_SCAN_SAFE_MM) &&
         (right_distance_mm <= 16'd3000));
    wire rear_rotation_emergency = tof_rear_ok &&
        (tof_rear_mm <= ROTATE_REAR_VETO_MM);
    // REAR IS NOT AN ENTRY REQUIREMENT: unknown rear, or confirmed
    // rear farther than the emergency threshold, does not block a turn.
    // Rear-distance clearance is still mandatory for BACKUP movement.
    wire rotation_space_ok = rotation_front_clear &&
        rotation_left_clear && rotation_right_clear &&
        !rear_rotation_emergency;

    // A short invalid pulse can occur while rotating because sensor
    // incidence/reflection changes. It is NOT evidence of a near obstacle.
    // The original stringent rotation_space_ok stays mandatory to ENTER.
    // Once in ST_ROTATE, a brief UNKNOWN does not immediately abort the
    // planned short segment. However any reliable, fresh near reading
    // revokes the permission, with no grace period.
    wire rotation_near_now =
        (tof_front_ok && tof_front_mm < ROTATE_FRONT_SAFE_MM) ||
        rear_rotation_emergency ||
        (tof_left_ok && tof_left_mm < ROTATE_SIDE_SAFE_MM) ||
        (tof_right_ok && tof_right_mm < ROTATE_SIDE_SAFE_MM) ||
        (ultra_front_fresh && ultra_front_last_mm < ROTATE_FRONT_SAFE_MM) ||
        (left_distance_mm != 16'd0 &&
         left_distance_mm < ROTATE_SCAN_SAFE_MM) ||
        (right_distance_mm != 16'd0 &&
         right_distance_mm < ROTATE_SCAN_SAFE_MM);

    reg [4:0] rotation_unknown_ticks;
    wire rotation_run_clear = !rotation_near_now &&
        (rotation_space_ok ||
         ((state == ST_ROTATE) &&
          (rotation_unknown_ticks < ROTATE_UNKNOWN_GRACE_TICKS)));

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            rotation_unknown_ticks <= 5'd0;
        else if ((state != ST_ROTATE) ||
                 (rx_done && rx_data == 8'h53))
            rotation_unknown_ticks <= 5'd0;
        else if (sample_tick) begin
            if (rotation_space_ok && !rotation_near_now)
                rotation_unknown_ticks <= 5'd0;
            else if (rotation_unknown_ticks < ROTATE_UNKNOWN_GRACE_TICKS)
                rotation_unknown_ticks <= rotation_unknown_ticks + 1'b1;
        end
    end

    // A newly completed scan is used for the initial decision.
    // The registered trial-permission becomes true on that same edge,
    // ready when A5 enters ST_FLOW_LEFT/RIGHT in the NEXT clock.
    wire left_unknown_scan_candidate = tof_left_unknown &&
        (left_distance_mm >= 16'd600) &&
        (left_distance_mm <= 16'd3000);
    wire right_unknown_scan_candidate = tof_right_unknown &&
        (right_distance_mm >= 16'd600) &&
        (right_distance_mm <= 16'd3000);
    wire pick_left = (left_flow_ok || left_unknown_scan_candidate) &&
        left_scan_good &&
        (!(right_flow_ok || right_unknown_scan_candidate) ||
         !right_scan_good || left_distance_mm >= right_distance_mm);
    wire pick_right = (right_flow_ok || right_unknown_scan_candidate) &&
        right_scan_good;

    // ========================================================
    // Desired continuous velocity. The safety gates in top.v
    // remain the final authority. No blind driving on bad data.
    // ========================================================
    always @(*) begin
        want_vx=13'sd0;
        want_vy=13'sd0;
        want_wz=13'sd0;
        case (state)
            ST_CRUISE, ST_RECOVER: begin
                if (front_valid) begin
                    // Multi-stage, monotonic approach profile.
                    // All bands are reachable with the top-level settings
                    // 600 / 400 / 320 / 280 / 260 mm.
                    // 260mm scan threshold is NOT a measured
                    // stopping-distance guarantee. Verify on the robot.
                    if (front_mm >= SLOW1_THRESHOLD)
                        want_vx = $signed({1'b0,FORWARD_SPEED});
                    else if (front_mm >= SLOW2_THRESHOLD)
                        want_vx = $signed({1'b0,SLOW1_SPEED});
                    else if (front_mm >= 16'd320)
                        want_vx = $signed({1'b0,SLOW2_SPEED});
                    else if (front_mm >= 16'd280)
                        want_vx = 13'sd8;
                    else if (front_mm > CRUISE_SCAN_TRIGGER_MM)
                        want_vx = 13'sd5;
                    else
                        want_vx = 13'sd0;
                    // UNKNOWN front TOF is not an obstacle: HC-SR04
                    // independently permits controlled forward cruise.
                    // If the last echo is getting old, cap the speed.
                    if (!tof_front_ok && ultra_front_age > 7'd10 &&
                        want_vx > 13'sd8)
                        want_vx = 13'sd8;
                    // Gentle wall centering while still moving forward.
                    if (front_mm >= 16'd320) begin
                        if (tof_left_ok && tof_left_mm < 16'd190 && right_flow_ok)
                            want_vy = 13'sd4;
                        else if (tof_right_ok && tof_right_mm < 16'd190 && left_flow_ok)
                            want_vy = -13'sd4;
                    end
                end
            end
            ST_FLOW_LEFT: begin
                if (left_continue_ok && (front_valid || tof_left_trial_ok) &&
                    side_travel < FLOW_MAX_SIDE_COUNT) begin
                    if (!tof_left_trial_ok && front_mm >= FLOW_FORWARD_MIN_MM)
                        want_vx = (front_mm >= 16'd320) ? 13'sd12 : 13'sd7;
                    // UNKNOWN ToF trial: Y-only, low speed. No diagonal forward.
                    want_vy = tof_left_trial_ok ? -13'sd5 :
                        ((tof_left_mm >= FLOW_MIN_SIDE_MM) ? -13'sd11 : -13'sd6);
                end
            end
            ST_FLOW_RIGHT: begin
                if (right_continue_ok && (front_valid || tof_right_trial_ok) &&
                    side_travel < FLOW_MAX_SIDE_COUNT) begin
                    if (!tof_right_trial_ok && front_mm >= FLOW_FORWARD_MIN_MM)
                        want_vx = (front_mm >= 16'd320) ? 13'sd12 : 13'sd7;
                    want_vy = tof_right_trial_ok ? 13'sd5 :
                        ((tof_right_mm >= FLOW_MIN_SIDE_MM) ? 13'sd11 : 13'sd6);
                end
            end
            ST_BACKUP: begin
                if (back_cont_ok && reverse_travel < BACKUP_MAX_COUNT)
                    want_vx = -$signed({1'b0,BACK_SPEED});
            end
            ST_ROTATE: begin
                if (rotation_run_clear && rotated_step_angle < ROTATE_STEP_CDEG &&
                    ticks < ROTATE_STEP_TICKS) begin
                    want_wz = rotate_right ?
                        $signed({1'b0,ROTATE_SPEED}) :
                        -$signed({1'b0,ROTATE_SPEED});
                end
            end
            default: begin end
        endcase
        // The calibrated side emergency line is set at 100mm. Degrade immediately
        // if the sensor corresponding to the movement direction is invalid.
        if ((want_vy < 0 && !tof_left_trial_ok &&
             (!tof_left_ok || tof_left_mm <= LATERAL_STOP_DISTANCE)) ||
            (want_vy > 0 && !tof_right_trial_ok &&
             (!tof_right_ok || tof_right_mm <= LATERAL_STOP_DISTANCE))) begin
            want_vx=13'sd0;
            want_vy=13'sd0;
            want_wz=13'sd0;
        end
        if (want_vx > 0 && (!front_valid || front_mm <= FRONT_THRESHOLD))
            want_vx=13'sd0;
        if (want_vx < 0 && !back_cont_ok)
            want_vx=13'sd0;
    end

    // ========================================================
    // TIMING FIX STAGE 2 (clk=50MHz): registered velocity targets.
    // Splits sensor -> decision -> ramp13 -> vy_out into separate
    // clock cycles, avoiding a 27-logic-level single-cycle path.
    // Targets are recalculated every 20ns, not every sample_tick.
    // The existing emergency gates are NOT disabled.
    // ========================================================
    reg signed [12:0] want_vx_q, want_vy_q, want_wz_q;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            want_vx_q <= 13'sd0;
            want_vy_q <= 13'sd0;
            want_wz_q <= 13'sd0;
        end else begin
            want_vx_q <= want_vx;
            want_vy_q <= want_vy;
            want_wz_q <= want_wz;
        end
    end

    // ========================================================
    // Slew-rate limited commands. Emergency/scan/HOLD stop is
    // immediate. Top.v retains independent motor collision gate.
    // ========================================================
    wire motion_state = (state == ST_CRUISE || state == ST_FLOW_LEFT ||
                         state == ST_FLOW_RIGHT || state == ST_BACKUP ||
                         state == ST_ROTATE || state == ST_RECOVER);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            vx_out <= 13'sd0;
            vy_out <= 13'sd0;
            wz_out <= 13'sd0;
        end else if (!motion_state || (rx_done && rx_data == 8'h53)) begin
            vx_out <= 13'sd0;
            vy_out <= 13'sd0;
            wz_out <= 13'sd0;
        end else if (state == ST_ROTATE && rotation_near_now) begin
            vx_out <= 13'sd0;
            vy_out <= 13'sd0;
            wz_out <= 13'sd0;
        end else if (sample_tick) begin
            // On an unsafe-axis request, stop that axis without decel delay.
            // These commands come from clocked stage-2 target registers.
            // Fail-safe per axis: no stale forward motion after the
            // forward sensor becomes UNKNOWN or reports an emergency
            // stop range.  This overrides the ordinary speed ramp.
            vx_out <= ((!front_valid && vx_out > 13'sd0) ||
                       (front_valid && front_mm <= FRONT_THRESHOLD &&
                        vx_out > 13'sd0) ||
                       (state == ST_BACKUP && !back_cont_ok)) ?
                13'sd0 : ramp13(vx_out,want_vx_q,$signed(SLEW_PER_TICK));
            // Permission expiry cancels an ongoing trial immediately.
            vy_out <= ((state == ST_FLOW_LEFT && !left_continue_ok) ||
                       (state == ST_FLOW_RIGHT && !right_continue_ok)) ?
                13'sd0 : ramp13(vy_out,want_vy_q,$signed(SLEW_PER_TICK));
            wz_out <= (state == ST_ROTATE && !rotation_run_clear) ?
                13'sd0 : ramp13(wz_out,want_wz_q,$signed(SLEW_PER_TICK));
        end
    end

    // ========================================================
    // Exploration FSM. Obstacles are NOT errors. Only sensor
    // faults/timeouts that invalidate the operation can halt it.
    // ========================================================
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_IDLE;
            ticks <= 16'd0;
            hold_ticks <= 16'd0;
            rotate_steps <= 4'd0;
            backup_steps <= 3'd0;
            rotate_right <= 1'b1;
            front_close_count <= 4'd0;
            front_open_count <= 4'd0;
            flow_start_y <= 32'sd0;
            reverse_start_x <= 32'sd0;
            rotate_start_yaw <= 32'sd0;
            explore_start_yaw <= 32'sd0;
            flow_attempts <= 2'd0;
            exploring_turn <= 1'b0;
            scan_start <= 1'b0;
        end else begin
            scan_start <= 1'b0;
            if (rx_done && rx_data == 8'h53) begin
                state <= ST_IDLE;
                ticks <= 16'd0;
                hold_ticks <= 16'd0;
                rotate_steps <= 4'd0;
                backup_steps <= 3'd0;
                front_close_count <= 4'd0;
                front_open_count <= 4'd0;
                flow_attempts <= 2'd0;
                exploring_turn <= 1'b0;
            end else if (state == ST_IDLE && rx_done &&
                         rx_data == 8'hA5 && !sequence_active) begin
                // UART RX is a 1-clk pulse: don't sample only every 10 ms.
                state <= ST_START;
                ticks <= 16'd0;
                rotate_steps <= 4'd0;
                backup_steps <= 3'd0;
                exploring_turn <= 1'b0;
                flow_attempts <= 2'd0;
                explore_start_yaw <= yaw_cdeg;
            end else if (state == ST_SCAN_WAIT && scan_done) begin
                // Scan-complete is a 1-clk pulse as well.
                ticks <= 16'd0;
                if ((center_scan_good &&
                     (!tof_front_ok || tof_front_mm >= SEARCH_CLEAR_DISTANCE)) ||
                    (tof_front_ok &&
                     tof_front_mm >= SEARCH_CLEAR_DISTANCE)) begin
                    // Never trust the scan center as a green light when
                    // an independent, calibrated front ToF sees an obstacle.
                    // Servo just returned to center. Wait for a NEW live
                    // forward reading, never reuse a range from the scan.
                    state <= ST_VERIFY_FRONT;
                    front_open_count <= 4'd0;
                end else if (pick_left && flow_attempts < 2) begin
                    flow_start_y <= odom_pos_y_count;
                    front_open_count <= 4'd0;
                    state <= ST_FLOW_LEFT;
                    exploring_turn <= 1'b0;
                end else if (pick_right && flow_attempts < 2) begin
                    flow_start_y <= odom_pos_y_count;
                    front_open_count <= 4'd0;
                    state <= ST_FLOW_RIGHT;
                    exploring_turn <= 1'b0;
                end else if (rotation_space_ok &&
                             rotate_steps < MAX_ROTATE_STEPS &&
                             rotated_total_angle < DEADEND_TURN_LIMIT_CDEG) begin
                    rotate_start_yaw <= yaw_cdeg;
                    if (!exploring_turn) begin
                        explore_start_yaw <= yaw_cdeg;
                        // Side-scan 0 means missing echo, not a reason
                        // to prefer right by default. Fall back to the
                        // confirmed LEFT/RIGHT ToF distances when scans
                        // cannot choose a direction.
                        if (left_scan_good && right_scan_good)
                            rotate_right <= (right_distance_mm >= left_distance_mm);
                        else if (left_scan_good)
                            rotate_right <= 1'b0;
                        else if (right_scan_good)
                            rotate_right <= 1'b1;
                        else if (tof_left_ok && tof_right_ok)
                            rotate_right <= (tof_right_mm >= tof_left_mm);
                        else if (tof_left_ok)
                            rotate_right <= 1'b0;
                        else if (tof_right_ok)
                            rotate_right <= 1'b1;
                        else
                            rotate_right <= !rotate_right;
                    end
                    exploring_turn <= 1'b1;
                    state <= ST_ROTATE;
                end else if (back_start_ok && backup_steps < 3'd6) begin
                    reverse_start_x <= odom_pos_x_count;
                    exploring_turn <= 1'b1;
                    state <= ST_BACKUP;
                end else begin
                    state <= ST_HOLD;
                    hold_ticks <= 16'd0;
                    exploring_turn <= 1'b1;
                end
            end else if (sample_tick) begin
                case (state)
                    ST_IDLE: begin
                        ticks <= 16'd0;
                        rotate_steps <= 4'd0;
                        backup_steps <= 3'd0;
                        flow_attempts <= 2'd0;
                        // Start pulse handled above at the full clock rate.
                    end
                    ST_START: begin
                        if (ticks >= START_SETTLE_TICKS-1) begin
                            ticks <= 16'd0;
                            // Missing all front range data is UNKNOWN.
                            // Stay armed and check for a new echo at center;
                            // do not launch a servo scan immediately.
                            state <= front_valid ? ST_CRUISE : ST_HOLD;
                            hold_ticks <= 16'd0;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_CRUISE: begin
                        ticks <= 16'd0;
                        if (!front_valid) begin
                            // Missing forward data is UNKNOWN, not NEAR.
                            // Keep A5 armed, listen at the center, and
                            // periodically scan only if HC never recovers.
                            state <= ST_HOLD;
                            hold_ticks <= 16'd0;
                            front_close_count <= 4'd0;
                            explore_start_yaw <= yaw_cdeg;
                        end else if (front_mm <= CRUISE_SCAN_TRIGGER_MM) begin
                            // Real near obstacle. 3 consecutive 10ms ticks
                            // reject one-off measurement spikes.
                            if (front_close_count >= 4'd2) begin
                                front_close_count <= 4'd0;
                                if (flow_attempts < 2 && left_flow_ok &&
                                    (!right_flow_ok || tof_left_mm >= tof_right_mm)) begin
                                    flow_start_y <= odom_pos_y_count;
                                    front_open_count <= 4'd0;
                                    state <= ST_FLOW_LEFT;
                                end else if (flow_attempts < 2 && right_flow_ok) begin
                                    flow_start_y <= odom_pos_y_count;
                                    front_open_count <= 4'd0;
                                    state <= ST_FLOW_RIGHT;
                                end else begin
                                    state <= ST_SCAN_SETTLE;
                                end
                            end else front_close_count <= front_close_count+1'b1;
                        end else if (front_mm <= FLOW_SIDE_ENTER_FRONT_MM &&
                                     (left_flow_ok || right_flow_ok) &&
                                     flow_attempts < 2) begin
                            // Optional early bypass, but ONLY when there is
                            // valid side clearance. Otherwise stay in low
                            // speed straight cruise: don't scan at 580mm.
                            if (front_close_count >= 4'd2) begin
                                front_close_count <= 4'd0;
                                flow_start_y <= odom_pos_y_count;
                                front_open_count <= 4'd0;
                                if (left_flow_ok &&
                                    (!right_flow_ok || tof_left_mm >= tof_right_mm))
                                    state <= ST_FLOW_LEFT;
                                else
                                    state <= ST_FLOW_RIGHT;
                            end else front_close_count <= front_close_count+1'b1;
                        end else begin
                            // 380..580mm is a low-speed cruise corridor,
                            // not a reason to stop and scan repeatedly.
                            front_close_count <= 4'd0;
                            if (front_mm >= FLOW_RELEASE_FRONT_MM) begin
                                rotate_steps <= 4'd0;
                                backup_steps <= 3'd0;
                                exploring_turn <= 1'b0;
                                flow_attempts <= 2'd0;
                            end
                        end
                    end
                    ST_FLOW_LEFT, ST_FLOW_RIGHT: begin
                        ticks <= ticks+1'b1;
                        if ((state == ST_FLOW_LEFT && !left_continue_ok) ||
                            (state == ST_FLOW_RIGHT && !right_continue_ok)) begin
                            // No valid side range: mark UNKNOWN, cancel Vy.
                            // If a fresh forward reading is clear, continue
                            // straight; do not stop the whole robot.
                            ticks <= 16'd0;
                            front_open_count <= 4'd0;
                            if (front_valid &&
                                front_mm >= (CRUISE_SCAN_TRIGGER_MM + 16'd20)) begin
                                flow_attempts <= 2'd0;
                                state <= ST_CRUISE;
                            end else begin
                                state <= ST_SCAN_SETTLE;
                            end
                        end else if ((!front_valid &&
                                     !((state == ST_FLOW_LEFT && tof_left_trial_ok) ||
                                       (state == ST_FLOW_RIGHT && tof_right_trial_ok))) ||
                                     side_travel >= FLOW_MAX_SIDE_COUNT ||
                                     ticks >= DIAG_TICKS) begin
                            ticks <= 16'd0;
                            if (flow_attempts < 2'd3)
                                flow_attempts <= flow_attempts+1'b1;
                            state <= ST_SCAN_SETTLE;
                        end else if (front_mm >= FLOW_RELEASE_FRONT_MM) begin
                            if (front_open_count >= 4'd10) begin
                                front_open_count <= 4'd0;
                                ticks <= 16'd0;
                                flow_attempts <= 2'd0;
                                state <= ST_RECOVER;
                            end else front_open_count <= front_open_count+1'b1;
                        end else front_open_count <= 4'd0;
                    end
                    ST_VERIFY_FRONT: begin
                        // The centered sonar must report a fresh echo.
                        // If it confirms a clear forward corridor, move;
                        // otherwise HOLD and re-examine the environment.
                        if (front_valid && (fresh_since_scan || tof_front_ok) &&
                            front_mm >= SEARCH_CLEAR_DISTANCE) begin
                            if (front_open_count >= 4'd2) begin
                                state <= ST_RECOVER;
                                exploring_turn <= 1'b0;
                                front_open_count <= 4'd0;
                                ticks <= 16'd0;
                            end else front_open_count <= front_open_count+1'b1;
                        end else if (front_valid && (fresh_since_scan || tof_front_ok) &&
                                     front_mm < SEARCH_CLEAR_DISTANCE) begin
                            state <= ST_HOLD;
                            hold_ticks <= 16'd0;
                            ticks <= 16'd0;
                            front_open_count <= 4'd0;
                        end else if (ticks >= 16'd150) begin
                            // No centered echo for 1.5 sec: do not move.
                            state <= ST_HOLD;
                            ticks <= 16'd0;
                            hold_ticks <= 16'd0;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_RECOVER: begin
                        flow_attempts <= 2'd0;
                        // Keep state=12 long enough for A6 supervisor to
                        // notice and hand control back to waypoint navigation.
                        if (ticks >= RECOVER_TICKS-1) begin
                            state <= ST_CRUISE;
                            ticks <= 16'd0;
                            rotate_steps <= 4'd0;
                            backup_steps <= 3'd0;
                            exploring_turn <= 1'b0;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_SCAN_SETTLE: begin
                        if (ticks >= STOP_SETTLE_TICKS-1) begin
                            ticks <= 16'd0;
                            scan_start <= 1'b1;
                            state <= ST_SCAN_WAIT;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_SCAN_WAIT: begin
                        if (ticks >= SCAN_TIMEOUT_TICKS) begin
                            // Servo timeout: no blind move. Recheck later.
                            ticks <= 16'd0;
                            state <= ST_HOLD;
                            hold_ticks <= 16'd0;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_BACKUP: begin
                        if (!back_cont_ok || reverse_travel >= BACKUP_MAX_COUNT ||
                            ticks >= BACK_TICKS) begin
                            ticks <= 16'd0;
                            if (backup_steps < 3'd7)
                                backup_steps <= backup_steps+1'b1;
                            state <= ST_SCAN_SETTLE;
                        end else ticks <= ticks+1'b1;
                    end
                    ST_ROTATE: begin
                        if (!rotation_run_clear) begin
                            ticks <= 16'd0;
                            if (back_start_ok && backup_steps < 3'd6) begin
                                reverse_start_x <= odom_pos_x_count;
                                state <= ST_BACKUP;
                            end else begin
                                state <= ST_HOLD;
                                hold_ticks <= 16'd0;
                            end
                        end else if (rotated_step_angle >= ROTATE_STEP_CDEG ||
                                     ticks >= ROTATE_STEP_TICKS) begin
                            ticks <= 16'd0;
                            if (rotated_step_angle < ROTATE_MIN_PROGRESS_CDEG) begin
                                state <= ST_HOLD;
                                hold_ticks <= 16'd0;
                            end else begin
                                rotate_steps <= rotate_steps+1'b1;
                                state <= ST_SCAN_SETTLE;
                            end
                        end else ticks <= ticks+1'b1;
                    end
                    ST_HOLD: begin
                        // UNKNOWN != BLOCKED. Stay in A5; re-evaluate
                        // automatically rather than latching ST_ERROR.
                        // On a fresh front reading after scanning, cruise.
                        if (front_valid &&
                            (fresh_since_scan || tof_front_ok) &&
                            front_mm >= (CRUISE_SCAN_TRIGGER_MM + 16'd20)) begin
                            hold_ticks <= 16'd0;
                            ticks <= 16'd0;
                            state <= ST_RECOVER;
                        end else if (hold_ticks >= HOLD_RESCAN_TICKS-1) begin
                            hold_ticks <= 16'd0;
                            ticks <= 16'd0;
                            // Reverse direction after exhausting a search arc.
                            if (rotate_steps >= MAX_ROTATE_STEPS ||
                                rotated_total_angle >= DEADEND_TURN_LIMIT_CDEG) begin
                                rotate_steps <= 4'd0;
                                rotate_right <= !rotate_right;
                                explore_start_yaw <= yaw_cdeg;
                            end
                            backup_steps <= 3'd0;
                            state <= ST_SCAN_SETTLE;
                        end else hold_ticks <= hold_ticks+1'b1;
                    end
                    ST_ERROR: begin
                        state <= ST_ERROR;
                    end
                    default: state <= ST_IDLE;
                endcase
            end
        end
    end
endmodule
