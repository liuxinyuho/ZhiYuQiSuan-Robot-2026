`timescale 1ns / 1ps
// ============================================================================
// A6 rectangle route for the existing ACG720 mecanum + MPU6500 project.
// Start (A6) at O=(0,0), using calibrated world odometry in millimetres.
// Leg 0: forward 3100 mm, then LEFT 90 deg.
// Leg 1: forward 1300 mm, then LEFT 90 deg.
// Leg 2: forward 3100 mm, then LEFT 90 deg.
// Leg 3: forward 1300 mm, then LEFT 90 deg and STOP at origin.
// Physical rotation is Wz positive, matching the previously proven P4 turn.
// Sensor yaw typically decreases on a left turn in this project.
// Pause freezes progress timers during local A6 detours, not target coordinates.
// CAUTION: If left-turn sign / world Y orientation are different on the board,
// stop on the FIRST 90-degree test and calibrate; do not try all 4 legs.
// ============================================================================
module waypoint_nav #(
    parameter signed [31:0] POSITION_TOL_MM = 32'sd45,
    parameter signed [31:0] SLOW_ZONE_MM    = 32'sd200,
    parameter signed [12:0] FAST_SPEED      = 13'sd18,
    parameter signed [12:0] SLOW_SPEED      = 13'sd8,
    parameter signed [31:0] P1_X_MM = 32'sd3100,
    parameter signed [31:0] P1_Y_MM = 32'sd0,
    parameter signed [31:0] P2_X_MM = 32'sd3100,
    parameter signed [31:0] P2_Y_MM = -32'sd1300,
    parameter signed [31:0] P3_X_MM = 32'sd0,
    parameter signed [31:0] P3_Y_MM = -32'sd1300,
    parameter signed [31:0] P4_X_MM = 32'sd0,
    parameter signed [31:0] P4_Y_MM = 32'sd0,
    parameter TEST_STOP_AFTER_P2 = 1'b0
)(
    input wire clk, reset_n, sample_tick,
    input wire [7:0] rx_data,
    input wire rx_done, calib_done, block_start, pause,
    input wire signed [31:0] world_x_mm, world_y_mm, yaw_cdeg,
    input wire turn_clear,
    output reg signed [12:0] vx_out, vy_out, wz_out,
    output reg active, done,
    output wire turning,
    output wire corner_ready,  // qualified safe near-corner release, never a motor override
    output reg [2:0] waypoint_index, state_debug, fault_code
);
    localparam [2:0] ST_IDLE=3'd0, ST_RUN=3'd1,
                     ST_HOLD=3'd2, ST_TURN=3'd3,
                     ST_POSTTURN=3'd4, ST_DONE=3'd5,
                     ST_FAULT=3'd6, ST_LOAD=3'd7;
    // Previous engineering values that moved the vehicle reliably.
    // A6 waypoint translation only. Preserve original turn rates and heading FSM.
    localparam signed [12:0] NAV_FAST=13'sd13, NAV_MID=13'sd10, NAV_SLOW=13'sd7;
    localparam signed [12:0] TURN_FAST=13'sd12, TURN_SLOW=13'sd7;
    localparam signed [31:0] TURN_TARGET_CDEG=32'sd8850; // 88.5deg; coast allowance
    // Terminal arrival corrections are intentionally more tolerant than
    // mid-leg tracking. A 97mm lateral residual previously prevented P1
    // from ever entering the first 90-degree turn.
    localparam signed [31:0] CROSS_TOL_MM=32'sd125;
    // <=150mm short of the requested end is acceptable only after 100ms
    // of continuous turn_clear, NOT because of an obstacle by itself.
    localparam signed [31:0] CORNER_EARLY_MM=32'sd150;
    localparam signed [31:0] CORNER_CROSS_MM=32'sd135;
    localparam [3:0] CORNER_STABLE_TICKS=4'd10;
    localparam signed [31:0] CROSS_FAULT_MM=32'sd900;
    localparam signed [31:0] OVERSHOOT_MM=32'sd300;
    localparam [15:0] RUN_TIMEOUT=16'd12000;  // 120s moving time per leg
    localparam [15:0] TURN_TIMEOUT=16'd3000;  // 30s moving time per corner
    localparam [9:0] TURN_BLOCK_TIMEOUT=10'd500; // 5s if safety cannot clear
    localparam [7:0] SETTLE_TICKS=8'd30;

    reg [2:0] state;
    reg signed [31:0] origin_x, origin_y, target_x, target_y;
    reg signed [31:0] turn_origin_yaw;
    reg signed [31:0] last_x, last_y;
    reg odom_seen;
    reg [15:0] run_ticks, turn_ticks;
    reg [9:0] block_ticks;
    reg [7:0] settle_ticks;
    reg [2:0] arrived_ticks;
    reg [3:0] corner_clear_ticks;

    wire signed [31:0] ex=target_x-world_x_mm;
    wire signed [31:0] ey=target_y-world_y_mm;
    // Cardinal body +X heading: 0:+worldX, 1:-worldY,
    // 2:-worldX, 3:+worldY. Body +Y is to the RIGHT.
    wire signed [31:0] along = (waypoint_index==3'd0) ? ex :
                               (waypoint_index==3'd1) ? -ey :
                               (waypoint_index==3'd2) ? -ex : ey;
    wire signed [31:0] cross = (waypoint_index==3'd0) ? ey :
                               (waypoint_index==3'd1) ? ex :
                               (waypoint_index==3'd2) ? -ey : -ex;
    wire signed [31:0] abs_cross=(cross<0)?-cross:cross;
    // Check corner position using signed along/cross error; never infer
    // arrival from an obstacle reading alone.
    wire corner_near=(along <= CORNER_EARLY_MM) &&
                     (along >= -(POSITION_TOL_MM+32'sd20)) &&
                     (abs_cross <= CORNER_CROSS_MM);
    wire jump_error;
    wire corner_clear_sample = (state==ST_RUN) && corner_near &&
                               turn_clear && !jump_error;
    assign corner_ready = corner_clear_sample &&
                          (corner_clear_ticks >= CORNER_STABLE_TICKS);
    // IMPORTANT: reaching a waypoint and permission to rotate are different.
    // A normal in-tolerance arrival must enter HOLD even when turn_clear=0.
    // Otherwise ST_RUN emits zero motion yet never starts blocked-turn
    // recovery (NS remains 0 forever). Early corner remains guarded.
    wire normal_arrived=(along<=POSITION_TOL_MM &&
                         along>=-POSITION_TOL_MM &&
                         abs_cross<=CROSS_TOL_MM);
    wire arrived_now=normal_arrived || corner_ready;
    // Pipeline arrival decision to break the long world_y_mm ->
    // waypoint state -> target_y enable setup path. 10 ms latency is
    // negligible compared with the existing 3-sample arrival qualifier.
    reg arrived_sampled;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) arrived_sampled<=1'b0;
        else if(rx_done && rx_data==8'h53) arrived_sampled<=1'b0;
        else if(sample_tick) begin
            arrived_sampled <= (state==ST_RUN) && arrived_now && !jump_error;
        end
    end
    wire arrived=arrived_sampled;
    wire leg_error=(along<-(POSITION_TOL_MM+OVERSHOOT_MM)) ||
                   (abs_cross>CROSS_FAULT_MM);
    wire signed [31:0] step_x=world_x_mm-last_x;
    wire signed [31:0] step_y=world_y_mm-last_y;
    wire signed [31:0] abs_step_x=step_x<0?-step_x:step_x;
    wire signed [31:0] abs_step_y=step_y<0?-step_y:step_y;
    assign jump_error=odom_seen && (abs_step_x>32'sd150 || abs_step_y>32'sd150);
    wire signed [31:0] yaw_raw=yaw_cdeg-turn_origin_yaw;
    wire signed [31:0] yaw_delta=(yaw_raw>32'sd18000) ? yaw_raw-32'sd36000 :
                                (yaw_raw< -32'sd18000) ? yaw_raw+32'sd36000 : yaw_raw;
    wire signed [31:0] left_progress=-yaw_delta;
    wire turn_complete=left_progress>=TURN_TARGET_CDEG;
    // Do not declare a powered-turn direction fault while the motor has
    // never been permitted to turn: yaw may change during a blocked turn
    // (manual handling, slip or IMU noise). If positive yaw persists once
    // commanded turning actually starts, retain the existing FC=4 stop.
    wire turn_wrong_way=(turn_ticks>=16'd20) &&
                        (yaw_delta>32'sd900);
    assign turning=(state==ST_TURN);

    // This independent 10ms qualifier continues while NS=6 pauses A6.
    // Clearing a blocked supervisor is allowed only AFTER this turns true.
    // A sustained near obstacle immediately resets the count.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) corner_clear_ticks <= 4'd0;
        else if (rx_done && rx_data==8'h53) corner_clear_ticks <= 4'd0;
        else if (sample_tick) begin
            if (corner_clear_sample) begin
                if (corner_clear_ticks < CORNER_STABLE_TICKS)
                    corner_clear_ticks <= corner_clear_ticks + 1'b1;
            end else corner_clear_ticks <= 4'd0;
        end
    end

    always @(*) begin
        vx_out=0; vy_out=0; wz_out=0;
        if(active && !pause) begin
            if(state==ST_RUN && !leg_error && !arrived &&
               run_ticks<RUN_TIMEOUT) begin
                // Prioritize forward progress. After a bypass, repair cross
                // error at the waypoint, with another distance safety gate.
                if(!arrived_now && along>POSITION_TOL_MM)
                    vx_out=(along>32'sd650)?NAV_FAST :
                           ((along>SLOW_ZONE_MM)?NAV_MID:NAV_SLOW);
                else if(!arrived_now && along< -POSITION_TOL_MM)
                    vx_out=-NAV_SLOW;
                else if(!arrived_now && cross>CROSS_TOL_MM)
                    vy_out=NAV_SLOW;
                else if(!arrived_now && cross< -CROSS_TOL_MM)
                    vy_out=-NAV_SLOW;
            end else if(state==ST_TURN && turn_clear && !turn_complete &&
                        !turn_wrong_way && turn_ticks<TURN_TIMEOUT) begin
                wz_out=(left_progress<32'sd6500)?TURN_FAST:TURN_SLOW;
            end
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=ST_IDLE; active<=0; done<=0;
            waypoint_index<=0; state_debug<=0; fault_code<=0;
            origin_x<=0; origin_y<=0; target_x<=0; target_y<=0;
            turn_origin_yaw<=0; last_x<=0; last_y<=0; odom_seen<=0;
            run_ticks<=0; turn_ticks<=0; block_ticks<=0;
            settle_ticks<=0; arrived_ticks<=0;
        end else begin
            state_debug<=state;
            if(sample_tick) begin
                last_x<=world_x_mm;
                last_y<=world_y_mm;
                odom_seen<=1;
            end
            if(rx_done && rx_data==8'h53) begin
                state<=ST_IDLE; active<=0; done<=0;
                waypoint_index<=0; fault_code<=0;
                run_ticks<=0; turn_ticks<=0; block_ticks<=0;
                settle_ticks<=0; arrived_ticks<=0;
            end else begin
                case(state)
                    ST_IDLE,ST_DONE: begin
                        active<=0; done<=(state==ST_DONE);
                        if(state==ST_IDLE) waypoint_index<=0;
                        if(rx_done && rx_data==8'hA6 && calib_done &&
                           !block_start && !pause) begin
                            origin_x<=world_x_mm; origin_y<=world_y_mm;
                            // Stage-1: latch odometry only.  No combinational
                            // world coordinate -> add -> target register path.
                            target_x<=32'sd0;
                            target_y<=32'sd0;
                            last_x<=world_x_mm; last_y<=world_y_mm;
                            odom_seen<=1;
                            state<=ST_LOAD; active<=1; done<=0;
                            waypoint_index<=0; fault_code<=0;
                            run_ticks<=0; turn_ticks<=0; block_ticks<=0;
                            settle_ticks<=0; arrived_ticks<=0;
                        end
                    end
                    // Stage-2: target from locally registered origin.
                    // The 50MHz path now starts at origin_x/y registers,
                    // not odometry's world_x/y output.
                    ST_LOAD: begin
                        target_x<=origin_x+P1_X_MM;
                        target_y<=origin_y+P1_Y_MM;
                        state<=ST_RUN;
                    end
                    ST_RUN: begin
                        active<=1; done<=0;
                        if(sample_tick && jump_error) begin
                            fault_code<=3'd1; state<=ST_FAULT; active<=0;
                        end else if(sample_tick && !pause) begin
                            if(leg_error) begin
                                fault_code<=3'd2; state<=ST_FAULT; active<=0;
                            end else if(run_ticks>=RUN_TIMEOUT-16'd1) begin
                                fault_code<=3'd3; state<=ST_FAULT; active<=0;
                            end else if(arrived) begin
                                if(arrived_ticks>=3'd2) begin
                                    arrived_ticks<=0;
                                    settle_ticks<=0;
                                    state<=ST_HOLD;
                                end else arrived_ticks<=arrived_ticks+1'b1;
                            end else begin
                                arrived_ticks<=0;
                                run_ticks<=run_ticks+1'b1;
                            end
                        end
                    end
                    ST_HOLD: begin
                        active<=1; done<=0;
                        if(sample_tick && !pause) begin
                            if(settle_ticks<SETTLE_TICKS-8'd1)
                                settle_ticks<=settle_ticks+1'b1;
                            else if(TEST_STOP_AFTER_P2 && waypoint_index==3'd1) begin
                                state<=ST_DONE; active<=0; done<=1;
                            end else begin
                                turn_origin_yaw<=yaw_cdeg;
                                turn_ticks<=0; block_ticks<=0;
                                state<=ST_TURN;
                            end
                        end
                    end
                    ST_TURN: begin
                        active<=1; done<=0;
                        if(sample_tick && !pause) begin
                            if(turn_wrong_way) begin
                                fault_code<=3'd4; state<=ST_FAULT; active<=0;
                            end else if(!turn_clear) begin
                                if(block_ticks>=TURN_BLOCK_TIMEOUT-10'd1) begin
                                    fault_code<=3'd6; state<=ST_FAULT; active<=0;
                                end else block_ticks<=block_ticks+1'b1;
                            end else if(turn_ticks>=TURN_TIMEOUT-16'd1) begin
                                fault_code<=3'd4; state<=ST_FAULT; active<=0;
                            end else if(turn_complete) begin
                                settle_ticks<=0;
                                state<=ST_POSTTURN;
                            end else begin
                                block_ticks<=0;
                                turn_ticks<=turn_ticks+1'b1;
                            end
                        end
                    end
                    ST_POSTTURN: begin
                        active<=1; done<=0;
                        if(sample_tick && !pause) begin
                            if(settle_ticks<SETTLE_TICKS-8'd1)
                                settle_ticks<=settle_ticks+1'b1;
                            else if(waypoint_index==3'd3) begin
                                // Finished the final 90-degree rotation at O.
                                active<=0; done<=1; state<=ST_DONE;
                            end else begin
                                waypoint_index<=waypoint_index+1'b1;
                                run_ticks<=0; arrived_ticks<=0;
                                state<=ST_RUN;
                                case(waypoint_index)
                                    3'd0: begin
                                        target_x<=origin_x+P2_X_MM;
                                        target_y<=origin_y+P2_Y_MM;
                                    end
                                    3'd1: begin
                                        target_x<=origin_x+P3_X_MM;
                                        target_y<=origin_y+P3_Y_MM;
                                    end
                                    3'd2: begin
                                        target_x<=origin_x+P4_X_MM;
                                        target_y<=origin_y+P4_Y_MM;
                                    end
                                    default: begin
                                        fault_code<=3'd5;
                                        active<=0; state<=ST_FAULT;
                                    end
                                endcase
                            end
                        end
                    end
                    ST_FAULT: begin active<=0; done<=0; end
                    default: begin state<=ST_IDLE; active<=0; done<=0; end
                endcase
            end
        end
    end
    wire unused_params=^{FAST_SPEED,SLOW_SPEED};
endmodule
