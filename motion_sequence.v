//////////////////////////////////////////////////////////////////////////////////
// File Name : motion_sequence.v
// Project   : FPGA Mecanum Robot Chassis
//
// Commands:
//
// B1:
//   前进500mm
//   -> 右转90°
//   -> 前进500mm
//   -> 停止
//
// D1:
//   前进500mm
//   -> 右转90°
//   -> 前进500mm
//   -> 左移500mm
//   -> 停止
//
// D2:
//   前进500mm
//   -> 右转90°
//   -> 前进500mm
//   -> 右转90°
//   -> 前进500mm
//   -> 右转90°
//   -> 前进500mm
//   -> 右转90°
//   -> 停止
//
// 53:
//   紧急停止
//
// sample_tick = 10ms
//////////////////////////////////////////////////////////////////////////////////

module motion_sequence #(

    // 50 * 10ms = 0.5s
    parameter [7:0] PAUSE_TICKS =
        8'd50,

    // 800 * 10ms = 8s
    parameter [9:0] STEP_TIMEOUT_TICKS =
        10'd800

)(
    input wire clk,
    input wire reset_n,

    input wire sample_tick,

    //================================================
    // UART
    //================================================
    input wire [7:0] rx_data,
    input wire       rx_done,

    //================================================
    // IMU
    //================================================
    input wire calib_done,

    //================================================
    // X轴距离控制器
    //================================================
    input wire distance_auto_active,
    input wire distance_auto_done,

    //================================================
    // 转角控制器
    //================================================
    input wire angle_auto_active,
    input wire angle_auto_done,

    //================================================
    // Y轴横移控制器
    //================================================
    input wire lateral_auto_active,
    input wire lateral_auto_done,

    //================================================
    // X轴定距离启动
    //================================================
    output reg distance_start,

    //================================================
    // 转角启动
    //================================================
    output reg angle_start,
    output reg angle_turn_right,

    //================================================
    // 横移启动
    //================================================
    output reg lateral_start,
    output reg lateral_move_right,

    //================================================
    // 停止/清状态
    //================================================
    output reg stop_req,

    //================================================
    // Debug
    //================================================
    output reg sequence_active,
    output reg sequence_done,
    output reg sequence_error,

    output reg [5:0] state_debug
);


    //================================================
    // 路径类型
    //================================================

    localparam [1:0]
        ROUTE_B1 = 2'd0,
        ROUTE_D1 = 2'd1,
        ROUTE_D2 = 2'd2;

    reg [1:0] route_mode;


    //================================================
    // 状态定义
    //================================================

    localparam [5:0]

        ST_IDLE              = 6'd0,

        ST_CLEAR_START       = 6'd1,
        ST_WAIT_CLEAR        = 6'd2,

        // 第一条边
        ST_FWD1_START        = 6'd3,
        ST_FWD1_WAIT         = 6'd4,
        ST_FWD1_CLEAR        = 6'd5,
        ST_PAUSE1            = 6'd6,

        // 第一次右转
        ST_TURN1_START       = 6'd7,
        ST_TURN1_WAIT        = 6'd8,
        ST_TURN1_CLEAR       = 6'd9,
        ST_PAUSE2            = 6'd10,

        // 第二条边
        ST_FWD2_START        = 6'd11,
        ST_FWD2_WAIT         = 6'd12,
        ST_FWD2_CLEAR        = 6'd13,
        ST_PAUSE3            = 6'd14,

        // D1专用左移
        ST_LATERAL_START     = 6'd15,
        ST_LATERAL_WAIT      = 6'd16,

        // D2第二次右转
        ST_TURN2_START       = 6'd17,
        ST_TURN2_WAIT        = 6'd18,
        ST_TURN2_CLEAR       = 6'd19,
        ST_PAUSE4            = 6'd20,

        // D2第三条边
        ST_FWD3_START        = 6'd21,
        ST_FWD3_WAIT         = 6'd22,
        ST_FWD3_CLEAR        = 6'd23,
        ST_PAUSE5            = 6'd24,

        // D2第三次右转
        ST_TURN3_START       = 6'd25,
        ST_TURN3_WAIT        = 6'd26,
        ST_TURN3_CLEAR       = 6'd27,
        ST_PAUSE6            = 6'd28,

        // D2第四条边
        ST_FWD4_START        = 6'd29,
        ST_FWD4_WAIT         = 6'd30,
        ST_FWD4_CLEAR        = 6'd31,
        ST_PAUSE7            = 6'd32,

        // D2第四次右转
        ST_TURN4_START       = 6'd33,
        ST_TURN4_WAIT        = 6'd34,

        // 完成
        ST_FINAL_STOP        = 6'd35,
        ST_FINISH            = 6'd36;


    reg [5:0] state;

    reg [7:0] pause_cnt;

    reg [9:0] step_timeout_cnt;


    //================================================
    // 主状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_IDLE;

            route_mode <= ROUTE_B1;

            pause_cnt <= 8'd0;

            step_timeout_cnt <= 10'd0;


            distance_start <= 1'b0;

            angle_start <= 1'b0;

            angle_turn_right <= 1'b1;

            lateral_start <= 1'b0;

            lateral_move_right <= 1'b0;

            stop_req <= 1'b0;


            sequence_active <= 1'b0;

            sequence_done <= 1'b0;

            sequence_error <= 1'b0;

            state_debug <= ST_IDLE;

        end

        else begin

            //================================================
            // 默认输出
            //================================================

            distance_start <= 1'b0;

            angle_start <= 1'b0;

            lateral_start <= 1'b0;

            stop_req <= 1'b0;

            sequence_done <= 1'b0;

            // 所有自动转角默认都是右转
            angle_turn_right <= 1'b1;

            // D1最后默认左移
            lateral_move_right <= 1'b0;

            state_debug <= state;


            //================================================
            // 53紧急停止
            //================================================

            if(
                sequence_active
                &&
                rx_done
                &&
                (rx_data == 8'h53)
            ) begin

                distance_start <= 1'b0;

                angle_start <= 1'b0;

                lateral_start <= 1'b0;

                stop_req <= 1'b1;


                sequence_active <= 1'b0;

                sequence_done <= 1'b0;

                sequence_error <= 1'b0;


                pause_cnt <= 8'd0;

                step_timeout_cnt <= 10'd0;

                state <= ST_IDLE;

            end

            else begin

                case(state)


                    //================================================
                    // IDLE
                    //================================================

                    ST_IDLE: begin

                        sequence_active <= 1'b0;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;


                        //============================================
                        // B1
                        //============================================

                        if(
                            rx_done
                            &&
                            (rx_data == 8'hB1)
                            &&
                            calib_done
                        ) begin

                            route_mode <= ROUTE_B1;

                            sequence_active <= 1'b1;

                            sequence_error <= 1'b0;

                            state <= ST_CLEAR_START;

                        end


                        //============================================
                        // D1
                        //============================================

                        else if(
                            rx_done
                            &&
                            (rx_data == 8'hD1)
                            &&
                            calib_done
                        ) begin

                            route_mode <= ROUTE_D1;

                            sequence_active <= 1'b1;

                            sequence_error <= 1'b0;

                            state <= ST_CLEAR_START;

                        end


                        //============================================
                        // D2 方形路径
                        //============================================

                        else if(
                            rx_done
                            &&
                            (rx_data == 8'hD2)
                            &&
                            calib_done
                        ) begin

                            route_mode <= ROUTE_D2;

                            sequence_active <= 1'b1;

                            sequence_error <= 1'b0;

                            state <= ST_CLEAR_START;

                        end

                    end


                    //================================================
                    // 启动前清状态
                    //================================================

                    ST_CLEAR_START: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_WAIT_CLEAR;

                    end


                    //================================================
                    // 等20ms
                    //================================================

                    ST_WAIT_CLEAR: begin

                        if(sample_tick) begin

                            if(pause_cnt >= 8'd1) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_FWD1_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // 第一条边：前进500mm
                    //================================================

                    ST_FWD1_START: begin

                        distance_start <= 1'b1;

                        if(distance_auto_active) begin

                            distance_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD1_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                distance_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD1_WAIT: begin

                        if(distance_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD1_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD1_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE1;

                    end


                    ST_PAUSE1: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_TURN1_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // 第一次右转90°
                    //================================================

                    ST_TURN1_START: begin

                        angle_turn_right <= 1'b1;

                        angle_start <= 1'b1;

                        if(angle_auto_active) begin

                            angle_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN1_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                angle_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN1_WAIT: begin

                        if(angle_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN1_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN1_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE2;

                    end


                    ST_PAUSE2: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_FWD2_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // 第二条边：前进500mm
                    //================================================

                    ST_FWD2_START: begin

                        distance_start <= 1'b1;

                        if(distance_auto_active) begin

                            distance_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD2_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                distance_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD2_WAIT: begin

                        if(distance_auto_done) begin

                            step_timeout_cnt <= 10'd0;


                            // B1到这里完成
                            if(route_mode == ROUTE_B1) begin

                                state <= ST_FINAL_STOP;

                            end

                            else begin

                                state <= ST_FWD2_CLEAR;

                            end

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD2_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE3;

                    end


                    ST_PAUSE3: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;


                                // D1进入左移
                                if(route_mode == ROUTE_D1) begin

                                    state <= ST_LATERAL_START;

                                end

                                // D2继续第二个90°
                                else begin

                                    state <= ST_TURN2_START;

                                end

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D1：左移500mm
                    //================================================

                    ST_LATERAL_START: begin

                        lateral_move_right <= 1'b0;

                        lateral_start <= 1'b1;

                        if(lateral_auto_active) begin

                            lateral_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_LATERAL_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                lateral_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_LATERAL_WAIT: begin

                        lateral_move_right <= 1'b0;

                        if(lateral_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FINAL_STOP;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D2：第二次右转90°
                    //================================================

                    ST_TURN2_START: begin

                        angle_turn_right <= 1'b1;

                        angle_start <= 1'b1;

                        if(angle_auto_active) begin

                            angle_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN2_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                angle_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN2_WAIT: begin

                        if(angle_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN2_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN2_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE4;

                    end


                    ST_PAUSE4: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_FWD3_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D2：第三条边
                    //================================================

                    ST_FWD3_START: begin

                        distance_start <= 1'b1;

                        if(distance_auto_active) begin

                            distance_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD3_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                distance_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD3_WAIT: begin

                        if(distance_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD3_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD3_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE5;

                    end


                    ST_PAUSE5: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_TURN3_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D2：第三次右转90°
                    //================================================

                    ST_TURN3_START: begin

                        angle_turn_right <= 1'b1;

                        angle_start <= 1'b1;

                        if(angle_auto_active) begin

                            angle_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN3_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                angle_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN3_WAIT: begin

                        if(angle_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN3_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN3_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE6;

                    end


                    ST_PAUSE6: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_FWD4_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D2：第四条边
                    //================================================

                    ST_FWD4_START: begin

                        distance_start <= 1'b1;

                        if(distance_auto_active) begin

                            distance_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD4_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                distance_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD4_WAIT: begin

                        if(distance_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FWD4_CLEAR;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_FWD4_CLEAR: begin

                        stop_req <= 1'b1;

                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_PAUSE7;

                    end


                    ST_PAUSE7: begin

                        if(sample_tick) begin

                            if(
                                pause_cnt
                                >=
                                PAUSE_TICKS - 8'd1
                            ) begin

                                pause_cnt <= 8'd0;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_TURN4_START;

                            end

                            else begin

                                pause_cnt <= pause_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // D2：第四次右转90°
                    //
                    // 这一转结束以后，理论航向重新回到起点
                    //================================================

                    ST_TURN4_START: begin

                        angle_turn_right <= 1'b1;

                        angle_start <= 1'b1;

                        if(angle_auto_active) begin

                            angle_start <= 1'b0;

                            step_timeout_cnt <= 10'd0;

                            state <= ST_TURN4_WAIT;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                angle_start <= 1'b0;

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    ST_TURN4_WAIT: begin

                        if(angle_auto_done) begin

                            step_timeout_cnt <= 10'd0;

                            state <= ST_FINAL_STOP;

                        end

                        else if(sample_tick) begin

                            if(
                                step_timeout_cnt
                                >=
                                STEP_TIMEOUT_TICKS - 10'd1
                            ) begin

                                stop_req <= 1'b1;

                                sequence_active <= 1'b0;

                                sequence_error <= 1'b1;

                                step_timeout_cnt <= 10'd0;

                                state <= ST_IDLE;

                            end

                            else begin

                                step_timeout_cnt <=
                                    step_timeout_cnt + 1'b1;

                            end

                        end

                    end


                    //================================================
                    // 最终停车
                    //================================================

                    ST_FINAL_STOP: begin

                        distance_start <= 1'b0;

                        angle_start <= 1'b0;

                        lateral_start <= 1'b0;

                        stop_req <= 1'b1;

                        state <= ST_FINISH;

                    end


                    //================================================
                    // 完成
                    //================================================

                    ST_FINISH: begin

                        sequence_active <= 1'b0;

                        sequence_done <= 1'b1;

                        sequence_error <= 1'b0;

                        state <= ST_IDLE;

                    end


                    //================================================
                    // 异常
                    //================================================

                    default: begin

                        distance_start <= 1'b0;

                        angle_start <= 1'b0;

                        lateral_start <= 1'b0;

                        stop_req <= 1'b1;


                        sequence_active <= 1'b0;

                        sequence_done <= 1'b0;

                        sequence_error <= 1'b1;


                        pause_cnt <= 8'd0;

                        step_timeout_cnt <= 10'd0;

                        state <= ST_IDLE;

                    end

                endcase

            end

        end

    end


endmodule