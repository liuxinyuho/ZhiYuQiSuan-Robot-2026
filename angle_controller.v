//////////////////////////////////////////////////////////////////////////////////
// File Name   : angle_controller.v
// Project     : FPGA Mecanum Robot Chassis
//
// 功能：
//   A1 -> 原地右转90°
//   A2 -> 原地左转90°
//   53 -> 停止/清除任务
//
// 外部状态机：
//   ext_start      -> 启动一次90°旋转
//   ext_turn_right -> 1=右转，0=左转
//   ext_stop       -> 停止/清除任务
//
// yaw单位：0.01°
// 90° = 9000
//
// 安全功能：
//   1. 正常到达目标角度自动停车
//   2. 超过3秒仍未完成则强制停车
//   3. auto_done=1 后锁定，不允许重复触发
//      必须先通过 53 或 ext_stop 清除
//////////////////////////////////////////////////////////////////////////////////

module angle_controller #(

    //================================================
    // 目标旋转角度：90°
    //================================================
    parameter signed [31:0] TARGET_ANGLE_CDEG =
        32'sd9000,

    //================================================
    // 最后15°进入低速
    //================================================
    parameter signed [31:0] SLOW_ZONE_CDEG =
        32'sd1500,

    //================================================
    // 提前停车补偿
    //
    // 当前实测：
    // 9000 - 1100 = 7900
    //
    // 约79°先停止驱动，
    // 依靠惯性最终接近90°
    //================================================
    parameter signed [31:0] STOP_EARLY_CDEG =
        32'sd1100,

    //================================================
    // 快速旋转速度
    //================================================
    parameter signed [12:0] FAST_WZ =
        13'sd16,

    //================================================
    // 慢速旋转速度
    //================================================
    parameter signed [12:0] SLOW_WZ =
        13'sd8,

    //================================================
    // 最大旋转时间
    //
    // sample_tick = 10ms
    // 300 × 10ms = 3秒
    //================================================
    parameter [8:0] MAX_TURN_TICKS =
        9'd300

)(
    input wire clk,
    input wire reset_n,

    //================================================
    // 10ms控制周期
    //================================================
    input wire sample_tick,

    //================================================
    // UART
    //================================================
    input wire [7:0] rx_data,
    input wire       rx_done,

    //================================================
    // 外部状态机控制接口
    //================================================
    input wire ext_start,

    // 1 = 右转
    // 0 = 左转
    input wire ext_turn_right,

    input wire ext_stop,

    //================================================
    // IMU校准完成
    //================================================
    input wire calib_done,

    //================================================
    // 当前Yaw
    // 单位：0.01°
    //================================================
    input wire signed [31:0] yaw_cdeg,

    //================================================
    // 上一级控制命令
    //================================================
    input wire signed [12:0] vx_in,
    input wire signed [12:0] vy_in,
    input wire signed [12:0] wz_in,

    //================================================
    // 最终输出
    //================================================
    output reg signed [12:0] vx_out,
    output reg signed [12:0] vy_out,
    output reg signed [12:0] wz_out,

    //================================================
    // 调试
    //================================================
    output reg auto_active,
    output reg auto_done,

    output wire signed [31:0] angle_travelled_cdeg
);


    //================================================
    // 旋转开始时Yaw
    //================================================

    reg signed [31:0] start_yaw;


    //================================================
    // 方向
    //
    // 1 = 右转
    // 0 = 左转
    //================================================

    reg turn_right;


    //================================================
    // 自动旋转速度
    //================================================

    reg signed [12:0] auto_wz;


    //================================================
    // 超时计数
    //
    // 每10ms +1
    //================================================

    reg [8:0] turn_timeout_cnt;


    //================================================
    // 当前Yaw与起始Yaw差值
    //================================================

    wire signed [31:0] yaw_delta_raw;

    assign yaw_delta_raw =
        yaw_cdeg - start_yaw;


    //================================================
    // 处理 ±180° 跨界
    //================================================

    reg signed [31:0] yaw_delta_norm;

    always @(*) begin

        if(yaw_delta_raw > 32'sd18000) begin

            yaw_delta_norm =
                yaw_delta_raw - 32'sd36000;

        end

        else if(yaw_delta_raw < -32'sd18000) begin

            yaw_delta_norm =
                yaw_delta_raw + 32'sd36000;

        end

        else begin

            yaw_delta_norm =
                yaw_delta_raw;

        end

    end


    //================================================
    // 已旋转角度
    //
    // 只取绝对值
    //================================================

    assign angle_travelled_cdeg =
        (yaw_delta_norm < 0)
        ?
        -yaw_delta_norm
        :
        yaw_delta_norm;


    //================================================
    // 开始减速位置
    //
    // 9000 - 1500 = 7500
    //================================================

    wire signed [31:0] slow_start_angle;

    assign slow_start_angle =
        TARGET_ANGLE_CDEG -
        SLOW_ZONE_CDEG;


    //================================================
    // 停车触发角度
    //
    // 9000 - 1100 = 7900
    //================================================

    wire signed [31:0] stop_angle;

    assign stop_angle =
        TARGET_ANGLE_CDEG -
        STOP_EARLY_CDEG;


    //================================================
    // 输出选择
    //================================================

    always @(*) begin

        //============================================
        // 自动旋转
        //============================================

        if(auto_active) begin

            vx_out = 13'sd0;
            vy_out = 13'sd0;
            wz_out = auto_wz;

        end


        //============================================
        // 自动任务完成
        //
        // 保持停车
        //============================================

        else if(auto_done) begin

            vx_out = 13'sd0;
            vy_out = 13'sd0;
            wz_out = 13'sd0;

        end


        //============================================
        // 普通模式
        //============================================

        else begin

            vx_out = vx_in;
            vy_out = vy_in;
            wz_out = wz_in;

        end

    end


    //================================================
    // 自动转角控制
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            start_yaw <= 32'sd0;

            turn_right <= 1'b1;

            auto_wz <= 13'sd0;

            auto_active <= 1'b0;

            auto_done <= 1'b0;

            turn_timeout_cnt <= 9'd0;

        end

        else begin


            //================================================
            // 停止 / 清除
            //
            // 优先级最高
            //
            // 53 = ASCII 'S'
            //================================================

            if(
                ext_stop
                ||
                (
                    rx_done
                    &&
                    rx_data == 8'h53
                )
            ) begin

                auto_active <= 1'b0;

                auto_done <= 1'b0;

                auto_wz <= 13'sd0;

                turn_timeout_cnt <= 9'd0;

            end


            //================================================
            // 外部状态机启动
            //
            // 必须：
            // auto_active = 0
            // auto_done   = 0
            //
            // 防止重复启动
            //================================================

            else if(
                ext_start
                &&
                calib_done
                &&
                !auto_active
                &&
                !auto_done
            ) begin

                start_yaw <= yaw_cdeg;

                turn_right <= ext_turn_right;


                if(ext_turn_right) begin

                    auto_wz <= FAST_WZ;

                end

                else begin

                    auto_wz <= -FAST_WZ;

                end


                auto_active <= 1'b1;

                auto_done <= 1'b0;

                turn_timeout_cnt <= 9'd0;

            end


            //================================================
            // A1 = 自动右转90°
            //
            // auto_done=1 时禁止再次响应A1
            // 防止一次命令被重复触发导致180°
            //================================================

            else if(
                rx_done
                &&
                rx_data == 8'hA1
                &&
                calib_done
                &&
                !auto_active
                &&
                !auto_done
            ) begin

                start_yaw <= yaw_cdeg;

                turn_right <= 1'b1;

                auto_wz <= FAST_WZ;

                auto_active <= 1'b1;

                auto_done <= 1'b0;

                turn_timeout_cnt <= 9'd0;

            end


            //================================================
            // A2 = 自动左转90°
            //
            // auto_done=1 时禁止再次响应A2
            //================================================

            else if(
                rx_done
                &&
                rx_data == 8'hA2
                &&
                calib_done
                &&
                !auto_active
                &&
                !auto_done
            ) begin

                start_yaw <= yaw_cdeg;

                turn_right <= 1'b0;

                auto_wz <= -FAST_WZ;

                auto_active <= 1'b1;

                auto_done <= 1'b0;

                turn_timeout_cnt <= 9'd0;

            end


            //================================================
            // 自动旋转控制
            //================================================

            else if(
                auto_active
                &&
                sample_tick
            ) begin


                //============================================
                // 1. 正常达到停车位置
                //============================================

                if(
                    angle_travelled_cdeg
                    >=
                    stop_angle
                ) begin

                    auto_wz <= 13'sd0;

                    auto_active <= 1'b0;

                    // 正常完成后置1
                    // 后续A1/A2被锁定
                    auto_done <= 1'b1;

                    turn_timeout_cnt <= 9'd0;

                end


                //============================================
                // 2. 3秒安全超时
                //
                // 如果IMU异常或者角度没有变化，
                // 不允许小车一直旋转
                //============================================

                else if(
                    turn_timeout_cnt
                    >=
                    MAX_TURN_TICKS - 9'd1
                ) begin

                    auto_wz <= 13'sd0;

                    auto_active <= 1'b0;

                    // 超时不算正常完成
                    auto_done <= 1'b0;

                    turn_timeout_cnt <= 9'd0;

                end


                //============================================
                // 3. 继续旋转
                //============================================

                else begin

                    turn_timeout_cnt <=
                        turn_timeout_cnt + 1'b1;


                    //========================================
                    // 接近目标 → 低速
                    //========================================

                    if(
                        angle_travelled_cdeg
                        >=
                        slow_start_angle
                    ) begin

                        if(turn_right) begin

                            auto_wz <= SLOW_WZ;

                        end

                        else begin

                            auto_wz <= -SLOW_WZ;

                        end

                    end


                    //========================================
                    // 正常快速旋转
                    //========================================

                    else begin

                        if(turn_right) begin

                            auto_wz <= FAST_WZ;

                        end

                        else begin

                            auto_wz <= -FAST_WZ;

                        end

                    end

                end

            end

        end

    end


endmodule