//////////////////////////////////////////////////////////////////////////////////
// File Name   : lateral_distance_controller.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   麦克纳姆轮 Y 轴定距离横移控制器
//
// Command:
//   0xC1 : 右移 500mm
//   0xC2 : 左移 500mm
//   0x53 : Stop
//
// 当前粗标定：
//   6308 count ≈ 1000mm
//   3154 count ≈ 500mm
//
// 约定：
//   +Vy = 右移
//   -Vy = 左移
//////////////////////////////////////////////////////////////////////////////////

module lateral_distance_controller #(

    // 500mm
    parameter signed [31:0] TARGET_COUNT =
        32'sd3154,

    // 剩余约160mm进入减速
    parameter signed [31:0] SLOW_ZONE_COUNT =
        32'sd1000,

    // 初期先提前约48mm撤掉驱动
    // 后续装车完成以后再精调
    parameter signed [31:0] STOP_EARLY_COUNT =
        32'sd300,

    // 横移速度先比前进慢一点
    parameter signed [12:0] FAST_SPEED =
        13'sd24,

    parameter signed [12:0] SLOW_SPEED =
        13'sd8

)(
    input wire clk,
    input wire reset_n,

    // 10ms
    input wire sample_tick,

    // UART
    input wire [7:0] rx_data,
    input wire       rx_done,

    // 后续给自动状态机使用
    input wire ext_start,
    input wire ext_move_right,
    input wire ext_stop,

    input wire calib_done,

    // 注意：
    // 这里必须连接“机体坐标系Y累计count”
    // 不是 world_y_mm
    input wire signed [31:0] pos_y_count,

    // 上一级控制器输入
    input wire signed [12:0] vx_in,
    input wire signed [12:0] vy_in,
    input wire signed [12:0] wz_in,

    // 输出给下一级
    output reg signed [12:0] vx_out,
    output reg signed [12:0] vy_out,
    output reg signed [12:0] wz_out,

    output reg auto_active,
    output reg auto_done,

    // 1 = 右移
    // 0 = 左移
    output reg move_right,

    // 调试使用
    output wire signed [31:0] travelled_count
);


    //================================================
    // 起点
    //================================================

    reg signed [31:0] start_y;


    //================================================
    // 自动横移速度
    //================================================

    reg signed [12:0] auto_vy;


    //================================================
    // 横移距离
    //
    // 无论右移还是左移，都转换为正数距离
    //================================================

    assign travelled_count =
        move_right
        ?
        (pos_y_count - start_y)
        :
        (start_y - pos_y_count);


    //================================================
    // 减速点
    //================================================

    wire signed [31:0] slow_start_count;

    assign slow_start_count =
        TARGET_COUNT - SLOW_ZONE_COUNT;


    //================================================
    // 提前停车点
    //================================================

    wire signed [31:0] stop_count;

    assign stop_count =
        TARGET_COUNT - STOP_EARLY_COUNT;


    //================================================
    // 输出仲裁
    //================================================

    always @(*) begin

        if(auto_active) begin

            // 横移时 X=0
            vx_out = 13'sd0;

            vy_out = auto_vy;

            // 航向修正交给 heading_hold
            wz_out = 13'sd0;

        end

        // 横移任务刚完成以后保持停车
        else if(auto_done) begin

            vx_out = 13'sd0;
            vy_out = 13'sd0;
            wz_out = 13'sd0;

        end

        else begin

            // 不横移时透明传递上一级
            vx_out = vx_in;
            vy_out = vy_in;
            wz_out = wz_in;

        end

    end


    //================================================
    // 横移状态控制
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            start_y <= 32'sd0;

            auto_vy <= 13'sd0;

            auto_active <= 1'b0;
            auto_done   <= 1'b0;

            move_right <= 1'b1;

        end

        else begin


            //================================================
            // 停止
            //================================================

            if(
                ext_stop
                ||
                (
                    rx_done
                    &&
                    (rx_data == 8'h53)
                )
            ) begin

                auto_active <= 1'b0;
                auto_done   <= 1'b0;

                auto_vy <= 13'sd0;

            end


            //================================================
            // C1：右移
            //================================================

            else if(
                rx_done
                &&
                (rx_data == 8'hC1)
                &&
                calib_done
                &&
                !auto_active
            ) begin

                start_y <= pos_y_count;

                move_right <= 1'b1;

                auto_active <= 1'b1;
                auto_done   <= 1'b0;

                auto_vy <= FAST_SPEED;

            end


            //================================================
            // C2：左移
            //================================================

            else if(
                rx_done
                &&
                (rx_data == 8'hC2)
                &&
                calib_done
                &&
                !auto_active
            ) begin

                start_y <= pos_y_count;

                move_right <= 1'b0;

                auto_active <= 1'b1;
                auto_done   <= 1'b0;

                auto_vy <= -FAST_SPEED;

            end


            //================================================
            // 后续状态机启动
            //================================================

            else if(
                ext_start
                &&
                calib_done
                &&
                !auto_active
            ) begin

                start_y <= pos_y_count;

                move_right <= ext_move_right;

                auto_active <= 1'b1;
                auto_done   <= 1'b0;

                if(ext_move_right)
                    auto_vy <= FAST_SPEED;
                else
                    auto_vy <= -FAST_SPEED;

            end


            //================================================
            // 自动横移
            //================================================

            else if(
                auto_active
                &&
                sample_tick
            ) begin


                //============================================
                // 到达提前停车点
                //============================================

                if(
                    travelled_count
                    >=
                    stop_count
                ) begin

                    auto_vy <= 13'sd0;

                    auto_active <= 1'b0;
                    auto_done   <= 1'b1;

                end


                //============================================
                // 减速
                //============================================

                else if(
                    travelled_count
                    >=
                    slow_start_count
                ) begin

                    if(move_right)
                        auto_vy <= SLOW_SPEED;
                    else
                        auto_vy <= -SLOW_SPEED;

                end


                //============================================
                // 正常速度
                //============================================

                else begin

                    if(move_right)
                        auto_vy <= FAST_SPEED;
                    else
                        auto_vy <= -FAST_SPEED;

                end

            end


            //================================================
            // 收到其他新命令以后释放done
            //
            // 这样C1/C2结束后，再发F/A1/B1不会被堵住
            //================================================

            else if(
                auto_done
                &&
                rx_done
                &&
                (rx_data != 8'hC1)
                &&
                (rx_data != 8'hC2)
            ) begin

                auto_done <= 1'b0;

            end

        end

    end


endmodule