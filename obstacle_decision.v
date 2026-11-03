//////////////////////////////////////////////////////////////////////////////////
// File Name : obstacle_decision.v
//
// Function:
//   根据三方向超声波距离进行避障决策
//
// Decision:
//   center > 300mm       -> CLEAR
//   center <= 300mm      -> 有障碍
//      left > right      -> LEFT
//      right >= left     -> RIGHT
//
// Clock : 50MHz
//////////////////////////////////////////////////////////////////////////////////

module obstacle_decision #(

    parameter [15:0] FRONT_THRESHOLD = 16'd300,
    parameter [15:0] SIDE_MIN        = 16'd250

)(

    input wire clk,
    input wire reset_n,

    // 扫描完成脉冲
    input wire scan_done,

    // 三方向距离，单位 mm
    input wire [15:0] left_distance_mm,
    input wire [15:0] center_distance_mm,
    input wire [15:0] right_distance_mm,

    //================================================
    // Decision output
    //================================================

    output reg path_clear,

    output reg obstacle_detected,

    output reg avoid_left,

    output reg avoid_right,

    output reg no_path,

    // 新决策完成脉冲
    output reg decision_valid

);


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            path_clear        <= 1'b0;

            obstacle_detected <= 1'b0;

            avoid_left        <= 1'b0;

            avoid_right       <= 1'b0;

            no_path           <= 1'b0;

            decision_valid    <= 1'b0;

        end


        else begin

            // 默认只保持1个时钟
            decision_valid <= 1'b0;


            if(scan_done) begin

                // 每次新判断先清空
                path_clear        <= 1'b0;
                obstacle_detected <= 1'b0;
                avoid_left        <= 1'b0;
                avoid_right       <= 1'b0;
                no_path           <= 1'b0;


                //================================================
                // 前方安全
                //================================================

                if(
                    center_distance_mm
                    >
                    FRONT_THRESHOLD
                ) begin

                    path_clear <= 1'b1;

                end


                //================================================
                // 前方存在障碍
                //================================================

                else begin

                    obstacle_detected <= 1'b1;


                    //============================================
                    // 左右都太窄
                    //============================================

                    if(
                        left_distance_mm < SIDE_MIN
                        &&
                        right_distance_mm < SIDE_MIN
                    ) begin

                        no_path <= 1'b1;

                    end


                    //============================================
                    // 左边空间更大
                    //============================================

                    else if(
                        left_distance_mm
                        >
                        right_distance_mm
                    ) begin

                        avoid_left <= 1'b1;

                    end


                    //============================================
                    // 右边空间更大
                    //============================================

                    else begin

                        avoid_right <= 1'b1;

                    end

                end


                decision_valid <= 1'b1;

            end

        end

    end


endmodule