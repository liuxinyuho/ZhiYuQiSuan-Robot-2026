//// car_cmd.v
// Description:
//   串口运动命令解析模块。
//   将前后左右、旋转、斜移和速度档位
//   转换为底盘 Vx、Vy、Wz 目标速度.
//
// Command:
//
//   46 = F   前进
//   42 = B   后退
//   4C = L   左移
//   52 = R   右移
//   51 = Q   左旋
//   45 = E   右旋
//   53 = S   停止
//
//   F1 = 左前
//   F2 = 右前
//   F3 = 左后
//   F4 = 右后
//
//   31 = 1档
//   32 = 2档
//   33 = 3档
//   34 = 4档
//
module car_cmd(

    input wire clk,
    input wire reset_n,

    input wire [7:0] rx_data,
    input wire       rx_done,

    output reg signed [12:0] vx_cmd,
    output reg signed [12:0] vy_cmd,
    output reg signed [12:0] wz_cmd

);


    //================================================
    // 目标速度
    //
    // 单位：
    // count / 10ms
    //================================================

    reg [11:0] speed_level;


    //================================================
    // 运动模式
    //
    // 原来3bit只能表示0~7
    // 现在增加4种斜移，所以扩大为4bit
    //================================================

    reg [3:0] motion_mode;


    localparam [3:0]

        MODE_STOP = 4'd0,

        MODE_F    = 4'd1,
        MODE_B    = 4'd2,

        MODE_L    = 4'd3,
        MODE_R    = 4'd4,

        MODE_Q    = 4'd5,
        MODE_E    = 4'd6,

        //============================================
        // Diagonal
        //============================================

        MODE_LF   = 4'd7,    // 左前
        MODE_RF   = 4'd8,    // 右前
        MODE_LB   = 4'd9,    // 左后
        MODE_RB   = 4'd10;   // 右后


    //================================================
    // 斜移分量
    //
    // 为了避免：
    //
    // Vx = 32
    // Vy = 32
    //
    // 导致某些轮子目标速度 = 64，
    //
    // 第一版斜移使用：
    //
    // diagonal_speed = speed_level / 2
    //
    // 比较安全。
    //================================================

    wire signed [12:0] diagonal_speed;


    assign diagonal_speed =
        $signed({
            1'b0,
            (speed_level >> 1)
        });


    //================================================
    // UART command decoder
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            // 默认档位2
            speed_level <= 12'd32;

            motion_mode <= MODE_STOP;

        end


        else if(rx_done) begin

            case(rx_data)


                //================================================
                // Basic motion
                //================================================


                // F 前进
                8'h46:
                    motion_mode <= MODE_F;


                // B 后退
                8'h42:
                    motion_mode <= MODE_B;


                // L 左移
                8'h4C:
                    motion_mode <= MODE_L;


                // R 右移
                8'h52:
                    motion_mode <= MODE_R;


                // Q 左旋
                8'h51:
                    motion_mode <= MODE_Q;


                // E 右旋
                8'h45:
                    motion_mode <= MODE_E;


                // S 停止
                8'h53:
                    motion_mode <= MODE_STOP;


                //================================================
                // Diagonal motion
                //================================================


                // F1 左前
                8'hF1:
                    motion_mode <= MODE_LF;


                // F2 右前
                8'hF2:
                    motion_mode <= MODE_RF;


                // F3 左后
                8'hF3:
                    motion_mode <= MODE_LB;


                // F4 右后
                8'hF4:
                    motion_mode <= MODE_RB;


                //================================================
                // Speed level
                //
                // count / 10ms
                //================================================


                // 1档
                8'h31:
                    speed_level <= 12'd16;


                // 2档
                8'h32:
                    speed_level <= 12'd32;


                // 3档
                8'h33:
                    speed_level <= 12'd48;


                // 4档
                8'h34:
                    speed_level <= 12'd64;


                default: begin

                    motion_mode <= motion_mode;

                    speed_level <= speed_level;

                end


            endcase

        end

    end


    //================================================
    // Motion mode -> Vx / Vy / Wz
    //
    // 当前坐标约定：
    //
    // +Vx = 前
    // -Vx = 后
    //
    // +Vy = 右
    // -Vy = 左
    //
    // +Wz = 右旋
    // -Wz = 左旋
    //================================================

    always @(*) begin


        // 默认停止
        vx_cmd = 13'sd0;
        vy_cmd = 13'sd0;
        wz_cmd = 13'sd0;


        case(motion_mode)


            //================================================
            // 前进
            //================================================

            MODE_F: begin

                vx_cmd =
                    $signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 后退
            //================================================

            MODE_B: begin

                vx_cmd =
                    -$signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 左移
            //================================================

            MODE_L: begin

                vy_cmd =
                    -$signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 右移
            //================================================

            MODE_R: begin

                vy_cmd =
                    $signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 左旋
            //================================================

            MODE_Q: begin

                wz_cmd =
                    -$signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 右旋
            //================================================

            MODE_E: begin

                wz_cmd =
                    $signed({
                        1'b0,
                        speed_level
                    });

            end


            //================================================
            // 左前
            //
            // Vx +
            // Vy -
            //================================================

            MODE_LF: begin

                vx_cmd = diagonal_speed;

                vy_cmd = -diagonal_speed;

                wz_cmd = 13'sd0;

            end


            //================================================
            // 右前
            //
            // Vx +
            // Vy +
            //================================================

            MODE_RF: begin

                vx_cmd = diagonal_speed;

                vy_cmd = diagonal_speed;

                wz_cmd = 13'sd0;

            end


            //================================================
            // 左后
            //
            // Vx -
            // Vy -
            //================================================

            MODE_LB: begin

                vx_cmd = -diagonal_speed;

                vy_cmd = -diagonal_speed;

                wz_cmd = 13'sd0;

            end


            //================================================
            // 右后
            //
            // Vx -
            // Vy +
            //================================================

            MODE_RB: begin

                vx_cmd = -diagonal_speed;

                vy_cmd = diagonal_speed;

                wz_cmd = 13'sd0;

            end


            //================================================
            // STOP
            //================================================

            MODE_STOP: begin

                vx_cmd = 13'sd0;
                vy_cmd = 13'sd0;
                wz_cmd = 13'sd0;

            end


            default: begin

                vx_cmd = 13'sd0;
                vy_cmd = 13'sd0;
                wz_cmd = 13'sd0;

            end


        endcase

    end


endmodule