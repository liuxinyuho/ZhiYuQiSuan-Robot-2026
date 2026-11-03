//////////////////////////////////////////////////////////////////////////////////
// File Name   : speed_pi.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   单轮速度 PI 闭环控制模块
//
// Optimized Version:
//   多时钟流水 PI
//
// Function:
//   1. PI速度闭环
//   2. PWM限幅
//   3. 积分限幅
//   4. Anti-Windup
//   5. 停车清积分
//   6. 正反转切换保护
//   7. 降低组合逻辑路径，改善50MHz时序
//
// Input:
//   sample_tick    - 10ms控制周期
//   target_speed   - 目标速度 count/10ms
//   feedback_speed - 编码器实际速度 count/10ms
//
// Output:
//   pwm_cmd        - -2499 ~ +2499
//
// Timing:
//
//   sample_tick
//       |
//       v
//   CAPTURE
//       |
//       v
//   ERROR
//       |
//       v
//   SUM + P
//       |
//       v
//   CLAMP
//       |
//       v
//   I
//       |
//       v
//   CONTROL
//       |
//       v
//   LIMIT
//       |
//       v
//   COMMIT
//
// 每次PI计算约7个50MHz时钟
// 约140ns
//
// 相对于10ms控制周期可忽略。
//////////////////////////////////////////////////////////////////////////////////

module speed_pi #(

    //================================================
    // PI参数
    //================================================

    parameter signed [15:0] KP =
        16'sd25,

    parameter signed [15:0] KI =
        16'sd1,


    //================================================
    // PWM最大值
    //================================================

    parameter signed [31:0] PWM_MAX =
        32'sd2499,


    //================================================
    // 积分最大值
    //================================================

    parameter signed [31:0] INTEGRAL_MAX =
        32'sd2000

)(

    input wire clk,

    input wire reset_n,

    input wire sample_tick,

    input wire signed [12:0] target_speed,

    input wire signed [31:0] feedback_speed,

    output reg signed [12:0] pwm_cmd

);


    //================================================
    // Pipeline State
    //================================================

    localparam [2:0]

        ST_IDLE    = 3'd0,

        ST_ERROR   = 3'd1,

        ST_SUM_P   = 3'd2,

        ST_CLAMP   = 3'd3,

        ST_I       = 3'd4,

        ST_CONTROL = 3'd5,

        ST_LIMIT   = 3'd6,

        ST_COMMIT  = 3'd7;


    reg [2:0] state;


    //================================================
    // Input registers
    //
    // sample_tick到来时锁存输入
    //
    // 后续所有PI计算都使用寄存器值，
    // 防止输入变化形成长组合路径。
    //================================================

    reg signed [12:0] target_reg;

    reg signed [31:0] target_ext_reg;

    reg signed [31:0] feedback_reg;


    //================================================
    // 上一个目标速度
    //
    // 用于判断正反转切换
    //================================================

    reg signed [12:0] target_prev;


    //================================================
    // Special control flags
    //================================================

    reg stop_request;

    reg reverse_request;


    //================================================
    // Error
    //================================================

    reg signed [31:0] error_reg;


    //================================================
    // Integral
    //================================================

    reg signed [31:0] integral;


    // 33bit防止32bit加法溢出
    reg signed [32:0] integral_sum_reg;


    // 限幅后的积分候选值
    reg signed [31:0] integral_candidate_reg;


    // 最终下一次需要写入的积分
    reg signed [31:0] integral_next_reg;


    //================================================
    // PI intermediate registers
    //================================================

    reg signed [47:0] p_term_reg;

    reg signed [47:0] i_term_reg;

    reg signed [47:0] control_reg;


    //================================================
    // PWM next register
    //================================================

    reg signed [12:0] pwm_next_reg;


    //================================================
    // Integral limit extension
    //
    // 32bit -> 33bit
    //================================================

    wire signed [32:0] integral_max_ext;

    wire signed [32:0] integral_min_ext;


    assign integral_max_ext =

        {
            INTEGRAL_MAX[31],
            INTEGRAL_MAX
        };


    assign integral_min_ext =

        -integral_max_ext;


    //================================================
    // PWM limit extension
    //
    // 32bit -> 48bit
    //================================================

    wire signed [47:0] pwm_max_ext;

    wire signed [47:0] pwm_min_ext;


    assign pwm_max_ext =

        {
            {16{PWM_MAX[31]}},
            PWM_MAX
        };


    assign pwm_min_ext =

        -pwm_max_ext;


    //================================================
    // 13bit PWM maximum
    //================================================

    wire signed [12:0] pwm_max_13;


    assign pwm_max_13 =

        PWM_MAX[12:0];


    //================================================
    // Main Pipeline
    //================================================

    always @(posedge clk or negedge reset_n) begin


        if(!reset_n) begin


            state <=
                ST_IDLE;


            target_reg <=
                13'sd0;


            target_ext_reg <=
                32'sd0;


            feedback_reg <=
                32'sd0;


            target_prev <=
                13'sd0;


            stop_request <=
                1'b0;


            reverse_request <=
                1'b0;


            error_reg <=
                32'sd0;


            integral <=
                32'sd0;


            integral_sum_reg <=
                33'sd0;


            integral_candidate_reg <=
                32'sd0;


            integral_next_reg <=
                32'sd0;


            p_term_reg <=
                48'sd0;


            i_term_reg <=
                48'sd0;


            control_reg <=
                48'sd0;


            pwm_next_reg <=
                13'sd0;


            pwm_cmd <=
                13'sd0;


        end


        else begin


            case(state)


                //================================================
                // IDLE
                //
                // 等待10ms控制周期
                //================================================

                ST_IDLE: begin


                    if(sample_tick) begin


                        //========================================
                        // 锁存目标
                        //========================================

                        target_reg <=
                            target_speed;


                        //========================================
                        // 13bit -> 32bit signed
                        //========================================

                        target_ext_reg <=
                            {
                                {19{target_speed[12]}},
                                target_speed
                            };


                        //========================================
                        // 锁存反馈速度
                        //========================================

                        feedback_reg <=
                            feedback_speed;


                        //========================================
                        // 是否停车
                        //========================================

                        stop_request <=

                            (
                                target_speed
                                ==
                                13'sd0
                            );


                        //========================================
                        // 是否正反方向切换
                        //
                        // +20 -> -20
                        // -20 -> +20
                        //========================================

                        reverse_request <=

                            (
                                target_prev
                                !=
                                13'sd0
                            )

                            &&

                            (
                                target_prev[12]
                                !=
                                target_speed[12]
                            );


                        //========================================
                        // 保存当前目标
                        //========================================

                        target_prev <=
                            target_speed;


                        state <=
                            ST_ERROR;


                    end


                end


                //================================================
                // Stage 1
                //
                // 停车 / 换向判断
                //
                // 或计算速度误差
                //================================================

                ST_ERROR: begin


                    //============================================
                    // 停车
                    //============================================

                    if(stop_request) begin


                        integral_next_reg <=
                            32'sd0;


                        pwm_next_reg <=
                            13'sd0;


                        state <=
                            ST_COMMIT;


                    end


                    //============================================
                    // 正反转突然切换
                    //
                    // 先停车一个PI周期
                    //============================================

                    else if(reverse_request) begin


                        integral_next_reg <=
                            32'sd0;


                        pwm_next_reg <=
                            13'sd0;


                        state <=
                            ST_COMMIT;


                    end


                    //============================================
                    // 正常计算error
                    //
                    // error = target - feedback
                    //============================================

                    else begin


                        error_reg <=

                            target_ext_reg

                            -

                            feedback_reg;


                        state <=
                            ST_SUM_P;


                    end


                end


                //================================================
                // Stage 2
                //
                // 积分加法
                // +
                // P项
                //
                // 把两个大运算放这一拍，
                // 输出全部进入寄存器。
                //================================================

                ST_SUM_P: begin


                    //============================================
                    // Integral Sum
                    //============================================

                    integral_sum_reg <=

                        {
                            integral[31],
                            integral
                        }

                        +

                        {
                            error_reg[31],
                            error_reg
                        };


                    //============================================
                    // P term
                    //============================================

                    p_term_reg <=

                        error_reg

                        *

                        KP;


                    state <=
                        ST_CLAMP;


                end


                //================================================
                // Stage 3
                //
                // Integral Clamp
                //================================================

                ST_CLAMP: begin


                    if(
                        integral_sum_reg
                        >
                        integral_max_ext
                    ) begin


                        integral_candidate_reg <=
                            INTEGRAL_MAX;


                    end


                    else if(
                        integral_sum_reg
                        <
                        integral_min_ext
                    ) begin


                        integral_candidate_reg <=
                            -INTEGRAL_MAX;


                    end


                    else begin


                        integral_candidate_reg <=
                            integral_sum_reg[31:0];


                    end


                    state <=
                        ST_I;


                end


                //================================================
                // Stage 4
                //
                // I term
                //================================================

                ST_I: begin


                    i_term_reg <=

                        integral_candidate_reg

                        *

                        KI;


                    state <=
                        ST_CONTROL;


                end


                //================================================
                // Stage 5
                //
                // PI Sum
                //
                // control = P + I
                //================================================

                ST_CONTROL: begin


                    control_reg <=

                        p_term_reg

                        +

                        i_term_reg;


                    state <=
                        ST_LIMIT;


                end


                //================================================
                // Stage 6
                //
                // PWM limit
                // +
                // Anti-Windup decision
                //
                // 注意：
                //
                // 这里只计算NEXT值，
                // 不直接修改integral。
                //
                // 所以Timing路径不会直接进入
                // integral寄存器CE。
                //================================================

                ST_LIMIT: begin


                    //============================================
                    // Positive saturation
                    //============================================

                    if(
                        control_reg
                        >
                        pwm_max_ext
                    ) begin


                        pwm_next_reg <=
                            pwm_max_13;


                        //========================================
                        // 已经正饱和
                        //
                        // error > 0仍想继续正向积分
                        //
                        // Anti-Windup：
                        // 保持旧积分
                        //========================================

                        if(
                            error_reg
                            >
                            32'sd0
                        ) begin


                            integral_next_reg <=
                                integral;


                        end


                        else begin


                            integral_next_reg <=
                                integral_candidate_reg;


                        end


                    end


                    //============================================
                    // Negative saturation
                    //============================================

                    else if(
                        control_reg
                        <
                        pwm_min_ext
                    ) begin


                        pwm_next_reg <=
                            -pwm_max_13;


                        //========================================
                        // 已经负饱和
                        //
                        // error < 0仍想继续负向积分
                        //
                        // Anti-Windup：
                        // 保持旧积分
                        //========================================

                        if(
                            error_reg
                            <
                            32'sd0
                        ) begin


                            integral_next_reg <=
                                integral;


                        end


                        else begin


                            integral_next_reg <=
                                integral_candidate_reg;


                        end


                    end


                    //============================================
                    // Normal range
                    //============================================

                    else begin


                        pwm_next_reg <=
                            control_reg[12:0];


                        integral_next_reg <=
                            integral_candidate_reg;


                    end


                    state <=
                        ST_COMMIT;


                end


                //================================================
                // Stage 7
                //
                // Commit Result
                //
                // integral真正更新只有这一处。
                //
                // 输入全部来自前一级寄存器，
                // 这里几乎没有组合路径。
                //================================================

                ST_COMMIT: begin


                    integral <=
                        integral_next_reg;


                    pwm_cmd <=
                        pwm_next_reg;


                    state <=
                        ST_IDLE;


                end


                //================================================
                // Safety
                //================================================

                default: begin


                    state <=
                        ST_IDLE;


                    integral <=
                        32'sd0;


                    pwm_cmd <=
                        13'sd0;


                end


            endcase


        end


    end


endmodule