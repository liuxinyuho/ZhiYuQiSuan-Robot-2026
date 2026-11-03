`timescale 1ns / 1ps

//============================================================
// ACG720-GW5AT60K 四电机硬件自检
//
// 时钟：50MHz
//
// 测试顺序，每个状态持续3秒：
//
// 0  停止
// 1  四轮正转
// 2  停止
// 3  四轮反转
// 4  停止
// 5  右横移组合
// 6  停止
// 7  左横移组合
// 8  停止
// 9  顺时针原地旋转
// 10 停止
// 11 逆时针原地旋转
// 然后循环
//
// 电机：
// A = 左前
// B = 左后
// C = 右前
// D = 右后
//
// 本程序完全绕过：
// UART / car_cmd / mecanum / PI / Encoder / IMU
//============================================================

module motor_io_test_top
(
    input wire clk,
    input wire reset_n,

    //========================================================
    // Motor A 左前
    //========================================================
    output wire motor_a_pwm,
    output reg  motor_a_in1,
    output reg  motor_a_in2,

    //========================================================
    // Motor B 左后
    //========================================================
    output wire motor_b_pwm,
    output reg  motor_b_in1,
    output reg  motor_b_in2,

    //========================================================
    // Motor C 右前
    //========================================================
    output wire motor_c_pwm,
    output reg  motor_c_in1,
    output reg  motor_c_in2,

    //========================================================
    // Motor D 右后
    //========================================================
    output wire motor_d_pwm,
    output reg  motor_d_in1,
    output reg  motor_d_in2
);


//============================================================
// 参数
//============================================================

// 50MHz / 2500 = 20kHz PWM
localparam integer PWM_PERIOD = 2500;

// 20%占空比
localparam integer PWM_DUTY = 500;

// 每个测试状态3秒
// 50MHz × 3s = 150,000,000
localparam integer STATE_TIME = 150_000_000;


//============================================================
// 测试状态
//============================================================

localparam [3:0]

    ST_STOP_0      = 4'd0,

    ST_FORWARD     = 4'd1,

    ST_STOP_1      = 4'd2,

    ST_BACKWARD    = 4'd3,

    ST_STOP_2      = 4'd4,

    ST_STRAFE_R    = 4'd5,

    ST_STOP_3      = 4'd6,

    ST_STRAFE_L    = 4'd7,

    ST_STOP_4      = 4'd8,

    ST_ROTATE_CW   = 4'd9,

    ST_STOP_5      = 4'd10,

    ST_ROTATE_CCW  = 4'd11;


//============================================================
// 状态计数
//============================================================

reg [27:0] state_cnt;

reg [3:0] state;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        state_cnt <= 28'd0;

        state <= ST_STOP_0;

    end

    else
    begin

        if(state_cnt >= STATE_TIME - 1)
        begin

            state_cnt <= 28'd0;

            if(state >= ST_ROTATE_CCW)

                state <= ST_STOP_0;

            else

                state <= state + 1'b1;

        end

        else
        begin

            state_cnt <= state_cnt + 1'b1;

        end

    end

end


//============================================================
// PWM发生器
//
// 20kHz
// 20%占空比
//============================================================

reg [11:0] pwm_cnt;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        pwm_cnt <= 12'd0;

    end

    else
    begin

        if(pwm_cnt >= PWM_PERIOD - 1)

            pwm_cnt <= 12'd0;

        else

            pwm_cnt <= pwm_cnt + 1'b1;

    end

end


//============================================================
// 是否处于停车状态
//============================================================

wire motor_enable;


assign motor_enable =

       (state == ST_FORWARD)

    || (state == ST_BACKWARD)

    || (state == ST_STRAFE_R)

    || (state == ST_STRAFE_L)

    || (state == ST_ROTATE_CW)

    || (state == ST_ROTATE_CCW);


//============================================================
// PWM输出
//
// 停止状态直接关闭PWM
//============================================================

wire pwm_wave;


assign pwm_wave =
    motor_enable
    &&
    (pwm_cnt < PWM_DUTY);


assign motor_a_pwm = pwm_wave;

assign motor_b_pwm = pwm_wave;

assign motor_c_pwm = pwm_wave;

assign motor_d_pwm = pwm_wave;


//============================================================
// 电机方向控制
//
// 正转：
// IN1 = 1
// IN2 = 0
//
// 反转：
// IN1 = 0
// IN2 = 1
//
// 停止：
// IN1 = 0
// IN2 = 0
//============================================================

always @*
begin

    //========================================================
    // 默认全部停止
    //========================================================

    motor_a_in1 = 1'b0;
    motor_a_in2 = 1'b0;

    motor_b_in1 = 1'b0;
    motor_b_in2 = 1'b0;

    motor_c_in1 = 1'b0;
    motor_c_in2 = 1'b0;

    motor_d_in1 = 1'b0;
    motor_d_in2 = 1'b0;


    case(state)


        //====================================================
        // 四轮全部正转
        //
        // A +
        // B +
        // C +
        // D +
        //====================================================

        ST_FORWARD:
        begin

            motor_a_in1 = 1'b1;
            motor_a_in2 = 1'b0;

            motor_b_in1 = 1'b1;
            motor_b_in2 = 1'b0;

            motor_c_in1 = 1'b1;
            motor_c_in2 = 1'b0;

            motor_d_in1 = 1'b1;
            motor_d_in2 = 1'b0;

        end


        //====================================================
        // 四轮全部反转
        //
        // A -
        // B -
        // C -
        // D -
        //====================================================

        ST_BACKWARD:
        begin

            motor_a_in1 = 1'b0;
            motor_a_in2 = 1'b1;

            motor_b_in1 = 1'b0;
            motor_b_in2 = 1'b1;

            motor_c_in1 = 1'b0;
            motor_c_in2 = 1'b1;

            motor_d_in1 = 1'b0;
            motor_d_in2 = 1'b1;

        end


        //====================================================
        // 右横移
        //
        // A +
        // B -
        // C -
        // D +
        //
        // 如果你实际车轮安装方向相反，
        // 可能表现为左横移，不影响本次硬件判断。
        //====================================================

        ST_STRAFE_R:
        begin

            motor_a_in1 = 1'b1;
            motor_a_in2 = 1'b0;

            motor_b_in1 = 1'b0;
            motor_b_in2 = 1'b1;

            motor_c_in1 = 1'b0;
            motor_c_in2 = 1'b1;

            motor_d_in1 = 1'b1;
            motor_d_in2 = 1'b0;

        end


        //====================================================
        // 左横移
        //
        // A -
        // B +
        // C +
        // D -
        //====================================================

        ST_STRAFE_L:
        begin

            motor_a_in1 = 1'b0;
            motor_a_in2 = 1'b1;

            motor_b_in1 = 1'b1;
            motor_b_in2 = 1'b0;

            motor_c_in1 = 1'b1;
            motor_c_in2 = 1'b0;

            motor_d_in1 = 1'b0;
            motor_d_in2 = 1'b1;

        end


        //====================================================
        // 顺时针原地旋转
        //
        // A +
        // B +
        // C -
        // D -
        //====================================================

        ST_ROTATE_CW:
        begin

            motor_a_in1 = 1'b1;
            motor_a_in2 = 1'b0;

            motor_b_in1 = 1'b1;
            motor_b_in2 = 1'b0;

            motor_c_in1 = 1'b0;
            motor_c_in2 = 1'b1;

            motor_d_in1 = 1'b0;
            motor_d_in2 = 1'b1;

        end


        //====================================================
        // 逆时针原地旋转
        //
        // A -
        // B -
        // C +
        // D +
        //====================================================

        ST_ROTATE_CCW:
        begin

            motor_a_in1 = 1'b0;
            motor_a_in2 = 1'b1;

            motor_b_in1 = 1'b0;
            motor_b_in2 = 1'b1;

            motor_c_in1 = 1'b1;
            motor_c_in2 = 1'b0;

            motor_d_in1 = 1'b1;
            motor_d_in2 = 1'b0;

        end


        //====================================================
        // 所有STOP状态
        //====================================================

        default:
        begin

            motor_a_in1 = 1'b0;
            motor_a_in2 = 1'b0;

            motor_b_in1 = 1'b0;
            motor_b_in2 = 1'b0;

            motor_c_in1 = 1'b0;
            motor_c_in2 = 1'b0;

            motor_d_in1 = 1'b0;
            motor_d_in2 = 1'b0;

        end


    endcase

end


endmodule