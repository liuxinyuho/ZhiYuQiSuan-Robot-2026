//////////////////////////////////////////////////////////////////////////////////
// File Name : servo_pwm.v
// Project   : FPGA Mecanum Robot
//
// Description:
//   SG90舵机PWM驱动
//
// FPGA Clock:
//   50MHz
//
// Servo:
//   PWM周期 = 20ms = 50Hz
//
//   pos = 0 : 左侧
//   pos = 1 : 中间
//   pos = 2 : 右侧
//////////////////////////////////////////////////////////////////////////////////

module servo_pwm(

    input wire clk,
    input wire reset_n,

    // 0 = 左
    // 1 = 中
    // 2 = 右
    input wire [1:0] position,

    output reg servo_pwm

);


    //================================================
    // 50MHz
    //
    // 20ms:
    // 50,000,000 * 0.020
    // = 1,000,000
    //================================================

    localparam integer PERIOD_COUNT =
        1_000_000;


    //================================================
    // 三个舵机位置
    //
    // 1.0ms = 50,000
    // 1.5ms = 75,000
    // 2.0ms = 100,000
    //================================================

// E1：右侧，增大转动角度
localparam integer PULSE_LEFT = 68_000;

// E2：正前方，已经校准好
localparam integer PULSE_CENTER = 82_000;

// E3：左侧，保持不变
localparam integer PULSE_RIGHT = 98_000;


    //================================================
    // 周期计数器
    //================================================

    reg [19:0] pwm_cnt;


    //================================================
    // 当前脉宽
    //================================================

    reg [19:0] pulse_width;


    //================================================
    // 根据position选择脉宽
    //================================================

    always @(*) begin

        case(position)

            2'd0:
                pulse_width = PULSE_LEFT;

            2'd1:
                pulse_width = PULSE_CENTER;

            2'd2:
                pulse_width = PULSE_RIGHT;

            default:
                pulse_width = PULSE_CENTER;

        endcase

    end


    //================================================
    // 20ms周期计数
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            pwm_cnt <= 20'd0;

        end

        else begin

            if(
                pwm_cnt
                >=
                PERIOD_COUNT - 1
            ) begin

                pwm_cnt <= 20'd0;

            end

            else begin

                pwm_cnt <=
                    pwm_cnt + 1'b1;

            end

        end

    end


    //================================================
    // PWM输出
    //================================================

    always @(*) begin

        if(
            pwm_cnt
            <
            pulse_width
        ) begin

            servo_pwm = 1'b1;

        end

        else begin

            servo_pwm = 1'b0;

        end

    end


endmodule