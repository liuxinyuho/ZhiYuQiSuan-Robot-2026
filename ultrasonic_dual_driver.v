
`timescale 1ns / 1ps

//============================================================
// 文件：ultrasonic_dual_driver.v
//
// ACG720 / GW5AT / 50MHz
//
// 功能：
// 1. 前方HC-SR04测距
// 2. 后方HC-SR04测距
// 3. 前后传感器交替触发，避免同时发射
// 4. 输出距离、数据有效脉冲、超时状态
// 5. 距离单位：mm
//
// 注意：
// HC-SR04 ECHO可能为5V，必须转换至3.3V
//
// 前方原来的SG90舵机扫描功能保持不变
//============================================================

module ultrasonic_dual_driver (

    input wire clk,
    input wire reset_n,

    //==============================
    // 前方超声波
    //==============================
    output reg front_trig,
    input wire front_echo,

    output reg [15:0] front_distance_mm,
    output reg front_distance_valid,

    output wire front_measure_busy,
    output reg front_echo_timeout,

    //==============================
    // 后方超声波
    //==============================
    output reg rear_trig,
    input wire rear_echo,

    output reg [15:0] rear_distance_mm,
    output reg rear_distance_valid,

    output wire rear_measure_busy,
    output reg rear_echo_timeout

);

//============================================================
// 一、时间参数
//============================================================

// 50MHz，20ns一个时钟

// 10us触发脉冲
localparam [21:0] TRIG_CYCLES =
    22'd500;

// 30ms回波等待超时
localparam [21:0] ECHO_TIMEOUT_CYCLES =
    22'd1500000;

// 两次声波发射之间，测量结束后再间隔30ms
localparam [21:0] QUIET_GAP_CYCLES =
    22'd1500000;


//============================================================
// 二、输入同步
//============================================================

reg front_echo_s1;
reg front_echo_s2;
reg front_echo_d;

reg rear_echo_s1;
reg rear_echo_s2;
reg rear_echo_d;

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        front_echo_s1 <= 1'b0;
        front_echo_s2 <= 1'b0;
        front_echo_d  <= 1'b0;

        rear_echo_s1 <= 1'b0;
        rear_echo_s2 <= 1'b0;
        rear_echo_d  <= 1'b0;

    end

    else begin

        front_echo_s1 <= front_echo;
        front_echo_s2 <= front_echo_s1;
        front_echo_d  <= front_echo_s2;

        rear_echo_s1 <= rear_echo;
        rear_echo_s2 <= rear_echo_s1;
        rear_echo_d  <= rear_echo_s2;

    end

end


//============================================================
// 三、前后回波边沿检测
//============================================================

wire front_rise;
wire front_fall;
wire rear_rise;
wire rear_fall;

assign front_rise =
    front_echo_s2 && !front_echo_d;

assign front_fall =
    !front_echo_s2 && front_echo_d;

assign rear_rise =
    rear_echo_s2 && !rear_echo_d;

assign rear_fall =
    !rear_echo_s2 && rear_echo_d;


//============================================================
// 四、传感器选择
//
// select_rear=0：前方
// select_rear=1：后方
//============================================================

reg select_rear;

wire selected_rise;
wire selected_fall;

assign selected_rise =
    select_rear ? rear_rise : front_rise;

assign selected_fall =
    select_rear ? rear_fall : front_fall;


//============================================================
// 五、主状态机
//============================================================

localparam [3:0]

    ST_GAP        = 4'd0,
    ST_TRIGGER    = 4'd1,
    ST_WAIT_RISE  = 4'd2,
    ST_ECHO_HIGH  = 4'd3,

    ST_MULT_A     = 4'd4,
    ST_MULT_SUM   = 4'd5,
    ST_RESULT     = 4'd6;

reg [3:0] state;


//============================================================
// 六、计数器
//============================================================

reg [21:0] gap_cnt;
reg [21:0] trig_cnt;
reg [21:0] wait_cnt;
reg [21:0] echo_cnt;

reg [21:0] echo_width_latched;


//============================================================
// 七、距离计算
//
// HC-SR04距离：
//
// 距离(mm)≈回波高电平时钟数×225/65536
//
// 225=128+64+32+1
//
// 采用三级流水线，避免组合除法。
//============================================================

wire [31:0] echo_width_ext;

assign echo_width_ext =
    {10'd0, echo_width_latched};

reg [31:0] mult_a;
reg [31:0] mult_b;
reg [31:0] mult_sum;

wire [15:0] calculated_mm;

assign calculated_mm =
    mult_sum[31:16];


//============================================================
// 八、忙标志
//============================================================

assign front_measure_busy =
    (state != ST_GAP) && !select_rear;

assign rear_measure_busy =
    (state != ST_GAP) && select_rear;


//============================================================
// 九、主控制
//============================================================

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        state <= ST_GAP;

        select_rear <= 1'b0;

        front_trig <= 1'b0;
        rear_trig <= 1'b0;

        front_distance_mm <= 16'd0;
        rear_distance_mm <= 16'd0;

        front_distance_valid <= 1'b0;
        rear_distance_valid <= 1'b0;

        // 上电后，尚未取得有效测距
        // 先按不安全处理
        front_echo_timeout <= 1'b1;
        rear_echo_timeout <= 1'b1;

        gap_cnt <= 22'd0;
        trig_cnt <= 22'd0;
        wait_cnt <= 22'd0;
        echo_cnt <= 22'd0;

        echo_width_latched <= 22'd0;

        mult_a <= 32'd0;
        mult_b <= 32'd0;
        mult_sum <= 32'd0;

    end

    else begin

        // 有效脉冲只持续一个时钟
        front_distance_valid <= 1'b0;
        rear_distance_valid <= 1'b0;

        case (state)

            //================================================
            // 1. 两次测量之间的安静间隔
            //================================================

            ST_GAP: begin

                front_trig <= 1'b0;
                rear_trig <= 1'b0;

                if (gap_cnt >= QUIET_GAP_CYCLES-1'b1) begin

                    gap_cnt <= 22'd0;
                    trig_cnt <= 22'd0;
                    wait_cnt <= 22'd0;
                    echo_cnt <= 22'd0;

                    // 只触发当前选中的一个传感器
                    if (select_rear)
                        rear_trig <= 1'b1;
                    else
                        front_trig <= 1'b1;

                    state <= ST_TRIGGER;

                end

                else begin

                    gap_cnt <= gap_cnt + 1'b1;

                end

            end


            //================================================
            // 2. 产生10us触发脉冲
            //================================================

            ST_TRIGGER: begin

                if (trig_cnt >= TRIG_CYCLES-1'b1) begin

                    trig_cnt <= 22'd0;
                    wait_cnt <= 22'd0;

                    front_trig <= 1'b0;
                    rear_trig <= 1'b0;

                    state <= ST_WAIT_RISE;

                end

                else begin

                    trig_cnt <= trig_cnt + 1'b1;

                end

            end


            //================================================
            // 3. 等待回波上升沿
            //================================================

            ST_WAIT_RISE: begin

                if (selected_rise) begin

                    echo_cnt <= 22'd0;
                    state <= ST_ECHO_HIGH;

                end

                else if (
                    wait_cnt >= ECHO_TIMEOUT_CYCLES-1'b1
                ) begin

                    // 没有收到回波，数据失效
                    if (select_rear)
                        rear_echo_timeout <= 1'b1;
                    else
                        front_echo_timeout <= 1'b1;

                    // 切换到另一只传感器
                    select_rear <= ~select_rear;

                    gap_cnt <= 22'd0;
                    state <= ST_GAP;

                end

                else begin

                    wait_cnt <= wait_cnt + 1'b1;

                end

            end


            //================================================
            // 4. 测量回波高电平宽度
            //================================================

            ST_ECHO_HIGH: begin

                if (selected_fall) begin

                    echo_width_latched <= echo_cnt;

                    state <= ST_MULT_A;

                end

                else if (
                    echo_cnt >= ECHO_TIMEOUT_CYCLES-1'b1
                ) begin

                    if (select_rear)
                        rear_echo_timeout <= 1'b1;
                    else
                        front_echo_timeout <= 1'b1;

                    select_rear <= ~select_rear;

                    gap_cnt <= 22'd0;
                    state <= ST_GAP;

                end

                else begin

                    echo_cnt <= echo_cnt + 1'b1;

                end

            end


            //================================================
            // 5. 距离转换第一级
            //================================================

            ST_MULT_A: begin

                // 128 + 64
                mult_a <=
                    (echo_width_ext << 7)
                    +
                    (echo_width_ext << 6);

                // 32 + 1
                mult_b <=
                    (echo_width_ext << 5)
                    +
                    echo_width_ext;

                state <= ST_MULT_SUM;

            end


            //================================================
            // 6. 距离转换第二级
            //================================================

            ST_MULT_SUM: begin

                mult_sum <= mult_a + mult_b;

                state <= ST_RESULT;

            end


            //================================================
            // 7. 输出测距结果
            //================================================

            ST_RESULT: begin

                // HC-SR04实测应保持在合理范围
                // 这里使用30~4000mm
                if (
                    calculated_mm >= 16'd30
                    &&
                    calculated_mm <= 16'd4000
                ) begin

                    if (select_rear) begin

                        rear_distance_mm <= calculated_mm;

                        rear_distance_valid <= 1'b1;

                        rear_echo_timeout <= 1'b0;

                    end

                    else begin

                        front_distance_mm <= calculated_mm;

                        front_distance_valid <= 1'b1;

                        front_echo_timeout <= 1'b0;

                    end

                end

                else begin

                    // 过近、过远或异常距离
                    // 不当作有效安全数据
                    if (select_rear)
                        rear_echo_timeout <= 1'b1;
                    else
                        front_echo_timeout <= 1'b1;

                end

                // 下一次测另一只传感器
                select_rear <= ~select_rear;

                gap_cnt <= 22'd0;

                state <= ST_GAP;

            end


            default: begin

                front_trig <= 1'b0;
                rear_trig <= 1'b0;

                front_echo_timeout <= 1'b1;
                rear_echo_timeout <= 1'b1;

                state <= ST_GAP;

            end

        endcase

    end

end

endmodule
