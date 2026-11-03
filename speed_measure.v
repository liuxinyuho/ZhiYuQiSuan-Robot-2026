// speed_measure.v
// Description:
//   电机编码器测速模块。
//   每 10ms 计算一次编码器位置差，
//   输出实际速度 speed_count，单位 count/10ms。
module speed_measure(
    input  wire clk,
    input  wire reset_n,

    input  wire signed [31:0] position,

    output reg  signed [31:0] speed_count,

    output reg sample_tick
);

    //================================================
    // 50MHz × 10ms = 500000
    //
    // 19bit:
    // 2^19 = 524288
    // 足够计数到499999
    //================================================

    reg [18:0] timer_cnt;


    // 上一次编码器位置
    reg signed [31:0] position_old;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            timer_cnt    <= 19'd0;

            position_old <= 32'sd0;

            speed_count  <= 32'sd0;

            sample_tick  <= 1'b0;

        end

        else begin

            // 默认低电平
            sample_tick <= 1'b0;


            //========================================
            // 10ms到
            //========================================

            if(timer_cnt >= 19'd499999) begin

                timer_cnt <= 19'd0;


                //====================================
                // 10ms内增加的编码器计数
                //====================================

                speed_count <=
                    position - position_old;


                // 保存当前值
                position_old <= position;


                // 产生一个时钟周期脉冲
                sample_tick <= 1'b1;

            end

            else begin

                timer_cnt <= timer_cnt + 1'b1;

            end

        end

    end


endmodule