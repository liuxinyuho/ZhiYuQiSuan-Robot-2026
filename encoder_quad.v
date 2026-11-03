// encoder_quad.v
// Description:
//   AB 相正交编码器解码模块。
//   对异步编码器信号进行同步并判断旋转方向，
//   输出有符号累计编码器位置 position。
module encoder_quad #(
    parameter REVERSE = 1'b0
)(
    input  wire clk,
    input  wire reset_n,

    input  wire enc_a,
    input  wire enc_b,

    output reg signed [31:0] position
);

    //================================================
    // 编码器输入同步
    // 编码器信号相对于50MHz时钟属于异步输入
    // 需要两级寄存器降低亚稳态风险
    //================================================

    reg enc_a_ff1;
    reg enc_a_ff2;

    reg enc_b_ff1;
    reg enc_b_ff2;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            enc_a_ff1 <= 1'b0;
            enc_a_ff2 <= 1'b0;

            enc_b_ff1 <= 1'b0;
            enc_b_ff2 <= 1'b0;

        end
        else begin

            enc_a_ff1 <= enc_a;
            enc_a_ff2 <= enc_a_ff1;

            enc_b_ff1 <= enc_b;
            enc_b_ff2 <= enc_b_ff1;

        end

    end


    //================================================
    // 当前AB状态
    //================================================

    wire [1:0] ab_now;

    assign ab_now = {
        enc_a_ff2,
        enc_b_ff2
    };


    //================================================
    // 保存上一次状态
    //================================================

    reg [1:0] ab_prev;


    //================================================
    // 单次变化量
    //================================================

    reg signed [1:0] step;


    always @(*) begin

        step = 2'sd0;

        case({ab_prev, ab_now})

            //----------------------------------------
            // 一个方向
            //
            // 00 → 01
            // 01 → 11
            // 11 → 10
            // 10 → 00
            //----------------------------------------

            4'b0001,
            4'b0111,
            4'b1110,
            4'b1000:
                step = 2'sd1;


            //----------------------------------------
            // 相反方向
            //
            // 00 → 10
            // 10 → 11
            // 11 → 01
            // 01 → 00
            //----------------------------------------

            4'b0010,
            4'b1011,
            4'b1101,
            4'b0100:
                step = -2'sd1;


            //----------------------------------------
            // 没变化或者非法跳变
            //----------------------------------------

            default:
                step = 2'sd0;

        endcase

    end


    //================================================
    // 累计编码器位置
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            ab_prev  <= 2'b00;
            position <= 32'sd0;

        end

        else begin

            // 保存当前状态
            ab_prev <= ab_now;


            // 编码器方向软件翻转
            if(REVERSE == 1'b0)
                position <= position + step;
            else
                position <= position - step;

        end

    end


endmodule