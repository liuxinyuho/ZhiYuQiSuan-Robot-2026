// pwm_gen.v
// Description:
//   PWM 发生器。
//   基于 50MHz 系统时钟产生约 20kHz PWM，
//   duty 范围为 0~2499。
module pwm_gen(
    input  wire       clk,
    input  wire       reset_n,
    input  wire [11:0] duty,      //占空比
    output reg        pwm_out
);

    reg [11:0] cnt;

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)
            cnt <= 12'd0;
        else if(cnt >= 12'd2499)
            cnt <= 12'd0;
        else
            cnt <= cnt + 1'b1;
    end

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)
            pwm_out <= 1'b0;
        else if(duty == 0)
            pwm_out <= 1'b0;
        else if(cnt < duty)
            pwm_out <= 1'b1;
        else
            pwm_out <= 1'b0;
    end

endmodule