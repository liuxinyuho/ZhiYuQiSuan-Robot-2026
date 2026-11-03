// uart_byte_tx.v
// Description:
//   UART 单字节发送模块。
//   当前使用 115200bps、8N1，
//   用于向 PC 串口助手返回调试数据。
module uart_byte_tx(
    input  wire       clk,
    input  wire       reset_n,

    input  wire [7:0] data_byte,
    input  wire       send_en,

    output reg        uart_tx,
    output reg        tx_done,
    output reg        tx_busy
);

    //================================================
    // 50MHz / 115200 ≈ 434
    //================================================
    localparam CLKS_PER_BIT = 16'd434;

    reg [15:0] clk_cnt;
    reg [3:0]  bit_cnt;

    //================================================
    // UART帧：
    //
    // bit0     起始位 0
    // bit1~8   数据位 D0~D7
    // bit9     停止位 1
    //================================================
    reg [9:0] tx_shift;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            uart_tx <= 1'b1;

            tx_done <= 1'b0;
            tx_busy <= 1'b0;

            clk_cnt <= 16'd0;
            bit_cnt <= 4'd0;

            tx_shift <= 10'b1111111111;

        end

        else begin

            tx_done <= 1'b0;


            //================================================
            // 空闲状态
            //================================================
            if(!tx_busy) begin

                uart_tx <= 1'b1;

                clk_cnt <= 16'd0;
                bit_cnt <= 4'd0;


                // 收到发送请求
                if(send_en) begin

                    // 停止位 + 8位数据 + 起始位
                    tx_shift <= {
                        1'b1,
                        data_byte,
                        1'b0
                    };

                    tx_busy <= 1'b1;

                    // 先发送起始位
                    uart_tx <= 1'b0;

                end

            end


            //================================================
            // 正在发送
            //================================================
            else begin

                if(clk_cnt >= CLKS_PER_BIT - 1'b1) begin

                    clk_cnt <= 16'd0;


                    // 已经发送完停止位
                    if(bit_cnt >= 4'd9) begin

                        tx_busy <= 1'b0;
                        tx_done <= 1'b1;

                        uart_tx <= 1'b1;

                    end

                    else begin

                        bit_cnt <= bit_cnt + 1'b1;

                        // 发送下一位
                        uart_tx <= tx_shift[bit_cnt + 1'b1];

                    end

                end

                else begin

                    clk_cnt <= clk_cnt + 1'b1;

                end

            end

        end

    end

endmodule