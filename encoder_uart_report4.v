// encoder_uart_report4.v
// Description:
//   四轮编码器 UART 调试输出模块。
//   周期性将 A/B/C/D 四轮速度转换为 ASCII/HEX 字符，
//   通过 uart_byte_tx 发送到 PC。
module encoder_uart_report4(
    input wire clk,
    input wire reset_n,

    input wire signed [31:0] speed_a,
    input wire signed [31:0] speed_b,
    input wire signed [31:0] speed_c,
    input wire signed [31:0] speed_d,

    output wire uart_tx
);

    //================================================
    // UART发送接口
    //================================================

    reg        tx_send;
    reg [7:0]  tx_data;

    wire       tx_done;
    wire       tx_busy;


    uart_byte_tx u_uart_byte_tx(
        .clk       (clk),
        .reset_n   (reset_n),

        .data_byte (tx_data),
        .send_en   (tx_send),

        .uart_tx   (uart_tx),
        .tx_done   (tx_done),
        .tx_busy   (tx_busy)
    );


    //================================================
    // 每100ms打印一次
    //
    // 50MHz × 100ms = 5,000,000
    //================================================

    reg [22:0] report_cnt;


    //================================================
    // 锁存四路速度
    //================================================

    reg signed [31:0] speed_a_latch;
    reg signed [31:0] speed_b_latch;
    reg signed [31:0] speed_c_latch;
    reg signed [31:0] speed_d_latch;


    //================================================
    // 绝对值
    //================================================

    wire [31:0] abs_a;
    wire [31:0] abs_b;
    wire [31:0] abs_c;
    wire [31:0] abs_d;


    assign abs_a =
        speed_a_latch[31]
        ? (~speed_a_latch + 1'b1)
        : speed_a_latch;


    assign abs_b =
        speed_b_latch[31]
        ? (~speed_b_latch + 1'b1)
        : speed_b_latch;


    assign abs_c =
        speed_c_latch[31]
        ? (~speed_c_latch + 1'b1)
        : speed_c_latch;


    assign abs_d =
        speed_d_latch[31]
        ? (~speed_d_latch + 1'b1)
        : speed_d_latch;


    //================================================
    // HEX → ASCII
    //================================================

    function [7:0] hex_ascii;

        input [3:0] hex;

        begin

            if(hex <= 4'd9)
                hex_ascii = 8'h30 + hex;
            else
                hex_ascii = 8'h41 + hex - 4'd10;

        end

    endfunction


    //================================================
    // 字符发送序号
    //
    // 输出：
    //
    // A=+00000000 B=+00000000
    // C=+00000000 D=+00000000
    //
    //================================================

    reg [5:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 当前字符
    //================================================

    always @(*) begin

        tx_data = 8'h20;

        case(char_index)

            //========================================
            // A=+00000000
            //========================================

            6'd0:
                tx_data = "A";

            6'd1:
                tx_data = "=";

            6'd2:
                tx_data =
                    speed_a_latch[31] ? "-" : "+";

            6'd3:
                tx_data = hex_ascii(abs_a[31:28]);

            6'd4:
                tx_data = hex_ascii(abs_a[27:24]);

            6'd5:
                tx_data = hex_ascii(abs_a[23:20]);

            6'd6:
                tx_data = hex_ascii(abs_a[19:16]);

            6'd7:
                tx_data = hex_ascii(abs_a[15:12]);

            6'd8:
                tx_data = hex_ascii(abs_a[11:8]);

            6'd9:
                tx_data = hex_ascii(abs_a[7:4]);

            6'd10:
                tx_data = hex_ascii(abs_a[3:0]);

            6'd11:
                tx_data = " ";


            //========================================
            // B=+00000000
            //========================================

            6'd12:
                tx_data = "B";

            6'd13:
                tx_data = "=";

            6'd14:
                tx_data =
                    speed_b_latch[31] ? "-" : "+";

            6'd15:
                tx_data = hex_ascii(abs_b[31:28]);

            6'd16:
                tx_data = hex_ascii(abs_b[27:24]);

            6'd17:
                tx_data = hex_ascii(abs_b[23:20]);

            6'd18:
                tx_data = hex_ascii(abs_b[19:16]);

            6'd19:
                tx_data = hex_ascii(abs_b[15:12]);

            6'd20:
                tx_data = hex_ascii(abs_b[11:8]);

            6'd21:
                tx_data = hex_ascii(abs_b[7:4]);

            6'd22:
                tx_data = hex_ascii(abs_b[3:0]);


            // 换行
            6'd23:
                tx_data = 8'h0D;

            6'd24:
                tx_data = 8'h0A;


            //========================================
            // C=+00000000
            //========================================

            6'd25:
                tx_data = "C";

            6'd26:
                tx_data = "=";

            6'd27:
                tx_data =
                    speed_c_latch[31] ? "-" : "+";

            6'd28:
                tx_data = hex_ascii(abs_c[31:28]);

            6'd29:
                tx_data = hex_ascii(abs_c[27:24]);

            6'd30:
                tx_data = hex_ascii(abs_c[23:20]);

            6'd31:
                tx_data = hex_ascii(abs_c[19:16]);

            6'd32:
                tx_data = hex_ascii(abs_c[15:12]);

            6'd33:
                tx_data = hex_ascii(abs_c[11:8]);

            6'd34:
                tx_data = hex_ascii(abs_c[7:4]);

            6'd35:
                tx_data = hex_ascii(abs_c[3:0]);

            6'd36:
                tx_data = " ";


            //========================================
            // D=+00000000
            //========================================

            6'd37:
                tx_data = "D";

            6'd38:
                tx_data = "=";

            6'd39:
                tx_data =
                    speed_d_latch[31] ? "-" : "+";

            6'd40:
                tx_data = hex_ascii(abs_d[31:28]);

            6'd41:
                tx_data = hex_ascii(abs_d[27:24]);

            6'd42:
                tx_data = hex_ascii(abs_d[23:20]);

            6'd43:
                tx_data = hex_ascii(abs_d[19:16]);

            6'd44:
                tx_data = hex_ascii(abs_d[15:12]);

            6'd45:
                tx_data = hex_ascii(abs_d[11:8]);

            6'd46:
                tx_data = hex_ascii(abs_d[7:4]);

            6'd47:
                tx_data = hex_ascii(abs_d[3:0]);


            // 回车换行
            6'd48:
                tx_data = 8'h0D;

            6'd49:
                tx_data = 8'h0A;


            default:
                tx_data = 8'h20;

        endcase

    end


    //================================================
    // 发送控制状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            report_cnt <= 23'd0;

            speed_a_latch <= 32'sd0;
            speed_b_latch <= 32'sd0;
            speed_c_latch <= 32'sd0;
            speed_d_latch <= 32'sd0;

            char_index <= 6'd0;

            sending    <= 1'b0;
            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

        end

        else begin

            tx_send <= 1'b0;


            //========================================
            // 等待下一次报告
            //========================================

            if(!sending) begin

                if(report_cnt >= 23'd4999999) begin

                    report_cnt <= 23'd0;


                    // 锁存四轮速度
                    speed_a_latch <= speed_a;
                    speed_b_latch <= speed_b;
                    speed_c_latch <= speed_c;
                    speed_d_latch <= speed_d;


                    char_index <= 6'd0;

                    sending <= 1'b1;

                    waiting_tx <= 1'b0;

                end

                else begin

                    report_cnt <=
                        report_cnt + 1'b1;

                end

            end


            //========================================
            // 正在打印
            //========================================

            else begin

                // UART空闲
                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                // 当前字符发完
                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    if(char_index >= 6'd49) begin

                        sending <= 1'b0;

                    end

                    else begin

                        char_index <=
                            char_index + 1'b1;

                    end

                end

            end

        end

    end

endmodule