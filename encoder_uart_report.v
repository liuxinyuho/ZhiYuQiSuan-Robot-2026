module encoder_uart_report(
    input  wire clk,
    input  wire reset_n,

    input  wire signed [31:0] position,
    input  wire signed [31:0] speed_count,

    output wire uart_tx
);

    //================================================
    // UART TX接口
    //================================================
    reg        tx_send;

    wire       tx_done;
    wire       tx_busy;

    wire [7:0] tx_data;


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
    // 每100ms发送一次
    //
    // 50MHz × 0.1s = 5,000,000
    //================================================
    reg [22:0] report_cnt;


    //================================================
    // 锁存本次要发送的数据
    // 防止发送过程中数值变化
    //================================================
    reg signed [31:0] position_latch;
    reg signed [31:0] speed_latch;


    //================================================
    // 求绝对值
    //================================================
    wire [31:0] position_abs;
    wire [31:0] speed_abs;

    assign position_abs =
        position_latch[31]
        ? (~position_latch + 1'b1)
        : position_latch;

    assign speed_abs =
        speed_latch[31]
        ? (~speed_latch + 1'b1)
        : speed_latch;


    //================================================
    // HEX → ASCII
    //
    // 0x0 → "0"
    // ...
    // 0xA → "A"
    //================================================
    function [7:0] hex_to_ascii;

        input [3:0] hex;

        begin

            if(hex < 4'd10)
                hex_to_ascii = 8'h30 + hex;
            else
                hex_to_ascii = 8'h41 + (hex - 4'd10);

        end

    endfunction


    //================================================
    // 当前字符序号
    //================================================
    reg [5:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // 根据char_index生成当前字符
    //
    // 最终格式：
    //
    // A_POS=+00000000 A_SPD=+00000000\r\n
    //================================================
    reg [7:0] current_char;

    assign tx_data = current_char;


    always @(*) begin

        current_char = 8'h20;


        case(char_index)

            // A_POS=
            6'd0  : current_char = "A";
            6'd1  : current_char = "_";
            6'd2  : current_char = "P";
            6'd3  : current_char = "O";
            6'd4  : current_char = "S";
            6'd5  : current_char = "=";


            // position符号
            6'd6  :
                current_char =
                position_latch[31] ? "-" : "+";


            // position 8位HEX
            6'd7  : current_char =
                hex_to_ascii(position_abs[31:28]);

            6'd8  : current_char =
                hex_to_ascii(position_abs[27:24]);

            6'd9  : current_char =
                hex_to_ascii(position_abs[23:20]);

            6'd10 : current_char =
                hex_to_ascii(position_abs[19:16]);

            6'd11 : current_char =
                hex_to_ascii(position_abs[15:12]);

            6'd12 : current_char =
                hex_to_ascii(position_abs[11:8]);

            6'd13 : current_char =
                hex_to_ascii(position_abs[7:4]);

            6'd14 : current_char =
                hex_to_ascii(position_abs[3:0]);


            // 空格
            6'd15 : current_char = " ";


            // A_SPD=
            6'd16 : current_char = "A";
            6'd17 : current_char = "_";
            6'd18 : current_char = "S";
            6'd19 : current_char = "P";
            6'd20 : current_char = "D";
            6'd21 : current_char = "=";


            // speed符号
            6'd22 :
                current_char =
                speed_latch[31] ? "-" : "+";


            // speed 8位HEX
            6'd23 : current_char =
                hex_to_ascii(speed_abs[31:28]);

            6'd24 : current_char =
                hex_to_ascii(speed_abs[27:24]);

            6'd25 : current_char =
                hex_to_ascii(speed_abs[23:20]);

            6'd26 : current_char =
                hex_to_ascii(speed_abs[19:16]);

            6'd27 : current_char =
                hex_to_ascii(speed_abs[15:12]);

            6'd28 : current_char =
                hex_to_ascii(speed_abs[11:8]);

            6'd29 : current_char =
                hex_to_ascii(speed_abs[7:4]);

            6'd30 : current_char =
                hex_to_ascii(speed_abs[3:0]);


            // 回车
            6'd31 : current_char = 8'h0D;

            // 换行
            6'd32 : current_char = 8'h0A;


            default:
                current_char = 8'h20;

        endcase

    end


    //================================================
    // 发送状态机
    //================================================
    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            report_cnt <= 23'd0;

            position_latch <= 32'sd0;
            speed_latch    <= 32'sd0;

            char_index <= 6'd0;

            sending    <= 1'b0;
            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

        end

        else begin

            // send_en默认只保持一个clk
            tx_send <= 1'b0;


            //================================================
            // 没有正在发送
            //================================================
            if(!sending) begin

                if(report_cnt >= 23'd4999999) begin

                    report_cnt <= 23'd0;


                    // 锁存一次编码器数据
                    position_latch <= position;
                    speed_latch    <= speed_count;


                    // 从字符0开始发送
                    char_index <= 6'd0;

                    sending    <= 1'b1;
                    waiting_tx <= 1'b0;

                end

                else begin

                    report_cnt <= report_cnt + 1'b1;

                end

            end


            //================================================
            // 正在发送整行数据
            //================================================
            else begin

                // UART空闲，可以发送当前字符
                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                // 当前字符已经发完
                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    // 最后一个字符
                    if(char_index >= 6'd32) begin

                        sending <= 1'b0;

                    end

                    else begin

                        char_index <= char_index + 1'b1;

                    end

                end

            end

        end

    end


endmodule