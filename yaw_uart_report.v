//////////////////////////////////////////////////////////////////////////////////
// File Name   : yaw_uart_report.v
//
// Description :
//   Yaw角UART显示。
//   yaw_cdeg单位为0.01°。
//
// Example:
//
//   CAL=1 YAW=+090.25
//
//   表示Yaw = +90.25°
//
//////////////////////////////////////////////////////////////////////////////////

module yaw_uart_report(
    input wire clk,
    input wire reset_n,

    input wire calib_done,

    input wire signed [31:0] yaw_cdeg,

    output wire uart_tx
);


    //================================================
    // UART
    //================================================

    reg       tx_send;
    reg [7:0] tx_data;

    wire tx_busy;
    wire tx_done;


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
    // 每200ms锁存一次Yaw
    //================================================

    reg signed [31:0] yaw_latch;

    reg calib_latch;


    wire [31:0] yaw_abs;


    assign yaw_abs =
        yaw_latch[31]
        ?
        (~yaw_latch + 1'b1)
        :
        yaw_latch;


    //================================================
    // Decimal digits
    //
    // yaw_abs范围0~18000
    //================================================

    wire [3:0] digit_hundred;
    wire [3:0] digit_ten;
    wire [3:0] digit_one;
    wire [3:0] digit_tenth;
    wire [3:0] digit_hundredth;


    assign digit_hundred =
        (yaw_abs / 10000) % 10;

    assign digit_ten =
        (yaw_abs / 1000) % 10;

    assign digit_one =
        (yaw_abs / 100) % 10;

    assign digit_tenth =
        (yaw_abs / 10) % 10;

    assign digit_hundredth =
        yaw_abs % 10;


    reg [23:0] report_cnt;

    reg [5:0] char_index;

    reg sending;
    reg waiting_tx;


    //================================================
    // CAL=1 YAW=+000.00
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        case(char_index)

            6'd0  : tx_data = "C";
            6'd1  : tx_data = "A";
            6'd2  : tx_data = "L";
            6'd3  : tx_data = "=";

            6'd4  :
                tx_data =
                    calib_latch ? "1" : "0";

            6'd5  : tx_data = " ";

            6'd6  : tx_data = "Y";
            6'd7  : tx_data = "A";
            6'd8  : tx_data = "W";
            6'd9  : tx_data = "=";


            6'd10 :
                tx_data =
                    yaw_latch[31] ? "-" : "+";


            6'd11 :
                tx_data =
                    8'h30 + digit_hundred;

            6'd12 :
                tx_data =
                    8'h30 + digit_ten;

            6'd13 :
                tx_data =
                    8'h30 + digit_one;


            6'd14 :
                tx_data = ".";


            6'd15 :
                tx_data =
                    8'h30 + digit_tenth;

            6'd16 :
                tx_data =
                    8'h30 + digit_hundredth;


            6'd17 :
                tx_data = 8'h0D;

            6'd18 :
                tx_data = 8'h0A;


            default:
                tx_data = 8'h20;

        endcase

    end


    //================================================
    // UART发送控制
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            report_cnt <= 24'd0;

            yaw_latch <= 32'sd0;

            calib_latch <= 1'b0;

            char_index <= 6'd0;

            sending <= 1'b0;

            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

        end


        else begin

            tx_send <= 1'b0;


            if(!sending) begin

                // 200ms
                if(report_cnt >= 24'd9999999) begin

                    report_cnt <= 24'd0;

                    yaw_latch <= yaw_cdeg;

                    calib_latch <= calib_done;

                    char_index <= 6'd0;

                    sending <= 1'b1;

                    waiting_tx <= 1'b0;

                end

                else begin

                    report_cnt <=
                        report_cnt + 1'b1;

                end

            end


            else begin

                if(!waiting_tx && !tx_busy) begin

                    tx_send <= 1'b1;

                    waiting_tx <= 1'b1;

                end


                else if(waiting_tx && tx_done) begin

                    waiting_tx <= 1'b0;


                    if(char_index >= 6'd18) begin

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