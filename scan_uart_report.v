//////////////////////////////////////////////////////////////////////////////////
// File Name : scan_uart_report.v
//
// Output example:
//
// L=00820 C=00310 R=01250 mm
//
// Clock : 50MHz
// UART  : 115200 8N1
//////////////////////////////////////////////////////////////////////////////////

module scan_uart_report #(

    parameter integer CLK_FREQ  = 50000000,
    parameter integer BAUD_RATE = 115200

)(

    input wire clk,
    input wire reset_n,

    // Scan finished pulse
    input wire scan_done,

    // Scan results
    input wire [15:0] left_distance_mm,
    input wire [15:0] center_distance_mm,
    input wire [15:0] right_distance_mm,

    output reg uart_tx

);


    //================================================
    // UART parameter
    //================================================

    localparam integer CLKS_PER_BIT =
        CLK_FREQ / BAUD_RATE;


    //================================================
    // Data latch
    //================================================

    reg [15:0] left_latched;
    reg [15:0] center_latched;
    reg [15:0] right_latched;


    //================================================
    // Message control
    //================================================

    reg sending;
    reg waiting_tx_done;

    reg [5:0] tx_index;

    reg [7:0] tx_data;
    reg       tx_start;


    //================================================
    // UART engine
    //================================================

    reg uart_busy;
    reg tx_done;

    reg [15:0] baud_counter;
    reg [3:0]  bit_index;

    reg [7:0] tx_shift;


    //================================================
    // Convert 16-bit number to decimal character
    // 00000 ~ 65535
    //================================================

    function [7:0] decimal_digit5;

        input [15:0] value;
        input [2:0] pos;

        reg [15:0] digit;

        begin

            case(pos)

                3'd0:
                    digit = (value / 16'd10000) % 10;

                3'd1:
                    digit = (value / 16'd1000) % 10;

                3'd2:
                    digit = (value / 16'd100) % 10;

                3'd3:
                    digit = (value / 16'd10) % 10;

                3'd4:
                    digit = value % 10;

                default:
                    digit = 0;

            endcase


            decimal_digit5 =
                8'h30 + digit[7:0];

        end

    endfunction


    //================================================
    // Message
    //
    // L=00820 C=00310 R=01250 mm\r\n
    //================================================

    reg [7:0] msg_char;

    always @(*) begin

        case(tx_index)

            // L=xxxxx
            6'd0:
                msg_char = "L";

            6'd1:
                msg_char = "=";

            6'd2:
                msg_char = decimal_digit5(left_latched, 3'd0);

            6'd3:
                msg_char = decimal_digit5(left_latched, 3'd1);

            6'd4:
                msg_char = decimal_digit5(left_latched, 3'd2);

            6'd5:
                msg_char = decimal_digit5(left_latched, 3'd3);

            6'd6:
                msg_char = decimal_digit5(left_latched, 3'd4);


            6'd7:
                msg_char = " ";


            // C=xxxxx
            6'd8:
                msg_char = "C";

            6'd9:
                msg_char = "=";

            6'd10:
                msg_char = decimal_digit5(center_latched, 3'd0);

            6'd11:
                msg_char = decimal_digit5(center_latched, 3'd1);

            6'd12:
                msg_char = decimal_digit5(center_latched, 3'd2);

            6'd13:
                msg_char = decimal_digit5(center_latched, 3'd3);

            6'd14:
                msg_char = decimal_digit5(center_latched, 3'd4);


            6'd15:
                msg_char = " ";


            // R=xxxxx
            6'd16:
                msg_char = "R";

            6'd17:
                msg_char = "=";

            6'd18:
                msg_char = decimal_digit5(right_latched, 3'd0);

            6'd19:
                msg_char = decimal_digit5(right_latched, 3'd1);

            6'd20:
                msg_char = decimal_digit5(right_latched, 3'd2);

            6'd21:
                msg_char = decimal_digit5(right_latched, 3'd3);

            6'd22:
                msg_char = decimal_digit5(right_latched, 3'd4);


            6'd23:
                msg_char = " ";

            6'd24:
                msg_char = "m";

            6'd25:
                msg_char = "m";

            6'd26:
                msg_char = 8'h0D;

            6'd27:
                msg_char = 8'h0A;


            default:
                msg_char = 8'h20;

        endcase

    end


    //================================================
    // Message controller
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            left_latched   <= 16'd0;
            center_latched <= 16'd0;
            right_latched  <= 16'd0;

            sending <= 1'b0;

            waiting_tx_done <= 1'b0;

            tx_index <= 6'd0;

            tx_data <= 8'd0;

            tx_start <= 1'b0;

        end


        else begin

            tx_start <= 1'b0;


            //================================================
            // Scan complete
            //================================================

            if(
                scan_done
                &&
                !sending
            ) begin

                left_latched <=
                    left_distance_mm;

                center_latched <=
                    center_distance_mm;

                right_latched <=
                    right_distance_mm;

                tx_index <= 6'd0;

                sending <= 1'b1;

                waiting_tx_done <= 1'b0;

            end


            //================================================
            // Sending
            //================================================

            else if(sending) begin


                // Start current character
                if(
                    !waiting_tx_done
                    &&
                    !uart_busy
                ) begin

                    tx_data <= msg_char;

                    tx_start <= 1'b1;

                    waiting_tx_done <= 1'b1;

                end


                // Character completed
                else if(
                    waiting_tx_done
                    &&
                    tx_done
                ) begin

                    waiting_tx_done <= 1'b0;


                    if(tx_index == 6'd27) begin

                        sending <= 1'b0;

                        tx_index <= 6'd0;

                    end

                    else begin

                        tx_index <=
                            tx_index + 1'b1;

                    end

                end

            end

        end

    end


    //================================================
    // UART transmitter
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            uart_tx <= 1'b1;

            uart_busy <= 1'b0;

            tx_done <= 1'b0;

            baud_counter <= 16'd0;

            bit_index <= 4'd0;

            tx_shift <= 8'd0;

        end


        else begin

            tx_done <= 1'b0;


            //================================================
            // Start byte
            //================================================

            if(
                tx_start
                &&
                !uart_busy
            ) begin

                uart_busy <= 1'b1;

                baud_counter <= 16'd0;

                bit_index <= 4'd0;

                tx_shift <= tx_data;

                // Start bit
                uart_tx <= 1'b0;

            end


            //================================================
            // Transmitting
            //================================================

            else if(uart_busy) begin

                if(
                    baud_counter
                    >=
                    CLKS_PER_BIT - 1
                ) begin

                    baud_counter <= 16'd0;


                    case(bit_index)

                        4'd0: begin
                            uart_tx <= tx_shift[0];
                            bit_index <= 4'd1;
                        end

                        4'd1: begin
                            uart_tx <= tx_shift[1];
                            bit_index <= 4'd2;
                        end

                        4'd2: begin
                            uart_tx <= tx_shift[2];
                            bit_index <= 4'd3;
                        end

                        4'd3: begin
                            uart_tx <= tx_shift[3];
                            bit_index <= 4'd4;
                        end

                        4'd4: begin
                            uart_tx <= tx_shift[4];
                            bit_index <= 4'd5;
                        end

                        4'd5: begin
                            uart_tx <= tx_shift[5];
                            bit_index <= 4'd6;
                        end

                        4'd6: begin
                            uart_tx <= tx_shift[6];
                            bit_index <= 4'd7;
                        end

                        4'd7: begin
                            uart_tx <= tx_shift[7];
                            bit_index <= 4'd8;
                        end


                        // Stop
                        4'd8: begin

                            uart_tx <= 1'b1;

                            bit_index <= 4'd9;

                        end


                        // Finish
                        4'd9: begin

                            uart_tx <= 1'b1;

                            uart_busy <= 1'b0;

                            tx_done <= 1'b1;

                            bit_index <= 4'd0;

                        end


                        default: begin

                            uart_tx <= 1'b1;

                            uart_busy <= 1'b0;

                            bit_index <= 4'd0;

                        end

                    endcase

                end

                else begin

                    baud_counter <=
                        baud_counter + 1'b1;

                end

            end


            else begin

                uart_tx <= 1'b1;

            end

        end

    end


endmodule