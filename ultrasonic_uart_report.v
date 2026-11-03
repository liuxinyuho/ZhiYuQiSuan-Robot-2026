//////////////////////////////////////////////////////////////////////////////////
// File Name : ultrasonic_uart_report.v
//
// Description:
//   HC-SR04 UART Debug Output
//
// Output:
//   DIST=00523 mm
//   DIST=00187 mm
//   DIST=TIMEOUT
//
// Clock : 50 MHz
// UART  : 115200 8N1
//////////////////////////////////////////////////////////////////////////////////

module ultrasonic_uart_report #(

    parameter integer CLK_FREQ  = 50000000,
    parameter integer BAUD_RATE = 115200

)(

    input wire clk,
    input wire reset_n,

    input wire [15:0] distance_mm,
    input wire        distance_valid,
    input wire        echo_timeout,

    output reg uart_tx

);


    //================================================
    // UART timing
    //================================================

    localparam integer CLKS_PER_BIT =
        CLK_FREQ / BAUD_RATE;


    //================================================
    // Distance latch
    //================================================

    reg [15:0] distance_latched;


    //================================================
    // Message control
    //================================================

    reg sending;

    reg timeout_message;

    reg [4:0] tx_index;

    reg [7:0] tx_data;

    reg tx_start;

    reg waiting_tx_done;


    //================================================
    // UART status
    //================================================

    reg uart_busy;

    reg tx_done;

    reg [15:0] baud_counter;

    reg [3:0] bit_index;

    reg [7:0] tx_shift;


    //================================================
    // Timeout edge detect
    //
    // 防止 echo_timeout 保持高电平时不停发送 TIMEOUT
    //================================================

    reg echo_timeout_d;

    wire echo_timeout_rise;

    assign echo_timeout_rise =
        echo_timeout && !echo_timeout_d;


    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            echo_timeout_d <= 1'b0;

        end

        else begin

            echo_timeout_d <= echo_timeout;

        end

    end


    //================================================
    // Convert distance digit
    //
    // 00000 ~ 65535
    //================================================

    function [7:0] decimal_digit5;

        input [15:0] value;
        input [2:0] pos;

        reg [15:0] digit;

        begin

            case(pos)

                3'd0:
                    digit =
                        (value / 16'd10000) % 10;

                3'd1:
                    digit =
                        (value / 16'd1000) % 10;

                3'd2:
                    digit =
                        (value / 16'd100) % 10;

                3'd3:
                    digit =
                        (value / 16'd10) % 10;

                3'd4:
                    digit =
                        value % 10;

                default:
                    digit = 0;

            endcase


            decimal_digit5 =
                8'h30 + digit[7:0];

        end

    endfunction


    //================================================
    // Message character generator
    //================================================

    reg [7:0] msg_char;


    always @(*) begin

        msg_char = 8'h20;


        //================================================
        // Timeout:
        //
        // DIST=TIMEOUT\r\n
        //================================================

        if(timeout_message) begin

            case(tx_index)

                5'd0:
                    msg_char = "D";

                5'd1:
                    msg_char = "I";

                5'd2:
                    msg_char = "S";

                5'd3:
                    msg_char = "T";

                5'd4:
                    msg_char = "=";

                5'd5:
                    msg_char = "T";

                5'd6:
                    msg_char = "I";

                5'd7:
                    msg_char = "M";

                5'd8:
                    msg_char = "E";

                5'd9:
                    msg_char = "O";

                5'd10:
                    msg_char = "U";

                5'd11:
                    msg_char = "T";

                5'd12:
                    msg_char = 8'h0D;

                5'd13:
                    msg_char = 8'h0A;

                default:
                    msg_char = 8'h20;

            endcase

        end


        //================================================
        // Normal:
        //
        // DIST=00523 mm\r\n
        //================================================

        else begin

            case(tx_index)

                5'd0:
                    msg_char = "D";

                5'd1:
                    msg_char = "I";

                5'd2:
                    msg_char = "S";

                5'd3:
                    msg_char = "T";

                5'd4:
                    msg_char = "=";


                5'd5:
                    msg_char =
                        decimal_digit5(
                            distance_latched,
                            3'd0
                        );


                5'd6:
                    msg_char =
                        decimal_digit5(
                            distance_latched,
                            3'd1
                        );


                5'd7:
                    msg_char =
                        decimal_digit5(
                            distance_latched,
                            3'd2
                        );


                5'd8:
                    msg_char =
                        decimal_digit5(
                            distance_latched,
                            3'd3
                        );


                5'd9:
                    msg_char =
                        decimal_digit5(
                            distance_latched,
                            3'd4
                        );


                5'd10:
                    msg_char = " ";


                5'd11:
                    msg_char = "m";


                5'd12:
                    msg_char = "m";


                5'd13:
                    msg_char = 8'h0D;


                5'd14:
                    msg_char = 8'h0A;


                default:
                    msg_char = 8'h20;

            endcase

        end

    end


    //================================================
    // Message controller
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            distance_latched <= 16'd0;

            sending <= 1'b0;

            timeout_message <= 1'b0;

            tx_index <= 5'd0;

            tx_data <= 8'd0;

            tx_start <= 1'b0;

            waiting_tx_done <= 1'b0;

        end


        else begin

            // default
            tx_start <= 1'b0;


            //================================================
            // New distance
            //================================================

            if(
                distance_valid
                &&
                !sending
            ) begin

                distance_latched <=
                    distance_mm;

                timeout_message <= 1'b0;

                tx_index <= 5'd0;

                sending <= 1'b1;

                waiting_tx_done <= 1'b0;

            end


            //================================================
            // Timeout event
            //================================================

            else if(
                echo_timeout_rise
                &&
                !sending
            ) begin

                timeout_message <= 1'b1;

                tx_index <= 5'd0;

                sending <= 1'b1;

                waiting_tx_done <= 1'b0;

            end


            //================================================
            // Sending message
            //================================================

            else if(sending) begin


                //================================================
                // Start current character
                //================================================

                if(
                    !waiting_tx_done
                    &&
                    !uart_busy
                ) begin

                    tx_data <= msg_char;

                    tx_start <= 1'b1;

                    waiting_tx_done <= 1'b1;

                end


                //================================================
                // Current character finished
                //================================================

                else if(
                    waiting_tx_done
                    &&
                    tx_done
                ) begin

                    waiting_tx_done <= 1'b0;


                    //============================================
                    // Normal message completed
                    //============================================

                    if(
                        !timeout_message
                        &&
                        tx_index == 5'd14
                    ) begin

                        sending <= 1'b0;

                        tx_index <= 5'd0;

                    end


                    //============================================
                    // Timeout message completed
                    //============================================

                    else if(
                        timeout_message
                        &&
                        tx_index == 5'd13
                    ) begin

                        sending <= 1'b0;

                        tx_index <= 5'd0;

                    end


                    //============================================
                    // Next character
                    //============================================

                    else begin

                        tx_index <=
                            tx_index + 1'b1;

                    end

                end

            end

        end

    end


    //================================================
    // UART TX
    //
    // 115200
    // 8 data bits
    // no parity
    // 1 stop bit
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

                        // Data bit 0
                        4'd0: begin

                            uart_tx <=
                                tx_shift[0];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd1: begin

                            uart_tx <=
                                tx_shift[1];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd2: begin

                            uart_tx <=
                                tx_shift[2];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd3: begin

                            uart_tx <=
                                tx_shift[3];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd4: begin

                            uart_tx <=
                                tx_shift[4];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd5: begin

                            uart_tx <=
                                tx_shift[5];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd6: begin

                            uart_tx <=
                                tx_shift[6];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        4'd7: begin

                            uart_tx <=
                                tx_shift[7];

                            bit_index <=
                                bit_index + 1'b1;

                        end


                        // Stop bit
                        4'd8: begin

                            uart_tx <= 1'b1;

                            bit_index <=
                                bit_index + 1'b1;

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