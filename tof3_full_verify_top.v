//============================================================
// UART sequencer state
//============================================================

localparam TX_IDLE  = 2'd0;
localparam TX_START = 2'd1;
localparam TX_BUSY  = 2'd2;
localparam TX_NEXT  = 2'd3;

reg [1:0] tx_state;


//============================================================
// UART sequencer
//============================================================

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        left_latched <=
            16'd0;

        center_latched <=
            16'd0;

        right_latched <=
            16'd0;


        error_latched <=
            3'b111;

        valid_latched <=
            3'b000;


        tx_data <=
            8'h00;

        tx_send <=
            1'b0;


        report_active <=
            1'b0;

        char_index <=
            6'd0;

        wait_busy <=
            1'b0;

        tx_state <=
            TX_IDLE;

    end

    else
    begin

        tx_send <=
            1'b0;


        case(tx_state)


        //====================================================
        // Waiting for report timer
        //====================================================

        TX_IDLE:
        begin

            if(report_request)
            begin

                left_latched <=
                    left_mm;

                center_latched <=
                    center_mm;

                right_latched <=
                    right_mm;


                error_latched <=
                {
                    left_error,
                    center_error,
                    right_error
                };


                valid_latched <=
                {
                    left_seen,
                    center_seen,
                    right_seen
                };


                char_index <=
                    6'd0;

                report_active <=
                    1'b1;

                tx_state <=
                    TX_START;

            end

        end


        //====================================================
        // Start one UART byte
        //====================================================

        TX_START:
        begin

            if(!tx_busy)
            begin

                tx_data <=
                    current_char;

                tx_send <=
                    1'b1;

                tx_state <=
                    TX_BUSY;

            end

        end


        //====================================================
        // Wait until transmitter becomes busy
        //====================================================

        TX_BUSY:
        begin

            if(tx_busy)
            begin

                tx_state <=
                    TX_NEXT;

            end

        end


        //====================================================
        // Wait byte finished
        //====================================================

        TX_NEXT:
        begin

            if(!tx_busy)
            begin

                if(char_index == 6'd33)
                begin

                    report_active <=
                        1'b0;

                    char_index <=
                        6'd0;

                    tx_state <=
                        TX_IDLE;

                end

                else
                begin

                    char_index <=
                        char_index + 1'b1;

                    tx_state <=
                        TX_START;

                end

            end

        end


        default:
        begin

            tx_state <=
                TX_IDLE;

        end


        endcase

    end

end


endmodule