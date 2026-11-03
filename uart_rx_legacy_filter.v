`timescale 1ns / 1ps

module uart_rx_legacy_filter (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [7:0] rx_data,
    input  wire       rx_done,
    output reg  [7:0] legacy_data,
    output reg        legacy_done
);

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_AFTER_AA  = 3'd1,
        ST_SKIP_CMD  = 3'd2,
        ST_SKIP_LEN  = 3'd3,
        ST_SKIP_DATA = 3'd4,
        ST_SKIP_SUM  = 3'd5;

    reg [2:0] state;
    reg [4:0] data_left;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= ST_IDLE;
            data_left   <= 5'd0;
            legacy_data <= 8'd0;
            legacy_done <= 1'b0;
        end else begin
            legacy_done <= 1'b0;

            if (rx_done) begin
                case (state)

                    ST_IDLE: begin
                        if (rx_data == 8'hAA) begin
                            state <= ST_AFTER_AA;
                        end else begin
                            legacy_data <= rx_data;
                            legacy_done <= 1'b1;
                        end
                    end

                    ST_AFTER_AA: begin
                        if (rx_data == 8'h55) begin
                            state <= ST_SKIP_CMD;
                        end else if (rx_data == 8'hAA) begin
                            state <= ST_AFTER_AA;
                        end else begin
                            legacy_data <= rx_data;
                            legacy_done <= 1'b1;
                            state <= ST_IDLE;
                        end
                    end

                    ST_SKIP_CMD: begin
                        state <= ST_SKIP_LEN;
                    end

                    ST_SKIP_LEN: begin
                        if (rx_data == 8'd0) begin
                            state <= ST_SKIP_SUM;
                        end else if (rx_data <= 8'd16) begin
                            data_left <= rx_data[4:0];
                            state <= ST_SKIP_DATA;
                        end else begin
                            state <= ST_IDLE;
                            data_left <= 5'd0;
                        end
                    end

                    ST_SKIP_DATA: begin
                        if (data_left <= 5'd1) begin
                            data_left <= 5'd0;
                            state <= ST_SKIP_SUM;
                        end else begin
                            data_left <= data_left - 1'b1;
                        end
                    end

                    ST_SKIP_SUM: begin
                        state <= ST_IDLE;
                    end

                    default: begin
                        state <= ST_IDLE;
                        data_left <= 5'd0;
                    end

                endcase
            end
        end
    end

endmodule