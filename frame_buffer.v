
`timescale 1ns / 1ps

//============================================================
// ACG720-GW5AT60K + OV5640
//
// Frame Buffer
//
// Resolution : 320 x 240
// Pixel Type : RGB565
// Pixel Bits : 16
//
// Memory Size:
// 320 * 240 * 16 = 1228800 bits
//
// Write Clock : camera_pclk
// Read Clock  : clk_vga
//============================================================

module frame_buffer (
    input               wr_clk,
    input               wr_en,
    input      [16:0]   wr_addr,
    input      [15:0]   wr_data,

    input               rd_clk,
    input               rd_en,
    input      [16:0]   rd_addr,
    output reg [15:0]   rd_data
);

    //================================================
    // RGB565 Frame Memory
    //================================================

    reg [15:0] frame_mem [0:76799];

    //================================================
    // Camera Write: registered, reversible address bridge
    //================================================
    // This isolates the short camera-clock register -> BSRAM ADA hold path.
    // All 17 address bits are encoded into registered mixed-XOR signals.
    // The corresponding decoded addresses are identical to wr_addr.
    // In this stage, wr_data/wr_en are delayed exactly the same one clock.
    // Extra write latency = one camera_pclk period; the pixels are unchanged.

    function [16:0] addr_encode17;
        input [16:0] a;
        integer k;
        begin
            addr_encode17 = 17'd0;

            for (k = 0; k < 4; k = k + 1) begin
                addr_encode17[3*k]   = a[3*k] ^ a[3*k+1];
                addr_encode17[3*k+1] = a[3*k+1] ^ a[3*k+2];
                addr_encode17[3*k+2] =
                    a[3*k] ^ a[3*k+1] ^ a[3*k+2];
            end

            addr_encode17[12] = a[12] ^ a[13];
            addr_encode17[13] = a[13] ^ a[14];
            addr_encode17[14] = a[12] ^ a[13] ^ a[14];
            addr_encode17[15] = a[15] ^ a[12];
            addr_encode17[16] = a[16] ^ a[15];
        end
    endfunction

    function [16:0] addr_decode17;
        input [16:0] e;
        integer k;
        begin
            addr_decode17 = 17'd0;

            for (k = 0; k < 4; k = k + 1) begin
                addr_decode17[3*k] =
                    e[3*k+1] ^ e[3*k+2];

                addr_decode17[3*k+1] =
                    e[3*k] ^ e[3*k+1] ^ e[3*k+2];

                addr_decode17[3*k+2] =
                    e[3*k] ^ e[3*k+2];
            end

            addr_decode17[12] = e[13] ^ e[14];

            addr_decode17[13] =
                e[12] ^ e[13] ^ e[14];

            addr_decode17[14] = e[12] ^ e[14];

            addr_decode17[15] =
                e[15] ^ e[13] ^ e[14];

            addr_decode17[16] =
                e[16] ^ e[15] ^ e[13] ^ e[14];
        end
    endfunction

    reg [16:0] wr_addr_encoded_q = 17'd0;
    reg [15:0] wr_data_q = 16'd0;
    reg        wr_en_q = 1'b0;

    wire [16:0] wr_addr_decoded =
        addr_decode17(wr_addr_encoded_q);

    always @(posedge wr_clk) begin
        wr_addr_encoded_q <= addr_encode17(wr_addr);
        wr_data_q         <= wr_data;
        wr_en_q           <= wr_en;
    end

    always @(posedge wr_clk) begin
        if (wr_en_q)
            frame_mem[wr_addr_decoded] <= wr_data_q;
    end

    //================================================
    // HDMI Read
    //================================================

    always @(posedge rd_clk) begin
        if (rd_en)
            rd_data <= frame_mem[rd_addr];
    end

endmodule
