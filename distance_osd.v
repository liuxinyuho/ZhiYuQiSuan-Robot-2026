
`timescale 1ns / 1ps

//==================================================
// ACG720 60K - DISTANCE OSD
//
// L: Left ToF
// F: Front ToF
// R: Right ToF
// U: Rear ToF
//
// Input unit: mm
// Invalid reading: ----
//==================================================

module distance_osd (
    input wire sys_clk,
    input wire pix_clk,
    input wire rst_n,

    input wire [15:0] left_mm,
    input wire [15:0] front_mm,
    input wire [15:0] right_mm,
    input wire [15:0] rear_mm,

    input wire left_ok,
    input wire front_ok,
    input wire right_ok,
    input wire rear_ok,

    input wire [11:0] hcount,
    input wire [11:0] vcount,
    input wire video_active,

    output wire osd_active,
    output wire [23:0] osd_rgb
);

//==================================================
// 1. Limit display to 0000~9999 mm
//==================================================

wire [15:0] val_l =
    (left_mm > 16'd9999) ? 16'd9999 : left_mm;

wire [15:0] val_f =
    (front_mm > 16'd9999) ? 16'd9999 : front_mm;

wire [15:0] val_r =
    (right_mm > 16'd9999) ? 16'd9999 : right_mm;

wire [15:0] val_u =
    (rear_mm > 16'd9999) ? 16'd9999 : rear_mm;

//==================================================
// 2. System-clock conversion and CDC handshake
//==================================================

localparam [1:0]
    ST_IDLE    = 2'd0,
    ST_CONVERT = 2'd1,
    ST_SEND    = 2'd2,
    ST_ACK     = 2'd3;

reg [1:0] state;

reg [22:0] refresh_count;
reg [3:0] bit_count;

reg [15:0] bin_l;
reg [15:0] bin_f;
reg [15:0] bin_r;
reg [15:0] bin_u;

// Four 4-digit BCD values
reg [63:0] bcd_sys;
reg [3:0] valid_sys;

reg request_sys;
reg ack_sys_s1;
reg ack_sys_s2;

// Add-three network for double-dabble
reg [63:0] bcd_adjust;
integer i;

always @(*) begin
    bcd_adjust = bcd_sys;

    for (i = 0; i < 16; i = i + 1) begin
        if (bcd_adjust[i*4 +: 4] >= 4'd5)
            bcd_adjust[i*4 +: 4] =
                bcd_adjust[i*4 +: 4] + 4'd3;
    end
end

//==================================================
// 3. Capture distances and convert to BCD
//==================================================

reg acknowledge_pix;

always @(posedge sys_clk or negedge rst_n) begin

    if (!rst_n) begin
        state <= ST_IDLE;
        refresh_count <= 23'd0;
        bit_count <= 4'd0;

        bin_l <= 16'd0;
        bin_f <= 16'd0;
        bin_r <= 16'd0;
        bin_u <= 16'd0;

        bcd_sys <= 64'd0;
        valid_sys <= 4'd0;

        request_sys <= 1'b0;
        ack_sys_s1 <= 1'b0;
        ack_sys_s2 <= 1'b0;

    end else begin

        ack_sys_s1 <= acknowledge_pix;
        ack_sys_s2 <= ack_sys_s1;

        case (state)

            ST_IDLE: begin

                if (refresh_count < 23'd4999999) begin
                    refresh_count <= refresh_count + 1'b1;
                end else if (request_sys == ack_sys_s2) begin

                    refresh_count <= 23'd0;

                    bin_l <= val_l;
                    bin_f <= val_f;
                    bin_r <= val_r;
                    bin_u <= val_u;

                    valid_sys <= {
                        rear_ok,
                        right_ok,
                        front_ok,
                        left_ok
                    };

                    bcd_sys <= 64'd0;
                    bit_count <= 4'd0;

                    state <= ST_CONVERT;

                end
            end

            ST_CONVERT: begin

                bcd_sys[15:0] <= {
                    bcd_adjust[14:0],
                    bin_l[15]
                };

                bcd_sys[31:16] <= {
                    bcd_adjust[30:16],
                    bin_f[15]
                };

                bcd_sys[47:32] <= {
                    bcd_adjust[46:32],
                    bin_r[15]
                };

                bcd_sys[63:48] <= {
                    bcd_adjust[62:48],
                    bin_u[15]
                };

                bin_l <= {bin_l[14:0],1'b0};
                bin_f <= {bin_f[14:0],1'b0};
                bin_r <= {bin_r[14:0],1'b0};
                bin_u <= {bin_u[14:0],1'b0};

                if (bit_count == 4'd15)
                    state <= ST_SEND;
                else
                    bit_count <= bit_count + 1'b1;
            end

            // BCD data is now stable.
            ST_SEND: begin
                request_sys <= ~request_sys;
                state <= ST_ACK;
            end

            // Keep the data stable until pixel domain
            // has captured it and acknowledged.
            ST_ACK: begin
                if (ack_sys_s2 == request_sys)
                    state <= ST_IDLE;
            end

            default:
                state <= ST_IDLE;

        endcase
    end
end

//==================================================
// 4. Pixel-clock receiver
//==================================================

reg req_pix_s1;
reg req_pix_s2;
reg req_seen_pix;

reg [63:0] bcd_pix;
reg [3:0] valid_pix;

always @(posedge pix_clk or negedge rst_n) begin

    if (!rst_n) begin

        req_pix_s1 <= 1'b0;
        req_pix_s2 <= 1'b0;
        req_seen_pix <= 1'b0;

        acknowledge_pix <= 1'b0;

        bcd_pix <= 64'd0;
        valid_pix <= 4'd0;

    end else begin

        req_pix_s1 <= request_sys;
        req_pix_s2 <= req_pix_s1;

        if (req_pix_s2 != req_seen_pix) begin

            bcd_pix <= bcd_sys;
            valid_pix <= valid_sys;

            req_seen_pix <= req_pix_s2;
            acknowledge_pix <= req_pix_s2;

        end
    end
end

//==================================================
// 5. Left-top OSD panel
//==================================================

wire panel =
    video_active &&
    (hcount >= 12'd16) &&
    (hcount < 12'd180) &&
    (vcount >= 12'd12) &&
    (vcount < 12'd148);

// Four rows
wire row_l = (vcount >= 12'd24 &&
              vcount < 12'd40);

wire row_f = (vcount >= 12'd52 &&
              vcount < 12'd68);

wire row_r = (vcount >= 12'd80 &&
              vcount < 12'd96);

wire row_u = (vcount >= 12'd108 &&
              vcount < 12'd124);

wire text_area =
    panel &&
    (hcount >= 12'd28) &&
    (hcount < 12'd156) &&
    (row_l || row_f || row_r || row_u);

// 16x16 character cells
wire [7:0] rel_x = hcount - 12'd28;

wire [3:0] rel_y =
    row_l ? (vcount - 12'd24) :
    row_f ? (vcount - 12'd52) :
    row_r ? (vcount - 12'd80) :
            (vcount - 12'd108);

wire [2:0] char_index = rel_x[6:4];
wire [2:0] font_x = rel_x[3:1];
wire [2:0] font_y = rel_y[3:1];

//==================================================
// 6. Select sensor row
//==================================================

reg [15:0] selected_bcd;
reg selected_valid;
reg [4:0] selected_label;

always @(*) begin

    selected_bcd = 16'd0;
    selected_valid = 1'b0;
    selected_label = 5'd10;

    if (row_l) begin
        selected_bcd = bcd_pix[15:0];
        selected_valid = valid_pix[0];
        selected_label = 5'd10; // L
    end
    else if (row_f) begin
        selected_bcd = bcd_pix[31:16];
        selected_valid = valid_pix[1];
        selected_label = 5'd11; // F
    end
    else if (row_r) begin
        selected_bcd = bcd_pix[47:32];
        selected_valid = valid_pix[2];
        selected_label = 5'd12; // R
    end
    else if (row_u) begin
        selected_bcd = bcd_pix[63:48];
        selected_valid = valid_pix[3];
        selected_label = 5'd13; // U
    end

end

//==================================================
// 7. Character sequence: L:1234MM
//==================================================

reg [4:0] symbol;

always @(*) begin

    case (char_index)

        3'd0: symbol = selected_label;
        3'd1: symbol = 5'd15; // :

        3'd2: symbol =
            selected_valid ?
            {1'b0,selected_bcd[15:12]} : 5'd14;

        3'd3: symbol =
            selected_valid ?
            {1'b0,selected_bcd[11:8]} : 5'd14;

        3'd4: symbol =
            selected_valid ?
            {1'b0,selected_bcd[7:4]} : 5'd14;

        3'd5: symbol =
            selected_valid ?
            {1'b0,selected_bcd[3:0]} : 5'd14;

        3'd6: symbol = 5'd16; // M
        3'd7: symbol = 5'd16; // M

        default: symbol = 5'd17;

    endcase
end

//==================================================
// 8. 8x8 bitmap font
//==================================================

reg [63:0] glyph;

always @(*) begin

    case (symbol)

        5'd0: glyph = 64'h70_88_98_A8_C8_88_70_00;
        5'd1: glyph = 64'h20_60_20_20_20_20_70_00;
        5'd2: glyph = 64'h70_88_08_10_20_40_F8_00;
        5'd3: glyph = 64'h70_88_08_30_08_88_70_00;
        5'd4: glyph = 64'h10_30_50_90_F8_10_10_00;
        5'd5: glyph = 64'hF8_80_F0_08_08_88_70_00;
        5'd6: glyph = 64'h30_40_80_F0_88_88_70_00;
        5'd7: glyph = 64'hF8_08_10_20_40_40_40_00;
        5'd8: glyph = 64'h70_88_88_70_88_88_70_00;
        5'd9: glyph = 64'h70_88_88_78_08_10_60_00;

        // L
        5'd10: glyph = 64'h80_80_80_80_80_80_F8_00;

        // F
        5'd11: glyph = 64'hF8_80_80_F0_80_80_80_00;

        // R
        5'd12: glyph = 64'hF0_88_88_F0_A0_90_88_00;

        // U
        5'd13: glyph = 64'h88_88_88_88_88_88_70_00;

        // -
        5'd14: glyph = 64'h00_00_00_F8_00_00_00_00;

        // :
        5'd15: glyph = 64'h00_20_20_00_20_20_00_00;

        // M
        5'd16: glyph = 64'h88_D8_A8_A8_88_88_88_00;

        default:
            glyph = 64'h00_00_00_00_00_00_00_00;

    endcase
end

//==================================================
// 9. Select font row
//==================================================

reg [7:0] glyph_row;

always @(*) begin

    case (font_y)
        3'd0: glyph_row = glyph[63:56];
        3'd1: glyph_row = glyph[55:48];
        3'd2: glyph_row = glyph[47:40];
        3'd3: glyph_row = glyph[39:32];
        3'd4: glyph_row = glyph[31:24];
        3'd5: glyph_row = glyph[23:16];
        3'd6: glyph_row = glyph[15:8];
        default: glyph_row = glyph[7:0];
    endcase
end

//==================================================
// 10. Final pixel output
//==================================================

wire text_pixel =
    text_area &&
    glyph_row[3'd7 - font_x];

wire border_pixel =
    panel &&
    (
        hcount == 12'd16 ||
        hcount == 12'd179 ||
        vcount == 12'd12 ||
        vcount == 12'd147
    );

wire number_pixel =
    (char_index >= 3'd2 &&
     char_index <= 3'd5);

assign osd_active = panel;

assign osd_rgb =
    text_pixel ?
        ((!selected_valid && number_pixel)
         ? 24'hFF7060 : 24'hFFFFFF) :
    border_pixel ? 24'h42D9EE :
                   24'h0B2233;

endmodule
