
`timescale 1ns / 1ps

//====================================================
// ACG720 GW5AT-60K
// Motion OSD - Encoder + XY Odometry
//
// HDMI 800x480
// Wheel A/B/C/D: raw encoder counts/sample
// X/Y: millimeters
//
// Refresh: about 100 ms at 50 MHz
// Display range: -9999 to +9999
//====================================================

module motion_osd (
    input wire sys_clk,
    input wire pix_clk,
    input wire rst_n,

    input wire signed [31:0] speed_a,
    input wire signed [31:0] speed_b,
    input wire signed [31:0] speed_c,
    input wire signed [31:0] speed_d,

    input wire signed [31:0] odom_x_mm,
    input wire signed [31:0] odom_y_mm,

    input wire [11:0] hcount,
    input wire [11:0] vcount,
    input wire video_active,

    output wire osd_active,
    output wire [23:0] osd_rgb
);

//====================================================
// 1. Signed magnitude, saturation to 9999
//====================================================

wire [31:0] mag_a =
    speed_a[31] ? (~speed_a + 32'd1) : speed_a;

wire [31:0] mag_b =
    speed_b[31] ? (~speed_b + 32'd1) : speed_b;

wire [31:0] mag_c =
    speed_c[31] ? (~speed_c + 32'd1) : speed_c;

wire [31:0] mag_d =
    speed_d[31] ? (~speed_d + 32'd1) : speed_d;

wire [31:0] mag_x =
    odom_x_mm[31] ? (~odom_x_mm + 32'd1) :
                     odom_x_mm;

wire [31:0] mag_y =
    odom_y_mm[31] ? (~odom_y_mm + 32'd1) :
                     odom_y_mm;

wire [15:0] val_a =
    mag_a > 32'd9999 ? 16'd9999 : mag_a[15:0];

wire [15:0] val_b =
    mag_b > 32'd9999 ? 16'd9999 : mag_b[15:0];

wire [15:0] val_c =
    mag_c > 32'd9999 ? 16'd9999 : mag_c[15:0];

wire [15:0] val_d =
    mag_d > 32'd9999 ? 16'd9999 : mag_d[15:0];

wire [15:0] val_x =
    mag_x > 32'd9999 ? 16'd9999 : mag_x[15:0];

wire [15:0] val_y =
    mag_y > 32'd9999 ? 16'd9999 : mag_y[15:0];

//====================================================
// 2. System clock conversion controller
//====================================================

localparam [1:0]
    ST_IDLE = 2'd0,
    ST_BCD  = 2'd1,
    ST_REQ  = 2'd2,
    ST_WAIT = 2'd3;

reg [1:0] state;
reg [22:0] refresh_count;
reg [3:0] bit_count;

reg [15:0] bin_a;
reg [15:0] bin_b;
reg [15:0] bin_c;
reg [15:0] bin_d;
reg [15:0] bin_x;
reg [15:0] bin_y;

// Six values, four BCD digits each
reg [95:0] bcd_sys;
reg [5:0] sign_sys;

reg request_sys;
reg ack_sys_s1;
reg ack_sys_s2;

//====================================================
// 3. Double-dabble combinational add-three
//====================================================

reg [95:0] bcd_adjust;
integer i;

always @(*) begin

    bcd_adjust = bcd_sys;

    for (i = 0; i < 24; i = i + 1) begin

        if (bcd_adjust[i*4 +: 4] >= 4'd5)

            bcd_adjust[i*4 +: 4] =
                bcd_adjust[i*4 +: 4] + 4'd3;

    end

end

//====================================================
// 4. System clock domain
//====================================================

reg acknowledge_pix;

always @(posedge sys_clk or negedge rst_n) begin

    if (!rst_n) begin

        state <= ST_IDLE;
        refresh_count <= 23'd0;
        bit_count <= 4'd0;

        bin_a <= 16'd0;
        bin_b <= 16'd0;
        bin_c <= 16'd0;
        bin_d <= 16'd0;
        bin_x <= 16'd0;
        bin_y <= 16'd0;

        bcd_sys <= 96'd0;
        sign_sys <= 6'd0;

        request_sys <= 1'b0;

        ack_sys_s1 <= 1'b0;
        ack_sys_s2 <= 1'b0;

    end else begin

        // Two-stage ACK synchronizer
        ack_sys_s1 <= acknowledge_pix;
        ack_sys_s2 <= ack_sys_s1;

        case (state)

            ST_IDLE: begin

                if (refresh_count < 23'd4999999) begin

                    refresh_count <= refresh_count + 1'b1;

                end else if (request_sys == ack_sys_s2) begin

                    // Capture all six inputs together
                    bin_a <= val_a;
                    bin_b <= val_b;
                    bin_c <= val_c;
                    bin_d <= val_d;
                    bin_x <= val_x;
                    bin_y <= val_y;

                    sign_sys <= {
                        odom_y_mm[31],
                        odom_x_mm[31],
                        speed_d[31],
                        speed_c[31],
                        speed_b[31],
                        speed_a[31]
                    };

                    bcd_sys <= 96'd0;
                    bit_count <= 4'd0;

                    state <= ST_BCD;

                end
            end

            ST_BCD: begin

                bcd_sys[15:0] <= {
                    bcd_adjust[14:0], bin_a[15]
                };

                bcd_sys[31:16] <= {
                    bcd_adjust[30:16], bin_b[15]
                };

                bcd_sys[47:32] <= {
                    bcd_adjust[46:32], bin_c[15]
                };

                bcd_sys[63:48] <= {
                    bcd_adjust[62:48], bin_d[15]
                };

                bcd_sys[79:64] <= {
                    bcd_adjust[78:64], bin_x[15]
                };

                bcd_sys[95:80] <= {
                    bcd_adjust[94:80], bin_y[15]
                };

                bin_a <= {bin_a[14:0],1'b0};
                bin_b <= {bin_b[14:0],1'b0};
                bin_c <= {bin_c[14:0],1'b0};
                bin_d <= {bin_d[14:0],1'b0};
                bin_x <= {bin_x[14:0],1'b0};
                bin_y <= {bin_y[14:0],1'b0};

                if (bit_count == 4'd15) begin

                    state <= ST_REQ;

                end else begin

                    bit_count <= bit_count + 1'b1;

                end

            end

            // Toggle request after BCD becomes stable
            ST_REQ: begin

                request_sys <= ~request_sys;
                state <= ST_WAIT;

            end

            // Never change the BCD transfer registers
            // while the pixel domain is reading them
            ST_WAIT: begin

                if (ack_sys_s2 == request_sys) begin

                    refresh_count <= 23'd0;
                    state <= ST_IDLE;

                end

            end

            default: state <= ST_IDLE;

        endcase

    end

end

//====================================================
// 5. Pixel clock domain
// Handshake-protected multi-bit transfer
//====================================================

reg req_pix_s1;
reg req_pix_s2;
reg req_seen_pix;

reg [95:0] bcd_pix;
reg [5:0] sign_pix;

always @(posedge pix_clk or negedge rst_n) begin

    if (!rst_n) begin

        req_pix_s1 <= 1'b0;
        req_pix_s2 <= 1'b0;
        req_seen_pix <= 1'b0;

        acknowledge_pix <= 1'b0;

        bcd_pix <= 96'd0;
        sign_pix <= 6'd0;

    end else begin

        req_pix_s1 <= request_sys;
        req_pix_s2 <= req_pix_s1;

        if (req_pix_s2 != req_seen_pix) begin

            bcd_pix <= bcd_sys;
            sign_pix <= sign_sys;

            req_seen_pix <= req_pix_s2;
            acknowledge_pix <= req_pix_s2;

        end

    end

end


//====================================================
// 6. NEW MOTION OSD LAYOUT
//
// HDMI: 800 x 480
//
// Encoder panel:
// X = 20..787
// Y = 359..424
//
// Position panel:
// X = 603..787
// Y = 210..310
//====================================================

wire panel_wheel =
    video_active &&
    (hcount >= 12'd20) &&
    (hcount <  12'd788) &&
    (vcount >= 12'd359) &&
    (vcount <  12'd425);

wire panel_xy =
    video_active &&
    (hcount >= 12'd603) &&
    (hcount <  12'd788) &&
    (vcount >= 12'd210) &&
    (vcount <  12'd311);

//====================================================
// 7. Text layout controller - parallel region decode
//
// Timing optimization for GW5AT-60K:
// Decode each text window independently instead of using a
// long nested if/else-if chain to select char_x0/char_y0.
// The windows are non-overlapping in the configured layout.
//====================================================

wire area_wheel_title =
    panel_wheel &&
    (vcount >= 12'd366) && (vcount < 12'd382) &&
    (hcount >= 12'd34)  && (hcount < 12'd146);

wire area_wheel_a =
    panel_wheel &&
    (vcount >= 12'd393) && (vcount < 12'd409) &&
    (hcount >= 12'd34)  && (hcount < 12'd146);

wire area_wheel_b =
    panel_wheel &&
    (vcount >= 12'd393) && (vcount < 12'd409) &&
    (hcount >= 12'd222) && (hcount < 12'd334);

wire area_wheel_c =
    panel_wheel &&
    (vcount >= 12'd393) && (vcount < 12'd409) &&
    (hcount >= 12'd410) && (hcount < 12'd522);

wire area_wheel_d =
    panel_wheel &&
    (vcount >= 12'd393) && (vcount < 12'd409) &&
    (hcount >= 12'd598) && (hcount < 12'd710);

wire area_xy_title =
    panel_xy &&
    (vcount >= 12'd219) && (vcount < 12'd235) &&
    (hcount >= 12'd615) && (hcount < 12'd695);

wire area_xy_x =
    panel_xy &&
    (vcount >= 12'd251) && (vcount < 12'd267) &&
    (hcount >= 12'd615) && (hcount < 12'd759);

wire area_xy_y =
    panel_xy &&
    (vcount >= 12'd279) && (vcount < 12'd295) &&
    (hcount >= 12'd615) && (hcount < 12'd759);

wire text_area = area_wheel_title | area_wheel_a | area_wheel_b |
                 area_wheel_c | area_wheel_d | area_xy_title |
                 area_xy_x | area_xy_y;

wire title_wheel = area_wheel_title;
wire title_xy    = area_xy_title;

// Select numeric value using one-hot region flags.
wire [2:0] selected_value =
    ({3{area_wheel_b}} & 3'd1) |
    ({3{area_wheel_c}} & 3'd2) |
    ({3{area_wheel_d}} & 3'd3) |
    ({3{area_xy_x}}    & 3'd4) |
    ({3{area_xy_y}}    & 3'd5);

// Compute each constant-origin coordinate in parallel.
// This removes the serial path:
// hcount -> nested region mux -> char_x0 -> variable subtract.
wire [11:0] dx_x0 = hcount - 12'd34;
wire [11:0] dx_x1 = hcount - 12'd222;
wire [11:0] dx_x2 = hcount - 12'd410;
wire [11:0] dx_x3 = hcount - 12'd598;
wire [11:0] dx_xy = hcount - 12'd615;

wire [11:0] dy_wheel_title = vcount - 12'd366;
wire [11:0] dy_wheel_data  = vcount - 12'd393;
wire [11:0] dy_xy_title    = vcount - 12'd219;
wire [11:0] dy_xy_x        = vcount - 12'd251;
wire [11:0] dy_xy_y        = vcount - 12'd279;

wire [11:0] char_dx =
    ({12{area_wheel_title | area_wheel_a}} & dx_x0) |
    ({12{area_wheel_b}} & dx_x1) |
    ({12{area_wheel_c}} & dx_x2) |
    ({12{area_wheel_d}} & dx_x3) |
    ({12{area_xy_title | area_xy_x | area_xy_y}} & dx_xy);

wire [11:0] char_dy =
    ({12{area_wheel_title}} & dy_wheel_title) |
    ({12{area_wheel_a | area_wheel_b | area_wheel_c | area_wheel_d}} & dy_wheel_data) |
    ({12{area_xy_title}} & dy_xy_title) |
    ({12{area_xy_x}} & dy_xy_x) |
    ({12{area_xy_y}} & dy_xy_y);

wire [3:0] char_index = char_dx[7:4];
wire [2:0] font_x = char_dx[3:1];
wire [2:0] font_y = char_dy[3:1];

//====================================================
// 9. Select data for current text

//====================================================

reg [15:0] selected_bcd;
reg selected_sign;
reg [7:0] selected_label;
reg selected_mm;

always @(*) begin

    selected_bcd = 16'd0;
    selected_sign = 1'b0;
    selected_label = " ";
    selected_mm = 1'b0;

    case (selected_value)

        3'd0: begin

            selected_bcd = bcd_pix[15:0];
            selected_sign = sign_pix[0];
            selected_label = "A";

        end

        3'd1: begin

            selected_bcd = bcd_pix[31:16];
            selected_sign = sign_pix[1];
            selected_label = "B";

        end

        3'd2: begin

            selected_bcd = bcd_pix[47:32];
            selected_sign = sign_pix[2];
            selected_label = "C";

        end

        3'd3: begin

            selected_bcd = bcd_pix[63:48];
            selected_sign = sign_pix[3];
            selected_label = "D";

        end

        3'd4: begin

            selected_bcd = bcd_pix[79:64];
            selected_sign = sign_pix[4];
            selected_label = "X";
            selected_mm = 1'b1;

        end

        3'd5: begin

            selected_bcd = bcd_pix[95:80];
            selected_sign = sign_pix[5];
            selected_label = "Y";
            selected_mm = 1'b1;

        end

        default: begin

            selected_bcd = 16'd0;
            selected_sign = 1'b0;
            selected_label = " ";
            selected_mm = 1'b0;

        end

    endcase

end

//====================================================
// 9A. ASCII text generation
//====================================================

reg [7:0] ascii_char;

always @(*) begin

    ascii_char = " ";

    //========================================
    // Encoder title: ENC RAW
    //========================================

    if (title_wheel) begin

        case (char_index)

            4'd0: ascii_char = "E";
            4'd1: ascii_char = "N";
            4'd2: ascii_char = "C";

            4'd4: ascii_char = "R";
            4'd5: ascii_char = "A";
            4'd6: ascii_char = "W";

            default: ascii_char = " ";

        endcase

    end

    //========================================
    // Position title: XY MM
    //========================================

    else if (title_xy) begin

        case (char_index)

            4'd0: ascii_char = "X";
            4'd1: ascii_char = "Y";

            4'd3: ascii_char = "M";
            4'd4: ascii_char = "M";

            default: ascii_char = " ";

        endcase

    end

    //========================================
    // Numeric values
    //========================================

    else if (text_area) begin

        case (char_index)

            4'd0:
                ascii_char = selected_label;

            4'd1:
                ascii_char = ":";

            4'd2:
                ascii_char =
                    selected_sign ? "-" : "+";

            4'd3:
                ascii_char =
                    8'd48 + {4'd0, selected_bcd[15:12]};

            4'd4:
                ascii_char =
                    8'd48 + {4'd0, selected_bcd[11:8]};

            4'd5:
                ascii_char =
                    8'd48 + {4'd0, selected_bcd[7:4]};

            4'd6:
                ascii_char =
                    8'd48 + {4'd0, selected_bcd[3:0]};

            4'd7:
                ascii_char =
                    selected_mm ? "M" : " ";

            4'd8:
                ascii_char =
                    selected_mm ? "M" : " ";

            default:
                ascii_char = " ";

        endcase

    end

end

//====================================================
// 10. ASCII 8x8 font ROM
//====================================================

reg [63:0] glyph;

always @(*) begin

    case (ascii_char)

        "0": glyph = 64'h70_88_98_A8_C8_88_70_00;
        "1": glyph = 64'h20_60_20_20_20_20_70_00;
        "2": glyph = 64'h70_88_08_10_20_40_F8_00;
        "3": glyph = 64'h70_88_08_30_08_88_70_00;
        "4": glyph = 64'h10_30_50_90_F8_10_10_00;
        "5": glyph = 64'hF8_80_F0_08_08_88_70_00;
        "6": glyph = 64'h30_40_80_F0_88_88_70_00;
        "7": glyph = 64'hF8_08_10_20_40_40_40_00;
        "8": glyph = 64'h70_88_88_70_88_88_70_00;
        "9": glyph = 64'h70_88_88_78_08_10_60_00;

        "A": glyph = 64'h70_88_88_F8_88_88_88_00;
        "B": glyph = 64'hF0_88_88_F0_88_88_F0_00;
        "C": glyph = 64'h70_88_80_80_80_88_70_00;
        "D": glyph = 64'hF0_88_88_88_88_88_F0_00;
        "E": glyph = 64'hF8_80_80_F0_80_80_F8_00;
        "M": glyph = 64'h88_D8_A8_A8_88_88_88_00;
        "N": glyph = 64'h88_C8_A8_98_88_88_88_00;
        "R": glyph = 64'hF0_88_88_F0_A0_90_88_00;
        "W": glyph = 64'h88_88_88_A8_A8_A8_50_00;
        "X": glyph = 64'h88_88_50_20_50_88_88_00;
        "Y": glyph = 64'h88_88_50_20_20_20_20_00;

        ":": glyph = 64'h00_20_20_00_20_20_00_00;
        "+": glyph = 64'h00_20_20_F8_20_20_00_00;
        "-": glyph = 64'h00_00_00_F8_00_00_00_00;

        default:
            glyph = 64'h00_00_00_00_00_00_00_00;

    endcase

end

//====================================================
// 11. Font row selection
//====================================================

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

        default:
            glyph_row = glyph[7:0];

    endcase

end


//====================================================
// 12. Final OSD pixel output
// New national-contest display layout
//====================================================

// 8x8 bitmap scaled to 16x16
wire text_pixel =
    text_area &&
    glyph_row[3'd7 - font_x];

//====================================================
// Panel borders
//====================================================

wire wheel_border =
    panel_wheel &&
    (
        hcount == 12'd20 ||
        hcount == 12'd787 ||
        vcount == 12'd359 ||
        vcount == 12'd424
    );

wire xy_border =
    panel_xy &&
    (
        hcount == 12'd603 ||
        hcount == 12'd787 ||
        vcount == 12'd210 ||
        vcount == 12'd310
    );

wire border_pixel =
    wheel_border || xy_border;

//====================================================
// Horizontal divider lines
//====================================================

wire wheel_divider =
    panel_wheel &&
    vcount == 12'd386 &&
    hcount >= 12'd32 &&
    hcount <  12'd776;

wire xy_divider =
    panel_xy &&
    vcount == 12'd243 &&
    hcount >= 12'd615 &&
    hcount <  12'd776;

//====================================================
// Vertical dividers between four wheels
//====================================================

wire wheel_vertical =
    panel_wheel &&
    vcount >= 12'd390 &&
    vcount <  12'd416 &&
    (
        hcount == 12'd211 ||
        hcount == 12'd399 ||
        hcount == 12'd587
    );

wire divider_pixel =
    wheel_divider ||
    xy_divider ||
    wheel_vertical;

//====================================================
// Different color for panel title bands
//====================================================

wire title_background =
    (
        panel_wheel &&
        vcount < 12'd386
    ) ||
    (
        panel_xy &&
        vcount < 12'd243
    );

//====================================================
// Output
//====================================================

assign osd_active =
    panel_wheel || panel_xy;

assign osd_rgb =
    text_pixel ?
        (
            (title_wheel || title_xy) ?
            24'h70DDED :
            24'hFFFFFF
        ) :
    border_pixel ?
        24'h38C9DC :
    divider_pixel ?
        24'h32617A :
    title_background ?
        24'h11354C :
        24'h09253A;

endmodule
