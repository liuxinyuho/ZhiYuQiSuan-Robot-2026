
`timescale 1ns / 1ps

//==================================================
// ACG720 60K ROBOT STATUS OSD
// HDMI: 800x480
//
// Bottom status bar:
// MODE: NAV      OBS: SAFE     WP:02/03
//
// Display only. No motor control output.
//==================================================

module robot_status_osd (
    input wire sys_clk,
    input wire pix_clk,
    input wire rst_n,

    input wire calib_done,
    input wire obstacle_active,
    input wire obstacle_error,

    input wire waypoint_active,
    input wire waypoint_done,
    input wire [2:0] waypoint_index,

    input wire nav_restore_active,
    input wire nav_waypoint_pause,
    input wire nav_avoid_cycle_active,

    input wire fault_stop,

    input wire tof_left_ok,
    input wire tof_front_ok,
    input wire tof_right_ok,
    input wire ultra_ok,

    input wire front_stop,
    input wire front_slow,
    input wire side_stop,

    input wire [11:0] hcount,
    input wire [11:0] vcount,
    input wire video_active,

    output wire osd_active,
    output wire [23:0] osd_rgb
);

//==================================================
// 1. MODE selection in system clock domain
//
// 0 CAL
// 1 FAULT
// 2 RESTORE
// 3 AVOID
// 4 PAUSE
// 5 NAV
// 6 DONE
// 7 MANUAL
//==================================================

wire [2:0] mode_now;

assign mode_now =
    !calib_done ? 3'd0 :
    (fault_stop || obstacle_error) ? 3'd1 :
    nav_restore_active ? 3'd2 :
    (obstacle_active || nav_avoid_cycle_active) ? 3'd3 :
    nav_waypoint_pause ? 3'd4 :
    waypoint_active ? 3'd5 :
    waypoint_done ? 3'd6 :
    3'd7;

//==================================================
// 2. Obstacle monitoring status
//
// 0 SAFE
// 1 SLOW
// 2 NEAR
// 3 CHECK
// 4 FAULT
//==================================================

wire [2:0] obstacle_now;

assign obstacle_now =
    (fault_stop || obstacle_error) ? 3'd4 :
    (!tof_left_ok || !tof_front_ok ||
     !tof_right_ok || !ultra_ok) ? 3'd3 :
    (front_stop || side_stop) ? 3'd2 :
    front_slow ? 3'd1 :
    3'd0;

//==================================================
// 3. Waypoint display
// Source index: 0,1,2
// Screen index: 1,2,3
//==================================================

wire wp_show_now;

assign wp_show_now =
    waypoint_active ||
    waypoint_done ||
    nav_waypoint_pause ||
    nav_restore_active ||
    nav_avoid_cycle_active;

wire [2:0] wp_number_now;

assign wp_number_now =
    waypoint_done ? 3'd3 :
    (waypoint_index >= 3'd2) ? 3'd3 :
    (waypoint_index + 3'd1);

//==================================================
// 4. CDC: request / acknowledge handshake
//
// System side sends a stable 10-bit snapshot.
// Pixel side captures it after synchronized req.
// Data remains unchanged until acknowledge.
//==================================================

wire [9:0] status_now = {
    mode_now,
    obstacle_now,
    wp_number_now,
    wp_show_now
};

reg [9:0] status_sys;
reg [9:0] status_pix;

reg request_sys;
reg acknowledge_pix;

reg ack_sys_s1;
reg ack_sys_s2;

reg req_pix_s1;
reg req_pix_s2;
reg req_seen_pix;

reg [22:0] refresh_count;

// System clock: update approximately every 100ms
// 50MHz * 0.1s = 5,000,000 clocks

always @(posedge sys_clk or negedge rst_n) begin

    if (!rst_n) begin

        status_sys <= 10'd0;
        request_sys <= 1'b0;

        ack_sys_s1 <= 1'b0;
        ack_sys_s2 <= 1'b0;

        refresh_count <= 23'd0;

    end else begin

        ack_sys_s1 <= acknowledge_pix;
        ack_sys_s2 <= ack_sys_s1;

        if (refresh_count < 23'd4999999) begin

            refresh_count <= refresh_count + 1'b1;

        end else if (request_sys == ack_sys_s2) begin

            refresh_count <= 23'd0;

            status_sys <= status_now;

            request_sys <= ~request_sys;

        end

    end

end

// Pixel clock side

always @(posedge pix_clk or negedge rst_n) begin

    if (!rst_n) begin

        req_pix_s1 <= 1'b0;
        req_pix_s2 <= 1'b0;
        req_seen_pix <= 1'b0;

        acknowledge_pix <= 1'b0;
        status_pix <= 10'd0;

    end else begin

        req_pix_s1 <= request_sys;
        req_pix_s2 <= req_pix_s1;

        if (req_pix_s2 != req_seen_pix) begin

            status_pix <= status_sys;

            req_seen_pix <= req_pix_s2;

            acknowledge_pix <= req_pix_s2;

        end

    end

end

wire [2:0] mode_pix = status_pix[9:7];
wire [2:0] obstacle_pix = status_pix[6:4];
wire [2:0] wp_number_pix = status_pix[3:1];
wire wp_show_pix = status_pix[0];

//==================================================
// 5. Status text selection
//==================================================

// Exactly seven ASCII characters per mode
reg [55:0] mode_text;

always @(*) begin

    case (mode_pix)

        3'd0: mode_text = "CAL    ";
        3'd1: mode_text = "FAULT  ";
        3'd2: mode_text = "RESTORE";
        3'd3: mode_text = "AVOID  ";
        3'd4: mode_text = "PAUSE  ";
        3'd5: mode_text = "NAV    ";
        3'd6: mode_text = "DONE   ";

        default:
              mode_text = "MANUAL ";

    endcase

end

// Exactly five ASCII characters per status
reg [39:0] obstacle_text;

always @(*) begin

    case (obstacle_pix)

        3'd0: obstacle_text = "SAFE ";
        3'd1: obstacle_text = "SLOW ";
        3'd2: obstacle_text = "NEAR ";
        3'd3: obstacle_text = "CHECK";

        default:
              obstacle_text = "FAULT";

    endcase

end

//==================================================
// 6. Bottom panel area
//==================================================

wire panel;

assign panel =
    video_active &&
    (hcount >= 12'd12) &&
    (hcount < 12'd788) &&
    (vcount >= 12'd430) &&
    (vcount < 12'd478);

wire text_area;

assign text_area =
    panel &&
    (hcount >= 12'd28) &&
    (hcount < 12'd764) &&
    (vcount >= 12'd446) &&
    (vcount < 12'd462);

wire [11:0] dx = hcount - 12'd28;
wire [11:0] dy = vcount - 12'd446;

wire [5:0] char_index = dx[9:4];

wire [2:0] font_x = dx[3:1];
wire [2:0] font_y = dy[3:1];

//==================================================
// 7. Character layout
//
// Index  0-11 : MODE:XXXXXXX
// Index 16-24 : OBS:XXXXX
// Index 32-39 : WP:01/03
//==================================================

reg [7:0] ascii_char;

always @(*) begin

    ascii_char = " ";

    case (char_index)

        6'd0: ascii_char = "M";
        6'd1: ascii_char = "O";
        6'd2: ascii_char = "D";
        6'd3: ascii_char = "E";
        6'd4: ascii_char = ":";

        6'd16: ascii_char = "O";
        6'd17: ascii_char = "B";
        6'd18: ascii_char = "S";
        6'd19: ascii_char = ":";

        6'd32: ascii_char = "W";
        6'd33: ascii_char = "P";
        6'd34: ascii_char = ":";

        6'd35:
            ascii_char =
                wp_show_pix ? "0" : "-";

        6'd36:
            ascii_char =
                wp_show_pix ?
                (8'd48 + {5'd0,wp_number_pix}) :
                "-";

        6'd37: ascii_char = "/";
        6'd38: ascii_char = "0";
        6'd39: ascii_char = "3";

        default: ascii_char = " ";

    endcase

    if (char_index >= 6'd5 &&
        char_index <= 6'd11) begin

        ascii_char =
            mode_text[
                ((6'd11-char_index)*8) +: 8
            ];

    end

    if (char_index >= 6'd20 &&
        char_index <= 6'd24) begin

        ascii_char =
            obstacle_text[
                ((6'd24-char_index)*8) +: 8
            ];

    end

end

//==================================================
// 8. ASCII character ROM
// 8x8 font, displayed at 16x16
//==================================================

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
        "F": glyph = 64'hF8_80_80_F0_80_80_80_00;
        "H": glyph = 64'h88_88_88_F8_88_88_88_00;
        "I": glyph = 64'h70_20_20_20_20_20_70_00;
        "K": glyph = 64'h88_90_A0_C0_A0_90_88_00;
        "L": glyph = 64'h80_80_80_80_80_80_F8_00;
        "M": glyph = 64'h88_D8_A8_A8_88_88_88_00;
        "N": glyph = 64'h88_C8_A8_98_88_88_88_00;
        "O": glyph = 64'h70_88_88_88_88_88_70_00;
        "P": glyph = 64'hF0_88_88_F0_80_80_80_00;
        "R": glyph = 64'hF0_88_88_F0_A0_90_88_00;
        "S": glyph = 64'h78_80_80_70_08_08_F0_00;
        "T": glyph = 64'hF8_20_20_20_20_20_20_00;
        "U": glyph = 64'h88_88_88_88_88_88_70_00;
        "V": glyph = 64'h88_88_88_88_88_50_20_00;
        "W": glyph = 64'h88_88_88_A8_A8_A8_50_00;
        "Y": glyph = 64'h88_88_50_20_20_20_20_00;

        ":": glyph = 64'h00_20_20_00_20_20_00_00;
        "/": glyph = 64'h08_08_10_20_40_80_80_00;
        "-": glyph = 64'h00_00_00_F8_00_00_00_00;
        "?": glyph = 64'h70_88_08_10_20_00_20_00;

        default:
             glyph = 64'h00_00_00_00_00_00_00_00;

    endcase

end

//==================================================
// 9. Font row selection
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

        default:
            glyph_row = glyph[7:0];

    endcase

end

//==================================================
// 10. OSD output
//==================================================

wire text_pixel;

assign text_pixel =
    text_area &&
    glyph_row[3'd7-font_x];

wire border_pixel;

assign border_pixel =
    panel &&
    (
        hcount == 12'd12 ||
        hcount == 12'd787 ||
        vcount == 12'd430 ||
        vcount == 12'd477
    );

wire warning_text;

assign warning_text =
    (obstacle_pix == 3'd4) &&
    (char_index >= 6'd20) &&
    (char_index <= 6'd24);

assign osd_active = panel;

assign osd_rgb =
    text_pixel ?
        (warning_text ? 24'hFF7777 : 24'hFFFFFF) :
    border_pixel ? 24'h52D9EE :
                   24'h102A3C;

endmodule
