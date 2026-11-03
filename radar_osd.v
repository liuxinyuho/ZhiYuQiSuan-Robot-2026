`timescale 1ns / 1ps

//============================================================
// ACG720 four-direction ToF radar + recovery-status OSD
// L = left, F = front, R = right, U = rear
// 800x480 HDMI layout. HC-SR04 remains in the existing front-assist path.
// This module is display-only: it does not change motor/A5/A6 decisions.
//============================================================
module radar_osd (
    input  wire        sys_clk,
    input  wire        pix_clk,
    input  wire        rst_n,

    input  wire [15:0] left_mm,
    input  wire [15:0] front_mm,
    input  wire [15:0] right_mm,
    input  wire [15:0] rear_mm,

    input  wire        left_ok,
    input  wire        front_ok,
    input  wire        right_ok,
    input  wire        rear_ok,

    input  wire        left_recovering,
    input  wire        front_recovering,
    input  wire        right_recovering,
    input  wire        rear_recovering,

    input  wire [11:0] hcount,
    input  wire [11:0] vcount,
    input  wire        video_active,
    output wire        osd_active,
    output wire [23:0] osd_rgb
);

//============================================================
// 1. System-clock domain: coherent 72-bit snapshot every ~100ms
//============================================================
reg [22:0] refresh_cnt;
reg [71:0] radar_sys;
reg        request_sys;
reg        ack_sys_s1;
reg        ack_sys_s2;
reg        acknowledge_pix;

always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin
        refresh_cnt     <= 23'd0;
        radar_sys       <= 72'd0;
        request_sys     <= 1'b0;
        ack_sys_s1      <= 1'b0;
        ack_sys_s2      <= 1'b0;
    end else begin
        ack_sys_s1 <= acknowledge_pix;
        ack_sys_s2 <= ack_sys_s1;

        if (refresh_cnt < 23'd4_999_999) begin
            refresh_cnt <= refresh_cnt + 1'b1;
        end else if (request_sys == ack_sys_s2) begin
            radar_sys <= {
                left_recovering, front_recovering,
                right_recovering, rear_recovering,
                left_ok, front_ok, right_ok, rear_ok,
                left_mm, front_mm, right_mm, rear_mm
            };
            request_sys <= ~request_sys;
            refresh_cnt <= 23'd0;
        end
    end
end

//============================================================
// 2. Pixel-clock domain: receive the stable snapshot
//============================================================
reg        req_pix_s1;
reg        req_pix_s2;
reg        req_seen_pix;
reg [71:0] radar_pix;

always @(posedge pix_clk or negedge rst_n) begin
    if (!rst_n) begin
        req_pix_s1      <= 1'b0;
        req_pix_s2      <= 1'b0;
        req_seen_pix    <= 1'b0;
        acknowledge_pix <= 1'b0;
        radar_pix       <= 72'd0;
    end else begin
        req_pix_s1 <= request_sys;
        req_pix_s2 <= req_pix_s1;

        if (req_pix_s2 != req_seen_pix) begin
            radar_pix       <= radar_sys;
            req_seen_pix    <= req_pix_s2;
            acknowledge_pix <= req_pix_s2;
        end
    end
end

wire left_recover_pix  = radar_pix[71];
wire front_recover_pix = radar_pix[70];
wire right_recover_pix = radar_pix[69];
wire rear_recover_pix  = radar_pix[68];

wire left_valid  = radar_pix[67];
wire front_valid = radar_pix[66];
wire right_valid = radar_pix[65];
wire rear_valid  = radar_pix[64];

wire [15:0] left_range  = radar_pix[63:48];
wire [15:0] front_range = radar_pix[47:32];
wire [15:0] right_range = radar_pix[31:16];
wire [15:0] rear_range  = radar_pix[15:0];

wire all_valid = left_valid && front_valid && right_valid && rear_valid;

//============================================================
// 3. Radar panel geometry
// Panel: x=603..787, y=62..205
// Bottom status row is reserved for L/F/R/U OK/REC/CHK.
//============================================================
localparam [11:0] PANEL_X0 = 12'd603;
localparam [11:0] PANEL_X1 = 12'd787;
localparam [11:0] PANEL_Y0 = 12'd62;
localparam [11:0] PANEL_Y1 = 12'd205;

localparam [11:0] CENTER_X = 12'd695;
localparam [11:0] CENTER_Y = 12'd143;

localparam [11:0] PLOT_X0 = 12'd616;
localparam [11:0] PLOT_X1 = 12'd774;
localparam [11:0] PLOT_Y0 = 12'd109;
localparam [11:0] PLOT_Y1 = 12'd177;

wire panel = video_active &&
             hcount >= PANEL_X0 && hcount <= PANEL_X1 &&
             vcount >= PANEL_Y0 && vcount <= PANEL_Y1;

wire border = panel &&
              (hcount == PANEL_X0 || hcount == PANEL_X1 ||
               vcount == PANEL_Y0 || vcount == PANEL_Y1);

wire title_divider = panel && vcount == 12'd95 &&
                     hcount >= 12'd612 && hcount <= 12'd780;

// Millimetres to pixels: 16mm/pixel, clamped to the panel edges.
wire [15:0] left_px_raw  = left_range  >> 4;
wire [15:0] front_px_raw = front_range >> 4;
wire [15:0] right_px_raw = right_range >> 4;
wire [15:0] rear_px_raw  = rear_range  >> 4;

wire [6:0] left_len =
    (left_px_raw >= 16'd79) ? 7'd79 : left_px_raw[6:0];
wire [6:0] right_len =
    (right_px_raw >= 16'd79) ? 7'd79 : right_px_raw[6:0];
wire [6:0] front_len =
    (front_px_raw >= 16'd34) ? 7'd34 : front_px_raw[6:0];
wire [6:0] rear_len =
    (rear_px_raw >= 16'd34) ? 7'd34 : rear_px_raw[6:0];

wire [11:0] left_x  = CENTER_X - {5'd0, left_len};
wire [11:0] right_x = CENTER_X + {5'd0, right_len};
wire [11:0] front_y = CENTER_Y - {5'd0, front_len};
wire [11:0] rear_y  = CENTER_Y + {5'd0, rear_len};

wire horizontal_axis = panel && vcount == CENTER_Y &&
                       hcount >= PLOT_X0 && hcount <= PLOT_X1;
wire vertical_axis = panel && hcount == CENTER_X &&
                     vcount >= PLOT_Y0 && vcount <= PLOT_Y1;

wire grid_tick = panel && (
    ((hcount == 12'd655 || hcount == 12'd675 ||
      hcount == 12'd715 || hcount == 12'd735) &&
     vcount >= 12'd139 && vcount <= 12'd147) ||
    ((vcount == 12'd123 || vcount == 12'd163) &&
     hcount >= 12'd691 && hcount <= 12'd699)
);

wire left_bar = panel && left_valid &&
                vcount >= 12'd141 && vcount <= 12'd145 &&
                hcount >= left_x && hcount < CENTER_X;
wire right_bar = panel && right_valid &&
                 vcount >= 12'd141 && vcount <= 12'd145 &&
                 hcount > CENTER_X && hcount <= right_x;
wire front_bar = panel && front_valid &&
                 hcount >= 12'd693 && hcount <= 12'd697 &&
                 vcount >= front_y && vcount < CENTER_Y;
wire rear_bar = panel && rear_valid &&
                hcount >= 12'd693 && hcount <= 12'd697 &&
                vcount > CENTER_Y && vcount <= rear_y;

wire left_marker = panel && left_valid &&
                   hcount >= (left_x - 12'd4) &&
                   hcount <= (left_x + 12'd4) &&
                   vcount >= 12'd139 && vcount <= 12'd147;
wire right_marker = panel && right_valid &&
                    hcount >= (right_x - 12'd4) &&
                    hcount <= (right_x + 12'd4) &&
                    vcount >= 12'd139 && vcount <= 12'd147;
wire front_marker = panel && front_valid &&
                    hcount >= 12'd691 && hcount <= 12'd699 &&
                    vcount >= (front_y - 12'd4) &&
                    vcount <= (front_y + 12'd4);
wire rear_marker = panel && rear_valid &&
                   hcount >= 12'd691 && hcount <= 12'd699 &&
                   vcount >= (rear_y - 12'd4) &&
                   vcount <= (rear_y + 12'd4);

wire robot_marker = panel &&
                    hcount >= 12'd690 && hcount <= 12'd700 &&
                    vcount >= 12'd138 && vcount <= 12'd148;

wire [23:0] left_color =
    (left_range < 16'd250) ? 24'hFF6868 :
    (left_range < 16'd500) ? 24'hFFD476 : 24'h70EAB8;
wire [23:0] front_color =
    (front_range < 16'd250) ? 24'hFF6868 :
    (front_range < 16'd500) ? 24'hFFD476 : 24'h70EAB8;
wire [23:0] right_color =
    (right_range < 16'd250) ? 24'hFF6868 :
    (right_range < 16'd500) ? 24'hFFD476 : 24'h70EAB8;
wire [23:0] rear_color =
    (rear_range < 16'd250) ? 24'hFF6868 :
    (rear_range < 16'd500) ? 24'hFFD476 : 24'h70EAB8;

//============================================================
// 4. Header text: RADAR + overall OK/CHK
//============================================================
wire title_area = panel &&
                  hcount >= 12'd614 && hcount < 12'd694 &&
                  vcount >= 12'd73 && vcount < 12'd89;
wire state_area = panel &&
                  hcount >= 12'd729 && hcount < 12'd777 &&
                  vcount >= 12'd73 && vcount < 12'd89;

wire [11:0] font_dx = hcount - (title_area ? 12'd614 : 12'd729);
wire [11:0] font_dy = vcount - 12'd73;
wire [3:0] char_index = font_dx[7:4];
wire [2:0] font_x = font_dx[3:1];
wire [2:0] font_y = font_dy[3:1];

reg [7:0] ascii_char;
always @(*) begin
    ascii_char = " ";
    if (title_area) begin
        case (char_index)
            4'd0: ascii_char = "R";
            4'd1: ascii_char = "A";
            4'd2: ascii_char = "D";
            4'd3: ascii_char = "A";
            4'd4: ascii_char = "R";
            default: ascii_char = " ";
        endcase
    end else if (state_area) begin
        if (all_valid) begin
            case (char_index)
                4'd0: ascii_char = "O";
                4'd1: ascii_char = "K";
                default: ascii_char = " ";
            endcase
        end else begin
            case (char_index)
                4'd0: ascii_char = "C";
                4'd1: ascii_char = "H";
                4'd2: ascii_char = "K";
                default: ascii_char = " ";
            endcase
        end
    end
end

reg [63:0] glyph;
always @(*) begin
    case (ascii_char)
        "A": glyph = 64'h70_88_88_F8_88_88_88_00;
        "C": glyph = 64'h70_88_80_80_80_88_70_00;
        "D": glyph = 64'hF0_88_88_88_88_88_F0_00;
        "H": glyph = 64'h88_88_88_F8_88_88_88_00;
        "K": glyph = 64'h88_90_A0_C0_A0_90_88_00;
        "O": glyph = 64'h70_88_88_88_88_88_70_00;
        "R": glyph = 64'hF0_88_88_F0_A0_90_88_00;
        default: glyph = 64'h00_00_00_00_00_00_00_00;
    endcase
end

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
wire glyph_pixel = (title_area || state_area) &&
                   glyph_row[3'd7 - font_x];

//============================================================
// 5. Direction labels around the radar plot
//============================================================
wire label_front = panel &&
                   hcount >= 12'd700 && hcount < 12'd708 &&
                   vcount >= 12'd109 && vcount < 12'd117;
wire label_left = panel &&
                  hcount >= 12'd619 && hcount < 12'd627 &&
                  vcount >= 12'd139 && vcount < 12'd147;
wire label_right = panel &&
                   hcount >= 12'd765 && hcount < 12'd773 &&
                   vcount >= 12'd139 && vcount < 12'd147;
wire label_rear = panel &&
                  hcount >= 12'd700 && hcount < 12'd708 &&
                  vcount >= 12'd170 && vcount < 12'd178;
wire label_area = label_front || label_left || label_right || label_rear;

wire [2:0] label_x = label_front || label_rear ? hcount[2:0] - 3'd4 :
                     label_left ? hcount[2:0] - 3'd3 :
                                  hcount[2:0] - 3'd5;
wire [2:0] label_y = label_front ? vcount[2:0] - 3'd5 :
                     label_rear  ? vcount[2:0] - 3'd2 :
                                   vcount[2:0] - 3'd3;

reg [7:0] label_char;
always @(*) begin
    if (label_front)      label_char = "F";
    else if (label_left)  label_char = "L";
    else if (label_right) label_char = "R";
    else                  label_char = "U";
end

reg [63:0] label_glyph;
always @(*) begin
    case (label_char)
        "F": label_glyph = 64'hF8_80_80_F0_80_80_80_00;
        "L": label_glyph = 64'h80_80_80_80_80_80_F8_00;
        "R": label_glyph = 64'hF0_88_88_F0_A0_90_88_00;
        "U": label_glyph = 64'h88_88_88_88_88_88_70_00;
        default: label_glyph = 64'd0;
    endcase
end

reg [7:0] label_glyph_row;
always @(*) begin
    case (label_y)
        3'd0: label_glyph_row = label_glyph[63:56];
        3'd1: label_glyph_row = label_glyph[55:48];
        3'd2: label_glyph_row = label_glyph[47:40];
        3'd3: label_glyph_row = label_glyph[39:32];
        3'd4: label_glyph_row = label_glyph[31:24];
        3'd5: label_glyph_row = label_glyph[23:16];
        3'd6: label_glyph_row = label_glyph[15:8];
        default: label_glyph_row = label_glyph[7:0];
    endcase
end
wire label_pixel = label_area && label_glyph_row[3'd7 - label_x];

wire [23:0] label_color =
    label_front ? (front_valid ? 24'h70EAB8 :
                   front_recover_pix ? 24'hFFD476 : 24'hFF6868) :
    label_left  ? (left_valid ? 24'h70EAB8 :
                   left_recover_pix ? 24'hFFD476 : 24'hFF6868) :
    label_right ? (right_valid ? 24'h70EAB8 :
                   right_recover_pix ? 24'hFFD476 : 24'hFF6868) :
                  (rear_valid ? 24'h70EAB8 :
                   rear_recover_pix ? 24'hFFD476 : 24'hFF6868);

//============================================================
// 6. Individual sensor recovery row: L:OK / F:REC / R:CHK / U:OK
// One native 8x8 font cell per character; four slots fit below the plot.
//============================================================
wire status_l_area = panel && hcount >= 12'd610 && hcount < 12'd650 &&
                     vcount >= 12'd188 && vcount < 12'd196;
wire status_f_area = panel && hcount >= 12'd652 && hcount < 12'd692 &&
                     vcount >= 12'd188 && vcount < 12'd196;
wire status_r_area = panel && hcount >= 12'd694 && hcount < 12'd734 &&
                     vcount >= 12'd188 && vcount < 12'd196;
wire status_u_area = panel && hcount >= 12'd736 && hcount < 12'd776 &&
                     vcount >= 12'd188 && vcount < 12'd196;
wire status_area = status_l_area || status_f_area ||
                   status_r_area || status_u_area;

wire [11:0] status_origin_x = status_l_area ? 12'd610 :
                              status_f_area ? 12'd652 :
                              status_r_area ? 12'd694 : 12'd736;
wire [5:0] status_dx = hcount - status_origin_x;
wire [2:0] status_char_index = status_dx[5:3];
wire [2:0] status_font_x = status_dx[2:0];
wire [2:0] status_font_y = vcount - 12'd188;

reg [7:0] status_char;
reg status_valid_selected;
reg status_recover_selected;
always @(*) begin
    status_char = " ";
    status_valid_selected = 1'b0;
    status_recover_selected = 1'b0;

    if (status_l_area) begin
        status_valid_selected = left_valid;
        status_recover_selected = left_recover_pix;
    end else if (status_f_area) begin
        status_valid_selected = front_valid;
        status_recover_selected = front_recover_pix;
    end else if (status_r_area) begin
        status_valid_selected = right_valid;
        status_recover_selected = right_recover_pix;
    end else begin
        status_valid_selected = rear_valid;
        status_recover_selected = rear_recover_pix;
    end

    case (status_char_index)
        3'd0: begin
            if (status_l_area)      status_char = "L";
            else if (status_f_area) status_char = "F";
            else if (status_r_area) status_char = "R";
            else                    status_char = "U";
        end
        3'd1: status_char = ":";
        3'd2: begin
            if (status_valid_selected) status_char = "O";
            else if (status_recover_selected) status_char = "R";
            else status_char = "C";
        end
        3'd3: begin
            if (status_valid_selected) status_char = "K";
            else if (status_recover_selected) status_char = "E";
            else status_char = "H";
        end
        3'd4: begin
            if (status_valid_selected) status_char = " ";
            else if (status_recover_selected) status_char = "C";
            else status_char = "K";
        end
        default: status_char = " ";
    endcase
end

reg [63:0] status_glyph;
always @(*) begin
    case (status_char)
        "C": status_glyph = 64'h70_88_80_80_80_88_70_00;
        "E": status_glyph = 64'hF8_80_80_F0_80_80_F8_00;
        "F": status_glyph = 64'hF8_80_80_F0_80_80_80_00;
        "H": status_glyph = 64'h88_88_88_F8_88_88_88_00;
        "K": status_glyph = 64'h88_90_A0_C0_A0_90_88_00;
        "L": status_glyph = 64'h80_80_80_80_80_80_F8_00;
        "O": status_glyph = 64'h70_88_88_88_88_88_70_00;
        "R": status_glyph = 64'hF0_88_88_F0_A0_90_88_00;
        "U": status_glyph = 64'h88_88_88_88_88_88_70_00;
        ":": status_glyph = 64'h00_20_20_00_20_20_00_00;
        default: status_glyph = 64'h00_00_00_00_00_00_00_00;
    endcase
end

reg [7:0] status_glyph_row;
always @(*) begin
    case (status_font_y)
        3'd0: status_glyph_row = status_glyph[63:56];
        3'd1: status_glyph_row = status_glyph[55:48];
        3'd2: status_glyph_row = status_glyph[47:40];
        3'd3: status_glyph_row = status_glyph[39:32];
        3'd4: status_glyph_row = status_glyph[31:24];
        3'd5: status_glyph_row = status_glyph[23:16];
        3'd6: status_glyph_row = status_glyph[15:8];
        default: status_glyph_row = status_glyph[7:0];
    endcase
end
wire status_text_pixel = status_area &&
                         status_glyph_row[3'd7 - status_font_x];
wire [23:0] status_text_color =
    status_valid_selected ? 24'h70EAB8 :
    status_recover_selected ? 24'hFFD476 : 24'hFF6868;

//============================================================
// 7. Compositing
//============================================================
assign osd_active = panel;
assign osd_rgb =
    border            ? 24'h38C9DC :
    glyph_pixel       ? (title_area ? 24'h70DDED :
                         all_valid ? 24'h70EAB8 : 24'hFFD476) :
    title_divider     ? 24'h32617A :
    status_text_pixel ? status_text_color :
    label_pixel       ? label_color :
    robot_marker      ? 24'hEDF6FC :
    front_marker      ? front_color :
    rear_marker       ? rear_color :
    left_marker       ? left_color :
    right_marker      ? right_color :
    front_bar         ? front_color :
    rear_bar          ? rear_color :
    left_bar          ? left_color :
    right_bar         ? right_color :
    grid_tick         ? 24'h52788A :
    horizontal_axis   ? 24'h52788A :
    vertical_axis     ? 24'h52788A :
                        24'h09253A;

endmodule
