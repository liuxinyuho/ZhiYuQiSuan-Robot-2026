`timescale 1ns / 1ps

//======================================================================
// ACG720 AI SETTINGS OSD
// Display-only page selected by SW0. It never controls motor/A5/A6 logic.
// Pixel clock domain: all asynchronous status bits are two-flop synchronized.
// 800x480 layout: camera left, AI/system status right, status footer.
//======================================================================
module ai_settings_osd (
    input  wire        pix_clk,
    input  wire        rst_n,
    input  wire        video_active,
    input  wire        page_enable,

    input  wire        host_mode_active,
    input  wire        host_ai_mode,
    input  wire        host_control_active,
    input  wire        host_watchdog_ok,
    input  wire        host_stop_req,
    input  wire        ultrasonic_ok,
    input  wire        ultrasonic_recovering,

    input  wire [11:0] hcount,
    input  wire [11:0] vcount,

    output wire        osd_active,
    output wire [23:0] osd_rgb
);

//----------------------------------------------------------------------
// 1. Synchronize persistent system status into the HDMI pixel domain.
//----------------------------------------------------------------------
reg host_mode_meta, host_mode_sync;
reg ai_mode_meta, ai_mode_sync;
reg control_meta, control_sync;
reg watchdog_meta, watchdog_sync;
reg stop_meta, stop_sync;
reg ultra_meta, ultra_sync;
reg ultra_recover_meta, ultra_recover_sync;

always @(posedge pix_clk or negedge rst_n) begin
    if (!rst_n) begin
        host_mode_meta <= 1'b0;
        host_mode_sync <= 1'b0;
        ai_mode_meta   <= 1'b0;
        ai_mode_sync   <= 1'b0;
        control_meta   <= 1'b0;
        control_sync   <= 1'b0;
        watchdog_meta  <= 1'b0;
        watchdog_sync  <= 1'b0;
        stop_meta      <= 1'b0;
        stop_sync      <= 1'b0;
        ultra_meta          <= 1'b0;
        ultra_sync          <= 1'b0;
        ultra_recover_meta  <= 1'b0;
        ultra_recover_sync  <= 1'b0;
    end else begin
        host_mode_meta <= host_mode_active;
        host_mode_sync <= host_mode_meta;
        ai_mode_meta   <= host_ai_mode;
        ai_mode_sync   <= ai_mode_meta;
        control_meta   <= host_control_active;
        control_sync   <= control_meta;
        watchdog_meta  <= host_watchdog_ok;
        watchdog_sync  <= watchdog_meta;
        stop_meta      <= host_stop_req;
        stop_sync      <= stop_meta;
        ultra_meta         <= ultrasonic_ok;
        ultra_sync         <= ultra_meta;
        ultra_recover_meta <= ultrasonic_recovering;
        ultra_recover_sync <= ultra_recover_meta;
    end
end

wire host_link_ok = host_mode_sync && watchdog_sync;
wire host_run_ok  = host_mode_sync && ai_mode_sync && control_sync &&
                     watchdog_sync && !stop_sync;

//----------------------------------------------------------------------
// 2. Panel geometry.
//----------------------------------------------------------------------
wire page_on = video_active && page_enable;

wire header_area = page_on &&
                   hcount >= 12'd16 && hcount <= 12'd784 &&
                   vcount >= 12'd12 && vcount <= 12'd52;

wire right_status_area = page_on &&
                         hcount >= 12'd530 && hcount <= 12'd784 &&
                         vcount >= 12'd62 && vcount <= 12'd286;

wire right_data_area = page_on &&
                       hcount >= 12'd530 && hcount <= 12'd784 &&
                       vcount >= 12'd298 && vcount <= 12'd420;

wire footer_area = page_on &&
                   hcount >= 12'd16 && hcount <= 12'd784 &&
                   vcount >= 12'd436 && vcount <= 12'd470;

wire camera_border = page_on &&
                     hcount >= 12'd16 && hcount <= 12'd520 &&
                     vcount >= 12'd62 && vcount <= 12'd420 &&
                     (hcount == 12'd16 || hcount == 12'd520 ||
                      vcount == 12'd62 || vcount == 12'd420);
wire camera_label_area = page_on &&
                         hcount >= 12'd32 && hcount < 12'd104 &&
                         vcount >= 12'd70 && vcount < 12'd78;

wire border_pixel =
    (header_area && (hcount == 12'd16 || hcount == 12'd784 ||
                     vcount == 12'd12 || vcount == 12'd52)) ||
    (right_status_area && (hcount == 12'd530 || hcount == 12'd784 ||
                           vcount == 12'd62 || vcount == 12'd286)) ||
    (right_data_area && (hcount == 12'd530 || hcount == 12'd784 ||
                         vcount == 12'd298 || vcount == 12'd420)) ||
    (footer_area && (hcount == 12'd16 || hcount == 12'd784 ||
                     vcount == 12'd436 || vcount == 12'd470)) ||
    camera_border;

wire divider_pixel = page_on &&
    ((hcount >= 12'd542 && hcount <= 12'd772 &&
      (vcount == 12'd98 || vcount == 12'd136 ||
       vcount == 12'd174 || vcount == 12'd212 ||
       vcount == 12'd250 || vcount == 12'd336 ||
       vcount == 12'd372 || vcount == 12'd404)) ||
     (hcount >= 12'd28 && hcount <= 12'd508 && vcount == 12'd88));

//----------------------------------------------------------------------
// 3. Text line selection. All text uses 8x8 glyphs at native pixel scale.
//----------------------------------------------------------------------
reg        text_area;
reg [4:0]  text_line;
reg [11:0] text_origin_x;
reg [11:0] text_origin_y;

always @(*) begin
    text_area     = 1'b0;
    text_line     = 5'd0;
    text_origin_x = 12'd0;
    text_origin_y = 12'd0;

    // Main title.
    if (header_area && hcount >= 12'd32 && hcount < 12'd176 &&
        vcount >= 12'd26 && vcount < 12'd34) begin
        text_area = 1'b1; text_line = 5'd0;
        text_origin_x = 12'd32; text_origin_y = 12'd26;
    end
    // Camera label.
    else if (page_on && hcount >= 12'd32 && hcount < 12'd104 &&
             vcount >= 12'd70 && vcount < 12'd78) begin
        text_area = 1'b1; text_line = 5'd1;
        text_origin_x = 12'd32; text_origin_y = 12'd70;
    end
    // System status title.
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd650 &&
             vcount >= 12'd74 && vcount < 12'd82) begin
        text_area = 1'b1; text_line = 5'd2;
        text_origin_x = 12'd546; text_origin_y = 12'd74;
    end
    // BM1684 link label and value.
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd634 &&
             vcount >= 12'd106 && vcount < 12'd114) begin
        text_area = 1'b1; text_line = 5'd3;
        text_origin_x = 12'd546; text_origin_y = 12'd106;
    end
    else if (right_status_area && hcount >= 12'd690 && hcount < 12'd754 &&
             vcount >= 12'd106 && vcount < 12'd114) begin
        text_area = 1'b1; text_line = host_link_ok ? 5'd14 : 5'd15;
        text_origin_x = 12'd690; text_origin_y = 12'd106;
    end
    // AI mode label and value.
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd602 &&
             vcount >= 12'd144 && vcount < 12'd152) begin
        text_area = 1'b1; text_line = 5'd4;
        text_origin_x = 12'd546; text_origin_y = 12'd144;
    end
    else if (right_status_area && hcount >= 12'd690 && hcount < 12'd754 &&
             vcount >= 12'd144 && vcount < 12'd152) begin
        text_area = 1'b1;
        text_line = (host_mode_sync && ai_mode_sync) ? 5'd16 : 5'd17;
        text_origin_x = 12'd690; text_origin_y = 12'd144;
    end
    // Host control label and value.
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd618 &&
             vcount >= 12'd182 && vcount < 12'd190) begin
        text_area = 1'b1; text_line = 5'd5;
        text_origin_x = 12'd546; text_origin_y = 12'd182;
    end
    else if (right_status_area && hcount >= 12'd690 && hcount < 12'd754 &&
             vcount >= 12'd182 && vcount < 12'd190) begin
        text_area = 1'b1; text_line = host_run_ok ? 5'd18 : 5'd19;
        text_origin_x = 12'd690; text_origin_y = 12'd182;
    end
    // Watchdog label and value.
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd618 &&
             vcount >= 12'd220 && vcount < 12'd228) begin
        text_area = 1'b1; text_line = 5'd6;
        text_origin_x = 12'd546; text_origin_y = 12'd220;
    end
    else if (right_status_area && hcount >= 12'd690 && hcount < 12'd754 &&
             vcount >= 12'd220 && vcount < 12'd228) begin
        text_area = 1'b1; text_line = watchdog_sync ? 5'd20 : 5'd21;
        text_origin_x = 12'd690; text_origin_y = 12'd220;
    end
    // Front ultrasonic health label and value (ToF remains the radar source).
    else if (right_status_area && hcount >= 12'd546 && hcount < 12'd626 &&
             vcount >= 12'd258 && vcount < 12'd266) begin
        text_area = 1'b1; text_line = 5'd7;
        text_origin_x = 12'd546; text_origin_y = 12'd258;
    end
    else if (right_status_area && hcount >= 12'd690 && hcount < 12'd754 &&
             vcount >= 12'd258 && vcount < 12'd266) begin
        text_area = 1'b1;
        text_line = ultra_sync ? 5'd22 :
                    ultra_recover_sync ? 5'd25 : 5'd21;
        text_origin_x = 12'd690; text_origin_y = 12'd258;
    end
    // AI data panel title.
    else if (right_data_area && hcount >= 12'd546 && hcount < 12'd602 &&
             vcount >= 12'd310 && vcount < 12'd318) begin
        text_area = 1'b1; text_line = 5'd8;
        text_origin_x = 12'd546; text_origin_y = 12'd310;
    end
    // Astra S and model telemetry are not in the current FPGA UART packet;
    // show WAIT rather than inventing a device status.
    else if (right_data_area && hcount >= 12'd546 && hcount < 12'd602 &&
             vcount >= 12'd346 && vcount < 12'd354) begin
        text_area = 1'b1; text_line = 5'd9;
        text_origin_x = 12'd546; text_origin_y = 12'd346;
    end
    else if (right_data_area && hcount >= 12'd658 && hcount < 12'd706 &&
             vcount >= 12'd346 && vcount < 12'd354) begin
        text_area = 1'b1; text_line = 5'd15;
        text_origin_x = 12'd658; text_origin_y = 12'd346;
    end
    else if (right_data_area && hcount >= 12'd546 && hcount < 12'd586 &&
             vcount >= 12'd382 && vcount < 12'd390) begin
        text_area = 1'b1; text_line = 5'd10;
        text_origin_x = 12'd546; text_origin_y = 12'd382;
    end
    else if (right_data_area && hcount >= 12'd658 && hcount < 12'd706 &&
             vcount >= 12'd382 && vcount < 12'd390) begin
        text_area = 1'b1; text_line = 5'd15;
        text_origin_x = 12'd658; text_origin_y = 12'd382;
    end
    else if (right_data_area && hcount >= 12'd546 && hcount < 12'd594 &&
             vcount >= 12'd406 && vcount < 12'd414) begin
        text_area = 1'b1; text_line = 5'd11;
        text_origin_x = 12'd546; text_origin_y = 12'd406;
    end
    else if (right_data_area && hcount >= 12'd658 && hcount < 12'd770 &&
             vcount >= 12'd406 && vcount < 12'd414) begin
        text_area = 1'b1; text_line = 5'd24;
        text_origin_x = 12'd658; text_origin_y = 12'd406;
    end
    // Footer hints.
    else if (footer_area && hcount >= 12'd32 && hcount < 12'd152 &&
             vcount >= 12'd450 && vcount < 12'd458) begin
        text_area = 1'b1; text_line = 5'd12;
        text_origin_x = 12'd32; text_origin_y = 12'd450;
    end
    else if (footer_area && hcount >= 12'd384 && hcount < 12'd568 &&
             vcount >= 12'd450 && vcount < 12'd458) begin
        text_area = 1'b1; text_line = 5'd13;
        text_origin_x = 12'd384; text_origin_y = 12'd450;
    end
end

wire [11:0] text_rel_x = hcount - text_origin_x;
wire [11:0] text_rel_y = vcount - text_origin_y;
wire [4:0] char_index = text_rel_x[7:3];
wire [2:0] font_x = text_rel_x[2:0];
wire [2:0] font_y = text_rel_y[2:0];

function [7:0] text_char;
    input [4:0] line_id;
    input [4:0] index;
    begin
        text_char = 8'h20;
        case (line_id)
            5'd0: case(index)
                0:text_char="A"; 1:text_char="C"; 2:text_char="G";
                3:text_char="7"; 4:text_char="2"; 5:text_char="0";
                6:text_char=" "; 7:text_char="A"; 8:text_char="I";
                9:text_char=" "; 10:text_char="S"; 11:text_char="E";
                12:text_char="T"; 13:text_char="T"; 14:text_char="I";
                15:text_char="N"; 16:text_char="G"; 17:text_char="S";
                default:text_char=" "; endcase
            5'd1: case(index)
                0:text_char="A"; 1:text_char="I"; 2:text_char=" ";
                3:text_char="C"; 4:text_char="A"; 5:text_char="M";
                6:text_char="E"; 7:text_char="R"; 8:text_char="A";
                default:text_char=" "; endcase
            5'd2: case(index)
                0:text_char="S"; 1:text_char="Y"; 2:text_char="S";
                3:text_char="T"; 4:text_char="E"; 5:text_char="M";
                6:text_char=" "; 7:text_char="S"; 8:text_char="T";
                9:text_char="A"; 10:text_char="T"; 11:text_char="U";
                12:text_char="S"; default:text_char=" "; endcase
            5'd3: case(index)
                0:text_char="B"; 1:text_char="M"; 2:text_char="1";
                3:text_char="6"; 4:text_char="8"; 5:text_char="4";
                6:text_char=" "; 7:text_char="L"; 8:text_char="I";
                9:text_char="N"; 10:text_char="K"; default:text_char=" "; endcase
            5'd4: case(index)
                0:text_char="A"; 1:text_char="I"; 2:text_char=" ";
                3:text_char="M"; 4:text_char="O"; 5:text_char="D";
                6:text_char="E"; default:text_char=" "; endcase
            5'd5: case(index)
                0:text_char="H"; 1:text_char="O"; 2:text_char="S";
                3:text_char="T"; 4:text_char=" "; 5:text_char="C";
                6:text_char="T"; 7:text_char="R"; 8:text_char="L";
                default:text_char=" "; endcase
            5'd6: case(index)
                0:text_char="W"; 1:text_char="A"; 2:text_char="T";
                3:text_char="C"; 4:text_char="H"; 5:text_char="D";
                6:text_char="O"; 7:text_char="G"; default:text_char=" "; endcase
            5'd7: case(index)
                0:text_char="U"; 1:text_char="S"; 2:text_char=" ";
                3:text_char="F"; 4:text_char="R"; 5:text_char="O";
                6:text_char="N"; 7:text_char="T"; default:text_char=" "; endcase
            5'd8: case(index)
                0:text_char="A"; 1:text_char="I"; 2:text_char=" ";
                3:text_char="D"; 4:text_char="A"; 5:text_char="T";
                6:text_char="A"; default:text_char=" "; endcase
            5'd9: case(index)
                0:text_char="A"; 1:text_char="S"; 2:text_char="T";
                3:text_char="R"; 4:text_char="A"; 5:text_char=" ";
                6:text_char="S"; default:text_char=" "; endcase
            5'd10: case(index)
                0:text_char="M"; 1:text_char="O"; 2:text_char="D";
                3:text_char="E"; 4:text_char="L"; default:text_char=" "; endcase
            5'd11: case(index)
                0:text_char="R"; 1:text_char="E"; 2:text_char="S";
                3:text_char="U"; 4:text_char="L"; 5:text_char="T";
                default:text_char=" "; endcase
            5'd12: case(index)
                0:text_char="S"; 1:text_char="W"; 2:text_char="0";
                3:text_char=":"; 4:text_char=" "; 5:text_char="B";
                6:text_char="A"; 7:text_char="S"; 8:text_char="I";
                9:text_char="C"; 10:text_char=" "; 11:text_char="/";
                12:text_char=" "; 13:text_char="A"; 14:text_char="I";
                default:text_char=" "; endcase
            5'd13: case(index)
                0:text_char="A"; 1:text_char="5"; 2:text_char="/";
                3:text_char="A"; 4:text_char="6"; 5:text_char=" ";
                6:text_char="C"; 7:text_char="O"; 8:text_char="N";
                9:text_char="T"; 10:text_char="R"; 11:text_char="O";
                12:text_char="L"; 13:text_char=" "; 14:text_char="U";
                15:text_char="N"; 16:text_char="C"; 17:text_char="H";
                18:text_char="A"; 19:text_char="N"; 20:text_char="G";
                21:text_char="E"; 22:text_char="D"; default:text_char=" "; endcase
            5'd14: case(index)
                0:text_char="O"; 1:text_char="N"; 2:text_char="L";
                3:text_char="I"; 4:text_char="N"; 5:text_char="E";
                default:text_char=" "; endcase
            5'd15: case(index)
                0:text_char="W"; 1:text_char="A"; 2:text_char="I";
                3:text_char="T"; default:text_char=" "; endcase
            5'd16: case(index)
                0:text_char="A"; 1:text_char="C"; 2:text_char="T";
                3:text_char="I"; 4:text_char="V"; 5:text_char="E";
                default:text_char=" "; endcase
            5'd17: case(index)
                0:text_char="I"; 1:text_char="D"; 2:text_char="L";
                3:text_char="E"; default:text_char=" "; endcase
            5'd18: case(index)
                0:text_char="R"; 1:text_char="U"; 2:text_char="N";
                default:text_char=" "; endcase
            5'd19: case(index)
                0:text_char="S"; 1:text_char="T"; 2:text_char="O";
                3:text_char="P"; default:text_char=" "; endcase
            5'd20: case(index)
                0:text_char="O"; 1:text_char="K"; default:text_char=" "; endcase
            5'd21: case(index)
                0:text_char="L"; 1:text_char="O"; 2:text_char="S";
                3:text_char="T"; default:text_char=" "; endcase
            5'd22: case(index)
                0:text_char="O"; 1:text_char="N"; 2:text_char="L";
                3:text_char="I"; 4:text_char="N"; 5:text_char="E";
                default:text_char=" "; endcase
            5'd23: case(index)
                0:text_char="H"; 1:text_char="O"; 2:text_char="S";
                3:text_char="T"; 4:text_char=" "; 5:text_char="D";
                6:text_char="A"; 7:text_char="T"; 8:text_char="A";
                default:text_char=" "; endcase
            5'd24: case(index)
                0:text_char="W"; 1:text_char="A"; 2:text_char="I";
                3:text_char="T"; 4:text_char=" "; 5:text_char="H";
                6:text_char="O"; 7:text_char="S"; 8:text_char="T";
                9:text_char=" "; 10:text_char="D"; 11:text_char="A";
                12:text_char="T"; 13:text_char="A"; default:text_char=" "; endcase
            5'd25: case(index)
                0:text_char="R"; 1:text_char="E"; 2:text_char="C";
                3:text_char="O"; 4:text_char="V"; 5:text_char="E";
                6:text_char="R"; default:text_char=" "; endcase
            default: text_char = 8'h20;
        endcase
    end
endfunction

wire [7:0] ascii_char = text_char(text_line, char_index);

reg [63:0] glyph;
always @(*) begin
    case (ascii_char)
        "0": glyph = 64'h3C_42_46_4A_52_62_3C_00;
        "1": glyph = 64'h08_18_28_08_08_08_3E_00;
        "2": glyph = 64'h3C_42_02_0C_30_40_7E_00;
        "4": glyph = 64'h0C_14_24_44_7E_04_04_00;
        "5": glyph = 64'h7E_40_7C_02_02_42_3C_00;
        "6": glyph = 64'h1C_20_40_7C_42_42_3C_00;
        "7": glyph = 64'h7E_02_04_08_10_10_10_00;
        "8": glyph = 64'h3C_42_42_3C_42_42_3C_00;
        "A": glyph = 64'h18_24_42_42_7E_42_42_00;
        "B": glyph = 64'h7C_42_42_7C_42_42_7C_00;
        "C": glyph = 64'h3C_42_40_40_40_42_3C_00;
        "D": glyph = 64'h78_44_42_42_42_44_78_00;
        "E": glyph = 64'h7E_40_40_7C_40_40_7E_00;
        "F": glyph = 64'h7E_40_40_7C_40_40_40_00;
        "G": glyph = 64'h3C_42_40_4E_42_42_3C_00;
        "H": glyph = 64'h42_42_42_7E_42_42_42_00;
        "I": glyph = 64'h3E_08_08_08_08_08_3E_00;
        "K": glyph = 64'h42_44_48_70_48_44_42_00;
        "L": glyph = 64'h40_40_40_40_40_40_7E_00;
        "M": glyph = 64'h42_66_5A_5A_42_42_42_00;
        "N": glyph = 64'h42_62_52_4A_46_42_42_00;
        "O": glyph = 64'h3C_42_42_42_42_42_3C_00;
        "P": glyph = 64'h7C_42_42_7C_40_40_40_00;
        "R": glyph = 64'h7C_42_42_7C_48_44_42_00;
        "S": glyph = 64'h3C_42_40_3C_02_42_3C_00;
        "T": glyph = 64'h7F_08_08_08_08_08_08_00;
        "U": glyph = 64'h42_42_42_42_42_42_3C_00;
        "V": glyph = 64'h42_42_42_42_42_24_18_00;
        "W": glyph = 64'h42_42_42_5A_5A_66_42_00;
        "Y": glyph = 64'h42_42_24_18_08_08_08_00;
        ":": glyph = 64'h00_18_18_00_18_18_00_00;
        "/": glyph = 64'h02_04_08_10_20_40_00_00;
        "-": glyph = 64'h00_00_00_7E_00_00_00_00;
        default: glyph = 64'd0;
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

wire text_pixel = text_area && glyph_row[3'd7 - font_x];
wire value_text = (text_line >= 5'd14);
wire [23:0] text_color =
    value_text ?
        ((text_line == 5'd14 || text_line == 5'd16 ||
          text_line == 5'd18 || text_line == 5'd20 ||
          text_line == 5'd22 || text_line == 5'd23) ?
          24'h50E6A0 : 24'hFFD36B) :
        24'h70DDED;

wire status_dot = page_on &&
    hcount >= 12'd766 && hcount <= 12'd775 &&
    ((vcount >= 12'd106 && vcount <= 12'd115) ||
     (vcount >= 12'd144 && vcount <= 12'd153) ||
     (vcount >= 12'd182 && vcount <= 12'd191) ||
     (vcount >= 12'd220 && vcount <= 12'd229) ||
     (vcount >= 12'd258 && vcount <= 12'd267));

wire dot_green =
    ((vcount >= 12'd106 && vcount <= 12'd115) && host_link_ok) ||
    ((vcount >= 12'd144 && vcount <= 12'd153) && ai_mode_sync) ||
    ((vcount >= 12'd182 && vcount <= 12'd191) && host_run_ok) ||
    ((vcount >= 12'd220 && vcount <= 12'd229) && watchdog_sync) ||
    ((vcount >= 12'd258 && vcount <= 12'd267) && ultra_sync);

assign osd_active = page_on &&
                    (header_area || right_status_area || right_data_area ||
                     footer_area || camera_border || camera_label_area);

assign osd_rgb =
    border_pixel ? 24'h27CFE8 :
    text_pixel   ? text_color :
    status_dot  ? (dot_green ? 24'h35E58B : 24'hE9B84F) :
    divider_pixel ? 24'h174B61 :
    header_area ? 24'h061B2A :
    right_status_area ? 24'h041725 :
    right_data_area ? 24'h041725 :
    footer_area ? 24'h061B2A :
    camera_border ? 24'h27CFE8 :
                    24'h000000;

endmodule
