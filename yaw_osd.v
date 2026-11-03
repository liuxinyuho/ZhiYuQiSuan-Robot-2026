
`timescale 1ns / 1ps

//===============================================
// ACG720 60K
// YAW OSD + MPU6500 CALIBRATION COUNTDOWN
//
// Before IMU ready : CAL:--S
// During calibration: CAL:11S ... CAL:01S
// Calibration done : YAW:+123.45
//
// Gyro calibration: 1024 samples
// Nominal sampling: 100 Hz
//===============================================

module yaw_osd (
    input wire sys_clk,
    input wire pix_clk,
    input wire rst_n,

    input wire signed [31:0] yaw_cdeg,

    input wire imu_init_done,
    input wire gyro_calib_done,
    input wire imu_data_valid,

    input wire [11:0] hcount,
    input wire [11:0] vcount,
    input wire video_active,

    output wire osd_active,
    output wire [23:0] osd_rgb
);

//===============================================
// 1. Calibration sample counter
//===============================================

reg [10:0] calib_sample_count;

always @(posedge sys_clk or negedge rst_n) begin

    if (!rst_n)
        calib_sample_count <= 11'd0;

    else if (!imu_init_done)
        calib_sample_count <= 11'd0;

    else if (!gyro_calib_done &&
             imu_data_valid &&
             calib_sample_count < 11'd1024)

        calib_sample_count <=
            calib_sample_count + 11'd1;

end

//===============================================
// 2. Calibration remaining seconds
//
// 1024 samples at nominal 100Hz = 10.24s
// Count advances only on valid IMU samples.
//===============================================

reg [3:0] secs_left;

always @(*) begin

    if (gyro_calib_done)
        secs_left = 4'd0;

    else if (calib_sample_count < 11'd24)
        secs_left = 4'd11;

    else if (calib_sample_count < 11'd124)
        secs_left = 4'd10;

    else if (calib_sample_count < 11'd224)
        secs_left = 4'd9;

    else if (calib_sample_count < 11'd324)
        secs_left = 4'd8;

    else if (calib_sample_count < 11'd424)
        secs_left = 4'd7;

    else if (calib_sample_count < 11'd524)
        secs_left = 4'd6;

    else if (calib_sample_count < 11'd624)
        secs_left = 4'd5;

    else if (calib_sample_count < 11'd724)
        secs_left = 4'd4;

    else if (calib_sample_count < 11'd824)
        secs_left = 4'd3;

    else if (calib_sample_count < 11'd924)
        secs_left = 4'd2;

    else
        secs_left = 4'd1;

end

//===============================================
// 3. YAW CDC handshake registers
//===============================================

reg req_toggle_pix;

reg ack_pix_s1;
reg ack_pix_s2;
reg ack_seen_pix;

reg [19:0] bcd_pix;
reg negative_pix;

reg imu_ready_pix;
reg calib_done_pix;
reg [3:0] secs_pix;

reg req_sys_s1;
reg req_sys_s2;
reg ack_toggle_sys;

reg negative_sys;

reg [19:0] bcd_sys;
reg [15:0] bin_sys;

reg [4:0] bit_count;
reg converting;

reg imu_ready_sys;
reg calib_done_sys;
reg [3:0] secs_sys;

//===============================================
// 4. YAW magnitude
//===============================================

wire [31:0] yaw_magnitude;

assign yaw_magnitude =
    yaw_cdeg[31] ?
    (~yaw_cdeg + 32'd1) :
    yaw_cdeg;

wire [15:0] yaw_limited;

assign yaw_limited =
    (yaw_magnitude > 32'd18000) ?
    16'd18000 :
    yaw_magnitude[15:0];

//===============================================
// 5. Binary to BCD
//===============================================

reg [19:0] bcd_adjusted;
integer nibble;

always @(*) begin

    bcd_adjusted = bcd_sys;

    for (nibble = 0; nibble < 5;
         nibble = nibble + 1) begin

        if (bcd_adjusted[nibble*4 +: 4] >= 4'd5)

            bcd_adjusted[nibble*4 +: 4] =
                bcd_adjusted[nibble*4 +: 4]
                + 4'd3;

    end

end

//===============================================
// 6. System clock domain
//===============================================

always @(posedge sys_clk or negedge rst_n) begin

    if (!rst_n) begin

        req_sys_s1 <= 1'b0;
        req_sys_s2 <= 1'b0;

        ack_toggle_sys <= 1'b0;

        negative_sys <= 1'b0;

        bcd_sys <= 20'd0;
        bin_sys <= 16'd0;

        bit_count <= 5'd0;
        converting <= 1'b0;

        imu_ready_sys <= 1'b0;
        calib_done_sys <= 1'b0;
        secs_sys <= 4'd11;

    end else begin

        req_sys_s1 <= req_toggle_pix;
        req_sys_s2 <= req_sys_s1;

        if (converting) begin

            bcd_sys <= {
                bcd_adjusted[18:0],
                bin_sys[15]
            };

            bin_sys <= {
                bin_sys[14:0],
                1'b0
            };

            if (bit_count == 5'd15) begin

                converting <= 1'b0;

                ack_toggle_sys <= req_sys_s2;

            end else begin

                bit_count <= bit_count + 5'd1;

            end

        end else if (
            req_sys_s2 != ack_toggle_sys
        ) begin

            negative_sys <= yaw_cdeg[31];

            bcd_sys <= 20'd0;
            bin_sys <= yaw_limited;

            bit_count <= 5'd0;
            converting <= 1'b1;

            // Capture calibration status
            // with the same stable-data handshake.

            imu_ready_sys <= imu_init_done;

            calib_done_sys <= gyro_calib_done;

            secs_sys <= secs_left;

        end

    end

end

//===============================================
// 7. HDMI pixel clock domain
//===============================================

always @(posedge pix_clk or negedge rst_n) begin

    if (!rst_n) begin

        req_toggle_pix <= 1'b0;

        ack_pix_s1 <= 1'b0;
        ack_pix_s2 <= 1'b0;
        ack_seen_pix <= 1'b0;

        bcd_pix <= 20'd0;
        negative_pix <= 1'b0;

        imu_ready_pix <= 1'b0;
        calib_done_pix <= 1'b0;
        secs_pix <= 4'd11;

    end else begin

        ack_pix_s1 <= ack_toggle_sys;
        ack_pix_s2 <= ack_pix_s1;

        // Preserve the working frame update point.

        if (video_active &&
            hcount == 12'd1 &&
            vcount == 12'd0 &&
            req_toggle_pix == ack_pix_s2) begin

            req_toggle_pix <= ~req_toggle_pix;

        end

        if (ack_pix_s2 != ack_seen_pix) begin

            bcd_pix <= bcd_sys;
            negative_pix <= negative_sys;

            imu_ready_pix <= imu_ready_sys;
            calib_done_pix <= calib_done_sys;
            secs_pix <= secs_sys;

            ack_seen_pix <= ack_pix_s2;

        end

    end

end

//===============================================
// 8. OSD panel position
//===============================================

wire panel;

assign panel =
    video_active &&
    hcount >= 12'd592 &&
    hcount < 12'd792 &&
    vcount >= 12'd12 &&
    vcount < 12'd56;

wire text_area;

assign text_area =
    video_active &&
    hcount >= 12'd608 &&
    hcount < 12'd784 &&
    vcount >= 12'd24 &&
    vcount < 12'd40;

wire [11:0] dx;
wire [11:0] dy;

assign dx = hcount - 12'd608;
assign dy = vcount - 12'd24;

wire [3:0] char_index;

assign char_index = dx[7:4];

wire [2:0] font_x;
wire [2:0] font_y;

assign font_x = dx[3:1];
assign font_y = dy[3:1];

//===============================================
// 9. Seconds digit conversion
//===============================================

wire [3:0] sec_tens;
wire [3:0] sec_ones;

assign sec_tens =
    (secs_pix >= 4'd10) ?
    4'd1 : 4'd0;

assign sec_ones =
    (secs_pix >= 4'd10) ?
    (secs_pix - 4'd10) :
    secs_pix;

//===============================================
// 10. Select display characters
//
// Before initialization: CAL:--S
// During calibration  : CAL:11S
// After calibration   : YAW:+123.45
//===============================================

reg [4:0] symbol;

always @(*) begin

    if (!calib_done_pix) begin

        case (char_index)

            4'd0: symbol = 5'd18; // C
            4'd1: symbol = 5'd11; // A
            4'd2: symbol = 5'd19; // L
            4'd3: symbol = 5'd13; // :

            4'd4: symbol =
                imu_ready_pix ?
                {1'b0,sec_tens} :
                5'd15;

            4'd5: symbol =
                imu_ready_pix ?
                {1'b0,sec_ones} :
                5'd15;

            4'd6: symbol = 5'd20; // S

            default: symbol = 5'd17;

        endcase

    end else begin

        case (char_index)

            4'd0: symbol = 5'd10; // Y
            4'd1: symbol = 5'd11; // A
            4'd2: symbol = 5'd12; // W
            4'd3: symbol = 5'd13; // :

            4'd4: symbol =
                negative_pix ? 5'd15 : 5'd14;

            4'd5: symbol =
                {1'b0,bcd_pix[19:16]};

            4'd6: symbol =
                {1'b0,bcd_pix[15:12]};

            4'd7: symbol =
                {1'b0,bcd_pix[11:8]};

            4'd8: symbol = 5'd16;

            4'd9: symbol =
                {1'b0,bcd_pix[7:4]};

            4'd10: symbol =
                {1'b0,bcd_pix[3:0]};

            default: symbol = 5'd17;

        endcase

    end

end

//===============================================
// 11. Character ROM
//===============================================

reg [63:0] glyph;

always @(*) begin

    case (symbol)

        // 0 - 9
        5'd0:  glyph = 64'h70_88_98_A8_C8_88_70_00;
        5'd1:  glyph = 64'h20_60_20_20_20_20_70_00;
        5'd2:  glyph = 64'h70_88_08_10_20_40_F8_00;
        5'd3:  glyph = 64'h70_88_08_30_08_88_70_00;
        5'd4:  glyph = 64'h10_30_50_90_F8_10_10_00;
        5'd5:  glyph = 64'hF8_80_F0_08_08_88_70_00;
        5'd6:  glyph = 64'h30_40_80_F0_88_88_70_00;
        5'd7:  glyph = 64'hF8_08_10_20_40_40_40_00;
        5'd8:  glyph = 64'h70_88_88_70_88_88_70_00;
        5'd9:  glyph = 64'h70_88_88_78_08_10_60_00;

        // Y A W
        5'd10: glyph = 64'h88_88_50_20_20_20_20_00;
        5'd11: glyph = 64'h70_88_88_F8_88_88_88_00;
        5'd12: glyph = 64'h88_88_88_A8_A8_A8_50_00;

        // : + - .
        5'd13: glyph = 64'h00_20_20_00_20_20_00_00;
        5'd14: glyph = 64'h00_20_20_F8_20_20_00_00;
        5'd15: glyph = 64'h00_00_00_F8_00_00_00_00;
        5'd16: glyph = 64'h00_00_00_00_00_20_20_00;

        // C L S
        5'd18: glyph = 64'h70_88_80_80_80_88_70_00;
        5'd19: glyph = 64'h80_80_80_80_80_80_F8_00;
        5'd20: glyph = 64'h70_88_80_70_08_88_70_00;

        default:
            glyph = 64'h00_00_00_00_00_00_00_00;

    endcase

end

//===============================================
// 12. Font row
//===============================================

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

//===============================================
// 13. Output OSD pixels
//===============================================

wire text_pixel;

assign text_pixel =
    text_area &&
    glyph_row[3'd7 - font_x];

wire border_pixel;

assign border_pixel =
    panel &&
    (
        hcount == 12'd592 ||
        hcount == 12'd791 ||
        vcount == 12'd12 ||
        vcount == 12'd55
    );

assign osd_active = panel;

assign osd_rgb =
    text_pixel ?
    (
        calib_done_pix ?
        24'hFFFFFF :
        24'hFFD476
    ) :
    border_pixel ? 24'h4FD9EE :
                   24'h0B2233;

endmodule
