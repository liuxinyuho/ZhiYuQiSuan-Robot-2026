
`timescale 1ns / 1ps

module hdmi_color #(
    parameter DISP_WIDTH  = 800,
    parameter DISP_HEIGHT = 480,
    parameter VIDEO_TEST  = 1'b1
)(
    input clk,
    input rst_n,

    // Display page selector and persistent host status.
    input wire ui_mode_ai,
    input wire status_host_mode,
    input wire status_host_ai_mode,
    input wire status_host_control_active,
    input wire status_host_watchdog_ok,
    input wire status_host_stop,
    input wire ultra_recovering,

    output [3:0] led,

    output camera_xclk,
    input camera_pclk,
    input camera_vsync,
    input camera_href,
    input [7:0] camera_data,

    output camera_rst_n,
    output camera_pwdn,
    output camera_sclk,
    inout camera_sdat,

    output tmds_clk_p,
    output tmds_clk_n,
    output [2:0] tmds_data_p,
    output [2:0] tmds_data_n,

    output snapshot_pixel_valid,
    output [15:0] snapshot_pixel_rgb565,
    output [10:0] snapshot_pixel_x,
    output [10:0] snapshot_pixel_y,

    input wire signed [31:0] yaw_cdeg,
    input wire imu_init_done,
    input wire gyro_calib_done,
    input wire imu_data_valid,

    input wire [15:0] tof_left_mm,
    input wire [15:0] tof_front_mm,
    input wire [15:0] tof_right_mm,
    input wire [15:0] tof_rear_mm,

    input wire tof_left_ok,
    input wire tof_front_ok,
    input wire tof_right_ok,
    input wire tof_rear_ok,
    input wire tof_left_recovering,
    input wire tof_front_recovering,
    input wire tof_right_recovering,
    input wire tof_rear_recovering,
    input wire ultra_ok,

    input wire status_calib_done,
    input wire status_obstacle_active,
    input wire status_obstacle_error,

    input wire status_waypoint_active,
    input wire status_waypoint_done,
    input wire [2:0] status_waypoint_index,

    input wire status_nav_restore_active,
    input wire status_nav_waypoint_pause,
    input wire status_nav_avoid_cycle_active,

    input wire status_fault_stop,
    input wire status_front_stop,
    input wire status_front_slow,
    input wire status_side_stop,

    input wire signed [31:0] motion_speed_a,
    input wire signed [31:0] motion_speed_b,
    input wire signed [31:0] motion_speed_c,
    input wire signed [31:0] motion_speed_d,

    input wire signed [31:0] motion_x_mm,
    input wire signed [31:0] motion_y_mm
);

//====================================================
// 1. HDMI PLL
//====================================================

wire clk_vga;
wire clk_vgax5;
wire pll_lock;

Gowin_PLL u_hdmi_pll (
    .clkin   (clk),
    .clkout0 (clk_vga),
    .clkout1 (clk_vgax5),
    .lock    (pll_lock),
    .mdclk   (clk)
);

//====================================================
// 2. Camera clock
//====================================================

reg cam_clk25;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cam_clk25 <= 1'b0;
    else
        cam_clk25 <= ~cam_clk25;
end

assign camera_xclk = cam_clk25;

//====================================================
// 3. OV5640 initialization
//====================================================

wire camera_init_done;

camera_init #(
    .SYS_CLOCK       (50_000_000),
    .SCL_CLOCK       (400_000),
    .CAMERA_TYPE     ("ov5640"),
    .IMAGE_TYPE      (0),
    .IMAGE_WIDTH     (640),
    .IMAGE_HEIGHT    (480),
    .IMAGE_FLIP_EN   (0),
    .IMAGE_MIRROR_EN (0)
) u_camera_init (
    .Clk          (clk),
    .Rst_n        (rst_n),
    .Init_Done    (camera_init_done),
    .camera_rst_n (camera_rst_n),
    .camera_pwdn  (camera_pwdn),
    .i2c_sclk     (camera_sclk),
    .i2c_sdat     (camera_sdat)
);

//====================================================
// 4. DVP capture
//====================================================

wire pixel_valid;
wire [15:0] pixel_rgb565;
wire [10:0] pixel_x;
wire [10:0] pixel_y;

DVP_Capture u_dvp_capture (
    .Rst_n      (rst_n),
    .PCLK       (camera_pclk),
    .Vsync      (camera_vsync),
    .Href       (camera_href),
    .Data       (camera_data),
    .ImageState (),
    .DataValid  (pixel_valid),
    .DataPixel  (pixel_rgb565),
    .DataHs     (),
    .DataVs     (),
    .Xaddr      (pixel_x),
    .Yaddr      (pixel_y)
);

assign snapshot_pixel_valid  = pixel_valid;
assign snapshot_pixel_rgb565 = pixel_rgb565;
assign snapshot_pixel_x      = pixel_x;
assign snapshot_pixel_y      = pixel_y;

//====================================================
// 5. Resolution detection
//====================================================

reg href_d;
reg vsync_d;

reg [11:0] byte_count;
reg [10:0] line_count;

reg width_ok_cam;
reg height_ok_cam;

always @(posedge camera_pclk or negedge rst_n) begin

    if (!rst_n) begin
        href_d        <= 1'b0;
        vsync_d       <= 1'b0;
        byte_count    <= 12'd0;
        line_count    <= 11'd0;
        width_ok_cam  <= 1'b0;
        height_ok_cam <= 1'b0;

    end else begin

        href_d  <= camera_href;
        vsync_d <= camera_vsync;

        if (camera_href && !href_d) begin
            byte_count <= 12'd1;
            line_count <= line_count + 1'b1;
        end else if (camera_href) begin
            byte_count <= byte_count + 1'b1;
        end

        if (!camera_href && href_d)
            width_ok_cam <= (byte_count == 12'd1280);

        if (camera_vsync && !vsync_d) begin
            height_ok_cam <= (line_count == 11'd480);
            line_count <= 11'd0;
        end

    end
end

wire width_ok  = width_ok_cam;
wire height_ok = height_ok_cam;

//====================================================
// 6. Camera pixel capture
//====================================================

wire pixel_we;

assign pixel_we =
    pixel_valid &&
    pixel_x[0] &&
    pixel_y[0] &&
    (pixel_x >= 11'd1) &&
    (pixel_x <= 11'd639) &&
    (pixel_y >= 11'd1) &&
    (pixel_y <= 11'd479);

wire [8:0] wr_x;
wire [7:0] wr_y;
wire [16:0] wr_addr;

assign wr_x = (pixel_x - 11'd1) >> 1;
assign wr_y = (pixel_y - 11'd1) >> 1;

assign wr_addr =
    ({9'd0, wr_y} << 8) +
    ({9'd0, wr_y} << 6) +
    {8'd0, wr_x};

//====================================================
// 7. Camera activity detection
//====================================================

reg pixel_seen;
reg pixel_changed;
reg [15:0] last_pixel;

always @(posedge camera_pclk or negedge rst_n) begin

    if (!rst_n) begin
        pixel_seen <= 1'b0;
        pixel_changed <= 1'b0;
        last_pixel <= 16'd0;

    end else if (pixel_valid) begin

        pixel_seen <= 1'b1;

        if (pixel_rgb565 != last_pixel)
            pixel_changed <= 1'b1;

        last_pixel <= pixel_rgb565;
    end

end

//====================================================
// 8. BSRAM write pipeline
//====================================================

reg [8:0] wr_x_s1;
reg [7:0] wr_y_s1;
reg [15:0] wr_data_s1;
reg wr_en_s1;

reg [16:0] wr_addr_s2;
reg [15:0] wr_data_s2;
reg wr_en_s2;

always @(posedge camera_pclk or negedge rst_n) begin

    if (!rst_n) begin
        wr_x_s1 <= 9'd0;
        wr_y_s1 <= 8'd0;
        wr_data_s1 <= 16'd0;
        wr_en_s1 <= 1'b0;

        wr_addr_s2 <= 17'd0;
        wr_data_s2 <= 16'd0;
        wr_en_s2 <= 1'b0;

    end else begin

        wr_x_s1 <= wr_x;
        wr_y_s1 <= wr_y;
        wr_data_s1 <= pixel_rgb565;
        wr_en_s1 <= pixel_we;

        wr_addr_s2 <=
            ({9'd0, wr_y_s1} << 8) +
            ({9'd0, wr_y_s1} << 6) +
            {8'd0, wr_x_s1};

        wr_data_s2 <= wr_data_s1;
        wr_en_s2 <= wr_en_s1;

    end
end

//====================================================
// 9. Camera status CDC
//====================================================

reg init_sync1;
reg init_sync2;

reg seen_sync1;
reg seen_sync2;

reg width_sync1;
reg width_sync2;

reg height_sync1;
reg height_sync2;

always @(posedge clk_vga or negedge rst_n) begin

    if (!rst_n) begin

        init_sync1 <= 1'b0;
        init_sync2 <= 1'b0;

        seen_sync1 <= 1'b0;
        seen_sync2 <= 1'b0;

        width_sync1 <= 1'b0;
        width_sync2 <= 1'b0;

        height_sync1 <= 1'b0;
        height_sync2 <= 1'b0;

    end else begin

        init_sync1 <= camera_init_done;
        init_sync2 <= init_sync1;

        seen_sync1 <= pixel_seen;
        seen_sync2 <= seen_sync1;

        width_sync1 <= width_ok;
        width_sync2 <= width_sync1;

        height_sync1 <= height_ok;
        height_sync2 <= height_sync1;

    end
end

//====================================================
// 10. HDMI timing signals
//====================================================

wire [11:0] hcount;
wire [11:0] vcount;

wire disp_data_req;

wire video_hs;
wire video_vs;
wire video_de;

wire [7:0] video_r;
wire [7:0] video_g;
wire [7:0] video_b;

// SW0 originates in the 50MHz system-clock domain. Synchronize it into
// the HDMI pixel-clock domain before changing the overlay page.
reg ui_mode_ai_meta;
reg ui_mode_ai_pix;
always @(posedge clk_vga or negedge rst_n) begin
    if (!rst_n) begin
        ui_mode_ai_meta <= 1'b0;
        ui_mode_ai_pix  <= 1'b0;
    end else begin
        ui_mode_ai_meta <= ui_mode_ai;
        ui_mode_ai_pix  <= ui_mode_ai_meta;
    end
end
wire basic_video_active = disp_data_req && !ui_mode_ai_pix;
wire ai_video_active    = disp_data_req &&  ui_mode_ai_pix;

//====================================================
// 11. Camera display window: 640x480, centered on 800x480 HDMI
//
// OV5640 capture remains 640x480.
// The existing BSRAM stores every second pixel/line, so the
// framebuffer is 320x240 RGB565. Read it at 2x scale into a
// 640x480 window (x=80..719, y=0..479).
// This restores the complete 4:3 camera frame and avoids the
// previous center crop + 800x480 enlargement.
//====================================================

wire in_picture;

assign in_picture =
    disp_data_req &&
    (hcount >= 12'd80) &&
    (hcount <  12'd720) &&
    (vcount <  12'd480);

wire [8:0] rd_x;
wire [7:0] rd_y;
wire [16:0] rd_addr;

// Screen X: 80..719 -> framebuffer X: 0..319.
assign rd_x = (hcount - 12'd80) >> 1;

// Screen Y: 0..479 -> framebuffer Y: 0..239.
assign rd_y = vcount[8:1];

// Address = Y * 320 + X; maximum address = 76799.
assign rd_addr =
    ({9'd0, rd_y} << 8) +
    ({9'd0, rd_y} << 6) +
    {8'd0, rd_x};

//====================================================
// 12. BSRAM framebuffer
//====================================================

wire [15:0] rd_rgb565;

frame_buffer u_frame (
    .wr_clk  (camera_pclk),
    .wr_en   (wr_en_s2),
    .wr_addr (wr_addr_s2),
    .wr_data (wr_data_s2),

    .rd_clk  (clk_vga),
    .rd_en   (in_picture && seen_sync2),
    .rd_addr (rd_addr),
    .rd_data (rd_rgb565)
);

reg region_d;

always @(posedge clk_vga or negedge rst_n) begin
    if (!rst_n)
        region_d <= 1'b0;
    else
        region_d <= in_picture;
end

//====================================================
// 13. RGB565 to RGB888
//====================================================

wire [23:0] rgb888;

assign rgb888 = {
    rd_rgb565[15:11], 3'b000,
    rd_rgb565[10:5],  2'b00,
    rd_rgb565[4:0],   3'b000
};

//====================================================
// 14. Camera resolution indicators
//====================================================

wire width_indicator;
wire height_indicator;

assign width_indicator =
    disp_data_req &&
    (hcount < 12'd32) &&
    (vcount < 12'd24);

assign height_indicator =
    disp_data_req &&
    (hcount >= 12'd38) &&
    (hcount < 12'd70) &&
    (vcount < 12'd24);

//====================================================
// 15. YAW and calibration OSD
//====================================================

wire osd_active;
wire [23:0] osd_rgb;

yaw_osd u_yaw_osd (
    .sys_clk        (clk),
    .pix_clk        (clk_vga),
    .rst_n          (rst_n),

    .yaw_cdeg       (yaw_cdeg),

    .imu_init_done  (imu_init_done),
    .gyro_calib_done(gyro_calib_done),
    .imu_data_valid (imu_data_valid),

    .hcount         (hcount),
    .vcount         (vcount),
    .video_active   (basic_video_active),

    .osd_active     (osd_active),
    .osd_rgb        (osd_rgb)
);

//====================================================
// 16. ToF and ultrasonic OSD
//====================================================

wire dist_osd_active;
wire [23:0] dist_osd_rgb;

distance_osd u_distance_osd (
    .sys_clk      (clk),
    .pix_clk      (clk_vga),
    .rst_n        (rst_n),

    .left_mm      (tof_left_mm),
    .front_mm     (tof_front_mm),
    .right_mm     (tof_right_mm),
    .rear_mm      (tof_rear_mm),

    .left_ok      (tof_left_ok),
    .front_ok     (tof_front_ok),
    .right_ok     (tof_right_ok),
    .rear_ok      (tof_rear_ok),

    .hcount       (hcount),
    .vcount       (vcount),
    .video_active (basic_video_active),

    .osd_active   (dist_osd_active),
    .osd_rgb      (dist_osd_rgb)
);

//====================================================
// 17. Robot status OSD
//====================================================

wire status_osd_active;
wire [23:0] status_osd_rgb;

robot_status_osd u_robot_status_osd (
    .sys_clk  (clk),
    .pix_clk  (clk_vga),
    .rst_n    (rst_n),

    .calib_done      (status_calib_done),
    .obstacle_active (status_obstacle_active),
    .obstacle_error  (status_obstacle_error),

    .waypoint_active (status_waypoint_active),
    .waypoint_done   (status_waypoint_done),
    .waypoint_index  (status_waypoint_index),

    .nav_restore_active     (status_nav_restore_active),
    .nav_waypoint_pause     (status_nav_waypoint_pause),
    .nav_avoid_cycle_active (status_nav_avoid_cycle_active),

    .fault_stop (status_fault_stop),

    .tof_left_ok  (tof_left_ok),
    .tof_front_ok (tof_front_ok),
    .tof_right_ok (tof_right_ok),
    .ultra_ok     (ultra_ok),

    .front_stop (status_front_stop),
    .front_slow (status_front_slow),
    .side_stop  (status_side_stop),

    .hcount       (hcount),
    .vcount       (vcount),
    .video_active (basic_video_active),

    .osd_active (status_osd_active),
    .osd_rgb    (status_osd_rgb)
);

//====================================================
// 18. Motion OSD
//====================================================

wire motion_osd_active;
wire [23:0] motion_osd_rgb;

motion_osd u_motion_osd (
    .sys_clk (clk),
    .pix_clk (clk_vga),
    .rst_n   (rst_n),

    .speed_a (motion_speed_a),
    .speed_b (motion_speed_b),
    .speed_c (motion_speed_c),
    .speed_d (motion_speed_d),

    .odom_x_mm (motion_x_mm),
    .odom_y_mm (motion_y_mm),

    .hcount       (hcount),
    .vcount       (vcount),
    .video_active (basic_video_active),

    .osd_active (motion_osd_active),
    .osd_rgb    (motion_osd_rgb)
);

//====================================================
// 19. NEW HUD TOP TITLE
//
// Center top: ACG720 VISION + LIVE
// Does not modify YAW/calibration handshake
//====================================================

wire hud_title_active;
wire [23:0] hud_title_rgb;

hud_title_osd u_hud_title (
    .hcount       (hcount),
    .vcount       (vcount),
    .video_active (basic_video_active),

    .osd_active (hud_title_active),
    .osd_rgb    (hud_title_rgb)
);

//====================================================
// 20. HDMI pixel compositing
//====================================================

//================================================
// ACG720 - Four Direction ToF Radar OSD
// L / F / R / U(rear) are four independent ToF sensors.
// HC-SR04 remains in the original front-assist/safety logic only.
//================================================

wire radar_osd_active;
wire [23:0] radar_osd_rgb;

radar_osd u_radar_osd (
    .sys_clk      (clk),
    .pix_clk      (clk_vga),
    .rst_n        (rst_n),

    .left_mm      (tof_left_mm),
    .front_mm     (tof_front_mm),
    .right_mm     (tof_right_mm),
    .rear_mm      (tof_rear_mm),

    .left_ok      (tof_left_ok),
    .front_ok     (tof_front_ok),
    .right_ok     (tof_right_ok),
    .rear_ok      (tof_rear_ok),

    // Recovery status belongs to radar_osd, not distance_osd.
    .left_recovering  (tof_left_recovering),
    .front_recovering (tof_front_recovering),
    .right_recovering (tof_right_recovering),
    .rear_recovering  (tof_rear_recovering),

    .hcount       (hcount),
    .vcount       (vcount),
    .video_active (basic_video_active),

    .osd_active   (radar_osd_active),
    .osd_rgb      (radar_osd_rgb)
);

wire ai_osd_active;
wire [23:0] ai_osd_rgb;

ai_settings_osd u_ai_settings_osd (
    .pix_clk              (clk_vga),
    .rst_n                (rst_n),
    .video_active         (ai_video_active),
    .page_enable          (ui_mode_ai_pix),
    .host_mode_active     (status_host_mode),
    .host_ai_mode         (status_host_ai_mode),
    .host_control_active  (status_host_control_active),
    .host_watchdog_ok     (status_host_watchdog_ok),
    .host_stop_req        (status_host_stop),
    .ultrasonic_ok        (ultra_ok),
    .ultrasonic_recovering(ultra_recovering),
    .hcount               (hcount),
    .vcount               (vcount),
    .osd_active           (ai_osd_active),
    .osd_rgb              (ai_osd_rgb)
);

wire [23:0] disp_pixel;

assign disp_pixel =
    !VIDEO_TEST        ? 24'h00FF00 :
    !init_sync2        ? 24'hFF00FF :
    !seen_sync2        ? 24'hFFFF00 :

    ai_osd_active      ? ai_osd_rgb :
    hud_title_active   ? hud_title_rgb :
    osd_active         ? osd_rgb :
    dist_osd_active    ? dist_osd_rgb :
    radar_osd_active   ? radar_osd_rgb :
    motion_osd_active  ? motion_osd_rgb :
    status_osd_active  ? status_osd_rgb :

    !region_d          ? 24'h000000 :
                         rgb888;


//====================================================
// 21. HDMI display driver
//====================================================

disp_driver u_disp (
    .ClkDisp (clk_vga),
    .Rst_n   (rst_n),
    .Data    (disp_pixel),

    .DataReq (disp_data_req),
    .H_Addr  (hcount),
    .V_Addr  (vcount),

    .Disp_HS    (video_hs),
    .Disp_VS    (video_vs),
    .Disp_Red   (video_r),
    .Disp_Green (video_g),
    .Disp_Blue  (video_b),

    .Disp_DE   (video_de),
    .Disp_PCLK ()
);

//====================================================
// 22. HDMI DVI encoder
//====================================================

dvi_encoder u_dvi_encoder (
    .pixelclk   (clk_vga),
    .pixelclk5x (clk_vgax5),
    .rst_n      (rst_n),

    .blue_din  (video_b),
    .green_din (video_g),
    .red_din   (video_r),

    .hsync (video_hs),
    .vsync (video_vs),
    .de    (video_de),

    .tmds_clk_p  (tmds_clk_p),
    .tmds_clk_n  (tmds_clk_n),
    .tmds_data_p (tmds_data_p),
    .tmds_data_n (tmds_data_n)
);

//====================================================
// 23. LEDs
//====================================================

assign led[3] = pll_lock;
assign led[2] = camera_init_done;
assign led[1] = pixel_seen;
assign led[0] = pixel_changed;

endmodule

//====================================================
// NEW TOP TITLE SUBMODULE
//
// Layout:
// Sensor OSD: left
// HUD TITLE : center
// YAW OSD   : right
//
// Resolution: 800 x 480
//====================================================

module hud_title_osd (
    input wire [11:0] hcount,
    input wire [11:0] vcount,
    input wire video_active,

    output wire osd_active,
    output wire [23:0] osd_rgb
);

//====================================================
// 1. Center panel
//====================================================

wire panel;

assign panel =
    video_active &&
    hcount >= 12'd218 &&
    hcount < 12'd589 &&
    vcount >= 12'd12 &&
    vcount < 12'd56;

wire border_pixel;

assign border_pixel =
    panel &&
    (
        hcount == 12'd218 ||
        hcount == 12'd588 ||
        vcount == 12'd12 ||
        vcount == 12'd55
    );

//====================================================
// 2. Title and LIVE regions
//====================================================

wire title_area;

assign title_area =
    panel &&
    hcount >= 12'd232 &&
    hcount < 12'd440 &&
    vcount >= 12'd25 &&
    vcount < 12'd41;

wire live_area;

assign live_area =
    panel &&
    hcount >= 12'd500 &&
    hcount < 12'd564 &&
    vcount >= 12'd25 &&
    vcount < 12'd41;

wire live_dot;

assign live_dot =
    panel &&
    hcount >= 12'd486 &&
    hcount < 12'd493 &&
    vcount >= 12'd29 &&
    vcount < 12'd36;

//====================================================
// 3. Character coordinates
// 8x8 glyph, scaled to 16x16
//====================================================

wire [11:0] char_dx;
wire [11:0] char_dy;

assign char_dx =
    title_area ?
    (hcount - 12'd232) :
    (hcount - 12'd500);

assign char_dy = vcount - 12'd25;

wire [3:0] char_index;
wire [2:0] font_x;
wire [2:0] font_y;

assign char_index = char_dx[7:4];
assign font_x = char_dx[3:1];
assign font_y = char_dy[3:1];

//====================================================
// 4. ASCII selector
//====================================================

reg [7:0] ascii_char;

always @(*) begin

    ascii_char = " ";

    if (title_area) begin

        // ACG720 VISION
        case (char_index)
            4'd0: ascii_char = "A";
            4'd1: ascii_char = "C";
            4'd2: ascii_char = "G";
            4'd3: ascii_char = "7";
            4'd4: ascii_char = "2";
            4'd5: ascii_char = "0";
            4'd6: ascii_char = " ";
            4'd7: ascii_char = "V";
            4'd8: ascii_char = "I";
            4'd9: ascii_char = "S";
            4'd10: ascii_char = "I";
            4'd11: ascii_char = "O";
            4'd12: ascii_char = "N";

            default: ascii_char = " ";
        endcase

    end else if (live_area) begin

        case (char_index)
            4'd0: ascii_char = "L";
            4'd1: ascii_char = "I";
            4'd2: ascii_char = "V";
            4'd3: ascii_char = "E";

            default: ascii_char = " ";
        endcase

    end

end

//====================================================
// 5. Font ROM
//====================================================

reg [63:0] glyph;

always @(*) begin

    case (ascii_char)

        "0": glyph = 64'h70_88_98_A8_C8_88_70_00;
        "2": glyph = 64'h70_88_08_10_20_40_F8_00;
        "7": glyph = 64'hF8_08_10_20_40_40_40_00;

        "A": glyph = 64'h70_88_88_F8_88_88_88_00;
        "C": glyph = 64'h70_88_80_80_80_88_70_00;
        "E": glyph = 64'hF8_80_80_F0_80_80_F8_00;
        "G": glyph = 64'h70_88_80_B8_88_88_78_00;

        "I": glyph = 64'h70_20_20_20_20_20_70_00;
        "L": glyph = 64'h80_80_80_80_80_80_F8_00;
        "N": glyph = 64'h88_C8_A8_98_88_88_88_00;
        "O": glyph = 64'h70_88_88_88_88_88_70_00;
        "S": glyph = 64'h70_88_80_70_08_88_70_00;
        "V": glyph = 64'h88_88_88_88_88_50_20_00;

        default:
            glyph = 64'h00_00_00_00_00_00_00_00;

    endcase

end

//====================================================
// 6. Font row
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
// 7. Final pixel
//====================================================

wire text_pixel;

assign text_pixel =
    (title_area || live_area) &&
    glyph_row[3'd7 - font_x];

assign osd_active = panel;

assign osd_rgb =
    live_dot                ? 24'h57E6A5 :
    text_pixel && live_area ? 24'h8AF4CD :
    text_pixel              ? 24'hFFFFFF :
    border_pixel            ? 24'h38C9DC :
                              24'h0C283C;

endmodule
