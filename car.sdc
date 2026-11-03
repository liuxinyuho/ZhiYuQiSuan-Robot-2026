
#====================================================
# ACG720 GW5AT-60K
# Robot + OV5640 + HDMI + YAW + DISTANCE OSD
#====================================================

# System clock: 50MHz
create_clock -name clk -period 20.000 [get_ports {clk}]


#====================================================
# ACG720 GW5AT-60K
# OV5640 Camera Pixel Clock
#
# Provisional conservative constraint: 50 MHz
# Verify actual PCLK with clock measurement.
#====================================================

create_clock -name camera_pclk -period 20.000 [get_ports {camera_pclk}]

#====================================================
# 1. Camera status CDC
#====================================================

set_false_path -to [get_pins {u_video/init_sync1_s0/D}]

set_false_path -to [get_pins {u_video/seen_sync1_s0/D}]


#====================================================
# 2. YAW OSD CDC
#====================================================

# Stable BCD bus
set_false_path -to [get_pins {u_video/u_yaw_osd/bcd_pix_*_s0/D}]


# Sign bit
set_false_path -to [get_pins {u_video/u_yaw_osd/negative_pix_s0/D}]

# Acknowledge synchronizer: first stage
set_false_path -to [get_pins {u_video/u_yaw_osd/ack_pix_s1_s0/D}]

# Request synchronizer: first stage
set_false_path -to [get_pins {u_video/u_yaw_osd/req_sys_s1_s0/D}]


#====================================================
# 3. DISTANCE OSD CDC
#====================================================

# 64-bit BCD data bus
# Source is held stable until handshake acknowledge

set_false_path -to [get_pins {u_video/u_distance_osd/bcd_pix_*_s0/D}]


#====================================================
# 4. Distance sensor validity flags
#====================================================

# 4-bit validity bus, same handshake as BCD

set_false_path -to [get_pins {u_video/u_distance_osd/valid_pix_*_s0/D}]


#====================================================
# 5. Distance OSD request synchronizer
# System clock -> HDMI pixel clock
# First synchronization stage only
#====================================================

set_false_path -to [get_pins {u_video/u_distance_osd/req_pix_s1_s0/D}]


#====================================================
# 6. Distance OSD acknowledge synchronizer
# HDMI pixel clock -> system clock
# First synchronization stage only
#====================================================

set_false_path -to [get_pins {u_video/u_distance_osd/ack_sys_s1_s0/D}]

#====================================================
# END
#====================================================

#===============================================
# ACG720 60K
# MPU6500 Calibration Countdown OSD CDC
#
# System clock 50MHz -> HDMI pixel clock
# Data held stable by YAW req/ack handshake
#===============================================

# MPU6500 initialization status
set_false_path -to [get_pins {u_video/u_yaw_osd/imu_ready_pix_s0/D}]

# MPU6500 calibration completed status
set_false_path -to [get_pins {u_video/u_yaw_osd/calib_done_pix_s0/D}]

# Calibration countdown 4-bit data
set_false_path -to [get_pins {u_video/u_yaw_osd/secs_pix_0_s0/D}]

set_false_path -to [get_pins {u_video/u_yaw_osd/secs_pix_1_s0/D}]

set_false_path -to [get_pins {u_video/u_yaw_osd/secs_pix_2_s0/D}]

set_false_path -to [get_pins {u_video/u_yaw_osd/secs_pix_3_s0/D}]

#====================================================
# ACG720 60K
# ROBOT STATUS OSD CDC
#
# FPGA System Clock -> HDMI Pixel Clock
#
# Status data is protected by
# request / acknowledge handshake
#====================================================


#====================================================
# 1. Request synchronizer
# sys_clk -> pix_clk
# First-stage input only
#====================================================

set_false_path -to [get_pins {u_video/u_robot_status_osd/req_pix_s1_s0/D}]


#====================================================
# 2. Acknowledge synchronizer
# pix_clk -> sys_clk
# First-stage input only
#====================================================

set_false_path -to [get_pins {u_video/u_robot_status_osd/ack_sys_s1_s0/D}]



#====================================================
# 3. ROBOT STATUS OSD DATA CDC
#
# 10-bit status data bus
# Source held stable by req/ack handshake
#
# Use wildcard to match only existing registers
#====================================================

set_false_path -to [get_pins {u_video/u_robot_status_osd/status_pix_*_s0/D}]

#====================================================
# END ROBOT STATUS OSD CDC
#====================================================


#====================================================
# ACG720 GW5AT-60K
# MOTION OSD CDC
#
# Four wheel encoder speeds
# XY odometry
#
# System clock 50MHz -> HDMI pixel clock
#====================================================


#====================================================
# 1. Motion OSD request synchronizer
# sys_clk -> pix_clk
# First synchronization stage only
#====================================================

set_false_path -to [get_pins {u_video/u_motion_osd/req_pix_s1_s0/D}]


#====================================================
# 2. Motion OSD acknowledge synchronizer
# pix_clk -> sys_clk
# First synchronization stage only
#====================================================

set_false_path -to [get_pins {u_video/u_motion_osd/ack_sys_s1_s0/D}]


#====================================================
# 3. Motion OSD BCD data bus
#
# 6 values x 16-bit BCD = 96 bits
#
# Source bus remains stable until
# request/acknowledge handshake completes
#====================================================

set_false_path -to [get_pins {u_video/u_motion_osd/bcd_pix_*_s0/D}]


#====================================================
# 4. Motion OSD sign data bus
#
# 6 sign bits
# Transferred using the same handshake
#====================================================

set_false_path -to [get_pins {u_video/u_motion_osd/sign_pix_*_s0/D}]


#====================================================
# END MOTION OSD CDC
#====================================================
set_false_path -to [get_pins {u_video/u_motion_osd/req_pix_s1_s0/D}]

set_false_path -to [get_pins {u_video/u_motion_osd/ack_sys_s1_s0/D}]

set_false_path -to [get_pins {u_video/u_motion_osd/bcd_pix_*_s0/D}]

set_false_path -to [get_pins {u_video/u_motion_osd/sign_pix_*_s0/D}]

#====================================================
# ACG720 RADAR OSD CDC
# 50MHz system clock <-> HDMI pixel clock
#====================================================

# Request first-stage synchronizer
set_false_path -to [get_pins {u_video/u_radar_osd/req_pix_s1_s0/D}]

# Acknowledge first-stage synchronizer
set_false_path -to [get_pins {u_video/u_radar_osd/ack_sys_s1_s0/D}]

# Stable snapshot bus protected by request/ack
set_false_path -to [get_pins {u_video/u_radar_osd/radar_pix_*_s0/D}]

#====================================================
# END RADAR CDC
#====================================================
#====================================================
# ACG720 AI SETTINGS OSD CDC
#
# System Clock 50MHz -> HDMI Pixel Clock
#
# All status signals use two-stage synchronizers.
# Only the first-stage D pins are excluded.
#
# IMPORTANT:
# Do NOT false-path the second-stage registers.
#====================================================


#====================================================
# 1. BM1684 Host Mode CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/host_mode_meta_s0/D}]


#====================================================
# 2. AI Mode CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/ai_mode_meta_s0/D}]


#====================================================
# 3. Host Control Active CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/control_meta_s0/D}]


#====================================================
# 4. Watchdog Status CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/watchdog_meta_s0/D}]


#====================================================
# 5. Stop Request CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/stop_meta_s0/D}]


#====================================================
# 6. Ultrasonic Status CDC
#====================================================

set_false_path -to [get_pins {u_video/u_ai_settings_osd/ultra_meta_s0/D}]

# Ultrasonic recovery indicator: asynchronous status into first CDC stage only
set_false_path -to [get_pins {u_video/u_ai_settings_osd/ultra_recover_meta_s0/D}]


#====================================================
# 7. SW0 Display Mode CDC
#
# System Clock -> HDMI Pixel Clock
# First-stage synchronizer only
#====================================================

set_false_path -to [get_pins {u_video/ui_mode_ai_meta_s0/D}]


#====================================================
# END AI SETTINGS OSD CDC
#====================================================