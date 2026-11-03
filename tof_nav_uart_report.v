
`timescale 1ns / 1ps

//============================================================
// 文件名：tof_nav_uart_report.v
//
// 功能：三 ToF + A5/A6 状态串口监测（独立调试版）
//
// FPGA：ACG720
// 主时钟：50MHz
// 串口：115200，8N1
//
// 每约200ms输出一行：
//
// L=00350 R=00480 LV=1 RV=1 A5=02 A6=00 WP=0 F=00550 FV=1 RE=0
//
// L  ：左侧 ToF 距离，单位mm
// R  ：右侧 ToF 距离，单位mm
// LV ：左侧数据有效
// RV ：右侧数据有效
// A5 ：避障状态编号
// A6 ：导航协调器状态编号
// WP ：当前航点编号
//
// 特点：
// 1. 不修改底盘运动控制
// 2. 不使用除法进行十进制转换
// 3. 采集一份数据快照，避免串口发送时数据变化
// 4. 复用工程现有 uart_byte_tx 模块
//============================================================

module tof_nav_uart_report (

    input wire clk,
    input wire reset_n,

    // 左右 ToF 距离
    input wire [15:0] left_mm,
    input wire [15:0] right_mm,
    input wire [15:0] front_mm,
    // 后方第四个VL53L0X
    input wire [15:0] rear_mm,
    input wire rear_ok,

    // 左右 ToF 健康状态
    input wire left_ok,
    input wire right_ok,
    input wire front_ok,
    input wire right_error,

    // A5 避障状态
    input wire [3:0] a5_state,

    // A6 协调器状态
    input wire [2:0] a6_state,

    // 当前航点编号
    input wire [2:0] waypoint_index,
    input wire [2:0] fault_code,
    input wire [2:0] gate_debug,
    input wire [2:0] supervisor_debug,
    input wire [1:0] detour_direction,
    input wire [15:0] detour_distance,
    input wire [2:0] detour_fault,
    input wire signed [12:0] detour_vx,
    input wire signed [12:0] detour_vy,
    input wire [2:0] turn_block_reason,

    // P4: fresh ToF vs ultrasonic cache vs minimum-range fusion.
    // HC/HV are a cached servo-facing ultrasonic sample and its validity.
    input wire [15:0] hc_front_mm,
    input wire hc_front_valid,
    input wire [7:0] hc_front_age,
    input wire [15:0] fused_front_mm,
    input wire fused_front_valid,
    // First blocked instant in this turn (latched by top.v).
    input wire turn_event_seen,
    input wire [15:0] turn_event_tof_mm,
    input wire turn_event_tof_valid,
    input wire [15:0] turn_event_hc_mm,
    input wire turn_event_hc_valid,
    input wire [15:0] turn_event_fused_mm,
    input wire turn_event_fused_valid,
    input wire [2:0] turn_event_reason,

    // HC-SR04 on SG90 pan servo, one completed 3-direction scan
    input wire scan_active,
    input wire scan_done,
    input wire [15:0] scan_left_mm,
    input wire [15:0] scan_center_mm,
    input wire [15:0] scan_right_mm,

    // D5: Existing FPGA status signals, no impact on motion control
    input wire signed [31:0] yaw_cdeg,    // 0.01 degree
    input wire signed [31:0] world_x_mm,  // mm
    input wire signed [31:0] world_y_mm,  // mm
    input wire signed [31:0] encoder_a,   // encoder ticks
    input wire signed [31:0] encoder_b,
    input wire signed [31:0] encoder_c,
    input wire signed [31:0] encoder_d,

    // 串口输出
    output wire uart_tx

);

//============================================================
// 一、报告周期
//
// 50MHz 时钟：
// 10,000,000个周期约为200ms
//============================================================

localparam [23:0] REPORT_PERIOD =
    24'd9999999;

reg [23:0] report_counter;


//============================================================
// 二、缓存本次需要发送的状态
//============================================================

reg left_ok_hold;
reg right_ok_hold;
reg front_ok_hold;
reg rear_ok_hold;
reg right_error_hold;

reg [3:0] a5_state_hold;
reg [2:0] a6_state_hold;
reg [2:0] waypoint_hold;
reg [2:0] fault_code_hold;
reg [2:0] gate_debug_hold;
reg [2:0] supervisor_debug_hold;
reg [1:0] detour_direction_hold;
reg [2:0] detour_fault_hold, turn_block_reason_hold;
reg detour_vx_neg_hold, detour_vy_neg_hold;
reg [6:0] detour_vx_mag_hold, detour_vy_mag_hold;
reg [35:0] detour_distance_shift;

// The scan FSM updates distance registers as it moves. The
// cached values below change ONLY on scan_done, so UART never
// mistakes an incomplete scan for a completed set of readings.
reg [15:0] scan_left_saved;
reg [15:0] scan_center_saved;
reg [15:0] scan_right_saved;
reg scan_complete;
reg scan_complete_hold;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        scan_left_saved   <= 16'd0;
        scan_center_saved <= 16'd0;
        scan_right_saved  <= 16'd0;
        scan_complete     <= 1'b0;
    end else if (scan_done) begin
        scan_left_saved   <= scan_left_mm;
        scan_center_saved <= scan_center_mm;
        scan_right_saved  <= scan_right_mm;
        // SV=1 only when all three directional ranges are trustworthy.
        // scan_done alone means FSM finished; zero means UNKNOWN.
        scan_complete     <= (scan_left_mm   != 16'd0) &&
                             (scan_center_mm != 16'd0) &&
                             (scan_right_mm  != 16'd0);
    end else if (scan_active) begin
        // Old results remain for reference but SV=0 while scanning.
        scan_complete <= 1'b0;
    end
end


//============================================================
// 三、二进制转十进制BCD
//
// 16bit距离最大65535
// 使用5位十进制数字表示
//
// 采用Double Dabble算法：
// 每个时钟处理1位，共16个时钟完成。
//============================================================

// 高20位：5位BCD数字
// 低16位：尚未转换的二进制数据
reg [35:0] left_shift;
reg [35:0] right_shift;
reg [35:0] front_shift;
reg [35:0] rear_shift;
reg [35:0] scan_left_shift;
reg [35:0] scan_center_shift;
reg [35:0] scan_right_shift;
// Extra P4 source diagnostics (six 16-bit double-dabble registers).
reg [35:0] hc_front_shift;
reg [35:0] hc_age_shift;
reg [35:0] fused_front_shift;
reg [35:0] turn_event_tof_shift;
reg [35:0] turn_event_hc_shift;
reg [35:0] turn_event_fused_shift;
reg hc_front_valid_hold, fused_front_valid_hold;
reg turn_event_seen_hold, turn_event_tof_valid_hold;
reg turn_event_hc_valid_hold, turn_event_fused_valid_hold;
reg [2:0] turn_event_reason_hold;


// D5 32-bit signed decimal conversion, 10 BCD digits each.
// Magnitudes are computed before conversion; sign is held separately.
reg [71:0] yaw32_shift;
reg [71:0] x32_shift;
reg [71:0] y32_shift;
reg [71:0] ea32_shift;
reg [71:0] eb32_shift;
reg [71:0] ec32_shift;
reg [71:0] ed32_shift;

reg yaw_neg_hold, x_neg_hold, y_neg_hold;
reg ea_neg_hold, eb_neg_hold, ec_neg_hold, ed_neg_hold;

reg [5:0] convert_count;


//============================================================
// BCD转换单步函数
//
// 每个BCD数字大于等于5时加3，
// 再整体左移1位。
//============================================================

function [35:0] bcd_step;

    input [35:0] value;

    reg [35:0] temp;

    begin

        temp = value;

        // 个位
        if (temp[19:16] >= 4'd5)
            temp[19:16] = temp[19:16] + 4'd3;

        // 十位
        if (temp[23:20] >= 4'd5)
            temp[23:20] = temp[23:20] + 4'd3;

        // 百位
        if (temp[27:24] >= 4'd5)
            temp[27:24] = temp[27:24] + 4'd3;

        // 千位
        if (temp[31:28] >= 4'd5)
            temp[31:28] = temp[31:28] + 4'd3;

        // 万位
        if (temp[35:32] >= 4'd5)
            temp[35:32] = temp[35:32] + 4'd3;

        bcd_step = temp << 1;

    end

endfunction


// D5 Double-Dabble, one binary bit per clock, 32 iterations.
// The input format is {40-bit BCD, 32-bit unsigned magnitude}.
function [71:0] bcd32_step;
    input [71:0] value;
    reg [71:0] tmp;
    integer digit;
    begin
        tmp = value;
        for (digit = 0; digit < 10; digit = digit + 1) begin
            if (tmp[32 + digit*4 +: 4] >= 4'd5)
                tmp[32 + digit*4 +: 4] = tmp[32 + digit*4 +: 4] + 4'd3;
        end
        bcd32_step = tmp << 1;
    end
endfunction

function [31:0] magnitude32;
    input [31:0] signed_value;
    begin
        magnitude32 = signed_value[31] ? (~signed_value + 32'd1) : signed_value;
    end
endfunction

//============================================================
// 四、串口发送模块
//
// 复用已经调通的 uart_byte_tx.v
//============================================================

reg [7:0] tx_data;
reg tx_send;

wire tx_busy;
wire tx_done;

uart_byte_tx u_report_uart_tx (

    .clk       (clk),
    .reset_n   (reset_n),

    .data_byte (tx_data),

    .send_en   (tx_send),

    .uart_tx   (uart_tx),

    .tx_done   (tx_done),
    .tx_busy   (tx_busy)

);


//============================================================
// 五、ASCII字符编码
//
// 完整输出：
// L=00350 R=00480 LV=1 RV=1 A5=02 A6=00 WP=0 F=00550 FV=1 RE=0
//
// 最后发送CR、LF，形成换行。
//============================================================

reg [8:0] char_index;

reg [7:0] current_char;


//============================================================
// 根据字符位置选择输出内容
//============================================================

always @(*) begin

    current_char = 8'h20;

    case (char_index)

        //========================================
        // 左 ToF：L=00350
        //========================================

        6'd0: current_char = "L";
        6'd1: current_char = "=";

        6'd2:
            current_char =
                8'h30 + {4'd0, left_shift[35:32]};

        6'd3:
            current_char =
                8'h30 + {4'd0, left_shift[31:28]};

        6'd4:
            current_char =
                8'h30 + {4'd0, left_shift[27:24]};

        6'd5:
            current_char =
                8'h30 + {4'd0, left_shift[23:20]};

        6'd6:
            current_char =
                8'h30 + {4'd0, left_shift[19:16]};

        6'd7: current_char = " ";


        //========================================
        // 右 ToF：R=00480
        //========================================

        6'd8: current_char = "R";
        6'd9: current_char = "=";

        6'd10:
            current_char =
                8'h30 + {4'd0, right_shift[35:32]};

        6'd11:
            current_char =
                8'h30 + {4'd0, right_shift[31:28]};

        6'd12:
            current_char =
                8'h30 + {4'd0, right_shift[27:24]};

        6'd13:
            current_char =
                8'h30 + {4'd0, right_shift[23:20]};

        6'd14:
            current_char =
                8'h30 + {4'd0, right_shift[19:16]};

        6'd15: current_char = " ";


        //========================================
        // 左 ToF 健康标志：LV=1
        //========================================

        6'd16: current_char = "L";
        6'd17: current_char = "V";
        6'd18: current_char = "=";

        6'd19:
            current_char =
                left_ok_hold ? "1" : "0";

        6'd20: current_char = " ";


        //========================================
        // 右 ToF 健康标志：RV=1
        //========================================

        6'd21: current_char = "R";
        6'd22: current_char = "V";
        6'd23: current_char = "=";

        6'd24:
            current_char =
                right_ok_hold ? "1" : "0";

        6'd25: current_char = " ";


        //========================================
        // A5状态：A5=02
        //
        // A5状态编号范围0~14
        //========================================

        6'd26: current_char = "A";
        6'd27: current_char = "5";
        6'd28: current_char = "=";

        // 十位
        6'd29:
            current_char =
                (a5_state_hold >= 4'd10) ?
                "1" : "0";

        // 个位
        6'd30:
            current_char =
                (a5_state_hold >= 4'd10) ?
                (8'h30 +
                    {4'd0, (a5_state_hold - 4'd10)}) :
                (8'h30 +
                    {4'd0, a5_state_hold});

        6'd31: current_char = " ";


        //========================================
        // A6状态：A6=00
        //========================================

        6'd32: current_char = "A";
        6'd33: current_char = "6";
        6'd34: current_char = "=";

        6'd35: current_char = "0";

        6'd36:
            current_char =
                8'h30 + {5'd0, a6_state_hold};

        6'd37: current_char = " ";


        //========================================
        // 当前航点：WP=0
        //========================================

        6'd38: current_char = "W";
        6'd39: current_char = "P";
        6'd40: current_char = "=";

        6'd41:
            current_char =
                8'h30 + {5'd0, waypoint_hold};


        //========================================
        // 换行
        //========================================

        // Front ToF distance + validity (3rd independent I2C)
        6'd42: current_char = " ";
        6'd43: current_char = "F";
        6'd44: current_char = "=";
        6'd45: current_char = 8'h30 + {4'd0, front_shift[35:32]};
        6'd46: current_char = 8'h30 + {4'd0, front_shift[31:28]};
        6'd47: current_char = 8'h30 + {4'd0, front_shift[27:24]};
        6'd48: current_char = 8'h30 + {4'd0, front_shift[23:20]};
        6'd49: current_char = 8'h30 + {4'd0, front_shift[19:16]};
        6'd50: current_char = " ";
        6'd51: current_char = "F";
        6'd52: current_char = "V";
        6'd53: current_char = "=";
        6'd54: current_char = front_ok_hold ? "1" : "0";
        // Right sensor hardware/driver error for RV=0 diagnostics
        6'd55: current_char = " ";
        6'd56: current_char = "R";
        6'd57: current_char = "E";
        6'd58: current_char = "=";
        6'd59: current_char = right_error_hold ? "1" : "0";
        6'd60: current_char = 8'h0D;
        6'd61: current_char = 8'h0A;

        // A second UART line: ultrasonic scan distance in millimeters
        // SL/SC/SR are HC-SR04 scan results (NOT fixed ToF sensors).
        // SV=1 means that the last three-direction scan completed.
        7'd62: current_char = "S";
        7'd63: current_char = "L";
        7'd64: current_char = "=";
        7'd65: current_char = 8'h30 + {4'd0, scan_left_shift[35:32]};
        7'd66: current_char = 8'h30 + {4'd0, scan_left_shift[31:28]};
        7'd67: current_char = 8'h30 + {4'd0, scan_left_shift[27:24]};
        7'd68: current_char = 8'h30 + {4'd0, scan_left_shift[23:20]};
        7'd69: current_char = 8'h30 + {4'd0, scan_left_shift[19:16]};
        7'd70: current_char = " ";
        7'd71: current_char = "S";
        7'd72: current_char = "C";
        7'd73: current_char = "=";
        7'd74: current_char = 8'h30 + {4'd0, scan_center_shift[35:32]};
        7'd75: current_char = 8'h30 + {4'd0, scan_center_shift[31:28]};
        7'd76: current_char = 8'h30 + {4'd0, scan_center_shift[27:24]};
        7'd77: current_char = 8'h30 + {4'd0, scan_center_shift[23:20]};
        7'd78: current_char = 8'h30 + {4'd0, scan_center_shift[19:16]};
        7'd79: current_char = " ";
        7'd80: current_char = "S";
        7'd81: current_char = "R";
        7'd82: current_char = "=";
        7'd83: current_char = 8'h30 + {4'd0, scan_right_shift[35:32]};
        7'd84: current_char = 8'h30 + {4'd0, scan_right_shift[31:28]};
        7'd85: current_char = 8'h30 + {4'd0, scan_right_shift[27:24]};
        7'd86: current_char = 8'h30 + {4'd0, scan_right_shift[23:20]};
        7'd87: current_char = 8'h30 + {4'd0, scan_right_shift[19:16]};
        7'd88: current_char = " ";
        7'd89: current_char = "S";
        7'd90: current_char = "V";
        7'd91: current_char = "=";
        7'd92: current_char = scan_complete_hold ? "1" : "0";
        7'd93: current_char = 8'h0D;
        7'd94: current_char = 8'h0A;

        // D5 status lines: YAW=centidegrees, X/Y=millimeters, E*=encoder ticks
        8'd95: current_char = "Y";
        8'd96: current_char = "A";
        8'd97: current_char = "W";
        8'd98: current_char = "=";
        8'd99: current_char = yaw_neg_hold ? "-" : "+";
        8'd100: current_char = 8'h30 + {4'd0, yaw32_shift[71:68]};
        8'd101: current_char = 8'h30 + {4'd0, yaw32_shift[67:64]};
        8'd102: current_char = 8'h30 + {4'd0, yaw32_shift[63:60]};
        8'd103: current_char = 8'h30 + {4'd0, yaw32_shift[59:56]};
        8'd104: current_char = 8'h30 + {4'd0, yaw32_shift[55:52]};
        8'd105: current_char = 8'h30 + {4'd0, yaw32_shift[51:48]};
        8'd106: current_char = 8'h30 + {4'd0, yaw32_shift[47:44]};
        8'd107: current_char = 8'h30 + {4'd0, yaw32_shift[43:40]};
        8'd108: current_char = 8'h30 + {4'd0, yaw32_shift[39:36]};
        8'd109: current_char = 8'h30 + {4'd0, yaw32_shift[35:32]};
        8'd110: current_char = " ";
        8'd111: current_char = "X";
        8'd112: current_char = "=";
        8'd113: current_char = x_neg_hold ? "-" : "+";
        8'd114: current_char = 8'h30 + {4'd0, x32_shift[71:68]};
        8'd115: current_char = 8'h30 + {4'd0, x32_shift[67:64]};
        8'd116: current_char = 8'h30 + {4'd0, x32_shift[63:60]};
        8'd117: current_char = 8'h30 + {4'd0, x32_shift[59:56]};
        8'd118: current_char = 8'h30 + {4'd0, x32_shift[55:52]};
        8'd119: current_char = 8'h30 + {4'd0, x32_shift[51:48]};
        8'd120: current_char = 8'h30 + {4'd0, x32_shift[47:44]};
        8'd121: current_char = 8'h30 + {4'd0, x32_shift[43:40]};
        8'd122: current_char = 8'h30 + {4'd0, x32_shift[39:36]};
        8'd123: current_char = 8'h30 + {4'd0, x32_shift[35:32]};
        8'd124: current_char = " ";
        8'd125: current_char = "Y";
        8'd126: current_char = "=";
        8'd127: current_char = y_neg_hold ? "-" : "+";
        8'd128: current_char = 8'h30 + {4'd0, y32_shift[71:68]};
        8'd129: current_char = 8'h30 + {4'd0, y32_shift[67:64]};
        8'd130: current_char = 8'h30 + {4'd0, y32_shift[63:60]};
        8'd131: current_char = 8'h30 + {4'd0, y32_shift[59:56]};
        8'd132: current_char = 8'h30 + {4'd0, y32_shift[55:52]};
        8'd133: current_char = 8'h30 + {4'd0, y32_shift[51:48]};
        8'd134: current_char = 8'h30 + {4'd0, y32_shift[47:44]};
        8'd135: current_char = 8'h30 + {4'd0, y32_shift[43:40]};
        8'd136: current_char = 8'h30 + {4'd0, y32_shift[39:36]};
        8'd137: current_char = 8'h30 + {4'd0, y32_shift[35:32]};
        8'd138: current_char = 8'h0D;
        8'd139: current_char = 8'h0A;
        8'd140: current_char = "E";
        8'd141: current_char = "A";
        8'd142: current_char = "=";
        8'd143: current_char = ea_neg_hold ? "-" : "+";
        8'd144: current_char = 8'h30 + {4'd0, ea32_shift[71:68]};
        8'd145: current_char = 8'h30 + {4'd0, ea32_shift[67:64]};
        8'd146: current_char = 8'h30 + {4'd0, ea32_shift[63:60]};
        8'd147: current_char = 8'h30 + {4'd0, ea32_shift[59:56]};
        8'd148: current_char = 8'h30 + {4'd0, ea32_shift[55:52]};
        8'd149: current_char = 8'h30 + {4'd0, ea32_shift[51:48]};
        8'd150: current_char = 8'h30 + {4'd0, ea32_shift[47:44]};
        8'd151: current_char = 8'h30 + {4'd0, ea32_shift[43:40]};
        8'd152: current_char = 8'h30 + {4'd0, ea32_shift[39:36]};
        8'd153: current_char = 8'h30 + {4'd0, ea32_shift[35:32]};
        8'd154: current_char = " ";
        8'd155: current_char = "E";
        8'd156: current_char = "B";
        8'd157: current_char = "=";
        8'd158: current_char = eb_neg_hold ? "-" : "+";
        8'd159: current_char = 8'h30 + {4'd0, eb32_shift[71:68]};
        8'd160: current_char = 8'h30 + {4'd0, eb32_shift[67:64]};
        8'd161: current_char = 8'h30 + {4'd0, eb32_shift[63:60]};
        8'd162: current_char = 8'h30 + {4'd0, eb32_shift[59:56]};
        8'd163: current_char = 8'h30 + {4'd0, eb32_shift[55:52]};
        8'd164: current_char = 8'h30 + {4'd0, eb32_shift[51:48]};
        8'd165: current_char = 8'h30 + {4'd0, eb32_shift[47:44]};
        8'd166: current_char = 8'h30 + {4'd0, eb32_shift[43:40]};
        8'd167: current_char = 8'h30 + {4'd0, eb32_shift[39:36]};
        8'd168: current_char = 8'h30 + {4'd0, eb32_shift[35:32]};
        8'd169: current_char = " ";
        8'd170: current_char = "E";
        8'd171: current_char = "C";
        8'd172: current_char = "=";
        8'd173: current_char = ec_neg_hold ? "-" : "+";
        8'd174: current_char = 8'h30 + {4'd0, ec32_shift[71:68]};
        8'd175: current_char = 8'h30 + {4'd0, ec32_shift[67:64]};
        8'd176: current_char = 8'h30 + {4'd0, ec32_shift[63:60]};
        8'd177: current_char = 8'h30 + {4'd0, ec32_shift[59:56]};
        8'd178: current_char = 8'h30 + {4'd0, ec32_shift[55:52]};
        8'd179: current_char = 8'h30 + {4'd0, ec32_shift[51:48]};
        8'd180: current_char = 8'h30 + {4'd0, ec32_shift[47:44]};
        8'd181: current_char = 8'h30 + {4'd0, ec32_shift[43:40]};
        8'd182: current_char = 8'h30 + {4'd0, ec32_shift[39:36]};
        8'd183: current_char = 8'h30 + {4'd0, ec32_shift[35:32]};
        8'd184: current_char = " ";
        8'd185: current_char = "E";
        8'd186: current_char = "D";
        8'd187: current_char = "=";
        8'd188: current_char = ed_neg_hold ? "-" : "+";
        8'd189: current_char = 8'h30 + {4'd0, ed32_shift[71:68]};
        8'd190: current_char = 8'h30 + {4'd0, ed32_shift[67:64]};
        8'd191: current_char = 8'h30 + {4'd0, ed32_shift[63:60]};
        8'd192: current_char = 8'h30 + {4'd0, ed32_shift[59:56]};
        8'd193: current_char = 8'h30 + {4'd0, ed32_shift[55:52]};
        8'd194: current_char = 8'h30 + {4'd0, ed32_shift[51:48]};
        8'd195: current_char = 8'h30 + {4'd0, ed32_shift[47:44]};
        8'd196: current_char = 8'h30 + {4'd0, ed32_shift[43:40]};
        8'd197: current_char = 8'h30 + {4'd0, ed32_shift[39:36]};
        8'd198: current_char = 8'h30 + {4'd0, ed32_shift[35:32]};
        8'd199: current_char = 8'h0D;
        8'd200: current_char = 8'h0A;
        // 新增第五行：B=00520 BV=1（后方ToF）
        8'd201: current_char = "B";
        8'd202: current_char = "=";
        8'd203: current_char = 8'h30 + {4'd0, rear_shift[35:32]};
        8'd204: current_char = 8'h30 + {4'd0, rear_shift[31:28]};
        8'd205: current_char = 8'h30 + {4'd0, rear_shift[27:24]};
        8'd206: current_char = 8'h30 + {4'd0, rear_shift[23:20]};
        8'd207: current_char = 8'h30 + {4'd0, rear_shift[19:16]};
        8'd208: current_char = " ";
        8'd209: current_char = "B";
        8'd210: current_char = "V";
        8'd211: current_char = "=";
        8'd212: current_char = rear_ok_hold ? "1" : "0";
        // FC=nav first fault, GB=currently blocking motion,
        // NS=A5 supervisor FSM (0 monitor,1 wait,2 avoid,3 stop,
        // 4 restore,5 fault).
        8'd213: current_char = " ";
        8'd214: current_char = "F";
        8'd215: current_char = "C";
        8'd216: current_char = "=";
        8'd217: current_char = 8'h30 + {5'd0, fault_code_hold};
        8'd218: current_char = " ";
        8'd219: current_char = "G";
        8'd220: current_char = "B";
        8'd221: current_char = "=";
        8'd222: current_char = 8'h30 + {5'd0, gate_debug_hold};
        8'd223: current_char = " ";
        8'd224: current_char = "N";
        8'd225: current_char = "S";
        8'd226: current_char = "=";
        8'd227: current_char = 8'h30 + {5'd0, supervisor_debug_hold};
        9'd228: current_char = " ";
        9'd229: current_char = "D";
        9'd230: current_char = "D";
        9'd231: current_char = "=";
        9'd232: current_char = 8'h30 + {6'd0, detour_direction_hold};
        9'd233: current_char = " ";
        9'd234: current_char = "D";
        9'd235: current_char = "V";
        9'd236: current_char = "X";
        9'd237: current_char = "=";
        9'd238: current_char = detour_vx_neg_hold ? "-" : "+";
        9'd239: current_char = 8'h30 + (detour_vx_mag_hold / 7'd10);
        9'd240: current_char = 8'h30 + (detour_vx_mag_hold % 7'd10);
        9'd241: current_char = " ";
        9'd242: current_char = "D";
        9'd243: current_char = "V";
        9'd244: current_char = "Y";
        9'd245: current_char = "=";
        9'd246: current_char = detour_vy_neg_hold ? "-" : "+";
        9'd247: current_char = 8'h30 + (detour_vy_mag_hold / 7'd10);
        9'd248: current_char = 8'h30 + (detour_vy_mag_hold % 7'd10);
        9'd249: current_char = " ";
        9'd250: current_char = "D";
        9'd251: current_char = "M";
        9'd252: current_char = "=";
        9'd253: current_char = 8'h30 + {4'd0, detour_distance_shift[35:32]};
        9'd254: current_char = 8'h30 + {4'd0, detour_distance_shift[31:28]};
        9'd255: current_char = 8'h30 + {4'd0, detour_distance_shift[27:24]};
        9'd256: current_char = 8'h30 + {4'd0, detour_distance_shift[23:20]};
        9'd257: current_char = 8'h30 + {4'd0, detour_distance_shift[19:16]};
        9'd258: current_char = " ";
        9'd259: current_char = "T";
        9'd260: current_char = "B";
        9'd261: current_char = "=";
        9'd262: current_char = 8'h30 + {5'd0, turn_block_reason_hold};
        9'd263: current_char = " ";
        9'd264: current_char = "D";
        9'd265: current_char = "F";
        9'd266: current_char = "=";
        9'd267: current_char = 8'h30 + {5'd0, detour_fault_hold};
        9'd268: current_char = 8'h0D;
        9'd269: current_char = 8'h0A;

        // P4 diagnostic lines. Existing indices 0..269 are UNCHANGED.
        // HC=ultrasonic cache; HV=usable now; HA=cache age in 10ms ticks.
        // FF=fused minimum range; FFV=usable now.
        // TE=latched first turn veto; ET/TV=ToF at veto;
        // EH/EV=ultrasonic cache at veto; EF/EFV=fused at veto;
        // ER=reason at veto. The event is retained after a safety stop.
        9'd270: current_char = "H";
        9'd271: current_char = "C";
        9'd272: current_char = "=";
        9'd273: current_char = 8'h30 + {4'd0, hc_front_shift[35:32]};
        9'd274: current_char = 8'h30 + {4'd0, hc_front_shift[31:28]};
        9'd275: current_char = 8'h30 + {4'd0, hc_front_shift[27:24]};
        9'd276: current_char = 8'h30 + {4'd0, hc_front_shift[23:20]};
        9'd277: current_char = 8'h30 + {4'd0, hc_front_shift[19:16]};
        9'd278: current_char = " ";
        9'd279: current_char = "H";
        9'd280: current_char = "V";
        9'd281: current_char = "=";
        9'd282: current_char = hc_front_valid_hold ? "1" : "0";
        9'd283: current_char = " ";
        9'd284: current_char = "H";
        9'd285: current_char = "A";
        9'd286: current_char = "=";
        9'd287: current_char = 8'h30 + {4'd0, hc_age_shift[27:24]};
        9'd288: current_char = 8'h30 + {4'd0, hc_age_shift[23:20]};
        9'd289: current_char = 8'h30 + {4'd0, hc_age_shift[19:16]};
        9'd290: current_char = " ";
        9'd291: current_char = "F";
        9'd292: current_char = "F";
        9'd293: current_char = "=";
        9'd294: current_char = 8'h30 + {4'd0, fused_front_shift[35:32]};
        9'd295: current_char = 8'h30 + {4'd0, fused_front_shift[31:28]};
        9'd296: current_char = 8'h30 + {4'd0, fused_front_shift[27:24]};
        9'd297: current_char = 8'h30 + {4'd0, fused_front_shift[23:20]};
        9'd298: current_char = 8'h30 + {4'd0, fused_front_shift[19:16]};
        9'd299: current_char = " ";
        9'd300: current_char = "F";
        9'd301: current_char = "F";
        9'd302: current_char = "V";
        9'd303: current_char = "=";
        9'd304: current_char = fused_front_valid_hold ? "1" : "0";
        9'd305: current_char = 8'h0D;
        9'd306: current_char = 8'h0A;
        9'd307: current_char = "T";
        9'd308: current_char = "E";
        9'd309: current_char = "=";
        9'd310: current_char = turn_event_seen_hold ? "1" : "0";
        9'd311: current_char = " ";
        9'd312: current_char = "E";
        9'd313: current_char = "T";
        9'd314: current_char = "=";
        9'd315: current_char = 8'h30 + {4'd0, turn_event_tof_shift[35:32]};
        9'd316: current_char = 8'h30 + {4'd0, turn_event_tof_shift[31:28]};
        9'd317: current_char = 8'h30 + {4'd0, turn_event_tof_shift[27:24]};
        9'd318: current_char = 8'h30 + {4'd0, turn_event_tof_shift[23:20]};
        9'd319: current_char = 8'h30 + {4'd0, turn_event_tof_shift[19:16]};
        9'd320: current_char = " ";
        9'd321: current_char = "T";
        9'd322: current_char = "V";
        9'd323: current_char = "=";
        9'd324: current_char = turn_event_tof_valid_hold ? "1" : "0";
        9'd325: current_char = " ";
        9'd326: current_char = "E";
        9'd327: current_char = "H";
        9'd328: current_char = "=";
        9'd329: current_char = 8'h30 + {4'd0, turn_event_hc_shift[35:32]};
        9'd330: current_char = 8'h30 + {4'd0, turn_event_hc_shift[31:28]};
        9'd331: current_char = 8'h30 + {4'd0, turn_event_hc_shift[27:24]};
        9'd332: current_char = 8'h30 + {4'd0, turn_event_hc_shift[23:20]};
        9'd333: current_char = 8'h30 + {4'd0, turn_event_hc_shift[19:16]};
        9'd334: current_char = " ";
        9'd335: current_char = "E";
        9'd336: current_char = "V";
        9'd337: current_char = "=";
        9'd338: current_char = turn_event_hc_valid_hold ? "1" : "0";
        9'd339: current_char = " ";
        9'd340: current_char = "E";
        9'd341: current_char = "F";
        9'd342: current_char = "=";
        9'd343: current_char = 8'h30 + {4'd0, turn_event_fused_shift[35:32]};
        9'd344: current_char = 8'h30 + {4'd0, turn_event_fused_shift[31:28]};
        9'd345: current_char = 8'h30 + {4'd0, turn_event_fused_shift[27:24]};
        9'd346: current_char = 8'h30 + {4'd0, turn_event_fused_shift[23:20]};
        9'd347: current_char = 8'h30 + {4'd0, turn_event_fused_shift[19:16]};
        9'd348: current_char = " ";
        9'd349: current_char = "E";
        9'd350: current_char = "F";
        9'd351: current_char = "V";
        9'd352: current_char = "=";
        9'd353: current_char = turn_event_fused_valid_hold ? "1" : "0";
        9'd354: current_char = " ";
        9'd355: current_char = "E";
        9'd356: current_char = "R";
        9'd357: current_char = "=";
        9'd358: current_char = 8'h30 + {5'd0, turn_event_reason_hold};
        9'd359: current_char = 8'h0D;
        9'd360: current_char = 8'h0A;

        default:
            current_char = 8'h20;

    endcase

end


//============================================================
// 六、发送状态机
//============================================================

localparam [2:0]

    ST_DELAY        = 3'd0,
    ST_CONVERT      = 3'd1,
    ST_SEND         = 3'd2,
    ST_WAIT_BUSY    = 3'd3,
    ST_WAIT_FINISH  = 3'd4;

reg [2:0] state;


//============================================================
// 七、主时序逻辑
//============================================================

always @(posedge clk or negedge reset_n) begin

    if (!reset_n) begin

        state <= ST_DELAY;

        report_counter <= 24'd0;

        left_shift <= 36'd0;
        right_shift <= 36'd0;
        front_shift <= 36'd0;
        rear_shift <= 36'd0;
        scan_left_shift <= 36'd0;
        scan_center_shift <= 36'd0;
        scan_right_shift <= 36'd0;
        hc_front_shift <= 36'd0;
        hc_age_shift <= 36'd0;
        fused_front_shift <= 36'd0;
        turn_event_tof_shift <= 36'd0;
        turn_event_hc_shift <= 36'd0;
        turn_event_fused_shift <= 36'd0;
        hc_front_valid_hold <= 1'b0;
        fused_front_valid_hold <= 1'b0;
        turn_event_seen_hold <= 1'b0;
        turn_event_tof_valid_hold <= 1'b0;
        turn_event_hc_valid_hold <= 1'b0;
        turn_event_fused_valid_hold <= 1'b0;
        turn_event_reason_hold <= 3'd0;

        yaw32_shift <= 72'd0;
        x32_shift <= 72'd0;
        y32_shift <= 72'd0;
        ea32_shift <= 72'd0;
        eb32_shift <= 72'd0;
        ec32_shift <= 72'd0;
        ed32_shift <= 72'd0;
        yaw_neg_hold <= 1'b0;
        x_neg_hold <= 1'b0;
        y_neg_hold <= 1'b0;
        ea_neg_hold <= 1'b0;
        eb_neg_hold <= 1'b0;
        ec_neg_hold <= 1'b0;
        ed_neg_hold <= 1'b0;

        convert_count <= 6'd0;

        left_ok_hold <= 1'b0;
        right_ok_hold <= 1'b0;
        front_ok_hold <= 1'b0;
        rear_ok_hold <= 1'b0;
        right_error_hold <= 1'b0;
        scan_complete_hold <= 1'b0;

        a5_state_hold <= 4'd0;
        a6_state_hold <= 3'd0;
        waypoint_hold <= 3'd0;
        fault_code_hold <= 3'd0;
        gate_debug_hold <= 3'd0;
        supervisor_debug_hold <= 3'd0;
        detour_direction_hold <= 0;
        detour_fault_hold <= 0;
        turn_block_reason_hold <= 0;
        detour_vx_neg_hold <= 0;
        detour_vy_neg_hold <= 0;
        detour_vx_mag_hold <= 0;
        detour_vy_mag_hold <= 0;
        detour_distance_shift <= 0;

        char_index <= 8'd0;

        tx_data <= 8'd0;
        tx_send <= 1'b0;

    end

    else begin

        // 发送脉冲默认只持续一个系统时钟
        tx_send <= 1'b0;

        case (state)

            //================================================
            // 1. 等待下一个报告周期
            //================================================

            ST_DELAY: begin

                if (report_counter >= REPORT_PERIOD) begin

                    report_counter <= 24'd0;

                    // 缓存当前左右距离
                    // 后续转换和串口输出使用同一份数据
                    left_shift <= {20'd0, left_mm};
                    right_shift <= {20'd0, right_mm};
                    front_shift <= {20'd0, front_mm};
                    rear_shift <= {20'd0, rear_mm};
                    detour_distance_shift <= {20'd0, detour_distance};
                    scan_left_shift <= {20'd0, scan_left_saved};
                    scan_center_shift <= {20'd0, scan_center_saved};
                    scan_right_shift <= {20'd0, scan_right_saved};
                    hc_front_shift <= {20'd0, hc_front_mm};
                    hc_age_shift <= {28'd0, hc_front_age};
                    fused_front_shift <= {20'd0, fused_front_mm};
                    turn_event_tof_shift <= {20'd0, turn_event_tof_mm};
                    turn_event_hc_shift <= {20'd0, turn_event_hc_mm};
                    turn_event_fused_shift <= {20'd0, turn_event_fused_mm};
                    hc_front_valid_hold <= hc_front_valid;
                    fused_front_valid_hold <= fused_front_valid;
                    turn_event_seen_hold <= turn_event_seen;
                    turn_event_tof_valid_hold <= turn_event_tof_valid;
                    turn_event_hc_valid_hold <= turn_event_hc_valid;
                    turn_event_fused_valid_hold <= turn_event_fused_valid;
                    turn_event_reason_hold <= turn_event_reason;


                    // D5 status snapshot; the complete BCD conversion uses this
                    // same frame, so live values cannot change mid-transmission.
                    yaw32_shift <= {40'd0, magnitude32(yaw_cdeg)};
                    x32_shift <= {40'd0, magnitude32(world_x_mm)};
                    y32_shift <= {40'd0, magnitude32(world_y_mm)};
                    ea32_shift <= {40'd0, magnitude32(encoder_a)};
                    eb32_shift <= {40'd0, magnitude32(encoder_b)};
                    ec32_shift <= {40'd0, magnitude32(encoder_c)};
                    ed32_shift <= {40'd0, magnitude32(encoder_d)};
                    yaw_neg_hold <= yaw_cdeg[31];
                    x_neg_hold <= world_x_mm[31];
                    y_neg_hold <= world_y_mm[31];
                    ea_neg_hold <= encoder_a[31];
                    eb_neg_hold <= encoder_b[31];
                    ec_neg_hold <= encoder_c[31];
                    ed_neg_hold <= encoder_d[31];

                    // 同时缓存健康标志和运行状态
                    left_ok_hold <= left_ok;
                    right_ok_hold <= right_ok;
                    front_ok_hold <= front_ok;
                    rear_ok_hold <= rear_ok;
                    right_error_hold <= right_error;
                    scan_complete_hold <= scan_complete && !scan_active;

                    a5_state_hold <= a5_state;
                    a6_state_hold <= a6_state;
                    waypoint_hold <= waypoint_index;
                    fault_code_hold <= fault_code;
                    gate_debug_hold <= gate_debug;
                    supervisor_debug_hold <= supervisor_debug;
                    detour_direction_hold <= detour_direction;
                    detour_fault_hold <= detour_fault;
                    turn_block_reason_hold <= turn_block_reason;
                    detour_vx_neg_hold <= detour_vx[12];
                    detour_vy_neg_hold <= detour_vy[12];
                    detour_vx_mag_hold <= detour_vx[12] ?
                        -detour_vx : detour_vx;
                    detour_vy_mag_hold <= detour_vy[12] ?
                        -detour_vy : detour_vy;

                    convert_count <= 6'd0;

                    state <= ST_CONVERT;

                end

                else begin

                    report_counter <= report_counter + 1'b1;

                end

            end


            //================================================
            // 2. 二进制转BCD
            //
            // 16个时钟周期完成
            //================================================

            ST_CONVERT: begin

                if (convert_count < 6'd16) begin
                    left_shift <= bcd_step(left_shift);
                    right_shift <= bcd_step(right_shift);
                    front_shift <= bcd_step(front_shift);
                    rear_shift <= bcd_step(rear_shift);
                    detour_distance_shift <= bcd_step(detour_distance_shift);
                    scan_left_shift <= bcd_step(scan_left_shift);
                    scan_center_shift <= bcd_step(scan_center_shift);
                    scan_right_shift <= bcd_step(scan_right_shift);
                    hc_front_shift <= bcd_step(hc_front_shift);
                    hc_age_shift <= bcd_step(hc_age_shift);
                    fused_front_shift <= bcd_step(fused_front_shift);
                    turn_event_tof_shift <= bcd_step(turn_event_tof_shift);
                    turn_event_hc_shift <= bcd_step(turn_event_hc_shift);
                    turn_event_fused_shift <= bcd_step(turn_event_fused_shift);

                end

                yaw32_shift <= bcd32_step(yaw32_shift);
                x32_shift <= bcd32_step(x32_shift);
                y32_shift <= bcd32_step(y32_shift);
                ea32_shift <= bcd32_step(ea32_shift);
                eb32_shift <= bcd32_step(eb32_shift);
                ec32_shift <= bcd32_step(ec32_shift);
                ed32_shift <= bcd32_step(ed32_shift);

                if (convert_count == 6'd31) begin

                    convert_count <= 6'd0;

                    char_index <= 8'd0;

                    state <= ST_SEND;

                end

                else begin

                    convert_count <= convert_count + 1'b1;

                end

            end


            //================================================
            // 3. 发出一个UART字符
            //================================================

            ST_SEND: begin

                if (!tx_busy) begin

                    tx_data <= current_char;

                    tx_send <= 1'b1;

                    state <= ST_WAIT_BUSY;

                end

            end


            //================================================
            // 4. 等待发送器进入忙状态
            //================================================

            ST_WAIT_BUSY: begin

                if (tx_busy) begin

                    state <= ST_WAIT_FINISH;

                end

            end


            //================================================
            // 5. 等待当前字符发送完成
            //================================================

            ST_WAIT_FINISH: begin

                if (!tx_busy) begin

                    // D5: four lines, 201 UART characters (index 0~200)
                    if (char_index == 9'd360) begin

                        char_index <= 9'd0;

                        report_counter <= 24'd0;

                        state <= ST_DELAY;

                    end

                    else begin

                        char_index <= char_index + 1'b1;

                        state <= ST_SEND;

                    end

                end

            end


            default: begin

                state <= ST_DELAY;

            end

        endcase

    end

end

endmodule
