////////////////////////////////////////////////////////////////////////////////
// GW5AT60K + VL53L0X 实时测距测试
//
// CLK       : 50MHz
// UART      : 115200 8N1
//
// SDA       : C20
// SCL       : D20
//
// VL53L0X:
// 7-bit I2C = 0x29
// WRITE     = 0x52
// READ      = 0x53
//
// 串口输出：
// DIST=0123 mm
// DIST=0456 mm
//
// XSHUT请继续直接接3.3V
////////////////////////////////////////////////////////////////////////////////

module tof_distance_test_top
(
    input  wire clk,
    input  wire reset_n,

    inout  wire imu_sda,
    inout  wire imu_scl,

    output wire uart_tx
);


////////////////////////////////////////////////////////////////////////////////
// I2C Master
////////////////////////////////////////////////////////////////////////////////

reg        i2c_start;
reg        i2c_read;
reg [7:0]  i2c_reg;
reg [7:0]  i2c_wdata;

wire       i2c_busy;
wire       i2c_done;
wire       i2c_ack_error;
wire [7:0] i2c_rdata;


tof_dist_i2c_master u_i2c_master
(
    .clk        (clk),
    .reset_n    (reset_n),

    .start      (i2c_start),
    .read_op    (i2c_read),

    .reg_addr   (i2c_reg),
    .write_data (i2c_wdata),

    .read_data  (i2c_rdata),

    .busy       (i2c_busy),
    .done       (i2c_done),
    .ack_error  (i2c_ack_error),

    .i2c_scl    (imu_scl),
    .i2c_sda    (imu_sda)
);


////////////////////////////////////////////////////////////////////////////////
// UART
////////////////////////////////////////////////////////////////////////////////

reg       uart_start;
reg [7:0] uart_data;

wire uart_busy;


tof_dist_uart_tx u_uart
(
    .clk     (clk),
    .reset_n (reset_n),

    .start   (uart_start),
    .data    (uart_data),

    .tx      (uart_tx),
    .busy    (uart_busy)
);


////////////////////////////////////////////////////////////////////////////////
// 上电等待
////////////////////////////////////////////////////////////////////////////////

reg [23:0] boot_cnt;


////////////////////////////////////////////////////////////////////////////////
// 初始化数据
////////////////////////////////////////////////////////////////////////////////

reg [7:0] stop_variable;

reg [7:0] temp_reg;

reg [7:0] spad_info;

wire [6:0] spad_count;
wire       spad_aperture;

assign spad_count    = spad_info[6:0];
assign spad_aperture = spad_info[7];


////////////////////////////////////////////////////////////////////////////////
// SPAD Map
////////////////////////////////////////////////////////////////////////////////

reg [47:0] spad_map;

reg [47:0] spad_map_masked;

integer spad_i;
integer spad_enabled;


always @*
begin

    spad_map_masked = spad_map;

    spad_enabled = 0;

    for(spad_i = 0; spad_i < 48; spad_i = spad_i + 1)
    begin

        if
        (
            (spad_i < (spad_aperture ? 12 : 0))
            ||
            (spad_enabled >= spad_count)
        )
        begin

            spad_map_masked[spad_i] = 1'b0;

        end

        else if(spad_map[spad_i])
        begin

            spad_enabled = spad_enabled + 1;

        end

    end

end


////////////////////////////////////////////////////////////////////////////////
// 初始化表
////////////////////////////////////////////////////////////////////////////////

function [15:0] tune_data;

    input [6:0] index;

    begin

        case(index)

        7'd0 : tune_data = 16'hFF01;
        7'd1 : tune_data = 16'h0000;
        7'd2 : tune_data = 16'hFF00;
        7'd3 : tune_data = 16'h0900;
        7'd4 : tune_data = 16'h1000;
        7'd5 : tune_data = 16'h1100;
        7'd6 : tune_data = 16'h2401;
        7'd7 : tune_data = 16'h25FF;
        7'd8 : tune_data = 16'h7500;

        7'd9 : tune_data = 16'hFF01;
        7'd10: tune_data = 16'h4E2C;
        7'd11: tune_data = 16'h4800;
        7'd12: tune_data = 16'h3020;

        7'd13: tune_data = 16'hFF00;
        7'd14: tune_data = 16'h3009;
        7'd15: tune_data = 16'h5400;
        7'd16: tune_data = 16'h3104;
        7'd17: tune_data = 16'h3203;
        7'd18: tune_data = 16'h4083;
        7'd19: tune_data = 16'h4625;
        7'd20: tune_data = 16'h6000;
        7'd21: tune_data = 16'h2700;
        7'd22: tune_data = 16'h5006;
        7'd23: tune_data = 16'h5100;
        7'd24: tune_data = 16'h5296;
        7'd25: tune_data = 16'h5608;
        7'd26: tune_data = 16'h5730;
        7'd27: tune_data = 16'h6100;
        7'd28: tune_data = 16'h6200;
        7'd29: tune_data = 16'h6400;
        7'd30: tune_data = 16'h6500;
        7'd31: tune_data = 16'h66A0;

        7'd32: tune_data = 16'hFF01;
        7'd33: tune_data = 16'h2232;
        7'd34: tune_data = 16'h4714;
        7'd35: tune_data = 16'h49FF;
        7'd36: tune_data = 16'h4A00;

        7'd37: tune_data = 16'hFF00;
        7'd38: tune_data = 16'h7A0A;
        7'd39: tune_data = 16'h7B00;
        7'd40: tune_data = 16'h7821;

        7'd41: tune_data = 16'hFF01;
        7'd42: tune_data = 16'h2334;
        7'd43: tune_data = 16'h4200;
        7'd44: tune_data = 16'h44FF;
        7'd45: tune_data = 16'h4526;
        7'd46: tune_data = 16'h4605;
        7'd47: tune_data = 16'h4040;
        7'd48: tune_data = 16'h0E06;
        7'd49: tune_data = 16'h201A;
        7'd50: tune_data = 16'h4340;

        7'd51: tune_data = 16'hFF00;
        7'd52: tune_data = 16'h3403;
        7'd53: tune_data = 16'h3544;

        7'd54: tune_data = 16'hFF01;
        7'd55: tune_data = 16'h3104;
        7'd56: tune_data = 16'h4B09;
        7'd57: tune_data = 16'h4C05;
        7'd58: tune_data = 16'h4D04;

        7'd59: tune_data = 16'hFF00;
        7'd60: tune_data = 16'h4400;
        7'd61: tune_data = 16'h4520;
        7'd62: tune_data = 16'h4708;
        7'd63: tune_data = 16'h4828;
        7'd64: tune_data = 16'h6700;
        7'd65: tune_data = 16'h7004;
        7'd66: tune_data = 16'h7101;
        7'd67: tune_data = 16'h72FE;
        7'd68: tune_data = 16'h7600;
        7'd69: tune_data = 16'h7700;

        7'd70: tune_data = 16'hFF01;
        7'd71: tune_data = 16'h0D01;

        7'd72: tune_data = 16'hFF00;
        7'd73: tune_data = 16'h8001;
        7'd74: tune_data = 16'h01F8;

        7'd75: tune_data = 16'hFF01;
        7'd76: tune_data = 16'h8E01;
        7'd77: tune_data = 16'h0001;

        7'd78: tune_data = 16'hFF00;
        7'd79: tune_data = 16'h8000;

        default:
            tune_data = 16'h0000;

        endcase

    end

endfunction


////////////////////////////////////////////////////////////////////////////////
// Controller States
////////////////////////////////////////////////////////////////////////////////

localparam
    S_BOOT            = 7'd0,

    S_ID              = 7'd1,

    S_PRE              = 7'd2,
    S_STOPVAR          = 7'd3,
    S_POST             = 7'd4,

    S_MSRC_READ        = 7'd5,
    S_MSRC_WRITE       = 7'd6,

    S_BASECFG          = 7'd7,

    S_SPAD_PRE1        = 7'd8,
    S_SPAD_R83         = 7'd9,
    S_SPAD_W83         = 7'd10,
    S_SPAD_PRE2        = 7'd11,

    S_SPAD_POLL        = 7'd12,
    S_SPAD_83_ONE      = 7'd13,
    S_SPAD_R92         = 7'd14,

    S_SPAD_CLEAN1      = 7'd15,
    S_SPAD_R83_2       = 7'd16,
    S_SPAD_W83_2       = 7'd17,
    S_SPAD_CLEAN2      = 7'd18,

    S_MAP_READ         = 7'd19,
    S_MAP_CFG          = 7'd20,
    S_MAP_WRITE        = 7'd21,

    S_TUNING           = 7'd22,

    S_INT_CFG1         = 7'd23,
    S_INT_R84          = 7'd24,
    S_INT_W84          = 7'd25,
    S_INT_CFG2         = 7'd26,

    S_CAL1_CFG         = 7'd27,
    S_CAL1_START       = 7'd28,
    S_CAL1_POLL        = 7'd29,
    S_CAL1_END         = 7'd30,

    S_CAL2_CFG         = 7'd31,
    S_CAL2_START       = 7'd32,
    S_CAL2_POLL        = 7'd33,
    S_CAL2_END         = 7'd34,

    S_RESTORE          = 7'd35,

    S_CONTINUOUS       = 7'd36,

    S_RUN_STATUS       = 7'd37,
    S_RUN_HIGH         = 7'd38,
    S_RUN_LOW          = 7'd39,
    S_RUN_CLEAR        = 7'd40,

    S_ERROR            = 7'd63;


reg [6:0] state;

reg [6:0] seq_index;

reg [6:0] tune_index;

reg [2:0] map_index;

reg op_active;

reg [15:0] poll_timeout;


////////////////////////////////////////////////////////////////////////////////
// 距离
////////////////////////////////////////////////////////////////////////////////

reg [7:0] distance_high;

reg [15:0] distance_mm;
reg        distance_valid;

////////////////////////////////////////////////////////////////////////////////
// 诊断寄存器
//
// 用于排查：
// 1) ID 能否读取
// 2) 初始化卡在哪个状态
// 3) 最近一次 I2C 访问的寄存器/返回值
// 4) 是否发生 ACK 错误
// 5) 是否曾经得到过有效距离
////////////////////////////////////////////////////////////////////////////////

reg [6:0] debug_last_state;
reg [7:0] debug_last_reg;
reg [7:0] debug_last_rdata;
reg       debug_last_ack;
reg [7:0] debug_device_id;
reg       debug_distance_seen;

wire [6:0] debug_state;
wire       debug_error;

assign debug_state =
       (state == S_ERROR)
       ? debug_last_state
       : state;

assign debug_error =
       (state == S_ERROR);


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        debug_last_state    <= 7'd0;
        debug_last_reg      <= 8'h00;
        debug_last_rdata    <= 8'h00;
        debug_last_ack      <= 1'b0;
        debug_device_id     <= 8'h00;
        debug_distance_seen <= 1'b0;

    end

    else
    begin

        // 如果最终进入 S_ERROR，保留进入错误前的最后状态
        if(state != S_ERROR)
            debug_last_state <= state;

        // 捕获最近一次 I2C 事务结果
        if(i2c_done)
        begin

            debug_last_reg   <= i2c_reg;
            debug_last_rdata <= i2c_rdata;
            debug_last_ack   <= i2c_ack_error;

            // 0xC0 = VL53L0X DEVICE ID
            if(i2c_read && (i2c_reg == 8'hC0))
                debug_device_id <= i2c_rdata;

        end

        if(distance_valid)
            debug_distance_seen <= 1'b1;

    end

end

////////////////////////////////////////////////////////////////////////////////
// 标定后的距离
////////////////////////////////////////////////////////////////////////////////

wire [15:0] distance_corrected_mm;


tof_distance_calibration
u_tof_distance_calibration
(
    .raw_distance_mm
    (
        distance_mm
    ),

    .corrected_distance_mm
    (
        distance_corrected_mm
    )
);


////////////////////////////////////////////////////////////////////////////////
// 当前初始化字
////////////////////////////////////////////////////////////////////////////////

wire [15:0] current_tuning;

assign current_tuning = tune_data(tune_index);


////////////////////////////////////////////////////////////////////////////////
// 小初始化表
////////////////////////////////////////////////////////////////////////////////

function [15:0] pre_data;
    input [2:0] index;
    begin
        case(index)

        3'd0: pre_data = 16'h8901;
        3'd1: pre_data = 16'h8800;
        3'd2: pre_data = 16'h8001;
        3'd3: pre_data = 16'hFF01;
        3'd4: pre_data = 16'h0000;

        default:
            pre_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] post_data;
    input [1:0] index;
    begin
        case(index)

        2'd0: post_data = 16'h0001;
        2'd1: post_data = 16'hFF00;
        2'd2: post_data = 16'h8000;

        default:
            post_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] base_cfg_data;
    input [1:0] index;
    begin
        case(index)

        // Signal rate limit = 0.25 MCPS = 0x0020
        2'd0: base_cfg_data = 16'h4400;
        2'd1: base_cfg_data = 16'h4520;

        // Sequence Config
        2'd2: base_cfg_data = 16'h01FF;

        default:
            base_cfg_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] spad_pre1_data;
    input [2:0] index;
    begin
        case(index)

        3'd0: spad_pre1_data = 16'h8001;
        3'd1: spad_pre1_data = 16'hFF01;
        3'd2: spad_pre1_data = 16'h0000;
        3'd3: spad_pre1_data = 16'hFF06;

        default:
            spad_pre1_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] spad_pre2_data;
    input [2:0] index;
    begin
        case(index)

        3'd0: spad_pre2_data = 16'hFF07;
        3'd1: spad_pre2_data = 16'h8101;
        3'd2: spad_pre2_data = 16'h8001;
        3'd3: spad_pre2_data = 16'h946B;
        3'd4: spad_pre2_data = 16'h8300;

        default:
            spad_pre2_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] spad_clean1_data;
    input [0:0] index;
    begin
        case(index)

        1'd0: spad_clean1_data = 16'h8100;
        1'd1: spad_clean1_data = 16'hFF06;

        default:
            spad_clean1_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] spad_clean2_data;
    input [1:0] index;
    begin
        case(index)

        2'd0: spad_clean2_data = 16'hFF01;
        2'd1: spad_clean2_data = 16'h0001;
        2'd2: spad_clean2_data = 16'hFF00;
        2'd3: spad_clean2_data = 16'h8000;

        default:
            spad_clean2_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] map_cfg_data;
    input [2:0] index;
    begin
        case(index)

        3'd0: map_cfg_data = 16'hFF01;
        3'd1: map_cfg_data = 16'h4F00;
        3'd2: map_cfg_data = 16'h4E2C;
        3'd3: map_cfg_data = 16'hFF00;
        3'd4: map_cfg_data = 16'hB6B4;

        default:
            map_cfg_data = 16'h0000;

        endcase
    end
endfunction


function [7:0] masked_map_byte;
    input [2:0] index;
    begin
        case(index)

        3'd0: masked_map_byte = spad_map_masked[7:0];
        3'd1: masked_map_byte = spad_map_masked[15:8];
        3'd2: masked_map_byte = spad_map_masked[23:16];
        3'd3: masked_map_byte = spad_map_masked[31:24];
        3'd4: masked_map_byte = spad_map_masked[39:32];
        3'd5: masked_map_byte = spad_map_masked[47:40];

        default:
            masked_map_byte = 8'h00;

        endcase
    end
endfunction


function [15:0] int_cfg1_data;
    input [0:0] index;
    begin
        case(index)

        1'd0: int_cfg1_data = 16'h0A04;

        default:
            int_cfg1_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] int_cfg2_data;
    input [1:0] index;
    begin
        case(index)

        2'd0: int_cfg2_data = 16'h0B01;
        2'd1: int_cfg2_data = 16'h01E8;

        default:
            int_cfg2_data = 16'h0000;

        endcase
    end
endfunction


function [15:0] continuous_data;
    input [3:0] index;
    input [7:0] stop_var;

    begin
        case(index)

        4'd0: continuous_data = 16'h8001;
        4'd1: continuous_data = 16'hFF01;
        4'd2: continuous_data = 16'h0000;

        // 必须恢复初始化时读取的stop_variable
        4'd3: continuous_data = {8'h91, stop_var};

        4'd4: continuous_data = 16'h0001;
        4'd5: continuous_data = 16'hFF00;
        4'd6: continuous_data = 16'h8000;

        // Back-to-back连续测距
        4'd7: continuous_data = 16'h0002;

        default:
            continuous_data = 16'h0000;

        endcase
    end
endfunction


////////////////////////////////////////////////////////////////////////////////
// 方便访问表
////////////////////////////////////////////////////////////////////////////////

wire [15:0] current_pre;
wire [15:0] current_post;
wire [15:0] current_base_cfg;
wire [15:0] current_spad_pre1;
wire [15:0] current_spad_pre2;
wire [15:0] current_spad_clean1;
wire [15:0] current_spad_clean2;
wire [15:0] current_map_cfg;
wire [15:0] current_int_cfg1;
wire [15:0] current_int_cfg2;
wire [15:0] current_cont;

assign current_pre =
       pre_data(seq_index[2:0]);

assign current_post =
       post_data(seq_index[1:0]);

assign current_base_cfg =
       base_cfg_data(seq_index[1:0]);

assign current_spad_pre1 =
       spad_pre1_data(seq_index[2:0]);

assign current_spad_pre2 =
       spad_pre2_data(seq_index[2:0]);

assign current_spad_clean1 =
       spad_clean1_data(seq_index[0]);

assign current_spad_clean2 =
       spad_clean2_data(seq_index[1:0]);

assign current_map_cfg =
       map_cfg_data(seq_index[2:0]);

assign current_int_cfg1 =
       int_cfg1_data(seq_index[0]);

assign current_int_cfg2 =
       int_cfg2_data(seq_index[1:0]);

assign current_cont =
       continuous_data(seq_index[3:0], stop_variable);


////////////////////////////////////////////////////////////////////////////////
// 主控制
////////////////////////////////////////////////////////////////////////////////

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        state <= S_BOOT;

        boot_cnt <= 24'd0;

        i2c_start <= 1'b0;
        i2c_read  <= 1'b0;
        i2c_reg   <= 8'h00;
        i2c_wdata <= 8'h00;

        op_active <= 1'b0;

        seq_index  <= 7'd0;
        tune_index <= 7'd0;
        map_index  <= 3'd0;

        stop_variable <= 8'h00;
        temp_reg      <= 8'h00;
        spad_info     <= 8'h00;

        spad_map <= 48'd0;

        poll_timeout <= 16'd0;

        distance_high <= 8'h00;
        distance_mm   <= 16'd0;
        distance_valid <= 1'b0;

    end

    else
    begin

        i2c_start <= 1'b0;

        distance_valid <= 1'b0;


        case(state)

        ////////////////////////////////////////////////////////////////////////
        // 上电等待100ms
        ////////////////////////////////////////////////////////////////////////

        S_BOOT:
        begin

            if(boot_cnt >= 24'd4_999_999)
            begin

                boot_cnt <= 24'd0;

                op_active <= 1'b0;

                state <= S_ID;

            end

            else
            begin

                boot_cnt <= boot_cnt + 1'b1;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 读取ID 0xC0
        ////////////////////////////////////////////////////////////////////////

        S_ID:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;
                i2c_reg  <= 8'hC0;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if
                (
                    i2c_ack_error
                    ||
                    i2c_rdata != 8'hEE
                )
                begin

                    state <= S_ERROR;

                end

                else
                begin

                    seq_index <= 7'd0;

                    state <= S_PRE;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 初始寄存器
        ////////////////////////////////////////////////////////////////////////

        S_PRE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read  <= 1'b0;

                i2c_reg   <= current_pre[15:8];
                i2c_wdata <= current_pre[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_ack_error)
                begin

                    state <= S_ERROR;

                end

                else if(seq_index == 7'd4)
                begin

                    seq_index <= 7'd0;

                    state <= S_STOPVAR;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 读stop_variable = 0x91
        ////////////////////////////////////////////////////////////////////////

        S_STOPVAR:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;
                i2c_reg  <= 8'h91;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_ack_error)
                begin

                    state <= S_ERROR;

                end

                else
                begin

                    stop_variable <= i2c_rdata;

                    seq_index <= 7'd0;

                    state <= S_POST;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 恢复Page
        ////////////////////////////////////////////////////////////////////////

        S_POST:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read  <= 1'b0;

                i2c_reg   <= current_post[15:8];
                i2c_wdata <= current_post[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_ack_error)
                begin

                    state <= S_ERROR;

                end

                else if(seq_index == 7'd2)
                begin

                    state <= S_MSRC_READ;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Read MSRC_CONFIG_CONTROL
        ////////////////////////////////////////////////////////////////////////

        S_MSRC_READ:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;
                i2c_reg  <= 8'h60;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                temp_reg <= i2c_rdata;

                state <= S_MSRC_WRITE;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Disable MSRC/PRE RANGE signal rate checks
        ////////////////////////////////////////////////////////////////////////

        S_MSRC_WRITE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read  <= 1'b0;
                i2c_reg   <= 8'h60;

                i2c_wdata <= temp_reg | 8'h12;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                seq_index <= 7'd0;

                state <= S_BASECFG;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Signal Rate / Sequence Config
        ////////////////////////////////////////////////////////////////////////

        S_BASECFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read  <= 1'b0;

                i2c_reg   <= current_base_cfg[15:8];
                i2c_wdata <= current_base_cfg[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd2)
                begin

                    seq_index <= 7'd0;

                    state <= S_SPAD_PRE1;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // SPAD INFO前半
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_PRE1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_spad_pre1[15:8];
                i2c_wdata <= current_spad_pre1[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd3)
                begin

                    state <= S_SPAD_R83;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 读0x83
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_R83:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;
                i2c_reg  <= 8'h83;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                temp_reg <= i2c_rdata;

                state <= S_SPAD_W83;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 0x83 bit2置1
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_W83:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h83;
                i2c_wdata <= temp_reg | 8'h04;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                seq_index <= 7'd0;

                state <= S_SPAD_PRE2;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // SPAD第二阶段
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_PRE2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_spad_pre2[15:8];
                i2c_wdata <= current_spad_pre2[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd4)
                begin

                    poll_timeout <= 16'd0;

                    state <= S_SPAD_POLL;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 等0x83 != 0
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;
                i2c_reg  <= 8'h83;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_rdata != 8'h00)
                begin

                    state <= S_SPAD_83_ONE;

                end

                else
                begin

                    poll_timeout <= poll_timeout + 1'b1;

                    if(poll_timeout > 16'd1000)
                        state <= S_ERROR;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 0x83 = 1
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_83_ONE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h83;
                i2c_wdata <= 8'h01;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                state <= S_SPAD_R92;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 读取SPAD数量和类型
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_R92:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h92;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                spad_info <= i2c_rdata;

                seq_index <= 7'd0;

                state <= S_SPAD_CLEAN1;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // SPAD Cleanup 1
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_CLEAN1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_spad_clean1[15:8];
                i2c_wdata <= current_spad_clean1[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd1)
                begin

                    state <= S_SPAD_R83_2;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 再读0x83
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_R83_2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h83;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                temp_reg <= i2c_rdata;

                state <= S_SPAD_W83_2;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Clear 0x83 bit2
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_W83_2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg <= 8'h83;

                i2c_wdata <= temp_reg & 8'hFB;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                seq_index <= 7'd0;

                state <= S_SPAD_CLEAN2;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // SPAD Cleanup 2
        ////////////////////////////////////////////////////////////////////////

        S_SPAD_CLEAN2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_spad_clean2[15:8];
                i2c_wdata <= current_spad_clean2[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd3)
                begin

                    map_index <= 3'd0;

                    state <= S_MAP_READ;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 读 B0~B5
        ////////////////////////////////////////////////////////////////////////

        S_MAP_READ:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'hB0 + map_index;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                case(map_index)

                    3'd0:
                        spad_map[7:0] <= i2c_rdata;

                    3'd1:
                        spad_map[15:8] <= i2c_rdata;

                    3'd2:
                        spad_map[23:16] <= i2c_rdata;

                    3'd3:
                        spad_map[31:24] <= i2c_rdata;

                    3'd4:
                        spad_map[39:32] <= i2c_rdata;

                    3'd5:
                        spad_map[47:40] <= i2c_rdata;

                endcase


                if(map_index == 3'd5)
                begin

                    seq_index <= 7'd0;

                    state <= S_MAP_CFG;

                end

                else
                begin

                    map_index <= map_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // SPAD Config
        ////////////////////////////////////////////////////////////////////////

        S_MAP_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_map_cfg[15:8];
                i2c_wdata <= current_map_cfg[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd4)
                begin

                    map_index <= 3'd0;

                    state <= S_MAP_WRITE;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 写回B0~B5
        ////////////////////////////////////////////////////////////////////////

        S_MAP_WRITE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg <= 8'hB0 + map_index;

                i2c_wdata <= masked_map_byte(map_index);

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(map_index == 3'd5)
                begin

                    tune_index <= 7'd0;

                    state <= S_TUNING;

                end

                else
                begin

                    map_index <= map_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Tuning表
        ////////////////////////////////////////////////////////////////////////

        S_TUNING:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_tuning[15:8];
                i2c_wdata <= current_tuning[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(tune_index == 7'd79)
                begin

                    seq_index <= 7'd0;

                    state <= S_INT_CFG1;

                end

                else
                begin

                    tune_index <= tune_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Interrupt Config
        ////////////////////////////////////////////////////////////////////////

        S_INT_CFG1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_int_cfg1[15:8];
                i2c_wdata <= current_int_cfg1[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                state <= S_INT_R84;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Read GPIO_HV_MUX_ACTIVE_HIGH
        ////////////////////////////////////////////////////////////////////////

        S_INT_R84:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h84;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                temp_reg <= i2c_rdata;

                state <= S_INT_W84;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Active low
        ////////////////////////////////////////////////////////////////////////

        S_INT_W84:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg <= 8'h84;

                i2c_wdata <= temp_reg & 8'hEF;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                seq_index <= 7'd0;

                state <= S_INT_CFG2;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Clear Interrupt + Sequence E8
        ////////////////////////////////////////////////////////////////////////

        S_INT_CFG2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_int_cfg2[15:8];
                i2c_wdata <= current_int_cfg2[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd1)
                begin

                    state <= S_CAL1_CFG;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Calibration 1
        ////////////////////////////////////////////////////////////////////////

        S_CAL1_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h01;
                i2c_wdata <= 8'h01;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                state <= S_CAL1_START;

            end

        end


        S_CAL1_START:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                // 0x01 | 0x40
                i2c_reg   <= 8'h00;
                i2c_wdata <= 8'h41;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                poll_timeout <= 16'd0;

                state <= S_CAL1_POLL;

            end

        end


        S_CAL1_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h13;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_rdata[2:0] != 3'b000)
                begin

                    seq_index <= 7'd0;

                    state <= S_CAL1_END;

                end

                else
                begin

                    poll_timeout <= poll_timeout + 1'b1;

                    if(poll_timeout > 16'd2000)
                        state <= S_ERROR;

                end

            end

        end


        S_CAL1_END:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                if(seq_index == 7'd0)
                begin
                    i2c_reg   <= 8'h0B;
                    i2c_wdata <= 8'h01;
                end
                else
                begin
                    i2c_reg   <= 8'h00;
                    i2c_wdata <= 8'h00;
                end

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd1)
                begin

                    state <= S_CAL2_CFG;

                end
                else
                begin

                    seq_index <= 7'd1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Calibration 2
        ////////////////////////////////////////////////////////////////////////

        S_CAL2_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h01;
                i2c_wdata <= 8'h02;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                state <= S_CAL2_START;

            end

        end


        S_CAL2_START:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h00;
                i2c_wdata <= 8'h01;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                poll_timeout <= 16'd0;

                state <= S_CAL2_POLL;

            end

        end


        S_CAL2_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h13;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_rdata[2:0] != 3'b000)
                begin

                    seq_index <= 7'd0;

                    state <= S_CAL2_END;

                end

                else
                begin

                    poll_timeout <= poll_timeout + 1'b1;

                    if(poll_timeout > 16'd2000)
                        state <= S_ERROR;

                end

            end

        end


        S_CAL2_END:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                if(seq_index == 7'd0)
                begin

                    i2c_reg   <= 8'h0B;
                    i2c_wdata <= 8'h01;

                end

                else
                begin

                    i2c_reg   <= 8'h00;
                    i2c_wdata <= 8'h00;

                end

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd1)
                begin

                    state <= S_RESTORE;

                end
                else
                begin

                    seq_index <= 7'd1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 恢复Sequence Config
        ////////////////////////////////////////////////////////////////////////

        S_RESTORE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h01;
                i2c_wdata <= 8'hE8;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                seq_index <= 7'd0;

                state <= S_CONTINUOUS;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 启动连续测距
        ////////////////////////////////////////////////////////////////////////

        S_CONTINUOUS:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= current_cont[15:8];
                i2c_wdata <= current_cont[7:0];

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(seq_index == 7'd7)
                begin

                    state <= S_RUN_STATUS;

                end

                else
                begin

                    seq_index <= seq_index + 1'b1;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 测量完成？
        ////////////////////////////////////////////////////////////////////////

        S_RUN_STATUS:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h13;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                if(i2c_ack_error)
                begin

                    state <= S_ERROR;

                end

                else if(i2c_rdata[2:0] != 3'b000)
                begin

                    state <= S_RUN_HIGH;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Distance High Byte
        //
        // RESULT_RANGE_STATUS + 10
        // 0x14 + 10 = 0x1E
        ////////////////////////////////////////////////////////////////////////

        S_RUN_HIGH:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h1E;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                distance_high <= i2c_rdata;

                state <= S_RUN_LOW;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Distance Low Byte
        ////////////////////////////////////////////////////////////////////////

        S_RUN_LOW:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b1;

                i2c_reg <= 8'h1F;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                distance_mm <=
                {
                    distance_high,
                    i2c_rdata
                };

                distance_valid <= 1'b1;

                state <= S_RUN_CLEAR;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Clear interrupt
        ////////////////////////////////////////////////////////////////////////

        S_RUN_CLEAR:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <= 1'b0;

                i2c_reg   <= 8'h0B;
                i2c_wdata <= 8'h01;

                i2c_start <= 1'b1;

                op_active <= 1'b1;

            end

            else if(op_active && i2c_done)
            begin

                op_active <= 1'b0;

                state <= S_RUN_STATUS;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Error
        ////////////////////////////////////////////////////////////////////////

        S_ERROR:
        begin

            state <= S_ERROR;

        end


        default:
        begin

            state <= S_BOOT;

        end

        endcase

    end

end


////////////////////////////////////////////////////////////////////////////////
// UART 诊断报告
//
// 每 500ms 输出一行：
//
// ID=EE ST=37 E=0 ACK=0 REG=13 RD=07 D=0240 V=1
//
// ID   : 0xC0 读取到的器件ID，VL53L0X正常通常为 EE
// ST   : 当前状态；如果已进入 S_ERROR，则显示出错前最后状态
// E    : 1=已经进入 S_ERROR，0=未进入错误状态
// ACK  : 最近一次 I2C 事务是否 ACK 错误
// REG  : 最近一次访问的寄存器地址（HEX）
// RD   : 最近一次 I2C 读取的数据（HEX）
// D    : 当前原始距离（mm）
// V    : 是否曾经产生过一次 distance_valid
////////////////////////////////////////////////////////////////////////////////

reg        report_active;
reg [5:0]  report_index;
reg [1:0]  report_phase;
reg [24:0] report_timer;

reg [7:0]  report_id;
reg [6:0]  report_state;
reg        report_error;
reg        report_ack;
reg [7:0]  report_reg;
reg [7:0]  report_rdata;
reg [15:0] report_distance;
reg        report_seen;

wire [3:0] state_tens;
wire [3:0] state_ones;

wire [3:0] dist_thousand;
wire [3:0] dist_hundred;
wire [3:0] dist_ten;
wire [3:0] dist_one;

assign state_tens = (report_state / 10) % 10;
assign state_ones = report_state % 10;

assign dist_thousand = (report_distance / 1000) % 10;
assign dist_hundred  = (report_distance / 100)  % 10;
assign dist_ten      = (report_distance / 10)   % 10;
assign dist_one      = report_distance % 10;


function [7:0] digit_ascii;

    input [3:0] digit;

    begin
        digit_ascii = 8'h30 + digit;
    end

endfunction


function [7:0] hex_ascii;

    input [3:0] nibble;

    begin

        if(nibble < 10)
            hex_ascii = 8'h30 + nibble;
        else
            hex_ascii = 8'h41 + (nibble - 10);

    end

endfunction


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        uart_start <= 1'b0;
        uart_data  <= 8'h00;

        report_active <= 1'b0;
        report_index  <= 6'd0;
        report_phase  <= 2'd0;
        report_timer  <= 25'd0;

        report_id       <= 8'h00;
        report_state    <= 7'd0;
        report_error    <= 1'b0;
        report_ack      <= 1'b0;
        report_reg      <= 8'h00;
        report_rdata    <= 8'h00;
        report_distance <= 16'd0;
        report_seen     <= 1'b0;

    end

    else
    begin

        uart_start <= 1'b0;

        ////////////////////////////////////////////////////////////////////////
        // 每 500ms 启动一次诊断输出
        // 50MHz * 0.5s = 25,000,000 clocks
        ////////////////////////////////////////////////////////////////////////

        if(!report_active)
        begin

            if(report_timer >= 25'd24_999_999)
            begin

                report_timer <= 25'd0;

                report_id       <= debug_device_id;
                report_state    <= debug_state;
                report_error    <= debug_error;
                report_ack      <= debug_last_ack;
                report_reg      <= debug_last_reg;
                report_rdata    <= debug_last_rdata;
                report_distance <= distance_mm;
                report_seen     <= debug_distance_seen;

                report_index  <= 6'd0;
                report_phase  <= 2'd0;
                report_active <= 1'b1;

            end
            else
            begin

                report_timer <= report_timer + 1'b1;

            end

        end

        ////////////////////////////////////////////////////////////////////////
        // UART发送状态机
        ////////////////////////////////////////////////////////////////////////

        if(report_active)
        begin

            case(report_phase)

            // 准备字符并拉高 start 一个clk
            2'd0:
            begin

                if(!uart_busy)
                begin

                    case(report_index)

                    // ID=EE
                    6'd0 : uart_data <= "I";
                    6'd1 : uart_data <= "D";
                    6'd2 : uart_data <= "=";
                    6'd3 : uart_data <= hex_ascii(report_id[7:4]);
                    6'd4 : uart_data <= hex_ascii(report_id[3:0]);
                    6'd5 : uart_data <= " ";

                    // ST=37
                    6'd6 : uart_data <= "S";
                    6'd7 : uart_data <= "T";
                    6'd8 : uart_data <= "=";
                    6'd9 : uart_data <= digit_ascii(state_tens);
                    6'd10: uart_data <= digit_ascii(state_ones);
                    6'd11: uart_data <= " ";

                    // E=0
                    6'd12: uart_data <= "E";
                    6'd13: uart_data <= "=";
                    6'd14: uart_data <= report_error ? "1" : "0";
                    6'd15: uart_data <= " ";

                    // ACK=0
                    6'd16: uart_data <= "A";
                    6'd17: uart_data <= "C";
                    6'd18: uart_data <= "K";
                    6'd19: uart_data <= "=";
                    6'd20: uart_data <= report_ack ? "1" : "0";
                    6'd21: uart_data <= " ";

                    // REG=13
                    6'd22: uart_data <= "R";
                    6'd23: uart_data <= "E";
                    6'd24: uart_data <= "G";
                    6'd25: uart_data <= "=";
                    6'd26: uart_data <= hex_ascii(report_reg[7:4]);
                    6'd27: uart_data <= hex_ascii(report_reg[3:0]);
                    6'd28: uart_data <= " ";

                    // RD=00
                    6'd29: uart_data <= "R";
                    6'd30: uart_data <= "D";
                    6'd31: uart_data <= "=";
                    6'd32: uart_data <= hex_ascii(report_rdata[7:4]);
                    6'd33: uart_data <= hex_ascii(report_rdata[3:0]);
                    6'd34: uart_data <= " ";

                    // D=0000
                    6'd35: uart_data <= "D";
                    6'd36: uart_data <= "=";
                    6'd37: uart_data <= digit_ascii(dist_thousand);
                    6'd38: uart_data <= digit_ascii(dist_hundred);
                    6'd39: uart_data <= digit_ascii(dist_ten);
                    6'd40: uart_data <= digit_ascii(dist_one);
                    6'd41: uart_data <= " ";

                    // V=0
                    6'd42: uart_data <= "V";
                    6'd43: uart_data <= "=";
                    6'd44: uart_data <= report_seen ? "1" : "0";

                    // CR LF
                    6'd45: uart_data <= 8'h0D;
                    6'd46: uart_data <= 8'h0A;

                    default:
                        uart_data <= 8'h20;

                    endcase

                    uart_start <= 1'b1;
                    report_phase <= 2'd1;

                end

            end

            // 等待UART接收start并进入busy
            2'd1:
            begin

                if(uart_busy)
                    report_phase <= 2'd2;

            end

            // 等待当前字符发送结束
            2'd2:
            begin

                if(!uart_busy)
                begin

                    if(report_index == 6'd46)
                    begin

                        report_active <= 1'b0;
                        report_index  <= 6'd0;
                        report_phase  <= 2'd0;

                    end
                    else
                    begin

                        report_index <= report_index + 1'b1;
                        report_phase <= 2'd0;

                    end

                end

            end

            default:
                report_phase <= 2'd0;

            endcase

        end

    end

end


endmodule



////////////////////////////////////////////////////////////////////////////////
//
// VL53L0X I2C MASTER
//
// 50MHz -> approximately 100kHz I2C
//
// Register Address = 8-bit
//
// Device Write = 0x52
// Device Read  = 0x53
//
////////////////////////////////////////////////////////////////////////////////

module tof_dist_i2c_master
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire read_op,

    input wire [7:0] reg_addr,

    input wire [7:0] write_data,

    output reg [7:0] read_data,

    output reg busy,
    output reg done,
    output reg ack_error,

    inout wire i2c_scl,
    inout wire i2c_sda
);


////////////////////////////////////////////////////////////////////////////////
// Open Drain
////////////////////////////////////////////////////////////////////////////////

reg scl_low;
reg sda_low;


assign i2c_scl =
       scl_low
       ?
       1'b0
       :
       1'bz;


assign i2c_sda =
       sda_low
       ?
       1'b0
       :
       1'bz;


wire sda_in;

assign sda_in = i2c_sda;


////////////////////////////////////////////////////////////////////////////////
// I2C Timing
//
// 50MHz
//
// 250 clocks = 5us
////////////////////////////////////////////////////////////////////////////////

reg [8:0] div_cnt;

wire tick;


assign tick =
       (div_cnt == 9'd249);


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        div_cnt <= 9'd0;

    end

    else if(!busy)
    begin

        div_cnt <= 9'd0;

    end

    else
    begin

        if(tick)
            div_cnt <= 9'd0;
        else
            div_cnt <= div_cnt + 1'b1;

    end

end


////////////////////////////////////////////////////////////////////////////////
// States
////////////////////////////////////////////////////////////////////////////////

localparam
    I_IDLE        = 5'd0,

    I_START1      = 5'd1,
    I_START2      = 5'd2,

    I_TX_LOW      = 5'd3,
    I_TX_HIGH     = 5'd4,

    I_ACK_LOW     = 5'd5,
    I_ACK_HIGH    = 5'd6,
    I_ACK_SAMPLE  = 5'd7,

    I_REP_LOW     = 5'd8,
    I_REP_HIGH    = 5'd9,
    I_REP_START   = 5'd10,

    I_RX_LOW      = 5'd11,
    I_RX_HIGH     = 5'd12,
    I_RX_SAMPLE   = 5'd13,

    I_NACK_LOW    = 5'd14,
    I_NACK_HIGH   = 5'd15,

    I_STOP_LOW    = 5'd16,
    I_STOP_HIGH   = 5'd17,
    I_STOP_REL    = 5'd18,

    I_DONE        = 5'd19;


reg [4:0] i_state;


reg [7:0] tx_byte;
reg [7:0] rx_byte;

reg [2:0] bit_index;

reg [2:0] stage;

reg read_latched;

reg [7:0] reg_latched;

reg [7:0] data_latched;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        scl_low <= 1'b0;
        sda_low <= 1'b0;

        busy <= 1'b0;
        done <= 1'b0;

        ack_error <= 1'b0;

        read_data <= 8'h00;

        i_state <= I_IDLE;

        tx_byte <= 8'h00;
        rx_byte <= 8'h00;

        bit_index <= 3'd7;

        stage <= 3'd0;

        read_latched <= 1'b0;

        reg_latched <= 8'h00;

        data_latched <= 8'h00;

    end

    else
    begin

        done <= 1'b0;


        ////////////////////////////////////////////////////////////////////////
        // Start
        ////////////////////////////////////////////////////////////////////////

        if((i_state == I_IDLE) && start)
        begin

            busy <= 1'b1;

            ack_error <= 1'b0;


            read_latched <= read_op;

            reg_latched <= reg_addr;

            data_latched <= write_data;


            // Device write address
            tx_byte <= 8'h52;

            bit_index <= 3'd7;

            stage <= 3'd0;


            scl_low <= 1'b0;
            sda_low <= 1'b0;

            i_state <= I_START1;

        end


        else if(busy && tick)
        begin

            case(i_state)

            I_START1:
            begin

                scl_low <= 1'b0;
                sda_low <= 1'b0;

                i_state <= I_START2;

            end


            I_START2:
            begin

                // SDA 1 -> 0 while SCL high
                scl_low <= 1'b0;
                sda_low <= 1'b1;

                i_state <= I_TX_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // TX
            ////////////////////////////////////////////////////////////////////

            I_TX_LOW:
            begin

                scl_low <= 1'b1;

                if(tx_byte[bit_index] == 1'b0)
                    sda_low <= 1'b1;
                else
                    sda_low <= 1'b0;

                i_state <= I_TX_HIGH;

            end


            I_TX_HIGH:
            begin

                scl_low <= 1'b0;

                if(bit_index == 3'd0)
                begin

                    i_state <= I_ACK_LOW;

                end

                else
                begin

                    bit_index <= bit_index - 1'b1;

                    i_state <= I_TX_LOW;

                end

            end


            ////////////////////////////////////////////////////////////////////
            // ACK
            ////////////////////////////////////////////////////////////////////

            I_ACK_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b0;

                i_state <= I_ACK_HIGH;

            end


            I_ACK_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                i_state <= I_ACK_SAMPLE;

            end


            I_ACK_SAMPLE:
            begin

                if(sda_in != 1'b0)
                    ack_error <= 1'b1;


                case(stage)

                ////////////////////////////////////////////////////////////////
                // Device Write ACK
                ////////////////////////////////////////////////////////////////

                3'd0:
                begin

                    tx_byte <= reg_latched;

                    bit_index <= 3'd7;

                    stage <= 3'd1;

                    i_state <= I_TX_LOW;

                end


                ////////////////////////////////////////////////////////////////
                // Register ACK
                ////////////////////////////////////////////////////////////////

                3'd1:
                begin

                    if(read_latched)
                    begin

                        i_state <= I_REP_LOW;

                    end

                    else
                    begin

                        tx_byte <= data_latched;

                        bit_index <= 3'd7;

                        stage <= 3'd2;

                        i_state <= I_TX_LOW;

                    end

                end


                ////////////////////////////////////////////////////////////////
                // Write Data ACK
                ////////////////////////////////////////////////////////////////

                3'd2:
                begin

                    i_state <= I_STOP_LOW;

                end


                ////////////////////////////////////////////////////////////////
                // Device Read ACK
                ////////////////////////////////////////////////////////////////

                3'd3:
                begin

                    rx_byte <= 8'h00;

                    bit_index <= 3'd7;

                    i_state <= I_RX_LOW;

                end


                default:
                begin

                    i_state <= I_STOP_LOW;

                end

                endcase

            end


            ////////////////////////////////////////////////////////////////////
            // Repeated START
            ////////////////////////////////////////////////////////////////////

            I_REP_LOW:
            begin

                scl_low <= 1'b1;
                sda_low <= 1'b0;

                i_state <= I_REP_HIGH;

            end


            I_REP_HIGH:
            begin

                scl_low <= 1'b0;
                sda_low <= 1'b0;

                i_state <= I_REP_START;

            end


            I_REP_START:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b1;


                tx_byte <= 8'h53;

                bit_index <= 3'd7;

                stage <= 3'd3;

                i_state <= I_TX_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // RX
            ////////////////////////////////////////////////////////////////////

            I_RX_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b0;

                i_state <= I_RX_HIGH;

            end


            I_RX_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                i_state <= I_RX_SAMPLE;

            end


            I_RX_SAMPLE:
            begin

                rx_byte[bit_index] <= sda_in;


                if(bit_index == 3'd0)
                begin

                    i_state <= I_NACK_LOW;

                end

                else
                begin

                    bit_index <= bit_index - 1'b1;

                    i_state <= I_RX_LOW;

                end

            end


            ////////////////////////////////////////////////////////////////////
            // NACK
            ////////////////////////////////////////////////////////////////////

            I_NACK_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b0;

                i_state <= I_NACK_HIGH;

            end


            I_NACK_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                i_state <= I_STOP_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // STOP
            ////////////////////////////////////////////////////////////////////

            I_STOP_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b1;

                i_state <= I_STOP_HIGH;

            end


            I_STOP_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b1;

                i_state <= I_STOP_REL;

            end


            I_STOP_REL:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                i_state <= I_DONE;

            end


            ////////////////////////////////////////////////////////////////////
            // DONE
            ////////////////////////////////////////////////////////////////////

            I_DONE:
            begin

                read_data <= rx_byte;

                busy <= 1'b0;

                done <= 1'b1;

                i_state <= I_IDLE;

            end


            default:
            begin

                busy <= 1'b0;

                scl_low <= 1'b0;
                sda_low <= 1'b0;

                i_state <= I_IDLE;

            end

            endcase

        end

    end

end


endmodule



////////////////////////////////////////////////////////////////////////////////
//
// UART TX
//
// CLK = 50MHz
// Baud = 115200
//
////////////////////////////////////////////////////////////////////////////////

module tof_dist_uart_tx
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire [7:0] data,

    output reg tx,

    output reg busy
);


localparam CLKS_PER_BIT = 16'd434;


reg [15:0] baud_cnt;

reg [3:0] bit_index;

reg [7:0] tx_data;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        tx <= 1'b1;

        busy <= 1'b0;

        baud_cnt <= 16'd0;

        bit_index <= 4'd0;

        tx_data <= 8'h00;

    end

    else
    begin

        if(!busy)
        begin

            tx <= 1'b1;

            baud_cnt <= 16'd0;

            bit_index <= 4'd0;


            if(start)
            begin

                tx_data <= data;

                busy <= 1'b1;

                tx <= 1'b0;

            end

        end


        else
        begin

            if(baud_cnt >= CLKS_PER_BIT - 1'b1)
            begin

                baud_cnt <= 16'd0;

                bit_index <= bit_index + 1'b1;


                case(bit_index)

                4'd0:
                    tx <= tx_data[0];

                4'd1:
                    tx <= tx_data[1];

                4'd2:
                    tx <= tx_data[2];

                4'd3:
                    tx <= tx_data[3];

                4'd4:
                    tx <= tx_data[4];

                4'd5:
                    tx <= tx_data[5];

                4'd6:
                    tx <= tx_data[6];

                4'd7:
                    tx <= tx_data[7];

                4'd8:
                    tx <= 1'b1;

                4'd9:
                begin

                    tx <= 1'b1;

                    busy <= 1'b0;

                end

                default:
                    tx <= 1'b1;

                endcase

            end

            else
            begin

                baud_cnt <= baud_cnt + 1'b1;

            end

        end

    end

end

endmodule


////////////////////////////////////////////////////////////////////////////////
// VL53L0X 5点距离标定模块
//
// 标定范围：10cm ~ 50cm
//
// 使用方法：
//
// 1. 把白纸分别放在：
//    10cm
//    20cm
//    30cm
//    40cm
//    50cm
//
// 2. 记录VL53L0X原始测量值
//
// 3. 修改下面：
//
//    MEAS_10CM
//    MEAS_20CM
//    MEAS_30CM
//    MEAS_40CM
//    MEAS_50CM
//
// 模块会自动进行分段线性插值。
////////////////////////////////////////////////////////////////////////////////

module tof_distance_calibration
(
    input  wire [15:0] raw_distance_mm,

    output reg  [15:0] corrected_distance_mm
);


////////////////////////////////////////////////////////////////////////////////
// 实际距离
//
// 这些一般不用改
////////////////////////////////////////////////////////////////////////////////

localparam [15:0] REAL_10CM = 16'd100;
localparam [15:0] REAL_20CM = 16'd200;
localparam [15:0] REAL_30CM = 16'd300;
localparam [15:0] REAL_40CM = 16'd400;
localparam [15:0] REAL_50CM = 16'd500;


////////////////////////////////////////////////////////////////////////////////
// ★★★★★ 标定数据修改区 ★★★★★
//
// 把你实际测出来的数据填写到这里
////////////////////////////////////////////////////////////////////////////////


//------------------------------------------------------------
// 实际距离 = 10cm
//
// 你目前测得：122mm
//------------------------------------------------------------

localparam [15:0] MEAS_10CM = 16'd122;


//------------------------------------------------------------
// 实际距离 = 20cm
//
// 你目前测得：230mm
//------------------------------------------------------------

localparam [15:0] MEAS_20CM = 16'd230;


//------------------------------------------------------------
// 实际距离 = 30cm
//
// 测量完成后修改这里
//
// 例如：如果显示335mm
//
// localparam [15:0] MEAS_30CM = 16'd335;
//------------------------------------------------------------

localparam [15:0] MEAS_30CM = 16'd330;  // TODO: 测完30cm后替换为RAW值


//------------------------------------------------------------
// 实际距离 = 40cm
//
// 测量完成后修改这里
//------------------------------------------------------------

localparam [15:0] MEAS_40CM = 16'd430;  // TODO: 测完40cm后替换为RAW值


//------------------------------------------------------------
// 实际距离 = 50cm
//
// 测量完成后修改这里
//------------------------------------------------------------

localparam [15:0] MEAS_50CM = 16'd530;  // TODO: 测完50cm后替换为RAW值


////////////////////////////////////////////////////////////////////////////////
// 中间计算变量
////////////////////////////////////////////////////////////////////////////////

reg [31:0] numerator;
reg [15:0] denominator;

reg [15:0] x0;
reg [15:0] x1;

reg [15:0] y0;
reg [15:0] y1;


////////////////////////////////////////////////////////////////////////////////
// 分段线性校准
//
// 公式：
//
// corrected = y0
//           + (raw - x0)
//           * (y1 - y0)
//           / (x1 - x0)
//
// x = 传感器测量值
// y = 实际距离
////////////////////////////////////////////////////////////////////////////////

always @*
begin

    ////////////////////////////////////////////////////////////////////////////
    // 默认值
    ////////////////////////////////////////////////////////////////////////////

    corrected_distance_mm = raw_distance_mm;

    numerator   = 32'd0;
    denominator = 16'd1;

    x0 = 16'd0;
    x1 = 16'd0;

    y0 = 16'd0;
    y1 = 16'd0;


    ////////////////////////////////////////////////////////////////////////////
    // 低于10cm标定点
    //
    // 为了避障稳定，直接限制为100mm
    ////////////////////////////////////////////////////////////////////////////

    if(raw_distance_mm <= MEAS_10CM)
    begin

        corrected_distance_mm = REAL_10CM;

    end


    ////////////////////////////////////////////////////////////////////////////
    // 10cm ~ 20cm
    ////////////////////////////////////////////////////////////////////////////

    else if(raw_distance_mm <= MEAS_20CM)
    begin

        x0 = MEAS_10CM;
        x1 = MEAS_20CM;

        y0 = REAL_10CM;
        y1 = REAL_20CM;

        numerator =
            (raw_distance_mm - x0)
            *
            (y1 - y0);

        denominator =
            x1 - x0;

        corrected_distance_mm =
            y0
            +
            numerator / denominator;

    end


    ////////////////////////////////////////////////////////////////////////////
    // 20cm ~ 30cm
    ////////////////////////////////////////////////////////////////////////////

    else if(raw_distance_mm <= MEAS_30CM)
    begin

        x0 = MEAS_20CM;
        x1 = MEAS_30CM;

        y0 = REAL_20CM;
        y1 = REAL_30CM;

        numerator =
            (raw_distance_mm - x0)
            *
            (y1 - y0);

        denominator =
            x1 - x0;

        corrected_distance_mm =
            y0
            +
            numerator / denominator;

    end


    ////////////////////////////////////////////////////////////////////////////
    // 30cm ~ 40cm
    ////////////////////////////////////////////////////////////////////////////

    else if(raw_distance_mm <= MEAS_40CM)
    begin

        x0 = MEAS_30CM;
        x1 = MEAS_40CM;

        y0 = REAL_30CM;
        y1 = REAL_40CM;

        numerator =
            (raw_distance_mm - x0)
            *
            (y1 - y0);

        denominator =
            x1 - x0;

        corrected_distance_mm =
            y0
            +
            numerator / denominator;

    end


    ////////////////////////////////////////////////////////////////////////////
    // 40cm ~ 50cm
    ////////////////////////////////////////////////////////////////////////////

    else if(raw_distance_mm <= MEAS_50CM)
    begin

        x0 = MEAS_40CM;
        x1 = MEAS_50CM;

        y0 = REAL_40CM;
        y1 = REAL_50CM;

        numerator =
            (raw_distance_mm - x0)
            *
            (y1 - y0);

        denominator =
            x1 - x0;

        corrected_distance_mm =
            y0
            +
            numerator / denominator;

    end


    ////////////////////////////////////////////////////////////////////////////
    // 超过50cm标定范围
    //
    // 当前先限制到500mm
    //
    // 后面正式导航时可以改成继续外推
    ////////////////////////////////////////////////////////////////////////////

    else
    begin

        corrected_distance_mm = REAL_50CM;

    end

end


endmodule
