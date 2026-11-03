module vl53l0x_sensor_core
(
    input  wire clk,
    input  wire reset_n,

    inout  wire imu_sda,
    inout  wire imu_scl,

    output wire uart_tx,

    output wire [15:0] distance_raw_mm_out,
    output wire [15:0] distance_cal_mm_out,

    output wire        distance_valid_out,
    output wire        sensor_error,
    output wire [6:0]  debug_state_out
);


//============================================================
// I2C
//============================================================

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


//============================================================
// UART
//============================================================

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


//============================================================
// Boot
//============================================================

reg [23:0] boot_cnt;


//============================================================
// VL53L0X Init Data
//============================================================

reg [7:0] stop_variable;
reg [7:0] temp_reg;
reg [7:0] spad_info;

wire [6:0] spad_count;
wire       spad_aperture;

assign spad_count =
       spad_info[6:0];

assign spad_aperture =
       spad_info[7];


//============================================================
// SPAD MAP
//
// 重要时序优化：
//
// 原来：
// 48个bit在一个组合for循环中一次计算
//
// 现在：
// 每个50MHz时钟只计算1bit
//
// 48bit只需要:
// 48 × 20ns = 960ns
//
// 但是每一个组合路径只经过很少逻辑。
//============================================================

reg [47:0] spad_map;

reg [47:0] spad_map_masked;

reg [5:0] spad_mask_index;

reg [6:0] spad_enabled_count;


//============================================================
// VL53L0X Tuning Table
//============================================================

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


//============================================================
// Controller States
//============================================================

localparam

    S_BOOT            = 7'd0,

    S_ID              = 7'd1,

    S_PRE             = 7'd2,
    S_STOPVAR         = 7'd3,
    S_POST            = 7'd4,

    S_MSRC_READ       = 7'd5,
    S_MSRC_WRITE      = 7'd6,

    S_BASECFG         = 7'd7,

    S_SPAD_PRE1       = 7'd8,
    S_SPAD_R83        = 7'd9,
    S_SPAD_W83        = 7'd10,
    S_SPAD_PRE2       = 7'd11,

    S_SPAD_POLL       = 7'd12,
    S_SPAD_83_ONE     = 7'd13,
    S_SPAD_R92        = 7'd14,

    S_SPAD_CLEAN1     = 7'd15,
    S_SPAD_R83_2      = 7'd16,
    S_SPAD_W83_2      = 7'd17,
    S_SPAD_CLEAN2     = 7'd18,

    S_MAP_READ        = 7'd19,
    S_MAP_CFG         = 7'd20,
    S_MAP_WRITE       = 7'd21,

    S_TUNING          = 7'd22,

    S_INT_CFG1        = 7'd23,
    S_INT_R84         = 7'd24,
    S_INT_W84         = 7'd25,
    S_INT_CFG2        = 7'd26,

    S_CAL1_CFG        = 7'd27,
    S_CAL1_START      = 7'd28,
    S_CAL1_POLL       = 7'd29,
    S_CAL1_END        = 7'd30,

    S_CAL2_CFG        = 7'd31,
    S_CAL2_START      = 7'd32,
    S_CAL2_POLL       = 7'd33,
    S_CAL2_END        = 7'd34,

    S_RESTORE         = 7'd35,

    S_CONTINUOUS      = 7'd36,

    S_RUN_STATUS      = 7'd37,
    S_RUN_HIGH        = 7'd38,
    S_RUN_LOW         = 7'd39,
    S_RUN_CLEAR       = 7'd40,

    // 新增：
    // SPAD MAP逐位处理
    S_MAP_MASK        = 7'd41,

    S_ERROR           = 7'd63;


reg [6:0] state;


//============================================================
// Debug state
//============================================================

reg [6:0] last_state;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        last_state <= 7'd0;

    end

    else
    begin

        if(state != S_ERROR)
            last_state <= state;

    end

end


assign debug_state_out =

       (state == S_ERROR)

       ?

       last_state

       :

       state;


//============================================================
// FSM counters
//============================================================

reg [6:0] seq_index;

reg [6:0] tune_index;

reg [2:0] map_index;

reg op_active;

reg [15:0] poll_timeout;


//============================================================
// Distance
//============================================================

reg [7:0] distance_high;

reg [15:0] distance_mm;

reg distance_valid;


//============================================================
// Calibrated distance
//============================================================

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


//============================================================
// External outputs
//============================================================

assign distance_raw_mm_out =
       distance_mm;


assign distance_cal_mm_out =
       distance_corrected_mm;


assign distance_valid_out =
       distance_valid;


// Same-edge error flag: mirrors every FSM transition, without a
// state-decode LUT chain on the sensor-health/safety output path.
// This is NOT a delayed sample of (state == S_ERROR).
reg sensor_error_q /* synthesis syn_preserve = 1 */;
assign sensor_error = sensor_error_q;


//============================================================
// Current tuning
//============================================================

wire [15:0] current_tuning;

assign current_tuning =
       tune_data(tune_index);


//============================================================
// Small Init Tables
//============================================================

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

        2'd0: base_cfg_data = 16'h4400;
        2'd1: base_cfg_data = 16'h4520;

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

        3'd0:
            masked_map_byte =
                spad_map_masked[7:0];

        3'd1:
            masked_map_byte =
                spad_map_masked[15:8];

        3'd2:
            masked_map_byte =
                spad_map_masked[23:16];

        3'd3:
            masked_map_byte =
                spad_map_masked[31:24];

        3'd4:
            masked_map_byte =
                spad_map_masked[39:32];

        3'd5:
            masked_map_byte =
                spad_map_masked[47:40];

        default:
            masked_map_byte =
                8'h00;

        endcase

    end

endfunction


function [15:0] int_cfg1_data;

    input [0:0] index;

    begin

        case(index)

        1'd0:
            int_cfg1_data = 16'h0A04;

        default:
            int_cfg1_data = 16'h0000;

        endcase

    end

endfunction


function [15:0] int_cfg2_data;

    input [1:0] index;

    begin

        case(index)

        2'd0:
            int_cfg2_data = 16'h0B01;

        2'd1:
            int_cfg2_data = 16'h01E8;

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

        4'd0:
            continuous_data = 16'h8001;

        4'd1:
            continuous_data = 16'hFF01;

        4'd2:
            continuous_data = 16'h0000;

        4'd3:
            continuous_data =
            {
                8'h91,
                stop_var
            };

        4'd4:
            continuous_data = 16'h0001;

        4'd5:
            continuous_data = 16'hFF00;

        4'd6:
            continuous_data = 16'h8000;

        4'd7:
            continuous_data = 16'h0002;

        default:
            continuous_data = 16'h0000;

        endcase

    end

endfunction


//============================================================
// Table Access
//============================================================

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
       continuous_data
       (
           seq_index[3:0],
           stop_variable
       );


//============================================================
// Main FSM
//============================================================

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        state <=
            S_BOOT;

        sensor_error_q <= 1'b0;

        boot_cnt <=
            24'd0;


        i2c_start <=
            1'b0;

        i2c_read <=
            1'b0;

        i2c_reg <=
            8'h00;

        i2c_wdata <=
            8'h00;


        op_active <=
            1'b0;


        seq_index <=
            7'd0;

        tune_index <=
            7'd0;

        map_index <=
            3'd0;


        stop_variable <=
            8'h00;

        temp_reg <=
            8'h00;

        spad_info <=
            8'h00;


        spad_map <=
            48'd0;

        spad_map_masked <=
            48'd0;

        spad_mask_index <=
            6'd0;

        spad_enabled_count <=
            7'd0;


        poll_timeout <=
            16'd0;


        distance_high <=
            8'h00;

        distance_mm <=
            16'd0;

        distance_valid <=
            1'b0;

    end


    else
    begin

        i2c_start <=
            1'b0;

        distance_valid <=
            1'b0;


        case(state)


        //====================================================
        // BOOT
        //====================================================

        S_BOOT:
        begin

            if
            (
                boot_cnt
                >=
                24'd4_999_999
            )
            begin

                boot_cnt <=
                    24'd0;

                op_active <=
                    1'b0;

                state <=
                    S_ID;

                sensor_error_q <= 1'b0;

            end

            else
            begin

                boot_cnt <=
                    boot_cnt + 1'b1;

            end

        end


        //====================================================
        // Read ID
        //====================================================

        S_ID:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'hC0;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if
                (
                    i2c_ack_error
                    ||
                    i2c_rdata != 8'hEE
                )
                begin

                    state <=
                        S_ERROR;

                    sensor_error_q <= 1'b1;

                end

                else
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_PRE;

                    sensor_error_q <= 1'b0;

                end

            end

        end


        //====================================================
        // PRE
        //====================================================

        S_PRE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_pre[15:8];

                i2c_wdata <=
                    current_pre[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(i2c_ack_error)
                begin

                    state <=
                        S_ERROR;

                    sensor_error_q <= 1'b1;

                end


                else if(seq_index == 7'd4)
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_STOPVAR;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // Stop Variable
        //====================================================

        S_STOPVAR:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h91;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(i2c_ack_error)
                begin

                    state <=
                        S_ERROR;

                    sensor_error_q <= 1'b1;

                end

                else
                begin

                    stop_variable <=
                        i2c_rdata;

                    seq_index <=
                        7'd0;

                    state <=
                        S_POST;

                    sensor_error_q <= 1'b0;

                end

            end

        end


        //====================================================
        // POST
        //====================================================

        S_POST:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_post[15:8];

                i2c_wdata <=
                    current_post[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(i2c_ack_error)
                begin

                    state <=
                        S_ERROR;

                    sensor_error_q <= 1'b1;

                end


                else if(seq_index == 7'd2)
                begin

                    state <=
                        S_MSRC_READ;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // MSRC READ
        //====================================================

        S_MSRC_READ:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h60;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                temp_reg <=
                    i2c_rdata;

                state <=
                    S_MSRC_WRITE;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // MSRC WRITE
        //====================================================

        S_MSRC_WRITE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h60;

                i2c_wdata <=
                    temp_reg | 8'h12;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                seq_index <=
                    7'd0;

                state <=
                    S_BASECFG;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // BASE CFG
        //====================================================

        S_BASECFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_base_cfg[15:8];

                i2c_wdata <=
                    current_base_cfg[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                if(seq_index == 7'd2)
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_SPAD_PRE1;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // SPAD PRE1
        //====================================================

        S_SPAD_PRE1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_spad_pre1[15:8];

                i2c_wdata <=
                    current_spad_pre1[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                if(seq_index == 7'd3)
                begin

                    state <=
                        S_SPAD_R83;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // SPAD R83
        //====================================================

        S_SPAD_R83:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h83;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                temp_reg <=
                    i2c_rdata;

                state <=
                    S_SPAD_W83;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // SPAD W83
        //====================================================

        S_SPAD_W83:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h83;

                i2c_wdata <=
                    temp_reg | 8'h04;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                seq_index <=
                    7'd0;

                state <=
                    S_SPAD_PRE2;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // SPAD PRE2
        //====================================================

        S_SPAD_PRE2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_spad_pre2[15:8];

                i2c_wdata <=
                    current_spad_pre2[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                if(seq_index == 7'd4)
                begin

                    poll_timeout <=
                        16'd0;

                    state <=
                        S_SPAD_POLL;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // SPAD POLL
        //====================================================

        S_SPAD_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h83;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(i2c_rdata != 8'h00)
                begin

                    state <=
                        S_SPAD_83_ONE;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    poll_timeout <=
                        poll_timeout + 1'b1;

                    if(poll_timeout > 16'd1000)
                    begin

                        state <=
                            S_ERROR;

                        sensor_error_q <= 1'b1;

                    end

                end

            end

        end


        //====================================================
        // SPAD 83 = 1
        //====================================================

        S_SPAD_83_ONE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h83;

                i2c_wdata <=
                    8'h01;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                state <=
                    S_SPAD_R92;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Read SPAD info
        //====================================================

        S_SPAD_R92:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h92;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                spad_info <=
                    i2c_rdata;

                seq_index <=
                    7'd0;

                state <=
                    S_SPAD_CLEAN1;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // SPAD CLEAN1
        //====================================================

        S_SPAD_CLEAN1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_spad_clean1[15:8];

                i2c_wdata <=
                    current_spad_clean1[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                if(seq_index == 7'd1)
                begin

                    state <=
                        S_SPAD_R83_2;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // SPAD R83 second
        //====================================================

        S_SPAD_R83_2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h83;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                temp_reg <=
                    i2c_rdata;

                state <=
                    S_SPAD_W83_2;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // SPAD W83 second
        //====================================================

        S_SPAD_W83_2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h83;

                i2c_wdata <=
                    temp_reg & 8'hFB;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                seq_index <=
                    7'd0;

                state <=
                    S_SPAD_CLEAN2;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // SPAD CLEAN2
        //====================================================

        S_SPAD_CLEAN2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_spad_clean2[15:8];

                i2c_wdata <=
                    current_spad_clean2[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd3)
                begin

                    map_index <=
                        3'd0;

                    state <=
                        S_MAP_READ;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // Read SPAD MAP B0~B5
        //====================================================

        S_MAP_READ:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'hB0 + map_index;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                case(map_index)

                    3'd0:
                    begin

                        spad_map[7:0] <=
                            i2c_rdata;

                    end


                    3'd1:
                    begin

                        spad_map[15:8] <=
                            i2c_rdata;

                    end


                    3'd2:
                    begin

                        spad_map[23:16] <=
                            i2c_rdata;

                    end


                    3'd3:
                    begin

                        spad_map[31:24] <=
                            i2c_rdata;

                    end


                    3'd4:
                    begin

                        spad_map[39:32] <=
                            i2c_rdata;

                    end


                    3'd5:
                    begin

                        spad_map[47:40] <=
                            i2c_rdata;

                    end


                    default:
                    begin

                        spad_map <=
                            spad_map;

                    end

                endcase


                if(map_index == 3'd5)
                begin

                    seq_index <=
                        7'd0;

                    map_index <=
                        3'd0;


                    spad_map_masked <=
                        48'd0;

                    spad_mask_index <=
                        6'd0;

                    spad_enabled_count <=
                        7'd0;


                    state <=
                        S_MAP_MASK;


                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    map_index <=
                        map_index + 1'b1;

                end

            end

        end


        //====================================================
        // SPAD MASK
        //
        // 每个时钟只处理1bit
        //====================================================

        S_MAP_MASK:
        begin

            // aperture:
            // 前12个SPAD关闭

            if
            (
                spad_aperture
                &&
                (
                    spad_mask_index
                    <
                    6'd12
                )
            )
            begin

                spad_map_masked[spad_mask_index]
                    <=
                    1'b0;

            end


            // 已达到需要的SPAD数量

            else if
            (
                spad_enabled_count
                >=
                spad_count
            )
            begin

                spad_map_masked[spad_mask_index]
                    <=
                    1'b0;

            end


            // 原本该SPAD有效

            else if
            (
                spad_map[spad_mask_index]
            )
            begin

                spad_map_masked[spad_mask_index]
                    <=
                    1'b1;


                spad_enabled_count
                    <=
                    spad_enabled_count + 1'b1;

            end


            // 原本为0

            else
            begin

                spad_map_masked[spad_mask_index]
                    <=
                    1'b0;

            end


            // 是否完成48bit

            if
            (
                spad_mask_index
                ==
                6'd47
            )
            begin

                spad_mask_index <=
                    6'd0;

                map_index <=
                    3'd0;

                seq_index <=
                    7'd0;


                state <=
                    S_MAP_CFG;


                sensor_error_q <= 1'b0;

            end

            else
            begin

                spad_mask_index <=
                    spad_mask_index + 1'b1;

            end

        end


        //====================================================
        // SPAD MAP CFG
        //====================================================

        S_MAP_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_map_cfg[15:8];

                i2c_wdata <=
                    current_map_cfg[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd4)
                begin

                    map_index <=
                        3'd0;

                    state <=
                        S_MAP_WRITE;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // Write B0~B5
        //====================================================

        S_MAP_WRITE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'hB0 + map_index;

                i2c_wdata <=
                    masked_map_byte(map_index);

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(map_index == 3'd5)
                begin

                    tune_index <=
                        7'd0;

                    state <=
                        S_TUNING;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    map_index <=
                        map_index + 1'b1;

                end

            end

        end


        //====================================================
        // Tuning
        //====================================================

        S_TUNING:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_tuning[15:8];

                i2c_wdata <=
                    current_tuning[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(tune_index == 7'd79)
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_INT_CFG1;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    tune_index <=
                        tune_index + 1'b1;

                end

            end

        end


        //====================================================
        // INT CFG1
        //====================================================

        S_INT_CFG1:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_int_cfg1[15:8];

                i2c_wdata <=
                    current_int_cfg1[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                state <=
                    S_INT_R84;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // INT R84
        //====================================================

        S_INT_R84:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h84;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                temp_reg <=
                    i2c_rdata;

                state <=
                    S_INT_W84;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // INT W84
        //====================================================

        S_INT_W84:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h84;

                i2c_wdata <=
                    temp_reg & 8'hEF;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                seq_index <=
                    7'd0;

                state <=
                    S_INT_CFG2;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // INT CFG2
        //====================================================

        S_INT_CFG2:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_int_cfg2[15:8];

                i2c_wdata <=
                    current_int_cfg2[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd1)
                begin

                    state <=
                        S_CAL1_CFG;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // Calibration 1 CFG
        //====================================================

        S_CAL1_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h01;

                i2c_wdata <=
                    8'h01;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                state <=
                    S_CAL1_START;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Calibration 1 Start
        //====================================================

        S_CAL1_START:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h00;

                i2c_wdata <=
                    8'h41;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                poll_timeout <=
                    16'd0;

                state <=
                    S_CAL1_POLL;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Calibration 1 Poll
        //====================================================

        S_CAL1_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h13;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if
                (
                    i2c_rdata[2:0]
                    !=
                    3'b000
                )
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_CAL1_END;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    poll_timeout <=
                        poll_timeout + 1'b1;


                    if
                    (
                        poll_timeout
                        >
                        16'd2000
                    )
                    begin

                        state <=
                            S_ERROR;

                        sensor_error_q <= 1'b1;

                    end

                end

            end

        end


        //====================================================
        // Calibration 1 End
        //====================================================

        S_CAL1_END:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;


                if(seq_index == 7'd0)
                begin

                    i2c_reg <=
                        8'h0B;

                    i2c_wdata <=
                        8'h01;

                end

                else
                begin

                    i2c_reg <=
                        8'h00;

                    i2c_wdata <=
                        8'h00;

                end


                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd1)
                begin

                    state <=
                        S_CAL2_CFG;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        7'd1;

                end

            end

        end


        //====================================================
        // Calibration 2 CFG
        //====================================================

        S_CAL2_CFG:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h01;

                i2c_wdata <=
                    8'h02;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                state <=
                    S_CAL2_START;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Calibration 2 Start
        //====================================================

        S_CAL2_START:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h00;

                i2c_wdata <=
                    8'h01;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                poll_timeout <=
                    16'd0;

                state <=
                    S_CAL2_POLL;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Calibration 2 Poll
        //====================================================

        S_CAL2_POLL:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h13;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if
                (
                    i2c_rdata[2:0]
                    !=
                    3'b000
                )
                begin

                    seq_index <=
                        7'd0;

                    state <=
                        S_CAL2_END;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    poll_timeout <=
                        poll_timeout + 1'b1;


                    if
                    (
                        poll_timeout
                        >
                        16'd2000
                    )
                    begin

                        state <=
                            S_ERROR;

                        sensor_error_q <= 1'b1;

                    end

                end

            end

        end


        //====================================================
        // Calibration 2 End
        //====================================================

        S_CAL2_END:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;


                if(seq_index == 7'd0)
                begin

                    i2c_reg <=
                        8'h0B;

                    i2c_wdata <=
                        8'h01;

                end

                else
                begin

                    i2c_reg <=
                        8'h00;

                    i2c_wdata <=
                        8'h00;

                end


                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd1)
                begin

                    state <=
                        S_RESTORE;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        7'd1;

                end

            end

        end


        //====================================================
        // Restore sequence
        //====================================================

        S_RESTORE:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h01;

                i2c_wdata <=
                    8'hE8;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                seq_index <=
                    7'd0;

                state <=
                    S_CONTINUOUS;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Continuous mode
        //====================================================

        S_CONTINUOUS:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    current_cont[15:8];

                i2c_wdata <=
                    current_cont[7:0];

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(seq_index == 7'd7)
                begin

                    state <=
                        S_RUN_STATUS;

                    sensor_error_q <= 1'b0;

                end

                else
                begin

                    seq_index <=
                        seq_index + 1'b1;

                end

            end

        end


        //====================================================
        // Run Status
        //====================================================

        S_RUN_STATUS:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h13;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                if(i2c_ack_error)
                begin

                    state <=
                        S_ERROR;

                    sensor_error_q <= 1'b1;

                end


                else if
                (
                    i2c_rdata[2:0]
                    !=
                    3'b000
                )
                begin

                    state <=
                        S_RUN_HIGH;

                    sensor_error_q <= 1'b0;

                end

            end

        end


        //====================================================
        // Distance high
        //====================================================

        S_RUN_HIGH:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h1E;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                distance_high <=
                    i2c_rdata;

                state <=
                    S_RUN_LOW;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Distance low
        //====================================================

        S_RUN_LOW:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b1;

                i2c_reg <=
                    8'h1F;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;


                distance_mm <=
                {
                    distance_high,
                    i2c_rdata
                };


                distance_valid <=
                    1'b1;


                state <=
                    S_RUN_CLEAR;


                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // Clear Interrupt
        //====================================================

        S_RUN_CLEAR:
        begin

            if(!op_active && !i2c_busy)
            begin

                i2c_read <=
                    1'b0;

                i2c_reg <=
                    8'h0B;

                i2c_wdata <=
                    8'h01;

                i2c_start <=
                    1'b1;

                op_active <=
                    1'b1;

            end


            else if(op_active && i2c_done)
            begin

                op_active <=
                    1'b0;

                state <=
                    S_RUN_STATUS;

                sensor_error_q <= 1'b0;

            end

        end


        //====================================================
        // ERROR
        //====================================================

        S_ERROR:
        begin

            state <=
                S_ERROR;

            sensor_error_q <= 1'b1;

        end


        default:
        begin

            state <=
                S_BOOT;

            sensor_error_q <= 1'b0;

        end


        endcase

    end

end


//============================================================
// UART Distance Report
//
// RAW=xxxx CAL=xxxx mm
//============================================================

reg report_active;

reg [4:0] report_index;

reg [1:0] report_phase;

reg [15:0] report_raw_distance;

reg [15:0] report_cal_distance;


wire [3:0] raw_digit_thousand;
wire [3:0] raw_digit_hundred;
wire [3:0] raw_digit_ten;
wire [3:0] raw_digit_one;

wire [3:0] cal_digit_thousand;
wire [3:0] cal_digit_hundred;
wire [3:0] cal_digit_ten;
wire [3:0] cal_digit_one;


assign raw_digit_thousand =
       (report_raw_distance / 1000) % 10;

assign raw_digit_hundred =
       (report_raw_distance / 100) % 10;

assign raw_digit_ten =
       (report_raw_distance / 10) % 10;

assign raw_digit_one =
       report_raw_distance % 10;


assign cal_digit_thousand =
       (report_cal_distance / 1000) % 10;

assign cal_digit_hundred =
       (report_cal_distance / 100) % 10;

assign cal_digit_ten =
       (report_cal_distance / 10) % 10;

assign cal_digit_one =
       report_cal_distance % 10;


function [7:0] digit_ascii;

    input [3:0] digit;

    begin

        digit_ascii =
            8'h30 + digit;

    end

endfunction


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        uart_start <=
            1'b0;

        uart_data <=
            8'h00;

        report_active <=
            1'b0;

        report_index <=
            5'd0;

        report_phase <=
            2'd0;

        report_raw_distance <=
            16'd0;

        report_cal_distance <=
            16'd0;

    end


    else
    begin

        uart_start <=
            1'b0;


        if
        (
            distance_valid
            &&
            !report_active
        )
        begin

            report_raw_distance <=
                distance_mm;

            report_cal_distance <=
                distance_corrected_mm;

            report_active <=
                1'b1;

            report_index <=
                5'd0;

            report_phase <=
                2'd0;

        end


        if(report_active)
        begin

            case(report_phase)


            2'd0:
            begin

                if(!uart_busy)
                begin

                    case(report_index)

                    5'd0:
                        uart_data <= "R";

                    5'd1:
                        uart_data <= "A";

                    5'd2:
                        uart_data <= "W";

                    5'd3:
                        uart_data <= "=";

                    5'd4:
                        uart_data <=
                            digit_ascii
                            (
                                raw_digit_thousand
                            );

                    5'd5:
                        uart_data <=
                            digit_ascii
                            (
                                raw_digit_hundred
                            );

                    5'd6:
                        uart_data <=
                            digit_ascii
                            (
                                raw_digit_ten
                            );

                    5'd7:
                        uart_data <=
                            digit_ascii
                            (
                                raw_digit_one
                            );

                    5'd8:
                        uart_data <= " ";

                    5'd9:
                        uart_data <= "C";

                    5'd10:
                        uart_data <= "A";

                    5'd11:
                        uart_data <= "L";

                    5'd12:
                        uart_data <= "=";

                    5'd13:
                        uart_data <=
                            digit_ascii
                            (
                                cal_digit_thousand
                            );

                    5'd14:
                        uart_data <=
                            digit_ascii
                            (
                                cal_digit_hundred
                            );

                    5'd15:
                        uart_data <=
                            digit_ascii
                            (
                                cal_digit_ten
                            );

                    5'd16:
                        uart_data <=
                            digit_ascii
                            (
                                cal_digit_one
                            );

                    5'd17:
                        uart_data <= " ";

                    5'd18:
                        uart_data <= "m";

                    5'd19:
                        uart_data <= "m";

                    5'd20:
                        uart_data <=
                            8'h0D;

                    5'd21:
                        uart_data <=
                            8'h0A;

                    default:
                        uart_data <=
                            8'h20;

                    endcase


                    uart_start <=
                        1'b1;

                    report_phase <=
                        2'd1;

                end

            end


            2'd1:
            begin

                if(uart_busy)
                begin

                    report_phase <=
                        2'd2;

                end

            end


            2'd2:
            begin

                if(!uart_busy)
                begin

                    if(report_index == 5'd21)
                    begin

                        report_active <=
                            1'b0;

                        report_index <=
                            5'd0;

                        report_phase <=
                            2'd0;

                    end

                    else
                    begin

                        report_index <=
                            report_index + 1'b1;

                        report_phase <=
                            2'd0;

                    end

                end

            end


            default:
            begin

                report_phase <=
                    2'd0;

            end


            endcase

        end

    end

end


endmodule



//==========================================================================
// VL53L0X I2C MASTER
//
// FPGA Clock = 50MHz
// I2C approximately 100kHz
//==========================================================================

//==============================================================
// Stable I2C master adapted from the ID reader verified on ACG720
// 50MHz; SCL approx 50kHz; device 0x29 (write 0x52/read 0x53)
// Register read: START, 52 ACK, REG ACK, RESTART, 53 ACK, RX, NACK, STOP
// Register write: START, 52 ACK, REG ACK, DATA ACK, STOP
// ACK failures are accumulated in ack_error; transaction always terminates.
//==============================================================
module tof_dist_i2c_master (
    input wire clk, input wire reset_n,
    input wire start, input wire read_op,
    input wire [7:0] reg_addr, input wire [7:0] write_data,
    output reg [7:0] read_data,
    output reg busy, output reg done, output reg ack_error,
    inout wire i2c_scl, inout wire i2c_sda
);
    reg scl_low, sda_low;
    assign i2c_scl = scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_low ? 1'b0 : 1'bz;
    wire sda_in = i2c_sda;

    // 5us step = 250 clocks @ 50MHz; 4 ticks/bit => ~50 kHz.
    reg [8:0] div_cnt;
    wire tick = (div_cnt == 9'd249);
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) div_cnt <= 9'd0;
        else if (!busy || tick) div_cnt <= 9'd0;
        else div_cnt <= div_cnt + 1'b1;
    end

    localparam [4:0]
        I_IDLE=5'd0, I_START_HIGH=5'd1, I_START_SDA=5'd2,
        I_TX_LOW=5'd3, I_TX_SETUP=5'd4,
        I_TX_HIGH=5'd5, I_TX_END=5'd6,
        I_ACK_LOW=5'd7, I_ACK_RELEASE=5'd8,
        I_ACK_HIGH=5'd9, I_ACK_SAMPLE=5'd10, I_ACK_DOWN=5'd11,
        I_REP_LOW=5'd12, I_REP_RELEASE=5'd13,
        I_REP_HIGH=5'd14, I_REP_START=5'd15,
        I_RX_LOW=5'd16, I_RX_HIGH=5'd17,
        I_RX_SAMPLE=5'd18, I_RX_DOWN=5'd19,
        I_NACK_LOW=5'd20, I_NACK_HIGH=5'd21,
        I_STOP_LOW=5'd22, I_STOP_SETUP=5'd23,
        I_STOP_HIGH=5'd24, I_STOP_RELEASE=5'd25,
        I_DONE=5'd26,
        I_BUS_CLEAR_LOW=5'd27, I_BUS_CLEAR_HIGH=5'd28,
        I_BUS_STOP_LOW=5'd29, I_BUS_STOP_HIGH=5'd30,
        I_BUS_STOP_RELEASE=5'd31;

    reg [4:0] state;
    reg [3:0] bus_clear_count;
    reg [7:0] tx_byte, rx_byte;
    reg [7:0] reg_latched, data_latched;
    reg read_latched;
    reg [2:0] bit_index;
    // phase: 0 device write address, 1 register, 2 device read, 3 write data
    reg [1:0] phase;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // After every driver reset, release SDA and issue nine SCL clocks.
            // This clears a VL53L0X I2C state machine that was interrupted by
            // motor noise or an incomplete transaction. Then generate STOP.
            state <= I_BUS_CLEAR_LOW;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            busy <= 1'b1;
            done <= 1'b0;
            ack_error <= 1'b0;
            read_data <= 8'h00;
            tx_byte <= 8'h00;
            rx_byte <= 8'h00;
            reg_latched <= 8'h00;
            data_latched <= 8'h00;
            read_latched <= 1'b0;
            bit_index <= 3'd7;
            phase <= 2'd0;
            bus_clear_count <= 4'd0;
        end else begin
            done <= 1'b0;
            if (state == I_IDLE) begin
                // All lines released while idle.
                scl_low <= 1'b0; sda_low <= 1'b0;
                if (start) begin
                    busy <= 1'b1;
                    ack_error <= 1'b0;
                    read_data <= 8'h00;
                    reg_latched <= reg_addr;
                    data_latched <= write_data;
                    read_latched <= read_op;
                    tx_byte <= 8'h52;
                    bit_index <= 3'd7;
                    phase <= 2'd0;
                    state <= I_START_HIGH;
                end
            end else if (busy && tick) begin
                case (state)
                    // Nine-clock I2C bus clear. SDA is always released while
                    // SCL is pulsed. The pulses are local to this ToF channel.
                    I_BUS_CLEAR_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        state <= I_BUS_CLEAR_HIGH;
                    end
                    I_BUS_CLEAR_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        if (bus_clear_count == 4'd8) begin
                            state <= I_BUS_STOP_LOW;
                        end else begin
                            bus_clear_count <= bus_clear_count + 1'b1;
                            state <= I_BUS_CLEAR_LOW;
                        end
                    end
                    I_BUS_STOP_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        state <= I_BUS_STOP_HIGH;
                    end
                    I_BUS_STOP_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_BUS_STOP_RELEASE;
                    end
                    I_BUS_STOP_RELEASE: begin
                        sda_low <= 1'b0;
                        busy <= 1'b0;
                        state <= I_IDLE;
                    end
                    I_START_HIGH: begin
                        scl_low <= 1'b0; sda_low <= 1'b0;
                        state <= I_START_SDA;
                    end
                    I_START_SDA: begin
                        // SDA falls while SCL high: START
                        sda_low <= 1'b1;
                        state <= I_TX_LOW;
                    end
                    I_TX_LOW: begin
                        // SCL falls first
                        scl_low <= 1'b1;
                        state <= I_TX_SETUP;
                    end
                    I_TX_SETUP: begin
                        // SDA changes ONLY after SCL is low
                        sda_low <= ~tx_byte[bit_index];
                        state <= I_TX_HIGH;
                    end
                    I_TX_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_TX_END;
                    end
                    I_TX_END: begin
                        if (bit_index == 3'd0) state <= I_ACK_LOW;
                        else begin
                            bit_index <= bit_index - 1'b1;
                            state <= I_TX_LOW;
                        end
                    end
                    I_ACK_LOW: begin
                        scl_low <= 1'b1;
                        state <= I_ACK_RELEASE;
                    end
                    I_ACK_RELEASE: begin
                        sda_low <= 1'b0;
                        state <= I_ACK_HIGH;
                    end
                    I_ACK_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_ACK_SAMPLE;
                    end
                    I_ACK_SAMPLE: begin
                        if (sda_in != 1'b0) ack_error <= 1'b1;
                        state <= I_ACK_DOWN;
                    end
                    I_ACK_DOWN: begin
                        scl_low <= 1'b1;
                        case (phase)
                            2'd0: begin
                                // Device address acknowledged; send register
                                tx_byte <= reg_latched;
                                bit_index <= 3'd7;
                                phase <= 2'd1;
                                state <= I_TX_LOW;
                            end
                            2'd1: begin
                                if (read_latched)
                                    state <= I_REP_LOW;
                                else begin
                                    // For register write, send payload byte
                                    tx_byte <= data_latched;
                                    bit_index <= 3'd7;
                                    phase <= 2'd3;
                                    state <= I_TX_LOW;
                                end
                            end
                            2'd2: begin
                                rx_byte <= 8'h00;
                                bit_index <= 3'd7;
                                state <= I_RX_LOW;
                            end
                            default: state <= I_STOP_LOW;
                        endcase
                    end
                    I_REP_LOW: begin
                        scl_low <= 1'b1;
                        state <= I_REP_RELEASE;
                    end
                    I_REP_RELEASE: begin
                        sda_low <= 1'b0;
                        state <= I_REP_HIGH;
                    end
                    I_REP_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_REP_START;
                    end
                    I_REP_START: begin
                        // Repeated START while SCL high
                        sda_low <= 1'b1;
                        tx_byte <= 8'h53;
                        phase <= 2'd2;
                        bit_index <= 3'd7;
                        state <= I_TX_LOW;
                    end
                    I_RX_LOW: begin
                        scl_low <= 1'b1; sda_low <= 1'b0;
                        state <= I_RX_HIGH;
                    end
                    I_RX_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_RX_SAMPLE;
                    end
                    I_RX_SAMPLE: begin
                        rx_byte[bit_index] <= sda_in;
                        state <= I_RX_DOWN;
                    end
                    I_RX_DOWN: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0) state <= I_NACK_LOW;
                        else begin
                            bit_index <= bit_index - 1'b1;
                            state <= I_RX_LOW;
                        end
                    end
                    I_NACK_LOW: begin
                        scl_low <= 1'b1; sda_low <= 1'b0;
                        state <= I_NACK_HIGH;
                    end
                    I_NACK_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_STOP_LOW;
                    end
                    I_STOP_LOW: begin
                        scl_low <= 1'b1;
                        state <= I_STOP_SETUP;
                    end
                    I_STOP_SETUP: begin
                        sda_low <= 1'b1;
                        state <= I_STOP_HIGH;
                    end
                    I_STOP_HIGH: begin
                        scl_low <= 1'b0;
                        state <= I_STOP_RELEASE;
                    end
                    I_STOP_RELEASE: begin
                        // SDA rising while SCL high: STOP
                        sda_low <= 1'b0;
                        state <= I_DONE;
                    end
                    I_DONE: begin
                        // Results registered one cycle before done observed
                        if (read_latched) read_data <= rx_byte;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= I_IDLE;
                    end
                    default: begin
                        scl_low <= 1'b0; sda_low <= 1'b0;
                        busy <= 1'b0;
                        state <= I_IDLE;
                    end
                endcase
            end
        end
    end
endmodule



//==========================================================================
// UART TX
//==========================================================================

module tof_dist_uart_tx
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire [7:0] data,

    output reg tx,

    output reg busy
);


localparam CLKS_PER_BIT =
    16'd434;


reg [15:0] baud_cnt;

reg [3:0] bit_index;

reg [7:0] tx_data;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        tx <=
            1'b1;

        busy <=
            1'b0;

        baud_cnt <=
            16'd0;

        bit_index <=
            4'd0;

        tx_data <=
            8'h00;

    end


    else
    begin

        if(!busy)
        begin

            tx <=
                1'b1;

            baud_cnt <=
                16'd0;

            bit_index <=
                4'd0;


            if(start)
            begin

                tx_data <=
                    data;

                busy <=
                    1'b1;

                tx <=
                    1'b0;

            end

        end


        else
        begin

            if
            (
                baud_cnt
                >=
                CLKS_PER_BIT - 1'b1
            )
            begin

                baud_cnt <=
                    16'd0;

                bit_index <=
                    bit_index + 1'b1;


                case(bit_index)

                4'd0:
                    tx <=
                        tx_data[0];

                4'd1:
                    tx <=
                        tx_data[1];

                4'd2:
                    tx <=
                        tx_data[2];

                4'd3:
                    tx <=
                        tx_data[3];

                4'd4:
                    tx <=
                        tx_data[4];

                4'd5:
                    tx <=
                        tx_data[5];

                4'd6:
                    tx <=
                        tx_data[6];

                4'd7:
                    tx <=
                        tx_data[7];

                4'd8:
                    tx <=
                        1'b1;

                4'd9:
                begin

                    tx <=
                        1'b1;

                    busy <=
                        1'b0;

                end

                default:
                    tx <=
                        1'b1;

                endcase

            end

            else
            begin

                baud_cnt <=
                    baud_cnt + 1'b1;

            end

        end

    end

end


endmodule



//==========================================================================
// VL53L0X Distance Calibration
//==========================================================================

//============================================================
// VL53L0X: RAW test mode. No combinational division in FPGA.
// For competition calibration, use separately clocked, pipelined
// fixed-point calibration. During this integration CAL=RAW.
//============================================================
module tof_distance_calibration (
    input  wire [15:0] raw_distance_mm,
    output wire [15:0] corrected_distance_mm
);
    assign corrected_distance_mm = raw_distance_mm;
endmodule