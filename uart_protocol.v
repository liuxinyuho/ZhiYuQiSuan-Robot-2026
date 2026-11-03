//////////////////////////////////////////////////////////////////////////////////
// File Name : uart_protocol.v
//
// Description:
//   硬石上位机 <-> FPGA 通信协议层
//
// Physical UART:
//   115200 bps
//   8N1
//
// Frame:
//   AA 55 CMD LEN DATA... CHECKSUM
//
// CHECKSUM:
//   CMD + LEN + DATA0 + DATA1 + ...，低8位
//
// Host -> FPGA:
//
//   CMD 0x01 : 设置速度
//              LEN = 6
//              DATA:
//                VX_L VX_H
//                VY_L VY_H
//                WZ_L WZ_H
//
//   CMD 0x02 : AI模式
//              LEN = 1
//              DATA0:
//                0 = OFF
//                1 = ON
//
//   CMD 0x03 : STOP
//              LEN = 0
//
//   CMD 0x04 : 启动Waypoint导航
//              LEN = 0
//
// FPGA -> Host:
//
//   CMD 0x81 : 状态数据
//              LEN = 18
//
//   Payload:
//       0~3   X mm
//       4~7   Y mm
//       8~11  Yaw cdeg
//       12~13 Ultrasonic mm
//       14    Obstacle State
//       15    Waypoint Index
//       16    Navigation State
//       17    Flags
//
//   每100ms自动上传一次
//////////////////////////////////////////////////////////////////////////////////

module uart_protocol(

    input wire clk,
    input wire reset_n,

    //================================================
    // 10ms tick
    //================================================
    input wire sample_tick,

    //================================================
    // uart_byte_rx
    //================================================
    input wire [7:0] rx_data,
    input wire       rx_done,

    //================================================
    // uart_byte_tx
    //================================================
    output reg [7:0] tx_data,
    output reg       tx_send,

    input wire tx_done,
    input wire tx_busy,

    //================================================
    // Host command outputs
    //================================================
    output reg signed [12:0] host_vx,
    output reg signed [12:0] host_vy,
    output reg signed [12:0] host_wz,

    output reg host_cmd_valid,

    output reg ai_mode,

    output reg stop_req,

    output reg waypoint_start_req,

    output reg frame_ok,

    //================================================
    // FPGA status input
    //================================================
    input wire signed [31:0] status_x_mm,
    input wire signed [31:0] status_y_mm,

    input wire signed [31:0] status_yaw_cdeg,

    input wire [15:0] status_distance_mm,

    input wire [3:0] status_obstacle_state,

    input wire [2:0] status_waypoint_index,

    input wire [2:0] status_nav_state,

    // Physical control-source selector: 0=local FPGA, 1=BM1684 host.
    input wire status_host_mode

);


    //================================================
    // Command
    //================================================

    localparam [7:0] CMD_SET_VELOCITY =
        8'h01;

    localparam [7:0] CMD_SET_AI_MODE =
        8'h02;

    localparam [7:0] CMD_STOP =
        8'h03;

    localparam [7:0] CMD_START_WAYPOINT =
        8'h04;

    localparam [7:0] CMD_TELEMETRY =
        8'h81;


    //================================================
    // RX State
    //================================================

    localparam [2:0]

        RX_WAIT_AA = 3'd0,

        RX_WAIT_55 = 3'd1,

        RX_CMD     = 3'd2,

        RX_LEN     = 3'd3,

        RX_DATA    = 3'd4,

        RX_CHECK   = 3'd5;


    reg [2:0] rx_state;

    reg [7:0] rx_cmd;

    reg [7:0] rx_len;

    reg [7:0] rx_checksum;

    reg [4:0] rx_index;


    // 最大16字节Payload
    reg [7:0] rx_buffer [0:15];


    //================================================
    // RX Protocol
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            rx_state <=
                RX_WAIT_AA;

            rx_cmd <=
                8'd0;

            rx_len <=
                8'd0;

            rx_checksum <=
                8'd0;

            rx_index <=
                5'd0;


            host_vx <=
                13'sd0;

            host_vy <=
                13'sd0;

            host_wz <=
                13'sd0;


            host_cmd_valid <=
                1'b0;

            ai_mode <=
                1'b0;

            stop_req <=
                1'b0;

            waypoint_start_req <=
                1'b0;

            frame_ok <=
                1'b0;

        end


        else begin

            //================================================
            // Pulse signals
            //================================================

            host_cmd_valid <=
                1'b0;

            stop_req <=
                1'b0;

            waypoint_start_req <=
                1'b0;

            frame_ok <=
                1'b0;


            if(rx_done) begin

                case(rx_state)


                    //========================================
                    // Header AA
                    //========================================

                    RX_WAIT_AA: begin

                        if(rx_data == 8'hAA)

                            rx_state <=
                                RX_WAIT_55;

                    end


                    //========================================
                    // Header 55
                    //========================================

                    RX_WAIT_55: begin

                        if(rx_data == 8'h55)

                            rx_state <=
                                RX_CMD;

                        else if(rx_data == 8'hAA)

                            rx_state <=
                                RX_WAIT_55;

                        else

                            rx_state <=
                                RX_WAIT_AA;

                    end


                    //========================================
                    // CMD
                    //========================================

                    RX_CMD: begin

                        rx_cmd <=
                            rx_data;

                        rx_checksum <=
                            rx_data;

                        rx_state <=
                            RX_LEN;

                    end


                    //========================================
                    // LEN
                    //========================================

                    RX_LEN: begin

                        rx_len <=
                            rx_data;

                        rx_checksum <=
                            rx_checksum
                            +
                            rx_data;

                        rx_index <=
                            5'd0;


                        // LEN = 0
                        if(rx_data == 8'd0)

                            rx_state <=
                                RX_CHECK;


                        // 最大16字节
                        else if(rx_data <= 8'd16)

                            rx_state <=
                                RX_DATA;


                        else

                            rx_state <=
                                RX_WAIT_AA;

                    end


                    //========================================
                    // DATA
                    //========================================

                    RX_DATA: begin

                        rx_buffer[rx_index] <=
                            rx_data;

                        rx_checksum <=
                            rx_checksum
                            +
                            rx_data;


                        if(
                            rx_index
                            ==
                            rx_len - 1'b1
                        ) begin

                            rx_state <=
                                RX_CHECK;

                            rx_index <=
                                5'd0;

                        end


                        else begin

                            rx_index <=
                                rx_index + 1'b1;

                        end

                    end


                    //========================================
                    // CHECKSUM
                    //========================================

                    RX_CHECK: begin

                        rx_state <=
                            RX_WAIT_AA;


                        if(
                            rx_data
                            ==
                            rx_checksum
                        ) begin

                            frame_ok <=
                                1'b1;


                            //================================
                            // SET VELOCITY
                            //================================

                            if(
                                rx_cmd
                                ==
                                CMD_SET_VELOCITY

                                &&

                                rx_len
                                ==
                                8'd6
                            ) begin

                                //================================
                                // 16bit数据低13bit作为FPGA速度
                                //
                                // 要求上位机发送范围：
                                // -4096 ~ +4095
                                //================================

                                host_vx <=
                                    $signed({
                                        rx_buffer[1][4:0],
                                        rx_buffer[0]
                                    });


                                host_vy <=
                                    $signed({
                                        rx_buffer[3][4:0],
                                        rx_buffer[2]
                                    });


                                host_wz <=
                                    $signed({
                                        rx_buffer[5][4:0],
                                        rx_buffer[4]
                                    });


                                host_cmd_valid <=
                                    1'b1;

                            end


                            //================================
                            // AI MODE
                            //================================

                            else if(
                                rx_cmd
                                ==
                                CMD_SET_AI_MODE

                                &&

                                rx_len
                                ==
                                8'd1
                            ) begin

                                ai_mode <=
                                    rx_buffer[0][0];

                            end


                            //================================
                            // STOP
                            //================================

                            else if(
                                rx_cmd
                                ==
                                CMD_STOP

                                &&

                                rx_len
                                ==
                                8'd0
                            ) begin

                                host_vx <=
                                    13'sd0;

                                host_vy <=
                                    13'sd0;

                                host_wz <=
                                    13'sd0;

                                ai_mode <=
                                    1'b0;

                                stop_req <=
                                    1'b1;

                            end


                            //================================
                            // START WAYPOINT
                            //================================

                            else if(
                                rx_cmd
                                ==
                                CMD_START_WAYPOINT

                                &&

                                rx_len
                                ==
                                8'd0
                            ) begin

                                waypoint_start_req <=
                                    1'b1;

                            end

                        end

                    end


                    default: begin

                        rx_state <=
                            RX_WAIT_AA;

                    end


                endcase

            end

        end

    end


    //================================================
    // TX Status Snapshot
    //================================================

    reg signed [31:0] tx_x_mm;

    reg signed [31:0] tx_y_mm;

    reg signed [31:0] tx_yaw_cdeg;

    reg [15:0] tx_distance_mm;

    reg [3:0] tx_obstacle_state;

    reg [2:0] tx_waypoint_index;

    reg [2:0] tx_nav_state;

    reg [7:0] tx_flags;


    //================================================
    // Telemetry Timer
    //
    // sample_tick = 10ms
    //
    // 10 * 10ms = 100ms
    // 10Hz
    //================================================

    reg [3:0] telemetry_tick;


    //================================================
    // TX FSM
    //================================================

    reg tx_active;

    reg tx_wait;

    reg [4:0] tx_index;

    reg [7:0] tx_checksum;


    //================================================
    // Current TX byte
    //
    // Frame:
    //
    // 0  AA
    // 1  55
    // 2  81
    // 3  12(hex) = 18
    //
    // 4~21 payload
    //
    // 22 checksum
    //================================================

    reg [7:0] tx_byte_mux;


    always @(*) begin

        tx_byte_mux =
            8'h00;


        case(tx_index)

            5'd0:
                tx_byte_mux = 8'hAA;

            5'd1:
                tx_byte_mux = 8'h55;

            5'd2:
                tx_byte_mux = CMD_TELEMETRY;

            5'd3:
                tx_byte_mux = 8'd18;


            // X mm
            5'd4:
                tx_byte_mux = tx_x_mm[7:0];

            5'd5:
                tx_byte_mux = tx_x_mm[15:8];

            5'd6:
                tx_byte_mux = tx_x_mm[23:16];

            5'd7:
                tx_byte_mux = tx_x_mm[31:24];


            // Y mm
            5'd8:
                tx_byte_mux = tx_y_mm[7:0];

            5'd9:
                tx_byte_mux = tx_y_mm[15:8];

            5'd10:
                tx_byte_mux = tx_y_mm[23:16];

            5'd11:
                tx_byte_mux = tx_y_mm[31:24];


            // Yaw cdeg
            5'd12:
                tx_byte_mux = tx_yaw_cdeg[7:0];

            5'd13:
                tx_byte_mux = tx_yaw_cdeg[15:8];

            5'd14:
                tx_byte_mux = tx_yaw_cdeg[23:16];

            5'd15:
                tx_byte_mux = tx_yaw_cdeg[31:24];


            // Ultrasonic
            5'd16:
                tx_byte_mux = tx_distance_mm[7:0];

            5'd17:
                tx_byte_mux = tx_distance_mm[15:8];


            // States
            5'd18:
                tx_byte_mux =
                    {
                        4'd0,
                        tx_obstacle_state
                    };


            5'd19:
                tx_byte_mux =
                    {
                        5'd0,
                        tx_waypoint_index
                    };


            5'd20:
                tx_byte_mux =
                    {
                        5'd0,
                        tx_nav_state
                    };


            5'd21:
                tx_byte_mux =
                    tx_flags;


            // Checksum
            5'd22:
                tx_byte_mux =
                    tx_checksum;


            default:
                tx_byte_mux =
                    8'h00;

        endcase

    end


    //================================================
    // TX Controller
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            tx_data <=
                8'd0;

            tx_send <=
                1'b0;


            telemetry_tick <=
                4'd0;


            tx_active <=
                1'b0;

            tx_wait <=
                1'b0;

            tx_index <=
                5'd0;

            tx_checksum <=
                8'd0;


            tx_x_mm <=
                32'sd0;

            tx_y_mm <=
                32'sd0;

            tx_yaw_cdeg <=
                32'sd0;

            tx_distance_mm <=
                16'd0;

            tx_obstacle_state <=
                4'd0;

            tx_waypoint_index <=
                3'd0;

            tx_nav_state <=
                3'd0;

            tx_flags <=
                8'd0;

        end


        else begin

            // send_en只能保持1clk
            tx_send <=
                1'b0;


            //================================================
            // Telemetry timing
            //================================================

            if(sample_tick) begin

                if(
                    telemetry_tick
                    >=
                    4'd9
                ) begin

                    telemetry_tick <=
                        4'd0;


                    //========================================
                    // 当前没有发送数据
                    // 开始新的状态帧
                    //========================================

                    if(!tx_active) begin

                        tx_x_mm <=
                            status_x_mm;

                        tx_y_mm <=
                            status_y_mm;

                        tx_yaw_cdeg <=
                            status_yaw_cdeg;

                        tx_distance_mm <=
                            status_distance_mm;

                        tx_obstacle_state <=
                            status_obstacle_state;

                        tx_waypoint_index <=
                            status_waypoint_index;

                        tx_nav_state <=
                            status_nav_state;


                        // bit0: software AI enable; bit1: physical host mode.
                        tx_flags <=
                            {
                                6'd0,
                                status_host_mode,
                                ai_mode
                            };


                        //====================================
                        // checksum =
                        // CMD + LEN + PAYLOAD
                        //====================================

                        tx_checksum <=

                            CMD_TELEMETRY

                            +

                            8'd18

                            +

                            status_x_mm[7:0]

                            +

                            status_x_mm[15:8]

                            +

                            status_x_mm[23:16]

                            +

                            status_x_mm[31:24]

                            +

                            status_y_mm[7:0]

                            +

                            status_y_mm[15:8]

                            +

                            status_y_mm[23:16]

                            +

                            status_y_mm[31:24]

                            +

                            status_yaw_cdeg[7:0]

                            +

                            status_yaw_cdeg[15:8]

                            +

                            status_yaw_cdeg[23:16]

                            +

                            status_yaw_cdeg[31:24]

                            +

                            status_distance_mm[7:0]

                            +

                            status_distance_mm[15:8]

                            +

                            {
                                4'd0,
                                status_obstacle_state
                            }

                            +

                            {
                                5'd0,
                                status_waypoint_index
                            }

                            +

                            {
                                5'd0,
                                status_nav_state
                            }

                            +

                            {
                                7'd0,
                                ai_mode
                            };


                        tx_index <=
                            5'd0;

                        tx_active <=
                            1'b1;

                        tx_wait <=
                            1'b0;

                    end

                end


                else begin

                    telemetry_tick <=
                        telemetry_tick + 1'b1;

                end

            end


            //================================================
            // Send one byte
            //================================================

            if(tx_active) begin


                //============================================
                // 当前字节尚未提交UART
                //============================================

                if(
                    !tx_wait

                    &&

                    !tx_busy
                ) begin

                    tx_data <=
                        tx_byte_mux;

                    tx_send <=
                        1'b1;

                    tx_wait <=
                        1'b1;

                end


                //============================================
                // 一个byte发送完成
                //============================================

                if(tx_done) begin

                    tx_wait <=
                        1'b0;


                    // 最后一个byte已经发送完
                    if(
                        tx_index
                        ==
                        5'd22
                    ) begin

                        tx_active <=
                            1'b0;

                        tx_index <=
                            5'd0;

                    end


                    else begin

                        tx_index <=
                            tx_index + 1'b1;

                    end

                end

            end

        end

    end


endmodule