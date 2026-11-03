////////////////////////////////////////////////////////////////////////////////
// GW5AT60K + VL53L0X / VL53L1X 型号自动识别
//
// CLK      : 50MHz
// UART     : 115200 8N1
//
// SDA      : C20
// SCL      : D20
//
// XSHUT    : 外部直接接3.3V
//
// UART输出示例：
//
// ID0=EE ID1=00 M=0
//      -> VL53L0X
//
// ID0=XX ID1=EA M=1
//      -> VL53L1X
//
// M=?      -> 未识别
////////////////////////////////////////////////////////////////////////////////

module tof_id_test_top
(
    input  wire clk,
    input  wire reset_n,

    inout  wire imu_sda,
    inout  wire imu_scl,

    output wire uart_tx
);


////////////////////////////////////////////////////////////////////////////////
// 型号定义
////////////////////////////////////////////////////////////////////////////////

localparam MODEL_UNKNOWN = 2'd0;
localparam MODEL_L0X     = 2'd1;
localparam MODEL_L1X     = 2'd2;


////////////////////////////////////////////////////////////////////////////////
// I2C读取控制
////////////////////////////////////////////////////////////////////////////////

reg        i2c_start;
reg        i2c_addr16;
reg [15:0] i2c_reg_addr;

wire       i2c_busy;
wire       i2c_done;
wire       i2c_ack_error;
wire [7:0] i2c_rd_data;


tof_i2c_reg_read
u_i2c
(
    .clk
    (
        clk
    ),

    .reset_n
    (
        reset_n
    ),

    .start
    (
        i2c_start
    ),

    .addr16
    (
        i2c_addr16
    ),

    .reg_addr
    (
        i2c_reg_addr
    ),

    .i2c_scl
    (
        imu_scl
    ),

    .i2c_sda
    (
        imu_sda
    ),

    .busy
    (
        i2c_busy
    ),

    .done
    (
        i2c_done
    ),

    .ack_error
    (
        i2c_ack_error
    ),

    .rd_data
    (
        i2c_rd_data
    )
);


////////////////////////////////////////////////////////////////////////////////
// 型号检测状态机
////////////////////////////////////////////////////////////////////////////////

localparam
    T_BOOT     = 4'd0,
    T_L0_START = 4'd1,
    T_L0_WAIT  = 4'd2,
    T_GAP      = 4'd3,
    T_L1_START = 4'd4,
    T_L1_WAIT  = 4'd5,
    T_DONE     = 4'd6;


reg [3:0] test_state;


// 上电等待100ms
reg [23:0] boot_cnt;


// 两次检测之间等待1ms
reg [15:0] gap_cnt;


reg [7:0] id_l0;
reg [7:0] id_l1;

reg [1:0] model;

reg model_ready;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        i2c_start    <= 1'b0;
        i2c_addr16   <= 1'b0;
        i2c_reg_addr <= 16'h0000;

        test_state <= T_BOOT;

        boot_cnt <= 24'd0;
        gap_cnt  <= 16'd0;

        id_l0 <= 8'h00;
        id_l1 <= 8'h00;

        model <= MODEL_UNKNOWN;

        model_ready <= 1'b0;

    end

    else
    begin

        // 默认Start只维持一个CLK
        i2c_start <= 1'b0;


        case(test_state)


        ////////////////////////////////////////////////////////////////////////
        // 上电等待
        ////////////////////////////////////////////////////////////////////////

        T_BOOT:
        begin

            if(boot_cnt >= 24'd4999999)
            begin

                boot_cnt <= 24'd0;

                test_state <= T_L0_START;

            end

            else
            begin

                boot_cnt <= boot_cnt + 1'b1;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 第一阶段：
        //
        // 按 VL53L0X 方式读取
        //
        // Register 0xC0
        ////////////////////////////////////////////////////////////////////////

        T_L0_START:
        begin

            if(!i2c_busy)
            begin

                // VL53L0X使用8bit Register Address
                i2c_addr16 <= 1'b0;

                i2c_reg_addr <= 16'h00C0;

                i2c_start <= 1'b1;

                test_state <= T_L0_WAIT;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 等VL53L0X读取完成
        ////////////////////////////////////////////////////////////////////////

        T_L0_WAIT:
        begin

            if(i2c_done)
            begin

                id_l0 <= i2c_rd_data;


                ////////////////////////////////////////////////////////////////
                // VL53L0X：
                //
                // 0xC0 = 0xEE
                ////////////////////////////////////////////////////////////////

                if
                (
                    (!i2c_ack_error)
                    &&
                    (i2c_rd_data == 8'hEE)
                )
                begin

                    model <= MODEL_L0X;

                    model_ready <= 1'b1;

                    test_state <= T_DONE;

                end

                else
                begin

                    // 不是L0X
                    // 继续测试L1X

                    gap_cnt <= 16'd0;

                    test_state <= T_GAP;

                end

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 等待1ms
        ////////////////////////////////////////////////////////////////////////

        T_GAP:
        begin

            if(gap_cnt >= 16'd49999)
            begin

                gap_cnt <= 16'd0;

                test_state <= T_L1_START;

            end

            else
            begin

                gap_cnt <= gap_cnt + 1'b1;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 第二阶段：
        //
        // 按 VL53L1X 方式读取
        //
        // Register 0x010F
        ////////////////////////////////////////////////////////////////////////

        T_L1_START:
        begin

            if(!i2c_busy)
            begin

                // VL53L1X使用16bit Register Address
                i2c_addr16 <= 1'b1;

                i2c_reg_addr <= 16'h010F;

                i2c_start <= 1'b1;

                test_state <= T_L1_WAIT;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // VL53L1X读取结果
        ////////////////////////////////////////////////////////////////////////

        T_L1_WAIT:
        begin

            if(i2c_done)
            begin

                id_l1 <= i2c_rd_data;


                ////////////////////////////////////////////////////////////////
                // VL53L1X：
                //
                // 0x010F = 0xEA
                ////////////////////////////////////////////////////////////////

                if
                (
                    (!i2c_ack_error)
                    &&
                    (i2c_rd_data == 8'hEA)
                )
                begin

                    model <= MODEL_L1X;

                end

                else
                begin

                    model <= MODEL_UNKNOWN;

                end


                model_ready <= 1'b1;

                test_state <= T_DONE;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // 完成
        ////////////////////////////////////////////////////////////////////////

        T_DONE:
        begin

            test_state <= T_DONE;

        end


        default:
        begin

            test_state <= T_BOOT;

        end


        endcase

    end

end


////////////////////////////////////////////////////////////////////////////////
// UART
////////////////////////////////////////////////////////////////////////////////

wire uart_busy;

reg uart_start;
reg [7:0] uart_data;


uart_tx_115200
u_uart
(
    .clk
    (
        clk
    ),

    .reset_n
    (
        reset_n
    ),

    .start
    (
        uart_start
    ),

    .data
    (
        uart_data
    ),

    .tx
    (
        uart_tx
    ),

    .busy
    (
        uart_busy
    )
);


////////////////////////////////////////////////////////////////////////////////
// HEX -> ASCII
////////////////////////////////////////////////////////////////////////////////

function [7:0] hex_ascii;

    input [3:0] nib;

    begin

        if(nib < 10)
            hex_ascii = 8'h30 + nib;

        else
            hex_ascii = 8'h41 + (nib - 10);

    end

endfunction


////////////////////////////////////////////////////////////////////////////////
// 每秒发送一次:
//
// ID0=EE ID1=00 M=0
//
// M=0 -> VL53L0X
// M=1 -> VL53L1X
// M=? -> Unknown
////////////////////////////////////////////////////////////////////////////////

reg [25:0] report_timer;

reg report_active;

reg [5:0] report_index;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        uart_start <= 1'b0;

        uart_data <= 8'h00;

        report_timer <= 26'd0;

        report_active <= 1'b0;

        report_index <= 6'd0;

    end

    else
    begin

        uart_start <= 1'b0;


        ////////////////////////////////////////////////////////////////////////
        // 每1秒启动一次发送
        ////////////////////////////////////////////////////////////////////////

        if(model_ready && !report_active)
        begin

            if(report_timer >= 26'd49999999)
            begin

                report_timer <= 26'd0;

                report_active <= 1'b1;

                report_index <= 6'd0;

            end

            else
            begin

                report_timer <= report_timer + 1'b1;

            end

        end

        else if(!model_ready)
        begin

            report_timer <= 26'd0;

        end


        ////////////////////////////////////////////////////////////////////////
        // UART发送
        ////////////////////////////////////////////////////////////////////////

        if(report_active && !uart_busy && !uart_start)
        begin

            case(report_index)


            // I
            6'd0:
            begin
                uart_data <= 8'h49;
                uart_start <= 1'b1;
                report_index <= 6'd1;
            end


            // D
            6'd1:
            begin
                uart_data <= 8'h44;
                uart_start <= 1'b1;
                report_index <= 6'd2;
            end


            // 0
            6'd2:
            begin
                uart_data <= 8'h30;
                uart_start <= 1'b1;
                report_index <= 6'd3;
            end


            // =
            6'd3:
            begin
                uart_data <= 8'h3D;
                uart_start <= 1'b1;
                report_index <= 6'd4;
            end


            // ID0 high
            6'd4:
            begin
                uart_data <= hex_ascii(id_l0[7:4]);
                uart_start <= 1'b1;
                report_index <= 6'd5;
            end


            // ID0 low
            6'd5:
            begin
                uart_data <= hex_ascii(id_l0[3:0]);
                uart_start <= 1'b1;
                report_index <= 6'd6;
            end


            // space
            6'd6:
            begin
                uart_data <= 8'h20;
                uart_start <= 1'b1;
                report_index <= 6'd7;
            end


            // I
            6'd7:
            begin
                uart_data <= 8'h49;
                uart_start <= 1'b1;
                report_index <= 6'd8;
            end


            // D
            6'd8:
            begin
                uart_data <= 8'h44;
                uart_start <= 1'b1;
                report_index <= 6'd9;
            end


            // 1
            6'd9:
            begin
                uart_data <= 8'h31;
                uart_start <= 1'b1;
                report_index <= 6'd10;
            end


            // =
            6'd10:
            begin
                uart_data <= 8'h3D;
                uart_start <= 1'b1;
                report_index <= 6'd11;
            end


            // ID1 high
            6'd11:
            begin
                uart_data <= hex_ascii(id_l1[7:4]);
                uart_start <= 1'b1;
                report_index <= 6'd12;
            end


            // ID1 low
            6'd12:
            begin
                uart_data <= hex_ascii(id_l1[3:0]);
                uart_start <= 1'b1;
                report_index <= 6'd13;
            end


            // space
            6'd13:
            begin
                uart_data <= 8'h20;
                uart_start <= 1'b1;
                report_index <= 6'd14;
            end


            // M
            6'd14:
            begin
                uart_data <= 8'h4D;
                uart_start <= 1'b1;
                report_index <= 6'd15;
            end


            // =
            6'd15:
            begin
                uart_data <= 8'h3D;
                uart_start <= 1'b1;
                report_index <= 6'd16;
            end


            ////////////////////////////////////////////////////////////////////
            // 型号
            ////////////////////////////////////////////////////////////////////

            6'd16:
            begin

                if(model == MODEL_L0X)
                    uart_data <= 8'h30;       // 0

                else if(model == MODEL_L1X)
                    uart_data <= 8'h31;       // 1

                else
                    uart_data <= 8'h3F;       // ?


                uart_start <= 1'b1;

                report_index <= 6'd17;

            end


            // CR
            6'd17:
            begin
                uart_data <= 8'h0D;
                uart_start <= 1'b1;
                report_index <= 6'd18;
            end


            // LF
            6'd18:
            begin
                uart_data <= 8'h0A;
                uart_start <= 1'b1;
                report_index <= 6'd19;
            end


            default:
            begin

                report_active <= 1'b0;

                report_index <= 6'd0;

            end


            endcase

        end

    end

end


endmodule



////////////////////////////////////////////////////////////////////////////////
//
// I2C Register Read
//
// 支持:
//
// VL53L0X  8bit Register Address
//
// VL53L1X 16bit Register Address
//
// Device:
// 7-bit address = 0x29
//
// Write = 0x52
// Read  = 0x53
//
////////////////////////////////////////////////////////////////////////////////

module tof_i2c_reg_read
(
    input wire clk,
    input wire reset_n,

    input wire start,

    input wire addr16,

    input wire [15:0] reg_addr,

    inout wire i2c_scl,
    inout wire i2c_sda,

    output reg busy,
    output reg done,

    output reg ack_error,

    output reg [7:0] rd_data
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
// I2C时钟
//
// 50MHz
//
// 每一步250 clocks = 5us
//
// SCL约100kHz量级
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

    S_IDLE       = 5'd0,

    S_START1     = 5'd1,
    S_START2     = 5'd2,

    S_TX_LOW     = 5'd3,
    S_TX_HIGH    = 5'd4,

    S_ACK_LOW    = 5'd5,
    S_ACK_HIGH   = 5'd6,
    S_ACK_SAMPLE = 5'd7,

    S_REP_LOW    = 5'd8,
    S_REP_HIGH   = 5'd9,
    S_REP_START  = 5'd10,

    S_RX_LOW     = 5'd11,
    S_RX_HIGH    = 5'd12,
    S_RX_SAMPLE  = 5'd13,

    S_NACK_LOW   = 5'd14,
    S_NACK_HIGH  = 5'd15,

    S_STOP_LOW   = 5'd16,
    S_STOP_HIGH  = 5'd17,
    S_STOP_REL   = 5'd18,

    S_DONE       = 5'd19;


reg [4:0] state;


reg [7:0] tx_byte;

reg [7:0] rx_byte;


reg [2:0] bit_index;


// 0 = Device Write
// 1 = Register Byte1
// 2 = Register Byte2
// 3 = Device Read

reg [2:0] tx_stage;


reg addr16_latched;

reg [15:0] reg_addr_latched;


////////////////////////////////////////////////////////////////////////////////
// I2C State Machine
////////////////////////////////////////////////////////////////////////////////

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        scl_low <= 1'b0;

        sda_low <= 1'b0;

        busy <= 1'b0;

        done <= 1'b0;

        ack_error <= 1'b0;

        rd_data <= 8'h00;

        state <= S_IDLE;

        tx_byte <= 8'h00;

        rx_byte <= 8'h00;

        bit_index <= 3'd7;

        tx_stage <= 3'd0;

        addr16_latched <= 1'b0;

        reg_addr_latched <= 16'h0000;

    end

    else
    begin

        done <= 1'b0;


        ////////////////////////////////////////////////////////////////////////
        // Start Command
        ////////////////////////////////////////////////////////////////////////

        if((state == S_IDLE) && start)
        begin

            busy <= 1'b1;

            ack_error <= 1'b0;


            addr16_latched <= addr16;

            reg_addr_latched <= reg_addr;


            // VL53L0/1X Write Address
            tx_byte <= 8'h52;


            bit_index <= 3'd7;

            tx_stage <= 3'd0;


            scl_low <= 1'b0;

            sda_low <= 1'b0;


            state <= S_START1;

        end


        ////////////////////////////////////////////////////////////////////////
        // I2C Run
        ////////////////////////////////////////////////////////////////////////

        else if(busy && tick)
        begin

            case(state)


            ////////////////////////////////////////////////////////////////////
            // Bus Idle
            ////////////////////////////////////////////////////////////////////

            S_START1:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                state <= S_START2;

            end


            ////////////////////////////////////////////////////////////////////
            // START:
            //
            // SDA High -> Low
            // while SCL High
            ////////////////////////////////////////////////////////////////////

            S_START2:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b1;

                state <= S_TX_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // TX bit
            ////////////////////////////////////////////////////////////////////

            S_TX_LOW:
            begin

                scl_low <= 1'b1;


                if(tx_byte[bit_index] == 1'b0)

                    sda_low <= 1'b1;

                else

                    sda_low <= 1'b0;


                state <= S_TX_HIGH;

            end


            S_TX_HIGH:
            begin

                scl_low <= 1'b0;


                if(bit_index == 3'd0)
                begin

                    state <= S_ACK_LOW;

                end

                else
                begin

                    bit_index <= bit_index - 1'b1;

                    state <= S_TX_LOW;

                end

            end


            ////////////////////////////////////////////////////////////////////
            // ACK
            ////////////////////////////////////////////////////////////////////

            S_ACK_LOW:
            begin

                scl_low <= 1'b1;

                // Release SDA
                sda_low <= 1'b0;

                state <= S_ACK_HIGH;

            end


            S_ACK_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                state <= S_ACK_SAMPLE;

            end


            S_ACK_SAMPLE:
            begin

                if(sda_in != 1'b0)

                    ack_error <= 1'b1;


                case(tx_stage)


                ////////////////////////////////////////////////////////////////
                // Device Write ACK完成
                ////////////////////////////////////////////////////////////////

                3'd0:
                begin

                    if(addr16_latched)

                        tx_byte <= reg_addr_latched[15:8];

                    else

                        tx_byte <= reg_addr_latched[7:0];


                    bit_index <= 3'd7;

                    tx_stage <= 3'd1;

                    state <= S_TX_LOW;

                end


                ////////////////////////////////////////////////////////////////
                // 第一个Register Byte
                ////////////////////////////////////////////////////////////////

                3'd1:
                begin

                    if(addr16_latched)
                    begin

                        tx_byte <= reg_addr_latched[7:0];

                        bit_index <= 3'd7;

                        tx_stage <= 3'd2;

                        state <= S_TX_LOW;

                    end

                    else
                    begin

                        state <= S_REP_LOW;

                    end

                end


                ////////////////////////////////////////////////////////////////
                // 第二个Register Byte
                ////////////////////////////////////////////////////////////////

                3'd2:
                begin

                    state <= S_REP_LOW;

                end


                ////////////////////////////////////////////////////////////////
                // Device Read ACK
                ////////////////////////////////////////////////////////////////

                3'd3:
                begin

                    bit_index <= 3'd7;

                    rx_byte <= 8'h00;

                    state <= S_RX_LOW;

                end


                default:
                begin

                    state <= S_STOP_LOW;

                end


                endcase

            end


            ////////////////////////////////////////////////////////////////////
            // Repeated START
            ////////////////////////////////////////////////////////////////////

            S_REP_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b0;

                state <= S_REP_HIGH;

            end


            S_REP_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                state <= S_REP_START;

            end


            S_REP_START:
            begin

                // START
                sda_low <= 1'b1;

                scl_low <= 1'b0;


                // Device Read Address
                tx_byte <= 8'h53;

                bit_index <= 3'd7;

                tx_stage <= 3'd3;


                state <= S_TX_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // RX
            ////////////////////////////////////////////////////////////////////

            S_RX_LOW:
            begin

                scl_low <= 1'b1;

                // Release SDA
                sda_low <= 1'b0;

                state <= S_RX_HIGH;

            end


            S_RX_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                state <= S_RX_SAMPLE;

            end


            S_RX_SAMPLE:
            begin

                rx_byte[bit_index] <= sda_in;


                if(bit_index == 3'd0)
                begin

                    state <= S_NACK_LOW;

                end

                else
                begin

                    bit_index <= bit_index - 1'b1;

                    state <= S_RX_LOW;

                end

            end


            ////////////////////////////////////////////////////////////////////
            // Master发送NACK
            ////////////////////////////////////////////////////////////////////

            S_NACK_LOW:
            begin

                scl_low <= 1'b1;

                // SDA保持释放 = NACK
                sda_low <= 1'b0;

                state <= S_NACK_HIGH;

            end


            S_NACK_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                state <= S_STOP_LOW;

            end


            ////////////////////////////////////////////////////////////////////
            // STOP
            ////////////////////////////////////////////////////////////////////

            S_STOP_LOW:
            begin

                scl_low <= 1'b1;

                sda_low <= 1'b1;

                state <= S_STOP_HIGH;

            end


            S_STOP_HIGH:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b1;

                state <= S_STOP_REL;

            end


            S_STOP_REL:
            begin

                scl_low <= 1'b0;

                // SDA Low -> High
                sda_low <= 1'b0;

                state <= S_DONE;

            end


            ////////////////////////////////////////////////////////////////////
            // Done
            ////////////////////////////////////////////////////////////////////

            S_DONE:
            begin

                rd_data <= rx_byte;

                busy <= 1'b0;

                done <= 1'b1;

                state <= S_IDLE;

            end


            default:
            begin

                scl_low <= 1'b0;

                sda_low <= 1'b0;

                busy <= 1'b0;

                state <= S_IDLE;

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
// 50MHz
// 115200
// 8N1
//
////////////////////////////////////////////////////////////////////////////////

module uart_tx_115200
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

        ////////////////////////////////////////////////////////////////////////
        // Idle
        ////////////////////////////////////////////////////////////////////////

        if(!busy)
        begin

            tx <= 1'b1;

            baud_cnt <= 16'd0;

            bit_index <= 4'd0;


            if(start)
            begin

                tx_data <= data;

                busy <= 1'b1;

                // Start Bit
                tx <= 1'b0;

            end

        end


        ////////////////////////////////////////////////////////////////////////
        // Sending
        ////////////////////////////////////////////////////////////////////////

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