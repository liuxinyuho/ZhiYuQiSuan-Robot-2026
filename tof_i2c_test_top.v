////////////////////////////////////////////////////////////////////////////////
// GW5AT60K TOF I2C ACK TEST
//
// Clock : 50MHz
//
// TOF:
// VL53L0X / VL53L1X
//
// Default 7-bit address : 0x29
// 8-bit write address   : 0x52
//
// UART:
// 115200 8N1
//
// Output:
//
// TOF=1   -> I2C ACK正常
// TOF=0   -> 没有ACK
////////////////////////////////////////////////////////////////////////////////

module tof_i2c_test_top
(
    input  wire clk,
    input  wire reset_n,

    inout  wire imu_sda,
    inout  wire imu_scl,

    output wire uart_tx
);


////////////////////////////////////////////////////////////////////////////////
// 每1秒检测一次
////////////////////////////////////////////////////////////////////////////////

reg [25:0] sec_cnt;
reg        probe_start;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        sec_cnt     <= 26'd0;
        probe_start <= 1'b0;

    end

    else
    begin

        probe_start <= 1'b0;


        if(sec_cnt >= 26'd49_999_999)
        begin

            sec_cnt <= 26'd0;

            probe_start <= 1'b1;

        end

        else
        begin

            sec_cnt <= sec_cnt + 1'b1;

        end

    end

end


////////////////////////////////////////////////////////////////////////////////
// I2C Probe
////////////////////////////////////////////////////////////////////////////////

wire probe_busy;
wire probe_done;
wire probe_ack;


i2c_address_probe
u_i2c_address_probe
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
        probe_start
    ),

    // VL53L0X / VL53L1X
    //
    // 7-bit = 0x29
    //
    // Write address = 0x52
    .device_address
    (
        8'h52
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
        probe_busy
    ),

    .done
    (
        probe_done
    ),

    .ack_ok
    (
        probe_ack
    )
);


////////////////////////////////////////////////////////////////////////////////
// 保存检测结果
////////////////////////////////////////////////////////////////////////////////

reg tof_present;

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        tof_present <= 1'b0;

    end

    else
    begin

        if(probe_done)
            tof_present <= probe_ack;

    end

end


////////////////////////////////////////////////////////////////////////////////
// UART报告
//
// TOF=1\r\n
//
// 或
//
// TOF=0\r\n
////////////////////////////////////////////////////////////////////////////////

reg        uart_start;
reg [7:0]  uart_data;

wire uart_busy;


uart_tx_115200
u_uart_tx
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
// UART字符串发送
////////////////////////////////////////////////////////////////////////////////

reg       report_active;
reg [2:0] report_index;

reg       report_value;


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        uart_start    <= 1'b0;
        uart_data     <= 8'h00;

        report_active <= 1'b0;
        report_index  <= 3'd0;

        report_value  <= 1'b0;

    end

    else
    begin

        uart_start <= 1'b0;


        ////////////////////////////////////////////////////////////////
        // I2C扫描完成
        ////////////////////////////////////////////////////////////////

        if(probe_done && !report_active)
        begin

            report_active <= 1'b1;

            report_index <= 3'd0;

            report_value <= probe_ack;

        end


        ////////////////////////////////////////////////////////////////
        // UART空闲
        ////////////////////////////////////////////////////////////////

        if(report_active && !uart_busy)
        begin

            case(report_index)

                // T
                3'd0:
                begin

                    uart_data  <= 8'h54;
                    uart_start <= 1'b1;

                    report_index <= 3'd1;

                end


                // O
                3'd1:
                begin

                    uart_data  <= 8'h4F;
                    uart_start <= 1'b1;

                    report_index <= 3'd2;

                end


                // F
                3'd2:
                begin

                    uart_data  <= 8'h46;
                    uart_start <= 1'b1;

                    report_index <= 3'd3;

                end


                // =
                3'd3:
                begin

                    uart_data  <= 8'h3D;
                    uart_start <= 1'b1;

                    report_index <= 3'd4;

                end


                // 1 / 0
                3'd4:
                begin

                    if(report_value)
                        uart_data <= 8'h31;
                    else
                        uart_data <= 8'h30;

                    uart_start <= 1'b1;

                    report_index <= 3'd5;

                end


                // \r
                3'd5:
                begin

                    uart_data  <= 8'h0D;
                    uart_start <= 1'b1;

                    report_index <= 3'd6;

                end


                // \n
                3'd6:
                begin

                    uart_data  <= 8'h0A;
                    uart_start <= 1'b1;

                    report_index <= 3'd7;

                end


                default:
                begin

                    report_active <= 1'b0;
                    report_index  <= 3'd0;

                end

            endcase

        end

    end

end


endmodule



////////////////////////////////////////////////////////////////////////////////
//
// I2C ADDRESS PROBE
//
// 50MHz
//
// SCL ≈ 100kHz
//
// Open-drain SDA / SCL
//
////////////////////////////////////////////////////////////////////////////////

module i2c_address_probe
(
    input  wire       clk,
    input  wire       reset_n,

    input  wire       start,

    input  wire [7:0] device_address,

    inout  wire       i2c_scl,
    inout  wire       i2c_sda,

    output reg        busy,
    output reg        done,
    output reg        ack_ok
);


////////////////////////////////////////////////////////////////////////////////
// Open Drain
////////////////////////////////////////////////////////////////////////////////

reg scl_drive_low;
reg sda_drive_low;


assign i2c_scl =
       scl_drive_low
       ?
       1'b0
       :
       1'bz;


assign i2c_sda =
       sda_drive_low
       ?
       1'b0
       :
       1'bz;


wire sda_in;

assign sda_in = i2c_sda;


////////////////////////////////////////////////////////////////////////////////
// 100kHz
//
// 50MHz / 100kHz = 500
//
// 半周期 = 250 clocks
////////////////////////////////////////////////////////////////////////////////

reg [8:0] clk_cnt;

wire step_tick;


assign step_tick =
       (clk_cnt == 9'd249);


always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        clk_cnt <= 9'd0;

    end

    else if(!busy)
    begin

        clk_cnt <= 9'd0;

    end

    else
    begin

        if(step_tick)
            clk_cnt <= 9'd0;

        else
            clk_cnt <= clk_cnt + 1'b1;

    end

end


////////////////////////////////////////////////////////////////////////////////
// State
////////////////////////////////////////////////////////////////////////////////

localparam
    ST_IDLE        = 5'd0,
    ST_START_1     = 5'd1,
    ST_START_2     = 5'd2,

    ST_BIT_LOW     = 5'd3,
    ST_BIT_HIGH    = 5'd4,

    ST_ACK_LOW     = 5'd5,
    ST_ACK_HIGH_1  = 5'd6,
    ST_ACK_HIGH_2  = 5'd7,

    ST_STOP_LOW    = 5'd8,
    ST_STOP_HIGH   = 5'd9,
    ST_STOP_RELEASE= 5'd10,

    ST_DONE        = 5'd11;


reg [4:0] state;

reg [7:0] tx_byte;

reg [2:0] bit_index;


////////////////////////////////////////////////////////////////////////////////
// I2C State Machine
////////////////////////////////////////////////////////////////////////////////

always @(posedge clk or negedge reset_n)
begin

    if(!reset_n)
    begin

        state <= ST_IDLE;

        scl_drive_low <= 1'b0;
        sda_drive_low <= 1'b0;

        tx_byte   <= 8'h00;
        bit_index <= 3'd7;

        busy   <= 1'b0;
        done   <= 1'b0;
        ack_ok <= 1'b0;

    end

    else
    begin

        done <= 1'b0;


        ////////////////////////////////////////////////////////////////
        // Start请求
        ////////////////////////////////////////////////////////////////

        if((state == ST_IDLE) && start)
        begin

            tx_byte <= device_address;

            bit_index <= 3'd7;

            ack_ok <= 1'b0;

            busy <= 1'b1;

            scl_drive_low <= 1'b0;
            sda_drive_low <= 1'b0;

            state <= ST_START_1;

        end


        ////////////////////////////////////////////////////////////////
        // I2C进行中
        ////////////////////////////////////////////////////////////////

        else if(busy && step_tick)
        begin

            case(state)


            ////////////////////////////////////////////////////////////
            // Bus Idle
            ////////////////////////////////////////////////////////////

            ST_START_1:
            begin

                scl_drive_low <= 1'b0;
                sda_drive_low <= 1'b0;

                state <= ST_START_2;

            end


            ////////////////////////////////////////////////////////////
            // START
            //
            // SDA: 1 -> 0
            //
            // SCL保持高
            ////////////////////////////////////////////////////////////

            ST_START_2:
            begin

                scl_drive_low <= 1'b0;
                sda_drive_low <= 1'b1;

                state <= ST_BIT_LOW;

            end


            ////////////////////////////////////////////////////////////
            // 数据准备
            //
            // SCL Low
            ////////////////////////////////////////////////////////////

            ST_BIT_LOW:
            begin

                scl_drive_low <= 1'b1;


                if(tx_byte[bit_index] == 1'b0)
                    sda_drive_low <= 1'b1;
                else
                    sda_drive_low <= 1'b0;


                state <= ST_BIT_HIGH;

            end


            ////////////////////////////////////////////////////////////
            // 数据有效
            //
            // SCL High
            ////////////////////////////////////////////////////////////

            ST_BIT_HIGH:
            begin

                scl_drive_low <= 1'b0;


                if(bit_index == 3'd0)
                begin

                    state <= ST_ACK_LOW;

                end

                else
                begin

                    bit_index <= bit_index - 1'b1;

                    state <= ST_BIT_LOW;

                end

            end


            ////////////////////////////////////////////////////////////
            // ACK准备
            //
            // FPGA释放SDA
            ////////////////////////////////////////////////////////////

            ST_ACK_LOW:
            begin

                scl_drive_low <= 1'b1;

                sda_drive_low <= 1'b0;

                state <= ST_ACK_HIGH_1;

            end


            ////////////////////////////////////////////////////////////
            // SCL High
            ////////////////////////////////////////////////////////////

            ST_ACK_HIGH_1:
            begin

                scl_drive_low <= 1'b0;

                sda_drive_low <= 1'b0;

                state <= ST_ACK_HIGH_2;

            end


            ////////////////////////////////////////////////////////////
            // Sample ACK
            //
            // SDA = 0 -> ACK
            ////////////////////////////////////////////////////////////

            ST_ACK_HIGH_2:
            begin

                if(sda_in == 1'b0)
                    ack_ok <= 1'b1;
                else
                    ack_ok <= 1'b0;


                state <= ST_STOP_LOW;

            end


            ////////////////////////////////////////////////////////////
            // STOP 1
            ////////////////////////////////////////////////////////////

            ST_STOP_LOW:
            begin

                scl_drive_low <= 1'b1;

                sda_drive_low <= 1'b1;

                state <= ST_STOP_HIGH;

            end


            ////////////////////////////////////////////////////////////
            // STOP 2
            //
            // SCL High
            // SDA Low
            ////////////////////////////////////////////////////////////

            ST_STOP_HIGH:
            begin

                scl_drive_low <= 1'b0;

                sda_drive_low <= 1'b1;

                state <= ST_STOP_RELEASE;

            end


            ////////////////////////////////////////////////////////////
            // STOP
            //
            // SDA Low -> High
            ////////////////////////////////////////////////////////////

            ST_STOP_RELEASE:
            begin

                scl_drive_low <= 1'b0;

                sda_drive_low <= 1'b0;

                state <= ST_DONE;

            end


            ////////////////////////////////////////////////////////////
            // Finished
            ////////////////////////////////////////////////////////////

            ST_DONE:
            begin

                busy <= 1'b0;

                done <= 1'b1;

                state <= ST_IDLE;

            end


            default:
            begin

                busy <= 1'b0;

                scl_drive_low <= 1'b0;
                sda_drive_low <= 1'b0;

                state <= ST_IDLE;

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
// Clock    = 50MHz
// Baudrate = 115200
// 8N1
//
////////////////////////////////////////////////////////////////////////////////

module uart_tx_115200
(
    input  wire       clk,
    input  wire       reset_n,

    input  wire       start,
    input  wire [7:0] data,

    output reg        tx,
    output reg        busy
);


////////////////////////////////////////////////////////////////////////////////
// 50MHz / 115200 ≈ 434
////////////////////////////////////////////////////////////////////////////////

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

        ////////////////////////////////////////////////////////////////
        // Start
        ////////////////////////////////////////////////////////////////

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


        ////////////////////////////////////////////////////////////////
        // Sending
        ////////////////////////////////////////////////////////////////

        else
        begin

            if(baud_cnt >= CLKS_PER_BIT - 1'b1)
            begin

                baud_cnt <= 16'd0;

                bit_index <= bit_index + 1'b1;


                case(bit_index)

                    // Data bit0
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

                    // Stop
                    4'd8:
                        tx <= 1'b1;

                    // Finish
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