//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_scanner.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   I2C总线地址扫描模块。
//   自动扫描标准7位I2C地址范围0x08 ~ 0x77。
//   如果某个地址收到ACK，则在found_map中记录。
//
// Example:
//
//   如果0x68存在设备：
//
//       found_map[8'h68] = 1
//
// Output:
//
//   scan_done   - 扫描完成
//   found_map   - 128位设备地址表
//   found_count - 一共发现多少个I2C设备
//////////////////////////////////////////////////////////////////////////////////

module i2c_scanner(
    input wire clk,
    input wire reset_n,

    inout wire imu_sda,
    inout wire imu_scl,

    output reg         scan_done,
    output reg [127:0] found_map,
    output reg [7:0]   found_count
);


    //================================================
    // 地址探测器接口
    //================================================

    reg probe_start;

    reg [6:0] scan_addr;

    wire probe_busy;
    wire probe_done;
    wire probe_ack_ok;


    i2c_addr_probe u_i2c_addr_probe(

        .clk      (clk),
        .reset_n  (reset_n),

        .start    (probe_start),

        .dev_addr (scan_addr),

        .imu_sda  (imu_sda),
        .imu_scl  (imu_scl),

        .busy     (probe_busy),
        .done     (probe_done),
        .ack_ok   (probe_ack_ok)

    );


    //================================================
    // 状态定义
    //================================================

    localparam ST_POWER_WAIT = 3'd0;
    localparam ST_START      = 3'd1;
    localparam ST_WAIT       = 3'd2;
    localparam ST_NEXT       = 3'd3;
    localparam ST_FINISH     = 3'd4;


    reg [2:0] state;


    //================================================
    // MPU上电等待约100ms
    //
    // 50MHz × 0.1s = 5,000,000
    //================================================

    reg [22:0] power_cnt;


    //================================================
    // 主扫描状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            state <= ST_POWER_WAIT;

            power_cnt <= 23'd0;

            scan_addr <= 7'h08;

            probe_start <= 1'b0;

            scan_done <= 1'b0;

            found_map <= 128'd0;

            found_count <= 8'd0;

        end

        else begin

            // 默认只产生一个CLK的start
            probe_start <= 1'b0;


            case(state)


                //====================================
                // 上电等待100ms
                //====================================

                ST_POWER_WAIT: begin

                    if(power_cnt >= 23'd4999999) begin

                        power_cnt <= 23'd0;

                        scan_addr <= 7'h08;

                        found_map <= 128'd0;

                        found_count <= 8'd0;

                        scan_done <= 1'b0;

                        state <= ST_START;

                    end

                    else begin

                        power_cnt <=
                            power_cnt + 1'b1;

                    end

                end


                //====================================
                // 启动当前地址探测
                //====================================

                ST_START: begin

                    if(!probe_busy) begin

                        probe_start <= 1'b1;

                        state <= ST_WAIT;

                    end

                end


                //====================================
                // 等待当前地址完成
                //====================================

                ST_WAIT: begin

                    if(probe_done) begin

                        // 有ACK
                        if(probe_ack_ok) begin

                            found_map[scan_addr] <= 1'b1;

                            found_count <=
                                found_count + 1'b1;

                        end

                        state <= ST_NEXT;

                    end

                end


                //====================================
                // 下一个地址
                //====================================

                ST_NEXT: begin

                    // 0x77已经扫描完成
                    if(scan_addr >= 7'h77) begin

                        state <= ST_FINISH;

                    end

                    else begin

                        scan_addr <=
                            scan_addr + 1'b1;

                        state <= ST_START;

                    end

                end


                //====================================
                // 扫描完成
                //====================================

                ST_FINISH: begin

                    scan_done <= 1'b1;

                    // 保持在这里
                    state <= ST_FINISH;

                end


                default: begin

                    state <= ST_POWER_WAIT;

                end

            endcase

        end

    end


endmodule