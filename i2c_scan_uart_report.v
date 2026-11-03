//////////////////////////////////////////////////////////////////////////////////
// File Name   : i2c_scan_uart_report.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   将I2C扫描结果通过UART发送到PC。
//   自动遍历found_map并打印所有检测到的地址。
//
// Example:
//
//   I2C_FOUND=0x68
//   SCAN_DONE
//
// UART:
//   115200 / 8N1
//////////////////////////////////////////////////////////////////////////////////

module i2c_scan_uart_report(
    input wire clk,
    input wire reset_n,

    input wire         scan_done,
    input wire [127:0] found_map,

    output wire uart_tx
);


    //================================================
    // UART TX接口
    //================================================

    reg       tx_send;
    reg [7:0] tx_data;

    wire tx_busy;
    wire tx_done;


    uart_byte_tx u_uart_byte_tx(

        .clk       (clk),
        .reset_n   (reset_n),

        .data_byte (tx_data),
        .send_en   (tx_send),

        .uart_tx   (uart_tx),

        .tx_done   (tx_done),
        .tx_busy   (tx_busy)

    );


    //================================================
    // HEX -> ASCII
    //================================================

    function [7:0] hex_ascii;

        input [3:0] hex;

        begin

            if(hex < 4'd10)

                hex_ascii =
                    8'h30 + hex;

            else

                hex_ascii =
                    8'h41 + hex - 4'd10;

        end

    endfunction


    //================================================
    // 当前正在检查/打印的地址
    //================================================

    reg [6:0] addr_index;

    reg [6:0] addr_latch;


    //================================================
    // 发送状态
    //================================================

    localparam MODE_SEARCH = 2'd0;
    localparam MODE_ADDR   = 2'd1;
    localparam MODE_DONE   = 2'd2;
    localparam MODE_END    = 2'd3;

    reg [1:0] mode;


    reg [4:0] char_index;

    reg waiting_tx;

    reg started;


    //================================================
    // 根据当前模式产生字符
    //================================================

    always @(*) begin

        tx_data = 8'h20;


        //============================================
        // 输出：
        //
        // I2C_FOUND=0x68\r\n
        //============================================

        if(mode == MODE_ADDR) begin

            case(char_index)

                5'd0 : tx_data = "I";
                5'd1 : tx_data = "2";
                5'd2 : tx_data = "C";
                5'd3 : tx_data = "_";
                5'd4 : tx_data = "F";
                5'd5 : tx_data = "O";
                5'd6 : tx_data = "U";
                5'd7 : tx_data = "N";
                5'd8 : tx_data = "D";
                5'd9 : tx_data = "=";
                5'd10: tx_data = "0";
                5'd11: tx_data = "x";

                5'd12:
                    tx_data =
                        hex_ascii({1'b0, addr_latch[6:4]});

                5'd13:
                    tx_data =
                        hex_ascii(addr_latch[3:0]);

                5'd14:
                    tx_data = 8'h0D;

                5'd15:
                    tx_data = 8'h0A;

                default:
                    tx_data = 8'h20;

            endcase

        end


        //============================================
        // 输出：
        //
        // SCAN_DONE\r\n
        //============================================

        else if(mode == MODE_DONE) begin

            case(char_index)

                5'd0 : tx_data = "S";
                5'd1 : tx_data = "C";
                5'd2 : tx_data = "A";
                5'd3 : tx_data = "N";
                5'd4 : tx_data = "_";
                5'd5 : tx_data = "D";
                5'd6 : tx_data = "O";
                5'd7 : tx_data = "N";
                5'd8 : tx_data = "E";

                5'd9 :
                    tx_data = 8'h0D;

                5'd10:
                    tx_data = 8'h0A;

                default:
                    tx_data = 8'h20;

            endcase

        end

    end


    //================================================
    // 串口结果发送状态机
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            addr_index <= 7'h08;
            addr_latch <= 7'd0;

            mode <= MODE_SEARCH;

            char_index <= 5'd0;

            waiting_tx <= 1'b0;

            tx_send <= 1'b0;

            started <= 1'b0;

        end

        else begin

            tx_send <= 1'b0;


            //========================================
            // 等待扫描完成
            //========================================

            if(!started) begin

                if(scan_done) begin

                    started <= 1'b1;

                    addr_index <= 7'h08;

                    mode <= MODE_SEARCH;

                end

            end


            //========================================
            // 已开始输出
            //========================================

            else begin

                case(mode)


                    //================================
                    // 查找下一个存在设备的地址
                    //================================

                    MODE_SEARCH: begin

                        waiting_tx <= 1'b0;

                        if(addr_index > 7'h77) begin

                            char_index <= 5'd0;

                            mode <= MODE_DONE;

                        end


                        else if(found_map[addr_index]) begin

                            addr_latch <= addr_index;

                            char_index <= 5'd0;

                            mode <= MODE_ADDR;

                        end


                        else begin

                            addr_index <=
                                addr_index + 1'b1;

                        end

                    end


                    //================================
                    // 打印一个地址
                    //================================

                    MODE_ADDR: begin

                        if(!waiting_tx && !tx_busy) begin

                            tx_send <= 1'b1;

                            waiting_tx <= 1'b1;

                        end


                        else if(waiting_tx && tx_done) begin

                            waiting_tx <= 1'b0;


                            if(char_index >= 5'd15) begin

                                addr_index <=
                                    addr_latch + 1'b1;

                                mode <= MODE_SEARCH;

                            end

                            else begin

                                char_index <=
                                    char_index + 1'b1;

                            end

                        end

                    end


                    //================================
                    // 打印SCAN_DONE
                    //================================

                    MODE_DONE: begin

                        if(!waiting_tx && !tx_busy) begin

                            tx_send <= 1'b1;

                            waiting_tx <= 1'b1;

                        end


                        else if(waiting_tx && tx_done) begin

                            waiting_tx <= 1'b0;


                            if(char_index >= 5'd10) begin

                                mode <= MODE_END;

                            end

                            else begin

                                char_index <=
                                    char_index + 1'b1;

                            end

                        end

                    end


                    //================================
                    // 输出完成
                    //================================

                    MODE_END: begin

                        mode <= MODE_END;

                    end


                    default: begin

                        mode <= MODE_SEARCH;

                    end

                endcase

            end

        end

    end


endmodule