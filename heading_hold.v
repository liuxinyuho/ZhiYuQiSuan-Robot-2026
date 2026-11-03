//////////////////////////////////////////////////////////////////////////////////
// File Name   : heading_hold.v
// Project     : FPGA Mecanum Robot Chassis
//
// Description :
//   IMU航向保持控制器
//
// 优化版：
//   1. 去除运行时除法器
//   2. 航向误差与Wz输出之间加入流水寄存器
//   3. 保留KP_DIV参数，top.v无需修改
//////////////////////////////////////////////////////////////////////////////////

module heading_hold #(

    // correction ≈ error / KP_DIV
    parameter signed [31:0] KP_DIV =
        32'sd5,

    parameter signed [12:0] MAX_CORRECTION =
        13'sd400,

    parameter REVERSE =
        1'b0

)(
    input wire clk,
    input wire reset_n,

    input wire signed [12:0] vx_cmd,
    input wire signed [12:0] vy_cmd,
    input wire signed [12:0] wz_user,

    input wire calib_done,

    input wire signed [31:0] yaw_cdeg,

    output reg signed [12:0] wz_final,

    output reg signed [31:0] target_yaw,
    output reg signed [31:0] yaw_error,

    output reg hold_active
);


    //================================================
    // 是否平移
    //================================================

    wire moving_xy;

    assign moving_xy =
        (vx_cmd != 13'sd0)
        ||
        (vy_cmd != 13'sd0);


    //================================================
    // 是否主动旋转
    //================================================

    wire rotating_manual;

    assign rotating_manual =
        (wz_user != 13'sd0);


    //================================================
    // 航向误差组合计算
    //================================================

    wire signed [31:0] error_raw;

    reg signed [31:0] yaw_error_comb;


    assign error_raw =
        target_yaw - yaw_cdeg;


    always @(*) begin

        if(error_raw > 32'sd18000)

            yaw_error_comb =
                error_raw - 32'sd36000;

        else if(error_raw < -32'sd18000)

            yaw_error_comb =
                error_raw + 32'sd36000;

        else

            yaw_error_comb =
                error_raw;

    end


    //================================================
    // 快速P控制计算
    //
    // 原来：
    //
    // correction = error / KP_DIV
    //
    // 现在使用Q14倒数：
    //
    // reciprocal =
    //     16384 / KP_DIV
    //
    // correction =
    //     error * reciprocal >> 14
    //
    // KP_DIV是parameter，因此这里的除法只在
    // 综合时计算，不会生成运行时硬件除法器。
    //
    // 例如KP_DIV=60：
    //
    // 16384 / 60 ≈ 273
    //
    // correction ≈ error * 273 / 16384
    //================================================

    localparam signed [31:0] KP_RECIP_Q14 =
        (
            32'sd16384
            +
            (KP_DIV >>> 1)
        )
        /
        KP_DIV;


    wire signed [31:0] correction_product;

    wire signed [31:0] correction_calc;


    assign correction_product =
        yaw_error_comb *
        KP_RECIP_Q14;


    assign correction_calc =
        correction_product >>> 14;


    //================================================
    // 限幅
    //================================================

    reg signed [12:0] correction_limited_comb;


    always @(*) begin

        if(correction_calc > MAX_CORRECTION)

            correction_limited_comb =
                MAX_CORRECTION;

        else if(correction_calc < -MAX_CORRECTION)

            correction_limited_comb =
                -MAX_CORRECTION;

        else

            correction_limited_comb =
                correction_calc[12:0];


        //============================================
        // 纠偏方向
        //============================================

        if(REVERSE)

            correction_limited_comb =
                -correction_limited_comb;

    end


    //================================================
    // 纠偏寄存器
    //
    // 这一层非常重要：
    //
    // yaw_cdeg
    //   ↓
    // error/correction
    //   ↓
    // correction_reg
    //   ↓
    // wz_final
    //
    // 打断原来的超长组合路径
    //================================================

    reg signed [12:0] correction_reg;


    //================================================
    // 航向保持状态
    //================================================

    always @(posedge clk or negedge reset_n) begin

        if(!reset_n) begin

            target_yaw <= 32'sd0;

            yaw_error <= 32'sd0;

            correction_reg <= 13'sd0;

            hold_active <= 1'b0;

            wz_final <= 13'sd0;

        end

        else begin

            //================================================
            // 每个时钟先锁存计算结果
            //================================================

            yaw_error <= yaw_error_comb;

            correction_reg <=
                correction_limited_comb;


            //================================================
            // IMU未校准
            //================================================

            if(!calib_done) begin

                target_yaw <= yaw_cdeg;

                hold_active <= 1'b0;

                wz_final <= wz_user;

            end


            //================================================
            // 用户主动旋转
            //================================================

            else if(rotating_manual) begin

                target_yaw <= yaw_cdeg;

                hold_active <= 1'b0;

                wz_final <= wz_user;

            end


            //================================================
            // 平移
            //================================================

            else if(moving_xy) begin

                if(!hold_active) begin

                    // 第一次平移，记录目标Yaw
                    target_yaw <= yaw_cdeg;

                    hold_active <= 1'b1;

                    wz_final <= 13'sd0;

                end

                else begin

                    // 使用上一拍已经锁存好的修正量
                    wz_final <= correction_reg;

                end

            end


            //================================================
            // 停车
            //================================================

            else begin

                target_yaw <= yaw_cdeg;

                hold_active <= 1'b0;

                wz_final <= 13'sd0;

            end

        end

    end


endmodule