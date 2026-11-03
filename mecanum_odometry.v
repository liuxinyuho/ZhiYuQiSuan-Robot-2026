
`timescale 1ns / 1ps

module mecanum_odometry #(
    parameter signed [31:0] MAX_WHEEL_DELTA = 32'sd128
)(
    input wire clk,
    input wire reset_n,
    input wire sample_tick,

    input wire signed [31:0] encoder_a_position,
    input wire signed [31:0] encoder_b_position,
    input wire signed [31:0] encoder_c_position,
    input wire signed [31:0] encoder_d_position,

    input wire signed [31:0] yaw_cdeg,
    input wire freeze_translation,

    output reg signed [31:0] delta_x_count,
    output reg signed [31:0] delta_y_count,
    output reg signed [31:0] delta_w_count,
    output reg signed [31:0] pos_x_count,
    output reg signed [31:0] pos_y_count,
    output reg signed [31:0] world_x_mm,
    output reg signed [31:0] world_y_mm,
    output reg signed [31:0] odom_yaw_cdeg
);

    // Original empirical wheel-count scaling.
    localparam signed [15:0] X_SCALE_Q14 = 16'sd3244;
    localparam signed [15:0] Y_SCALE_Q14 = 16'sd2572;

    // Stage 0: encoder snapshot.
    reg signed [31:0] a0, b0, c0, d0;
    reg signed [31:0] yaw0;
    reg freeze0, v0;

    // Stage 1: bounded wheel deltas.
    reg signed [31:0] pa, pb, pc, pd;
    reg first_sample;
    reg signed [15:0] da1, db1, dc1, dd1;
    reg signed [31:0] yaw1;
    reg freeze1, v1;

    wire signed [31:0] wa = a0 - pa;
    wire signed [31:0] wb = b0 - pb;
    wire signed [31:0] wc = c0 - pc;
    wire signed [31:0] wd = d0 - pd;

    wire wheel_deltas_ok =
        wa >= -MAX_WHEEL_DELTA &&
        wa <=  MAX_WHEEL_DELTA &&
        wb >= -MAX_WHEEL_DELTA &&
        wb <=  MAX_WHEEL_DELTA &&
        wc >= -MAX_WHEEL_DELTA &&
        wc <=  MAX_WHEEL_DELTA &&
        wd >= -MAX_WHEEL_DELTA &&
        wd <=  MAX_WHEEL_DELTA;

    // Stage 2: signed pair sums.
    reg signed [17:0] ab2, cd2, ad2, bc2;
    reg signed [31:0] yaw2;
    reg freeze2, v2;

    wire signed [17:0] ax = {{2{da1[15]}}, da1};
    wire signed [17:0] bx = {{2{db1[15]}}, db1};
    wire signed [17:0] cx = {{2{dc1[15]}}, dc1};
    wire signed [17:0] dx = {{2{dd1[15]}}, dd1};

    // Stage 3: mecanum forward kinematics.
    reg signed [15:0] body_dx3, body_dy3, body_dw3;
    reg signed [31:0] yaw3;
    reg freeze3, v3;

    wire signed [17:0] body_x_wide =
        (ab2 + cd2) >>> 2;

    wire signed [17:0] body_y_wide =
        (ad2 - bc2) >>> 2;

    wire signed [17:0] body_w_wide =
        (ab2 - cd2) >>> 2;

    // Stage 4: count to Q14 millimetres.
    reg signed [31:0] dx_q14_4, dy_q14_4;
    reg signed [31:0] yaw4;
    reg freeze4, v4;

    // Stage 5: body to world coordinates.
    reg signed [31:0] world_dx_q14_5;
    reg signed [31:0] world_dy_q14_5;
    reg freeze5, v5;

    // Stage 6: position accumulator.
    reg signed [47:0] world_x_acc_q14;
    reg signed [47:0] world_y_acc_q14;
    reg v6;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            a0 <= 0;
            b0 <= 0;
            c0 <= 0;
            d0 <= 0;
            yaw0 <= 0;
            freeze0 <= 0;
            v0 <= 0;

            pa <= 0;
            pb <= 0;
            pc <= 0;
            pd <= 0;
            first_sample <= 1'b1;

            da1 <= 0;
            db1 <= 0;
            dc1 <= 0;
            dd1 <= 0;
            yaw1 <= 0;
            freeze1 <= 0;
            v1 <= 0;

            ab2 <= 0;
            cd2 <= 0;
            ad2 <= 0;
            bc2 <= 0;
            yaw2 <= 0;
            freeze2 <= 0;
            v2 <= 0;

            body_dx3 <= 0;
            body_dy3 <= 0;
            body_dw3 <= 0;
            yaw3 <= 0;
            freeze3 <= 0;
            v3 <= 0;

            dx_q14_4 <= 0;
            dy_q14_4 <= 0;
            yaw4 <= 0;
            freeze4 <= 0;
            v4 <= 0;

            world_dx_q14_5 <= 0;
            world_dy_q14_5 <= 0;
            freeze5 <= 0;
            v5 <= 0;

            world_x_acc_q14 <= 0;
            world_y_acc_q14 <= 0;
            v6 <= 0;

            delta_x_count <= 0;
            delta_y_count <= 0;
            delta_w_count <= 0;
            pos_x_count <= 0;
            pos_y_count <= 0;
            world_x_mm <= 0;
            world_y_mm <= 0;
            odom_yaw_cdeg <= 0;

        end else begin

            // Stage 0
            v0 <= 1'b0;

            if (sample_tick) begin
                a0 <= encoder_a_position;
                b0 <= encoder_b_position;
                c0 <= encoder_c_position;
                d0 <= encoder_d_position;
                yaw0 <= yaw_cdeg;
                freeze0 <= freeze_translation;
                odom_yaw_cdeg <= yaw_cdeg;
                v0 <= 1'b1;
            end

            // Stage 1
            v1 <= 1'b0;

            if (v0) begin
                pa <= a0;
                pb <= b0;
                pc <= c0;
                pd <= d0;

                if (first_sample) begin
                    first_sample <= 1'b0;
                end else if (wheel_deltas_ok) begin
                    da1 <= wa[15:0];
                    db1 <= wb[15:0];
                    dc1 <= wc[15:0];
                    dd1 <= wd[15:0];

                    yaw1 <= yaw0;
                    freeze1 <= freeze0;
                    v1 <= 1'b1;
                end
            end

            // Stage 2
            v2 <= v1;

            if (v1) begin
                ab2 <= ax + bx;
                cd2 <= cx + dx;
                ad2 <= ax + dx;
                bc2 <= bx + cx;

                yaw2 <= yaw1;
                freeze2 <= freeze1;
            end

            // Stage 3
            v3 <= v2;

            if (v2) begin
                body_dx3 <= body_x_wide[15:0];
                body_dy3 <= body_y_wide[15:0];
                body_dw3 <= body_w_wide[15:0];

                yaw3 <= yaw2;
                freeze3 <= freeze2;
            end

            // Stage 4
            v4 <= v3;

            if (v3) begin
                delta_x_count <=
                    {{16{body_dx3[15]}}, body_dx3};
                delta_y_count <=
                    {{16{body_dy3[15]}}, body_dy3};
                delta_w_count <=
                    {{16{body_dw3[15]}}, body_dw3};

                pos_x_count <= pos_x_count +
                    {{16{body_dx3[15]}}, body_dx3};

                pos_y_count <= pos_y_count +
                    {{16{body_dy3[15]}}, body_dy3};

                dx_q14_4 <= $signed(body_dx3) * X_SCALE_Q14;
                dy_q14_4 <= $signed(body_dy3) * Y_SCALE_Q14;

                yaw4 <= yaw3;
                freeze4 <= freeze3;
            end

            // Stage 5: original four-quadrant yaw mapping.
            v5 <= v4;

            if (v4) begin
                freeze5 <= freeze4;

                if (
                    yaw4 >= -32'sd4500 &&
                    yaw4 <   32'sd4500
                ) begin
                    world_dx_q14_5 <= dx_q14_4;
                    world_dy_q14_5 <= dy_q14_4;

                end else if (
                    yaw4 >= -32'sd13500 &&
                    yaw4 <  -32'sd4500
                ) begin
                    world_dx_q14_5 <= dy_q14_4;
                    world_dy_q14_5 <= -dx_q14_4;

                end else if (
                    yaw4 >= 32'sd4500 &&
                    yaw4 <  32'sd13500
                ) begin
                    world_dx_q14_5 <= -dy_q14_4;
                    world_dy_q14_5 <= dx_q14_4;

                end else begin
                    world_dx_q14_5 <= -dx_q14_4;
                    world_dy_q14_5 <= -dy_q14_4;
                end
            end

            // Stage 6
            v6 <= v5;

            if (v5 && !freeze5) begin
                world_x_acc_q14 <= world_x_acc_q14 +
                    {{16{world_dx_q14_5[31]}},
                        world_dx_q14_5};

                world_y_acc_q14 <= world_y_acc_q14 +
                    {{16{world_dy_q14_5[31]}},
                        world_dy_q14_5};
            end

            if (v6) begin
                world_x_mm <=
                    $signed(world_x_acc_q14[45:14]);
                world_y_mm <=
                    $signed(world_y_acc_q14[45:14]);
            end
        end
    end
endmodule
