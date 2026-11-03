
`timescale 1ns / 1ps

// A5 / A6 physical button controller
// Clock: 50 MHz
// Buttons: active LOW
// KEY1: A5 / STOP
// KEY2: A6 / STOP

module a5_a6_button_control #(
    parameter integer DEBOUNCE_TICKS = 1000000,
    parameter integer SWITCH_TICKS   = 2500000
)(
    input  wire       clk,
    input  wire       reset_n,

    input  wire       key_a5_n,
    input  wire       key_a6_n,

    input  wire [7:0] uart_data,
    input  wire       uart_done,

    output reg  [7:0] cmd_data,
    output reg        cmd_done,
    output reg  [1:0] selected_mode
);

    wire a5_press;
    wire a6_press;

    a5_a6_key_debounce #(
        .DEBOUNCE_TICKS(DEBOUNCE_TICKS)
    ) u_a5_key (
        .clk(clk),
        .reset_n(reset_n),
        .key_n(key_a5_n),
        .press_pulse(a5_press)
    );

    a5_a6_key_debounce #(
        .DEBOUNCE_TICKS(DEBOUNCE_TICKS)
    ) u_a6_key (
        .clk(clk),
        .reset_n(reset_n),
        .key_n(key_a6_n),
        .press_pulse(a6_press)
    );

    reg pending;
    reg [7:0] pending_cmd;
    reg [21:0] switch_count;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            cmd_data      <= 8'h53;
            cmd_done      <= 1'b0;
            selected_mode <= 2'd0;

            pending       <= 1'b0;
            pending_cmd   <= 8'h53;
            switch_count  <= 22'd0;
        end
        else begin
            cmd_done <= 1'b0;

            // PC UART has highest priority
            if (uart_done) begin
                cmd_data <= uart_data;
                cmd_done <= 1'b1;

                pending <= 1'b0;
                switch_count <= 22'd0;

                case (uart_data)
                    8'h53: selected_mode <= 2'd0;
                    8'hA5: selected_mode <= 2'd1;
                    8'hA6: selected_mode <= 2'd2;
                    default: ;
                endcase
            end

            // Both keys pressed: STOP
            else if (a5_press && a6_press) begin
                cmd_data <= 8'h53;
                cmd_done <= 1'b1;
                selected_mode <= 2'd0;

                pending <= 1'b0;
                switch_count <= 22'd0;
            end

            // KEY1: A5 / STOP
            else if (a5_press) begin
                cmd_data <= 8'h53;
                cmd_done <= 1'b1;

                pending <= (selected_mode != 2'd1);
                pending_cmd <= 8'hA5;
                switch_count <= SWITCH_TICKS;

                if (selected_mode == 2'd1)
                    selected_mode <= 2'd0;
                else
                    selected_mode <= 2'd1;
            end

            // KEY2: A6 / STOP
            else if (a6_press) begin
                cmd_data <= 8'h53;
                cmd_done <= 1'b1;

                pending <= (selected_mode != 2'd2);
                pending_cmd <= 8'hA6;
                switch_count <= SWITCH_TICKS;

                if (selected_mode == 2'd2)
                    selected_mode <= 2'd0;
                else
                    selected_mode <= 2'd2;
            end

            // Start selected mode after STOP
            else if (pending) begin
                if (switch_count == 22'd0) begin
                    cmd_data <= pending_cmd;
                    cmd_done <= 1'b1;
                    pending <= 1'b0;
                end
                else begin
                    switch_count <= switch_count - 1'b1;
                end
            end
        end
    end

endmodule


// ===================================================
// Key debounce
// 20ms debounce at 50MHz
// ===================================================

module a5_a6_key_debounce #(
    parameter integer DEBOUNCE_TICKS = 1000000
)(
    input wire clk,
    input wire reset_n,
    input wire key_n,

    output reg press_pulse
);

    reg key_meta;
    reg key_sync;
    reg key_stable;

    reg [19:0] debounce_count;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
            key_stable <= 1'b1;

            debounce_count <= 20'd0;
            press_pulse <= 1'b0;
        end
        else begin
            key_meta <= key_n;
            key_sync <= key_meta;
            press_pulse <= 1'b0;

            if (key_sync == key_stable) begin
                debounce_count <= 20'd0;
            end
            else if (
                debounce_count == DEBOUNCE_TICKS - 1
            ) begin
                debounce_count <= 20'd0;
                key_stable <= key_sync;

                if (!key_sync)
                    press_pulse <= 1'b1;
            end
            else begin
                debounce_count <= debounce_count + 1'b1;
            end
        end
    end

endmodule
