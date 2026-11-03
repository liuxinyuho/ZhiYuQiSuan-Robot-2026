`timescale 1ns/1ps

//////////////////////////////////////////////////////////////////////////////////
// File: tof_channel_recovery.v
// ACG720 Four Independent ToF Recovery System
//
// Clock              : 50MHz
// Top-level fast profile: data timeout 300ms, driver reset 10ms,
// retry cooldown 150..300ms, error confirmation 2ms,
// recovery confirmation 2 fresh distance samples.
//
// IMPORTANT:
// This module resets one VL53L0X FPGA driver only.
// The I2C master performs a nine-clock bus-clear sequence after reset.
// It does not physically power-cycle the sensor.
//////////////////////////////////////////////////////////////////////////////////

module tof_channel_recovery #(
    parameter integer CLK_HZ                = 50_000_000,
    parameter integer DATA_TIMEOUT_MS       = 300,
    parameter integer RESET_HOLD_MS         = 10,
    parameter integer STARTUP_TIMEOUT_MS    = 15_000,
    parameter integer RETRY_COOLDOWN_MS     = 200,
    parameter integer ERROR_CONFIRM_MS      = 2,
    parameter integer VALID_CONFIRM_SAMPLES = 2,
    parameter integer INITIAL_DELAY_MS      = 0,
    parameter [6:0] RUN_STATE_MIN            = 7'd36,
    parameter integer STOP_STABLE_MS        = 20
)(
    input wire clk,
    input wire reset_n,

    input wire sensor_error,
    input wire distance_valid,
    input wire [6:0] debug_state,
    input wire reset_permitted,

    output wire driver_reset_n,
    output reg healthy,
    output wire recovering,
    output reg fault_latched,
    output reg [7:0] recovery_count
);

    localparam integer MS_DIV = CLK_HZ / 1000;

    localparam [2:0]
        ST_INIT_DELAY = 3'd0,
        ST_STARTUP    = 3'd1,
        ST_HEALTHY    = 3'd2,
        ST_WAIT_SAFE  = 3'd3,
        ST_RESET      = 3'd4;

    reg [2:0] state;

    reg [31:0] ms_div_cnt;
    wire ms_tick = (ms_div_cnt >= MS_DIV - 1);

    reg [31:0] init_delay_ms;
    reg [31:0] startup_elapsed_ms;
    reg [31:0] data_age_ms;
    reg [31:0] cooldown_ms;
    reg [31:0] reset_elapsed_ms;
    reg [31:0] stopped_elapsed_ms;

    reg [7:0] error_confirm_ms;
    reg [7:0] valid_sample_count;
    reg distance_valid_pending;

    reg recovery_in_progress;

    // Capture one-clock distance_valid pulses.
    wire distance_sample_event =
        distance_valid || distance_valid_pending;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            distance_valid_pending <= 1'b0;
        else if (ms_tick)
            distance_valid_pending <= 1'b0;
        else if (distance_valid)
            distance_valid_pending <= 1'b1;
    end

    assign driver_reset_n =
        reset_n &&
        (state != ST_INIT_DELAY) &&
        (state != ST_RESET);

    assign recovering = (state != ST_HEALTHY);

    // 1ms time base
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            ms_div_cnt <= 32'd0;
        else if (ms_tick)
            ms_div_cnt <= 32'd0;
        else
            ms_div_cnt <= ms_div_cnt + 1'b1;
    end

    // Independent recovery FSM
    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin
            state                 <= ST_INIT_DELAY;

            init_delay_ms         <= 32'd0;
            startup_elapsed_ms    <= 32'd0;
            data_age_ms           <= 32'd0;
            cooldown_ms           <= 32'd0;
            reset_elapsed_ms      <= 32'd0;
            stopped_elapsed_ms    <= 32'd0;

            error_confirm_ms      <= 8'd0;
            valid_sample_count    <= 8'd0;

            recovery_in_progress  <= 1'b0;

            healthy               <= 1'b0;
            fault_latched         <= 1'b0;
            recovery_count        <= 8'd0;
        end

        else if (ms_tick) begin

            case (state)

                //====================================
                // Initial staggered startup
                //====================================

                ST_INIT_DELAY: begin

                    healthy <= 1'b0;

                    valid_sample_count <= 8'd0;

                    if ((INITIAL_DELAY_MS == 0) ||
                        (init_delay_ms >= INITIAL_DELAY_MS - 1)) begin

                        init_delay_ms <= 32'd0;
                        startup_elapsed_ms <= 32'd0;
                        data_age_ms <= 32'd0;
                        error_confirm_ms <= 8'd0;

                        state <= ST_STARTUP;
                    end

                    else begin
                        init_delay_ms <= init_delay_ms + 1'b1;
                    end

                end


                //====================================
                // Wait for complete VL53L0X init
                //====================================

                ST_STARTUP: begin

                    healthy <= 1'b0;

                    if (sensor_error) begin

                        if (error_confirm_ms >= ERROR_CONFIRM_MS - 1) begin

                            error_confirm_ms <= 8'd0;
                            fault_latched <= 1'b1;

                            cooldown_ms <= RETRY_COOLDOWN_MS;
                            stopped_elapsed_ms <= 32'd0;
                            valid_sample_count <= 8'd0;

                            startup_elapsed_ms <= 32'd0;

                            state <= ST_WAIT_SAFE;
                        end

                        else begin
                            error_confirm_ms <= error_confirm_ms + 1'b1;
                        end

                    end

                    else begin

                        error_confirm_ms <= 8'd0;

                        if ((debug_state >= RUN_STATE_MIN) &&
                            distance_sample_event) begin

                            if (valid_sample_count >=
                                VALID_CONFIRM_SAMPLES - 1) begin

                                valid_sample_count <= 8'd0;

                                startup_elapsed_ms <= 32'd0;
                                data_age_ms <= 32'd0;

                                healthy <= 1'b1;
                                fault_latched <= 1'b0;

                                state <= ST_HEALTHY;

                                if (recovery_in_progress) begin
                                    if (recovery_count != 8'hFF)
                                        recovery_count <= recovery_count + 1'b1;
                                end

                                recovery_in_progress <= 1'b0;
                            end

                            else begin
                                valid_sample_count <= valid_sample_count + 1'b1;
                                startup_elapsed_ms <= startup_elapsed_ms + 1'b1;
                            end
                        end

                        else if (startup_elapsed_ms >=
                                 STARTUP_TIMEOUT_MS - 1) begin

                            fault_latched <= 1'b1;
                            cooldown_ms <= RETRY_COOLDOWN_MS;
                            stopped_elapsed_ms <= 32'd0;

                            valid_sample_count <= 8'd0;
                            startup_elapsed_ms <= 32'd0;

                            state <= ST_WAIT_SAFE;
                        end

                        else begin
                            startup_elapsed_ms <= startup_elapsed_ms + 1'b1;
                        end

                    end

                end


                //====================================
                // Normal monitoring
                //====================================

                ST_HEALTHY: begin

                    healthy <= 1'b1;

                    if (sensor_error) begin

                        if (error_confirm_ms >= ERROR_CONFIRM_MS - 1) begin

                            healthy <= 1'b0;
                            fault_latched <= 1'b1;

                            error_confirm_ms <= 8'd0;
                            cooldown_ms <= RETRY_COOLDOWN_MS;

                            stopped_elapsed_ms <= 32'd0;
                            data_age_ms <= 32'd0;

                            state <= ST_WAIT_SAFE;
                        end

                        else begin
                            error_confirm_ms <= error_confirm_ms + 1'b1;
                        end

                    end

                    else begin

                        error_confirm_ms <= 8'd0;

                        if (distance_sample_event) begin
                            data_age_ms <= 32'd0;
                        end

                        else if (data_age_ms >= DATA_TIMEOUT_MS - 1) begin

                            healthy <= 1'b0;
                            fault_latched <= 1'b1;

                            cooldown_ms <= RETRY_COOLDOWN_MS;
                            stopped_elapsed_ms <= 32'd0;
                            data_age_ms <= 32'd0;

                            state <= ST_WAIT_SAFE;
                        end

                        else begin
                            data_age_ms <= data_age_ms + 1'b1;
                        end

                    end

                end


                //====================================
                // Wait until the top-level reset permission is stable
                //====================================

                ST_WAIT_SAFE: begin

                    healthy <= 1'b0;

                    if (cooldown_ms != 0)
                        cooldown_ms <= cooldown_ms - 1'b1;

                    if (reset_permitted) begin

                        if (stopped_elapsed_ms < STOP_STABLE_MS)
                            stopped_elapsed_ms <= stopped_elapsed_ms + 1'b1;

                    end

                    else begin
                        stopped_elapsed_ms <= 32'd0;
                    end

                    if ((cooldown_ms == 0) &&
                        (stopped_elapsed_ms >= STOP_STABLE_MS - 1)) begin

                        reset_elapsed_ms <= 32'd0;
                        valid_sample_count <= 8'd0;
                        startup_elapsed_ms <= 32'd0;
                        error_confirm_ms <= 8'd0;

                        recovery_in_progress <= 1'b1;

                        state <= ST_RESET;
                    end

                end


                //====================================
                // Reset this channel only
                //====================================

                ST_RESET: begin

                    healthy <= 1'b0;

                    if (reset_elapsed_ms >= RESET_HOLD_MS - 1) begin

                        reset_elapsed_ms <= 32'd0;
                        startup_elapsed_ms <= 32'd0;
                        data_age_ms <= 32'd0;
                        valid_sample_count <= 8'd0;
                        error_confirm_ms <= 8'd0;

                        state <= ST_STARTUP;
                    end

                    else begin
                        reset_elapsed_ms <= reset_elapsed_ms + 1'b1;
                    end

                end


                //====================================
                // Invalid state protection
                //====================================

                default: begin

                    healthy <= 1'b0;
                    fault_latched <= 1'b1;

                    cooldown_ms <= RETRY_COOLDOWN_MS;
                    stopped_elapsed_ms <= 32'd0;

                    state <= ST_WAIT_SAFE;
                end

            endcase

        end

    end

endmodule