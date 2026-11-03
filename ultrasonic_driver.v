//////////////////////////////////////////////////////////////////////////////////
// File Name : ultrasonic_driver.v
// Project   : FPGA Mecanum Robot
// Device    : Gowin GW5AT-60K / ACG720
// Clock     : 50 MHz
// Sensor    : HC-SR04
//
// Fast fault detection and recovery:
// 1. Measurement interval >= 60ms.
// 2. Echo timeout = 20ms.
// 3. Two consecutive failures trigger FPGA soft recovery.
// 4. Three consecutive echo responses confirm sensor online.
// 5. Out-of-range results are UNKNOWN, not valid distance.
//
// IMPORTANT:
// HC-SR04 has no reset pin. This is an FPGA-side soft reset.
// True power-cycle requires external power switch.
// ECHO is approximately 5V: use voltage divider before FPGA input.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module ultrasonic_driver #(
    parameter integer CLK_FREQ_HZ            = 50_000_000,
    parameter integer MEASURE_INTERVAL_MS    = 60,
    parameter integer ECHO_TIMEOUT_MS        = 20,
    parameter integer RECOVERY_RESET_MS      = 2,
    parameter integer RECOVERY_FAIL_LIMIT    = 2,
    parameter integer RECOVERY_CONFIRM_COUNT = 3
)(
    input  wire        clk,
    input  wire        reset_n,

    output reg         ultra_trig,
    input  wire        ultra_echo,

    output reg [15:0]  distance_mm,
    output reg         distance_valid,
    output reg         measure_busy,
    output reg         echo_timeout,

    // Health / recovery telemetry
    output reg         measurement_done,
    output reg         sensor_online,
    output reg         sensor_recovering,
    output reg [7:0]   recovery_count
);

    //============================================================
    // Timing constants
    //============================================================

    localparam [31:0] TRIG_CYCLES =
        (CLK_FREQ_HZ / 1_000_000) * 10;

    localparam [31:0] MEASURE_INTERVAL_CYCLES =
        (CLK_FREQ_HZ / 1000) * MEASURE_INTERVAL_MS;

    localparam [31:0] ECHO_TIMEOUT_CYCLES =
        (CLK_FREQ_HZ / 1000) * ECHO_TIMEOUT_MS;

    localparam [31:0] RECOVERY_RESET_CYCLES =
        (CLK_FREQ_HZ / 1000) * RECOVERY_RESET_MS;

    // 80mm ~ 3000mm
    localparam [31:0] MIN_VALID_ECHO_TICKS = 32'd23300;
    localparam [31:0] MAX_VALID_ECHO_TICKS = 32'd873800;

    //============================================================
    // State machine
    //============================================================

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_TRIG      = 3'd1,
        ST_WAIT_ECHO = 3'd2,
        ST_MEASURE   = 3'd3,
        ST_RECOVERY  = 3'd4;

    reg [2:0] state;

    reg [31:0] interval_cnt;
    reg [31:0] trig_cnt;
    reg [31:0] wait_cnt;
    reg [31:0] echo_cnt;
    reg [31:0] recovery_cnt;

    reg [1:0] failure_streak;
    reg [1:0] response_streak;

    reg recovery_lockout;

    //============================================================
    // ECHO synchronizer and glitch filter
    //============================================================

    reg echo_sync1;
    reg echo_sync2;

    reg [3:0] echo_filter_history;

    reg echo_stable;
    reg echo_stable_d;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin

            echo_sync1          <= 1'b0;
            echo_sync2          <= 1'b0;
            echo_filter_history <= 4'b0000;

            echo_stable         <= 1'b0;
            echo_stable_d       <= 1'b0;

        end else begin

            echo_sync1 <= ultra_echo;
            echo_sync2 <= echo_sync1;

            if (state == ST_RECOVERY) begin

                echo_filter_history <= 4'b0000;
                echo_stable         <= 1'b0;
                echo_stable_d       <= 1'b0;

            end else begin

                echo_filter_history <=
                    {echo_filter_history[2:0], echo_sync2};

                if (&echo_filter_history)
                    echo_stable <= 1'b1;

                else if (~|echo_filter_history)
                    echo_stable <= 1'b0;

                echo_stable_d <= echo_stable;

            end
        end
    end

    wire echo_rise = echo_stable && !echo_stable_d;
    wire echo_fall = !echo_stable && echo_stable_d;

    //============================================================
    // Distance conversion
    // mm ~= echo_ticks * 225 / 65536
    //============================================================

    wire [47:0] echo_count_ext =
        {16'd0, echo_cnt};

    wire [47:0] distance_scaled =
          (echo_count_ext << 7)
        + (echo_count_ext << 6)
        + (echo_count_ext << 5)
        +  echo_count_ext;

    //============================================================
    // Main state machine
    //============================================================

    always @(posedge clk or negedge reset_n) begin

        if (!reset_n) begin

            state             <= ST_IDLE;

            interval_cnt      <= 32'd0;
            trig_cnt          <= 32'd0;
            wait_cnt          <= 32'd0;
            echo_cnt          <= 32'd0;
            recovery_cnt      <= 32'd0;

            ultra_trig        <= 1'b0;

            distance_mm       <= 16'd0;
            distance_valid    <= 1'b0;

            measure_busy      <= 1'b0;
            echo_timeout      <= 1'b0;

            measurement_done  <= 1'b0;

            sensor_online     <= 1'b0;
            sensor_recovering <= 1'b0;

            recovery_count    <= 8'd0;

            failure_streak    <= 2'd0;
            response_streak   <= 2'd0;

            recovery_lockout  <= 1'b0;

        end else begin

            // One-clock event pulses
            distance_valid   <= 1'b0;
            measurement_done <= 1'b0;

            // Trigger interval protection
            if (state == ST_IDLE) begin

                if (interval_cnt < MEASURE_INTERVAL_CYCLES - 1)
                    interval_cnt <= interval_cnt + 1'b1;

            end else begin

                if (interval_cnt < MEASURE_INTERVAL_CYCLES)
                    interval_cnt <= interval_cnt + 1'b1;

            end

            case (state)

                //================================================
                // IDLE
                //================================================

                ST_IDLE: begin

                    ultra_trig   <= 1'b0;
                    measure_busy <= 1'b0;

                    trig_cnt     <= 32'd0;
                    wait_cnt     <= 32'd0;
                    echo_cnt     <= 32'd0;
                    recovery_cnt <= 32'd0;

                    if (interval_cnt >=
                        MEASURE_INTERVAL_CYCLES - 1) begin

                        interval_cnt <= 32'd0;
                        state        <= ST_TRIG;

                    end
                end

                //================================================
                // Generate 10us trigger pulse
                //================================================

                ST_TRIG: begin

                    measure_busy <= 1'b1;

                    if (trig_cnt >= TRIG_CYCLES - 1) begin

                        ultra_trig <= 1'b0;

                        trig_cnt <= 32'd0;
                        wait_cnt <= 32'd0;
                        echo_cnt <= 32'd0;

                        state <= ST_WAIT_ECHO;

                    end else begin

                        ultra_trig <= 1'b1;
                        trig_cnt <= trig_cnt + 1'b1;

                    end
                end

                //================================================
                // Wait for ECHO
                //================================================

                ST_WAIT_ECHO: begin

                    ultra_trig   <= 1'b0;
                    measure_busy <= 1'b1;

                    if (echo_rise) begin

                        echo_cnt <= 32'd0;
                        wait_cnt <= 32'd0;

                        state <= ST_MEASURE;

                    end else if (
                        echo_stable ||
                        (wait_cnt >= ECHO_TIMEOUT_CYCLES - 1)
                    ) begin

                        measurement_done <= 1'b1;

                        distance_valid <= 1'b0;
                        echo_timeout   <= 1'b1;

                        measure_busy <= 1'b0;
                        sensor_online <= 1'b0;

                        response_streak <= 2'd0;
                        wait_cnt <= 32'd0;

                        if (!recovery_lockout) begin

                            if (failure_streak >=
                                RECOVERY_FAIL_LIMIT - 1) begin

                                failure_streak <= 2'd0;

                                sensor_recovering <= 1'b1;
                                recovery_lockout  <= 1'b1;

                                if (recovery_count != 8'hFF)
                                    recovery_count <= recovery_count + 1'b1;

                                recovery_cnt <= 32'd0;
                                state <= ST_RECOVERY;

                            end else begin

                                failure_streak <= failure_streak + 1'b1;
                                state <= ST_IDLE;

                            end

                        end else begin

                            failure_streak <= 2'd0;
                            state <= ST_IDLE;

                        end

                    end else begin

                        wait_cnt <= wait_cnt + 1'b1;

                    end
                end

                //================================================
                // Measure ECHO pulse width
                //================================================

                ST_MEASURE: begin

                    ultra_trig   <= 1'b0;
                    measure_busy <= 1'b1;

                    if (echo_fall) begin

                        if (distance_scaled[47:16] > 32'd65535)
                            distance_mm <= 16'hFFFF;
                        else
                            distance_mm <= distance_scaled[31:16];

                        measurement_done <= 1'b1;

                        distance_valid <=
                            (echo_cnt >= MIN_VALID_ECHO_TICKS) &&
                            (echo_cnt <= MAX_VALID_ECHO_TICKS);

                        if (
                            (echo_cnt >= MIN_VALID_ECHO_TICKS) &&
                            (echo_cnt <= MAX_VALID_ECHO_TICKS)
                        )
                            echo_timeout <= 1'b0;
                        else
                            echo_timeout <= 1'b1;

                        measure_busy <= 1'b0;

                        echo_cnt <= 32'd0;

                        failure_streak <= 2'd0;
                        recovery_lockout <= 1'b0;

                        if (response_streak < RECOVERY_CONFIRM_COUNT)
                            response_streak <= response_streak + 1'b1;

                        if (response_streak >=
                            RECOVERY_CONFIRM_COUNT - 1) begin

                            sensor_online <= 1'b1;
                            sensor_recovering <= 1'b0;

                        end

                        state <= ST_IDLE;

                    end else if (
                        echo_cnt >= ECHO_TIMEOUT_CYCLES - 1
                    ) begin

                        measurement_done <= 1'b1;

                        distance_valid <= 1'b0;
                        echo_timeout <= 1'b1;

                        measure_busy <= 1'b0;

                        echo_cnt <= 32'd0;

                        sensor_online <= 1'b0;
                        response_streak <= 2'd0;

                        if (!recovery_lockout) begin

                            if (failure_streak >=
                                RECOVERY_FAIL_LIMIT - 1) begin

                                failure_streak <= 2'd0;

                                sensor_recovering <= 1'b1;
                                recovery_lockout <= 1'b1;

                                if (recovery_count != 8'hFF)
                                    recovery_count <= recovery_count + 1'b1;

                                recovery_cnt <= 32'd0;
                                state <= ST_RECOVERY;

                            end else begin

                                failure_streak <= failure_streak + 1'b1;
                                state <= ST_IDLE;

                            end

                        end else begin

                            failure_streak <= 2'd0;
                            state <= ST_IDLE;

                        end

                    end else begin

                        echo_cnt <= echo_cnt + 1'b1;

                    end
                end

                //================================================
                // FPGA-side soft recovery
                //================================================

                ST_RECOVERY: begin

                    ultra_trig   <= 1'b0;
                    measure_busy <= 1'b0;

                    trig_cnt <= 32'd0;
                    wait_cnt <= 32'd0;
                    echo_cnt <= 32'd0;

                    if (recovery_cnt >= RECOVERY_RESET_CYCLES - 1) begin

                        recovery_cnt <= 32'd0;
                        state <= ST_IDLE;

                    end else begin

                        recovery_cnt <= recovery_cnt + 1'b1;

                    end
                end

                //================================================
                // Illegal state recovery
                //================================================

                default: begin

                    state <= ST_IDLE;

                    ultra_trig <= 1'b0;
                    measure_busy <= 1'b0;

                    trig_cnt <= 32'd0;
                    wait_cnt <= 32'd0;
                    echo_cnt <= 32'd0;
                    recovery_cnt <= 32'd0;

                    sensor_online <= 1'b0;
                    sensor_recovering <= 1'b1;

                    response_streak <= 2'd0;
                    failure_streak <= 2'd0;

                    recovery_lockout <= 1'b1;
                    echo_timeout <= 1'b1;

                end

            endcase
        end
    end

endmodule