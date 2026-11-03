// Algorithm-independent dual-axis follow controller; pin binding is supplied by the top level.
// Config/arm and observations must already be synchronous to I_clk.
// Calibrate pulse limits and polarity for the installed servos before enabling output.
module gimbal_follow_core #(
    parameter integer ANTI_HUNT=0, SETTLE_MS=80, BOTH_AXES=0,
    parameter integer CLK_HZ=75000000,
    parameter integer IMAGE_WIDTH=1280,IMAGE_HEIGHT=720,
    parameter integer CONFIRM_COUNT=3,MAX_JUMP=160,
    parameter integer DEAD_X=12,DEAD_Y=10,GAIN_SHIFT=4,MAX_STEP_US=16,
    parameter integer CONTROL_INTERVAL_US=20000,TIMEOUT_US=250000,MAX_INPUT_AGE_MS=100,
    parameter integer PERIOD_US=20000,
    parameter integer PAN_MIN_US=1100,PAN_MID_US=1500,PAN_MAX_US=1900,
    parameter integer TILT_MIN_US=1100,TILT_MID_US=1500,TILT_MAX_US=1900,
    parameter integer PAN_REVERSE=0,TILT_REVERSE=1
)(
    input wire I_clk,I_rst_n,I_run,I_arm,I_config_valid,I_stream_restart,
    input wire I_observation,I_valid,I_fresh,
    input wire [31:0] I_frame_id,I_now_ms,
    input wire [15:0] I_age_ms,
    input wire [11:0] I_xmin,I_xmax,I_ymin,I_ymax,I_center_x,I_center_y,
    input wire [11:0] I_reference_x,I_reference_y,
    output wire O_pan_pwm,O_tilt_pwm,O_locked,O_update,
    output wire [15:0] O_pan_us,O_tilt_us,
    output wire [1:0] O_state,
    output wire [3:0] O_reason,
    output wire [31:0] O_accepted,O_rejected
);
    localparam integer CYCLES_PER_US=CLK_HZ/1000000;
    localparam integer DIV_W=CYCLES_PER_US<2?1:$clog2(CYCLES_PER_US);
    reg [DIV_W-1:0] us_div;
    wire tick_us=us_div==CYCLES_PER_US-1;
    wire enabled=I_run && I_arm && I_config_valid;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)us_div<=0;
        else if(tick_us)us_div<=0;
        else us_div<=us_div+1'b1;
    end
    gf_follow_controller #(
        .ANTI_HUNT(ANTI_HUNT),.SETTLE_MS(SETTLE_MS),.BOTH_AXES(BOTH_AXES),
        .IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .CONFIRM_COUNT(CONFIRM_COUNT),.MAX_JUMP(MAX_JUMP),
        .DEAD_X(DEAD_X),.DEAD_Y(DEAD_Y),.GAIN_SHIFT(GAIN_SHIFT),
        .MAX_STEP_US(MAX_STEP_US),.CONTROL_INTERVAL_US(CONTROL_INTERVAL_US),
        .TIMEOUT_US(TIMEOUT_US),.MAX_INPUT_AGE_MS(MAX_INPUT_AGE_MS),
        .PAN_MIN_US(PAN_MIN_US),.PAN_MID_US(PAN_MID_US),.PAN_MAX_US(PAN_MAX_US),
        .TILT_MIN_US(TILT_MIN_US),.TILT_MID_US(TILT_MID_US),.TILT_MAX_US(TILT_MAX_US),
        .PAN_REVERSE(PAN_REVERSE),.TILT_REVERSE(TILT_REVERSE)
    )u_controller(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tick_us(tick_us),.I_enable(enabled),
        .I_stream_restart(I_stream_restart),.I_observation(I_observation),
        .I_valid(I_valid),.I_fresh(I_fresh),.I_frame_id(I_frame_id),.I_age_ms(I_age_ms),.I_now_ms(I_now_ms),
        .I_xmin(I_xmin),.I_xmax(I_xmax),.I_ymin(I_ymin),.I_ymax(I_ymax),
        .I_center_x(I_center_x),.I_center_y(I_center_y),
        .I_reference_x(I_reference_x),.I_reference_y(I_reference_y),
        .O_pan_us(O_pan_us),.O_tilt_us(O_tilt_us),.O_locked(O_locked),
        .O_update(O_update),.O_state(O_state),.O_reason(O_reason),
        .O_accepted(O_accepted),.O_rejected(O_rejected));
    gf_servo_pwm #(.PERIOD_US(PERIOD_US),.MIN_US(PAN_MIN_US),.MAX_US(PAN_MAX_US),
        .MID_US(PAN_MID_US))u_pan(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tick_us(tick_us),
        .I_enable(enabled),.I_pulse_us(O_pan_us),.O_pwm(O_pan_pwm));
    gf_servo_pwm #(.PERIOD_US(PERIOD_US),.MIN_US(TILT_MIN_US),.MAX_US(TILT_MAX_US),
        .MID_US(TILT_MID_US))u_tilt(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tick_us(tick_us),
        .I_enable(enabled),.I_pulse_us(O_tilt_us),.O_pwm(O_tilt_pwm));
`ifndef SYNTHESIS
    initial if(CLK_HZ<1000000 || CLK_HZ%1000000!=0)
        $fatal(1,"gimbal_follow_core clock must be an integer MHz");
`endif
endmodule
