// Position servo PWM. Pulse widths are captured only at a period boundary.
// Disabled/reset output is low. Re-enabling waits for the next full period.
module gf_servo_pwm #(
    parameter integer PERIOD_US=20000, MIN_US=1100, MAX_US=1900, MID_US=1500
)(
    input wire I_clk,I_rst_n,I_tick_us,I_enable,
    input wire [15:0] I_pulse_us,
    output wire O_pwm
);
    localparam integer PHASE_W=PERIOD_US<2?1:$clog2(PERIOD_US);
    reg [PHASE_W-1:0] phase;
    reg active;
    reg [15:0] pulse_us;
    wire [15:0] bounded=I_pulse_us<MIN_US?MIN_US:
                        (I_pulse_us>MAX_US?MAX_US:I_pulse_us);
    assign O_pwm=I_rst_n && I_enable && active && phase<pulse_us;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin phase<=0;active<=0;pulse_us<=MID_US;end
        else begin
            if(!I_enable)active<=0;
            if(I_tick_us)begin
                if(phase==PERIOD_US-1)begin
                    phase<=0;pulse_us<=bounded;active<=I_enable;
                end else phase<=phase+1'b1;
            end
        end
    end
`ifndef SYNTHESIS
    initial if(MIN_US<1 || MIN_US>MID_US || MID_US>MAX_US ||
        MAX_US>=PERIOD_US || MAX_US>65535) $fatal(1,"gf_servo_pwm invalid parameters");
`endif
endmodule
