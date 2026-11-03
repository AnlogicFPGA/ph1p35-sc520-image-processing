// Observation-to-position-servo controller. All inputs are synchronous to I_clk.
// Only new, fresh, geometrically valid observations can change a setpoint.
// I_enable=0 / stream_restart cancels acquisition but preserves the last command.
module gf_follow_controller #(
    parameter integer ANTI_HUNT=0, SETTLE_MS=80, BOTH_AXES=0,
    parameter integer PAN_STOP_PX=12,PAN_START_PX=20,TILT_STOP_PX=8,TILT_START_PX=16,
    parameter integer IMAGE_WIDTH=1280, IMAGE_HEIGHT=720,
    parameter integer CONFIRM_COUNT=3, MAX_JUMP=160,
    parameter integer MAX_BOX_WIDTH=IMAGE_WIDTH, MAX_BOX_HEIGHT=IMAGE_HEIGHT,
    parameter integer DEAD_X=12, DEAD_Y=10, GAIN_SHIFT=4,
    parameter integer MAX_STEP_US=16, CONTROL_INTERVAL_US=20000,
    parameter integer TIMEOUT_US=250000, MAX_INPUT_AGE_MS=100,
    parameter integer PAN_MIN_US=1100, PAN_MID_US=1500, PAN_MAX_US=1900,
    parameter integer TILT_MIN_US=1100, TILT_MID_US=1500, TILT_MAX_US=1900,
    parameter integer PAN_REVERSE=0, TILT_REVERSE=1
)(
    input wire I_clk,I_rst_n,I_tick_us,I_enable,I_stream_restart,
    input wire I_observation,I_valid,I_fresh,
    input wire [31:0] I_frame_id,I_now_ms,
    input wire [15:0] I_age_ms,
    input wire [11:0] I_xmin,I_xmax,I_ymin,I_ymax,I_center_x,I_center_y,
    input wire [11:0] I_reference_x,I_reference_y,
    output reg [15:0] O_pan_us,O_tilt_us,
    output reg O_locked,O_update,
    output reg [1:0] O_state,
    output reg [3:0] O_reason,
    output reg [31:0] O_accepted,O_rejected
);
    // State: 0 disabled, 1 acquiring, 2 tracking, 3 lost.
    // Reason: 0 OK, 1 invalid, 2 not fresh/too old, 3 geometry,
    //         4 jump, 5 watchdog, 6 invalid reference.
    localparam integer AGE_W=TIMEOUT_US<1?1:$clog2(TIMEOUT_US+1);
    localparam integer CONTROL_W=CONTROL_INTERVAL_US<2?1:$clog2(CONTROL_INTERVAL_US);
    localparam integer STREAK_W=CONFIRM_COUNT<1?1:$clog2(CONFIRM_COUNT+1);
    reg seen_id,pending;
    reg pan_pending,pan_active,pan_dir_valid,pan_dir,reverse_seen,pan_wait;
    reg [8:0] pan_source_after;
    wire signed [12:0] observed_ex=$signed({1'b0,I_center_x})-$signed({1'b0,I_reference_x});
    wire [12:0] observed_abs=observed_ex<0?-observed_ex:observed_ex;
    wire observed_dir=observed_ex<0;
    wire [8:0] source_since_wait=I_frame_id[8:0]-pan_source_after;
    wire [8:0] pan_now_since_wait=I_now_ms[8:0]-pan_source_after;
    wire settled_source=!pan_wait || !source_since_wait[8];
    wire reversing=pan_dir_valid && observed_dir!=pan_dir;

    reg tilt_pending,tilt_active,tilt_dir_valid,tilt_dir,tilt_reverse_seen,tilt_wait;
    reg [8:0] tilt_source_after;
    wire signed [12:0] tilt_observed_ey=$signed({1'b0,I_center_y})-$signed({1'b0,I_reference_y});
    wire [12:0] tilt_observed_abs=tilt_observed_ey<0?-tilt_observed_ey:tilt_observed_ey;
    wire tilt_observed_dir=tilt_observed_ey<0;
    wire [8:0] tilt_since_wait=I_frame_id[8:0]-tilt_source_after;
    wire [8:0] tilt_now_since_wait=I_now_ms[8:0]-tilt_source_after;
    wire tilt_settled_source=!tilt_wait || !tilt_since_wait[8];
    wire tilt_reversing=tilt_dir_valid && tilt_observed_dir!=tilt_dir;

    reg [31:0] last_id;
    reg [AGE_W-1:0] age_us;
    reg [CONTROL_W-1:0] control_us;
    reg [STREAK_W-1:0] streak;
    reg [11:0] last_x,last_y,target_x,target_y;
    wire [31:0] delta_id=I_frame_id-last_id;
    // Modulo sequence order: forward gaps must be less than 2^31.
    wire new_id=!seen_id || (delta_id!=0 && !delta_id[31]);
    wire consume=I_observation && new_id;
    wire [12:0] box_w={1'b0,I_xmax}-{1'b0,I_xmin}+13'd1;
    wire [12:0] box_h={1'b0,I_ymax}-{1'b0,I_ymin}+13'd1;
    wire geometry=I_xmin<=I_xmax && I_ymin<=I_ymax &&
                  I_xmax<IMAGE_WIDTH && I_ymax<IMAGE_HEIGHT &&
                  box_w>=2 && box_h>=2 && box_w<=MAX_BOX_WIDTH && box_h<=MAX_BOX_HEIGHT &&
                  I_center_x>=I_xmin && I_center_x<=I_xmax &&
                  I_center_y>=I_ymin && I_center_y<=I_ymax;
    wire reference_ok=I_reference_x<IMAGE_WIDTH && I_reference_y<IMAGE_HEIGHT;
    wire [12:0] dx=I_center_x>last_x?I_center_x-last_x:last_x-I_center_x;
    wire [12:0] dy=I_center_y>last_y?I_center_y-last_y:last_y-I_center_y;
    wire expired=age_us>=TIMEOUT_US;
    wire jump_ok=streak==0 || expired || (dx<=MAX_JUMP && dy<=MAX_JUMP);
    wire good=I_valid && I_fresh && I_age_ms<=MAX_INPUT_AGE_MS && geometry &&
              reference_ok && jump_ok;
    wire rejected_now=consume && !good;
    wire deadline=I_tick_us && age_us==TIMEOUT_US-1;
    // Twelve-bit coordinates differ by at most 4095. Eighteen signed bits
    // also cover a 16-bit pulse command plus/minus that entire difference.
    reg signed [12:0] ex,ey;
    reg [12:0] abs_x,abs_y;
    reg signed [17:0] step_x,step_y,next_pan,next_tilt;
    wire control_commit=I_tick_us && control_us==CONTROL_INTERVAL_US-1 && pending &&
        O_locked && !expired && !deadline && !rejected_now && reference_ok;
    wire pan_motion_allowed=pan_pending && (!consume ||
        (observed_abs>PAN_STOP_PX && observed_dir==(target_x<I_reference_x)));
    wire pan_commit=control_commit && pan_motion_allowed && next_pan!=O_pan_us;
    wire tilt_motion_allowed=tilt_pending && (!consume ||
        (tilt_observed_abs>TILT_STOP_PX && tilt_observed_dir==(target_y<I_reference_y)));
    wire tilt_commit=control_commit && tilt_motion_allowed && next_tilt!=O_tilt_us;
    always @* begin
        ex={1'b0,target_x}-{1'b0,I_reference_x};
        ey={1'b0,target_y}-{1'b0,I_reference_y};
        abs_x=ex<0?-ex:ex;abs_y=ey<0?-ey:ey;
        step_x=abs_x>>(ANTI_HUNT?6:GAIN_SHIFT);step_y=abs_y>>((ANTI_HUNT && BOTH_AXES)?6:GAIN_SHIFT);
        if(ANTI_HUNT && BOTH_AXES && abs_y<=96 && step_y>2)step_y=2;
        if(ANTI_HUNT && abs_x<=128 && step_x>2)step_x=2;
        if(step_x<1)step_x=1;if(step_y<1)step_y=1;
        if(step_x>MAX_STEP_US)step_x=MAX_STEP_US;
        if(step_y>MAX_STEP_US)step_y=MAX_STEP_US;
        if(abs_x<=(ANTI_HUNT?PAN_STOP_PX:DEAD_X))step_x=0;if(abs_y<=((ANTI_HUNT && BOTH_AXES)?TILT_STOP_PX:DEAD_Y))step_y=0;
        if(ex<0)step_x=-step_x;if(ey<0)step_y=-step_y;
        if(PAN_REVERSE)step_x=-step_x;if(TILT_REVERSE)step_y=-step_y;
        next_pan=O_pan_us+step_x;next_tilt=O_tilt_us+step_y;
        if(next_pan<PAN_MIN_US)next_pan=PAN_MIN_US;
        if(next_pan>PAN_MAX_US)next_pan=PAN_MAX_US;
        if(next_tilt<TILT_MIN_US)next_tilt=TILT_MIN_US;
        if(next_tilt>TILT_MAX_US)next_tilt=TILT_MAX_US;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            pan_pending<=0;pan_active<=0;pan_dir_valid<=0;pan_dir<=0;reverse_seen<=0;pan_wait<=0;pan_source_after<=0;
            tilt_pending<=0;tilt_active<=0;tilt_dir_valid<=0;tilt_dir<=0;tilt_reverse_seen<=0;tilt_wait<=0;tilt_source_after<=0;
            O_pan_us<=PAN_MID_US;O_tilt_us<=TILT_MID_US;
            O_locked<=0;O_update<=0;O_state<=0;O_reason<=0;
            O_accepted<=0;O_rejected<=0;seen_id<=0;pending<=0;
            last_id<=0;age_us<=TIMEOUT_US;control_us<=0;streak<=0;
            last_x<=0;last_y<=0;target_x<=0;target_y<=0;
        end else begin
            O_update<=0;
            // All <=MAX_INPUT_AGE_MS-old valid sources are now past cutoff.
            // Clear before modular low-bit timestamp wrap can alias old time.
            if(pan_wait && !pan_now_since_wait[8] && pan_now_since_wait>=MAX_INPUT_AGE_MS)pan_wait<=0;
            if(tilt_wait && !tilt_now_since_wait[8] && tilt_now_since_wait>=MAX_INPUT_AGE_MS)tilt_wait<=0;
            if(!I_enable || I_stream_restart) begin
                pan_pending<=0;pan_active<=0;pan_dir_valid<=0;reverse_seen<=0;pan_wait<=0;
                tilt_pending<=0;tilt_active<=0;tilt_dir_valid<=0;tilt_reverse_seen<=0;tilt_wait<=0;
                seen_id<=0;pending<=0;streak<=0;age_us<=TIMEOUT_US;control_us<=0;
                O_locked<=0;O_state<=I_enable?1:0;O_reason<=0;
            end else begin
                if(O_state==0)O_state<=1;
                if(I_tick_us) begin
                    if(age_us<TIMEOUT_US)age_us<=age_us+1'b1;
                    if(control_us==CONTROL_INTERVAL_US-1)begin
                        control_us<=0;
                        // One integration step per consumed measurement at most.
                        if(pending && O_locked && !expired && !deadline &&
                           !rejected_now && reference_ok)begin
                            if(!ANTI_HUNT || pan_motion_allowed)O_pan_us<=next_pan;
                            if(!ANTI_HUNT || !BOTH_AXES || tilt_motion_allowed)O_tilt_us<=next_tilt;
                            pan_pending<=0;
                            if(ANTI_HUNT && pan_motion_allowed && next_pan!=O_pan_us)begin
                                pan_dir_valid<=1;pan_dir<=ex<0;
                                pan_source_after<=I_now_ms[8:0]+SETTLE_MS;
                                pan_wait<=1;
                            end
                            tilt_pending<=0;
                            if(ANTI_HUNT && BOTH_AXES && tilt_motion_allowed && next_tilt!=O_tilt_us)begin
                                tilt_dir_valid<=1;tilt_dir<=ey<0;
                                tilt_source_after<=I_now_ms[8:0]+SETTLE_MS;
                                tilt_wait<=1;
                            end
                            O_update<=1;pending<=0;
                        end
                    end else control_us<=control_us+1'b1;
                    if(deadline)begin
                        pan_pending<=0;pan_active<=0;pan_dir_valid<=0;reverse_seen<=0;pan_wait<=0;
                        tilt_pending<=0;tilt_active<=0;tilt_dir_valid<=0;tilt_reverse_seen<=0;tilt_wait<=0;
                        pending<=0;streak<=0;O_locked<=0;O_state<=3;O_reason<=5;
                    end
                end
                if(!reference_ok)begin
                    pan_pending<=0;pan_active<=0;pan_dir_valid<=0;reverse_seen<=0;pan_wait<=0;
                    tilt_pending<=0;tilt_active<=0;tilt_dir_valid<=0;tilt_reverse_seen<=0;tilt_wait<=0;
                    pending<=0;streak<=0;O_locked<=0;O_state<=3;O_reason<=6;
                end
                if(consume)begin
                    seen_id<=1;last_id<=I_frame_id;
                    if(good)begin
                        if(ANTI_HUNT)begin
                            // A newly arriving frame must never re-arm an action
                            // on the same clock that a preceding command commits.
                            pan_pending<=0;
                            if(observed_abs<=PAN_STOP_PX)begin pan_active<=0;reverse_seen<=0;end
                            else if(pan_active || observed_abs>=PAN_START_PX)begin
                                pan_active<=1;
                                if(!pan_commit && ((observed_abs>128 && !reversing) || settled_source))begin
                                    if(reversing && !reverse_seen)reverse_seen<=1;
                                    else begin pan_pending<=1;reverse_seen<=0;end
                                end else reverse_seen<=0;
                            end else reverse_seen<=0;
                        end
                        if(ANTI_HUNT && BOTH_AXES)begin
                            // A newly arriving frame must never re-arm an action
                            // on the same clock that a preceding command commits.
                            tilt_pending<=0;
                            if(tilt_observed_abs<=TILT_STOP_PX)begin tilt_active<=0;tilt_reverse_seen<=0;end
                            else if(tilt_active || tilt_observed_abs>=TILT_START_PX)begin
                                tilt_active<=1;
                                if(!tilt_commit && ((tilt_observed_abs>96 && !tilt_reversing) || tilt_settled_source))begin
                                    if(tilt_reversing && !tilt_reverse_seen)tilt_reverse_seen<=1;
                                    else begin tilt_pending<=1;tilt_reverse_seen<=0;end
                                end else tilt_reverse_seen<=0;
                            end else tilt_reverse_seen<=0;
                        end
                        O_accepted<=O_accepted+1'b1;age_us<=0;
                        last_x<=I_center_x;last_y<=I_center_y;O_reason<=0;
                        if(expired || deadline || streak==0)begin
                            streak<=1;O_locked<=CONFIRM_COUNT==1;
                            pending<=CONFIRM_COUNT==1;
                            O_state<=CONFIRM_COUNT==1?2:1;
                        end else if(streak<CONFIRM_COUNT)begin
                            streak<=streak+1'b1;
                            if(streak>=CONFIRM_COUNT-1)begin
                                pending<=1;O_locked<=1;O_state<=2;
                            end
                        end else begin pending<=1;O_locked<=1;O_state<=2;end
                        target_x<=I_center_x;target_y<=I_center_y;
                    end else begin
                        pan_pending<=0;pan_active<=0;pan_dir_valid<=0;reverse_seen<=0;pan_wait<=0;
                        tilt_pending<=0;tilt_active<=0;tilt_dir_valid<=0;tilt_reverse_seen<=0;tilt_wait<=0;
                        O_rejected<=O_rejected+1'b1;pending<=0;streak<=0;
                        O_locked<=0;O_state<=3;
                        if(!I_valid)O_reason<=1;
                        else if(!I_fresh || I_age_ms>MAX_INPUT_AGE_MS)O_reason<=2;
                        else if(!geometry)O_reason<=3;
                        else if(!reference_ok)O_reason<=6;
                        else O_reason<=4;
                    end
                end
            end
        end
    end
`ifndef SYNTHESIS
    initial begin
        if(IMAGE_WIDTH>4096 || IMAGE_HEIGHT>4096 || IMAGE_WIDTH<4 || IMAGE_HEIGHT<4 ||
           PAN_STOP_PX<1 || PAN_START_PX<=PAN_STOP_PX || TILT_STOP_PX<1 || TILT_START_PX<=TILT_STOP_PX ||
           CONFIRM_COUNT<1 || CONTROL_INTERVAL_US<1 || TIMEOUT_US<CONTROL_INTERVAL_US ||
           GAIN_SHIFT<0 || GAIN_SHIFT>12 || MAX_STEP_US<1 ||
           PAN_MIN_US<1 || PAN_MIN_US>PAN_MID_US || PAN_MID_US>PAN_MAX_US ||
           TILT_MIN_US<1 || TILT_MIN_US>TILT_MID_US || TILT_MID_US>TILT_MAX_US ||
           PAN_MAX_US>65535 || TILT_MAX_US>65535)
            $fatal(1,"gf_follow_controller invalid parameters");
    end
`endif
endmodule
