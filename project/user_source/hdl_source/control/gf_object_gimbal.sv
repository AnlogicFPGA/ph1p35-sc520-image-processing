// Adapt the existing OBJECT stabilizer to the algorithm-independent controller.
// Coordinates, metadata and control all stay in the HDMI pixel clock domain.
// Timestamp is carried in the actual video prefix. It detects repeated display
// frames, but is not proof of a DDR whole-frame commit or absence of tearing.
module gf_object_gimbal #(
    parameter integer ANTI_HUNT=0, SETTLE_MS=80, BOTH_AXES=0,
    parameter integer CLK_HZ=75000000, WIDTH=1280, HEIGHT=720,
    parameter integer TARGET_CLASS=3, // 0 accepts the frontend's selected class.
    parameter integer REQUIRE_EXTERNAL_ARM=1, // Standalone option; project uses mode only.
    parameter integer REF_X=WIDTH/2, REF_Y=HEIGHT/2,
    parameter integer PAN_REVERSE=0,TILT_REVERSE=1,
    parameter integer PAN_MIN_US=1400,PAN_MAX_US=1600,
    parameter integer TILT_MIN_US=1400,TILT_MAX_US=1600,
    parameter integer MAX_STEP_US=16, CONTROL_INTERVAL_US=20000,TIMEOUT_US=250000
)(
    input wire I_clk,I_rst_n,I_mode_active,I_arm_async,
    input wire I_frame_start,I_pixel_valid,
    input wire [11:0] I_x,I_y,
    input wire I_source_valid,
    input wire [31:0] I_source_ms,I_now_ms,
    // Pulse after the stabilizers and owner selector have committed this result.
    input wire I_publish,I_target_valid,I_target_fresh,
    input wire [2:0] I_class_id,
    input wire [11:0] I_xmin,I_xmax,I_ymin,I_ymax,I_center_x,I_center_y,
    output wire O_pan_pwm,O_tilt_pwm,O_locked,
    output wire [15:0] O_pan_us,O_tilt_us,
    output wire [1:0] O_state,
    output wire [3:0] O_reason,
    output wire O_diag_update,
    output reg [4:0] O_diag_flags,
    output reg [2:0] O_diag_gate
);
    localparam integer PIXELS=WIDTH*HEIGHT, CW=$clog2(PIXELS+1);
    localparam [11:0] REF_X_VALUE=REF_X, REF_Y_VALUE=REF_Y;
    wire drive_permit;
    generate if(REQUIRE_EXTERNAL_ARM)begin:g_external_arm
        (* preserve=1,keep=1,ASYNC_REG="TRUE" *) reg arm_sync1,arm_sync2;
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin arm_sync1<=0;arm_sync2<=0;end
            else begin arm_sync1<=I_arm_async;arm_sync2<=arm_sync1;end
        end
        assign drive_permit=arm_sync2;
    end else begin:g_mode_only
        assign drive_permit=1'b1;
    end endgenerate
    reg frame_seen,frame_bad,ready_complete,ready_source_valid;
    reg [CW-1:0] pixel_count;
    reg [11:0] expected_x,expected_y;
    reg [31:0] ready_source_ms;
    // Snapshot PREVIOUS frame metadata on the same edge that the detector
    // snapshots its frame statistics. Never associate it with the new prefix.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            frame_seen<=0;frame_bad<=0;pixel_count<=0;expected_x<=0;expected_y<=0;
            ready_complete<=0;ready_source_valid<=0;ready_source_ms<=0;
        end else if(I_frame_start) begin
            ready_complete<=frame_seen && !frame_bad && pixel_count==PIXELS;
            ready_source_valid<=I_source_valid;
            ready_source_ms<=I_source_ms;
            frame_seen<=1;frame_bad<=I_pixel_valid && (I_x!=0 || I_y!=0);
            pixel_count<=I_pixel_valid?1:0;
            expected_x<=I_pixel_valid?1:0;expected_y<=0;
        end else if(frame_seen && I_pixel_valid) begin
            if(pixel_count<PIXELS)pixel_count<=pixel_count+1'b1;
            else frame_bad<=1;
            if(I_x!=expected_x || I_y!=expected_y || I_x>=WIDTH || I_y>=HEIGHT)
                frame_bad<=1;
            if(expected_x==WIDTH-1)begin expected_x<=0;expected_y<=expected_y+1'b1;end
            else expected_x<=expected_x+1'b1;
        end
    end
    reg seen_source,obs,restart,obs_valid,obs_fresh;
    reg [31:0] last_source,obs_id;
    reg [15:0] obs_age;
    reg [11:0] xmin,xmax,ymin,ymax,cx,cy;
    wire [31:0] source_delta=ready_source_ms-last_source;
    wire source_new=!seen_source || (source_delta!=0 && !source_delta[31]);
    wire [31:0] age_now=I_now_ms-ready_source_ms;
    wire selected=TARGET_CLASS==0 || I_class_id==TARGET_CLASS;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            seen_source<=0;last_source<=0;obs<=0;restart<=0;
            obs_valid<=0;obs_fresh<=0;obs_id<=0;obs_age<=0;
            xmin<=0;xmax<=0;ymin<=0;ymax<=0;cx<=0;cy<=0;
        end else begin
            obs<=0;restart<=0;
            if(!I_mode_active)begin seen_source<=0;restart<=1;end
            else if(I_publish)begin
                if(!ready_complete || !ready_source_valid || age_now[31])
                    restart<=1;
                else if(source_new)begin
                    seen_source<=1;last_source<=ready_source_ms;
                    obs<=1;obs_id<=ready_source_ms;
                    obs_age<=age_now>65535?16'hffff:age_now[15:0];
                    obs_valid<=I_target_valid && selected;
                    obs_fresh<=I_target_fresh;
                    xmin<=I_xmin;xmax<=I_xmax;ymin<=I_ymin;ymax<=I_ymax;
                    cx<=I_center_x;cy<=I_center_y;
                end
                // Duplicate/old source prefixes do not refresh the watchdog.
            end
        end
    end
    // Read-only diagnostics, captured at the same publish as the decision.
    // Flags: complete frame, valid prefix, new source, selected target, fresh.
    // Gate: 0 pass, 1 frame, 2 prefix, 3 future age, 4 duplicate, 5 mode off.
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin O_diag_flags<=0;O_diag_gate<=5;end
        else if(!I_mode_active)begin O_diag_flags<=0;O_diag_gate<=5;end
        else if(I_publish)begin
            O_diag_flags<={ready_complete,ready_source_valid,source_new,
                           I_target_valid && selected,I_target_fresh};
            if(!ready_complete)O_diag_gate<=1;
            else if(!ready_source_valid)O_diag_gate<=2;
            else if(age_now[31])O_diag_gate<=3;
            else if(!source_new)O_diag_gate<=4;
            else O_diag_gate<=0;
        end
    end
    gimbal_follow_core #(
        .ANTI_HUNT(ANTI_HUNT),.SETTLE_MS(SETTLE_MS),.BOTH_AXES(BOTH_AXES),
        .CLK_HZ(CLK_HZ),.IMAGE_WIDTH(WIDTH),.IMAGE_HEIGHT(HEIGHT),
        .CONTROL_INTERVAL_US(CONTROL_INTERVAL_US),.TIMEOUT_US(TIMEOUT_US),
        .MAX_STEP_US(MAX_STEP_US),
        .PAN_MIN_US(PAN_MIN_US),.PAN_MAX_US(PAN_MAX_US),
        .TILT_MIN_US(TILT_MIN_US),.TILT_MAX_US(TILT_MAX_US),
        .PAN_REVERSE(PAN_REVERSE),.TILT_REVERSE(TILT_REVERSE)
    )u_follow(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_run(I_mode_active),
        // I_run is the frame-committed OBJECT enable. No external ARM is
        // required in the project integration; standalone tests may use one.
        .I_arm(drive_permit),.I_config_valid(drive_permit),.I_stream_restart(restart),
        .I_observation(obs),.I_valid(obs_valid),.I_fresh(obs_fresh),
        .I_frame_id(obs_id),.I_age_ms(obs_age),.I_now_ms(I_now_ms),
        .I_xmin(xmin),.I_xmax(xmax),.I_ymin(ymin),.I_ymax(ymax),
        .I_center_x(cx),.I_center_y(cy),
        .I_reference_x(REF_X_VALUE),.I_reference_y(REF_Y_VALUE),
        .O_pan_pwm(O_pan_pwm),.O_tilt_pwm(O_tilt_pwm),.O_locked(O_locked),
        .O_pan_us(O_pan_us),.O_tilt_us(O_tilt_us),.O_state(O_state),.O_reason(O_reason),
        .O_update(O_diag_update),.O_accepted(),.O_rejected());
endmodule

// Shared iterative converter; captures both pulse commands on the same frame.
module gf_pulse_pair_bcd(input I_clk,I_rst_n,I_start,input [15:0] I_pan,I_tilt,
 output reg [31:0] O_bcd);
 reg busy,axis;reg [3:0] count;reg [11:0] binary_value,tilt_value;
 reg [15:0] work_bcd,pan_bcd;reg [15:0] adjusted;integer n;
 always @*begin
   adjusted=work_bcd;
   for(n=0;n<4;n=n+1)if(work_bcd[n*4+:4]>=5)adjusted[n*4+:4]=work_bcd[n*4+:4]+3;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin busy<=0;axis<=0;count<=0;binary_value<=0;tilt_value<=0;work_bcd<=0;pan_bcd<=0;O_bcd<=32'h15001500;end
   else if(I_start)begin busy<=1;axis<=0;count<=0;binary_value<=I_pan[11:0];tilt_value<=I_tilt[11:0];work_bcd<=0;end
   else if(busy)begin
     work_bcd<={adjusted[14:0],binary_value[11]};binary_value<={binary_value[10:0],1'b0};
     if(count==11)begin
       count<=0;work_bcd<=0;
       if(!axis)begin pan_bcd<={adjusted[14:0],binary_value[11]};binary_value<=tilt_value;axis<=1;end
       else begin O_bcd<={pan_bcd,adjusted[14:0],binary_value[11]};busy<=0;end
     end else count<=count+1'b1;
   end
 end
endmodule
