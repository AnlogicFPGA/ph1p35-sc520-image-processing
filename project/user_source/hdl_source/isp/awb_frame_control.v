// Four-pixel statistics, one complete frame per estimate. Ratios use one
// iterative divider; no pixel-rate 50-stage divider is used here.
// Modes: 0=AUTO, 1=LOCK, 2=OFF. Gains are unsigned Q8, limited to 0.5..2x.
module awb_frame_control #(
    parameter integer IMG_WIDTH=1280,IMG_HEIGHT=720,
    parameter integer MIN_SAMPLES=128,
    parameter [9:0] MIN_GAIN=128,MAX_GAIN=512
)(
    input wire I_clk,I_rst_n,I_tuser,I_tvalid,
    input wire [95:0] I_tdata,
    input wire [1:0] I_mode_request,
    output wire [9:0] O_gain_r,O_gain_b,
    output reg [1:0] O_mode,
    output reg O_calibrated
);
    localparam integer FRAME_BEATS=IMG_WIDTH*IMG_HEIGHT/4;
    localparam integer BW=(FRAME_BEATS<2)?1:$clog2(FRAME_BEATS);
    reg [BW-1:0] beats;
    reg frame_active;
    reg [31:0] sum_r,sum_g,sum_b;
    reg [19:0] samples;
    reg [10:0] batch_r,batch_g,batch_b;
    reg [2:0] batch_n;
    reg [7:0] pixel_r[0:3],pixel_g[0:3],pixel_b[0:3];
    reg [3:0] pixel_n;
    reg stats_user_1,stats_user_2,stats_valid_1,stats_valid_2;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin:g_statistics
        wire [7:0] red=I_tdata[lane*24+16+:8],green=I_tdata[lane*24+8+:8],blue=I_tdata[lane*24+:8];
        wire [7:0] hi_rg=(red>green)?red:green,lo_rg=(red<green)?red:green;
        wire [7:0] hi=(hi_rg>blue)?hi_rg:blue,lo=(lo_rg<blue)?lo_rg:blue;
        wire eligible=lo>=16 && hi<=240 && (hi-lo)<=(hi>>1);
        // Register eligibility before summation and the frame accumulator.
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin pixel_r[lane]<=0;pixel_g[lane]<=0;pixel_b[lane]<=0;pixel_n[lane]<=0;end
            else begin
                pixel_r[lane]<=eligible?red:0;pixel_g[lane]<=eligible?green:0;pixel_b[lane]<=eligible?blue:0;
                pixel_n[lane]<=eligible;
            end
        end
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            batch_r<=0;batch_g<=0;batch_b<=0;batch_n<=0;
            stats_user_1<=0;stats_user_2<=0;stats_valid_1<=0;stats_valid_2<=0;
        end else begin
            batch_r<=(pixel_r[0]+pixel_r[1])+(pixel_r[2]+pixel_r[3]);
            batch_g<=(pixel_g[0]+pixel_g[1])+(pixel_g[2]+pixel_g[3]);
            batch_b<=(pixel_b[0]+pixel_b[1])+(pixel_b[2]+pixel_b[3]);
            batch_n<=({2'd0,pixel_n[0]}+{2'd0,pixel_n[1]})+({2'd0,pixel_n[2]}+{2'd0,pixel_n[3]});
            stats_user_1<=I_tuser;stats_user_2<=stats_user_1;
            stats_valid_1<=I_tvalid;stats_valid_2<=stats_valid_1;
        end
    end
    wire [31:0] next_r=(stats_user_2?32'd0:sum_r)+batch_r;
    wire [31:0] next_g=(stats_user_2?32'd0:sum_g)+batch_g;
    wire [31:0] next_b=(stats_user_2?32'd0:sum_b)+batch_b;
    wire [19:0] next_n=(stats_user_2?20'd0:samples)+batch_n;
    reg [31:0] snapshot_r,snapshot_g,snapshot_b;
    reg [19:0] snapshot_n;
    reg snapshot_valid;
    reg busy,blue_phase,target_ready;
    reg [39:0] dividend,quotient,saved_numerator;
    reg [31:0] denominator,saved_blue;
    reg [32:0] remainder;
    reg [5:0] bit_index;
    reg [9:0] result_r,target_r,target_b,gain_r,gain_b;
    wire [32:0] shifted={remainder[31:0],dividend[39]};
    wire [33:0] trial_difference={1'b0,shifted}-{2'b00,denominator};
    wire subtract_ok=!trial_difference[33];
    wire [39:0] next_q={quotient[38:0],subtract_ok};
    assign O_gain_r=(O_mode==2)?10'd256:gain_r;
    assign O_gain_b=(O_mode==2)?10'd256:gain_b;
    function [9:0] clamp_gain;
        input [39:0] value;
        begin
            if(value>MAX_GAIN) clamp_gain=MAX_GAIN;
            else if(value<MIN_GAIN) clamp_gain=MIN_GAIN;
            else clamp_gain=value[9:0];
        end
    endfunction
    function [9:0] smooth_gain;
        input [9:0] current_value,target;
        reg [10:0] distance,step;
        begin
            distance=(target>current_value)?target-current_value:current_value-target;
            step=(distance+3)>>2;
            smooth_gain=(target>current_value)?current_value+step:current_value-step;
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            beats<=0;frame_active<=0;sum_r<=0;sum_g<=0;sum_b<=0;samples<=0;
            busy<=0;blue_phase<=0;target_ready<=0;dividend<=0;quotient<=0;
            saved_numerator<=0;denominator<=1;saved_blue<=1;remainder<=0;bit_index<=0;
            result_r<=256;target_r<=256;target_b<=256;gain_r<=256;gain_b<=256;
            O_mode<=0;O_calibrated<=0;
            snapshot_r<=0;snapshot_g<=0;snapshot_b<=0;snapshot_n<=0;snapshot_valid<=0;
        end else begin
            snapshot_valid<=0;
            if(I_tuser) begin
                O_mode<=(I_mode_request==3)?0:I_mode_request;
                if(target_ready) begin
                    target_ready<=0;
                    if(I_mode_request==0) begin
                        gain_r<=smooth_gain(gain_r,target_r);gain_b<=smooth_gain(gain_b,target_b);
                        O_calibrated<=1;
                    end
                end
            end
            if(stats_user_2) begin
                beats<=0;frame_active<=1;sum_r<=0;sum_g<=0;sum_b<=0;samples<=0;
            end
            if(stats_valid_2 && (frame_active || stats_user_2)) begin
                sum_r<=next_r;sum_g<=next_g;sum_b<=next_b;samples<=next_n;
                if((stats_user_2?0:beats)==FRAME_BEATS-1) begin
                    frame_active<=0;
                    snapshot_r<=next_r;snapshot_g<=next_g;snapshot_b<=next_b;
                    snapshot_n<=next_n;snapshot_valid<=1;
                end else beats<=(stats_user_2?0:beats)+1'b1;
            end
            if(snapshot_valid && !busy && snapshot_n>=MIN_SAMPLES && snapshot_r!=0 && snapshot_b!=0) begin
                saved_numerator<={snapshot_g,8'd0};saved_blue<=snapshot_b;
                dividend<={snapshot_g,8'd0};denominator<=snapshot_r;
                quotient<=0;remainder<=0;bit_index<=0;blue_phase<=0;busy<=1;
            end
            if(busy) begin
                dividend<={dividend[38:0],1'b0};quotient<=next_q;
                remainder<=subtract_ok ? trial_difference[32:0] : shifted;
                if(bit_index==39) begin
                    if(!blue_phase) begin
                        result_r<=clamp_gain(next_q);blue_phase<=1;
                        dividend<=saved_numerator;denominator<=saved_blue;
                        quotient<=0;remainder<=0;bit_index<=0;
                    end else begin
                        target_r<=result_r;target_b<=clamp_gain(next_q);
                        target_ready<=1;busy<=0;
                    end
                end else bit_index<=bit_index+1'b1;
            end
        end
    end
endmodule
