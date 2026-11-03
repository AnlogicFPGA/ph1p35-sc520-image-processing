// Manual SC520 calibration, entirely in the sensor configuration clock domain.
// Exposure units are half-lines; VTS=1000 limits exposure to 1991 half-lines.
// Status = {error,busy,applied_valid,gain_index[2:0],exposure_index[2:0]}.
// On error, bits [7:6] instead carry 1=NACK, 2=start timeout, 3=done timeout.
// "applied" means all grouped I2C writes completed without a reported bus error;
// it is not a sensor register readback.
module camera_calibration_ctrl #(
    parameter integer TIMEOUT_CYCLES = 2380952,
    parameter integer AUTO_EXPOSURE_ENABLE = 0,
    parameter integer AE_FRAME_SAMPLES = 230400
)(
    input wire I_clk, I_rst_n,
    input wire [2:0] I_adjust_toggle,
    input wire I_auto_resume_toggle,
    input wire [69:0] I_auto_stats,
    input wire I_cam_cfg_done, I_ae_cfg_done, I_iic_error,
    output reg O_ae_req,
    output reg [15:0] O_ae, O_ag,
    output wire [8:0] O_status,
    output wire O_auto_active
);
    (* preserve=1, keep=1, ASYNC_REG="TRUE" *) reg [2:0] toggle_sync1, toggle_sync2;
    reg [2:0] toggle_seen;
    wire [2:0] press = toggle_sync2 ^ toggle_seen;
    reg [2:0] exposure_index, gain_index;
    reg [2:0] sent_exposure, sent_gain, applied_exposure, applied_gain;
    reg applied_valid, error, force_write;
    reg [1:0] error_code;
    localparam [1:0] IDLE=0, WAIT_START=1, WAIT_DONE=2, FAILED=3;
    reg [1:0] state;
    localparam integer TIMER_WIDTH = (TIMEOUT_CYCLES < 2) ? 1 : $clog2(TIMEOUT_CYCLES);
    reg [TIMER_WIDTH-1:0] timeout_count;
    (* preserve=1, keep=1, ASYNC_REG="TRUE" *) reg auto_sync1, auto_sync2;
    reg auto_seen, auto_active;
    reg [7:0] stats_seen;
    reg [3:0] settle_frames;
    reg [1:0] bright_confirm, dark_confirm;
    wire [7:0] stats_seq=I_auto_stats[69:62];
    wire [25:0] stats_sum=I_auto_stats[61:36];
    wire [17:0] stats_near=I_auto_stats[35:18];
    wire [17:0] stats_clip=I_auto_stats[17:0];
    wire new_stats=(stats_seq!=stats_seen);
    wire low_mean=(stats_sum<AE_FRAME_SAMPLES*105);
    // Meter H/C now counts the maximum post-WB RGB channel. Near saturation
    // holds exposure, while actual channel clipping drives it down. Do not
    // darken a safe saturated-color object merely because its luminance is low.
    wire too_bright=(stats_clip>AE_FRAME_SAMPLES/50) ||
                    (stats_sum>AE_FRAME_SAMPLES*235);
    wire too_dark=low_mean && (stats_near<=AE_FRAME_SAMPLES/100);
    wire pending = force_write || !applied_valid || exposure_index!=applied_exposure || gain_index!=applied_gain;
    wire busy = !error && (state!=IDLE || pending);
    assign O_status = {error,(error ? error_code : {busy,applied_valid}),applied_gain,applied_exposure};
    assign O_auto_active = (AUTO_EXPOSURE_ENABLE!=0) && auto_active;

    function [15:0] exposure_value;
        input [2:0] index;
        begin
            case(index)
                0:exposure_value=125; 1:exposure_value=250;
                2:exposure_value=500; 3:exposure_value=750;
                4:exposure_value=1000; 5:exposure_value=1250;
                6:exposure_value=1500; default:exposure_value=1991;
            endcase
        end
    endfunction
    function [15:0] gain_control;
        input [2:0] index;
        begin
            case(index)
                0:gain_control=16; 1:gain_control=48; 2:gain_control=80;
                3:gain_control=112; 4:gain_control=144;
                default:gain_control=144;
            endcase
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            toggle_sync1<=0; toggle_sync2<=0; toggle_seen<=0;
            exposure_index<=4; gain_index<=0;
            auto_sync1<=0; auto_sync2<=0; auto_seen<=0;
            auto_active<=AUTO_EXPOSURE_ENABLE!=0;
            stats_seen<=0; settle_frames<=8;
            bright_confirm<=0; dark_confirm<=0;
        end else begin
            toggle_sync1<=I_adjust_toggle; toggle_sync2<=toggle_sync1;
            toggle_seen<=toggle_sync2;
            auto_sync1<=I_auto_resume_toggle; auto_sync2<=auto_sync1;
            auto_seen<=auto_sync2;
            if(press[2]) gain_index<=(gain_index==4) ? 0 : gain_index+1'b1;
            else if(press[0] && exposure_index!=0) exposure_index<=exposure_index-1'b1;
            else if(press[1] && exposure_index!=7) exposure_index<=exposure_index+1'b1;
            if(AUTO_EXPOSURE_ENABLE!=0) begin
                if(|press) begin
                    auto_active<=0;
                    bright_confirm<=0; dark_confirm<=0;
                end
                if(auto_sync2!=auto_seen) begin
                    auto_active<=1;
                    settle_frames<=8;
                    bright_confirm<=0; dark_confirm<=0;
                end
                if(new_stats) begin
                    stats_seen<=stats_seq;
                    if(settle_frames!=0) begin
                        settle_frames<=settle_frames-1'b1;
                        bright_confirm<=0; dark_confirm<=0;
                    end
                    else if(auto_active && !(|press) && state==IDLE &&
                            applied_valid && !error && !pending) begin
                        if(too_bright) begin
                            dark_confirm<=0;
                            if(bright_confirm==2) begin
                                if(gain_index!=0) gain_index<=gain_index-1'b1;
                                else if(exposure_index!=0) exposure_index<=exposure_index-1'b1;
                                bright_confirm<=0;
                                settle_frames<=8;
                            end else bright_confirm<=bright_confirm+1'b1;
                        end else if(too_dark) begin
                            bright_confirm<=0;
                            if(dark_confirm==2) begin
                                if(exposure_index!=7) exposure_index<=exposure_index+1'b1;
                                else if(gain_index<3) gain_index<=gain_index+1'b1;
                                dark_confirm<=0;
                                settle_frames<=8;
                            end else dark_confirm<=dark_confirm+1'b1;
                        end else begin
                            bright_confirm<=0; dark_confirm<=0;
                        end
                    end else begin
                        bright_confirm<=0; dark_confirm<=0;
                    end
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_ae_req<=0; O_ae<=1000; O_ag<=16;
            sent_exposure<=4; sent_gain<=0;
            applied_exposure<=4; applied_gain<=0;
            applied_valid<=0; error<=0; error_code<=0; force_write<=1;
            state<=IDLE; timeout_count<=0;
        end else begin
            O_ae_req<=0;
            case(state)
                IDLE: if(I_cam_cfg_done && I_ae_cfg_done && pending) begin
                    // Latch the whole transaction. New presses are coalesced
                    // until this grouped write completes; its bus never tears.
                    O_ae<=exposure_value(exposure_index);
                    O_ag<=gain_control(gain_index);
                    sent_exposure<=exposure_index; sent_gain<=gain_index;
                    O_ae_req<=1; timeout_count<=0; state<=WAIT_START;
                    force_write<=0;
                end else if(I_cam_cfg_done && pending) begin
                    if(timeout_count==TIMEOUT_CYCLES-1) begin
                        error<=1; error_code<=3; state<=FAILED;
                    end else timeout_count<=timeout_count+1'b1;
                end
                WAIT_START, WAIT_DONE: begin
                    if(timeout_count==TIMEOUT_CYCLES-1) begin
                        error<=1; error_code<=(state==WAIT_START) ? 2 : 3;
                        state<=FAILED;
                    end else begin
                        timeout_count<=timeout_count+1'b1;
                        if(state==WAIT_START && !I_ae_cfg_done) state<=WAIT_DONE;
                        else if(state==WAIT_DONE && I_ae_cfg_done) begin
                            if(I_iic_error) begin
                                error<=1; error_code<=1; state<=FAILED;
                            end else begin
                                applied_exposure<=sent_exposure; applied_gain<=sent_gain;
                                applied_valid<=1; state<=IDLE; timeout_count<=0;
                            end
                        end
                    end
                end
                FAILED: if(|press || auto_sync2!=auto_seen) begin
                    // A manual key or K4 long-press AUTO resume explicitly
                    // retries a failed transaction. Never publish a failed payload.
                    error<=0; error_code<=0; force_write<=1;
                    timeout_count<=0; state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
