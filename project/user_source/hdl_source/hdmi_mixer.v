module hdmi_mixer #(
    parameter H_OFFSET = 0,
    parameter V_OFFSET = 0,
    parameter IMG_WIDTH = 1600,
    parameter IMG_HEIGHT = 900,
    parameter integer DEBUG_MODE = 0,
    parameter integer CAMERA_CALIBRATION = 0,
    parameter integer FRAME_LATENCY_ENABLE = 1,
    parameter integer SC520_CAPTURE_ENABLE = 0,
    parameter integer SC520_RGB_CAPTURE = 0,
    parameter integer OBJECT_MASK_PREVIEW = 0,
    parameter integer OBJECT_RGB_PROBE = 0,
    parameter integer OSD_PIPELINE_ENABLE = 0,
    parameter integer MOTION_TRACK_ENABLE = 0,
    parameter integer TRACK_EVENT_COUNT = 0,
    parameter integer TRACK_COUNT_CLOCK_HZ = 75000000,
    parameter integer PINGPONG_DETECT_ENABLE = 0,
    parameter integer OBJECT_HOLD_FRAMES = 60,
    parameter integer RED_MIN_TARGET_SPAN = 50,
    parameter integer RED_MIN_TARGET_SAMPLES = 80,
    parameter integer AE_DISPLAY = 0,
    parameter integer VIDEO_VALID_CHECK = 0,
    parameter integer WARM_DISPLAY_ENABLE = 0,
    parameter integer MIDTONE_DISPLAY_ENABLE = 0,
    parameter integer CHROMA_DENOISE_ENABLE = 0,
    parameter integer GIMBAL_ANTI_HUNT=0,GIMBAL_BOTH_AXES=0,GIMBAL_DIAG=0,
    parameter integer GIMBAL_PAN_MIN_US=1400,GIMBAL_PAN_MAX_US=1600,
    parameter integer GIMBAL_TILT_MIN_US=1400,GIMBAL_TILT_MAX_US=1600,
    parameter integer GIMBAL_FOLLOW_ENABLE = 0,
    parameter integer GIMBAL_CLK_HZ = 75000000,
    parameter integer GIMBAL_READ_LATENCY = 0,
    parameter integer GIMBAL_TARGET_CLASS = 0, GIMBAL_LOST_FRAMES = 15,
    parameter integer GIMBAL_REF_X = IMG_WIDTH/2, GIMBAL_REF_Y = IMG_HEIGHT/2,
    parameter integer GIMBAL_PAN_REVERSE = 0, GIMBAL_TILT_REVERSE = 1
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[2:0]   I_image_mode,
    input wire[8:0]   I_camera_status,
    input wire I_ae_auto,
    input wire [31:0] I_ae_meter,
    input wire[1:0]   I_wb_mode,
    input wire I_wb_calibrated,
    input wire[31:0]  I_latency_now_ms,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,
    input wire        I_video_rd_valid,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data,
    output wire       O_capture_uart_tx,
    output wire       O_gimbal_pan_pwm,O_gimbal_tilt_pwm
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg S_video_valid_1d, S_video_valid_2d;
    reg        S_window_active_1d;
    reg        S_window_active_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;
    wire[23:0] S_osd_video_data;
    wire[23:0] S_latency_video_data;
    wire[23:0] S_composed_video_data;
    wire[23:0] S_clean_video_data;
    wire[23:0] S_denoised_video_data;
    wire[23:0] S_midtone_video_data;
    wire[23:0] S_display_video_data;
    // Diagnostic pixels match the 021 detector predicates and are delayed to
    // the display-denoise output coordinate.  Magenta=warm, cyan=blue.
    wire [7:0] S_mask_r = S_clean_video_data[23:16];
    wire [7:0] S_mask_g = S_clean_video_data[15:8];
    wire [7:0] S_mask_b = S_clean_video_data[7:0];
    wire S_mask_warm_now = S_mask_r >= 8'd48 &&
        {1'b0,S_mask_r} >= {1'b0,S_mask_g}+9'd48 &&
        {1'b0,S_mask_r} >= {1'b0,S_mask_b}+9'd32;
    wire S_mask_blue_now = S_mask_b >= 8'd45 &&
        {1'b0,S_mask_b} >= {1'b0,S_mask_r}+9'd12 &&
        {1'b0,S_mask_b} >= {1'b0,S_mask_g}+9'd8;
    reg [3:0] S_mask_warm_delay, S_mask_blue_delay;
    wire S_mask_warm_display = CHROMA_DENOISE_ENABLE && DEBUG_MODE==0 ?
                               S_mask_warm_delay[3] : S_mask_warm_now;
    wire S_mask_blue_display = CHROMA_DENOISE_ENABLE && DEBUG_MODE==0 ?
                               S_mask_blue_delay[3] : S_mask_blue_now;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_mask_warm_delay <= 4'b0;
            S_mask_blue_delay <= 4'b0;
        end else begin
            S_mask_warm_delay <= {S_mask_warm_delay[2:0],S_mask_warm_now};
            S_mask_blue_delay <= {S_mask_blue_delay[2:0],S_mask_blue_now};
        end
    end
    wire S_disp_vsync, S_disp_hsync, S_disp_de, S_disp_window_active;
    wire [11:0] S_disp_x, S_disp_y;
    localparam integer FONT_PIPE = OSD_PIPELINE_ENABLE && DEBUG_MODE==0;
    reg S_font_vs,S_font_hs,S_font_de,S_font_active,S_font_latency_sel;
    reg S_font_warm,S_font_blue;
    reg [23:0] S_font_latency_rgb;
    wire S_output_active=FONT_PIPE ? S_font_active : S_disp_window_active;
    wire S_output_warm=FONT_PIPE ? S_font_warm : S_mask_warm_display;
    wire S_output_blue=FONT_PIPE ? S_font_blue : S_mask_blue_display;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_font_vs<=0;S_font_hs<=0;S_font_de<=0;S_font_active<=0;
            S_font_latency_sel<=0;S_font_latency_rgb<=0;S_font_warm<=0;S_font_blue<=0;
        end else begin
            S_font_vs<=S_disp_vsync;S_font_hs<=S_disp_hsync;S_font_de<=S_disp_de;
            S_font_active<=S_disp_window_active;
            S_font_latency_sel<=FRAME_LATENCY_ENABLE && S_disp_x>=1000 && S_disp_x<1132 &&
                                S_disp_y>=24 && S_disp_y<46;
            S_font_latency_rgb<=S_latency_video_data;
            S_font_warm<=S_mask_warm_display;S_font_blue<=S_mask_blue_display;
        end
    end
    wire        S_latency_valid;
    wire[15:0] S_latency_bcd;
    wire S_source_valid;
    wire S_pingpong_publish;
    wire [31:0] S_source_ms;
    reg S_object_publish_d;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) S_object_publish_d<=0;
        else S_object_publish_d<=S_pingpong_publish;
    reg S_user_1d, S_user_2d;
    reg [2:0] S_display_mode;
    (* preserve=1, keep=1 *) reg [8:0] S_camera_status_frame;
    reg [1:0] S_wb_mode_frame;
    reg S_wb_calibrated_frame;
    reg S_ae_auto_frame;
    reg [31:0] S_ae_meter_frame;
    wire [2:0] S_requested_mode=I_image_mode==3'd5 ? 3'd0 : I_image_mode;
    wire [2:0] S_mode_for_frame = S_user_2d ? S_requested_mode : S_display_mode;
    wire S_motion_valid, S_motion_fresh, S_motion_publish;
    wire [11:0] S_motion_x_min, S_motion_x_max, S_motion_y_min, S_motion_y_max;
    wire [11:0] S_motion_center_x, S_motion_center_y;
    wire [15:0] S_motion_x_bcd, S_motion_y_bcd;
    wire [2:0] S_motion_diag_reason;
    wire [15:0] S_motion_diag_changed, S_motion_diag_count_bcd;
    wire [15:0] S_motion_frame_bcd;
    wire [15:0] S_throw_count_bcd;
    generate if(MOTION_TRACK_ENABLE && TRACK_EVENT_COUNT)begin : g_throw_count
        motion_event_counter #(.CLOCK_HZ(TRACK_COUNT_CLOCK_HZ)) u_counter(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(S_mode_for_frame==6 && DEBUG_MODE==0),
            .I_publish(S_motion_publish),.I_fresh(S_motion_fresh),
            .O_count_bcd(S_throw_count_bcd),.O_increment());
    end else begin : g_no_throw_count
        assign S_throw_count_bcd=0;
    end endgenerate
    wire S_pingpong_valid, S_pingpong_fresh;
    wire [2:0] S_pingpong_class_id;
    wire S_pingpong_raw_valid;
    wire [2:0] S_pingpong_raw_class_id;
    wire [11:0] S_pingpong_raw_x_min, S_pingpong_raw_x_max;
    wire [11:0] S_pingpong_raw_y_min, S_pingpong_raw_y_max;
    wire [11:0] S_pingpong_raw_center_x, S_pingpong_raw_center_y;
    wire [11:0] S_pingpong_x_min, S_pingpong_x_max, S_pingpong_y_min, S_pingpong_y_max;
    wire [11:0] S_pingpong_center_x, S_pingpong_center_y;
    wire [2:0] S_pingpong_diag_reason;
    wire [15:0] S_pingpong_diag_count;
    wire [15:0] S_pingpong_diag_count_bcd;
    wire [15:0] S_blue_diag_count_bcd;
    wire [15:0] S_pingpong_x_bcd, S_pingpong_y_bcd;
    wire S_orange_valid, S_blue_valid;
    wire [2:0] S_warm_class_id;
    wire [2:0] S_palette_class_id;
    wire [11:0] S_orange_x_min, S_orange_x_max, S_orange_y_min, S_orange_y_max;
    wire [11:0] S_blue_x_min, S_blue_x_max, S_blue_y_min, S_blue_y_max;
    wire [11:0] S_orange_center_x, S_orange_center_y;
    wire [11:0] S_blue_center_x, S_blue_center_y;
    wire [2:0] S_orange_diag_reason, S_blue_diag_reason;
    wire [15:0] S_orange_diag_count, S_blue_diag_count;
    wire S_orange_publish, S_blue_publish;
    wire S_red_locked,S_blue_locked,S_red_fresh,S_blue_fresh;
    wire S_red_box_valid=S_red_locked && S_red_fresh;
    wire S_blue_box_valid=S_blue_locked && S_blue_fresh;
    wire [1:0] S_object_count={1'b0,S_red_box_valid}+{1'b0,S_blue_box_valid};
    wire [11:0] S_red_box_x_min,S_red_box_x_max,S_red_box_y_min,S_red_box_y_max;
    wire [11:0] S_blue_box_x_min,S_blue_box_x_max,S_blue_box_y_min,S_blue_box_y_max;
    wire [11:0] S_red_cx,S_red_cy,S_blue_cx,S_blue_cy;
    wire S_result_tick;
    wire S_red_display_valid,S_blue_display_valid;
    wire [1:0] S_object_display_count={1'b0,S_red_display_valid}+{1'b0,S_blue_display_valid};
    wire [11:0] S_red_display_x_min,S_red_display_x_max,S_red_display_y_min,S_red_display_y_max;
    wire [11:0] S_blue_display_x_min,S_blue_display_x_max,S_blue_display_y_min,S_blue_display_y_max;
    wire [63:0] S_object_coord_bcd;
    generate if(PINGPONG_DETECT_ENABLE)begin : g_pair_coordinates
        object_pair_coordinates u_coordinates(
            .clk(I_clk),.rst_n(I_rst_n),.update(S_result_tick),
            .red_x_min(S_red_display_x_min),.red_x_max(S_red_display_x_max),
            .red_y_min(S_red_display_y_min),.red_y_max(S_red_display_y_max),
            .blue_x_min(S_blue_display_x_min),.blue_x_max(S_blue_display_x_max),
            .blue_y_min(S_blue_display_y_min),.blue_y_max(S_blue_display_y_max),
            .coordinates_bcd(S_object_coord_bcd),.publish());
    end else begin : g_no_pair_coordinates
        assign S_object_coord_bcd=0;
    end endgenerate
    object_display_hold #(.HOLD_FRAMES(OBJECT_HOLD_FRAMES)) u_red_display_hold (
        .clk(I_clk),.rst_n(I_rst_n),.enable(PINGPONG_DETECT_ENABLE && S_mode_for_frame==7),
        .frame_tick(S_result_tick),.fresh(S_red_box_valid),
        .x_min(S_red_box_x_min),.x_max(S_red_box_x_max),.y_min(S_red_box_y_min),.y_max(S_red_box_y_max),
        .valid(S_red_display_valid),.out_x_min(S_red_display_x_min),.out_x_max(S_red_display_x_max),
        .out_y_min(S_red_display_y_min),.out_y_max(S_red_display_y_max));
    object_display_hold #(.HOLD_FRAMES(OBJECT_HOLD_FRAMES)) u_blue_display_hold (
        .clk(I_clk),.rst_n(I_rst_n),.enable(PINGPONG_DETECT_ENABLE && S_mode_for_frame==7),
        .frame_tick(S_result_tick),.fresh(S_blue_box_valid),
        .x_min(S_blue_box_x_min),.x_max(S_blue_box_x_max),.y_min(S_blue_box_y_min),.y_max(S_blue_box_y_max),
        .valid(S_blue_display_valid),.out_x_min(S_blue_display_x_min),.out_x_max(S_blue_display_x_max),
        .out_y_min(S_blue_display_y_min),.out_y_max(S_blue_display_y_max));
    // When SEARCH sees both a held blue object and a warm hand/background,
    // prefer the qualified blue square. Once locked, preserve the old class.
    wire S_choose_blue = S_blue_box_valid;
    wire S_choose_orange = S_red_box_valid && !S_choose_blue;
    wire [1:0] S_diag_choice = (S_choose_orange || S_choose_blue) ?
        (S_choose_orange ? (S_warm_class_id==3'd3 ? 2'd2 : 2'd0) : 2'd1) :
        (S_blue_diag_count>=16'd16) ? 2'd1 : 2'd0;
    wire S_pingpong_mode = PINGPONG_DETECT_ENABLE && S_display_mode==3'd7;
    wire S_track_mode = MOTION_TRACK_ENABLE && S_display_mode==3'd6;
    wire S_osd_detect_valid = S_pingpong_mode ? S_pingpong_valid :
                              S_track_mode && S_motion_valid;
    wire [2:0] S_osd_detect_class =
        S_pingpong_mode ? (S_pingpong_fresh ? S_pingpong_class_id : 3'd5) : S_track_mode ?
        (S_motion_fresh ? 3'd6 : 3'd5) : 3'd0;
    wire [11:0] S_osd_x_min = S_pingpong_mode ? S_pingpong_x_min :
                              S_track_mode ? S_motion_x_min : 12'd0;
    wire [11:0] S_osd_x_max = S_pingpong_mode ? S_pingpong_x_max :
                              S_track_mode ? S_motion_x_max : 12'd0;
    wire [11:0] S_osd_y_min = S_pingpong_mode ? S_pingpong_y_min :
                              S_track_mode ? S_motion_y_min : 12'd0;
    wire [11:0] S_osd_y_max = S_pingpong_mode ? S_pingpong_y_max :
                              S_track_mode ? S_motion_y_max : 12'd0;
    wire [15:0] S_osd_center_x_bcd =
        S_pingpong_mode ? S_pingpong_x_bcd : S_track_mode ?
        S_motion_x_bcd : 16'd0;
    wire [15:0] S_osd_center_y_bcd =
        S_pingpong_mode ? S_pingpong_y_bcd : S_track_mode ?
        S_motion_y_bcd : 16'd0;

    wire S_window_active;
    wire S_analysis_valid = S_window_active_2d &&
                            (!VIDEO_VALID_CHECK || S_video_valid_2d);
    wire [23:0] S_probe_rgb;
    wire S_probe_valid;
    generate if (OBJECT_RGB_PROBE) begin : g_rgb_probe
        object_rgb_probe #(.X(IMG_WIDTH/2),.Y(IMG_HEIGHT/2)) u_probe (
            .clk(I_clk),.rst_n(I_rst_n),.frame_start(S_user_2d),
            .valid(S_analysis_valid),.x(S_x_2d),.y(S_y_2d),
            .rgb(S_clean_video_data),.out_rgb(S_probe_rgb),.out_valid(S_probe_valid));
    end else begin : g_no_rgb_probe
        assign S_probe_rgb=24'd0;
        assign S_probe_valid=1'b0;
    end endgenerate
    // USER can coincide with the first active pixel, before S_y resets.
    wire [11:0] S_raster_y=I_video_user ? 12'd0 : S_y;
    assign S_window_active = I_video_de &&
                             (S_x >= H_OFFSET) &&
                             (S_x < (H_OFFSET + IMG_WIDTH)) &&
                             (S_raster_y >= V_OFFSET) &&
                             (S_raster_y < (V_OFFSET + IMG_HEIGHT));

    // Read exactly IMG_WIDTH pixels on each of IMG_HEIGHT lines.  The HDMI
    // active area may be larger than the stored frame, in which case remaining
    // pixels are emitted as black borders without consuming DDR data.
    assign O_video_rd_en = S_window_active;

    // Diagnostic capture samples aligned camera pixels before the
    // latency marker, image modes and OSD modify the displayed picture.
    generate if (SC520_CAPTURE_ENABLE) begin : g_sc520_capture
        sc520_frame_dump #(.RGB_CAPTURE(SC520_RGB_CAPTURE)) u_sc520_frame_dump (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_frame_start(S_user_2d), .I_pixel_valid(S_analysis_valid),
            .I_x(S_x_2d), .I_y(S_y_2d), .I_rgb(S_video_data_2d),
            .O_uart_tx(O_capture_uart_tx)
        );
    end else begin : g_no_sc520_capture
        assign O_capture_uart_tx = 1'b1;
    end endgenerate

    // Display-only preference.  Keep analysis, AE and stored RGB unchanged.
    // Midtones gain a little red and lose half as much blue.  Headroom makes
    // the effect fade to zero at a clipped highlight or fully saturated color.
    function [23:0] warm_display_rgb;
        input [23:0] rgb;
        reg [7:0] max_rg, max_rgb, headroom, red_lift, blue_cut;
        reg [8:0] red_sum;
        begin
            max_rg = rgb[23:16] > rgb[15:8] ? rgb[23:16] : rgb[15:8];
            max_rgb = max_rg > rgb[7:0] ? max_rg : rgb[7:0];
            headroom = (8'd255 - max_rgb) >> 1;
            red_lift = rgb[23:16] >> 3;
            if (red_lift > headroom) red_lift = headroom;
            blue_cut = red_lift >> 1;
            if (blue_cut > rgb[7:0]) blue_cut = rgb[7:0];
            red_sum = {1'b0, rgb[23:16]} + {1'b0, red_lift};
            warm_display_rgb = {red_sum[7:0], rgb[15:8],
                                rgb[7:0] - blue_cut};
        end
    endfunction

    // Gentle display-only tone curve.  Leave near-black codes untouched so
    // sensor color speckles there are not boosted.  Lift midtones by at most
    // 16 codes, roll the lift away before 224, and preserve 224..255 exactly.
    // The curve is monotonic and needs only shifts/adds (no frame delay).
    function [7:0] midtone_gamma8;
        input [7:0] value;
        reg [8:0] result;
        begin
            if (value < 8'd32)
                result = {1'b0, value};
            else if (value < 8'd64)
                result = {1'b0, value} + (({1'b0, value} - 9'd32) >> 3);
            else if (value < 8'd128)
                result = {1'b0, value} + 9'd4 +
                         (({1'b0, value} - 9'd64) >> 3);
            else if (value < 8'd192)
                result = {1'b0, value} + 9'd12 +
                         (({1'b0, value} - 9'd128) >> 4);
            else if (value < 8'd224)
                result = {1'b0, value} + 9'd16 -
                         (({1'b0, value} - 9'd192) >> 1);
            else
                result = {1'b0, value};
            midtone_gamma8 = result[7:0];
        end
    endfunction

    function [23:0] midtone_display_rgb;
        input [23:0] rgb;
        begin
            midtone_display_rgb = {midtone_gamma8(rgb[23:16]),
                                   midtone_gamma8(rgb[15:8]),
                                   midtone_gamma8(rgb[7:0])};
        end
    endfunction

    // Chroma denoising acts only on the HDMI display branch.
    // All analysis, AE and diagnostic capture taps stay before this stage.
    generate if (CHROMA_DENOISE_ENABLE && DEBUG_MODE == 0) begin : g_chroma_denoise
        display_chroma_denoise #(.IMG_WIDTH(IMG_WIDTH)) u_display_chroma_denoise (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_rgb(S_clean_video_data), .I_x(S_x_2d), .I_y(S_y_2d),
            .I_window_active(S_window_active_2d),
            .I_enable(S_mode_for_frame == 3'd0 ||

                      S_mode_for_frame == 3'd6 || S_mode_for_frame == 3'd7),
            .I_vsync(S_video_vsync_2d), .I_hsync(S_video_hsync_2d),
            .I_de(S_video_de_2d),
            .O_rgb(S_denoised_video_data),
            .O_x(S_disp_x), .O_y(S_disp_y),
            .O_window_active(S_disp_window_active),
            .O_vsync(S_disp_vsync), .O_hsync(S_disp_hsync),
            .O_de(S_disp_de)
        );
    end else begin : g_no_chroma_denoise
        assign S_denoised_video_data = S_clean_video_data;
        assign S_disp_x = S_x_2d; assign S_disp_y = S_y_2d;
        assign S_disp_window_active = S_window_active_2d;
        assign S_disp_vsync = S_video_vsync_2d;
        assign S_disp_hsync = S_video_hsync_2d;
        assign S_disp_de = S_video_de_2d;
    end endgenerate

    assign S_midtone_video_data =
        MIDTONE_DISPLAY_ENABLE &&
        (S_mode_for_frame == 3'd0 ||
         S_mode_for_frame == 3'd6 || S_mode_for_frame == 3'd7)
            ? midtone_display_rgb(S_denoised_video_data) : S_denoised_video_data;

    assign S_display_video_data =
        WARM_DISPLAY_ENABLE &&
        (S_mode_for_frame == 3'd0 ||
         S_mode_for_frame == 3'd6 || S_mode_for_frame == 3'd7)
            ? warm_display_rgb(S_midtone_video_data) : S_midtone_video_data;

    generate if (MOTION_TRACK_ENABLE) begin : g_motion_track
        motion_track_display #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT))
        u_motion_track_display (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_enable(S_mode_for_frame==3'd6 && DEBUG_MODE==0),
            .I_frame_start(S_user_2d), .I_pixel_valid(S_analysis_valid),
            .I_x(S_x_2d), .I_y(S_y_2d), .I_rgb(S_clean_video_data),
            .O_track_valid(S_motion_valid), .O_fresh(S_motion_fresh),
            .O_x_min(S_motion_x_min), .O_x_max(S_motion_x_max),
            .O_y_min(S_motion_y_min), .O_y_max(S_motion_y_max),
            .O_center_x(S_motion_center_x), .O_center_y(S_motion_center_y),
            .O_publish_pulse(S_motion_publish),
            .O_frame_bcd(S_motion_frame_bcd),
            .O_diag_reason(S_motion_diag_reason),
            .O_diag_changed(S_motion_diag_changed)
        );
        frame_latency_bcd u_motion_x_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(S_motion_publish),
            .O_in_ready(), .I_ms({4'b0,S_motion_center_x}),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_motion_x_bcd)
        );
        frame_latency_bcd u_motion_y_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(S_motion_publish),
            .O_in_ready(), .I_ms({4'b0,S_motion_center_y}),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_motion_y_bcd)
        );
        frame_latency_bcd u_motion_diag_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(S_motion_publish),
            .O_in_ready(), .I_ms(S_motion_diag_changed),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_motion_diag_count_bcd)
        );
    end else begin : g_no_motion_track
        assign S_motion_valid = 1'b0;
        assign S_motion_fresh = 1'b0;
        assign S_motion_publish = 1'b0;
        assign S_motion_x_min = 12'd0; assign S_motion_x_max = 12'd0;
        assign S_motion_y_min = 12'd0; assign S_motion_y_max = 12'd0;
        assign S_motion_center_x = 12'd0; assign S_motion_center_y = 12'd0;
        assign S_motion_x_bcd = 16'd0; assign S_motion_y_bcd = 16'd0;
        assign S_motion_diag_reason = 3'd0;
        assign S_motion_diag_changed = 16'd0;
        assign S_motion_diag_count_bcd = 16'd0;
        assign S_motion_frame_bcd = 16'd0;
    end endgenerate

    generate if (PINGPONG_DETECT_ENABLE) begin : g_pingpong_detect
        pingpong_orange_detector #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),
                                   .COLOR_KIND(0), .SHAPE_KIND(0), .CLASS_ID(3'd1),
                                   .MIN_TARGET_SPAN(RED_MIN_TARGET_SPAN),
                                   .MIN_TARGET_SAMPLES(RED_MIN_TARGET_SAMPLES),
                                   .SC520_RELATIVE_COLOR(1))
        u_orange_ball_detector (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_enable(S_mode_for_frame==3'd7 && DEBUG_MODE==0),
            .I_frame_start(S_user_2d), .I_pixel_valid(S_analysis_valid),
            .I_x(S_x_2d), .I_y(S_y_2d), .I_rgb(S_clean_video_data),
            .O_target_valid(S_orange_valid), .O_class_id(S_warm_class_id),
            .O_x_min(S_orange_x_min), .O_x_max(S_orange_x_max),
            .O_y_min(S_orange_y_min), .O_y_max(S_orange_y_max),
            .O_center_x(S_orange_center_x), .O_center_y(S_orange_center_y),
            .O_publish_pulse(S_orange_publish),
            .O_diag_reason(S_orange_diag_reason), .O_diag_count(S_orange_diag_count)
        );
        pingpong_orange_detector #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),
                                   .COLOR_KIND(1), .SHAPE_KIND(1), .CLASS_ID(3'd2),
                                   .SC520_RELATIVE_COLOR(1))
        u_blue_cube_detector (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_enable(S_mode_for_frame==3'd7 && DEBUG_MODE==0),
            .I_frame_start(S_user_2d), .I_pixel_valid(S_analysis_valid),
            .I_x(S_x_2d), .I_y(S_y_2d), .I_rgb(S_clean_video_data),
            .O_target_valid(S_blue_valid), .O_class_id(S_palette_class_id),
            .O_x_min(S_blue_x_min), .O_x_max(S_blue_x_max),
            .O_y_min(S_blue_y_min), .O_y_max(S_blue_y_max),
            .O_center_x(S_blue_center_x), .O_center_y(S_blue_center_y),
            .O_publish_pulse(S_blue_publish),
            .O_diag_reason(S_blue_diag_reason), .O_diag_count(S_blue_diag_count)
        );
        // Blue's rotation check finishes after the warm candidate. Publish
        // the shared snapshot only when both results belong to this frame.
        assign S_pingpong_publish=S_blue_publish;
        // Keep orange internal to diagnostics; publish only red or blue.
        assign S_pingpong_raw_valid=(S_choose_orange && S_warm_class_id==3'd3) || S_choose_blue;
        assign S_pingpong_raw_class_id=S_choose_orange ? S_warm_class_id :
                                       S_choose_blue ? S_palette_class_id : 3'd0;
        assign S_pingpong_raw_x_min=S_choose_orange ? S_orange_x_min :
                                     S_blue_x_min;
        assign S_pingpong_raw_x_max=S_choose_orange ? S_orange_x_max :
                                     S_blue_x_max;
        assign S_pingpong_raw_y_min=S_choose_orange ? S_orange_y_min :
                                     S_blue_y_min;
        assign S_pingpong_raw_y_max=S_choose_orange ? S_orange_y_max :
                                     S_blue_y_max;
        assign S_pingpong_raw_center_x=S_choose_orange ? S_orange_center_x :
                                         S_blue_center_x;
        assign S_pingpong_raw_center_y=S_choose_orange ? S_orange_center_y :
                                         S_blue_center_y;
        assign S_pingpong_diag_reason=S_diag_choice==1 ? S_blue_diag_reason : S_orange_diag_reason;
        assign S_pingpong_diag_count=S_diag_choice==1 ? S_blue_diag_count : S_orange_diag_count;
        // Confirm and associate the two color classes independently. A blue
        // candidate must not hide or reset an otherwise valid red candidate.
        pingpong_target_stabilizer u_red_target_stabilizer (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_enable(S_mode_for_frame==3'd7 && DEBUG_MODE==0),
            .I_publish(S_pingpong_publish),
            .I_candidate_valid(S_orange_valid && S_warm_class_id==3'd3),.I_class_id(3'd3),
            .I_x_min(S_orange_x_min), .I_x_max(S_orange_x_max),
            .I_y_min(S_orange_y_min), .I_y_max(S_orange_y_max),
            .I_center_x(S_orange_center_x),.I_center_y(S_orange_center_y),
            .O_target_valid(S_red_locked),.O_fresh(S_red_fresh),.O_class_id(),
            .O_x_min(S_red_box_x_min),.O_x_max(S_red_box_x_max),
            .O_y_min(S_red_box_y_min),.O_y_max(S_red_box_y_max),
            .O_center_x(S_red_cx),.O_center_y(S_red_cy)
        );
        pingpong_target_stabilizer u_blue_target_stabilizer (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_enable(S_mode_for_frame==3'd7 && DEBUG_MODE==0),.I_publish(S_pingpong_publish),
            .I_candidate_valid(S_blue_valid),.I_class_id(3'd2),
            .I_x_min(S_blue_x_min),.I_x_max(S_blue_x_max),
            .I_y_min(S_blue_y_min),.I_y_max(S_blue_y_max),
            .I_center_x(S_blue_center_x),.I_center_y(S_blue_center_y),
            .O_target_valid(S_blue_locked),.O_fresh(S_blue_fresh),.O_class_id(),
            .O_x_min(S_blue_box_x_min),.O_x_max(S_blue_box_x_max),
            .O_y_min(S_blue_box_y_min),.O_y_max(S_blue_box_y_max),
            .O_center_x(S_blue_cx),.O_center_y(S_blue_cy)
        );
        assign S_pingpong_valid=S_red_box_valid || S_blue_box_valid;
        assign S_pingpong_fresh=S_pingpong_valid;
        assign S_pingpong_class_id=S_blue_box_valid ? 3'd2 : S_red_box_valid ? 3'd3 : 3'd0;
        assign S_pingpong_x_min=S_blue_box_valid ? S_blue_box_x_min : S_red_box_x_min;
        assign S_pingpong_x_max=S_blue_box_valid ? S_blue_box_x_max : S_red_box_x_max;
        assign S_pingpong_y_min=S_blue_box_valid ? S_blue_box_y_min : S_red_box_y_min;
        assign S_pingpong_y_max=S_blue_box_valid ? S_blue_box_y_max : S_red_box_y_max;
        assign S_pingpong_center_x=S_blue_box_valid ? S_blue_cx : S_red_cx;
        assign S_pingpong_center_y=S_blue_box_valid ? S_blue_cy : S_red_cy;
        // The stabilizer latches a published candidate on the next edge.
        // Start BCD conversion one edge later so the OSD shows that target.
        reg coord_publish_d;
        assign S_result_tick=coord_publish_d;
        always @(posedge I_clk or negedge I_rst_n)
            if (!I_rst_n) coord_publish_d <= 1'b0;
            else coord_publish_d <= S_pingpong_publish;
        frame_latency_bcd u_pingpong_x_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(coord_publish_d),
            .O_in_ready(), .I_ms({4'b0,S_pingpong_center_x}),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_pingpong_x_bcd)
        );
        frame_latency_bcd u_pingpong_y_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(coord_publish_d),
            .O_in_ready(), .I_ms({4'b0,S_pingpong_center_y}),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_pingpong_y_bcd)
        );
        frame_latency_bcd u_pingpong_diag_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(S_pingpong_publish),
            .O_in_ready(), .I_ms(S_pingpong_diag_count),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_pingpong_diag_count_bcd)
        );
        frame_latency_bcd u_blue_diag_bcd (
            .I_clk(I_clk), .I_rst(!I_rst_n), .I_in_valid(S_blue_publish),
            .O_in_ready(), .I_ms(S_blue_diag_count),
            .O_out_valid(), .O_out_of_range(), .O_bcd(S_blue_diag_count_bcd)
        );
    end else begin : g_no_pingpong_detect
        assign S_result_tick=0;
        assign S_red_locked=0;assign S_blue_locked=0;assign S_red_fresh=0;assign S_blue_fresh=0;
        assign S_red_box_x_min=0;assign S_red_box_x_max=0;assign S_red_box_y_min=0;assign S_red_box_y_max=0;
        assign S_blue_box_x_min=0;assign S_blue_box_x_max=0;assign S_blue_box_y_min=0;assign S_blue_box_y_max=0;
        assign S_red_cx=0;assign S_red_cy=0;assign S_blue_cx=0;assign S_blue_cy=0;
        assign S_palette_class_id=3'd0;
        assign S_pingpong_valid=1'b0; assign S_pingpong_publish=1'b0;
        assign S_pingpong_fresh=1'b0; assign S_pingpong_raw_valid=1'b0;
        assign S_pingpong_class_id=3'd0;
        assign S_pingpong_raw_class_id=3'd0;
        assign S_pingpong_raw_x_min=12'd0; assign S_pingpong_raw_x_max=12'd0;
        assign S_pingpong_raw_y_min=12'd0; assign S_pingpong_raw_y_max=12'd0;
        assign S_pingpong_raw_center_x=12'd0; assign S_pingpong_raw_center_y=12'd0;
        assign S_pingpong_x_min=12'd0; assign S_pingpong_x_max=12'd0;
        assign S_pingpong_y_min=12'd0; assign S_pingpong_y_max=12'd0;
        assign S_pingpong_center_x=12'd0; assign S_pingpong_center_y=12'd0;
        assign S_pingpong_diag_reason=3'd0; assign S_pingpong_diag_count=16'd0;
        assign S_pingpong_diag_count_bcd=16'd0;
        assign S_blue_diag_count_bcd=16'd0;
        assign S_pingpong_x_bcd=16'd0; assign S_pingpong_y_bcd=16'd0;
        assign S_orange_valid=1'b0; assign S_blue_valid=1'b0; assign S_red_valid=1'b0;
        assign S_orange_diag_count=16'd0; assign S_blue_diag_count=16'd0; assign S_red_diag_count=16'd0;
        assign S_orange_diag_reason=3'd0; assign S_blue_diag_reason=3'd0; assign S_red_diag_reason=3'd0;
    end endgenerate

    generate if (FRAME_LATENCY_ENABLE) begin : g_frame_latency
        frame_latency_inband_display u_frame_latency_inband_display (
            .I_clk(I_clk), .I_rst_n(I_rst_n),
            .I_frame_start(S_user_2d), .I_pixel_valid(S_analysis_valid),
            .I_x(S_x_2d), .I_y(S_y_2d), .I_rgb(S_video_data_2d),
            .I_now_ms(I_latency_now_ms),
            .O_clean_rgb(S_clean_video_data),
            .O_latency_valid(S_latency_valid), .O_latency_bcd(S_latency_bcd),
            .O_source_valid(S_source_valid), .O_source_ms(S_source_ms)
        );
    end else begin : g_no_frame_latency
        assign S_clean_video_data = S_video_data_2d;
        assign S_latency_valid = 1'b0;
        assign S_latency_bcd = 16'd0;
        assign S_source_valid=1'b0;
        assign S_source_ms=32'd0;
    end endgenerate

    wire [15:0] S_gimbal_pan_us,S_gimbal_tilt_us;
    wire [1:0] S_gimbal_state;
    wire [31:0] S_gimbal_pulse_bcd;
    generate if(GIMBAL_DIAG)begin:g_gimbal_decimal
        gf_pulse_pair_bcd u_bcd(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(S_user_2d),
          .I_pan(S_gimbal_pan_us),.I_tilt(S_gimbal_tilt_us),.O_bcd(S_gimbal_pulse_bcd));
    end else assign S_gimbal_pulse_bcd=0;endgenerate
    generate if(GIMBAL_FOLLOW_ENABLE)begin:g_gimbal
        // video_out responds after two clocks. Delay only the integrity-check
        // raster by that read latency; retain the existing detector/OSD pipeline.
        wire guard_valid;
        wire [11:0] guard_x,guard_y;
        if(GIMBAL_READ_LATENCY==0)begin:g_direct_raster
            assign guard_valid=S_analysis_valid;
            assign guard_x=S_x_2d;assign guard_y=S_y_2d;
        end else begin:g_read_raster
            reg [GIMBAL_READ_LATENCY-1:0] window_delay;
            reg [11:0] x_delay[0:GIMBAL_READ_LATENCY-1];
            reg [11:0] y_delay[0:GIMBAL_READ_LATENCY-1];
            integer delay_index;
            always @(posedge I_clk or negedge I_rst_n)begin
                if(!I_rst_n)begin
                    window_delay<=0;
                    for(delay_index=0;delay_index<GIMBAL_READ_LATENCY;delay_index=delay_index+1)begin
                        x_delay[delay_index]<=0;y_delay[delay_index]<=0;
                    end
                end else begin
                    window_delay[0]<=S_window_active_2d;
                    x_delay[0]<=S_x_2d;y_delay[0]<=S_y_2d;
                    for(delay_index=1;delay_index<GIMBAL_READ_LATENCY;delay_index=delay_index+1)begin
                        window_delay[delay_index]<=window_delay[delay_index-1];
                        x_delay[delay_index]<=x_delay[delay_index-1];
                        y_delay[delay_index]<=y_delay[delay_index-1];
                    end
                end
            end
            assign guard_valid=window_delay[GIMBAL_READ_LATENCY-1] &&
                               (!VIDEO_VALID_CHECK || S_video_valid_2d);
            assign guard_x=x_delay[GIMBAL_READ_LATENCY-1];
            assign guard_y=y_delay[GIMBAL_READ_LATENCY-1];
        end
        // OBJECT publishes independent red/blue tracks. The shared OSD center
        // prioritizes blue, so select a class before passing fresh coordinates
        // to the controller. Never use the one-second presentation HOLD boxes.
        wire [2:0] target_class;
        wire selected_publish;
        gf_target_selector #(.LOST_FRAMES(GIMBAL_LOST_FRAMES),.FORCE_CLASS(GIMBAL_TARGET_CLASS))
        u_target_selector(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(S_mode_for_frame==7 && DEBUG_MODE==0),
            .I_publish(S_object_publish_d),.I_red_fresh(S_red_box_valid),.I_blue_fresh(S_blue_box_valid),
            .I_red_x(S_red_cx),.I_blue_x(S_blue_cx),.O_class_id(target_class),.O_publish(selected_publish));
        wire use_red=target_class==3;
        wire use_blue=target_class==2;
        wire target_valid=use_red ? S_red_box_valid : use_blue ? S_blue_box_valid : 1'b0;
        wire target_fresh=use_red ? S_red_fresh : use_blue ? S_blue_fresh : 1'b0;
        wire [11:0] xmin=use_red ? S_red_box_x_min : use_blue ? S_blue_box_x_min : 12'd0;
        wire [11:0] xmax=use_red ? S_red_box_x_max : use_blue ? S_blue_box_x_max : 12'd0;
        wire [11:0] ymin=use_red ? S_red_box_y_min : use_blue ? S_blue_box_y_min : 12'd0;
        wire [11:0] ymax=use_red ? S_red_box_y_max : use_blue ? S_blue_box_y_max : 12'd0;
        wire [11:0] cx=use_red ? S_red_cx : use_blue ? S_blue_cx : 12'd0;
        wire [11:0] cy=use_red ? S_red_cy : use_blue ? S_blue_cy : 12'd0;
        gf_object_gimbal #(.CLK_HZ(GIMBAL_CLK_HZ),.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),
            .REQUIRE_EXTERNAL_ARM(0),.ANTI_HUNT(GIMBAL_ANTI_HUNT),.BOTH_AXES(GIMBAL_BOTH_AXES),
            .PAN_MIN_US(GIMBAL_PAN_MIN_US),.PAN_MAX_US(GIMBAL_PAN_MAX_US),
            .TILT_MIN_US(GIMBAL_TILT_MIN_US),.TILT_MAX_US(GIMBAL_TILT_MAX_US),
            .MAX_STEP_US(8),.CONTROL_INTERVAL_US(20000),
            .TARGET_CLASS(GIMBAL_TARGET_CLASS),.REF_X(GIMBAL_REF_X),.REF_Y(GIMBAL_REF_Y),
            .PAN_REVERSE(GIMBAL_PAN_REVERSE),.TILT_REVERSE(GIMBAL_TILT_REVERSE))u_object_gimbal(
            .I_clk(I_clk),.I_rst_n(I_rst_n),
            .I_mode_active(S_mode_for_frame==3'd7 && DEBUG_MODE==0),.I_arm_async(1'b0),
            .I_frame_start(S_user_2d),.I_pixel_valid(guard_valid),
            .I_x(guard_x),.I_y(guard_y),.I_source_valid(S_source_valid),
            .I_source_ms(S_source_ms),.I_now_ms(I_latency_now_ms),
            .I_publish(selected_publish),.I_target_valid(target_valid),
            .I_target_fresh(target_fresh),.I_class_id(target_class),
            .I_xmin(xmin),.I_xmax(xmax),.I_ymin(ymin),.I_ymax(ymax),
            .I_center_x(cx),.I_center_y(cy),
            .O_pan_pwm(O_gimbal_pan_pwm),.O_tilt_pwm(O_gimbal_tilt_pwm),
            .O_locked(),.O_pan_us(S_gimbal_pan_us),.O_tilt_us(S_gimbal_tilt_us),.O_state(S_gimbal_state),.O_reason());
    end else begin:g_no_gimbal
        assign O_gimbal_pan_pwm=1'b0;assign O_gimbal_tilt_pwm=1'b0;
        assign S_gimbal_pan_us=1500;assign S_gimbal_tilt_us=1500;assign S_gimbal_state=0;
    end endgenerate

    anlogic_osd_overlay #(.GIMBAL_DIAG(GIMBAL_DIAG),.CAMERA_CALIBRATION(CAMERA_CALIBRATION),
                         .AE_DISPLAY(AE_DISPLAY),.RGB_PROBE(OBJECT_RGB_PROBE),
                         .PROBE_X(IMG_WIDTH/2),.PROBE_Y(IMG_HEIGHT/2),
                         .PIPELINED(FONT_PIPE),.OBJECT_PAIR(PINGPONG_DETECT_ENABLE),
                         .TRACK_EVENT_COUNT(TRACK_EVENT_COUNT),.COUNT_X(IMG_WIDTH/2-66),
                         .FRAME_WIDTH(IMG_WIDTH),.FRAME_HEIGHT(IMG_HEIGHT)) u_anlogic_osd_overlay (
        .I_clk(I_clk),.I_rst_n(I_rst_n),
        .I_enable ( 1'b1             ),
        .I_mode   ( S_display_mode   ),
        .I_camera_status(S_camera_status_frame),
        .I_ae_auto(S_ae_auto_frame),
        .I_ae_meter(S_ae_meter_frame),.I_gimbal_bcd(S_gimbal_pulse_bcd),.I_gimbal_state(S_gimbal_state),
        .I_wb_mode(S_wb_mode_frame),
        .I_wb_calibrated(S_wb_calibrated_frame),
        .I_x      ( S_disp_x         ),
        .I_y      ( S_disp_y         ),
        .I_rgb    ( S_display_video_data ),
        .I_detect_valid(S_osd_detect_valid), .I_detect_class(S_osd_detect_class),
        .I_detect_x_min(S_osd_x_min), .I_detect_x_max(S_osd_x_max),
        .I_detect_y_min(S_osd_y_min), .I_detect_y_max(S_osd_y_max),
        .I_center_x_bcd(S_osd_center_x_bcd), .I_center_y_bcd(S_osd_center_y_bcd),
        .I_track_diag_reason(S_pingpong_mode ? S_pingpong_diag_reason : S_motion_diag_reason),
        .I_track_diag_count_bcd(S_pingpong_mode ? S_pingpong_diag_count_bcd : S_motion_diag_count_bcd),
        .I_track_frame_bcd(S_motion_frame_bcd),
        .I_object_diag_color(S_diag_choice),
        .I_blue_count_bcd(S_blue_diag_count_bcd),
        .I_probe_rgb(S_probe_rgb),.I_probe_valid(S_probe_valid),
        .I_event_count_bcd(S_throw_count_bcd),
        .I_object_coordinates_bcd(S_object_coord_bcd),
        .I_object_count(S_object_display_count),.I_red_valid(S_red_display_valid),.I_blue_valid(S_blue_display_valid),
        .I_red_fresh(S_red_box_valid),.I_blue_fresh(S_blue_box_valid),
        .I_red_x_min(S_red_display_x_min),.I_red_x_max(S_red_display_x_max),
        .I_red_y_min(S_red_display_y_min),.I_red_y_max(S_red_display_y_max),
        .I_blue_x_min(S_blue_display_x_min),.I_blue_x_max(S_blue_display_x_max),
        .I_blue_y_min(S_blue_display_y_min),.I_blue_y_max(S_blue_display_y_max),
        .O_rgb    ( S_osd_video_data )
    );

    frame_latency_osd #(.X_ORIGIN(1000), .Y_ORIGIN(24))
    u_frame_latency_osd (
        .I_enable(FRAME_LATENCY_ENABLE != 0), .I_valid(S_latency_valid),
        .I_ms_bcd(S_latency_bcd), .I_x(S_disp_x), .I_y(S_disp_y),
        .I_rgb(S_display_video_data), .O_rgb(S_latency_video_data)
    );

    // Render the two OSD blocks in parallel. Cascading their combinational
    // font logic made the HDMI output path miss setup timing in synthesis.
    assign S_composed_video_data = FONT_PIPE ?
        (S_font_latency_sel ? S_font_latency_rgb : S_osd_video_data) :
        FRAME_LATENCY_ENABLE &&
        S_disp_x >= 12'd1000 && S_disp_x < 12'd1132 &&
        S_disp_y >= 12'd24 && S_disp_y < 12'd46
            ? S_latency_video_data : S_osd_video_data;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else begin
            if(I_video_user)
                S_y <= 12'd0;
            else if(I_video_de && I_video_last)
                S_y <= S_y + 12'd1;

            if(!I_video_de || I_video_last)
                S_x <= 12'd0;
            else
                S_x <= S_x + 12'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            S_window_active_1d <= 1'b0;
            S_window_active_2d <= 1'b0;
            S_video_valid_1d <= 1'b0;
            S_video_valid_2d <= 1'b0;
            S_x_1d             <= 12'd0;
            S_x_2d             <= 12'd0;
            S_y_1d             <= 12'd0;
            S_y_2d             <= 12'd0;
            S_user_1d <= 0; S_user_2d <= 0; S_display_mode <= 3'd1;
            S_camera_status_frame <= 9'h004;
            S_wb_mode_frame <= 0;
            S_wb_calibrated_frame <= 0;
            S_ae_auto_frame <= 0;
            S_ae_meter_frame <= 0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= FONT_PIPE ? S_font_vs : S_disp_vsync;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= FONT_PIPE ? S_font_hs : S_disp_hsync;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            S_window_active_1d <= S_window_active;
            S_window_active_2d <= S_window_active_1d;
            S_x_1d             <= S_x;
            S_x_2d             <= S_x_1d;
            S_y_1d             <= S_raster_y;
            S_y_2d             <= S_y_1d;
            S_user_1d <= I_video_user;
            S_user_2d <= S_user_1d;
            if(S_user_2d) S_display_mode <= S_requested_mode;
            if(S_user_2d) S_camera_status_frame <= I_camera_status;
            if(S_user_2d) S_wb_mode_frame <= I_wb_mode;
            if(S_user_2d) S_wb_calibrated_frame <= I_wb_calibrated;
            if(S_user_2d) begin
                S_ae_auto_frame <= I_ae_auto;
                S_ae_meter_frame <= I_ae_meter;
            end
            O_hdmi_de        <= FONT_PIPE ? S_font_de : S_disp_de;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;
            S_video_valid_1d <= I_video_rd_valid;
            S_video_valid_2d <= S_video_valid_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 4) begin
                // Full-screen MIPI receive diagnostic.  The status is
                // generated and synchronized in design_top_wrapper.
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'd0: O_hdmi_data <= 24'h000080; // blue: sensor config incomplete
                        4'd1: O_hdmi_data <= 24'h000000; // black: no MIPI HS traffic
                        4'd2: O_hdmi_data <= 24'h00ffff; // cyan: HS present, no complete frame
                        4'd3: O_hdmi_data <= 24'hff0000; // red: D-PHY lane error
                        4'd4: O_hdmi_data <= 24'hff00ff; // magenta: RAW10 line length error
                        4'd5: O_hdmi_data <= 24'hffff00; // yellow: not 720 lines per frame
                        4'd6: O_hdmi_data <= 24'h00ff00; // green: MIPI frame structure passed
                        4'd7: O_hdmi_data <= 24'hff8000; // orange: sensor I2C transaction error
                        default: O_hdmi_data <= 24'hffffff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if (OBJECT_MASK_PREVIEW && S_display_mode==3'd7 &&
                    S_output_active && S_output_warm)
                    O_hdmi_data <= 24'hff00ff;
                else if (OBJECT_MASK_PREVIEW && S_display_mode==3'd7 &&
                         S_output_active && S_output_blue)
                    O_hdmi_data <= 24'h00ffff;
                else
                    O_hdmi_data <= S_output_active ? S_composed_video_data : 24'd0;
            end
        end
    end

endmodule

// Convert the two retained display boxes, not the selected primary target.
// Bit order: {blue Y, blue X, red Y, red X}, four decimal digits each.
// Delay update one edge because object_display_hold captures on the same tick.
module object_pair_coordinates(
    input wire clk,rst_n,update,
    input wire [11:0] red_x_min,red_x_max,red_y_min,red_y_max,
    input wire [11:0] blue_x_min,blue_x_max,blue_y_min,blue_y_max,
    output reg [63:0] coordinates_bcd,
    output reg publish
);
    reg start;
    wire [12:0] rx={1'b0,red_x_min}+{1'b0,red_x_max};
    wire [12:0] ry={1'b0,red_y_min}+{1'b0,red_y_max};
    wire [12:0] bx={1'b0,blue_x_min}+{1'b0,blue_x_max};
    wire [12:0] by={1'b0,blue_y_min}+{1'b0,blue_y_max};
    wire [63:0] binary_coordinates={{4'b0,by[12:1]},{4'b0,bx[12:1]},
                                  {4'b0,ry[12:1]},{4'b0,rx[12:1]}};
    wire [63:0] converted;
    wire [3:0] done;
    generate for(genvar axis=0;axis<4;axis=axis+1)begin : g_decimal
        frame_latency_bcd u_bcd(.I_clk(clk),.I_rst(!rst_n),.I_in_valid(start),.O_in_ready(),
            .I_ms(binary_coordinates[axis*16+:16]),.O_out_valid(done[axis]),
            .O_out_of_range(),.O_bcd(converted[axis*16+:16]));
    end endgenerate
    always @(posedge clk)begin
        if(!rst_n)begin start<=0;coordinates_bcd<=0;publish<=0;end
        else begin
            start<=update;publish<=&done;
            if(&done)coordinates_bcd<=converted;
        end
    end
endmodule

// Mean of the exact 8x8 detector-input patch. Publish only a complete patch;
// missing/invalid pixels never silently contribute black samples.
module object_rgb_probe #(parameter integer X=640,Y=360)(
    input wire clk,rst_n,frame_start,valid,
    input wire [11:0] x,y,
    input wire [23:0] rgb,
    output reg [23:0] out_rgb,
    output reg out_valid
);
    reg [13:0] sr,sg,sb;
    reg [6:0] count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sr<=0;sg<=0;sb<=0;count<=0;out_rgb<=0;out_valid<=0; end
        else if(frame_start) begin
            out_valid<=count==64;
            if(count==64) out_rgb<={sr[13:6],sg[13:6],sb[13:6]};
            sr<=0;sg<=0;sb<=0;count<=0;
        end else if(valid && x>=X && x<X+8 && y>=Y && y<Y+8 && count<64) begin
            sr<=sr+{6'b0,rgb[23:16]};
            sg<=sg+{6'b0,rgb[15:8]};
            sb<=sb+{6'b0,rgb[7:0]};
            count<=count+1'b1;
        end
    end
endmodule

// One-pixel-per-clock chroma denoising for the display branch. The causal 3x3
// neighborhood uses this pixel, the two pixels to its left, and the matching
// positions in the two previous lines.  Keeping the output at the same x/y
// uses four clocks of latency. This causal neighborhood is asymmetric;
// a centered neighborhood would require one line of video/control delay.
module display_chroma_denoise #(
    parameter integer IMG_WIDTH = 1280
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [23:0] I_rgb,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire        I_window_active,
    input  wire        I_enable,
    input  wire        I_vsync,
    input  wire        I_hsync,
    input  wire        I_de,
    output wire [23:0] O_rgb,
    output wire [11:0] O_x,
    output wire [11:0] O_y,
    output wire        O_window_active,
    output wire        O_vsync,
    output wire        O_hsync,
    output wire        O_de
);
    reg [23:0] line0 [0:IMG_WIDTH-1];
    reg [23:0] line1 [0:IMG_WIDTH-1];
    reg [23:0] row0_cur, row0_left1, row0_left2;
    reg [23:0] row1_cur, row1_left1, row1_left2;
    reg [23:0] row2_cur, row2_left1, row2_left2;

    // No reset on RAM: top/left neighbor masks hide unwritten locations.
    always @(posedge I_clk) begin
        if (I_rst_n && I_window_active && I_x < IMG_WIDTH) begin
            row0_left2 <= row0_left1;
            row0_left1 <= row0_cur;
            row0_cur   <= I_rgb;
            row1_left2 <= row1_left1;
            row1_left1 <= row1_cur;
            row1_cur   <= line0[I_x];
            row2_left2 <= row2_left1;
            row2_left1 <= row2_cur;
            row2_cur   <= line1[I_x];
            line1[I_x] <= line0[I_x];
            line0[I_x] <= I_rgb;
        end
    end

    function [7:0] luma8;
        input [23:0] rgb;
        reg [9:0] value;
        begin
            value = {2'b0,rgb[23:16]} + {1'b0,rgb[15:8],1'b0} +
                    {2'b0,rgb[7:0]};
            luma8 = value[9:2];
        end
    endfunction

    function similar_luma;
        input [23:0] rgb;
        input [7:0] center_y, threshold;
        reg [7:0] neighbor_y;
        reg [8:0] difference;
        begin
            neighbor_y = luma8(rgb);
            difference = neighbor_y >= center_y ?
                         {1'b0,neighbor_y} - {1'b0,center_y} :
                         {1'b0,center_y} - {1'b0,neighbor_y};
            similar_luma = difference <= {1'b0,threshold};
        end
    endfunction

    function signed [9:0] red_chroma_delta;
        input [23:0] neighbor, center;
        reg signed [8:0] n_chroma, c_chroma;
        begin
            n_chroma = $signed({1'b0,neighbor[23:16]}) -
                       $signed({1'b0,neighbor[15:8]});
            c_chroma = $signed({1'b0,center[23:16]}) -
                       $signed({1'b0,center[15:8]});
            red_chroma_delta = $signed({n_chroma[8],n_chroma}) -
                               $signed({c_chroma[8],c_chroma});
        end
    endfunction

    function signed [9:0] blue_chroma_delta;
        input [23:0] neighbor, center;
        reg signed [8:0] n_chroma, c_chroma;
        begin
            n_chroma = $signed({1'b0,neighbor[7:0]}) -
                       $signed({1'b0,neighbor[15:8]});
            c_chroma = $signed({1'b0,center[7:0]}) -
                       $signed({1'b0,center[15:8]});
            blue_chroma_delta = $signed({n_chroma[8],n_chroma}) -
                                $signed({c_chroma[8],c_chroma});
        end
    endfunction

    function [7:0] clip8;
        input signed [13:0] value;
        begin
            if (value < 0) clip8 = 8'd0;
            else if (value > 14'sd255) clip8 = 8'd255;
            else clip8 = value[7:0];
        end
    endfunction

    reg [23:0] s0_rgb, s1_rgb, s2_rgb, s3_rgb;
    reg [11:0] s0_x,s1_x,s2_x,s3_x;
    reg [11:0] s0_y,s1_y,s2_y,s3_y;
    reg s0_de,s0_hs,s0_vs,s0_window,s0_enable;
    reg s1_de,s1_hs,s1_vs,s1_window,s1_enable;
    reg s2_de,s2_hs,s2_vs,s2_window,s2_enable;
    reg s3_de,s3_hs,s3_vs,s3_window,s3_enable;
    wire [23:0] neighbor [0:7];
    wire [7:0] present, accepted;
    wire [7:0] center_y = luma8(row0_cur);
    wire [7:0] threshold = center_y < 8'd64 ? 8'd24 : 8'd12;
    assign neighbor[0]=row0_left1;
    assign neighbor[1]=row0_left2;
    assign neighbor[2]=row1_cur;
    assign neighbor[3]=row1_left1;
    assign neighbor[4]=row1_left2;
    assign neighbor[5]=row2_cur;
    assign neighbor[6]=row2_left1;
    assign neighbor[7]=row2_left2;
    assign present[0]=(s0_x >= 12'd1);
    assign present[1]=(s0_x >= 12'd2);
    assign present[2]=(s0_y >= 12'd1);
    assign present[3]=(s0_x >= 12'd1 && s0_y >= 12'd1);
    assign present[4]=(s0_x >= 12'd2 && s0_y >= 12'd1);
    assign present[5]=(s0_y >= 12'd2);
    assign present[6]=(s0_x >= 12'd1 && s0_y >= 12'd2);
    assign present[7]=(s0_x >= 12'd2 && s0_y >= 12'd2);

    genvar gi;
    generate for (gi=0;gi<8;gi=gi+1) begin : g_neighbor_check
        assign accepted[gi] = present[gi] &&
                              similar_luma(neighbor[gi],center_y,threshold);
    end endgenerate

    reg signed [9:0] term_r [0:7], term_b [0:7];
    reg signed [10:0] pair_r [0:3], pair_b [0:3];
    wire signed [12:0] sum_r =
        {{2{pair_r[0][10]}},pair_r[0]} +
        {{2{pair_r[1][10]}},pair_r[1]} +
        {{2{pair_r[2][10]}},pair_r[2]} +
        {{2{pair_r[3][10]}},pair_r[3]};
    wire signed [12:0] sum_b =
        {{2{pair_b[0][10]}},pair_b[0]} +
        {{2{pair_b[1][10]}},pair_b[1]} +
        {{2{pair_b[2][10]}},pair_b[2]} +
        {{2{pair_b[3][10]}},pair_b[3]};
    wire signed [13:0] red_value =
        $signed({6'b0,s2_rgb[23:16]}) +
        ($signed({sum_r[12],sum_r}) >>> 4);
    wire signed [13:0] blue_value =
        $signed({6'b0,s2_rgb[7:0]}) +
        ($signed({sum_b[12],sum_b}) >>> 4);

    integer i;
    integer j;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s0_rgb<=0; s1_rgb<=0; s2_rgb<=0; s3_rgb<=0;
            s0_x<=0; s1_x<=0; s2_x<=0; s3_x<=0;
            s0_y<=0; s1_y<=0; s2_y<=0; s3_y<=0;
            s0_de<=0; s1_de<=0; s2_de<=0; s3_de<=0;
            s0_hs<=0; s1_hs<=0; s2_hs<=0; s3_hs<=0;
            s0_vs<=0; s1_vs<=0; s2_vs<=0; s3_vs<=0;
            s0_window<=0; s1_window<=0; s2_window<=0; s3_window<=0;
            s0_enable<=0; s1_enable<=0; s2_enable<=0; s3_enable<=0;
            for (i=0;i<8;i=i+1) begin term_r[i]<=0; term_b[i]<=0; end
            for (j=0;j<4;j=j+1) begin pair_r[j]<=0; pair_b[j]<=0; end
        end else begin
            s0_rgb<=I_rgb; s1_rgb<=s0_rgb; s2_rgb<=s1_rgb;
            s3_rgb<=s2_enable ?
                    {clip8(red_value),s2_rgb[15:8],clip8(blue_value)} :
                    s2_rgb;
            s0_x<=I_x; s1_x<=s0_x; s2_x<=s1_x; s3_x<=s2_x;
            s0_y<=I_y; s1_y<=s0_y; s2_y<=s1_y; s3_y<=s2_y;
            s0_de<=I_de; s1_de<=s0_de; s2_de<=s1_de; s3_de<=s2_de;
            s0_hs<=I_hsync; s1_hs<=s0_hs; s2_hs<=s1_hs; s3_hs<=s2_hs;
            s0_vs<=I_vsync; s1_vs<=s0_vs; s2_vs<=s1_vs; s3_vs<=s2_vs;
            s0_window<=I_window_active;
            s1_window<=s0_window; s2_window<=s1_window; s3_window<=s2_window;
            s0_enable<=I_enable && I_window_active;
            s1_enable<=s0_enable; s2_enable<=s1_enable; s3_enable<=s2_enable;
            for (i=0;i<8;i=i+1) begin
                term_r[i] <= s0_enable && accepted[i] ?
                             red_chroma_delta(neighbor[i],row0_cur) : 10'sd0;
                term_b[i] <= s0_enable && accepted[i] ?
                             blue_chroma_delta(neighbor[i],row0_cur) : 10'sd0;
            end
            for (j=0;j<4;j=j+1) begin
                pair_r[j] <= $signed({term_r[2*j][9],term_r[2*j]}) +
                             $signed({term_r[2*j+1][9],term_r[2*j+1]});
                pair_b[j] <= $signed({term_b[2*j][9],term_b[2*j]}) +
                             $signed({term_b[2*j+1][9],term_b[2*j+1]});
            end
        end
    end
    assign O_rgb=s3_rgb;
    assign O_x=s3_x; assign O_y=s3_y;
    assign O_window_active=s3_window;
    assign O_vsync=s3_vs; assign O_hsync=s3_hs;
    assign O_de=s3_de;
endmodule

// Presentation memory only: never feeds a retained box back to the detector
// or current measurement/servo coordinates. At 720p60, 60 missed frames ~=1s.
module object_display_hold #(parameter integer HOLD_FRAMES=60)(
    input wire clk,rst_n,enable,frame_tick,fresh,
    input wire [11:0] x_min,x_max,y_min,y_max,
    output reg valid,
    output reg [11:0] out_x_min,out_x_max,out_y_min,out_y_max
);
    localparam integer AGE_W=HOLD_FRAMES<2 ? 1 : $clog2(HOLD_FRAMES+1);
    reg [AGE_W-1:0] remaining;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            valid<=0;remaining<=0;out_x_min<=0;out_x_max<=0;out_y_min<=0;out_y_max<=0;
        end else if(!enable)begin valid<=0;remaining<=0;end
        else if(frame_tick)begin
            if(fresh)begin
                valid<=1;remaining<=HOLD_FRAMES;
                out_x_min<=x_min;out_x_max<=x_max;out_y_min<=y_min;out_y_max<=y_max;
            end else if(remaining>1)begin remaining<=remaining-1'b1;end
            else begin valid<=0;remaining<=0;end
        end
    end
endmodule
