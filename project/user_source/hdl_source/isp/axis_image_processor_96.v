// Six-mode image processor for the SC520 four-pixel RGB888 stream.
// Mode changes and threshold changes become active only on a frame-start beat,
// so a button press cannot split one frame between two configurations.
//   mode 0: color
//   mode 1: BT.709 gray
//   mode 2: binary image
//   mode 3: Gaussian + Scharr edge image
//   mode 4: quantized color with black Canny outlines
//   mode 5: color passthrough for HDMI-domain detection and annotation
module axis_image_processor_96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire [2:0]  I_mode_request,
    input  wire [7:0]  I_binary_threshold_request,
    input  wire [10:0] I_edge_threshold_request,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata,
    output wire        O_tvalid,
    output wire [2:0]  O_mode_active,
    output wire [7:0]  O_binary_threshold_active,
    output wire [10:0] O_edge_threshold_active
);

    reg [2:0] S_mode_active;
    // The existing 8-bit binary control is retained, but is now interpreted
    // around 128 as a signed manual offset to the frame-based Otsu result.
    reg [7:0] S_binary_threshold_active;
    reg [10:0] S_edge_threshold_active;

    // SC520 asserts tuser during the frame blanking interval, before the first
    // valid pixel beat.  Do not qualify it with tvalid: doing so loses the
    // frame boundary, prevents mode updates, and stops video_in from rotating
    // its frame buffers.
    wire S_frame_start = I_tuser;
    wire [2:0] S_mode_for_beat = S_frame_start ? I_mode_request : S_mode_active;
    wire [7:0] S_binary_threshold_for_beat = S_frame_start
                                           ? I_binary_threshold_request
                                           : S_binary_threshold_active;
    wire [10:0] S_edge_threshold_for_beat = S_frame_start
                                          ? I_edge_threshold_request
                                          : S_edge_threshold_active;

    assign O_mode_active             = S_mode_active;
    wire [7:0] S_otsu_threshold_active;
    // Otsu (0..255) plus the manual offset (-128..127) needs ten signed bits:
    // its full range is -128..382.  Nine bits wrap before saturation.
    wire signed [9:0] S_binary_threshold_sum =
        $signed({1'b0,S_otsu_threshold_active})
      + $signed({1'b0,S_binary_threshold_active}) - 10'sd128;
    wire [7:0] S_binary_threshold_effective = S_binary_threshold_sum[9]
                                               ? 8'd0
                                               : (S_binary_threshold_sum > 10'sd255)
                                                 ? 8'd255
                                                 : S_binary_threshold_sum[7:0];
    wire signed [9:0] S_binary_threshold_beat_sum =
        $signed({1'b0,S_otsu_threshold_active})
      + $signed({1'b0,S_binary_threshold_for_beat}) - 10'sd128;
    wire [7:0] S_binary_threshold_for_compare = S_binary_threshold_beat_sum[9]
                                                 ? 8'd0
                                                 : (S_binary_threshold_beat_sum > 10'sd255)
                                                   ? 8'd255
                                                   : S_binary_threshold_beat_sum[7:0];

    assign O_binary_threshold_active = S_binary_threshold_effective;
    assign O_edge_threshold_active   = S_edge_threshold_active;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_mode_active             <= 3'd1;
            S_binary_threshold_active <= 8'd128;
            S_edge_threshold_active   <= 11'd256;
        end else if (S_frame_start) begin
            S_mode_active             <= I_mode_request;
            S_binary_threshold_active <= I_binary_threshold_request;
            S_edge_threshold_active   <= I_edge_threshold_request;
        end
    end

    wire [95:0] S_gray;
    rgb96_to_gray_bt709 u_rgb96_to_gray_bt709 (
        .I_rgb  (I_tdata),
        .O_gray (S_gray)
    );

    otsu_threshold_96 u_otsu_threshold_96 (
        .I_clk              (I_clk),
        .I_rst_n            (I_rst_n),
        .I_frame_start      (S_frame_start),
        .I_valid            (I_tvalid),
        .I_gray             (S_gray),
        .O_threshold_active (S_otsu_threshold_active),
        .O_threshold_result (),
        .O_result_valid     ()
    );

    // Otsu's background class includes the selected threshold bin (<= T).
    wire [23:0] S_binary0 = (S_gray[23:16] > S_binary_threshold_for_compare)
                           ? 24'hffffff : 24'h000000;
    wire [23:0] S_binary1 = (S_gray[47:40] > S_binary_threshold_for_compare)
                           ? 24'hffffff : 24'h000000;
    wire [23:0] S_binary2 = (S_gray[71:64] > S_binary_threshold_for_compare)
                           ? 24'hffffff : 24'h000000;
    wire [23:0] S_binary3 = (S_gray[95:88] > S_binary_threshold_for_compare)
                           ? 24'hffffff : 24'h000000;
    wire [95:0] S_binary = {S_binary3,S_binary2,S_binary1,S_binary0};

    // Delay the three bypass paths by the same ten clocks used by the
    // Gaussian-plus-Scharr-NMS-hysteresis branch.  The final three stages
    // match the streaming 3x3 hysteresis line-buffer pipeline.
    reg [95:0] S_color_1d;
    reg [95:0] S_gray_1d;
    reg [95:0] S_binary_1d;
    reg [2:0]  S_mode_1d;
    reg [95:0] S_color_2d;
    reg [95:0] S_gray_2d;
    reg [95:0] S_binary_2d;
    reg [2:0]  S_mode_2d;
    reg [95:0] S_color_3d;
    reg [95:0] S_gray_3d;
    reg [95:0] S_binary_3d;
    reg [2:0]  S_mode_3d;
    reg [95:0] S_color_4d;
    reg [95:0] S_gray_4d;
    reg [95:0] S_binary_4d;
    reg [2:0]  S_mode_4d;
    reg [95:0] S_color_5d;
    reg [95:0] S_gray_5d;
    reg [95:0] S_binary_5d;
    reg [2:0]  S_mode_5d;
    reg [95:0] S_color_6d;
    reg [95:0] S_gray_6d;
    reg [95:0] S_binary_6d;
    reg [2:0]  S_mode_6d;
    reg [95:0] S_color_7d;
    reg [95:0] S_gray_7d;
    reg [95:0] S_binary_7d;
    reg [2:0]  S_mode_7d;
    reg [95:0] S_color_8d;
    reg [95:0] S_gray_8d;
    reg [95:0] S_binary_8d;
    reg [2:0]  S_mode_8d;
    reg [95:0] S_color_9d;
    reg [95:0] S_gray_9d;
    reg [95:0] S_binary_9d;
    reg [2:0]  S_mode_9d;
    reg [95:0] S_color_10d;
    reg [95:0] S_gray_10d;
    reg [95:0] S_binary_10d;
    reg [2:0]  S_mode_10d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_color_1d  <= 96'd0;
            S_gray_1d   <= 96'd0;
            S_binary_1d <= 96'd0;
            S_mode_1d   <= 3'd1;
            S_color_2d  <= 96'd0;
            S_gray_2d   <= 96'd0;
            S_binary_2d <= 96'd0;
            S_mode_2d   <= 3'd1;
            S_color_3d  <= 96'd0;
            S_gray_3d   <= 96'd0;
            S_binary_3d <= 96'd0;
            S_mode_3d   <= 3'd1;
            S_color_4d  <= 96'd0;
            S_gray_4d   <= 96'd0;
            S_binary_4d <= 96'd0;
            S_mode_4d   <= 3'd1;
            S_color_5d  <= 96'd0;
            S_gray_5d   <= 96'd0;
            S_binary_5d <= 96'd0;
            S_mode_5d   <= 3'd1;
            S_color_6d  <= 96'd0;
            S_gray_6d   <= 96'd0;
            S_binary_6d <= 96'd0;
            S_mode_6d   <= 3'd1;
            S_color_7d  <= 96'd0;
            S_gray_7d   <= 96'd0;
            S_binary_7d <= 96'd0;
            S_mode_7d   <= 3'd1;
            S_color_8d  <= 96'd0;
            S_gray_8d   <= 96'd0;
            S_binary_8d <= 96'd0;
            S_mode_8d   <= 3'd1;
            S_color_9d  <= 96'd0;
            S_gray_9d   <= 96'd0;
            S_binary_9d <= 96'd0;
            S_mode_9d   <= 3'd1;
            S_color_10d  <= 96'd0;
            S_gray_10d   <= 96'd0;
            S_binary_10d <= 96'd0;
            S_mode_10d   <= 3'd1;
        end else begin
            if(I_tvalid) begin
                S_color_1d  <= I_tdata;
                S_gray_1d   <= S_gray;
                S_binary_1d <= S_binary;
                S_mode_1d   <= S_mode_for_beat;
            end
            S_color_2d  <= S_color_1d;
            S_gray_2d   <= S_gray_1d;
            S_binary_2d <= S_binary_1d;
            S_mode_2d   <= S_mode_1d;
            S_color_3d  <= S_color_2d;
            S_gray_3d   <= S_gray_2d;
            S_binary_3d <= S_binary_2d;
            S_mode_3d   <= S_mode_2d;
            S_color_4d  <= S_color_3d;
            S_gray_4d   <= S_gray_3d;
            S_binary_4d <= S_binary_3d;
            S_mode_4d   <= S_mode_3d;
            S_color_5d  <= S_color_4d;
            S_gray_5d   <= S_gray_4d;
            S_binary_5d <= S_binary_4d;
            S_mode_5d   <= S_mode_4d;
            S_color_6d  <= S_color_5d;
            S_gray_6d   <= S_gray_5d;
            S_binary_6d <= S_binary_5d;
            S_mode_6d   <= S_mode_5d;
            S_color_7d  <= S_color_6d;
            S_gray_7d   <= S_gray_6d;
            S_binary_7d <= S_binary_6d;
            S_mode_7d   <= S_mode_6d;
            S_color_8d  <= S_color_7d;
            S_gray_8d   <= S_gray_7d;
            S_binary_8d <= S_binary_7d;
            S_mode_8d   <= S_mode_7d;
            S_color_9d  <= S_color_8d;
            S_gray_9d   <= S_gray_8d;
            S_binary_9d <= S_binary_8d;
            S_mode_9d   <= S_mode_8d;
            S_color_10d  <= S_color_9d;
            S_gray_10d   <= S_gray_9d;
            S_binary_10d <= S_binary_9d;
            S_mode_10d   <= S_mode_9d;
        end
    end

    wire [95:0] S_gaussian;
    wire S_gaussian_last;
    wire S_gaussian_user;
    wire S_gaussian_valid;
    wire S_gaussian_ready;

    gaussian_blur_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_gaussian_blur_96 (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tlast  (I_tlast),
        .I_tuser  (I_tuser),
        .I_tdata  (S_gray),
        .I_tvalid (I_tvalid),
        .I_tready (1'b1),
        .O_tlast  (S_gaussian_last),
        .O_tuser  (S_gaussian_user),
        .O_tdata  (S_gaussian),
        .O_tvalid (S_gaussian_valid),
        .O_tready (S_gaussian_ready)
    );

    wire [95:0] S_edge;
    wire S_edge_last;
    wire S_edge_user;
    wire S_edge_valid;
    wire S_edge_ready;

    edge_detect_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .INPUT_IS_GRAY (1)
    ) u_edge_detect_96 (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_tlast     (S_gaussian_last),
        .I_tuser     (S_gaussian_user),
        .I_tdata     (S_gaussian),
        .I_tvalid    (S_gaussian_valid),
        .I_tready    (1'b1),
        .I_threshold (S_edge_threshold_for_beat),
        .O_tlast     (S_edge_last),
        .O_tuser     (S_edge_user),
        .O_tdata     (S_edge),
        .O_tvalid    (S_edge_valid),
        .O_tready    (S_edge_ready)
    );

    wire [95:0] S_cartoon;
    cartoon_quantize_96 u_cartoon_quantize_96 (
        .I_color   (S_color_10d),
        .I_edge    (S_edge),
        .O_cartoon (S_cartoon)
    );

    video_mode_mux_96 u_video_mode_mux_96 (
        .I_mode   (S_mode_10d),
        .I_color  (S_color_10d),
        .I_gray   (S_gray_10d),
        .I_binary (S_binary_10d),
        .I_edge   (S_edge),
        .I_cartoon(S_cartoon),
        .O_video  (O_tdata)
    );

    assign O_tlast  = S_edge_last;
    assign O_tuser  = S_edge_user;
    assign O_tvalid = S_edge_valid;

endmodule
