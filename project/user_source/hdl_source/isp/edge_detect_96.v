// Four-sample-per-beat Scharr edge detector for the SC520 video stream.
// Every RGB888 input pixel is converted to BT.709 luminance and receives an
// independent Scharr result; horizontal resolution is not reduced.
module edge_detect_96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer INPUT_IS_GRAY = 0
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tready,
    input  wire [10:0] I_threshold,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata,
    output wire        O_tvalid,
    output wire        O_tready
);

    // Use the same ITU-R BT.709 luminance conversion as the gray/binary paths.
    // Keeping this conversion local makes the first correction independent of
    // the line-buffer and Scharr pipeline changes that follow.
    wire [95:0] gray_converted;
    rgb96_to_gray_bt709 u_edge_rgb96_to_gray_bt709 (
        .I_rgb  (I_tdata),
        .O_gray (gray_converted)
    );
    wire [95:0] gray_bt709 = INPUT_IS_GRAY ? I_tdata : gray_converted;

    // Register BT.709 luminance before the line-buffer/Scharr stage.  This
    // keeps the coefficient multiply-add and Scharr adders in separate clock
    // periods at the 103 MHz MIPI byte clock.
    wire [31:0] gray_word_comb = {gray_bt709[95:88], gray_bt709[71:64],
                                  gray_bt709[47:40], gray_bt709[23:16]};
    reg  [31:0] gray_word_s0;
    reg  [10:0] threshold_s0;
    reg         valid_s0, last_s0, user_s0;

    wire [7:0] gray0 = gray_word_s0[31:24];
    wire [7:0] gray1 = gray_word_s0[23:16];
    wire [7:0] gray2 = gray_word_s0[15:8];
    wire [7:0] gray3 = gray_word_s0[7:0];

    localparam integer IMG_WIDTH_GRP = IMG_WIDTH / 4;

    (* ram_style = "block" *) reg [31:0] line1_mem [0:IMG_WIDTH_GRP-1];
    (* ram_style = "block" *) reg [31:0] line2_mem [0:IMG_WIDTH_GRP-1];

    reg [10:0] x_grp;
    reg [10:0] y_pos;

    wire [31:0] top_word = line2_mem[x_grp];
    wire [31:0] mid_word = line1_mem[x_grp];
    wire [31:0] bot_word = {gray0,gray1,gray2,gray3};

    wire [7:0] top0 = top_word[31:24];
    wire [7:0] top1 = top_word[23:16];
    wire [7:0] top2 = top_word[15:8];
    wire [7:0] top3 = top_word[7:0];
    wire [7:0] mid0 = mid_word[31:24];
    wire [7:0] mid1 = mid_word[23:16];
    wire [7:0] mid2 = mid_word[15:8];
    wire [7:0] mid3 = mid_word[7:0];

    reg [7:0] top_prev2, top_prev1;
    reg [7:0] mid_prev2, mid_prev1;
    reg [7:0] bot_prev2, bot_prev1;

    // Scharr kernels:
    //   Gx = [-3 0 3; -10 0 10; -3 0 3]
    //   Gy = [ 3 10 3; 0 0 0; -3 -10 -3]
    // The largest signed component is 16*255=4080, which fits in 13 bits.
    function signed [12:0] scharr_gx;
        input [7:0] a00, a01, a02;
        input [7:0] a10, a11, a12;
        input [7:0] a20, a21, a22;
        reg signed [12:0] left_f;
        reg signed [12:0] right_f;
        begin
            left_f  = ($signed({5'b0,a00}) <<< 1) + $signed({5'b0,a00})
                    + ($signed({5'b0,a10}) <<< 3) + ($signed({5'b0,a10}) <<< 1)
                    + ($signed({5'b0,a20}) <<< 1) + $signed({5'b0,a20});
            right_f = ($signed({5'b0,a02}) <<< 1) + $signed({5'b0,a02})
                    + ($signed({5'b0,a12}) <<< 3) + ($signed({5'b0,a12}) <<< 1)
                    + ($signed({5'b0,a22}) <<< 1) + $signed({5'b0,a22});
            scharr_gx = right_f - left_f;
        end
    endfunction

    function signed [12:0] scharr_gy;
        input [7:0] a00, a01, a02;
        input [7:0] a10, a11, a12;
        input [7:0] a20, a21, a22;
        reg signed [12:0] top_f;
        reg signed [12:0] bottom_f;
        begin
            top_f    = ($signed({5'b0,a00}) <<< 1) + $signed({5'b0,a00})
                     + ($signed({5'b0,a01}) <<< 3) + ($signed({5'b0,a01}) <<< 1)
                     + ($signed({5'b0,a02}) <<< 1) + $signed({5'b0,a02});
            bottom_f = ($signed({5'b0,a20}) <<< 1) + $signed({5'b0,a20})
                     + ($signed({5'b0,a21}) <<< 3) + ($signed({5'b0,a21}) <<< 1)
                     + ($signed({5'b0,a22}) <<< 1) + $signed({5'b0,a22});
            scharr_gy = top_f - bottom_f;
        end
    endfunction

    wire signed [12:0] gx0_comb = scharr_gx(top_prev2,top_prev1,top0,
                                            mid_prev2,mid_prev1,mid0,
                                            bot_prev2,bot_prev1,gray0);
    wire signed [12:0] gy0_comb = scharr_gy(top_prev2,top_prev1,top0,
                                            mid_prev2,mid_prev1,mid0,
                                            bot_prev2,bot_prev1,gray0);
    wire signed [12:0] gx1_comb = scharr_gx(top_prev1,top0,top1,
                                            mid_prev1,mid0,mid1,
                                            bot_prev1,gray0,gray1);
    wire signed [12:0] gy1_comb = scharr_gy(top_prev1,top0,top1,
                                            mid_prev1,mid0,mid1,
                                            bot_prev1,gray0,gray1);
    wire signed [12:0] gx2_comb = scharr_gx(top0,top1,top2,
                                            mid0,mid1,mid2,
                                            gray0,gray1,gray2);
    wire signed [12:0] gy2_comb = scharr_gy(top0,top1,top2,
                                            mid0,mid1,mid2,
                                            gray0,gray1,gray2);
    wire signed [12:0] gx3_comb = scharr_gx(top1,top2,top3,
                                            mid1,mid2,mid3,
                                            gray1,gray2,gray3);
    wire signed [12:0] gy3_comb = scharr_gy(top1,top2,top3,
                                            mid1,mid2,mid3,
                                            gray1,gray2,gray3);

    reg signed [12:0] gx0_s1, gy0_s1, gx1_s1, gy1_s1;
    reg signed [12:0] gx2_s1, gy2_s1, gx3_s1, gy3_s1;
    reg [10:0] threshold_s1;
    reg [3:0] window_valid_s1;
    reg valid_s1, last_s1, user_s1;

    wire [12:0] abs_gx0 = gx0_s1[12] ? (~gx0_s1 + 1'b1) : gx0_s1;
    wire [12:0] abs_gy0 = gy0_s1[12] ? (~gy0_s1 + 1'b1) : gy0_s1;
    wire [12:0] abs_gx1 = gx1_s1[12] ? (~gx1_s1 + 1'b1) : gx1_s1;
    wire [12:0] abs_gy1 = gy1_s1[12] ? (~gy1_s1 + 1'b1) : gy1_s1;
    wire [12:0] abs_gx2 = gx2_s1[12] ? (~gx2_s1 + 1'b1) : gx2_s1;
    wire [12:0] abs_gy2 = gy2_s1[12] ? (~gy2_s1 + 1'b1) : gy2_s1;
    wire [12:0] abs_gx3 = gx3_s1[12] ? (~gx3_s1 + 1'b1) : gx3_s1;
    wire [12:0] abs_gy3 = gy3_s1[12] ? (~gy3_s1 + 1'b1) : gy3_s1;
    wire [13:0] mag0_raw_s2 = {1'b0,abs_gx0} + {1'b0,abs_gy0};
    wire [13:0] mag1_raw_s2 = {1'b0,abs_gx1} + {1'b0,abs_gy1};
    wire [13:0] mag2_raw_s2 = {1'b0,abs_gx2} + {1'b0,abs_gy2};
    wire [13:0] mag3_raw_s2 = {1'b0,abs_gx3} + {1'b0,abs_gy3};
    // Divide by four with rounding.  This keeps the normalized Scharr range
    // at 0..2040, matching the existing 11-bit manual threshold interface.
    wire [11:0] mag0_s2 = (mag0_raw_s2 + 14'd2) >> 2;
    wire [11:0] mag1_s2 = (mag1_raw_s2 + 14'd2) >> 2;
    wire [11:0] mag2_s2 = (mag2_raw_s2 + 14'd2) >> 2;
    wire [11:0] mag3_s2 = (mag3_raw_s2 + 14'd2) >> 2;
    function [1:0] quantize_direction;
        input [12:0] abs_gx;
        input [12:0] abs_gy;
        input gx_negative;
        input gy_negative;
        reg [14:0] gx_times_2, gy_times_2;
        reg [14:0] gx_times_5, gy_times_5;
        begin
            gx_times_2 = {2'b0,abs_gx} << 1;
            gy_times_2 = {2'b0,abs_gy} << 1;
            gx_times_5 = ({2'b0,abs_gx} << 2) + {2'b0,abs_gx};
            gy_times_5 = ({2'b0,abs_gy} << 2) + {2'b0,abs_gy};
            if (gy_times_5 <= gx_times_2)
                quantize_direction = 2'd0;
            else if (gx_times_5 <= gy_times_2)
                quantize_direction = 2'd2;
            else if (gx_negative == gy_negative)
                quantize_direction = 2'd3;
            else
                quantize_direction = 2'd1;
        end
    endfunction

    wire [7:0] direction_s2 = {
        quantize_direction(abs_gx0,abs_gy0,gx0_s1[12],gy0_s1[12]),
        quantize_direction(abs_gx1,abs_gy1,gx1_s1[12],gy1_s1[12]),
        quantize_direction(abs_gx2,abs_gy2,gx2_s1[12],gy2_s1[12]),
        quantize_direction(abs_gx3,abs_gy3,gx3_s1[12],gy3_s1[12])
    };
    wire [47:0] magnitude_s2 = {
        window_valid_s1[3] ? mag0_s2 : 12'd0,
        window_valid_s1[2] ? mag1_s2 : 12'd0,
        window_valid_s1[1] ? mag2_s2 : 12'd0,
        window_valid_s1[0] ? mag3_s2 : 12'd0
    };

    wire        nms_tlast;
    wire        nms_tuser;
    wire [95:0] nms_tdata;
    wire        nms_tvalid;
    wire        nms_tready;

    nonmax_suppression_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_nonmax_suppression_96 (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_tlast     (last_s1),
        .I_tuser     (user_s1),
        .I_mag       (magnitude_s2),
        .I_dir       (direction_s2),
        .I_tvalid    (valid_s1),
        .I_tready    (I_tready),
        .I_threshold (threshold_s1),
        .O_tlast     (nms_tlast),
        .O_tuser     (nms_tuser),
        .O_tdata     (nms_tdata),
        .O_tvalid    (nms_tvalid),
        .O_tready    (nms_tready)
    );

    edge_hysteresis_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_edge_hysteresis_96 (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_tlast     (nms_tlast),
        .I_tuser     (nms_tuser),
        .I_tdata     (nms_tdata),
        .I_tvalid    (nms_tvalid),
        .I_tready    (I_tready),
        .O_tlast     (O_tlast),
        .O_tuser     (O_tuser),
        .O_tdata     (O_tdata),
        .O_tvalid    (O_tvalid),
        .O_tready    (O_tready)
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gray_word_s0 <= 32'd0;
            threshold_s0 <= 11'd0;
            valid_s0 <= 1'b0; last_s0 <= 1'b0; user_s0 <= 1'b0;
            gx0_s1 <= 13'sd0; gy0_s1 <= 13'sd0;
            gx1_s1 <= 13'sd0; gy1_s1 <= 13'sd0;
            gx2_s1 <= 13'sd0; gy2_s1 <= 13'sd0;
            gx3_s1 <= 13'sd0; gy3_s1 <= 13'sd0;
            threshold_s1 <= 11'd0;
            window_valid_s1 <= 4'b0000;
            valid_s1 <= 1'b0; last_s1 <= 1'b0; user_s1 <= 1'b0;
            x_grp    <= 11'd0;
            y_pos    <= 11'd0;
            top_prev2 <= 8'd0; top_prev1 <= 8'd0;
            mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
            bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        end else begin
            valid_s0 <= I_tvalid && I_tready;
            last_s0  <= I_tlast && I_tvalid && I_tready;
            valid_s1 <= valid_s0;
            last_s1  <= last_s0 && valid_s0;
            // Frame start is independent of pixel valid on the SC520 stream.
            // Preserve it through the same three control pipeline registers.
            user_s0  <= I_tuser;
            user_s1  <= user_s0;

            if (I_tvalid && I_tready) begin
                gray_word_s0 <= gray_word_comb;
                threshold_s0 <= I_threshold;
            end

            if (user_s0 && !valid_s0) begin
                // The normal SC520 case: frame start arrives in blanking.
                x_grp <= 11'd0;
                y_pos <= 11'd0;
                top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
            end else if (valid_s0) begin
                gx0_s1 <= gx0_comb; gy0_s1 <= gy0_comb;
                gx1_s1 <= gx1_comb; gy1_s1 <= gy1_comb;
                gx2_s1 <= gx2_comb; gy2_s1 <= gy2_comb;
                gx3_s1 <= gx3_comb; gy3_s1 <= gy3_comb;
                threshold_s1 <= threshold_s0;
                // The first two results of a line require pixels from the
                // previous beat.  Results 2 and 3 already have a complete
                // three-pixel horizontal neighborhood in the first beat.
                window_valid_s1[3:2] <= {(2){(y_pos >= 11'd2) &&
                                             (x_grp != 11'd0) && !user_s0}};
                window_valid_s1[1:0] <= {(2){(y_pos >= 11'd2) && !user_s0}};
                line2_mem[x_grp] <= mid_word;
                line1_mem[x_grp] <= bot_word;

                top_prev2 <= top2;  top_prev1 <= top3;
                mid_prev2 <= mid2;  mid_prev1 <= mid3;
                bot_prev2 <= gray2; bot_prev1 <= gray3;

                if (user_s0) begin
                    x_grp <= 11'd1;
                    y_pos <= 11'd0;
                end else if (x_grp == IMG_WIDTH_GRP - 1) begin
                    x_grp <= 11'd0;
                    if (y_pos == IMG_HEIGHT - 1)
                        y_pos <= 11'd0;
                    else
                        y_pos <= y_pos + 1'b1;
                    top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                    mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                    bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
                end else begin
                    x_grp <= x_grp + 1'b1;
                end
            end
        end
    end

endmodule
