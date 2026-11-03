// Four-pixel-per-beat 3x3 Gaussian pre-filter for a BT.709 grayscale stream.
// Kernel: (1/16) * [1 2 1; 2 4 2; 1 2 1].
// I_tdata contains grayscale replicated onto R/G/B.  The conversion is shared
// with the gray/binary paths so this stage needs no additional DSP multipliers.
module gaussian_blur_96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tready,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output wire        O_tready
);

    assign O_tready = I_tready;

    wire [31:0] gray_word_comb = {I_tdata[95:88], I_tdata[71:64],
                                  I_tdata[47:40], I_tdata[23:16]};
    reg [31:0] gray_word_s0;
    reg valid_s0, last_s0, user_s0;

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
    wire [31:0] bot_word = gray_word_s0;

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

    function [7:0] gaussian_pixel;
        input [7:0] a00, a01, a02;
        input [7:0] a10, a11, a12;
        input [7:0] a20, a21, a22;
        reg [11:0] weighted_sum;
        begin
            weighted_sum = {4'd0,a00} + ({4'd0,a01} << 1) + {4'd0,a02}
                         + ({4'd0,a10} << 1) + ({4'd0,a11} << 2)
                         + ({4'd0,a12} << 1)
                         + {4'd0,a20} + ({4'd0,a21} << 1) + {4'd0,a22};
            gaussian_pixel = (weighted_sum + 12'd8) >> 4;
        end
    endfunction

    wire [7:0] blur0 = gaussian_pixel(top_prev2,top_prev1,top0,
                                      mid_prev2,mid_prev1,mid0,
                                      bot_prev2,bot_prev1,gray0);
    wire [7:0] blur1 = gaussian_pixel(top_prev1,top0,top1,
                                      mid_prev1,mid0,mid1,
                                      bot_prev1,gray0,gray1);
    wire [7:0] blur2 = gaussian_pixel(top0,top1,top2,
                                      mid0,mid1,mid2,
                                      gray0,gray1,gray2);
    wire [7:0] blur3 = gaussian_pixel(top1,top2,top3,
                                      mid1,mid2,mid3,
                                      gray1,gray2,gray3);

    wire rows_valid = (y_pos >= 11'd2) && !user_s0;
    wire prior_beat_valid = (x_grp != 11'd0);
    wire [23:0] out0 = (rows_valid && prior_beat_valid) ? {3{blur0}} : 24'd0;
    wire [23:0] out1 = (rows_valid && prior_beat_valid) ? {3{blur1}} : 24'd0;
    wire [23:0] out2 = rows_valid ? {3{blur2}} : 24'd0;
    wire [23:0] out3 = rows_valid ? {3{blur3}} : 24'd0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gray_word_s0 <= 32'd0;
            valid_s0 <= 1'b0;
            last_s0  <= 1'b0;
            user_s0  <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            x_grp <= 11'd0;
            y_pos <= 11'd0;
            top_prev2 <= 8'd0; top_prev1 <= 8'd0;
            mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
            bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        end else begin
            valid_s0 <= I_tvalid && I_tready;
            last_s0  <= I_tlast && I_tvalid && I_tready;
            user_s0  <= I_tuser;
            O_tvalid <= valid_s0;
            O_tlast  <= last_s0 && valid_s0;
            O_tuser  <= user_s0;

            if (I_tvalid && I_tready)
                gray_word_s0 <= gray_word_comb;

            if (valid_s0)
                O_tdata <= {out0,out1,out2,out3};
            else
                O_tdata <= 96'd0;

            if (user_s0 && !valid_s0) begin
                x_grp <= 11'd0;
                y_pos <= 11'd0;
                top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
            end else if (valid_s0) begin
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
                    y_pos <= (y_pos == IMG_HEIGHT - 1) ? 11'd0 : y_pos + 1'b1;
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
