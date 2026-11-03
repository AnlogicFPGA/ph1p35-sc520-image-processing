// Four-pixel-per-beat non-maximum suppression for normalized Scharr data.
// Each input magnitude is 12 bits (0..2040); each direction is 2 bits:
// 0=horizontal gradient, 1=45 degrees, 2=vertical, 3=135 degrees.
module nonmax_suppression_96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [47:0] I_mag,
    input  wire [7:0]  I_dir,
    input  wire        I_tvalid,
    input  wire        I_tready,
    input  wire [10:0] I_threshold,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output wire        O_tready
);

    localparam integer IMG_WIDTH_GRP = IMG_WIDTH / 4;
    localparam [1:0] DIR_H = 2'd0;
    localparam [1:0] DIR_D1 = 2'd1;
    localparam [1:0] DIR_V = 2'd2;
    localparam [1:0] DIR_D2 = 2'd3;

    assign O_tready = I_tready;

    // Pack both magnitude rows and the center-row directions into one PH1P
    // embedded-RAM bank.  Port A performs the registered read for the current
    // group; port B writes the updated two-row history one clock later.
    // Explicit instantiation is used because TD otherwise maps this shallow,
    // wide 320x104 buffer into distributed LUT RAM even with a block-RAM hint.
    wire [103:0] line_read_q;
    reg [10:0] x_grp;
    reg [10:0] y_pos;
    reg [47:0] mag_s0;
    reg [7:0]  dir_s0;
    reg [10:0] threshold_s0;
    reg valid_s0, last_s0, user_s0;

    reg [47:0] mag_s1;
    reg [7:0]  dir_s1;
    reg [10:0] threshold_s1;
    reg [10:0] x_grp_s1;
    reg [10:0] y_pos_s1;
    reg valid_s1, last_s1, user_s1;

    wire [47:0] above_word  = line_read_q[103:56];
    wire [47:0] center_word = line_read_q[55:8];
    wire [7:0]  center_dirs = line_read_q[7:0];
    wire [103:0] line_write_data = {center_word, mag_s1, dir_s1};
    wire [103:0] line_unused;

    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A (104),
        .DATA_WIDTH_B (104),
        .ADDR_WIDTH_A (9),
        .ADDR_WIDTH_B (9),
        .DATA_DEPTH_A (IMG_WIDTH_GRP),
        .DATA_DEPTH_B (IMG_WIDTH_GRP),
        .MODE         ("DP"),
        .REGMODE_A    ("NOREG"),
        .REGMODE_B    ("NOREG"),
        .WRITEMODE_A  ("NORMAL"),
        .WRITEMODE_B  ("NORMAL"),
        .RESETMODE_A  ("ASYNC"),
        .RESETMODE_B  ("ASYNC"),
        .CLKMODE      ("SYNC"),
        .SSROVERCE    ("DISABLE"),
        .OREGSET_A    ("RESET"),
        .OREGSET_B    ("RESET"),
        .ASYNC_RESET_RELEASE_A ("ASYNC"),
        .ASYNC_RESET_RELEASE_B ("ASYNC"),
        .IMPLEMENT    ("20K(FAST)")
    ) u_nms_line_eram (
        .dia          (104'd0),
        .addra        (x_grp[8:0]),
        .cea          (valid_s0),
        .ocea         (1'b0),
        .clka         (I_clk),
        .wea          (1'b0),
        .rsta         (1'b0),
        .bea          (1'b0),
        .dib          (line_write_data),
        .addrb        (x_grp_s1[8:0]),
        .ceb          (valid_s1),
        .oceb         (1'b0),
        .clkb         (I_clk),
        .web          (valid_s1),
        .rstb         (1'b0),
        .beb          (1'b0),
        .ecc_sbiterrinj (1'b0),
        .ecc_dbiterrinj (1'b0),
        .doa          (line_read_q),
        .dob          (line_unused),
        .ecc_sbiterr  (),
        .ecc_dbiterr  ()
    );

    wire [11:0] above0 = above_word[47:36];
    wire [11:0] above1 = above_word[35:24];
    wire [11:0] above2 = above_word[23:12];
    wire [11:0] above3 = above_word[11:0];
    wire [11:0] center0 = center_word[47:36];
    wire [11:0] center1 = center_word[35:24];
    wire [11:0] center2 = center_word[23:12];
    wire [11:0] center3 = center_word[11:0];
    wire [11:0] below0 = mag_s1[47:36];
    wire [11:0] below1 = mag_s1[35:24];
    wire [11:0] below2 = mag_s1[23:12];
    wire [11:0] below3 = mag_s1[11:0];

    wire [1:0] center_dir0 = center_dirs[7:6];
    wire [1:0] center_dir1 = center_dirs[5:4];
    wire [1:0] center_dir2 = center_dirs[3:2];

    reg [11:0] above_prev2, above_prev1;
    reg [11:0] center_prev2, center_prev1;
    reg [11:0] below_prev2, below_prev1;
    reg [1:0]  center_dir_prev1;

    function [11:0] keep_peak;
        input [11:0] center;
        input [11:0] left, right, up, down;
        input [11:0] up_left, up_right, down_left, down_right;
        input [1:0] direction;
        reg [11:0] neighbor_a, neighbor_b;
        begin
            case (direction)
                DIR_H: begin neighbor_a = left;     neighbor_b = right;      end
                DIR_D1:begin neighbor_a = up_left;  neighbor_b = down_right; end
                DIR_V: begin neighbor_a = up;       neighbor_b = down;       end
                default:begin neighbor_a = up_right; neighbor_b = down_left; end
            endcase
            // The asymmetric tie rule keeps one side of a flat ridge instead
            // of retaining a two-pixel-wide plateau.
            keep_peak = ((center > neighbor_a) && (center >= neighbor_b))
                      ? center : 12'd0;
        end
    endfunction

    // Delay the spatial result by one pixel so the right-hand neighbor is
    // available without stalling the four-pixel stream.
    wire [11:0] peak0 = keep_peak(center_prev1,
                                  center_prev2, center0,
                                  above_prev1, below_prev1,
                                  above_prev2, above0, below_prev2, below0,
                                  center_dir_prev1);
    wire [11:0] peak1 = keep_peak(center0,
                                  center_prev1, center1,
                                  above0, below0,
                                  above_prev1, above1, below_prev1, below1,
                                  center_dir0);
    wire [11:0] peak2 = keep_peak(center1,
                                  center0, center2,
                                  above1, below1,
                                  above0, above2, below0, below2,
                                  center_dir1);
    wire [11:0] peak3 = keep_peak(center2,
                                  center1, center3,
                                  above2, below2,
                                  above1, above3, below1, below3,
                                  center_dir2);

    wire vertical_valid = (y_pos_s1 >= 11'd2);
    wire [3:0] window_valid = {{2{vertical_valid && (x_grp_s1 != 11'd0)}},
                               {2{vertical_valid}}};
    // First Canny classification stage.  The existing 11-bit control is the
    // high threshold; the low threshold is one half of it, a common fixed
    // hardware ratio.  0xff marks a strong edge, 0x80 a weak edge, and 0x00
    // a suppressed sample.  Keeping weak edges visible in this intermediate
    // stage makes the classification independently testable before the next
    // hysteresis-connection step is added.
    wire [10:0] low_threshold_s1 = (threshold_s1 <= 11'd1)
                                  ? 11'd1 : (threshold_s1 >> 1);

    function [7:0] classify_peak;
        input        is_valid;
        input [11:0] peak;
        input [10:0] high_threshold;
        input [10:0] low_threshold;
        begin
            if (!is_valid || (peak < {1'b0,low_threshold}))
                classify_peak = 8'h00;
            else if (peak >= {1'b0,high_threshold})
                classify_peak = 8'hff;
            else
                classify_peak = 8'h80;
        end
    endfunction

    wire [7:0] edge_class0 = classify_peak(window_valid[3], peak0,
                                            threshold_s1, low_threshold_s1);
    wire [7:0] edge_class1 = classify_peak(window_valid[2], peak1,
                                            threshold_s1, low_threshold_s1);
    wire [7:0] edge_class2 = classify_peak(window_valid[1], peak2,
                                            threshold_s1, low_threshold_s1);
    wire [7:0] edge_class3 = classify_peak(window_valid[0], peak3,
                                            threshold_s1, low_threshold_s1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            x_grp <= 11'd0;
            y_pos <= 11'd0;
            mag_s0 <= 48'd0;
            dir_s0 <= 8'd0;
            threshold_s0 <= 11'd0;
            valid_s0 <= 1'b0;
            last_s0 <= 1'b0;
            user_s0 <= 1'b0;
            mag_s1 <= 48'd0;
            dir_s1 <= 8'd0;
            threshold_s1 <= 11'd0;
            x_grp_s1 <= 11'd0;
            y_pos_s1 <= 11'd0;
            valid_s1 <= 1'b0;
            last_s1 <= 1'b0;
            user_s1 <= 1'b0;
            above_prev2 <= 12'd0; above_prev1 <= 12'd0;
            center_prev2 <= 12'd0; center_prev1 <= 12'd0;
            below_prev2 <= 12'd0; below_prev1 <= 12'd0;
            center_dir_prev1 <= DIR_H;
        end else begin
            mag_s0 <= I_mag;
            dir_s0 <= I_dir;
            threshold_s0 <= I_threshold;
            valid_s0 <= I_tvalid && I_tready;
            last_s0 <= I_tlast && I_tvalid && I_tready;
            user_s0 <= I_tuser;

            valid_s1 <= valid_s0;
            last_s1 <= last_s0 && valid_s0;
            user_s1 <= user_s0;
            if (valid_s0) begin
                // Registered ERAM read request and matching stream metadata.
                mag_s1 <= mag_s0;
                dir_s1 <= dir_s0;
                threshold_s1 <= threshold_s0;
                x_grp_s1 <= x_grp;
                y_pos_s1 <= y_pos;
            end

            O_tlast  <= last_s1;
            O_tuser  <= user_s1;
            O_tvalid <= valid_s1;
            if (valid_s1)
                O_tdata <= {{3{edge_class0}}, {3{edge_class1}},
                            {3{edge_class2}}, {3{edge_class3}}};
            else
                O_tdata <= 96'd0;

            // Advance the read-request coordinates with stage 0.
            if (user_s0 && !valid_s0) begin
                x_grp <= 11'd0;
                y_pos <= 11'd0;
            end else if (valid_s0) begin
                if (user_s0) begin
                    x_grp <= 11'd1;
                    y_pos <= 11'd0;
                end else if (x_grp == IMG_WIDTH_GRP - 1) begin
                    x_grp <= 11'd0;
                    if (y_pos == IMG_HEIGHT - 1)
                        y_pos <= 11'd0;
                    else
                        y_pos <= y_pos + 1'b1;
                end else begin
                    x_grp <= x_grp + 1'b1;
                end
            end

            // Consume the registered memory response one clock later.  The
            // old center row is shifted upward while the new row is written.
            if (user_s1 && !valid_s1) begin
                above_prev2 <= 12'd0; above_prev1 <= 12'd0;
                center_prev2 <= 12'd0; center_prev1 <= 12'd0;
                below_prev2 <= 12'd0; below_prev1 <= 12'd0;
                center_dir_prev1 <= DIR_H;
            end else if (valid_s1) begin
                if (x_grp_s1 == IMG_WIDTH_GRP - 1) begin
                    above_prev2 <= 12'd0; above_prev1 <= 12'd0;
                    center_prev2 <= 12'd0; center_prev1 <= 12'd0;
                    below_prev2 <= 12'd0; below_prev1 <= 12'd0;
                    center_dir_prev1 <= DIR_H;
                end else begin
                    above_prev2 <= above2; above_prev1 <= above3;
                    center_prev2 <= center2; center_prev1 <= center3;
                    below_prev2 <= below2; below_prev1 <= below3;
                    center_dir_prev1 <= center_dirs[1:0];
                end
            end
        end
    end
endmodule
