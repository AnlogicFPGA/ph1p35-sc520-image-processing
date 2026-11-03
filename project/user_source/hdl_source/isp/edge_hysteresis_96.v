// Streaming 3x3 Canny hysteresis for four pixels per clock.
// Input pixel values are the double-threshold classes produced by NMS:
//   8'hff = strong edge, 8'h80 = weak edge, 8'h00 = non-edge.
// A weak center pixel is retained only when at least one of its eight direct
// neighbors is strong.  Output is a final binary edge image.
module edge_hysteresis_96 #(
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

    localparam integer IMG_WIDTH_GRP = IMG_WIDTH / 4;
    localparam [1:0] CLASS_NONE   = 2'b00;
    localparam [1:0] CLASS_WEAK   = 2'b01;
    localparam [1:0] CLASS_STRONG = 2'b10;

    assign O_tready = I_tready;

    function [1:0] decode_class;
        input [7:0] pixel;
        begin
            if (pixel == 8'hff)
                decode_class = CLASS_STRONG;
            else if (pixel == 8'h80)
                decode_class = CLASS_WEAK;
            else
                decode_class = CLASS_NONE;
        end
    endfunction

    wire [7:0] class_word_comb = {
        decode_class(I_tdata[95:88]),
        decode_class(I_tdata[71:64]),
        decode_class(I_tdata[47:40]),
        decode_class(I_tdata[23:16])
    };

    // One 320x16 dual-port ERAM stores the two completed classification rows.
    // Port A reads the current group; port B writes {old center,new below} one
    // clock later, matching the line-buffer rotation already used by NMS.
    wire [15:0] line_read_q;
    wire [15:0] line_unused;
    reg  [10:0] x_grp;
    reg  [10:0] y_pos;
    reg  [7:0]  class_s0;
    reg         valid_s0, last_s0, user_s0;

    reg  [7:0]  class_s1;
    reg  [10:0] x_grp_s1;
    reg  [10:0] y_pos_s1;
    reg         valid_s1, last_s1, user_s1;

    wire [7:0] above_word  = line_read_q[15:8];
    wire [7:0] center_word = line_read_q[7:0];
    wire [15:0] line_write_data = {center_word, class_s1};

    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A (16),
        .DATA_WIDTH_B (16),
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
    ) u_hysteresis_line_eram (
        .dia          (16'd0),
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

    wire [1:0] above0 = above_word[7:6];
    wire [1:0] above1 = above_word[5:4];
    wire [1:0] above2 = above_word[3:2];
    wire [1:0] above3 = above_word[1:0];
    wire [1:0] center0 = center_word[7:6];
    wire [1:0] center1 = center_word[5:4];
    wire [1:0] center2 = center_word[3:2];
    wire [1:0] center3 = center_word[1:0];
    wire [1:0] below0 = class_s1[7:6];
    wire [1:0] below1 = class_s1[5:4];
    wire [1:0] below2 = class_s1[3:2];
    wire [1:0] below3 = class_s1[1:0];

    reg [1:0] above_prev2, above_prev1;
    reg [1:0] center_prev2, center_prev1;
    reg [1:0] below_prev2, below_prev1;

    function [7:0] resolve_edge;
        input        is_valid;
        input [1:0]  center;
        input [1:0]  left, right, up, down;
        input [1:0]  up_left, up_right, down_left, down_right;
        reg          touches_strong;
        begin
            touches_strong = (left       == CLASS_STRONG)
                           || (right      == CLASS_STRONG)
                           || (up         == CLASS_STRONG)
                           || (down       == CLASS_STRONG)
                           || (up_left    == CLASS_STRONG)
                           || (up_right   == CLASS_STRONG)
                           || (down_left  == CLASS_STRONG)
                           || (down_right == CLASS_STRONG);
            if (!is_valid)
                resolve_edge = 8'h00;
            else if (center == CLASS_STRONG)
                resolve_edge = 8'hff;
            else if ((center == CLASS_WEAK) && touches_strong)
                resolve_edge = 8'hff;
            else
                resolve_edge = 8'h00;
        end
    endfunction

    wire vertical_valid = (y_pos_s1 >= 11'd2);
    wire [3:0] window_valid = {{2{vertical_valid && (x_grp_s1 != 11'd0)}},
                               {2{vertical_valid}}};

    // Pixel 0 is the final pixel of the preceding four-pixel group.  This
    // one-pixel spatial delay makes its right-hand neighbors available and
    // also covers adjacency across a 96-bit beat boundary.
    wire [7:0] edge0 = resolve_edge(window_valid[3], center_prev1,
                                    center_prev2, center0,
                                    above_prev1, below_prev1,
                                    above_prev2, above0,
                                    below_prev2, below0);
    wire [7:0] edge1 = resolve_edge(window_valid[2], center0,
                                    center_prev1, center1,
                                    above0, below0,
                                    above_prev1, above1,
                                    below_prev1, below1);
    wire [7:0] edge2 = resolve_edge(window_valid[1], center1,
                                    center0, center2,
                                    above1, below1,
                                    above0, above2,
                                    below0, below2);
    wire [7:0] edge3 = resolve_edge(window_valid[0], center2,
                                    center1, center3,
                                    above2, below2,
                                    above1, above3,
                                    below1, below3);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            x_grp <= 11'd0;
            y_pos <= 11'd0;
            class_s0 <= 8'd0;
            valid_s0 <= 1'b0;
            last_s0 <= 1'b0;
            user_s0 <= 1'b0;
            class_s1 <= 8'd0;
            x_grp_s1 <= 11'd0;
            y_pos_s1 <= 11'd0;
            valid_s1 <= 1'b0;
            last_s1 <= 1'b0;
            user_s1 <= 1'b0;
            above_prev2 <= CLASS_NONE;
            above_prev1 <= CLASS_NONE;
            center_prev2 <= CLASS_NONE;
            center_prev1 <= CLASS_NONE;
            below_prev2 <= CLASS_NONE;
            below_prev1 <= CLASS_NONE;
        end else begin
            class_s0 <= class_word_comb;
            valid_s0 <= I_tvalid && I_tready;
            last_s0  <= I_tlast && I_tvalid && I_tready;
            user_s0  <= I_tuser;

            valid_s1 <= valid_s0;
            last_s1  <= last_s0 && valid_s0;
            user_s1  <= user_s0;
            if (valid_s0) begin
                class_s1 <= class_s0;
                x_grp_s1 <= x_grp;
                y_pos_s1 <= y_pos;
            end

            O_tlast  <= last_s1;
            O_tuser  <= user_s1;
            O_tvalid <= valid_s1;
            if (valid_s1)
                O_tdata <= {{3{edge0}}, {3{edge1}},
                            {3{edge2}}, {3{edge3}}};
            else
                O_tdata <= 96'd0;

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

            if (user_s1 && !valid_s1) begin
                above_prev2 <= CLASS_NONE;
                above_prev1 <= CLASS_NONE;
                center_prev2 <= CLASS_NONE;
                center_prev1 <= CLASS_NONE;
                below_prev2 <= CLASS_NONE;
                below_prev1 <= CLASS_NONE;
            end else if (valid_s1) begin
                if (x_grp_s1 == IMG_WIDTH_GRP - 1) begin
                    above_prev2 <= CLASS_NONE;
                    above_prev1 <= CLASS_NONE;
                    center_prev2 <= CLASS_NONE;
                    center_prev1 <= CLASS_NONE;
                    below_prev2 <= CLASS_NONE;
                    below_prev1 <= CLASS_NONE;
                end else begin
                    above_prev2 <= above2;
                    above_prev1 <= above3;
                    center_prev2 <= center2;
                    center_prev1 <= center3;
                    below_prev2 <= below2;
                    below_prev1 <= below3;
                end
            end
        end
    end
endmodule
