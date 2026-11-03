// Frame-based Otsu threshold estimator for a four-pixel RGB888 gray stream.
//
// Four independent histogram banks accept all four pixels every clock.  Two
// ping-pong histogram sets allow one frame to be accumulated while the
// previous frame is evaluated.  The resulting threshold is committed only at
// a later frame boundary, so every pixel in one frame uses one stable value.
//
// The between-class variance comparison uses integer class means.  This is a
// fixed-point hardware form of Otsu: divisions are shared in time over the
// vertical blanking/frame interval rather than inferred as large combinational
// dividers.
module otsu_threshold_96 (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_valid,
    input  wire [95:0] I_gray,
    output reg  [7:0]  O_threshold_active,
    output reg  [7:0]  O_threshold_result,
    output reg         O_result_valid
);
    localparam [3:0] ST_INIT0     = 4'd0;
    localparam [3:0] ST_INIT1     = 4'd1;
    localparam [3:0] ST_IDLE      = 4'd2;
    localparam [3:0] ST_SCAN_SET  = 4'd3;
    localparam [3:0] ST_SCAN_WAIT = 4'd4;
    localparam [3:0] ST_SCAN_CALC = 4'd5;
    localparam [3:0] ST_DIV_START = 4'd6;
    localparam [3:0] ST_DIV_WAIT  = 4'd7;
    localparam [3:0] ST_SCORE     = 4'd8;
    localparam [3:0] ST_CLEAR     = 4'd9;
    localparam [3:0] ST_MUL_WEIGHT= 4'd10;
    localparam [3:0] ST_MUL_SCORE = 4'd11;
    localparam [3:0] ST_SCAN_LATCH= 4'd12;
    localparam [3:0] ST_SCAN_MULT = 4'd13;

    reg [3:0] S_state;
    reg       S_active_set;
    reg       S_scan_set;
    reg       S_frame_seen;
    reg [7:0] S_addr;

    reg [19:0] S_total_count_live;
    reg [27:0] S_total_sum_live;
    reg [19:0] S_total_count_scan;
    reg [27:0] S_total_sum_scan;
    reg [19:0] S_weight_b;
    reg [27:0] S_sum_b;
    reg [19:0] S_div_den_b;
    reg [19:0] S_div_den_f;
    reg [27:0] S_div_num_b;
    reg [27:0] S_div_num_f;
    reg [19:0] S_score_weight_b;
    reg [19:0] S_score_weight_f;
    reg [19:0] S_bin_count_reg;
    reg [27:0] S_bin_weighted_reg;
    reg [57:0] S_best_score;
    reg [17:0] S_diff_square_reg;
    reg [39:0] S_weight_acc;
    reg [39:0] S_weight_multiplicand;
    reg [19:0] S_weight_multiplier;
    reg [57:0] S_score_acc;
    reg [57:0] S_score_multiplicand;
    reg [17:0] S_score_multiplier;
    reg [57:0] S_score_reg;
    reg [5:0]  S_mult_count;
    reg [7:0]  S_best_threshold;
    reg [7:0]  S_threshold_pending;
    reg        S_pending_valid;

    // Isolate the BT.709 multiply/add network from histogram arithmetic.  The
    // one-clock statistics pipeline has no effect on the video path and is
    // naturally absorbed by the frame blanking interval.
    reg        S_valid_d;
    reg [31:0] S_pixels_d;
    wire [7:0] S_pixel0 = S_pixels_d[7:0];
    wire [7:0] S_pixel1 = S_pixels_d[15:8];
    wire [7:0] S_pixel2 = S_pixels_d[23:16];
    wire [7:0] S_pixel3 = S_pixels_d[31:24];
    wire [9:0] S_pixel_sum = {2'b0,S_pixel0} + {2'b0,S_pixel1}
                           + {2'b0,S_pixel2} + {2'b0,S_pixel3};

    wire S_collect = (S_state != ST_INIT0) && (S_state != ST_INIT1);
    wire S_accum0 = S_collect && S_valid_d && (S_active_set == 1'b0);
    wire S_accum1 = S_collect && S_valid_d && (S_active_set == 1'b1);
    wire S_clear0 = ((S_state == ST_INIT0) ||
                    ((S_state == ST_CLEAR) && (S_scan_set == 1'b0)));
    wire S_clear1 = ((S_state == ST_INIT1) ||
                    ((S_state == ST_CLEAR) && (S_scan_set == 1'b1)));
    wire [7:0] S_scan_addr = S_addr;

    wire [19:0] S_h00, S_h01, S_h02, S_h03;
    wire [19:0] S_h10, S_h11, S_h12, S_h13;

    otsu_histogram_bank u_hist00 (.I_clk(I_clk),.I_accum_enable(S_accum0),.I_accum_addr(S_pixel0),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear0),.I_clear_addr(S_addr),.O_scan_data(S_h00));
    otsu_histogram_bank u_hist01 (.I_clk(I_clk),.I_accum_enable(S_accum0),.I_accum_addr(S_pixel1),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear0),.I_clear_addr(S_addr),.O_scan_data(S_h01));
    otsu_histogram_bank u_hist02 (.I_clk(I_clk),.I_accum_enable(S_accum0),.I_accum_addr(S_pixel2),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear0),.I_clear_addr(S_addr),.O_scan_data(S_h02));
    otsu_histogram_bank u_hist03 (.I_clk(I_clk),.I_accum_enable(S_accum0),.I_accum_addr(S_pixel3),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear0),.I_clear_addr(S_addr),.O_scan_data(S_h03));
    otsu_histogram_bank u_hist10 (.I_clk(I_clk),.I_accum_enable(S_accum1),.I_accum_addr(S_pixel0),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear1),.I_clear_addr(S_addr),.O_scan_data(S_h10));
    otsu_histogram_bank u_hist11 (.I_clk(I_clk),.I_accum_enable(S_accum1),.I_accum_addr(S_pixel1),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear1),.I_clear_addr(S_addr),.O_scan_data(S_h11));
    otsu_histogram_bank u_hist12 (.I_clk(I_clk),.I_accum_enable(S_accum1),.I_accum_addr(S_pixel2),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear1),.I_clear_addr(S_addr),.O_scan_data(S_h12));
    otsu_histogram_bank u_hist13 (.I_clk(I_clk),.I_accum_enable(S_accum1),.I_accum_addr(S_pixel3),.I_scan_addr(S_scan_addr),.I_clear_enable(S_clear1),.I_clear_addr(S_addr),.O_scan_data(S_h13));

    wire [21:0] S_bin_count0 = {2'b0,S_h00}+{2'b0,S_h01}+{2'b0,S_h02}+{2'b0,S_h03};
    wire [21:0] S_bin_count1 = {2'b0,S_h10}+{2'b0,S_h11}+{2'b0,S_h12}+{2'b0,S_h13};
    wire [21:0] S_bin_count = S_scan_set ? S_bin_count1 : S_bin_count0;
    wire [19:0] S_bin_count_20 = S_bin_count[19:0];
    wire [27:0] S_bin_weighted_calc = S_bin_count_reg * S_addr;
    wire [19:0] S_weight_b_next = S_weight_b + S_bin_count_reg;
    wire [27:0] S_sum_b_next = S_sum_b + S_bin_weighted_reg;
    wire [19:0] S_weight_f_next = S_total_count_scan - S_weight_b_next;

    wire S_div_start = (S_state == ST_DIV_START);
    wire [27:0] S_mean_b;
    wire [27:0] S_mean_f;
    wire S_div_done_b;
    wire S_div_done_f;
    otsu_unsigned_divider u_div_b (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(S_div_start),
        .I_numerator(S_div_num_b),.I_denominator(S_div_den_b),
        .O_quotient(S_mean_b),.O_done(S_div_done_b)
    );
    otsu_unsigned_divider u_div_f (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(S_div_start),
        .I_numerator(S_div_num_f),.I_denominator(S_div_den_f),
        .O_quotient(S_mean_f),.O_done(S_div_done_f)
    );

    wire [8:0] S_mean_diff = (S_mean_b >= S_mean_f)
                           ? (S_mean_b[8:0] - S_mean_f[8:0])
                           : (S_mean_f[8:0] - S_mean_b[8:0]);
    wire [17:0] S_diff_square = S_mean_diff * S_mean_diff;
    wire [39:0] S_weight_acc_next = S_weight_multiplier[0]
                                          ? (S_weight_acc + S_weight_multiplicand)
                                          : S_weight_acc;
    wire [57:0] S_score_acc_next = S_score_multiplier[0]
                                          ? (S_score_acc + S_score_multiplicand)
                                          : S_score_acc;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state              <= ST_INIT0;
            S_active_set         <= 1'b0;
            S_scan_set           <= 1'b0;
            S_frame_seen         <= 1'b0;
            S_addr               <= 8'd0;
            S_total_count_live   <= 20'd0;
            S_total_sum_live     <= 28'd0;
            S_total_count_scan   <= 20'd0;
            S_total_sum_scan     <= 28'd0;
            S_weight_b           <= 20'd0;
            S_sum_b              <= 28'd0;
            S_div_den_b          <= 20'd1;
            S_div_den_f          <= 20'd1;
            S_div_num_b          <= 28'd0;
            S_div_num_f          <= 28'd0;
            S_score_weight_b     <= 20'd0;
            S_score_weight_f     <= 20'd0;
            S_bin_count_reg      <= 20'd0;
            S_bin_weighted_reg   <= 28'd0;
            S_best_score         <= 58'd0;
            S_diff_square_reg    <= 18'd0;
            S_weight_acc         <= 40'd0;
            S_weight_multiplicand<= 40'd0;
            S_weight_multiplier  <= 20'd0;
            S_score_acc          <= 58'd0;
            S_score_multiplicand <= 58'd0;
            S_score_multiplier   <= 18'd0;
            S_score_reg          <= 58'd0;
            S_mult_count         <= 6'd0;
            S_best_threshold     <= 8'd128;
            S_threshold_pending  <= 8'd128;
            S_pending_valid      <= 1'b0;
            O_threshold_active   <= 8'd128;
            O_threshold_result   <= 8'd128;
            O_result_valid       <= 1'b0;
            S_valid_d            <= 1'b0;
            S_pixels_d           <= 32'd0;
        end else begin
            O_result_valid <= 1'b0;
            S_valid_d      <= I_valid;
            if (I_valid)
                S_pixels_d <= {I_gray[95:88],I_gray[71:64],
                               I_gray[47:40],I_gray[23:16]};

            if (S_collect && S_valid_d) begin
                S_total_count_live <= S_total_count_live + 20'd4;
                S_total_sum_live   <= S_total_sum_live + S_pixel_sum;
            end

            if (I_frame_start && S_collect) begin
                if (S_pending_valid) begin
                    O_threshold_active <= S_threshold_pending;
                    S_pending_valid    <= 1'b0;
                end
                if (!S_frame_seen) begin
                    S_frame_seen       <= 1'b1;
                    S_total_count_live <= 20'd0;
                    S_total_sum_live   <= 28'd0;
                end else if (S_state == ST_IDLE) begin
                    S_scan_set         <= S_active_set;
                    S_active_set       <= ~S_active_set;
                    S_total_count_scan <= S_total_count_live;
                    S_total_sum_scan   <= S_total_sum_live;
                    S_total_count_live <= 20'd0;
                    S_total_sum_live   <= 28'd0;
                    S_weight_b         <= 20'd0;
                    S_sum_b            <= 28'd0;
                    S_best_score       <= 58'd0;
                    S_best_threshold   <= 8'd0;
                    S_addr             <= 8'd0;
                    S_state            <= ST_SCAN_SET;
                end
            end

            case (S_state)
                ST_INIT0: begin
                    if (S_addr == 8'd255) begin
                        S_addr  <= 8'd0;
                        S_state <= ST_INIT1;
                    end else S_addr <= S_addr + 8'd1;
                end
                ST_INIT1: begin
                    if (S_addr == 8'd255) begin
                        S_addr  <= 8'd0;
                        S_state <= ST_IDLE;
                    end else S_addr <= S_addr + 8'd1;
                end
                ST_SCAN_SET:  S_state <= ST_SCAN_WAIT;
                ST_SCAN_WAIT: S_state <= ST_SCAN_LATCH;
                ST_SCAN_LATCH: begin
                    S_bin_count_reg <= S_bin_count_20;
                    S_state         <= ST_SCAN_MULT;
                end
                ST_SCAN_MULT: begin
                    S_bin_weighted_reg <= S_bin_weighted_calc;
                    S_state            <= ST_SCAN_CALC;
                end
                ST_SCAN_CALC: begin
                    S_weight_b <= S_weight_b_next;
                    S_sum_b    <= S_sum_b_next;
                    if ((S_weight_b_next == 20'd0) || (S_weight_f_next == 20'd0)) begin
                        if (S_addr == 8'd254) begin
                            S_threshold_pending <= S_best_threshold;
                            O_threshold_result  <= S_best_threshold;
                            O_result_valid      <= 1'b1;
                            S_pending_valid     <= 1'b1;
                            S_addr              <= 8'd0;
                            S_state             <= ST_CLEAR;
                        end else begin
                            S_addr  <= S_addr + 8'd1;
                            S_state <= ST_SCAN_SET;
                        end
                    end else begin
                        S_div_num_b      <= S_sum_b_next;
                        S_div_den_b      <= S_weight_b_next;
                        S_div_num_f      <= S_total_sum_scan - S_sum_b_next;
                        S_div_den_f      <= S_weight_f_next;
                        S_score_weight_b <= S_weight_b_next;
                        S_score_weight_f <= S_weight_f_next;
                        S_state          <= ST_DIV_START;
                    end
                end
                ST_DIV_START: S_state <= ST_DIV_WAIT;
                ST_DIV_WAIT: begin
                    if (S_div_done_b && S_div_done_f) begin
                        S_diff_square_reg     <= S_diff_square;
                        S_weight_acc          <= 40'd0;
                        S_weight_multiplicand <= {20'd0,S_score_weight_b};
                        S_weight_multiplier   <= S_score_weight_f;
                        S_mult_count          <= 6'd0;
                        S_state               <= ST_MUL_WEIGHT;
                    end
                end
                ST_MUL_WEIGHT: begin
                    S_weight_acc            <= S_weight_acc_next;
                    S_weight_multiplicand   <= S_weight_multiplicand << 1;
                    S_weight_multiplier     <= S_weight_multiplier >> 1;
                    if (S_mult_count == 6'd19) begin
                        S_score_acc          <= 58'd0;
                        S_score_multiplicand <= {18'd0,S_weight_acc_next};
                        S_score_multiplier   <= S_diff_square_reg;
                        S_mult_count         <= 6'd0;
                        S_state              <= ST_MUL_SCORE;
                    end else S_mult_count <= S_mult_count + 6'd1;
                end
                ST_MUL_SCORE: begin
                    S_score_acc            <= S_score_acc_next;
                    S_score_multiplicand   <= S_score_multiplicand << 1;
                    S_score_multiplier     <= S_score_multiplier >> 1;
                    if (S_mult_count == 6'd17) begin
                        S_score_reg  <= S_score_acc_next;
                        S_mult_count <= 6'd0;
                        S_state      <= ST_SCORE;
                    end else S_mult_count <= S_mult_count + 6'd1;
                end
                ST_SCORE: begin
                    if (S_score_reg > S_best_score) begin
                        S_best_score     <= S_score_reg;
                        S_best_threshold <= S_addr;
                    end
                    if (S_addr == 8'd254) begin
                        S_threshold_pending <= (S_score_reg > S_best_score) ? S_addr : S_best_threshold;
                        O_threshold_result  <= (S_score_reg > S_best_score) ? S_addr : S_best_threshold;
                        O_result_valid      <= 1'b1;
                        S_pending_valid     <= 1'b1;
                        S_addr              <= 8'd0;
                        S_state             <= ST_CLEAR;
                    end else begin
                        S_addr  <= S_addr + 8'd1;
                        S_state <= ST_SCAN_SET;
                    end
                end
                ST_CLEAR: begin
                    if (S_addr == 8'd255) begin
                        S_addr  <= 8'd0;
                        S_state <= ST_IDLE;
                    end else S_addr <= S_addr + 8'd1;
                end
                default: ;
            endcase
        end
    end
endmodule

// One histogram lane.  The read/modify/write bypass preserves exact counts
// when consecutive pixels in this lane have the same gray value.
module otsu_histogram_bank (
    input  wire        I_clk,
    input  wire        I_accum_enable,
    input  wire [7:0]  I_accum_addr,
    input  wire [7:0]  I_scan_addr,
    input  wire        I_clear_enable,
    input  wire [7:0]  I_clear_addr,
    output wire [19:0] O_scan_data
);
    reg        S_accum_valid_d;
    reg [7:0]  S_accum_addr_d;
    reg        S_last_write_valid;
    reg [7:0]  S_last_write_addr;
    reg [19:0] S_last_write_data;

    wire [7:0] S_read_addr = I_accum_enable ? I_accum_addr : I_scan_addr;
    wire [19:0] S_read_data;
    wire [19:0] S_old_count = (S_last_write_valid &&
                               (S_last_write_addr == S_accum_addr_d))
                              ? S_last_write_data : S_read_data;
    wire [19:0] S_new_count = S_old_count + 20'd1;
    wire S_write_accum = S_accum_valid_d && !I_clear_enable;
    wire S_write_enable = I_clear_enable || S_write_accum;
    wire [7:0] S_write_addr = I_clear_enable ? I_clear_addr : S_accum_addr_d;
    wire [19:0] S_write_data = I_clear_enable ? 20'd0 : S_new_count;

    always @(posedge I_clk) begin
        S_accum_valid_d <= I_accum_enable;
        S_accum_addr_d  <= I_accum_addr;
        if (I_clear_enable) begin
            S_last_write_valid <= 1'b0;
        end else if (S_write_accum) begin
            S_last_write_valid <= 1'b1;
            S_last_write_addr  <= S_accum_addr_d;
            S_last_write_data  <= S_new_count;
        end
    end

    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
        .ADDR_WIDTH_A(8),.ADDR_WIDTH_B(8),
        .DATA_DEPTH_A(256),.DATA_DEPTH_B(256),
        .BYTE_ENABLE(0),.BYTE_A(1),.BYTE_B(1),
        .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
        .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
        .IMPLEMENT("20K")
    ) u_hist_eram (
        .dia(20'd0),.addra(S_read_addr),.cea(1'b1),.ocea(1'b1),
        .clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b1),
        .dib(S_write_data),.addrb(S_write_addr),.ceb(1'b1),.oceb(1'b1),
        .clkb(I_clk),.web(S_write_enable),.rstb(1'b0),.beb(1'b1),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),
        .doa(S_read_data),.dob(),.ecc_sbiterr(),.ecc_dbiterr()
    );
    assign O_scan_data = S_read_data;
endmodule

// 28-bit by 20-bit restoring divider.  One quotient is produced in 28 clocks.
module otsu_unsigned_divider (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_start,
    input  wire [27:0] I_numerator,
    input  wire [19:0] I_denominator,
    output reg  [27:0] O_quotient,
    output reg         O_done
);
    reg [27:0] S_dividend;
    reg [20:0] S_remainder;
    reg [19:0] S_divisor;
    reg [5:0]  S_count;
    reg        S_busy;
    wire [20:0] S_shifted_remainder = {S_remainder[19:0],S_dividend[27]};
    wire S_subtract = (S_shifted_remainder >= {1'b0,S_divisor});
    wire [20:0] S_remainder_next = S_subtract
                                  ? (S_shifted_remainder-{1'b0,S_divisor})
                                  : S_shifted_remainder;
    wire [27:0] S_dividend_next = {S_dividend[26:0],S_subtract};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_dividend  <= 28'd0;
            S_remainder <= 21'd0;
            S_divisor   <= 20'd1;
            S_count     <= 6'd0;
            S_busy      <= 1'b0;
            O_quotient  <= 28'd0;
            O_done      <= 1'b0;
        end else begin
            O_done <= 1'b0;
            if (I_start && !S_busy) begin
                S_dividend  <= I_numerator;
                S_remainder <= 21'd0;
                S_divisor   <= I_denominator;
                S_count     <= 6'd0;
                S_busy      <= 1'b1;
            end else if (S_busy) begin
                S_remainder <= S_remainder_next;
                S_dividend  <= S_dividend_next;
                if (S_count == 6'd27) begin
                    O_quotient <= S_dividend_next;
                    O_done     <= 1'b1;
                    S_busy     <= 1'b0;
                end else S_count <= S_count + 6'd1;
            end
        end
    end
endmodule
