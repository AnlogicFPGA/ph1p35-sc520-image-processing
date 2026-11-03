// Color/shape candidate detector. RGB is sampled before display
// enhancements and OSD. Threshold selection depends on SC520 calibration,
// scene illumination, and target/background color contrast.
module pingpong_orange_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0] RED_OVER_GREEN = 8'd48,
    parameter integer COLOR_KIND = 0, // 0 warm (red/orange), 1 blue
    parameter integer SHAPE_KIND = 0, // 0 round, 1 square face
    parameter integer SC520_RELATIVE_COLOR = 0,
    parameter integer MIN_TARGET_SPAN = 16,
    parameter integer MIN_TARGET_SAMPLES = 8,
    parameter [2:0] CLASS_ID = 3'd1
)(
    input wire I_clk, I_rst_n, I_enable, I_frame_start, I_pixel_valid,
    input wire [11:0] I_x, I_y,
    input wire [23:0] I_rgb,
    output reg O_target_valid,
    output reg [11:0] O_x_min, O_x_max, O_y_min, O_y_max,
    output reg [11:0] O_center_x, O_center_y,
    output reg O_publish_pulse,
    output reg [2:0] O_class_id,
    output reg [2:0] O_diag_reason,
    output reg [15:0] O_diag_count
);
    localparam integer GROUPS = 4;
    localparam integer ROW_COUNT_W = $clog2((IMG_WIDTH+3)/4+1);
    wire [GROUPS-1:0] group_size_ok, group_fill_ok;
    reg [11:0] xlo [0:GROUPS-1], xhi [0:GROUPS-1];
    reg [11:0] ylo [0:GROUPS-1], yhi [0:GROUPS-1];
    reg [15:0] hits [0:GROUPS-1];
    reg [15:0] red_votes [0:GROUPS-1];
    reg [11:0] row_y [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] row_hits [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] first_row_hits [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] max_row_hits [0:GROUPS-1];
    reg [1:0] completed_rows [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] first_three_max [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] previous_row [0:GROUPS-1], previous_two_row [0:GROUPS-1];
    reg [ROW_COUNT_W-1:0] row_pairs [0:GROUPS-1],flat_pairs [0:GROUPS-1];
    wire [GROUPS-1:0] adjacent_flat,flat_profile_ok;
    reg [11:0] top_x [0:GROUPS-1],bottom_x [0:GROUPS-1],row_start_x [0:GROUPS-1];
    reg [11:0] left_y [0:GROUPS-1],right_y [0:GROUPS-1];
    reg [11:0] left_start_y [0:GROUPS-1],right_start_y [0:GROUPS-1];
    wire [7:0] red = I_rgb[23:16];
    wire [7:0] green = I_rgb[15:8];
    wire [7:0] blue = I_rgb[7:0];
    // One warm mask preserves the whole ball when shadows/white balance shift
    // parts of it between red and orange.  Classify the complete group later.
    // SC520 relative-color mask: dominant channel >=24,
    // then compare channel differences relative to that channel. Shifts avoid
    // a divider; fixed floors reject weak chroma near black.
    wire [7:0] red_blue_gap = red[7:3]<5 ? 8'd5 : {3'b0,red[7:3]};
    wire [7:0] blue_green_gap = blue[7:4]<2 ? 8'd2 : {4'b0,blue[7:4]};
    wire warm_hit = SC520_RELATIVE_COLOR ?
                    (red>=8'd24 &&
                     {1'b0,red}>={1'b0,green}+{3'b0,red[7:2]} &&
                     {1'b0,red}>={1'b0,blue}+{1'b0,red_blue_gap}) :
                    red >= 8'd48 &&
                    {1'b0,red} >= {1'b0,green}+{1'b0,RED_OVER_GREEN} &&
                    {1'b0,red} >= {1'b0,blue}+9'd32;
    wire blue_hit = SC520_RELATIVE_COLOR ?
                    (blue>=8'd24 &&
                     {1'b0,blue}>={1'b0,red}+{3'b0,blue[7:2]} &&
                     {1'b0,blue}>={1'b0,green}+{1'b0,blue_green_gap}) :
                    blue >= 8'd45 &&
                    {1'b0,blue} >= {1'b0,red}+9'd12 &&
                    {1'b0,blue} >= {1'b0,green}+9'd8;
    // An orange surface usually has substantially more green than blue.
    // Keep this vote separate from the broad warm mask so highlights do not
    // fragment the box while orange remains a distinct class.
    wire red_vote = SC520_RELATIVE_COLOR ?
                    (warm_hit && {1'b0,green}<={1'b0,blue}+{1'b0,red_blue_gap}) :
                    ({1'b0,red} >= {1'b0,green}+9'd56) &&
                    (({1'b0,green} <= {1'b0,blue}+9'd24) ||
                     ({1'b0,green,1'b0} <= {1'b0,red}));
    wire color_hit = COLOR_KIND==0 ? warm_hit : blue_hit;
    wire sample_hit = I_enable && I_pixel_valid && I_x < IMG_WIDTH &&
                      I_y < IMG_HEIGHT && I_x[1:0]==2'b00 &&
                      I_y[1:0]==2'b00 && color_hit;

    integer choose_slot;
    reg replace_slot;
    always @* begin
        choose_slot = -1;
        replace_slot = 1'b0;
        // Keep separated red objects apart; prefer an existing local group.
        for (integer j=0; j<GROUPS; j=j+1)
            if (choose_slot < 0 && hits[j]!=0 &&
                {1'b0,I_x}+13'd12 >= {1'b0,xlo[j]} &&
                {1'b0,I_y}+13'd12 >= {1'b0,ylo[j]} &&
                {1'b0,I_x} <= {1'b0,xhi[j]}+13'd12 &&
                {1'b0,I_y} <= {1'b0,yhi[j]}+13'd12)
                choose_slot = j;
        for (integer j=0; j<GROUPS; j=j+1)
            if (choose_slot < 0 && hits[j]==0) choose_slot = j;
        // Raster scanning cannot revisit a row. Once all slots are occupied,
        // Recycle completed specks and zero-thickness raster streaks. Use
        // equality here to keep the per-pixel slot selection path short.
        // A real size-qualified target spans both dimensions.
        for (integer j=0; j<GROUPS; j=j+1)
            if (choose_slot < 0 &&
                (hits[j]<16'd8 || yhi[j]==ylo[j] || xhi[j]==xlo[j]) &&
                {1'b0,I_y} > {1'b0,yhi[j]}+13'd12) begin
                choose_slot = j;
                replace_slot = 1'b1;
            end
    end

    reg eval_pending;
    reg [3:0] eval_ok;
    reg [3:0] eval_size_ok;
    reg [15:0] eval_hits [0:GROUPS-1];
    reg [25:0] eval_sample_area [0:GROUPS-1];
    reg [15:0] eval_red_votes [0:GROUPS-1];
    reg [11:0] eval_xlo [0:GROUPS-1], eval_xhi [0:GROUPS-1];
    reg [11:0] eval_ylo [0:GROUPS-1], eval_yhi [0:GROUPS-1];
    reg [11:0] eval_top_x [0:GROUPS-1],eval_bottom_x [0:GROUPS-1];
    reg [11:0] eval_left_y [0:GROUPS-1],eval_right_y [0:GROUPS-1];
    localparam integer ROTATION_EVAL = COLOR_KIND==1 && SHAPE_KIND==1 && SC520_RELATIVE_COLOR;
    reg [3:0] eval_delay;
    wire eval_done=ROTATION_EVAL ? eval_delay==1 : eval_pending;
    wire [3:0] issue_index=4'd11-eval_delay;
    wire [1:0] issue_slot=issue_index[1:0];
    wire signed [12:0] tb_dx=$signed({1'b0,eval_bottom_x[issue_slot]})-$signed({1'b0,eval_top_x[issue_slot]});
    wire signed [12:0] tb_dy=$signed({1'b0,eval_yhi[issue_slot]})-$signed({1'b0,eval_ylo[issue_slot]});
    wire signed [12:0] lr_dx=$signed({1'b0,eval_xhi[issue_slot]})-$signed({1'b0,eval_xlo[issue_slot]});
    wire signed [12:0] lr_dy=$signed({1'b0,eval_right_y[issue_slot]})-$signed({1'b0,eval_left_y[issue_slot]});
    wire signed [13:0] balance_x=$signed({2'b0,eval_top_x[issue_slot]})+
        $signed({2'b0,eval_bottom_x[issue_slot]})-$signed({2'b0,eval_xlo[issue_slot]})-
        $signed({2'b0,eval_xhi[issue_slot]});
    wire signed [13:0] balance_y=$signed({2'b0,eval_left_y[issue_slot]})+
        $signed({2'b0,eval_right_y[issue_slot]})-$signed({2'b0,eval_ylo[issue_slot]})-
        $signed({2'b0,eval_yhi[issue_slot]});
    wire [13:0] abs_balance_x=balance_x[13] ? -balance_x : balance_x;
    wire [13:0] abs_balance_y=balance_y[13] ? -balance_y : balance_y;
    reg mult_balanced,area_balanced;
    reg signed [25:0] cross_p,cross_q;
    reg [25:0] box_product;
    reg mult_valid,area_valid;
    reg [1:0] mult_slot,area_slot;
    wire signed [26:0] cross_diff=$signed({cross_p[25],cross_p})-$signed({cross_q[25],cross_q});
    reg [26:0] quad_area2;
    reg [25:0] area_box;
    wire [30:0] support_area2={10'b0,eval_hits[area_slot],5'b0};
    wire [30:0] quad_extended={4'b0,quad_area2};
    // Compare sampled support area to the quadrilateral through the four
    // directional extrema. A rotated square's extrema approach its corners;
    // a disk has more area outside that quadrilateral. This is a limited
    // shape descriptor, not a minimum-area box or general contour classifier.
    // Minimum box occupancy and opposite-point balance reject sparse masks
    // and triangles. Axis-aligned faces retain the row-profile path below.
    wire rotated_shape_ok=eval_size_ok[area_slot] && area_balanced &&
        {12'd0,eval_hits[area_slot],3'b0} >=
            (({5'b0,eval_sample_area[area_slot]}<<1)+{5'b0,eval_sample_area[area_slot]}) &&
        eval_xhi[area_slot]-eval_xlo[area_slot]>=32 &&
        eval_yhi[area_slot]-eval_ylo[area_slot]>=32 && area_box!=0 &&
        (quad_extended<<2)>=(({5'b0,area_box}<<1)+{5'b0,area_box}) &&
        (support_area2<<2)>=((quad_extended<<1)+quad_extended) &&
        ((support_area2<<1)+support_area2)<=(quad_extended<<2);
    wire [12:0] group_span_x [0:GROUPS-1];
    wire [12:0] group_span_y [0:GROUPS-1];
    wire [12:0] sample_w [0:GROUPS-1], sample_h [0:GROUPS-1];
    wire [25:0] sample_area [0:GROUPS-1];
    wire [19:0] hit_x8 [0:GROUPS-1];
    wire [12:0] widest_row [0:GROUPS-1];
    wire [12:0] first_edge [0:GROUPS-1], last_edge [0:GROUPS-1];
    wire [12:0] last_two_max [0:GROUPS-1];
    wire [GROUPS-1:0] square_edges_ok;
    genvar g;
    generate for (g=0; g<GROUPS; g=g+1) begin : g_span
        assign adjacent_flat[g]=previous_row[g]!=0 &&
            {1'b0,row_hits[g]}<={1'b0,previous_row[g]}+1'b1 &&
            {1'b0,previous_row[g]}<={1'b0,row_hits[g]}+1'b1;
        assign flat_profile_ok[g]=
            (({2'b0,flat_pairs[g]}+adjacent_flat[g])<<2) >=
            (({2'b0,row_pairs[g]}<<1)+{2'b0,row_pairs[g]});
        assign group_span_x[g]={1'b0,xhi[g]}-{1'b0,xlo[g]}+13'd1;
        assign group_span_y[g]={1'b0,yhi[g]}-{1'b0,ylo[g]}+13'd1;
        assign sample_w[g]=(group_span_x[g]+13'd3)>>2;
        assign sample_h[g]=(group_span_y[g]+13'd3)>>2;
        assign sample_area[g]=sample_w[g]*sample_h[g];
        assign hit_x8[g]={1'b0,hits[g],3'b0};
        assign widest_row[g]=max_row_hits[g]>row_hits[g] ?
                              max_row_hits[g] : row_hits[g];
        assign first_edge[g]=(completed_rows[g]<3 && row_hits[g]>first_three_max[g]) ?
                              row_hits[g] : first_three_max[g];
        assign last_two_max[g]=previous_row[g]>previous_two_row[g] ?
                               previous_row[g] : previous_two_row[g];
        assign last_edge[g]=row_hits[g]>last_two_max[g] ? row_hits[g] : last_two_max[g];
        // Ignore one-pixel corner/noise rows. A weak-density square must have
        // nearly full-width support near both ends, not just one bright row.
        assign square_edges_ok[g]=SC520_RELATIVE_COLOR ?
            (({first_edge[g],3'b0} >= (({3'b0,widest_row[g]}<<3)-{3'b0,widest_row[g]})) &&
             ({last_edge[g],3'b0} >= (({3'b0,widest_row[g]}<<3)-{3'b0,widest_row[g]}))) :
            ({1'b0,first_row_hits[g],1'b0} >= {2'b0,widest_row[g]} &&
             {1'b0,row_hits[g],1'b0} >= {2'b0,widest_row[g]});
        // Filter each group before ranking; a rejected laser-sized group must
        // not become the winning candidate or suppress a separate real ball.
        assign group_size_ok[g]=hits[g]>=MIN_TARGET_SAMPLES &&
            group_span_x[g]>=MIN_TARGET_SPAN && group_span_y[g]>=MIN_TARGET_SPAN &&
            group_span_x[g]<=13'd720 && group_span_y[g]<=13'd720 &&
            group_span_x[g]<=(group_span_y[g]+(group_span_y[g]>>1)) &&
            group_span_y[g]<=(group_span_x[g]+(group_span_x[g]>>1));
        assign group_fill_ok[g]=SHAPE_KIND==0 ?
            ((!SC520_RELATIVE_COLOR || (hits[g]>=24 &&
               {7'd0,hit_x8[g]} >= (sample_area[g]<<1))) &&
             // The 4x4 lattice can make a small disk's sampled bounding box
             // >7/8 full (r=24: 109/121). Retain short end rows, but allow
             // 15/16 occupancy for calibrated SC520 round candidates.
             (SC520_RELATIVE_COLOR ?
                 {11'd0,hits[g],4'b0} <= ((sample_area[g]<<4)-sample_area[g]) :
                 {7'd0,hit_x8[g]} <= ((sample_area[g]<<3)-sample_area[g])) &&
             {first_row_hits[g],2'b0} <= ({2'b0,widest_row[g]}+{1'b0,widest_row[g],1'b0}) &&
             {row_hits[g],2'b0} <= ({2'b0,widest_row[g]}+{1'b0,widest_row[g],1'b0})) :
            // A shaded square face can be below 7/8 density. The weaker
            // path also needs broad first and last rows, unlike a blue ball.
            ((!SC520_RELATIVE_COLOR || flat_profile_ok[g]) &&
             (({7'd0,hit_x8[g]} >= ((sample_area[g]<<3)-sample_area[g])) ||
             ({7'd0,hit_x8[g]} >= ((sample_area[g]<<2)+(sample_area[g]>>1)) &&
              square_edges_ok[g])));
    end endgenerate
    reg candidate_valid;
    reg [1:0] candidate_slot;
    reg [15:0] candidate_hits;
    reg [16:0] total_hits;
    always @* begin
        candidate_valid=1'b0; candidate_slot=2'd0; candidate_hits=16'd0;
        total_hits=17'd0;
        for (integer j=0; j<GROUPS; j=j+1) begin
            total_hits=total_hits+{1'b0,hits[j]};
            if (eval_ok[j] &&
                    (!candidate_valid || eval_hits[j]>candidate_hits)) begin
                    candidate_valid=1'b1;
                    candidate_slot=j[1:0];
                    candidate_hits=eval_hits[j];
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_target_valid<=1'b0; O_publish_pulse<=1'b0;
            O_x_min<=0; O_x_max<=0; O_y_min<=0; O_y_max<=0;
            O_center_x<=0; O_center_y<=0;
            O_diag_reason<=3'd0; O_diag_count<=16'd0;
            O_class_id<=3'd0;
            eval_pending<=0; eval_ok<=0; eval_size_ok<=0;
            eval_delay<=0;mult_valid<=0;area_valid<=0;
            cross_p<=0;cross_q<=0;box_product<=0;mult_slot<=0;
            mult_balanced<=0;area_balanced<=0;
            quad_area2<=0;area_box<=0;area_slot<=0;
            for (integer j=0; j<GROUPS; j=j+1) begin
                xlo[j]<=0; xhi[j]<=0; ylo[j]<=0; yhi[j]<=0; hits[j]<=0;
                red_votes[j]<=0; eval_red_votes[j]<=0;
                row_y[j]<=0; row_hits[j]<=0;
                first_row_hits[j]<=0; max_row_hits[j]<=0;
                completed_rows[j]<=0; first_three_max[j]<=0;
                previous_row[j]<=0; previous_two_row[j]<=0;
                row_pairs[j]<=0;flat_pairs[j]<=0;
                top_x[j]<=0;bottom_x[j]<=0;row_start_x[j]<=0;left_y[j]<=0;right_y[j]<=0;
                left_start_y[j]<=0;right_start_y[j]<=0;
                eval_top_x[j]<=0;eval_bottom_x[j]<=0;eval_left_y[j]<=0;eval_right_y[j]<=0;
                eval_hits[j]<=0;eval_sample_area[j]<=0; eval_xlo[j]<=0; eval_xhi[j]<=0;
                eval_ylo[j]<=0; eval_yhi[j]<=0;
            end
        end else begin
            O_publish_pulse<=1'b0;
            if (eval_done) begin
                O_target_valid<=I_enable && candidate_valid;
                O_class_id<=I_enable && candidate_valid ?
                    (COLOR_KIND==0 ?
                        ((({3'b0,eval_red_votes[candidate_slot]}<<2)+
                          {3'b0,eval_red_votes[candidate_slot]}) >=
                         ({3'b0,eval_hits[candidate_slot]}<<1) ? 3'd3 : 3'd1) : CLASS_ID) : 3'd0;
                O_publish_pulse<=I_enable;
                O_diag_reason<=!I_enable ? 3'd0 :
                               O_diag_count<16'd8 ? 3'd1 :
                               candidate_valid ? 3'd7 :
                               (|eval_size_ok) ? 3'd6 : 3'd3;
                if (I_enable && candidate_valid) begin
                    O_x_min<=eval_xlo[candidate_slot]; O_x_max<=eval_xhi[candidate_slot];
                    O_y_min<=eval_ylo[candidate_slot]; O_y_max<=eval_yhi[candidate_slot];
                    O_center_x<=(eval_xlo[candidate_slot]+eval_xhi[candidate_slot])>>1;
                    O_center_y<=(eval_ylo[candidate_slot]+eval_yhi[candidate_slot])>>1;
                end
            end
            eval_pending<=I_frame_start && I_enable;
            if(ROTATION_EVAL) begin
                if(!I_enable) begin eval_delay<=0;mult_valid<=0;area_valid<=0;end
                else begin
                    if(eval_delay!=0) eval_delay<=eval_delay-1'b1;
                    mult_valid<=eval_delay>=8 && eval_delay<=11;
                    if(eval_delay>=8 && eval_delay<=11) begin
                        cross_p<=tb_dx*lr_dy;cross_q<=tb_dy*lr_dx;
                        box_product<=tb_dy*lr_dx;mult_slot<=issue_slot;
                        mult_balanced<=abs_balance_x<=($unsigned(lr_dx)>>1) &&
                                       abs_balance_y<=($unsigned(tb_dy)>>1);
                    end
                    area_valid<=mult_valid;
                    if(mult_valid) begin
                        quad_area2<=cross_diff[26] ? -cross_diff : cross_diff;
                        area_box<=box_product;area_slot<=mult_slot;
                        area_balanced<=mult_balanced;
                    end
                    if(area_valid && rotated_shape_ok) eval_ok[area_slot]<=1'b1;
                end
            end
            if (I_frame_start) begin
                if(ROTATION_EVAL) begin eval_delay<=I_enable ? 11 : 0;mult_valid<=0;area_valid<=0;end
                O_diag_count<=total_hits[16] ? 16'hffff : total_hits[15:0];
                for (integer j=0; j<GROUPS; j=j+1) begin
                    eval_size_ok[j]<=group_size_ok[j];
                    eval_ok[j]<=group_size_ok[j] && group_fill_ok[j];
                    eval_hits[j]<=hits[j];
                    eval_sample_area[j]<=sample_area[j];
                    eval_red_votes[j]<=red_votes[j];
                    eval_xlo[j]<=xlo[j]; eval_xhi[j]<=xhi[j];
                    eval_ylo[j]<=ylo[j]; eval_yhi[j]<=yhi[j];
                    eval_top_x[j]<=top_x[j];eval_bottom_x[j]<=bottom_x[j];
                    eval_left_y[j]<=left_y[j];eval_right_y[j]<=right_y[j];
                    xlo[j]<=0; xhi[j]<=0; ylo[j]<=0; yhi[j]<=0; hits[j]<=0;
                    red_votes[j]<=0;
                    row_y[j]<=0; row_hits[j]<=0;
                    first_row_hits[j]<=0; max_row_hits[j]<=0;
                    completed_rows[j]<=0; first_three_max[j]<=0;
                    previous_row[j]<=0; previous_two_row[j]<=0;
                    row_pairs[j]<=0;flat_pairs[j]<=0;
                    top_x[j]<=0;bottom_x[j]<=0;row_start_x[j]<=0;left_y[j]<=0;right_y[j]<=0;
                    left_start_y[j]<=0;right_start_y[j]<=0;
                end
            end else if (sample_hit && choose_slot>=0) begin
                for (integer j=0; j<GROUPS; j=j+1) if (choose_slot==j) begin
                    if (hits[j]==0 || replace_slot) begin
                        xlo[j]<=I_x; xhi[j]<=I_x; ylo[j]<=I_y; yhi[j]<=I_y;
                        hits[j]<=16'd1;
                        red_votes[j]<=red_vote ? 16'd1 : 16'd0;
                        row_y[j]<=I_y; row_hits[j]<=13'd1;
                        first_row_hits[j]<=0; max_row_hits[j]<=0;
                        completed_rows[j]<=0; first_three_max[j]<=0;
                        previous_row[j]<=0; previous_two_row[j]<=0;
                        row_pairs[j]<=0;flat_pairs[j]<=0;
                        top_x[j]<=I_x;bottom_x[j]<=I_x;row_start_x[j]<=I_x;
                        left_y[j]<=I_y;right_y[j]<=I_y;
                        left_start_y[j]<=I_y;right_start_y[j]<=I_y;
                    end else begin
                        if (I_x<xlo[j]) begin xlo[j]<=I_x;left_y[j]<=I_y;left_start_y[j]<=I_y;end
                        else if(I_x==xlo[j]) left_y[j]<=({1'b0,left_start_y[j]}+{1'b0,I_y})>>1;
                        if (I_x>xhi[j]) begin xhi[j]<=I_x;right_y[j]<=I_y;right_start_y[j]<=I_y;end
                        else if(I_x==xhi[j]) right_y[j]<=({1'b0,right_start_y[j]}+{1'b0,I_y})>>1;
                        if(I_y==ylo[j]) top_x[j]<=({1'b0,xlo[j]}+{1'b0,I_x})>>1;
                        if (I_y<ylo[j]) ylo[j]<=I_y;
                        if (I_y>yhi[j]) yhi[j]<=I_y;
                        if (I_y==row_y[j]) begin
                            bottom_x[j]<=({1'b0,row_start_x[j]}+{1'b0,I_x})>>1;
                            if (row_hits[j]!={ROW_COUNT_W{1'b1}})
                                row_hits[j]<=row_hits[j]+13'd1;
                        end else begin
                            if(row_pairs[j]!={ROW_COUNT_W{1'b1}}) row_pairs[j]<=row_pairs[j]+1'b1;
                            if(adjacent_flat[j] && flat_pairs[j]!={ROW_COUNT_W{1'b1}})
                                flat_pairs[j]<=flat_pairs[j]+1'b1;
                            row_start_x[j]<=I_x;bottom_x[j]<=I_x;
                            if(completed_rows[j]<3) begin
                                completed_rows[j]<=completed_rows[j]+1'b1;
                                if(row_hits[j]>first_three_max[j]) first_three_max[j]<=row_hits[j];
                            end
                            previous_two_row[j]<=previous_row[j];
                            previous_row[j]<=row_hits[j];
                            if (first_row_hits[j]==0)
                                first_row_hits[j]<=row_hits[j];
                            if (row_hits[j]>max_row_hits[j])
                                max_row_hits[j]<=row_hits[j];
                            row_y[j]<=I_y;
                            row_hits[j]<=13'd1;
                        end
                        if (hits[j]!=16'hffff) begin
                            hits[j]<=hits[j]+16'd1;
                            if (red_vote) red_votes[j]<=red_votes[j]+16'd1;
                        end
                    end
                end
            end
        end
    end
endmodule
