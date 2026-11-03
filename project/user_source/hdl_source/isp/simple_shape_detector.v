// Single dark silhouette on a white background. Sample one pixel per 4x4 tile.
// All coordinates refer to the displayed image BEFORE any OSD is applied.
// One global region is measured; separate objects require connected-component labeling.
// Class: 0=no target, 1=circle, 2=square, 3=upright triangle, 4=unknown.
module simple_shape_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer ROI_X_MIN = 160,
    parameter integer ROI_X_MAX = 1120, // exclusive
    parameter integer ROI_Y_MIN = 160,
    parameter integer ROI_Y_MAX = 680,  // exclusive
    parameter [7:0] DARK_THRESHOLD = 8'd96,
    parameter integer MIN_ROW_PIXELS = 3,
    parameter integer MIN_SIZE = 40, // sampled pixels, in both dimensions
    parameter integer MIN_AREA = 400,
    parameter integer STABLE_FRAMES = 3
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_enable,
    input wire I_frame_start,
    input wire I_valid,
    input wire [11:0] I_x,
    input wire [11:0] I_y,
    input wire [23:0] I_rgb,
    output reg O_target_valid,
    output reg [2:0] O_class,
    output reg [11:0] O_x_min, O_x_max, O_y_min, O_y_max,
    output reg [15:0] O_center_x_bcd, O_center_y_bcd,
    // 0=WAIT, 1=NONE, 2=AREA, 3=SIZE, 4=ROWS, 5=BORD,
    // 6=STAB (three-frame confirmation), 7=OK.
    output reg [2:0] O_diag_reason
);
    // Width/height must be multiples of four. Sample coordinates fit 9/8 bits,
    // and sampled area must fit 16 bits; the integrated target is 1280x720.
    wire [9:0] luma_sum = {2'b0,I_rgb[23:16]} + {1'b0,I_rgb[15:8],1'b0}
                       + {2'b0,I_rgb[7:0]};
    wire sample_dark = I_valid && I_x[1:0]==0 && I_y[1:0]==0
                     && I_x>=ROI_X_MIN && I_x<ROI_X_MAX
                     && I_y>=ROI_Y_MIN && I_y<ROI_Y_MAX
                     && luma_sum[9:2]<DARK_THRESHOLD;
    wire row_end = I_valid && I_x==IMG_WIDTH-1 && I_y[1:0]==0;
    wire frame_end = I_valid && I_x==IMG_WIDTH-1 && I_y==IMG_HEIGHT-1;

    reg [9:0] row_count;
    reg [8:0] row_min, row_max;
    reg [15:0] area;
    reg [24:0] sum_y;
    reg [8:0] x_min, x_max;
    reg [7:0] y_min, y_max;
    reg have_region;
    reg [7:0] rows_used;
    wire [17:0] row_moment = row_count * I_y[9:2];

    reg [3:0] calc_state;
    reg [15:0] calc_area;
    reg [24:0] calc_sum_y;
    reg [8:0] calc_x_min, calc_x_max;
    reg [7:0] calc_y_min, calc_y_max;
    reg calc_have;
    reg [7:0] calc_rows;
    reg [9:0] box_width;
    reg [8:0] box_height;
    reg [18:0] box_area;
    reg [24:0] origin_moment;
    reg [24:0] relative_moment;
    reg [24:0] area_height;
    reg [2:0] calc_class;
    reg calc_valid;
    reg [2:0] calc_reason, pending_reason;
    reg [11:0] center_x, center_y;
    reg [15:0] bcd_x, bcd_y;
    reg [11:0] shift_x, shift_y;
    reg [3:0] bcd_step;
    reg pending;
    reg pending_valid;
    reg [2:0] pending_class;
    reg [11:0] pending_x_min, pending_x_max, pending_y_min, pending_y_max;
    reg [15:0] pending_bcd_x, pending_bcd_y;
    reg [2:0] previous_class;
    reg [7:0] streak;

    // Widen BEFORE shifting; no division is required for the feature tests.
    wire [29:0] fill16 = {14'b0,calc_area} << 4;
    // Explicit shifts/adds prevent TD from allocating DSPs to constant scales.
    wire [29:0] box_ext = {11'b0,box_area};
    wire [29:0] box6 = (box_ext<<2)+(box_ext<<1);
    wire [29:0] box10 = (box_ext<<3)+(box_ext<<1);
    wire [29:0] box11 = box10+box_ext;
    wire [29:0] box14 = (box_ext<<3)+(box_ext<<2)+(box_ext<<1);
    wire [29:0] moment16 = {5'b0,relative_moment} << 4;
    wire [29:0] height_ext = {5'b0,area_height};
    wire [29:0] height6 = (height_ext<<2)+(height_ext<<1);
    wire [29:0] height9 = (height_ext<<3)+height_ext;
    wire [29:0] height10 = (height_ext<<3)+(height_ext<<1);
    wire [29:0] height13 = (height_ext<<3)+(height_ext<<2)+height_ext;
    wire [12:0] width_ext = {3'b0,box_width};
    wire [12:0] row_height_ext = {4'b0,box_height};
    wire [12:0] width3 = (width_ext<<1)+width_ext;
    wire [12:0] row_height3 = (row_height_ext<<1)+row_height_ext;
    wire near_square = ((width_ext<<2) >= row_height3)
                     && (width3 <= (row_height_ext<<2));
    wire [12:0] width5 = (width_ext<<2)+width_ext;
    wire triangle_aspect = width5>=row_height3 && width5<=(row_height_ext<<3);
    wire balanced = moment16>=height6 && moment16<=height10;

    function [15:0] bcd_adjust;
        input [15:0] value;
        integer digit;
        begin
            bcd_adjust=value;
            for(digit=0;digit<4;digit=digit+1)
                if(value[digit*4 +: 4]>=5)
                    bcd_adjust[digit*4 +: 4]=value[digit*4 +: 4]+4'd3;
        end
    endfunction

    always @(posedge I_clk) begin
        if(!I_rst_n) begin
            row_count<=0; row_min<=9'h1ff; row_max<=0;
            area<=0; sum_y<=0; x_min<=9'h1ff; x_max<=0;
            y_min<=8'hff; y_max<=0; have_region<=0; rows_used<=0;
            calc_state<=0; pending<=0; previous_class<=0; streak<=0;
            O_target_valid<=0; O_class<=0;
            O_x_min<=0; O_x_max<=0; O_y_min<=0; O_y_max<=0;
            O_center_x_bcd<=0; O_center_y_bcd<=0;
            O_diag_reason<=0; calc_reason<=0; pending_reason<=0;
            calc_area<=0; calc_sum_y<=0; calc_x_min<=0; calc_x_max<=0;
            calc_y_min<=0; calc_y_max<=0; calc_have<=0; calc_rows<=0;
            box_width<=0; box_height<=0; box_area<=0; origin_moment<=0;
            relative_moment<=0; area_height<=0; calc_class<=0; calc_valid<=0;
            center_x<=0; center_y<=0; bcd_x<=0; bcd_y<=0;
            shift_x<=0; shift_y<=0; bcd_step<=0;
            pending_valid<=0; pending_class<=0;
            pending_x_min<=0; pending_x_max<=0; pending_y_min<=0; pending_y_max<=0;
            pending_bcd_x<=0; pending_bcd_y<=0;
        end else if(!I_enable) begin
            row_count<=0; row_min<=9'h1ff; row_max<=0;
            area<=0; sum_y<=0; x_min<=9'h1ff; x_max<=0;
            y_min<=8'hff; y_max<=0; have_region<=0; rows_used<=0;
            calc_state<=0; pending<=0; previous_class<=0; streak<=0;
            O_target_valid<=0; O_class<=0;
            O_diag_reason<=0;
        end else begin
            if(I_frame_start) begin
                area<=0; sum_y<=0; x_min<=9'h1ff; x_max<=0;
                y_min<=8'hff; y_max<=0; have_region<=0; rows_used<=0;
                row_count<=0; row_min<=9'h1ff; row_max<=0;
                // Publish one coherent result only at an output frame boundary.
                if(pending) begin
                    pending<=0;
                    O_diag_reason<=pending_valid ? 3'd6 : pending_reason;
                    if(!pending_valid) begin
                        O_target_valid<=0; O_class<=0;
                        previous_class<=0; streak<=0;
                    end else begin
                        previous_class<=pending_class;
                        if(pending_class==previous_class) begin
                            if(streak<STABLE_FRAMES) streak<=streak+1'b1;
                        end else streak<=1;
                        if(STABLE_FRAMES<=1 ||
                           (pending_class==previous_class && streak>=STABLE_FRAMES-1)) begin
                            O_target_valid<=1; O_class<=pending_class;
                            O_diag_reason<=3'd7;
                            O_x_min<=pending_x_min; O_x_max<=pending_x_max;
                            O_y_min<=pending_y_min; O_y_max<=pending_y_max;
                            O_center_x_bcd<=pending_bcd_x; O_center_y_bcd<=pending_bcd_y;
                        end else begin
                            O_target_valid<=0; O_class<=0;
                        end
                    end
                end
            end else begin
                if(sample_dark) begin
                    row_count<=row_count+1'b1;
                    if(I_x[10:2]<row_min) row_min<=I_x[10:2];
                    if(I_x[10:2]>row_max) row_max<=I_x[10:2];
                end
                if(row_end) begin
                    if(row_count>=MIN_ROW_PIXELS) begin
                        area<=area+row_count;
                        sum_y<=sum_y+row_moment;
                        if(row_min<x_min) x_min<=row_min;
                        if(row_max>x_max) x_max<=row_max;
                        if(!have_region) y_min<=I_y[9:2];
                        y_max<=I_y[9:2]; have_region<=1;
                        rows_used<=rows_used+1'b1;
                    end
                    row_count<=0; row_min<=9'h1ff; row_max<=0;
                end
            end
            if(frame_end && calc_state==0) begin
                calc_area<=area; calc_sum_y<=sum_y;
                calc_x_min<=x_min; calc_x_max<=x_max;
                calc_y_min<=y_min; calc_y_max<=y_max;
                calc_have<=have_region; calc_rows<=rows_used;
                calc_state<=1;
            end
            // The following arithmetic runs during vertical blanking.
            case(calc_state)
                1: begin
                    box_width<={1'b0,calc_x_max}-{1'b0,calc_x_min}+10'd1;
                    box_height<={1'b0,calc_y_max}-{1'b0,calc_y_min}+9'd1;
                    center_x<=({3'b0,calc_x_min}+{3'b0,calc_x_max})*12'd2+12'd1;
                    center_y<=({4'b0,calc_y_min}+{4'b0,calc_y_max})*12'd2+12'd1;
                    calc_state<=2;
                end
                2: begin
                    box_area<=box_width*box_height;
                    origin_moment<=calc_area*calc_y_min;
                    area_height<=calc_area*box_height;
                    calc_state<=3;
                end
                3: begin
                    relative_moment<=calc_sum_y-origin_moment;
                    calc_state<=4;
                end
                4: begin
                    calc_valid<=calc_have && calc_area>=MIN_AREA
                        && box_width>=MIN_SIZE && box_height>=MIN_SIZE
                        && calc_rows>=box_height-2
                        && calc_x_min>ROI_X_MIN/4 && calc_x_max<(ROI_X_MAX/4)-1
                        && calc_y_min>ROI_Y_MIN/4 && calc_y_max<(ROI_Y_MAX/4)-1;
                    if(!calc_have) calc_reason<=3'd1;
                    else if(calc_area<MIN_AREA) calc_reason<=3'd2;
                    else if(box_width<MIN_SIZE || box_height<MIN_SIZE)
                        calc_reason<=3'd3;
                    else if(calc_rows<box_height-2) calc_reason<=3'd4;
                    else if(calc_x_min<=ROI_X_MIN/4 || calc_x_max>=(ROI_X_MAX/4)-1 ||
                            calc_y_min<=ROI_Y_MIN/4 || calc_y_max>=(ROI_Y_MAX/4)-1)
                        calc_reason<=3'd5;
                    else calc_reason<=3'd6;
                    calc_class<=4;
                    if(near_square && balanced && fill16>=box14)
                        calc_class<=2;
                    else if(near_square && balanced && fill16>=box11 && fill16<box14)
                        calc_class<=1;
                    else if(triangle_aspect && fill16>=box6 && fill16<=box10
                            && moment16>=height9 && moment16<=height13)
                        calc_class<=3;
                    bcd_x<=0; bcd_y<=0; shift_x<=center_x; shift_y<=center_y;
                    bcd_step<=0; calc_state<=5;
                end
                5: begin
                    bcd_x<=(bcd_adjust(bcd_x)<<1)|shift_x[11];
                    bcd_y<=(bcd_adjust(bcd_y)<<1)|shift_y[11];
                    shift_x<=shift_x<<1; shift_y<=shift_y<<1;
                    if(bcd_step==11) calc_state<=6;
                    else bcd_step<=bcd_step+1'b1;
                end
                6: begin
                    pending<=1; pending_valid<=calc_valid; pending_class<=calc_class;
                    pending_reason<=calc_reason;
                    pending_x_min<={1'b0,calc_x_min,2'b00};
                    pending_x_max<={1'b0,calc_x_max,2'b11};
                    pending_y_min<={2'b0,calc_y_min,2'b00};
                    pending_y_max<={2'b0,calc_y_max,2'b11};
                    pending_bcd_x<=bcd_x; pending_bcd_y<=bcd_y;
                    calc_state<=0;
                end
                default: ;
            endcase
        end
    end
endmodule
