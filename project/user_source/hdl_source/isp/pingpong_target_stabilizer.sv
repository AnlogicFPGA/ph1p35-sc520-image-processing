// Frame-level association for the orange candidate.  It does not change the
// RGB mask; a new/distant candidate must repeat before replacing the target.
module pingpong_target_stabilizer #(
    parameter [11:0] MAX_STEP_X = 12'd96,
    parameter [11:0] MAX_STEP_Y = 12'd72
)(
    input wire I_clk, I_rst_n, I_enable, I_publish,
    input wire I_candidate_valid,
    input wire [2:0] I_class_id,
    input wire [11:0] I_x_min, I_x_max, I_y_min, I_y_max,
    input wire [11:0] I_center_x, I_center_y,
    output reg O_target_valid, O_fresh,
    output reg [2:0] O_class_id,
    output reg [11:0] O_x_min, O_x_max, O_y_min, O_y_max,
    output reg [11:0] O_center_x, O_center_y
);
    reg pending_valid;
    reg [2:0] pending_class_id;
    reg [11:0] pending_x, pending_y;
    reg missed_once;
    reg [1:0] warm_flip_count;

    wire [12:0] new_w = {1'b0,I_x_max}-{1'b0,I_x_min}+13'd1;
    wire [12:0] new_h = {1'b0,I_y_max}-{1'b0,I_y_min}+13'd1;
    wire [12:0] old_w = {1'b0,O_x_max}-{1'b0,O_x_min}+13'd1;
    wire [12:0] old_h = {1'b0,O_y_max}-{1'b0,O_y_min}+13'd1;
    wire near_old = {1'b0,I_center_x} <= {1'b0,O_center_x}+{1'b0,MAX_STEP_X} &&
                    {1'b0,O_center_x} <= {1'b0,I_center_x}+{1'b0,MAX_STEP_X} &&
                    {1'b0,I_center_y} <= {1'b0,O_center_y}+{1'b0,MAX_STEP_Y} &&
                    {1'b0,O_center_y} <= {1'b0,I_center_y}+{1'b0,MAX_STEP_Y};
    wire near_pending = {1'b0,I_center_x} <= {1'b0,pending_x}+{1'b0,MAX_STEP_X} &&
                        {1'b0,pending_x} <= {1'b0,I_center_x}+{1'b0,MAX_STEP_X} &&
                        {1'b0,I_center_y} <= {1'b0,pending_y}+{1'b0,MAX_STEP_Y} &&
                        {1'b0,pending_y} <= {1'b0,I_center_y}+{1'b0,MAX_STEP_Y};
    wire size_consistent = {1'b0,new_w,1'b0} >= {1'b0,old_w} &&
                           {1'b0,old_w,1'b0} >= {1'b0,new_w} &&
                           {1'b0,new_h,1'b0} >= {1'b0,old_h} &&
                           {1'b0,old_h,1'b0} >= {1'b0,new_h};
    wire warm_pair = (I_class_id==3'd1 || I_class_id==3'd3) &&
                     (O_class_id==3'd1 || O_class_id==3'd3);
    wire keep_target = O_target_valid &&
                       ((I_class_id==O_class_id) || warm_pair) &&
                       near_old && size_consistent;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_target_valid<=0; O_fresh<=0;
            O_x_min<=0; O_x_max<=0; O_y_min<=0; O_y_max<=0;
            O_center_x<=0; O_center_y<=0;
            pending_valid<=0; pending_class_id<=0; pending_x<=0; pending_y<=0;
            O_class_id<=0;
            missed_once<=0; warm_flip_count<=0;
        end else if (!I_enable) begin
            O_target_valid<=0; O_fresh<=0;
            O_class_id<=0;
            pending_valid<=0; missed_once<=0; warm_flip_count<=0;
        end else if (I_publish) begin
            if (I_candidate_valid &&
                (keep_target || (pending_valid && I_class_id==pending_class_id &&
                                  near_pending && (!O_target_valid || size_consistent)))) begin
                O_target_valid<=1; O_fresh<=1;
                if (warm_pair && I_class_id!=O_class_id &&
                    warm_flip_count<2'd2) begin
                    warm_flip_count<=warm_flip_count+2'd1;
                end else begin
                    O_class_id<=I_class_id;
                    warm_flip_count<=0;
                end
                O_x_min<=I_x_min; O_x_max<=I_x_max;
                O_y_min<=I_y_min; O_y_max<=I_y_max;
                O_center_x<=I_center_x; O_center_y<=I_center_y;
                pending_valid<=0; missed_once<=0;
            end else begin
                O_fresh<=0;
                warm_flip_count<=0;
                if (I_candidate_valid) begin
                    pending_valid<=1;
                    pending_class_id<=I_class_id;
                    pending_x<=I_center_x; pending_y<=I_center_y;
                end else pending_valid<=0;
                if (O_target_valid && !missed_once)
                    missed_once<=1;
                else begin
                    O_target_valid<=0;
                    O_class_id<=0;
                    missed_once<=0;
                    if (O_target_valid) pending_valid<=0;
                end
            end
        end
    end
endmodule
