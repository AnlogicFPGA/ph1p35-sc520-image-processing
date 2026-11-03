// Class-free single-object motion box in the HDMI pixel domain.
// Consecutive *display* frames are compared; this is not a CSI source-frame ID.
// One 8-bit sample per 8x8 source pixels uses 160x90 bytes at 1280x720.
module motion_track_display #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer DIFF_THRESHOLD = 20,
    parameter integer MIN_CHANGED = 6,
    parameter integer MAX_CHANGED = 1200,
    parameter integer MAX_BOX_WIDTH = 640,
    parameter integer MAX_BOX_HEIGHT = 480,
    parameter integer MAX_STEP = 96,
    parameter integer HOLD_FRAMES = 3,
    parameter integer CLUSTER_GAP_X = 96,
    parameter integer CLUSTER_GAP_Y = 64,
    parameter integer MIN_POLARITY_CHANGED = 2
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_enable,
    input  wire        I_frame_start,
    input  wire        I_pixel_valid,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [23:0] I_rgb,
    output reg         O_track_valid,
    output reg         O_fresh,
    output reg  [11:0] O_x_min,
    output reg  [11:0] O_x_max,
    output reg  [11:0] O_y_min,
    output reg  [11:0] O_y_max,
    output reg  [11:0] O_center_x,
    output reg  [11:0] O_center_y,
    output reg         O_publish_pulse,
    output reg  [15:0] O_frame_bcd,
    // 0=WAIT, 1=LOW, 2=HIGH, 3=SIZE, 4=STEP, 5=OK, 6=QUAL.
    output reg  [2:0]  O_diag_reason,
    output reg  [15:0] O_diag_changed
);
    localparam integer GRID_WIDTH = IMG_WIDTH / 8;
    localparam integer GRID_HEIGHT = IMG_HEIGHT / 8;
    localparam integer GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;
    localparam integer ADDR_BITS = $clog2(GRID_SIZE);
    localparam integer CLUSTER_SLOTS = 4;
    // The display timing can omit a sampled edge column/row at the user
    // boundary. Allow at most one missing sampled row before accepting a
    // reference; a newly appearing address on a later frame remains a risk.
    localparam integer MIN_FRAME_SAMPLES = GRID_SIZE - GRID_WIDTH;

    // Read-before-write returns the previous displayed frame's sample.
    // The memory is not reset; comparisons are disabled until a full frame
    // has populated every sample address.
    reg [7:0] previous_gray [0:GRID_SIZE-1];
    reg [7:0] old_gray;
    reg [7:0] current_gray_d;
    reg [11:0] sample_x_d, sample_y_d;
    reg sample_d;
    reg [15:0] sample_count;
    reg have_previous_frame;

    reg [15:0] changed_count;
    // Four local motion groups keep unrelated changed samples from inflating
    // the single displayed box. An unmatched sample replaces the weakest group
    // if all slots are occupied; a dense object can therefore displace noise.
    reg [15:0] cluster_count [0:CLUSTER_SLOTS-1];
    reg [15:0] cluster_rise_count [0:CLUSTER_SLOTS-1];
    reg [15:0] cluster_fall_count [0:CLUSTER_SLOTS-1];
    reg [11:0] cluster_x_min [0:CLUSTER_SLOTS-1];
    reg [11:0] cluster_x_max [0:CLUSTER_SLOTS-1];
    reg [11:0] cluster_y_min [0:CLUSTER_SLOTS-1];
    reg [11:0] cluster_y_max [0:CLUSTER_SLOTS-1];
    // Freeze the completed frame before clearing the live accumulators. Scan
    // one group per pixel clock rather than selecting through four groups and
    // driving the OSD box in the same 75 MHz cycle.
    reg [15:0] snap_count [0:CLUSTER_SLOTS-1];
    reg [15:0] snap_rise_count [0:CLUSTER_SLOTS-1];
    reg [15:0] snap_fall_count [0:CLUSTER_SLOTS-1];
    reg [11:0] snap_x_min [0:CLUSTER_SLOTS-1];
    reg [11:0] snap_x_max [0:CLUSTER_SLOTS-1];
    reg [11:0] snap_y_min [0:CLUSTER_SLOTS-1];
    reg [11:0] snap_y_max [0:CLUSTER_SLOTS-1];
    reg [2:0] hold_count;
    integer slot;

    reg [1:0] update_slot;
    reg update_match;
    reg scan_active, select_pending, publish_pending;
    reg [1:0] scan_idx;
    reg [1:0] best_global_slot, best_nearby_slot;
    reg [15:0] best_global_count, best_nearby_count;
    reg saw_oversize_quality;
    reg frame_ready, frame_enabled;
    reg [15:0] frame_sample_count, frame_changed_count;
    reg [15:0] chosen_count;
    reg chosen_qualified;
    reg [11:0] chosen_x_min, chosen_x_max, chosen_y_min, chosen_y_max;

    wire sample = I_pixel_valid && I_x < IMG_WIDTH && I_y < IMG_HEIGHT &&
                  I_x[2:0] == 3'b000 && I_y[2:0] == 3'b000;
    wire [ADDR_BITS-1:0] sample_addr =
        ((I_y >> 3) * GRID_WIDTH) + (I_x >> 3);
    wire [9:0] gray_sum = {2'b00, I_rgb[23:16]} +
                          {1'b0, I_rgb[15:8], 1'b0} +
                          {2'b00, I_rgb[7:0]};
    wire [7:0] gray = gray_sum[9:2];
    wire [7:0] delta = current_gray_d >= old_gray ?
        current_gray_d - old_gray : old_gray - current_gray_d;
    wire changed = sample_d && have_previous_frame && I_enable &&
                   delta >= DIFF_THRESHOLD;
    wire changed_rising = current_gray_d > old_gray;
    // Compare all four groups in parallel. Keep the original first-match and
    // earliest-minimum tie rules without serially gating the minimum chain.
    wire [CLUSTER_SLOTS-1:0] cluster_match;
    generate for (genvar k = 0; k < CLUSTER_SLOTS; k = k+1) begin : g_match
            assign cluster_match[k] = cluster_count[k] != 0 &&
                ({1'b0, sample_x_d} + CLUSTER_GAP_X >= {1'b0, cluster_x_min[k]}) &&
                ({1'b0, sample_x_d} <= {1'b0, cluster_x_max[k]} + CLUSTER_GAP_X) &&
                ({1'b0, sample_y_d} + CLUSTER_GAP_Y >= {1'b0, cluster_y_min[k]}) &&
                ({1'b0, sample_y_d} <= {1'b0, cluster_y_max[k]} + CLUSTER_GAP_Y);
    end endgenerate
    wire min01_is0 = cluster_count[0] <= cluster_count[1];
    wire min23_is2 = cluster_count[2] <= cluster_count[3];
    wire [15:0] min01 = min01_is0 ? cluster_count[0] : cluster_count[1];
    wire [15:0] min23 = min23_is2 ? cluster_count[2] : cluster_count[3];
    wire [1:0] minimum_slot = min01 <= min23 ? {1'b0, !min01_is0} :
                                                           {1'b1, !min23_is2};
    always @* begin
        update_match = |cluster_match;
        if (cluster_match[0]) update_slot = 2'd0;
        else if (cluster_match[1]) update_slot = 2'd1;
        else if (cluster_match[2]) update_slot = 2'd2;
        else if (cluster_match[3]) update_slot = 2'd3;
        else update_slot = minimum_slot;
    end

    wire [11:0] scan_cx = ({1'b0, snap_x_min[scan_idx]} +
                           {1'b0, snap_x_max[scan_idx]}) >> 1;
    wire [11:0] scan_cy = ({1'b0, snap_y_min[scan_idx]} +
                           {1'b0, snap_y_max[scan_idx]}) >> 1;
    wire [11:0] scan_dx = scan_cx >= O_center_x ? scan_cx - O_center_x :
                          O_center_x - scan_cx;
    wire [11:0] scan_dy = scan_cy >= O_center_y ? scan_cy - O_center_y :
                          O_center_y - scan_cy;
    wire [11:0] scan_span_x = snap_x_max[scan_idx] - snap_x_min[scan_idx];
    wire [11:0] scan_span_y = snap_y_max[scan_idx] - snap_y_min[scan_idx];
    // A moving object normally reveals background at its old position and
    // covers background at its new position. A one-sided brightness step is
    // deliberately held out of the class-free motion candidates.
    wire scan_quality = snap_count[scan_idx] >= MIN_CHANGED &&
                          snap_rise_count[scan_idx] >= MIN_POLARITY_CHANGED &&
                          snap_fall_count[scan_idx] >= MIN_POLARITY_CHANGED &&
                          {7'b0, scan_span_x} <= {snap_count[scan_idx], 3'b0} &&
                          {7'b0, scan_span_y} <= {snap_count[scan_idx], 3'b0};
    wire scan_fits_box = scan_span_x <= MAX_BOX_WIDTH &&
                         scan_span_y <= MAX_BOX_HEIGHT;
    wire scan_qualified = scan_quality && scan_fits_box;
    wire scan_nearby = scan_dx <= MAX_STEP && scan_dy <= MAX_STEP;
    wire choose_nearby = O_track_valid && hold_count < HOLD_FRAMES &&
                         best_nearby_count >= MIN_CHANGED;
    wire [1:0] select_slot = choose_nearby ? best_nearby_slot : best_global_slot;
    wire measured = frame_ready && frame_changed_count <= MAX_CHANGED &&
                    chosen_qualified &&
                    chosen_x_max - chosen_x_min <= MAX_BOX_WIDTH &&
                    chosen_y_max - chosen_y_min <= MAX_BOX_HEIGHT;
    wire [12:0] measured_x_sum = {1'b0, chosen_x_min} + {1'b0, chosen_x_max};
    wire [12:0] measured_y_sum = {1'b0, chosen_y_min} + {1'b0, chosen_y_max};
    wire [11:0] measured_cx = measured_x_sum[12:1];
    wire [11:0] measured_cy = measured_y_sum[12:1];
    wire [11:0] step_x = measured_cx >= O_center_x ?
        measured_cx - O_center_x : O_center_x - measured_cx;
    wire [11:0] step_y = measured_cy >= O_center_y ?
        measured_cy - O_center_y : O_center_y - measured_cy;
    wire associate = step_x <= MAX_STEP && step_y <= MAX_STEP;

    // Decimal display-frame index supplied to the diagnostic OSD.
    // This counter is not a CSI source-frame identifier.
    function automatic [15:0] F_next_bcd(input [15:0] value);
        reg [15:0] next_value;
        begin
            next_value = value;
            if (value[3:0] < 4'd9)
                next_value[3:0] = value[3:0] + 4'd1;
            else begin
                next_value[3:0] = 4'd0;
                if (value[7:4] < 4'd9)
                    next_value[7:4] = value[7:4] + 4'd1;
                else begin
                    next_value[7:4] = 4'd0;
                    if (value[11:8] < 4'd9)
                        next_value[11:8] = value[11:8] + 4'd1;
                    else begin
                        next_value[11:8] = 4'd0;
                        next_value[15:12] = value[15:12] < 4'd9 ?
                                             value[15:12] + 4'd1 : 4'd0;
                    end
                end
            end
            F_next_bcd = next_value;
        end
    endfunction

    always @(posedge I_clk) begin
        sample_d <= sample;
        if (sample) begin
            old_gray <= previous_gray[sample_addr];
            previous_gray[sample_addr] <= gray;
            current_gray_d <= gray;
            sample_x_d <= I_x;
            sample_y_d <= I_y;
        end
    end

    always @(posedge I_clk) begin
        if (!I_rst_n) begin
            sample_count <= 16'd0;
            have_previous_frame <= 1'b0;
            changed_count <= 16'd0;
            for (slot = 0; slot < CLUSTER_SLOTS; slot = slot+1) begin
                cluster_count[slot] <= 16'd0;
                cluster_rise_count[slot] <= 16'd0;
                cluster_fall_count[slot] <= 16'd0;
                cluster_x_min[slot] <= 12'd0;
                cluster_x_max[slot] <= 12'd0;
                cluster_y_min[slot] <= 12'd0;
                cluster_y_max[slot] <= 12'd0;
                snap_count[slot] <= 16'd0;
                snap_rise_count[slot] <= 16'd0;
                snap_fall_count[slot] <= 16'd0;
                snap_x_min[slot] <= 12'd0;
                snap_x_max[slot] <= 12'd0;
                snap_y_min[slot] <= 12'd0;
                snap_y_max[slot] <= 12'd0;
            end
            scan_active <= 1'b0;
            select_pending <= 1'b0;
            publish_pending <= 1'b0;
            scan_idx <= 2'd0;
            best_global_slot <= 2'd0;
            best_nearby_slot <= 2'd0;
            best_global_count <= 16'd0;
            best_nearby_count <= 16'd0;
            saw_oversize_quality <= 1'b0;
            frame_ready <= 1'b0;
            frame_enabled <= 1'b0;
            frame_sample_count <= 16'd0;
            frame_changed_count <= 16'd0;
            chosen_count <= 16'd0;
            chosen_qualified <= 1'b0;
            chosen_x_min <= 12'd0; chosen_x_max <= 12'd0;
            chosen_y_min <= 12'd0; chosen_y_max <= 12'd0;
            hold_count <= 3'd0;
            O_track_valid <= 1'b0;
            O_fresh <= 1'b0;
            O_x_min <= 12'd0; O_x_max <= 12'd0;
            O_y_min <= 12'd0; O_y_max <= 12'd0;
            O_center_x <= 12'd0; O_center_y <= 12'd0;
            O_publish_pulse <= 1'b0;
            O_frame_bcd <= 16'h0000;
            O_diag_reason <= 3'd0;
            O_diag_changed <= 16'd0;
        end else begin
            O_publish_pulse <= 1'b0;
            if (I_frame_start) begin
                if (I_enable)
                    O_frame_bcd <= F_next_bcd(O_frame_bcd);
                O_fresh <= 1'b0;
                frame_ready <= have_previous_frame && sample_count >= MIN_FRAME_SAMPLES;
                frame_enabled <= I_enable;
                frame_sample_count <= sample_count;
                frame_changed_count <= changed_count;
                scan_active <= 1'b1;
                select_pending <= 1'b0;
                publish_pending <= 1'b0;
                scan_idx <= 2'd0;
                best_global_count <= 16'd0;
                best_nearby_count <= 16'd0;
                saw_oversize_quality <= 1'b0;
                best_global_slot <= 2'd0;
                best_nearby_slot <= 2'd0;
                have_previous_frame <= sample_count >= MIN_FRAME_SAMPLES;
                sample_count <= sample ? 16'd1 : 16'd0;
                changed_count <= 16'd0;
                for (slot = 0; slot < CLUSTER_SLOTS; slot = slot+1) begin
                    snap_count[slot] <= cluster_count[slot];
                    snap_rise_count[slot] <= cluster_rise_count[slot];
                    snap_fall_count[slot] <= cluster_fall_count[slot];
                    snap_x_min[slot] <= cluster_x_min[slot];
                    snap_x_max[slot] <= cluster_x_max[slot];
                    snap_y_min[slot] <= cluster_y_min[slot];
                    snap_y_max[slot] <= cluster_y_max[slot];
                    cluster_count[slot] <= 16'd0;
                    cluster_rise_count[slot] <= 16'd0;
                    cluster_fall_count[slot] <= 16'd0;
                    cluster_x_min[slot] <= 12'd0;
                    cluster_x_max[slot] <= 12'd0;
                    cluster_y_min[slot] <= 12'd0;
                    cluster_y_max[slot] <= 12'd0;
                end
            end else begin
                if (scan_active) begin
                    if (scan_quality && !scan_fits_box)
                        saw_oversize_quality <= 1'b1;
                    if (scan_qualified &&
                        snap_count[scan_idx] > best_global_count) begin
                        best_global_count <= snap_count[scan_idx];
                        best_global_slot <= scan_idx;
                    end
                    if (scan_qualified &&
                        snap_count[scan_idx] > best_nearby_count && scan_nearby) begin
                        best_nearby_count <= snap_count[scan_idx];
                        best_nearby_slot <= scan_idx;
                    end
                    if (scan_idx == CLUSTER_SLOTS-1) begin
                        scan_active <= 1'b0;
                        select_pending <= 1'b1;
                    end else
                        scan_idx <= scan_idx+2'd1;
                end else if (select_pending) begin
                    chosen_count <= snap_count[select_slot];
                    chosen_qualified <= choose_nearby ?
                        (best_nearby_count >= MIN_CHANGED) :
                        (best_global_count >= MIN_CHANGED);
                    chosen_x_min <= snap_x_min[select_slot];
                    chosen_x_max <= snap_x_max[select_slot];
                    chosen_y_min <= snap_y_min[select_slot];
                    chosen_y_max <= snap_y_max[select_slot];
                    select_pending <= 1'b0;
                    publish_pending <= 1'b1;
                end else if (publish_pending) begin
                    publish_pending <= 1'b0;
                    O_publish_pulse <= 1'b1;
                    // S counts received samples / 4 until a reference is ready.
                    O_diag_changed <= frame_ready ? frame_changed_count :
                                      (frame_sample_count >> 2);
                    if (!frame_enabled || !frame_ready)
                        O_diag_reason <= 3'd0;
                    else if (frame_changed_count > MAX_CHANGED)
                        O_diag_reason <= 3'd2;
                    else if (!chosen_qualified && saw_oversize_quality)
                        O_diag_reason <= 3'd3;
                    else if (chosen_count < MIN_CHANGED)
                        O_diag_reason <= 3'd1;
                    else if (!chosen_qualified)
                        O_diag_reason <= 3'd6;
                    else if (chosen_x_max - chosen_x_min > MAX_BOX_WIDTH ||
                             chosen_y_max - chosen_y_min > MAX_BOX_HEIGHT)
                        O_diag_reason <= 3'd3;
                    else if (O_track_valid && hold_count < HOLD_FRAMES && !associate)
                        O_diag_reason <= 3'd4;
                    else
                        O_diag_reason <= 3'd5;
                    if (!frame_enabled) begin
                        O_track_valid <= 1'b0;
                        hold_count <= 3'd0;
                    end else if (measured &&
                                 (!O_track_valid || hold_count >= HOLD_FRAMES || associate)) begin
                        O_track_valid <= 1'b1;
                        O_fresh <= 1'b1;
                        hold_count <= 3'd0;
                        O_x_min <= chosen_x_min >= 8 ? chosen_x_min-12'd8 : 12'd0;
                        O_x_max <= chosen_x_max+8 < IMG_WIDTH ?
                                   chosen_x_max+12'd8 : IMG_WIDTH-1;
                        O_y_min <= chosen_y_min >= 8 ? chosen_y_min-12'd8 : 12'd0;
                        O_y_max <= chosen_y_max+8 < IMG_HEIGHT ?
                                   chosen_y_max+12'd8 : IMG_HEIGHT-1;
                        O_center_x <= measured_cx;
                        O_center_y <= measured_cy;
                    end else if (O_track_valid) begin
                        if (hold_count >= HOLD_FRAMES) begin
                            O_track_valid <= 1'b0;
                            hold_count <= 3'd0;
                        end else
                            hold_count <= hold_count+3'd1;
                    end
                end
                if (sample && sample_count < GRID_SIZE)
                    sample_count <= sample_count+16'd1;
                if (changed) begin
                    if (changed_count != 16'hffff)
                        changed_count <= changed_count+16'd1;
                    // Constant-index feedback avoids a second dynamic read
                    // mux between group selection and each counter update.
                    for (integer k = 0; k < CLUSTER_SLOTS; k = k+1) begin
                        if (update_slot == k) begin
                            if (!update_match) begin
                                cluster_count[k] <= 16'd1;
                                cluster_rise_count[k] <= changed_rising ? 16'd1 : 16'd0;
                                cluster_fall_count[k] <= changed_rising ? 16'd0 : 16'd1;
                                cluster_x_min[k] <= sample_x_d;
                                cluster_x_max[k] <= sample_x_d;
                                cluster_y_min[k] <= sample_y_d;
                                cluster_y_max[k] <= sample_y_d;
                            end else begin
                                if (cluster_count[k] != 16'hffff)
                                    cluster_count[k] <= cluster_count[k]+16'd1;
                                if (changed_rising && cluster_rise_count[k] != 16'hffff)
                                    cluster_rise_count[k] <= cluster_rise_count[k]+16'd1;
                                if (!changed_rising && cluster_fall_count[k] != 16'hffff)
                                    cluster_fall_count[k] <= cluster_fall_count[k]+16'd1;
                                if (sample_x_d < cluster_x_min[k]) cluster_x_min[k] <= sample_x_d;
                                if (sample_x_d > cluster_x_max[k]) cluster_x_max[k] <= sample_x_d;
                                if (sample_y_d < cluster_y_min[k]) cluster_y_min[k] <= sample_y_d;
                                if (sample_y_d > cluster_y_max[k]) cluster_y_max[k] <= sample_y_d;
                            end
                        end
                    end
                end
            end
        end
    end
endmodule

// Count separated motion episodes, never retained display/HOLD boxes.
// A confirmation during cooldown is consumed (not deferred until it expires).
// Count survives mode changes; only global reset clears it. Re-entering TRACK
// requires a quiet interval before a fresh episode can be counted.
module motion_event_counter #(
    parameter integer CLOCK_HZ=75000000,
    parameter integer CONFIRM_FRAMES=2,
    parameter integer QUIET_FRAMES=6
)(
    input wire I_clk,I_rst_n,I_enable,I_publish,I_fresh,
    output reg [15:0] O_count_bcd,
    output reg O_increment
);
    localparam integer COOLDOWN_CYCLES=CLOCK_HZ<2 ? 1 : (CLOCK_HZ+1)/2;
    localparam integer CW=COOLDOWN_CYCLES<2 ? 1 : $clog2(COOLDOWN_CYCLES+1);
    localparam integer HW=CONFIRM_FRAMES<2 ? 1 : $clog2(CONFIRM_FRAMES+1);
    localparam integer QW=QUIET_FRAMES<2 ? 1 : $clog2(QUIET_FRAMES+1);
    reg [CW-1:0] cooldown;
    reg [HW-1:0] hits;
    reg [QW-1:0] quiet;
    reg armed;
    function [15:0] next_bcd(input [15:0] v);
        begin
            next_bcd=v;
            if(v!=16'h9999)begin
                if(v[3:0]!=9)next_bcd[3:0]=v[3:0]+1'b1;
                else begin next_bcd[3:0]=0;
                    if(v[7:4]!=9)next_bcd[7:4]=v[7:4]+1'b1;
                    else begin next_bcd[7:4]=0;
                        if(v[11:8]!=9)next_bcd[11:8]=v[11:8]+1'b1;
                        else begin next_bcd[11:8]=0;next_bcd[15:12]=v[15:12]+1'b1;end
                    end
                end
            end
        end
    endfunction
    always @(posedge I_clk)begin
        if(!I_rst_n)begin
            O_count_bcd<=0;O_increment<=0;cooldown<=0;hits<=0;quiet<=0;armed<=1;
        end else begin
            O_increment<=0;
            if(cooldown!=0)cooldown<=cooldown-1'b1;
            if(!I_enable)begin hits<=0;quiet<=0;armed<=0;end
            else if(I_publish)begin
                if(I_fresh)begin
                    quiet<=0;
                    if(hits<CONFIRM_FRAMES)hits<=hits+1'b1;
                    if(armed && hits>=CONFIRM_FRAMES-1)begin
                        armed<=0;
                        if(cooldown==0 && O_count_bcd!=16'h9999)begin
                            O_count_bcd<=next_bcd(O_count_bcd);O_increment<=1;
                            cooldown<=COOLDOWN_CYCLES-1;
                        end
                    end
                end else begin
                    hits<=0;
                    if(quiet<QUIET_FRAMES)quiet<=quiet+1'b1;
                    if(quiet>=QUIET_FRAMES-1)armed<=1;
                end
            end
        end
    end
endmodule
