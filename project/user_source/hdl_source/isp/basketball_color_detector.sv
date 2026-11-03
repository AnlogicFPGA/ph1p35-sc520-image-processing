// Basketball candidate detector. RGB is sampled before display
// enhancements and OSD. Threshold selection depends on SC520 calibration,
// scene illumination, and target/background color contrast.
module basketball_color_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter [7:0] RED_OVER_GREEN = 8'd14
)(
    input wire I_clk, I_rst_n, I_enable, I_frame_start, I_pixel_valid,
    input wire [11:0] I_x, I_y,
    input wire [23:0] I_rgb,
    output reg O_target_valid,
    output reg [11:0] O_x_min, O_x_max, O_y_min, O_y_max,
    output reg [11:0] O_center_x, O_center_y,
    output reg O_publish_pulse,
    output reg [2:0] O_diag_reason,
    output reg [15:0] O_diag_count
);
    localparam integer GROUPS = 4;
    reg [11:0] xlo [0:GROUPS-1], xhi [0:GROUPS-1];
    reg [11:0] ylo [0:GROUPS-1], yhi [0:GROUPS-1];
    reg [15:0] hits [0:GROUPS-1];
    wire [7:0] red = I_rgb[23:16];
    wire [7:0] green = I_rgb[15:8];
    wire [7:0] blue = I_rgb[7:0];
    wire sample_hit = I_enable && I_pixel_valid && I_x < IMG_WIDTH &&
                      I_y < IMG_HEIGHT && I_x[2:0]==3'b000 &&
                      I_y[2:0]==3'b000 && red >= 8'd40 && red <= 8'd245 &&
                      {1'b0,red} >= ({1'b0,green}+{1'b0,RED_OVER_GREEN}) &&
                      green >= blue;

    integer choose_slot, k;
    always @* begin
        choose_slot = -1;
        // Keep separated red objects apart; prefer an existing local group.
        for (integer j=0; j<GROUPS; j=j+1)
            if (choose_slot < 0 && hits[j]!=0 &&
                {1'b0,I_x}+13'd24 >= {1'b0,xlo[j]} &&
                {1'b0,I_y}+13'd24 >= {1'b0,ylo[j]} &&
                {1'b0,I_x} <= {1'b0,xhi[j]}+13'd24 &&
                {1'b0,I_y} <= {1'b0,yhi[j]}+13'd24)
                choose_slot = j;
        for (integer j=0; j<GROUPS; j=j+1)
            if (choose_slot < 0 && hits[j]==0) choose_slot = j;
    end

    reg candidate_valid;
    reg [1:0] candidate_slot;
    reg [15:0] candidate_hits;
    reg [16:0] total_hits;
    reg [12:0] span_x, span_y;
    integer s;
    always @* begin
        candidate_valid=1'b0; candidate_slot=2'd0; candidate_hits=16'd0;
        total_hits=17'd0; span_x=13'd0; span_y=13'd0;
        for (integer j=0; j<GROUPS; j=j+1) begin
            total_hits=total_hits+{1'b0,hits[j]};
            if (hits[j]!=0) begin
                span_x={1'b0,xhi[j]}-{1'b0,xlo[j]}+13'd1;
                span_y={1'b0,yhi[j]}-{1'b0,ylo[j]}+13'd1;
                if (hits[j]>=16'd24 && span_x>=13'd48 && span_y>=13'd48 &&
                    span_x<=13'd640 && span_y<=13'd640 &&
                    span_x <= (span_y+(span_y>>1)) &&
                    span_y <= (span_x+(span_x>>1)) &&
                    (!candidate_valid || hits[j]>candidate_hits)) begin
                    candidate_valid=1'b1;
                    candidate_slot=j[1:0];
                    candidate_hits=hits[j];
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_target_valid<=1'b0; O_publish_pulse<=1'b0;
            O_x_min<=0; O_x_max<=0; O_y_min<=0; O_y_max<=0;
            O_center_x<=0; O_center_y<=0;
            O_diag_reason<=3'd0; O_diag_count<=16'd0;
            for (integer j=0; j<GROUPS; j=j+1) begin
                xlo[j]<=0; xhi[j]<=0; ylo[j]<=0; yhi[j]<=0; hits[j]<=0;
            end
        end else begin
            O_publish_pulse<=1'b0;
            if (I_frame_start) begin
                O_target_valid<=I_enable && candidate_valid;
                O_publish_pulse<=I_enable;
                O_diag_count<=total_hits[16] ? 16'hffff : total_hits[15:0];
                O_diag_reason<=!I_enable ? 3'd0 :
                               total_hits<17'd24 ? 3'd1 :
                               candidate_valid ? 3'd7 : 3'd3;
                if (I_enable && candidate_valid) begin
                    O_x_min<=xlo[candidate_slot]; O_x_max<=xhi[candidate_slot];
                    O_y_min<=ylo[candidate_slot]; O_y_max<=yhi[candidate_slot];
                    O_center_x<=(xlo[candidate_slot]+xhi[candidate_slot])>>1;
                    O_center_y<=(ylo[candidate_slot]+yhi[candidate_slot])>>1;
                end
                for (integer j=0; j<GROUPS; j=j+1) begin
                    xlo[j]<=0; xhi[j]<=0; ylo[j]<=0; yhi[j]<=0; hits[j]<=0;
                end
            end else if (sample_hit && choose_slot>=0) begin
                for (integer j=0; j<GROUPS; j=j+1) if (choose_slot==j) begin
                    if (hits[j]==0) begin
                        xlo[j]<=I_x; xhi[j]<=I_x; ylo[j]<=I_y; yhi[j]<=I_y;
                        hits[j]<=16'd1;
                    end else begin
                        if (I_x<xlo[j]) xlo[j]<=I_x;
                        if (I_x>xhi[j]) xhi[j]<=I_x;
                        if (I_y<ylo[j]) ylo[j]<=I_y;
                        if (I_y>yhi[j]) yhi[j]<=I_y;
                        if (hits[j]!=16'hffff) hits[j]<=hits[j]+16'd1;
                    end
                end
            end
        end
    end
endmodule
