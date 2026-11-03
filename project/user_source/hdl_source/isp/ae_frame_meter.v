// Complete-frame brightness statistics after AWB, before display-mode processing.
// One RGB pixel is sampled per four-pixel beat. A snapshot is published only
// when the next source frame starts and the previous frame had every beat.
module ae_frame_meter #(
    parameter integer FRAME_BEATS = 230400
)(
    input wire I_clk, I_rst_n,
    input wire I_tuser, I_tvalid,
    input wire [95:0] I_tdata,
    // {sequence, luminance sum, max(R,G,B)>=208 count, max(R,G,B)>=250 count}
    output reg [69:0] O_stats
);
    reg started;
    reg [17:0] count;
    reg [25:0] sum;
    reg [17:0] near_white, clipped;
    wire [9:0] luma_sum = {2'b0,I_tdata[23:16]} +
                          {1'b0,I_tdata[15:8],1'b0} +
                          {2'b0,I_tdata[7:0]};
    wire [7:0] luma = luma_sum[9:2];
    // A saturated blue or red channel can have low Y. Protect channel headroom
    // as well as luminance; otherwise AE can increase gain into colored halos.
    wire [7:0] max_rg = I_tdata[23:16] > I_tdata[15:8] ? I_tdata[23:16] : I_tdata[15:8];
    wire [7:0] max_rgb = max_rg > I_tdata[7:0] ? max_rg : I_tdata[7:0];
    wire is_near_white = max_rgb >= 8'd208;
    wire is_clipped = max_rgb >= 8'd250;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            started<=0; count<=0; sum<=0; near_white<=0; clipped<=0; O_stats<=0;
        end else if (I_tuser) begin
            if (started && count==FRAME_BEATS)
                O_stats <= {O_stats[69:62]+8'd1,sum,near_white,clipped};
            started<=1;
            count<=I_tvalid ? 18'd1 : 18'd0;
            sum<=I_tvalid ? {18'd0,luma} : 26'd0;
            near_white<=I_tvalid ? {17'd0,is_near_white} : 18'd0;
            clipped<=I_tvalid ? {17'd0,is_clipped} : 18'd0;
        end else if (started && I_tvalid && count<FRAME_BEATS) begin
            count<=count+1'b1;
            sum<=sum+{18'd0,luma};
            near_white<=near_white+is_near_white;
            clipped<=clipped+is_clipped;
        end
    end
endmodule
