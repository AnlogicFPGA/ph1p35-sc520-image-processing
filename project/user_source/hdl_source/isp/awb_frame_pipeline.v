// AWB transport uses two input registers and a 50-cycle line
// delay, output staging, early frame-start, and reconstructed line end.
// Only the statistics/gain policy and pixel gain precision are replaced.
module awb_frame_pipeline #(
    parameter integer IMG_WIDTH=1280,IMG_HEIGHT=720
)(
    input wire I_clk,I_rst_n,I_tlast,I_tuser,I_tvalid,
    input wire [95:0] I_tdata,
    input wire [1:0] I_mode_request,
    output wire I_tready,
    input wire O_tready,
    output wire O_tlast,O_tuser,O_tvalid,
    output wire [95:0] O_tdata,
    output wire O_calibrated
);
    wire [9:0] gain_r,gain_b;
    awb_frame_control #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_control(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tuser(I_tuser),.I_tvalid(I_tvalid),
        .I_tdata(I_tdata),.I_mode_request(I_mode_request),
        .O_gain_r(gain_r),.O_gain_b(gain_b),.O_mode(),.O_calibrated(O_calibrated));
    reg [95:0] input_d0,input_d1,data_d1;
    reg valid_0,valid_1,valid_d1,valid_d2;
    wire [95:0] delayed_data;
    wire delayed_valid,delayed_user;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            input_d0<=0;input_d1<=0;valid_0<=0;valid_1<=0;
            data_d1<=0;valid_d1<=0;valid_d2<=0;
        end else begin
            input_d0<=I_tdata;input_d1<=input_d0;
            valid_0<=I_tvalid;valid_1<=valid_0;
            data_d1<=delayed_data;valid_d1<=delayed_valid;valid_d2<=valid_d1;
        end
    end
    signal_delay #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.DATA_WIDTH(96),.DELAY_CYCLE(50)) u_delay(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_tuser(I_tuser),
        .I_valid(valid_1),.I_data(input_d1),.O_valid(delayed_valid),
        .O_data(delayed_data),.O_tuser(delayed_user));
    reg [17:0] corrected_r[0:3],corrected_b[0:3];
    reg [7:0] corrected_g[0:3];
    wire [7:0] red[0:3],blue[0:3];
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin:g_pixel
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin corrected_r[lane]<=0;corrected_b[lane]<=0;corrected_g[lane]<=0;end
            else if(valid_d1) begin
                corrected_r[lane]<=data_d1[lane*24+16+:8]*gain_r+18'd128;
                corrected_g[lane]<=data_d1[lane*24+8+:8];
                corrected_b[lane]<=data_d1[lane*24+:8]*gain_b+18'd128;
            end
        end
        assign red[lane]=(|corrected_r[lane][17:16])?8'd255:corrected_r[lane][15:8];
        assign blue[lane]=(|corrected_b[lane][17:16])?8'd255:corrected_b[lane][15:8];
        assign O_tdata[lane*24+:24]=O_tvalid ? {red[lane],corrected_g[lane],blue[lane]} : 24'd0;
    end endgenerate
    assign O_tuser=I_tuser;
    assign O_tvalid=valid_d2;
    assign O_tlast=!valid_d1 && valid_d2;
    assign I_tready=O_tready;
endmodule
