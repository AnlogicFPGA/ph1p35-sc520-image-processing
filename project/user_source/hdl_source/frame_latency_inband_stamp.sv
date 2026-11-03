// The first four pixels of the first DDR word carry a timestamp with the
// image itself. Only the displayed pixels can subsequently validate a sample.
module frame_latency_inband_stamp #(
    parameter integer HDMI_CYCLES_PER_MS = 75000
) (
    input  wire         I_rst_n,
    input  wire         I_hdmi_clk,
    input  wire         I_ddr_clk,
    input  wire         I_frame_start_ddr,
    input  wire         I_wr_en,
    input  wire [24:0]  I_wr_addr,
    input  wire [127:0] I_wr_data,
    output wire [127:0] O_wr_data,
    output wire         O_tagged,
    output wire [31:0]  O_hdmi_ms
);
    localparam [31:0] MAGIC = 32'hd24ba701;
    localparam [24:0] BASE_0 = 25'd0;
    localparam [24:0] BASE_1 = 25'd7000000;
    localparam [24:0] BASE_2 = 25'd14000000;
    localparam [24:0] BASE_3 = 25'd21000000;

    wire ms_tick;
    reg [31:0] hdmi_ms;
    reg [31:0] hdmi_ms_gray;
    (* ASYNC_REG = "TRUE" *) reg [31:0] gray_sync_1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] gray_sync_2;
    reg [31:0] frame_start_ms;
    reg tag_pending;
    wire [31:0] next_hdmi_ms = hdmi_ms + 32'd1;
    wire [31:0] synced_ms;
    wire at_base = I_wr_addr == BASE_0 || I_wr_addr == BASE_1 ||
                   I_wr_addr == BASE_2 || I_wr_addr == BASE_3;

    frame_latency_ms_tick #(.CYCLES_PER_MS(HDMI_CYCLES_PER_MS)) u_ms_tick (
        .I_clk(I_hdmi_clk), .I_rst(~I_rst_n), .O_ms_tick(ms_tick)
    );

    always @(posedge I_hdmi_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            hdmi_ms <= 32'd0;
            hdmi_ms_gray <= 32'd0;
        end else if (ms_tick) begin
            hdmi_ms <= next_hdmi_ms;
            // Register the Gray code so only one physical source bit changes.
            hdmi_ms_gray <= next_hdmi_ms ^ (next_hdmi_ms >> 1);
        end
    end

    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gray_sync_1 <= 32'd0;
            gray_sync_2 <= 32'd0;
            frame_start_ms <= 32'd0;
            tag_pending <= 1'b0;
        end else begin
            gray_sync_1 <= hdmi_ms_gray;
            gray_sync_2 <= gray_sync_1;
            if (I_frame_start_ddr) begin
                frame_start_ms <= synced_ms;
                tag_pending <= 1'b1;
            end else if (I_wr_en && at_base) begin
                tag_pending <= 1'b0;
            end
        end
    end

    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer bit_index;
        begin
            gray_to_binary[31] = gray[31];
            for (bit_index = 30; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] = gray_to_binary[bit_index+1] ^ gray[bit_index];
        end
    endfunction

    assign synced_ms = gray_to_binary(gray_sync_2);
    assign O_tagged = tag_pending && I_wr_en && at_base && !I_frame_start_ddr;
    assign O_wr_data = O_tagged ?
        {MAGIC, frame_start_ms, ~frame_start_ms, I_wr_data[31:0]} : I_wr_data;
    assign O_hdmi_ms = hdmi_ms;
endmodule
