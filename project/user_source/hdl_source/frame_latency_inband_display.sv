// Decode only a marker carried by pixels that actually reached the HDMI
// stream. A 16-pixel top-left guard hides the metadata and tolerates the
// existing video_out/hdmi_mixer pipeline offset.
module frame_latency_inband_display (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_pixel_valid,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [23:0] I_rgb,
    input  wire [31:0] I_now_ms,
    output wire [23:0] O_clean_rgb,
    output reg         O_latency_valid,
    output reg [15:0]  O_latency_bcd,
    output reg         O_source_valid,
    output reg [31:0]  O_source_ms
);
    localparam [31:0] MAGIC = 32'hd24ba701;
    reg [71:0] prefix;
    reg [2:0] seen_count;
    reg sample_latched;
    reg convert_valid;
    reg [15:0] convert_ms;
    wire convert_ready;
    wire bcd_valid;
    wire bcd_out_of_range;
    wire [15:0] converted_bcd;
    wire search_pixel = I_pixel_valid && I_y == 12'd0 && I_x < 12'd16;
    wire [95:0] marker = {prefix, I_rgb};
    wire [31:0] stamped_ms = marker[63:32];
    wire [31:0] age_ms = I_now_ms - stamped_ms;
    wire marker_ok = seen_count >= 3'd3 && marker[95:64] == MAGIC &&
                     marker[31:0] == ~stamped_ms;

    assign O_clean_rgb = (I_pixel_valid && I_y == 12'd0 && I_x < 12'd16)
                         ? 24'd0 : I_rgb;

    frame_latency_bcd u_bcd (
        .I_clk(I_clk), .I_rst(~I_rst_n),
        .I_in_valid(convert_valid), .O_in_ready(convert_ready),
        .I_ms(convert_ms), .O_out_valid(bcd_valid),
        .O_out_of_range(bcd_out_of_range), .O_bcd(converted_bcd)
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            prefix <= 72'd0;
            seen_count <= 3'd0;
            sample_latched <= 1'b0;
            convert_valid <= 1'b0;
            convert_ms <= 16'd0;
            O_latency_valid <= 1'b0;
            O_latency_bcd <= 16'd0;
            O_source_valid <= 1'b0;
            O_source_ms <= 32'd0;
        end else begin
            convert_valid <= 1'b0;
            if (I_frame_start) begin
                prefix <= 72'd0;
                seen_count <= 3'd0;
                sample_latched <= 1'b0;
                O_latency_valid <= 1'b0;
                O_source_valid <= 1'b0;
            end
            if (search_pixel) begin
                if (I_frame_start) begin
                    // The first active pixel can coincide with frame start.
                    // Start the shift window with that pixel, not stale data
                    // from the preceding display frame.
                    prefix <= {48'd0, I_rgb};
                    seen_count <= 3'd1;
                end else begin
                    prefix <= {prefix[47:0], I_rgb};
                    if (seen_count < 3'd4)
                        seen_count <= seen_count + 3'd1;
                end
            end
            if (search_pixel && !I_frame_start && !sample_latched && marker_ok &&
                age_ms <= 32'd9999 && convert_ready) begin
                convert_ms <= age_ms[15:0];
                convert_valid <= 1'b1;
                sample_latched <= 1'b1;
                O_source_valid <= 1'b1;
                O_source_ms <= stamped_ms;
            end
            if (bcd_valid && !I_frame_start) begin
                O_latency_bcd <= converted_bcd;
                O_latency_valid <= !bcd_out_of_range;
            end
        end
    end
endmodule
