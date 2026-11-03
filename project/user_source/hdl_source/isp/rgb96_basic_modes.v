// Four-pixel RGB888 basic image modes for the 96-bit ISP stream.
// Pixel packing is unchanged: each consecutive 24-bit word is {R,G,B}.
module rgb96_basic_modes #(
    parameter [7:0] BINARY_THRESHOLD = 8'd128
)(
    input  wire [95:0] I_rgb,
    output wire [95:0] O_gray,
    output wire [95:0] O_binary
);

    wire [7:0] gray0 = O_gray[23:16];
    wire [7:0] gray1 = O_gray[47:40];
    wire [7:0] gray2 = O_gray[71:64];
    wire [7:0] gray3 = O_gray[95:88];

    rgb96_to_gray_bt709 u_gray (
        .I_rgb  (I_rgb),
        .O_gray (O_gray)
    );

    assign O_binary[23:0]  = (gray0 >= BINARY_THRESHOLD) ? 24'hffffff : 24'h000000;
    assign O_binary[47:24] = (gray1 >= BINARY_THRESHOLD) ? 24'hffffff : 24'h000000;
    assign O_binary[71:48] = (gray2 >= BINARY_THRESHOLD) ? 24'hffffff : 24'h000000;
    assign O_binary[95:72] = (gray3 >= BINARY_THRESHOLD) ? 24'hffffff : 24'h000000;

endmodule
