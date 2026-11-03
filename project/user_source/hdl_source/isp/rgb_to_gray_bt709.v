module rgb_to_gray_bt709 (
    input  wire [23:0] I_rgb,
    output wire [23:0] O_gray
);

wire [7:0] red;
wire [7:0] green;
wire [7:0] blue;
wire [17:0] gray_sum;
wire [7:0] gray;

assign red   = I_rgb[23:16];
assign green = I_rgb[15:8];
assign blue  = I_rgb[7:0];

// ITU-R BT.709 fixed-point approximation:
// Y = 0.2126 R + 0.7152 G + 0.0722 B.
// Coefficients are scaled by 1024; +512 rounds to nearest.
assign gray_sum = red   * 18'd218
                + green * 18'd732
                + blue  * 18'd74
                + 18'd512;

assign gray   = gray_sum[17:10];
assign O_gray = {gray, gray, gray};

endmodule
