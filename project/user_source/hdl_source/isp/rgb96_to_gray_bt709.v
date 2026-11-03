module rgb96_to_gray_bt709 (
    input  wire [95:0] I_rgb,
    output wire [95:0] O_gray
);

rgb_to_gray_bt709 u_gray_pixel0 (
    .I_rgb  (I_rgb[23:0]),
    .O_gray (O_gray[23:0])
);

rgb_to_gray_bt709 u_gray_pixel1 (
    .I_rgb  (I_rgb[47:24]),
    .O_gray (O_gray[47:24])
);

rgb_to_gray_bt709 u_gray_pixel2 (
    .I_rgb  (I_rgb[71:48]),
    .O_gray (O_gray[71:48])
);

rgb_to_gray_bt709 u_gray_pixel3 (
    .I_rgb  (I_rgb[95:72]),
    .O_gray (O_gray[95:72])
);

endmodule
