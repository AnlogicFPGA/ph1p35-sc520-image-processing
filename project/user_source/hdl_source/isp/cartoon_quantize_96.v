// Four-pixel RGB888 cartoonizer for the 96-bit ISP stream.
// Each color channel is reduced to eight evenly spread levels.  A final
// binary Canny edge replaces the corresponding quantized pixel with black,
// producing a streaming posterized image with a dark outline.
module cartoon_quantize_96 (
    input  wire [95:0] I_color,
    input  wire [95:0] I_edge,
    output wire [95:0] O_cartoon
);

    function [7:0] quantize8;
        input [7:0] value;
        begin
            // Replication, rather than zero filling, keeps the eight bins
            // distributed across the full 0..255 output range.
            quantize8 = {value[7:5], value[7:5], value[7:6]};
        end
    endfunction

    function [23:0] cartoon_pixel;
        input [23:0] color;
        input [23:0] edge_pixel;
        begin
            if (edge_pixel[23:16] != 8'h00)
                cartoon_pixel = 24'h000000;
            else
                cartoon_pixel = {quantize8(color[23:16]),
                                 quantize8(color[15:8]),
                                 quantize8(color[7:0])};
        end
    endfunction

    assign O_cartoon[23:0]  = cartoon_pixel(I_color[23:0],  I_edge[23:0]);
    assign O_cartoon[47:24] = cartoon_pixel(I_color[47:24], I_edge[47:24]);
    assign O_cartoon[71:48] = cartoon_pixel(I_color[71:48], I_edge[71:48]);
    assign O_cartoon[95:72] = cartoon_pixel(I_color[95:72], I_edge[95:72]);

endmodule
