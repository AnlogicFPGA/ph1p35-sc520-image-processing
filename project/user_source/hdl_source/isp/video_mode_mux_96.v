// 0: color, 1: BT.709 gray, 2: binary, 3: edge, 4: cartoon, 5: detect.
// DETECT uses color here; analysis and overlay run after DDR in the HDMI domain.
module video_mode_mux_96 (
    input  wire [2:0]  I_mode,
    input  wire [95:0] I_color,
    input  wire [95:0] I_gray,
    input  wire [95:0] I_binary,
    input  wire [95:0] I_edge,
    input  wire [95:0] I_cartoon,
    output reg  [95:0] O_video
);

    always @(*) begin
        case (I_mode)
            3'd0: O_video = I_color;
            3'd1: O_video = I_gray;
            3'd2: O_video = I_binary;
            3'd3: O_video = I_edge;
            3'd4: O_video = I_cartoon;
            3'd5: O_video = I_color;
            default: O_video = I_color;
        endcase
    end

endmodule
