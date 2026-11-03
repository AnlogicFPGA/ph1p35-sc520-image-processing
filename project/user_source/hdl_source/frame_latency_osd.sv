// Combinational latency overlay. Its inputs must be
// latched at the matching HDMI frame boundary by the video adapter.
module frame_latency_osd #(
    parameter integer X_ORIGIN = 1000,
    parameter integer Y_ORIGIN = 24
) (
    input  wire        I_enable,
    input  wire        I_valid,
    input  wire [15:0] I_ms_bcd,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [23:0] I_rgb,
    output reg  [23:0] O_rgb
);
    reg [11:0] lx, ly;
    reg [3:0] index;
    reg [2:0] row, col;
    reg [7:0] chr;
    reg [4:0] bits;

    function [7:0] character;
        input valid;
        input [15:0] bcd;
        input [3:0] n;
        begin
            character = " ";
            case (n)
                0: character = "L";
                1: character = "A";
                2: character = "T";
                3: character = ":";
                4: character = valid ? ("0" + bcd[15:12]) : "W";
                5: character = valid ? ("0" + bcd[11:8]) : "A";
                6: character = valid ? ("0" + bcd[7:4]) : "I";
                7: character = valid ? ("0" + bcd[3:0]) : "T";
                8: character = valid ? "M" : " ";
                9: character = valid ? "S" : " ";
                default: character = " ";
            endcase
        end
    endfunction

    function [4:0] font;
        input [7:0] c;
        input [2:0] r;
        begin
            font = 0;
            case (c)
                "L": case(r) 0,1,2,3,4,5:font=5'b10000;6:font=5'b11111;endcase
                "A": case(r) 0:font=5'b01110;1,2:font=5'b10001;3:font=5'b11111;4,5,6:font=5'b10001;endcase
                "T": case(r) 0:font=5'b11111;1,2,3,4,5,6:font=5'b00100;endcase
                "W": case(r) 0,1,2,3:font=5'b10001;4,5:font=5'b10101;6:font=5'b01010;endcase
                "I": case(r) 0,6:font=5'b11111;1,2,3,4,5:font=5'b00100;endcase
                "M": case(r) 0:font=5'b10001;1:font=5'b11011;2:font=5'b10101;3,4,5,6:font=5'b10001;endcase
                "S": case(r) 0:font=5'b01111;1,2:font=5'b10000;3:font=5'b01110;4,5:font=5'b00001;6:font=5'b11110;endcase
                ":": case(r) 2,5:font=5'b00100;endcase
                "0": case(r) 0,6:font=5'b01110;1,5:font=5'b10001;2:font=5'b10011;3:font=5'b10101;4:font=5'b11001;endcase
                "1": case(r) 0:font=5'b00100;1:font=5'b01100;2,3,4,5:font=5'b00100;6:font=5'b01110;endcase
                "2": case(r) 0:font=5'b01110;1:font=5'b10001;2:font=5'b00001;3:font=5'b00010;4:font=5'b00100;5:font=5'b01000;6:font=5'b11111;endcase
                "3": case(r) 0:font=5'b11110;1,2:font=5'b00001;3:font=5'b01110;4,5:font=5'b00001;6:font=5'b11110;endcase
                "4": case(r) 0:font=5'b00010;1:font=5'b00110;2:font=5'b01010;3:font=5'b10010;4:font=5'b11111;5,6:font=5'b00010;endcase
                "5": case(r) 0:font=5'b11111;1,2:font=5'b10000;3:font=5'b11110;4,5:font=5'b00001;6:font=5'b11110;endcase
                "6": case(r) 0:font=5'b01110;1,2:font=5'b10000;3:font=5'b11110;4,5:font=5'b10001;6:font=5'b01110;endcase
                "7": case(r) 0:font=5'b11111;1:font=5'b00001;2:font=5'b00010;3:font=5'b00100;4,5,6:font=5'b01000;endcase
                "8": case(r) 0,3,6:font=5'b01110;1,2,4,5:font=5'b10001;endcase
                "9": case(r) 0,6:font=5'b01110;1,2:font=5'b10001;3:font=5'b01111;4,5:font=5'b00001;endcase
                default:font=0;
            endcase
        end
    endfunction

    always @* begin
        lx = 0; ly = 0; index = 0; row = 0; col = 0;
        chr = " "; bits = 0; O_rgb = I_rgb;
        if (I_enable && I_x >= X_ORIGIN && I_x < X_ORIGIN + 132 &&
            I_y >= Y_ORIGIN && I_y < Y_ORIGIN + 22) begin
            lx = I_x - X_ORIGIN;
            ly = I_y - Y_ORIGIN;
            O_rgb = 24'h101820;
            if (lx >= 6 && lx < 126 && ly >= 4 && ly < 18) begin
                index = (lx - 6) / 12;
                col = ((lx - 6) % 12) >> 1;
                row = (ly - 4) >> 1;
                chr = character(I_valid, I_ms_bcd, index);
                bits = font(chr, row);
                if (col < 5 && bits[4-col]) O_rgb = 24'hfff200;
            end
        end
    end
endmodule
