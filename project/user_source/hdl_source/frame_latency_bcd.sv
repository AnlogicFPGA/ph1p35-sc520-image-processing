// Small sequential decimal formatter; one conversion takes 16 clocks.
module frame_latency_bcd (
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_in_valid,
    output wire        O_in_ready,
    input  wire [15:0] I_ms,
    output reg         O_out_valid,
    output reg         O_out_of_range,
    output reg  [15:0] O_bcd
);
    reg busy;
    reg [4:0] remaining;
    reg [15:0] binary_shift;
    reg [15:0] bcd_work;
    reg pending_out_of_range;
    reg [15:0] corrected;
    integer i;
    assign O_in_ready = !busy;

    always @* begin
        corrected = bcd_work;
        for (i = 0; i < 4; i = i + 1)
            if (corrected[4*i +: 4] >= 5)
                corrected[4*i +: 4] = corrected[4*i +: 4] + 4'd3;
    end

    always @(posedge I_clk) begin
        if (I_rst) begin
            busy <= 1'b0;
            remaining <= 5'd0;
            binary_shift <= 16'd0;
            bcd_work <= 16'd0;
            pending_out_of_range <= 1'b0;
            O_out_valid <= 1'b0;
            O_out_of_range <= 1'b0;
            O_bcd <= 16'd0;
        end else begin
            O_out_valid <= 1'b0;
            if (I_in_valid && O_in_ready) begin
                busy <= 1'b1;
                remaining <= 5'd16;
                binary_shift <= I_ms > 16'd9999 ? 16'd9999 : I_ms;
                pending_out_of_range <= I_ms > 16'd9999;
                bcd_work <= 16'd0;
            end else if (busy) begin
                bcd_work <= {corrected[14:0], binary_shift[15]};
                binary_shift <= {binary_shift[14:0], 1'b0};
                remaining <= remaining - 5'd1;
                if (remaining == 5'd1) begin
                    busy <= 1'b0;
                    O_out_valid <= 1'b1;
                    O_out_of_range <= pending_out_of_range;
                    O_bcd <= {corrected[14:0], binary_shift[15]};
                end
            end
        end
    end
endmodule
