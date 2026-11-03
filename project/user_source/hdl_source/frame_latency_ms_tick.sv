// Nominal DDR usr_clk period: 15.000 ns (approximately 66.666667 MHz).
// 66667 cycles correspond to 1.000005 ms. Check the actual clock period
// and set CYCLES_PER_MS accordingly when integrating a different clock.
module frame_latency_ms_tick #(
    parameter integer CYCLES_PER_MS = 66667
) (
    input  wire I_clk,
    input  wire I_rst,
    output wire O_ms_tick
);
    localparam integer COUNT_W = CYCLES_PER_MS <= 1 ? 1 : $clog2(CYCLES_PER_MS);
    reg [COUNT_W-1:0] count;
    assign O_ms_tick = count == CYCLES_PER_MS - 1;

    always @(posedge I_clk) begin
        if (I_rst)
            count <= {COUNT_W{1'b0}};
        else if (O_ms_tick)
            count <= {COUNT_W{1'b0}};
        else
            count <= count + {{COUNT_W-1{1'b0}}, 1'b1};
    end
endmodule
