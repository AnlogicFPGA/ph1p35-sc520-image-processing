// Input is an already debounced active-low key. Short events occur on release;
// a long hold produces one event and never also produces a short event.
module key_short_long #(
    parameter integer LONG_CYCLES=80000000,
    parameter integer WIDTH=$clog2(LONG_CYCLES+1)
)(
    input wire I_clk,I_rst_n,I_key_n,
    output reg O_short_pulse,O_long_toggle
);
    reg held,long_seen;
    reg [WIDTH-1:0] count;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            held<=0;long_seen<=0;count<=0;
            O_short_pulse<=0;O_long_toggle<=0;
        end else begin
            O_short_pulse<=0;
            if(I_key_n) begin
                if(held && !long_seen) O_short_pulse<=1;
                held<=0;long_seen<=0;count<=0;
            end else if(!held) begin
                held<=1;count<=1;
            end else if(!long_seen) begin
                if(count==LONG_CYCLES-1) begin
                    O_long_toggle<=~O_long_toggle;long_seen<=1;
                end else count<=count+1'b1;
            end
        end
    end
endmodule
