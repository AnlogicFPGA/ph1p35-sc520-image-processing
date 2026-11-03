// A slow key event crosses as a toggle. The display and camera domains each
// receive the same commands; camera pixels apply the request at frame start.
module wb_mode_select(
    input wire I_clk,I_rst_n,I_toggle,
    output reg [1:0] O_mode
);
    (* preserve=1,keep=1,ASYNC_REG="TRUE" *) reg toggle_sync1,toggle_sync2;
    reg seen;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            toggle_sync1<=0;toggle_sync2<=0;seen<=0;O_mode<=0;
        end else begin
            toggle_sync1<=I_toggle;toggle_sync2<=toggle_sync1;seen<=toggle_sync2;
            if(toggle_sync2!=seen) O_mode<=(O_mode==2) ? 0 : O_mode+1'b1;
        end
    end
endmodule
