// Bundled-data mailbox. Source holds data until the destination acknowledges.
// Continuously refresh while idle so either clock domain may reset on its own.
module cdc_status_snapshot #(
    parameter integer WIDTH=9,
    parameter [WIDTH-1:0] RESET_VALUE=9'h004
)(
    input wire I_src_clk, I_src_rst_n,
    input wire [WIDTH-1:0] I_data,
    input wire I_dst_clk, I_dst_rst_n,
    output wire [WIDTH-1:0] O_data
);
    // TD's preserve/keep attributes retain the CDC stage boundaries and names.
    (* preserve=1, keep=1 *) reg [WIDTH-1:0] held_data;
    (* preserve=1, keep=1 *) reg [WIDTH-1:0] sampled_data;
    (* preserve=1, keep=1 *) reg request, acknowledge;
    (* preserve=1, keep=1, ASYNC_REG="TRUE" *) reg ack_sync1, ack_sync2;
    (* preserve=1, keep=1, ASYNC_REG="TRUE" *) reg req_sync1, req_sync2;
    assign O_data=sampled_data;
    always @(posedge I_src_clk or negedge I_src_rst_n) begin
        if(!I_src_rst_n) begin
            held_data<=RESET_VALUE; request<=0; ack_sync1<=0; ack_sync2<=0;
        end else begin
            ack_sync1<=acknowledge; ack_sync2<=ack_sync1;
            if(request==ack_sync2) begin
                held_data<=I_data; request<=~request;
            end
        end
    end
    always @(posedge I_dst_clk or negedge I_dst_rst_n) begin
        if(!I_dst_rst_n) begin
            req_sync1<=0; req_sync2<=0; acknowledge<=0; sampled_data<=RESET_VALUE;
        end else begin
            req_sync1<=request; req_sync2<=req_sync1;
            if(req_sync2!=acknowledge) begin
                sampled_data<=held_data; acknowledge<=req_sync2;
            end
        end
    end
endmodule
