// Select the first independently confirmed OBJECT track. A later class cannot
// steal ownership. Presentation HOLD boxes must not be connected here.
module gf_target_selector #(
    parameter integer LOST_FRAMES=15,
    parameter integer FORCE_CLASS=0 // 0 first arrival; 2 blue; 3 red.
)(
    input wire I_clk,I_rst_n,I_enable,I_publish,
    input wire I_red_fresh,I_blue_fresh,
    input wire [11:0] I_red_x,I_blue_x,
    output wire [2:0] O_class_id,
    output reg O_publish
);
    localparam integer MISS_W=LOST_FRAMES<2?1:$clog2(LOST_FRAMES);
    reg [2:0] owner;
    reg [MISS_W-1:0] missed;
    // Temporal ordering is unavailable for two tracks confirmed in one frame.
    // The leftmost center wins this tie; equal centers use red as final tie.
    wire [2:0] first_choice=I_red_fresh && I_blue_fresh ?
        (I_red_x<=I_blue_x ? 3'd3 : 3'd2) :
        I_red_fresh ? 3'd3 : I_blue_fresh ? 3'd2 : 3'd0;
    wire owner_fresh=owner==3 ? I_red_fresh : owner==2 ? I_blue_fresh : 1'b0;
    assign O_class_id=I_enable ? (FORCE_CLASS!=0 ? FORCE_CLASS : owner) : 3'd0;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin owner<=0;missed<=0;O_publish<=0;end
        else begin
            // Adapter consumes the new owner and the same stable coordinates
            // one edge later, after nonblocking assignments have taken effect.
            O_publish<=I_enable && I_publish;
            if(!I_enable)begin owner<=0;missed<=0;end
            else if(FORCE_CLASS!=0)begin owner<=FORCE_CLASS;missed<=0;end
            else if(I_publish)begin
                if(owner==0)begin owner<=first_choice;missed<=0;end
                else if(owner_fresh)missed<=0;
                else if(missed==LOST_FRAMES-1)begin owner<=first_choice;missed<=0;end
                else missed<=missed+1'b1;
            end
        end
    end
`ifndef SYNTHESIS
    initial if(LOST_FRAMES<1 || (FORCE_CLASS!=0 && FORCE_CLASS!=2 && FORCE_CLASS!=3))
        $fatal(1,"gf_target_selector invalid parameters");
`endif
endmodule
