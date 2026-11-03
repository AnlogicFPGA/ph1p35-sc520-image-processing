// Synchronize and debounce four active-low push buttons.
// Each confirmed press produces one clock-cycle pulse on O_press_pulse.
module key_debounce_one #(
    parameter integer DEBOUNCE_CYCLES = 1000000,
    parameter integer COUNTER_WIDTH   = 20
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_key_n,
    output reg  O_press_pulse,
    output wire O_key_n
);

    reg S_sync_1d;
    reg S_sync_2d;
    reg S_stable_key_n;
    reg [COUNTER_WIDTH-1:0] S_count;
    assign O_key_n=S_stable_key_n;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sync_1d <= 1'b1;
            S_sync_2d <= 1'b1;
        end else begin
            S_sync_1d <= I_key_n;
            S_sync_2d <= S_sync_1d;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_stable_key_n <= 1'b1;
            S_count        <= {COUNTER_WIDTH{1'b0}};
            O_press_pulse  <= 1'b0;
        end else begin
            O_press_pulse <= 1'b0;

            if (S_sync_2d == S_stable_key_n) begin
                S_count <= {COUNTER_WIDTH{1'b0}};
            end else if (S_count == DEBOUNCE_CYCLES - 1) begin
                S_stable_key_n <= S_sync_2d;
                S_count        <= {COUNTER_WIDTH{1'b0}};
                if (!S_sync_2d)
                    O_press_pulse <= 1'b1;
            end else begin
                S_count <= S_count + 1'b1;
            end
        end
    end

endmodule


module key_debounce_4 #(
    parameter integer DEBOUNCE_CYCLES = 1000000,
    parameter integer COUNTER_WIDTH   = 20
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [3:0] I_key_n,
    output wire [3:0] O_press_pulse,
    output wire [3:0] O_key_n
);

    genvar key_index;
    generate
        for (key_index = 0; key_index < 4; key_index = key_index + 1) begin : g_key
            key_debounce_one #(
                .DEBOUNCE_CYCLES (DEBOUNCE_CYCLES),
                .COUNTER_WIDTH   (COUNTER_WIDTH)
            ) u_key_debounce_one (
                .I_clk          (I_clk),
                .I_rst_n        (I_rst_n),
                .I_key_n        (I_key_n[key_index]),
                .O_press_pulse  (O_press_pulse[key_index]),
                .O_key_n(O_key_n[key_index])
            );
        end
    endgenerate

endmodule
