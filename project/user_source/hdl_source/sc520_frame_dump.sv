// Diagnostic frame export only. Connect to the HDMI pixel clock and the
// pre-OSD RGB/coordinate signals after their existing two-cycle alignment.
// Captures one complete decimated frame, then freezes RAM while transmitting.
module sc520_frame_dump #(
    parameter integer CLK_HZ = 75000000,
    parameter integer BAUD = 115200,
    parameter integer SRC_WIDTH = 1280,
    parameter integer SRC_HEIGHT = 720,
    parameter integer STEP = 8,
    parameter integer RGB_CAPTURE = 0
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_pixel_valid,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    input  wire [23:0] I_rgb,
    output wire        O_uart_tx
);
    localparam integer WIDTH = SRC_WIDTH / STEP;
    localparam integer HEIGHT = SRC_HEIGHT / STEP;
    localparam integer PIXELS = WIDTH * HEIGHT;
    localparam integer PAYLOAD_BYTES = PIXELS * (RGB_CAPTURE ? 3 : 1);
    localparam integer ADDR_W = $clog2(PIXELS);
    localparam integer COUNT_W = $clog2(PIXELS + 1);
    localparam integer PACKET_BYTES = 10 + PAYLOAD_BYTES + 2;
    localparam integer DIVISOR = (CLK_HZ + BAUD / 2) / BAUD;

    initial begin
        if (SRC_WIDTH % STEP != 0 || SRC_HEIGHT % STEP != 0 ||
            WIDTH < 1 || HEIGHT < 1 || WIDTH > 65535 || HEIGHT > 65535 ||
            PIXELS < 2 || DIVISOR < 4)
            $error("sc520_frame_dump: invalid parameters");
    end

    localparam [2:0] WAIT_FRAME = 0, CAPTURE = 1, PREPARE = 2,
                     RAM_WAIT = 3, RAM_LATCH = 4, START_BYTE = 5,
                     WAIT_BUSY = 6, WAIT_DONE = 7;
    reg [2:0] state;
    reg [23:0] frame_mem [0:PIXELS-1];
    reg [ADDR_W-1:0] capture_addr;
    reg [15:0] expected_x, expected_y;
    reg [15:0] checksum, frame_seq;
    reg [COUNT_W+3:0] byte_index;
    reg [ADDR_W-1:0] read_addr;
    reg [23:0] read_data, pixel_data;
    reg [ADDR_W-1:0] payload_addr;
    reg [1:0] color_channel;
    reg [7:0] uart_data;
    reg uart_start;
    wire uart_busy;
    wire sample = I_pixel_valid && I_x < SRC_WIDTH && I_y < SRC_HEIGHT &&
                  (I_x % STEP == 0) && (I_y % STEP == 0);
    wire [9:0] rgb_sum = {2'b0, I_rgb[23:16]} +
                          {1'b0, I_rgb[15:8], 1'b0} +
                          {2'b0, I_rgb[7:0]};
    wire [7:0] gray = rgb_sum[9:2];
    wire [9:0] rgb_byte_sum = {2'b0,I_rgb[23:16]} +
                              {2'b0,I_rgb[15:8]} +
                              {2'b0,I_rgb[7:0]};
    wire [15:0] sample_checksum = RGB_CAPTURE ? {6'b0,rgb_byte_sum}
                                                 : {8'b0,gray};
    wire first_sample = state == WAIT_FRAME && I_frame_start && sample &&
                        I_x == 0 && I_y == 0;
    wire next_sample = state == CAPTURE && !I_frame_start && sample &&
                       I_x == expected_x * STEP && I_y == expected_y * STEP;
    wire [ADDR_W-1:0] write_addr = first_sample ? {ADDR_W{1'b0}} : capture_addr;

    // Keep the memory outside the resettable control process so TD can infer
    // a synchronous single-write/single-read block RAM instead of registers.
    always @(posedge I_clk) begin
        if (first_sample || next_sample)
            frame_mem[write_addr] <= RGB_CAPTURE ? I_rgb : {16'b0,gray};
        read_data <= frame_mem[read_addr];
    end

    // Only one write port is used while capture is active. Transmission uses
    // the same RAM after the final sample, so no double buffer is required.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state <= WAIT_FRAME;
            capture_addr <= 0;
            expected_x <= 0;
            expected_y <= 0;
            checksum <= 0;
            frame_seq <= 0;
            byte_index <= 0;
            read_addr <= 0;
            payload_addr <= 0;
            color_channel <= 0;
            pixel_data <= 0;
            uart_data <= 0;
            uart_start <= 0;
        end else begin
            uart_start <= 0;
            case (state)
                WAIT_FRAME: begin
                    if (I_frame_start) begin
                        checksum <= 0;
                        capture_addr <= 0;
                        expected_x <= 0;
                        expected_y <= 0;
                        state <= CAPTURE;
                        if (sample && I_x == 0 && I_y == 0) begin
                            checksum <= sample_checksum;
                            capture_addr <= 1;
                            expected_x <= 1;
                        end
                    end
                end
                CAPTURE: begin
                    // A new frame before the expected last sample invalidates
                    // the partial capture. Resynchronize at its first pixel.
                    if (I_frame_start) begin
                        state <= WAIT_FRAME;
                    end else if (sample) begin
                        if (I_x != expected_x * STEP ||
                            I_y != expected_y * STEP) begin
                            state <= WAIT_FRAME;
                        end else begin
                            checksum <= checksum + sample_checksum;
                            if (capture_addr == PIXELS - 1) begin
                                frame_seq <= frame_seq + 1'b1;
                                byte_index <= 0;
                                payload_addr <= 0;
                                color_channel <= 0;
                                state <= PREPARE;
                            end else begin
                                capture_addr <= capture_addr + 1'b1;
                                if (expected_x == WIDTH - 1) begin
                                    expected_x <= 0;
                                    expected_y <= expected_y + 1'b1;
                                end else begin
                                    expected_x <= expected_x + 1'b1;
                                end
                            end
                        end
                    end
                end
                PREPARE: begin
                    if (byte_index >= 10 && byte_index < 10 + PAYLOAD_BYTES) begin
                        if (RGB_CAPTURE && color_channel != 0) begin
                            uart_data <= color_channel == 1
                                       ? pixel_data[15:8] : pixel_data[7:0];
                            state <= START_BYTE;
                        end else begin
                            read_addr <= payload_addr;
                            state <= RAM_WAIT;
                        end
                    end else begin
                        case (byte_index)
                            0: uart_data <= "S";
                            1: uart_data <= "C";
                            2: uart_data <= RGB_CAPTURE ? "R" : "F";
                            3: uart_data <= "1";
                            4: uart_data <= frame_seq[7:0];
                            5: uart_data <= frame_seq[15:8];
                            6: uart_data <= WIDTH & 8'hff;
                            7: uart_data <= (WIDTH >> 8) & 8'hff;
                            8: uart_data <= HEIGHT & 8'hff;
                            9: uart_data <= (HEIGHT >> 8) & 8'hff;
                            10 + PAYLOAD_BYTES: uart_data <= checksum[7:0];
                            default: uart_data <= checksum[15:8];
                        endcase
                        state <= START_BYTE;
                    end
                end
                RAM_WAIT: state <= RAM_LATCH;
                RAM_LATCH: begin
                    pixel_data <= read_data;
                    uart_data <= RGB_CAPTURE ? read_data[23:16] : read_data[7:0];
                    state <= START_BYTE;
                end
                START_BYTE: begin
                    if (!uart_busy) begin
                        uart_start <= 1;
                        state <= WAIT_BUSY;
                    end
                end
                WAIT_BUSY: if (uart_busy) state <= WAIT_DONE;
                WAIT_DONE: if (!uart_busy) begin
                    if (byte_index == PACKET_BYTES - 1)
                        state <= WAIT_FRAME;
                    else begin
                        if (byte_index >= 10 &&
                            byte_index < 10 + PAYLOAD_BYTES) begin
                            if (RGB_CAPTURE) begin
                                if (color_channel == 2) begin
                                    color_channel <= 0;
                                    payload_addr <= payload_addr + 1'b1;
                                end else color_channel <= color_channel + 1'b1;
                            end else payload_addr <= payload_addr + 1'b1;
                        end
                        byte_index <= byte_index + 1'b1;
                        state <= PREPARE;
                    end
                end
                default: state <= WAIT_FRAME;
            endcase
        end
    end

    sc520_uart_tx #(.DIVISOR(DIVISOR)) u_tx (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_start(uart_start),
        .I_data(uart_data), .O_busy(uart_busy), .O_tx(O_uart_tx)
    );
endmodule

module sc520_uart_tx #(
    parameter integer DIVISOR = 651
) (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_start,
    input wire [7:0] I_data,
    output reg O_busy,
    output reg O_tx
);
    localparam integer DIV_W = $clog2(DIVISOR);
    reg [DIV_W-1:0] ticks;
    reg [3:0] bit_index;
    reg [7:0] held;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ticks <= 0;
            bit_index <= 0;
            held <= 0;
            O_busy <= 0;
            O_tx <= 1;
        end else if (!O_busy) begin
            O_tx <= 1;
            if (I_start) begin
                held <= I_data;
                ticks <= DIVISOR - 1;
                bit_index <= 0;
                O_tx <= 0;
                O_busy <= 1;
            end
        end else if (ticks != 0) begin
            ticks <= ticks - 1'b1;
        end else begin
            ticks <= DIVISOR - 1;
            if (bit_index < 8) begin
                O_tx <= held[bit_index];
                bit_index <= bit_index + 1'b1;
            end else if (bit_index == 8) begin
                O_tx <= 1;
                bit_index <= 9;
            end else begin
                O_busy <= 0;
            end
        end
    end
endmodule
