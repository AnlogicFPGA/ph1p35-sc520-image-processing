// Image mode and independent processing-threshold controller.
// pulse[0]: next mode; pulse[1]: active-mode threshold + step;
// pulse[2]: active-mode threshold - step;
// pulse[3]: restore the active mode's default threshold.
// In COLOR/TRACK/OBJECT, pulse[1]/[2] select lower/higher exposure and pulse[3]
// cycles analog gain through toggle events to the sensor controller.
module image_mode_ctrl #(
    parameter [2:0]  DEFAULT_MODE             = 3'd1,
    parameter integer TRACK_MODE_ENABLE      = 0,
    parameter integer PINGPONG_MODE_ENABLE       = 0,
    parameter [7:0]  BINARY_DEFAULT_THRESHOLD = 8'd128,
    parameter [7:0]  BINARY_THRESHOLD_STEP    = 8'd8,
    parameter [10:0] EDGE_DEFAULT_THRESHOLD   = 11'd256,
    parameter [10:0] EDGE_THRESHOLD_STEP      = 11'd32
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [3:0] I_press_pulse,
    output reg  [2:0] O_mode_request,
    output reg  [7:0] O_binary_threshold_request,
    output reg [10:0] O_edge_threshold_request,
    output reg  [2:0] O_camera_adjust_toggle
);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_mode_request             <= DEFAULT_MODE==3'd5 ? 3'd0 : DEFAULT_MODE;
            O_camera_adjust_toggle     <= 3'b000;
            O_binary_threshold_request <= BINARY_DEFAULT_THRESHOLD;
            O_edge_threshold_request   <= EDGE_DEFAULT_THRESHOLD;
        end else begin
            // Toggle events survive the transfer from 100 MHz to sensor clock.
            // Debounced events are at least 20 ms apart. K1 has priority.
            if (!I_press_pulse[0] &&
                (O_mode_request == 3'd0 ||
                 (TRACK_MODE_ENABLE && O_mode_request == 3'd6) ||
                 (PINGPONG_MODE_ENABLE && O_mode_request == 3'd7))) begin
                if (I_press_pulse[3])
                    O_camera_adjust_toggle[2] <= ~O_camera_adjust_toggle[2];
                else if (I_press_pulse[1])
                    O_camera_adjust_toggle[0] <= ~O_camera_adjust_toggle[0];
                else if (I_press_pulse[2])
                    O_camera_adjust_toggle[1] <= ~O_camera_adjust_toggle[1];
            end
            if (I_press_pulse[0]) begin
                case(O_mode_request)
                    3'd4: O_mode_request <= 3'd0;
                    3'd0: O_mode_request <= TRACK_MODE_ENABLE ? 3'd6 :
                                           PINGPONG_MODE_ENABLE ? 3'd7 : 3'd1;
                    3'd5: O_mode_request <= 3'd0; // Reserved mode ID returns to COLOR.
                    3'd6: O_mode_request <= PINGPONG_MODE_ENABLE ? 3'd7 : 3'd1;
                    3'd7: O_mode_request <= 3'd1; // PINGPONG -> GRAY
                    default: O_mode_request <= O_mode_request + 1'b1;
                endcase
            end

            // Threshold keys affect only the processing mode currently shown.
            // COLOR/TRACK/OBJECT use K2/K3/K4 for camera calibration instead.
            if (O_mode_request == 3'd2) begin
                if (I_press_pulse[3]) begin
                    O_binary_threshold_request <= BINARY_DEFAULT_THRESHOLD;
                end else if (I_press_pulse[1]) begin
                    if (O_binary_threshold_request > (8'hff - BINARY_THRESHOLD_STEP))
                        O_binary_threshold_request <= 8'hff;
                    else
                        O_binary_threshold_request <= O_binary_threshold_request
                                                    + BINARY_THRESHOLD_STEP;
                end else if (I_press_pulse[2]) begin
                    if (O_binary_threshold_request < BINARY_THRESHOLD_STEP)
                        O_binary_threshold_request <= 8'h00;
                    else
                        O_binary_threshold_request <= O_binary_threshold_request
                                                    - BINARY_THRESHOLD_STEP;
                end
            end else if (O_mode_request == 3'd3) begin
                if (I_press_pulse[3]) begin
                    O_edge_threshold_request <= EDGE_DEFAULT_THRESHOLD;
                end else if (I_press_pulse[1]) begin
                    if (O_edge_threshold_request > (11'h7ff - EDGE_THRESHOLD_STEP))
                        O_edge_threshold_request <= 11'h7ff;
                    else
                        O_edge_threshold_request <= O_edge_threshold_request
                                                  + EDGE_THRESHOLD_STEP;
                end else if (I_press_pulse[2]) begin
                    if (O_edge_threshold_request < EDGE_THRESHOLD_STEP)
                        O_edge_threshold_request <= 11'h000;
                    else
                        O_edge_threshold_request <= O_edge_threshold_request
                                                  - EDGE_THRESHOLD_STEP;
                end
            end
        end
    end

endmodule
