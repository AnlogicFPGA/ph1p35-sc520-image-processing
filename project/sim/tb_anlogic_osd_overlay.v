`timescale 1ns/1ps
module tb_anlogic_osd_overlay;
    reg enable; reg [2:0] mode; reg [11:0] x,y; reg [23:0] rgb_in;
    wire [23:0] rgb_out;
    reg detect_valid=0; reg [2:0] detect_class=0;
    reg test_done=0;
    anlogic_osd_overlay dut(.I_enable(enable),.I_mode(mode),.I_camera_status(9'h004),.I_ae_auto(1'b0),.I_ae_meter(32'd0),.I_wb_mode(2'd0),.I_wb_calibrated(1'b1),.I_x(x),
        .I_y(y),.I_rgb(rgb_in),.O_rgb(rgb_out),
        .I_detect_valid(detect_valid),.I_detect_class(detect_class),
        .I_detect_x_min(12'd400),.I_detect_x_max(12'd599),
        .I_detect_y_min(12'd300),.I_detect_y_max(12'd499),
        .I_center_x_bcd(16'h0499),.I_center_y_bcd(16'h0399),
        .I_track_diag_reason(3'd0),.I_track_diag_count_bcd(16'd0));
    task check_pixel;
        input [11:0] tx,ty; input [23:0] expected;
        begin
            x=tx; y=ty; #1;
            if(rgb_out!==expected) begin
                $display("FAIL x=%0d y=%0d expected=%h got=%h",tx,ty,expected,rgb_out);
                $finish;
            end
        end
    endtask
    initial begin
        enable=1'b1; mode=3'd1; rgb_in=24'h123456;
        check_pixel(0,0,24'h123456);
        check_pixel(24,24,24'h101820);
        check_pixel(112,44,24'hffffff);
        enable=1'b0;
        check_pixel(24,24,24'h123456);
        enable=1; mode=5;
        check_pixel(160,170,24'h123456); // Mode 5 has no ROI overlay.
        check_pixel(160,160,24'h123456);
        mode=6;
        check_pixel(400,350,24'h123456); // no stale box with no target
        detect_valid=1;detect_class=6;
        check_pixel(400,350,24'h00ff00);
        check_pixel(450,350,24'h123456);
        check_pixel(499,399,24'hff00ff); // box center
        if(dut.F_target_char(1,6,7)!="M" || dut.F_target_char(1,5,7)!="H")
            $fatal(1,"TRACK/HOLD label mismatch");
        check_pixel(62,120,24'hffffff); // decimal coordinate '0'
        detect_class=5; check_pixel(400,350,24'hfff200);
        mode=0;check_pixel(400,350,24'h123456); // COLOR has no target graphics
        test_done=1; $display("ANLOGIC OSD TEST PASS");
        $finish;
    end
endmodule
