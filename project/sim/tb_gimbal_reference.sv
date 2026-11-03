`timescale 1ns/1ps
module tb_gimbal_reference;
    localparam integer REF_X=684, REF_Y=264;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, observation=0, enable=1, valid=1, test_done=0;
    reg [31:0] now_ms=1000, frame_id=990;
    reg [11:0] center_x=REF_X,center_y=REF_Y;
    wire [15:0] pan[0:1],tilt[0:1];
    integer checks=0,n,p0,t0;
    generate for(genvar g=0;g<2;g++)begin:controllers
        gf_follow_controller #(
            .ANTI_HUNT(1),.BOTH_AXES(1),.CONFIRM_COUNT(1),
            .CONTROL_INTERVAL_US(20),.TIMEOUT_US(1000),.MAX_STEP_US(8),
            .PAN_MIN_US(500),.PAN_MAX_US(2500),
            .TILT_MIN_US(500),.TILT_MAX_US(2500),
            .PAN_REVERSE(1),.TILT_REVERSE(1)
        ) dut(
            .I_clk(clk),.I_rst_n(rst_n),.I_tick_us(1'b1),
            .I_enable(enable),.I_stream_restart(1'b0),
            .I_observation(observation),.I_valid(valid),.I_fresh(1'b1),
            .I_frame_id(frame_id),.I_now_ms(now_ms),.I_age_ms(16'd10),
            .I_xmin(center_x-12'd25),.I_xmax(center_x+12'd25),
            .I_ymin(center_y-12'd25),.I_ymax(center_y+12'd25),
            .I_center_x(center_x),.I_center_y(center_y),
            .I_reference_x(g==0 ? 12'd640 : 12'd684),
            .I_reference_y(g==0 ? 12'd360 : 12'd264),
            .O_pan_us(pan[g]),.O_tilt_us(tilt[g]),
            .O_locked(),.O_update(),.O_state(),.O_reason(),.O_accepted(),.O_rejected()
        );
    end endgenerate
    task tick;
        begin @(posedge clk);#1;@(negedge clk);now_ms=now_ms+1;end
    endtask
    task advance(input integer count);repeat(count)tick();endtask
    task sample;
        begin frame_id=now_ms-10;observation=1;tick();observation=0;end
    endtask
    task reset_case;
        begin rst_n=0;observation=0;enable=1;valid=1;advance(4);rst_n=1;end
    endtask
    task check(input bit ok,input string message);
        begin
            checks=checks+1;
            if(!ok)begin $display("FAIL REFERENCE %0d %s",checks,message);$fatal(1);end
        end
    endtask
    initial begin
        reset_case();center_x=REF_X;center_y=REF_Y;sample();advance(24);
        check(pan[1]==1500 && tilt[1]==1500,"reference position holds both axes");
        check(pan[0]<1500 && tilt[0]>1500,"optical center differs from laser reference");
        reset_case();center_x=640;center_y=360;sample();advance(24);
        check(pan[0]==1500 && tilt[0]==1500,"optical reference remains unchanged in comparison instance");
        check(pan[1]>1500,"horizontal correction uses calibrated reference and existing polarity");
        check(tilt[1]<1500,"vertical correction uses calibrated reference and existing polarity");
        reset_case();center_x=682;center_y=352;sample();advance(24);
        check(pan[1]==1500,"near-reference horizontal center stays within hold band");
        check(tilt[1]<1500,"vertical reference offset requests correction");
        check(tilt[0]==1500,"comparison instance holds at its vertical boundary");
        reset_case();center_x=REF_X;center_y=REF_Y;sample();advance(24);
        p0=pan[1];t0=tilt[1];
        for(n=0;n<12;n=n+1)begin
            center_x=n[0] ? REF_X+10 : REF_X-10;
            center_y=n[0] ? REF_Y+6 : REF_Y-6;
            sample();advance(24);
        end
        check(pan[1]==p0 && tilt[1]==t0,"existing stop band rejects local coordinate noise");
        check(controllers[1].dut.PAN_STOP_PX==12 && controllers[1].dut.TILT_STOP_PX==8,"stop thresholds are retained");
        check(controllers[1].dut.PAN_START_PX==20 && controllers[1].dut.TILT_START_PX==16,"start thresholds are retained");
        valid=0;sample();advance(24);
        check(!controllers[1].dut.pan_pending && !controllers[1].dut.tilt_pending,"invalid observation cancels pending motion");
        enable=0;advance(3);
        check(controllers[1].dut.O_state==0,"disabled mode clears tracking state");
        check(pan[1]==p0 && tilt[1]==t0,"disabled mode preserves the position command");
        test_done=1;
        $display("PASS REFERENCE checks=%0d reference=(%0d,%0d)",checks,REF_X,REF_Y);
        $finish;
    end
endmodule
