

module design_top_wrapper #(
    parameter integer GIMBAL_TARGET_CLASS=0,GIMBAL_LOST_FRAMES=15,
    parameter integer GIMBAL_REF_X=684,GIMBAL_REF_Y=264,
    parameter integer GIMBAL_DIAG_ENABLE=1,
    parameter integer GIMBAL_PAN_MIN_US=500,GIMBAL_PAN_MAX_US=2500,
    parameter integer GIMBAL_TILT_MIN_US=500,GIMBAL_TILT_MAX_US=2500,
    parameter integer GIMBAL_PAN_REVERSE=1,GIMBAL_TILT_REVERSE=1
)(
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    input wire [3:0]  I_button,
    output wire       O_gimbal_pan_pwm,O_gimbal_tilt_pwm,

    output wire       O_screen_pwm,
    output wire       O_capture_uart_tx,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
    wire S_cam_ae_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
    wire [2:0] S_camera_adjust_toggle;
    wire [12:0] S_camera_status_24m, S_camera_status_hdmi;
    wire [8:0] S_camera_write_status_24m;
    wire [69:0] S_ae_stats_csi, S_ae_stats_24m;
    wire [31:0] S_ae_meter_24m, S_ae_meter_hdmi;
    wire [7:0] S_ae_mean_24m = S_ae_stats_24m[61:36] / 26'd230400;
    wire [7:0] S_ae_near_pct_24m = S_ae_stats_24m[35:18] / 18'd2304;
    wire [7:0] S_ae_clip_pct_24m = S_ae_stats_24m[17:0] / 18'd2304;
    assign S_ae_meter_24m={S_ae_mean_24m,S_ae_near_pct_24m,
                            S_ae_clip_pct_24m,S_ae_stats_24m[69:62]};
    wire S_ae_auto_active;
    wire [1:0] S_wb_mode_camera,S_wb_mode_display;
    wire S_wb_toggle,S_key1_short,S_key4_short,S_ae_resume_toggle;
    wire [3:0] S_button_stable_n,S_button_action_pulse;
    wire S_wb_calibrated;
    (* preserve=1,keep=1,ASYNC_REG="TRUE" *) reg S_wb_calibrated_sync1,S_wb_calibrated_sync2;
    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin S_wb_calibrated_sync1<=0;S_wb_calibrated_sync2<=0;end
        else begin S_wb_calibrated_sync1<=S_wb_calibrated;S_wb_calibrated_sync2<=S_wb_calibrated_sync1;end
    end
    assign S_camera_status_24m={S_ae_auto_active,S_wb_calibrated_sync2,S_wb_mode_display,S_camera_write_status_24m};
    assign S_button_action_pulse={S_key4_short&S_button_stable_n[0],
                                  S_button_press_pulse[2:1]&{2{S_button_stable_n[0]}},
                                  S_key1_short};
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
	(* keep = "true" *) wire [15:0] S_csi_data_length;
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;

	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	wire [2:0]      S_image_mode_active;
	wire [7:0]      S_binary_threshold_active;
	wire [10:0]     S_edge_threshold_active;
	wire [3:0]      S_button_press_pulse;
	wire [2:0]      S_image_mode_request_100m;
	wire [7:0]      S_binary_threshold_request_100m;
	wire [10:0]     S_edge_threshold_request_100m;
	reg  [2:0]      S_image_mode_request_csi_1d;
	reg  [2:0]      S_image_mode_request_csi_2d;
	reg  [7:0]      S_binary_threshold_request_csi_1d;
	reg  [7:0]      S_binary_threshold_request_csi_2d;
	reg  [10:0]     S_edge_threshold_request_csi_1d;
	reg  [10:0]     S_edge_threshold_request_csi_2d;
	reg  [2:0]      S_image_mode_hdmi_1d;
	reg  [2:0]      S_image_mode_hdmi_2d;
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire        S_frame_start_ddr;
    wire[1:0]   S_video_out_rp;     
    wire[31:0]  S_hdmi_latency_ms;
    wire[127:0] S_vi_tagged_wr_data;

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    wire        S_video_rd_valid;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;
    wire        S_csi_packet_start;
    reg         S_csi_length_active_d;
    reg [10:0]  S_mipi_line_count;
    reg         S_mipi_length_error;
    reg         S_mipi_lane_error;
    reg         S_mipi_frame_seen;
    reg         S_mipi_frame_done;
    reg         S_mipi_last_length_error;
    reg         S_mipi_last_line_error;
    reg         S_mipi_last_lane_error;
    reg         S_mipi_frame_open;
    reg         S_mipi_previous_frame_error;
    wire[3:0]   S_mipi_diag_status_async;
    reg [3:0]   S_mipi_diag_status_sync1;
    reg [3:0]   S_mipi_diag_status_sync2;


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    // A RAW10 line contains 1280 * 10 / 8 = 1600 bytes.  The CSI parser
    // reports the number of 16-bit lane words, therefore the expected value
    // is 1600 / 2 = 800 = 16'h0320.  Count one long packet whenever that
    // decoded length changes from zero to non-zero.
    assign S_csi_packet_start = (S_csi_data_length != 16'd0) &&
                                !S_csi_length_active_d;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_csi_length_active_d      <= 1'b0;
            S_mipi_line_count          <= 11'd0;
            S_mipi_length_error        <= 1'b0;
            S_mipi_lane_error          <= 1'b0;
            S_mipi_frame_seen          <= 1'b0;
            S_mipi_frame_done          <= 1'b0;
            S_mipi_last_length_error   <= 1'b0;
            S_mipi_last_line_error     <= 1'b0;
            S_mipi_last_lane_error     <= 1'b0;
            S_mipi_frame_open          <= 1'b0;
            S_mipi_previous_frame_error <= 1'b0;
        end
        else begin
            S_csi_length_active_d <= (S_csi_data_length != 16'd0);

            if(S_csi_frame_start) begin
                // A new start before an end marks the preceding frame bad.
                if(S_mipi_frame_open)
                    S_mipi_previous_frame_error <= 1'b1;
                S_mipi_frame_open <= 1'b1;
                S_mipi_line_count   <= 11'd0;
                S_mipi_length_error <= 1'b0;
                S_mipi_lane_error   <= 1'b0;
                S_mipi_frame_seen   <= 1'b1;
            end
            else begin
                if(S_csi_packet_start) begin
                    S_mipi_line_count <= S_mipi_line_count + 11'd1;
                    if(S_csi_data_length != 16'h0320)
                        S_mipi_length_error <= 1'b1;
                end

                if(S_lane_error_any)
                    S_mipi_lane_error <= 1'b1;
            end

            // Publish only a completed frame, so the HDMI-domain diagnostic
            // sees a stable result rather than counters changing every line.
            if(S_csi_frame_end) begin
                S_mipi_frame_open <= 1'b0;
                S_mipi_previous_frame_error <= S_mipi_length_error ||
                    (S_mipi_line_count != 11'd720) || S_mipi_lane_error ||
                    S_lane_error_any || !S_mipi_frame_open;
                S_mipi_frame_done        <= 1'b1;
                S_mipi_last_length_error <= S_mipi_length_error;
                S_mipi_last_line_error   <= (S_mipi_line_count != 11'd720);
                S_mipi_last_lane_error   <= S_mipi_lane_error |
                                            S_lane_error_any;
            end
        end
    end

    // Diagnostic result codes consumed by hdmi_mixer DEBUG_MODE=4:
    // 0=config incomplete, 1=no HS traffic, 2=no completed CSI frame,
    // 3=lane error, 4=wrong RAW10 line length, 5=wrong lines/frame,
    // 6=the observed MIPI frame structure is correct, 7=I2C error.
    // I2C error has the highest priority because O_cfg_done only indicates
    // that the write sequencer reached its end; it does not prove that every
    // sensor register write was acknowledged successfully.
    assign S_mipi_diag_status_async =
        S_cam_i2c_error              ? 4'd7 :
        !S_cam_cfg_done              ? 4'd0 :
        !S_dbg_hs_seen               ? 4'd1 :
        !(S_mipi_frame_seen &&
          S_mipi_frame_done)         ? 4'd2 :
        S_mipi_last_lane_error       ? 4'd3 :
        S_mipi_last_length_error     ? 4'd4 :
        S_mipi_last_line_error       ? 4'd5 : 4'd6;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_mipi_diag_status_sync1 <= 4'd0;
            S_mipi_diag_status_sync2 <= 4'd0;
        end
        else begin
            S_mipi_diag_status_sync1 <= S_mipi_diag_status_async;
            S_mipi_diag_status_sync2 <= S_mipi_diag_status_sync1;
        end
    end

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = S_mipi_diag_status_sync2;
	// J9 pin 37 is connected to the module's PWDNB/XSHUTDN input.
	// Hold the sensor in power-down after PLL lock, then allow its internal
	// power-up sequence to finish before the first I2C transaction.
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // The four on-board keys are active-low. Debounce them in the stable
  // 100 MHz domain, then retain each requested value until another press.
  key_debounce_4 #(
      .DEBOUNCE_CYCLES(2000000),
      .COUNTER_WIDTH(21)
  ) u_key_debounce_4 (
      .I_clk(S_100m_clk),
      .I_rst_n(S_rst_n),
      .I_key_n(I_button),
      .O_press_pulse(S_button_press_pulse),
      .O_key_n(S_button_stable_n)
  );

  key_short_long u_key1_action (
      .I_clk(S_100m_clk),.I_rst_n(S_rst_n),.I_key_n(S_button_stable_n[0]),
      .O_short_pulse(S_key1_short),.O_long_toggle(S_wb_toggle)
  );
  key_short_long u_key4_action (
      .I_clk(S_100m_clk),.I_rst_n(S_rst_n),.I_key_n(S_button_stable_n[3]),
      .O_short_pulse(S_key4_short),.O_long_toggle(S_ae_resume_toggle)
  );
  wb_mode_select u_wb_mode_camera (
      .I_clk(S_csi_rx_clk),.I_rst_n(S_rst_n),.I_toggle(S_wb_toggle),.O_mode(S_wb_mode_camera)
  );
  wb_mode_select u_wb_mode_display (
      .I_clk(S_24m_clk),.I_rst_n(S_rst_n),.I_toggle(S_wb_toggle),.O_mode(S_wb_mode_display)
  );

  image_mode_ctrl #(.DEFAULT_MODE(3'd0), .TRACK_MODE_ENABLE(1),
                    .PINGPONG_MODE_ENABLE(1)) u_image_mode_ctrl (
      .I_clk(S_100m_clk),
      .I_rst_n(S_rst_n),
      .I_press_pulse(S_button_action_pulse),
      .O_mode_request(S_image_mode_request_100m),
      .O_binary_threshold_request(S_binary_threshold_request_100m),
      .O_edge_threshold_request(S_edge_threshold_request_100m),
      .O_camera_adjust_toggle(S_camera_adjust_toggle)
  );

  // The control bus changes only after a debounced key press and then stays
  // stable. Two-stage sampling transfers it into the camera stream domain;
  // the ISP itself commits the new values only at the next frame boundary.
  always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
      if(!S_rst_n) begin
          S_image_mode_request_csi_1d      <= 3'd1;
          S_image_mode_request_csi_2d      <= 3'd1;
          S_binary_threshold_request_csi_1d <= 8'd128;
          S_binary_threshold_request_csi_2d <= 8'd128;
          S_edge_threshold_request_csi_1d   <= 11'd256;
          S_edge_threshold_request_csi_2d   <= 11'd256;
      end else begin
          S_image_mode_request_csi_1d      <= S_image_mode_request_100m;
          S_image_mode_request_csi_2d      <= S_image_mode_request_csi_1d;
          S_binary_threshold_request_csi_1d <= S_binary_threshold_request_100m;
          S_binary_threshold_request_csi_2d <= S_binary_threshold_request_csi_1d;
          S_edge_threshold_request_csi_1d   <= S_edge_threshold_request_100m;
          S_edge_threshold_request_csi_2d   <= S_edge_threshold_request_csi_1d;
      end
  end

  // Send the active, frame-aligned mode to the HDMI-domain OSD.
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if(!S_hdmi_rst_n) begin
          S_image_mode_hdmi_1d <= 3'd1;
          S_image_mode_hdmi_2d <= 3'd1;
      end else begin
          S_image_mode_hdmi_1d <= S_image_mode_active;
          S_image_mode_hdmi_2d <= S_image_mode_hdmi_1d;
      end
  end

  cdc_status_snapshot #(.WIDTH(70),.RESET_VALUE(70'd0)) u_ae_stats_snapshot (
      .I_src_clk(S_csi_rx_clk), .I_src_rst_n(S_rst_n),
      .I_data(S_ae_stats_csi),
      .I_dst_clk(S_24m_clk), .I_dst_rst_n(S_rst_n),
      .O_data(S_ae_stats_24m)
  );

  cdc_status_snapshot #(.WIDTH(32),.RESET_VALUE(32'd0)) u_ae_meter_snapshot (
      .I_src_clk(S_24m_clk), .I_src_rst_n(S_rst_n),
      .I_data(S_ae_meter_24m),
      .I_dst_clk(S_hdmi_pixel_clk), .I_dst_rst_n(S_hdmi_rst_n),
      .O_data(S_ae_meter_hdmi)
  );

  camera_calibration_ctrl #(.AUTO_EXPOSURE_ENABLE(1)) u_camera_calibration_ctrl (
      .I_clk(S_24m_clk),
      .I_rst_n(S_rst_n),
      .I_adjust_toggle(S_camera_adjust_toggle),
      .I_auto_resume_toggle(S_ae_resume_toggle),
      .I_auto_stats(S_ae_stats_24m),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .I_iic_error(S_cam_ae_error),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag),
      .O_status(S_camera_write_status_24m),
      .O_auto_active(S_ae_auto_active)
  );

  cdc_status_snapshot #(.WIDTH(13),.RESET_VALUE(13'h1004)) u_camera_status_snapshot (
      .I_src_clk(S_24m_clk), .I_src_rst_n(S_rst_n),
      .I_data(S_camera_status_24m),
      .I_dst_clk(S_hdmi_pixel_clk), .I_dst_rst_n(S_hdmi_rst_n),
      .O_data(S_camera_status_hdmi)
  );
  
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error),
      .O_ae_error(S_cam_ae_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        ),
.O_data_length         ( S_csi_data_length )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// SC520CS原厂HD模式已经直接输出1280x720 RAW10，无需FPGA窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_mode_request  (S_image_mode_request_csi_2d),
.I_wb_mode_request(S_wb_mode_camera),
.O_wb_calibrated(S_wb_calibrated),
.O_ae_stats(S_ae_stats_csi),
.I_binary_threshold_request(S_binary_threshold_request_csi_2d),
.I_edge_threshold_request(S_edge_threshold_request_csi_2d),
.O_tready        (1'b1),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.O_mode_active   (S_image_mode_active),
.O_binary_threshold_active(S_binary_threshold_active),
.O_edge_threshold_active(S_edge_threshold_active),
.I_tready        ()
  );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( S_mipi_previous_frame_error ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_frame_start_ddr    ( S_frame_start_ddr     ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );

    frame_latency_inband_stamp #(.HDMI_CYCLES_PER_MS(75000))
    u_frame_latency_inband_stamp (
        .I_rst_n(S_rst_n),
        .I_hdmi_clk(S_hdmi_pixel_clk),
        .I_ddr_clk(S_ddr_clk),
        .I_frame_start_ddr(S_frame_start_ddr),
        .I_wr_en(S_vi_ddr_wr_en),
        .I_wr_addr(S_vi_ddr_wr_addr),
        .I_wr_data(S_vi_ddr_wr_data),
        .O_wr_data(S_vi_tagged_wr_data),
        .O_tagged(),
        .O_hdmi_ms(S_hdmi_latency_ms)
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_tagged_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_vdieo_valid       ( S_video_rd_valid     )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        // Normal camera display.  Set to 4 only when the MIPI diagnostic
        // full-screen status page is needed.
        .DEBUG_MODE ( 0    ),
        .CAMERA_CALIBRATION(1),
        .MOTION_TRACK_ENABLE(1),
        .TRACK_EVENT_COUNT(1),
        .PINGPONG_DETECT_ENABLE(1),
        .VIDEO_VALID_CHECK(1),
        .WARM_DISPLAY_ENABLE(1),
        .MIDTONE_DISPLAY_ENABLE(1),
        .CHROMA_DENOISE_ENABLE(1),
        .AE_DISPLAY(1),
        // RGB capture is disabled in the standard display configuration.
        // Capture frame RAM is omitted when this option is disabled.
        .SC520_CAPTURE_ENABLE(0),
        .SC520_RGB_CAPTURE(0),
        .OBJECT_MASK_PREVIEW(0),
        .OBJECT_RGB_PROBE(0),
        .OSD_PIPELINE_ENABLE(1),
        .GIMBAL_FOLLOW_ENABLE(1),.GIMBAL_ANTI_HUNT(1),.GIMBAL_BOTH_AXES(1),.GIMBAL_DIAG(GIMBAL_DIAG_ENABLE),
        .GIMBAL_PAN_MIN_US(GIMBAL_PAN_MIN_US),.GIMBAL_PAN_MAX_US(GIMBAL_PAN_MAX_US),
        .GIMBAL_TILT_MIN_US(GIMBAL_TILT_MIN_US),.GIMBAL_TILT_MAX_US(GIMBAL_TILT_MAX_US),.GIMBAL_TARGET_CLASS(GIMBAL_TARGET_CLASS),
        .GIMBAL_LOST_FRAMES(GIMBAL_LOST_FRAMES),
        // Registered read request and RGB output in u_video_out: two clocks.
        .GIMBAL_READ_LATENCY(2),
        .GIMBAL_REF_X(GIMBAL_REF_X),.GIMBAL_REF_Y(GIMBAL_REF_Y),
        .GIMBAL_PAN_REVERSE(GIMBAL_PAN_REVERSE),.GIMBAL_TILT_REVERSE(GIMBAL_TILT_REVERSE)
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_image_mode    ( S_image_mode_hdmi_2d),
        .I_camera_status ( S_camera_status_hdmi[8:0]),
        .I_ae_auto(S_camera_status_hdmi[12]),
        .I_ae_meter(S_ae_meter_hdmi),
        .I_wb_mode(S_camera_status_hdmi[10:9]),
        .I_wb_calibrated(S_camera_status_hdmi[11]),
        .I_latency_now_ms(S_hdmi_latency_ms),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .I_video_rd_valid( S_video_rd_valid   ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    ),
        .O_capture_uart_tx(O_capture_uart_tx),
        .O_gimbal_pan_pwm(O_gimbal_pan_pwm),.O_gimbal_tilt_pwm(O_gimbal_tilt_pwm)
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );

    
endmodule
