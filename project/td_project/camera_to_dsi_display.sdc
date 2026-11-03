create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 2.415 -waveform {0.000 1.208} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]


set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

# Manual calibration CDC only. Do not cut entire PLL clock groups.
# Keep get_regs expressions inline: TD serializes these SDC object queries.
# Verify effective paths in report_timing_exception after synthesis/routing.
set_false_path -to [get_regs {u_camera_calibration_ctrl/toggle_sync1*}]
set_false_path -to [get_regs {u_camera_calibration_ctrl/auto_sync1*}]
set_false_path -to [get_regs {u_wb_mode_camera/toggle_sync1*}]
set_false_path -to [get_regs {u_wb_mode_display/toggle_sync1*}]
set_false_path -to [get_regs {u_camera_status_snapshot/req_sync1*}]
set_false_path -to [get_regs {u_camera_status_snapshot/ack_sync1*}]
set_false_path -to [get_regs {u_ae_meter_snapshot/req_sync1*}]
set_false_path -to [get_regs {u_ae_meter_snapshot/ack_sync1*}]
# Data is held through request -> 2 synchronizer stages -> capture -> ACK.
# Earliest capture is two complete 13.333 ns HDMI periods after launch.
# Budget 20 ns including clock-to-Q, route and setup (< 26.666 ns).
# TD datapath_only also suppresses the inapplicable cross-clock hold check.
set_max_delay -from [get_regs {u_camera_status_snapshot/held_data*}] -to [get_regs {u_camera_status_snapshot/sampled_data*}] -datapath_only 20.000
set_max_delay -from [get_regs {u_ae_meter_snapshot/held_data*}] -to [get_regs {u_ae_meter_snapshot/sampled_data*}] -datapath_only 20.000
