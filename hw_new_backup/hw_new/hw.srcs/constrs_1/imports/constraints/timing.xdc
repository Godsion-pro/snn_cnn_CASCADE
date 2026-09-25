# Maximum targeted pixel clock frequency for dynamic video clock generator is 148.5 MHz.
# However, BUFIO/BUFR/OSERDESE2 primitives need to be overclocked for FPGA. The maximum they will do is 600/120MHz.
# So we are underconstraining the pixel clock tree in general to avoid pulse width errors in the report on those primitives.
# However, we are constraining the pixel clock to its actual frequency. This way the quality of implementation will not change.

# Pixel clock tree underconstrained to 120 MHz to avoid pulse width errors on BUFIO/BUFR/OSERDESE2
# The MMCM outputs 5x this frequency for DVI serial clock, and is divided back by a BUFR.
# Period = 1/(5*120MHz) = 1.667 ns
create_clock -period 1.667 -name video_dynclk [get_pins -regexp .*video_dynclk/.*/mmcm_adv_inst/CLKOUT0 -hierarchical]
# Uncomment below to disable underconstraining and live with the Pulse Width errors (line with BUFR clock further down needs to be commented)
#create_clock -period 1.347 -name video_dynclk [get_pins -regexp .*video_dynclk/.*/mmcm_adv_inst/CLKOUT0 -hierarchical]

# Pixel clock constrained to 148.5 MHz on the output of BUFR
# Works because the clock path delay is not necessary to be analyzed all the way from the source clock of the MMCM, since there is no
# phase requirement between the source clock and the pixel clock
# Comment below to disable underconstraining and live with the Pulse Width errors
create_clock -period 6.734 -name pixel_dynclk [get_pins -regexp .*DVIClocking_0/U0/PixelClkBuffer/O -hierarchical]

# MIPI D-PHY data rate 420Mbps/lane = 210 MHz HS_Clk
create_clock -period 4.761 -name dphy_hs_clock_p -waveform {0.000 2.380} [get_ports dphy_hs_clock_clk_p]

# Workaround for FIFO XDC not getting applied (it seems there is no need for this anymore in 2017.4)
#set_false_path -through [get_pins system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/DataFIFO/s_aresetn] -to [get_pins -hierarchical -filter {NAME =~ system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/DataFIFO/*rstblk*/*PRE}]
#set_false_path -from [get_cells -hierarchical -filter {NAME =~ system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/DataFIFO/*rstblk*/*rst_reg_reg[*]}]

#set_false_path -from [get_pins {system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/SyncSReset/SyncAsyncx/oSyncStages_reg[1]/C}] -to [get_pins system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/DataFIFO/U0/inst_fifo_gen/gaxis_fifo.gaxisf.axisf/grf.rf/rstblk/ngwrdrst.grst.g7serrst.rst_rd_reg1_reg/PRE]
#set_false_path -from [get_pins {system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/SyncSReset/SyncAsyncx/oSyncStages_reg[1]/C}] -to [get_pins system_i/MIPI_CSI_2_RX_0/U0/MIPI_CSI2_Rx_inst/LLP_inst/DataFIFO/U0/inst_fifo_gen/gaxis_fifo.gaxisf.axisf/grf.rf/rstblk/ngwrdrst.grst.g7serrst.rst_rd_reg2_reg/PRE]

#=============================================================
# Combined CDC constraints
#   - clk_fpga_0 (100MHz, s_axi_aclk)
#   - clk_out2_pcam_base_720p_clk_wiz_0_0 (150MHz, s_axis_aclk)
#=============================================================

#-------------------------------------------------------------
# 1) snn_top : weight memory (BRAM TDP) CDC
#    write port (s_axi_aclk, 100MHz) ↔ read port (s_axis_aclk, 150MHz)
#    BRAM이 도메인 분리를 물리적으로 처리하므로 양방향 false_path.
#-------------------------------------------------------------
# write(100MHz) → read(150MHz)
set_false_path \
    -from [get_clocks clk_fpga_0] \
    -to   [get_clocks clk_out2_pcam_base_720p_clk_wiz_0_0] \
    -through [get_cells -hier -filter {NAME =~ *u_snn_top/fc1_w_reg*}]

set_false_path \
    -from [get_clocks clk_fpga_0] \
    -to   [get_clocks clk_out2_pcam_base_720p_clk_wiz_0_0] \
    -through [get_cells -hier -filter {NAME =~ *u_snn_top/fc2_w_reg*}]

# 반대 방향도 보수적으로 처리
set_false_path \
    -from [get_clocks clk_out2_pcam_base_720p_clk_wiz_0_0] \
    -to   [get_clocks clk_fpga_0] \
    -through [get_cells -hier -filter {NAME =~ *u_snn_top/fc1_w_reg*}]

set_false_path \
    -from [get_clocks clk_out2_pcam_base_720p_clk_wiz_0_0] \
    -to   [get_clocks clk_fpga_0] \
    -through [get_cells -hier -filter {NAME =~ *u_snn_top/fc2_w_reg*}]

#-------------------------------------------------------------
# 2) Top : gpio_threshold 2-FF synchronizer
#    AXI GPIO channel 2, 100MHz → Top threshold sync, 150MHz
#-------------------------------------------------------------
set_max_delay -datapath_only \
    -from [get_cells -hier -filter {NAME =~ *axi_gpio_0/U0/gpio_core_1/Dual.gpio2_Data_Out_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *Top_0/U0/gpio_thr_sync1_reg*}] \
    6.667

set_bus_skew \
    -from [get_cells -hier -filter {NAME =~ *axi_gpio_0/U0/gpio_core_1/Dual.gpio2_Data_Out_reg*}] \
    -to   [get_cells -hier -filter {NAME =~ *Top_0/U0/gpio_thr_sync1_reg*}] \
    6.667

#-------------------------------------------------------------
# 3) dronet_accel_axi : status reporting CDC
#    s_axis_aclk (150MHz) → s_axi_aclk (100MHz)
#    Destination clock period = 10.000 ns
#-------------------------------------------------------------
# 3-1) completed_frame_id (16-bit data sync, frame당 1회 변경, 33ms간 stable)
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/completed_frame_id_w_reg[*]}] -to [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/cfid_meta1_reg[*]}] 10.000

set_bus_skew -from [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/completed_frame_id_w_reg[*]}] -to [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/cfid_meta1_reg[*]}] 10.000

# 3-2) completed_buf_sel (1-bit data sync)
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/completed_buf_sel_w_reg}] -to [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/cbs_meta1_reg}] 10.000

# 3-3) frame_done_toggle (1-bit toggle sync)
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/frame_done_toggle_w_reg}] -to [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/frame_done_sync_r_reg[0]}] 10.000

# 3-4) last_complete_valid (1-bit level sync)
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/last_complete_valid_w_reg}] -to [get_cells -hier -filter {NAME =~ *u_pingpong_frame_buffer/lcv_meta1_reg}] 10.000

# 3-5) flying_obj_toggle (1-bit toggle sync)
set_max_delay -datapath_only -from [get_cells -hier -filter {NAME =~ *dronet_accel_axi_0/U0/flying_obj_toggle_axis_reg}] -to [get_cells -hier -filter {NAME =~ *dronet_accel_axi_0/U0/flying_obj_sync_reg[0]}] 10.000




set_false_path -from [get_ports {{dphy_data_hs_p[*]} {dphy_data_hs_n[*]}}] -to [get_pins -hier -filter {NAME =~ "*MIPI_D_PHY_RX_0*HSDeserializerX*Deserializer/DDLY"}]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk]
