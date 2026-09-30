set proj_dir [get_property DIRECTORY [current_project]]
set wdir [file normalize "$proj_dir/cnn_weights"]

set_property CONFIG.CONV1_W_FILE [file normalize "$wdir/conv1_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.CONV1_B_FILE [file normalize "$wdir/conv1_b.mem"] [get_bd_cells dronet_accel_axi_0]

set_property CONFIG.CONV2_W_FILE [file normalize "$wdir/conv2_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.CONV2_B_FILE [file normalize "$wdir/conv2_b.mem"] [get_bd_cells dronet_accel_axi_0]

set_property CONFIG.CONV3_W_FILE [file normalize "$wdir/conv3_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.CONV3_B_FILE [file normalize "$wdir/conv3_b.mem"] [get_bd_cells dronet_accel_axi_0]

set_property CONFIG.CONV4_W_FILE [file normalize "$wdir/conv4_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.CONV4_B_FILE [file normalize "$wdir/conv4_b.mem"] [get_bd_cells dronet_accel_axi_0]

set_property CONFIG.CONV5_W_FILE [file normalize "$wdir/conv5_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.CONV5_B_FILE [file normalize "$wdir/conv5_b.mem"] [get_bd_cells dronet_accel_axi_0]

set_property CONFIG.DET_W_FILE [file normalize "$wdir/det_w.mem"] [get_bd_cells dronet_accel_axi_0]
set_property CONFIG.DET_B_FILE [file normalize "$wdir/det_b.mem"] [get_bd_cells dronet_accel_axi_0]

validate_bd_design
save_bd_design