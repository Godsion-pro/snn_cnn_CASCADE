# =====================================================================
# run_power_saif.tcl
# ---------------------------------------------------------------------
#  SNN + CNN 서브블록(snn_cnn_power_dut) 만 떼서 SAIF 를 뽑고,
#  그 switching activity 를 report_power 에 적용하는 플로우.
#  레벨: post-synthesis functional simulation.
#
#  SAIF 측정 구간은 TB 의 $stop 두 개로 "결정적"으로 잡힌다:
#    [SAIF] MEASURE WINDOW START  -> $stop  (weight 로드 끝, 추론 직전)
#    [SAIF] MEASURE WINDOW END    -> $stop  (마지막 프레임 CNN 추론 완료)
#  => 그 사이 구간만 log_saif 로 기록 (weight 로드 제외).
#
#  준비물: sim_data/ 에 input_frames.hex, fc1_weight.hex, fc2_weight.hex,
#          cnn_w.hex, cnn_b.hex  (TB 가 절대경로로 읽음)
#  ★ part 는 보드에 맞게 교체.
# =====================================================================

set PART     xc7z020clg400-1
set DUT      snn_cnn_power_dut
set TB       tb_snn_cnn_power
set SAIF     snn_cnn.saif

set RTL_DIR  hw_new_backup/hw_new/hw.srcs/sources_1/imports/rtl_0426
set CORE_DIR CORE_IP/src
set NEW_DIR  hw_new_backup/hw_new/hw.srcs/sources_1/new
set SIM_DIR  hw_new_backup/hw_new/hw.srcs/sim_1/new

# =====================================================================
# 1) SNN+CNN DUT 만 Out-Of-Context 합성 -> functional netlist
# =====================================================================
read_verilog [glob $RTL_DIR/dronet_*.v]
read_verilog $RTL_DIR/clk_en_gate.v
read_verilog $RTL_DIR/snn_cnn_sel.v
read_verilog $CORE_DIR/snn_top.v
read_verilog $CORE_DIR/feature_extractor.v
read_verilog $CORE_DIR/Top.v
read_verilog $NEW_DIR/frame_diff.v
read_verilog $NEW_DIR/snn_cnn_power_dut.v

synth_design -top $DUT -part $PART -mode out_of_context
write_verilog -mode funcsim -force ${DUT}_funcsim.v
write_checkpoint -force ${DUT}_synth.dcp

# =====================================================================
# 2) Post-synth functional simulation + SAIF (배치 xsim)
# =====================================================================
exec xvlog ${DUT}_funcsim.v $SIM_DIR/${TB}.v
exec xvlog [file join $::env(XILINX_VIVADO) data verilog src glbl.v]
exec xelab -debug typical -L unisims_ver -L secureip ${TB} glbl -s ${TB}_sim

# SAIF 기록 스크립트.  TB 의 $stop 으로 구간이 결정적이라 시간 추측 불필요.
set fp [open run_xsim.tcl w]
puts $fp {
    run all                 ;# -> MEASURE WINDOW START 의 $stop 에서 멈춤
    open_saif snn_cnn.saif
    log_saif [get_objects -r /tb_snn_cnn_power/dut/*]
    run all                 ;# -> MEASURE WINDOW END 의 $stop 까지 (이 구간만 기록)
    close_saif
    run all                 ;# -> $finish 까지
    quit
}
close $fp
exec xsim ${TB}_sim -tclbatch run_xsim.tcl
puts "==> SAIF 생성 완료: $SAIF"

# =====================================================================
# 3-A) (가장 정확) 떼어낸 DUT 자체 전력 — 이름 100% 일치
# =====================================================================
#  open_checkpoint ${DUT}_synth.dcp
#  read_saif -strip_path ${TB}/dut $SAIF
#  report_power -file power_snn_cnn_block.rpt

# =====================================================================
# 3-B) 전체 BD 에 적용
# =====================================================================
#  open_run impl_1
#  read_saif -strip_path ${TB}/dut/u_top    -instance <BD>/Top_0/inst              $SAIF
#  read_saif -strip_path ${TB}/dut/u_dronet -instance <BD>/dronet_accel_axi_0/inst $SAIF
#  report_power -file power_full_bd_with_saif.rpt
#  주의: OOC 네트 이름이 BD in-context 와 달라 매칭 안 되는 노드는 vectorless 추정.
