`timescale 1ns / 1ps
// =====================================================================
// snn_cnn_sel : single-node SNN/CNN mode splitter
// ---------------------------------------------------------------------
//  Takes ONE GPIO bit (mode_sel) and produces the two complementary
//  clock-enable signals, so SNN and CNN are always mutually exclusive:
//
//      snn_en     =  mode_sel
//      cnn_clk_en = ~mode_sel      (=> snn_en = ~cnn_clk_en)
//
//  mode_sel = 1 -> SNN mode (SNN clocked, CNN gated off)
//  mode_sel = 0 -> CNN mode (CNN clocked, SNN gated off)
//
//  Wire in the block design:
//      GPIO[0]            -> mode_sel
//      snn_en             -> SNN  clk_en_gate.en
//      cnn_clk_en         -> dronet_accel_axi_0/cnn_clk_en
//
//  Pure combinational; the metastability sync lives inside each
//  clk_en_gate (on the respective clk_in domain).
// =====================================================================
module snn_cnn_sel (
    input  wire mode_sel,
    output wire snn_en,
    output wire cnn_clk_en
);
    assign snn_en     =  mode_sel;
    assign cnn_clk_en = ~mode_sel;
endmodule
