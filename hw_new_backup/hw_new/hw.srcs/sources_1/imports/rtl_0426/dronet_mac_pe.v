`timescale 1ns / 1ps

module dronet_mac_pe (
    input  wire signed [7:0]  in_val,
    input  wire signed [7:0]  weight,
    input  wire signed [31:0] acc_in,
    output wire signed [31:0] acc_out
);

    assign acc_out = acc_in + (in_val * weight);

endmodule
