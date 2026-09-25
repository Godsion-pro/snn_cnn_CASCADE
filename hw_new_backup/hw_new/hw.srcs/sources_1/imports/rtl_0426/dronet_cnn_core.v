`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_cnn_core #(
    parameter integer CONV1_SHIFT = 8,
    parameter integer CONV2_SHIFT = 8,
    parameter integer CONV3_SHIFT = 8,
    parameter integer CONV4_SHIFT = 8,
    parameter integer CONV5_SHIFT = 8,
    parameter integer DET_SHIFT   = 8
) (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              soft_reset,
    input  wire                              start,
    input  wire                              test_pattern_enable,
    input  wire [15:0]                       frame_id,
    output wire [`DRONET_FRAME_ADDR_W-1:0]   frame_rd_addr,
    input  wire [7:0]                        frame_rd_data,
    output wire                              raw_wr_en,
    output wire [`DRONET_RAW_ADDR_W-1:0]     raw_wr_addr,
    output wire signed [7:0]                 raw_wr_data,
    output wire                              busy,
    output wire                              done_pulse,
    output reg  [15:0]                       raw_frame_id,
    output wire [3:0]                        current_step,
    output wire [23:0]                       cycles_left,
    output wire [7:0]                        dbg_last_frame_byte,
    // PS weight/bias load — passed through to compute engine
    input  wire                              w_load_en,
    input  wire [13:0]                       w_load_addr,
    input  wire signed [7:0]                 w_load_data,
    input  wire                              b_load_en,
    input  wire [6:0]                        b_load_addr,
    input  wire signed [31:0]                b_load_data
);

    dronet_compute_engine #(
        .CONV1_SHIFT  (CONV1_SHIFT),
        .CONV2_SHIFT  (CONV2_SHIFT),
        .CONV3_SHIFT  (CONV3_SHIFT),
        .CONV4_SHIFT  (CONV4_SHIFT),
        .CONV5_SHIFT  (CONV5_SHIFT),
        .DET_SHIFT    (DET_SHIFT)
    ) u_compute_engine (
        .clk(clk),
        .rst_n(rst_n),
        .soft_reset(soft_reset),
        .start(start),
        .test_pattern_enable(test_pattern_enable),
        .frame_rd_addr(frame_rd_addr),
        .frame_rd_data(frame_rd_data),
        .raw_wr_en(raw_wr_en),
        .raw_wr_addr(raw_wr_addr),
        .raw_wr_data(raw_wr_data),
        .busy(busy),
        .done_pulse(done_pulse),
        .current_step(current_step),
        .cycles_left(cycles_left),
        .dbg_last_frame_byte(dbg_last_frame_byte),
        .w_load_en(w_load_en),
        .w_load_addr(w_load_addr),
        .w_load_data(w_load_data),
        .b_load_en(b_load_en),
        .b_load_addr(b_load_addr),
        .b_load_data(b_load_data)
    );

    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            raw_frame_id <= 16'd0;
        end else if (start) begin
            raw_frame_id <= frame_id;
        end
    end

endmodule
