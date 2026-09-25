`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_output_buffer #(
    parameter RAW_BYTES = `DRONET_RAW_BYTES,
    parameter RAW_ADDR_W = `DRONET_RAW_ADDR_W,
    parameter RAW_WORD_ADDR_W = `DRONET_RAW_WORD_ADDR_W
) (
    input  wire                        clk,
    input  wire                        wr_en,
    input  wire [RAW_ADDR_W-1:0]       wr_addr,
    input  wire signed [7:0]           wr_data,
    input  wire [RAW_WORD_ADDR_W-1:0]  rd_word_addr,
    output reg  [31:0]                 rd_word_data
);

    // 1320 bytes — small enough for distributed RAM (combinational read OK)
    reg signed [7:0] mem [0:RAW_BYTES-1];

    // FIX: Use proper sized wire instead of integer for address computation
    wire [RAW_ADDR_W-1:0] rd_base = {rd_word_addr, 2'b00};

    integer i;

    initial begin
        for (i = 0; i < RAW_BYTES; i = i + 1) begin
            mem[i] = 8'sd0;
        end
    end

    always @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    always @(*) begin
        rd_word_data = 32'd0;
        if (rd_base < RAW_BYTES) begin
            rd_word_data[7:0] = mem[rd_base];
        end
        if ((rd_base + 1) < RAW_BYTES) begin
            rd_word_data[15:8] = mem[rd_base + 1];
        end
        if ((rd_base + 2) < RAW_BYTES) begin
            rd_word_data[23:16] = mem[rd_base + 2];
        end
        if ((rd_base + 3) < RAW_BYTES) begin
            rd_word_data[31:24] = mem[rd_base + 3];
        end
    end

endmodule
