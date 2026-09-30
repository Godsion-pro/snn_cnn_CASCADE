`timescale 1ns / 1ps

module dronet_line_buffer_3x3 #(
    parameter integer MAX_WIDTH = 162,
    parameter integer ADDR_W    = 8
) (
    input  wire                     clk,
    input  wire                     load_en,
    input  wire [1:0]               load_row_sel,
    input  wire [ADDR_W-1:0]        load_addr,
    input  wire signed [7:0]        load_data,
    input  wire [ADDR_W-1:0]        window_x,
    output wire signed [7:0]        tap00,
    output wire signed [7:0]        tap01,
    output wire signed [7:0]        tap02,
    output wire signed [7:0]        tap10,
    output wire signed [7:0]        tap11,
    output wire signed [7:0]        tap12,
    output wire signed [7:0]        tap20,
    output wire signed [7:0]        tap21,
    output wire signed [7:0]        tap22
);

    reg signed [7:0] row0 [0:MAX_WIDTH-1];
    reg signed [7:0] row1 [0:MAX_WIDTH-1];
    reg signed [7:0] row2 [0:MAX_WIDTH-1];

    integer i;

    initial begin
        for (i = 0; i < MAX_WIDTH; i = i + 1) begin
            row0[i] = 8'sd0;
            row1[i] = 8'sd0;
            row2[i] = 8'sd0;
        end
    end

    always @(posedge clk) begin
        if (load_en) begin
            case (load_row_sel)
                2'd0: row0[load_addr] <= load_data;
                2'd1: row1[load_addr] <= load_data;
                2'd2: row2[load_addr] <= load_data;
                default: begin
                end
            endcase
        end
    end

    assign tap00 = row0[window_x];
    assign tap01 = row0[window_x + 1'b1];
    assign tap02 = row0[window_x + 2'd2];
    assign tap10 = row1[window_x];
    assign tap11 = row1[window_x + 1'b1];
    assign tap12 = row1[window_x + 2'd2];
    assign tap20 = row2[window_x];
    assign tap21 = row2[window_x + 1'b1];
    assign tap22 = row2[window_x + 2'd2];

endmodule
