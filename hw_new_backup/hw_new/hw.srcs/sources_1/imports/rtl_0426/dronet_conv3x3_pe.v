`timescale 1ns / 1ps

// 3-stage pipelined 9-MAC tree (no accumulator inside).
//   Stage 1: 9 signed 8x8 products            -> m[0..8]   (16-bit)
//   Stage 2: three partial sums (3 each)       -> p0,p1,p2  (18-bit)
//   Stage 3: final sum of the 9 products       -> psum_out  (sign-extended 32-bit)
//
// Output latency = 3 clocks from {px,wt} inputs.
// The external accumulation (acc_row[x] += psum_out) is done by the
// caller (dronet_compute_engine), which also delays the write index/valid
// by 3 cycles to match this latency.
module dronet_conv3x3_pe (
    input  wire                clk,
    input  wire signed [7:0]   px00,
    input  wire signed [7:0]   px01,
    input  wire signed [7:0]   px02,
    input  wire signed [7:0]   px10,
    input  wire signed [7:0]   px11,
    input  wire signed [7:0]   px12,
    input  wire signed [7:0]   px20,
    input  wire signed [7:0]   px21,
    input  wire signed [7:0]   px22,
    input  wire signed [7:0]   wt00,
    input  wire signed [7:0]   wt01,
    input  wire signed [7:0]   wt02,
    input  wire signed [7:0]   wt10,
    input  wire signed [7:0]   wt11,
    input  wire signed [7:0]   wt12,
    input  wire signed [7:0]   wt20,
    input  wire signed [7:0]   wt21,
    input  wire signed [7:0]   wt22,
    output wire signed [31:0]  psum_out
);

    // Stage 1: products (8x8 -> 16 bits is sufficient: max |127*-128| = 16256)
    reg signed [15:0] m0, m1, m2, m3, m4, m5, m6, m7, m8;
    always @(posedge clk) begin
        m0 <= px00 * wt00;
        m1 <= px01 * wt01;
        m2 <= px02 * wt02;
        m3 <= px10 * wt10;
        m4 <= px11 * wt11;
        m5 <= px12 * wt12;
        m6 <= px20 * wt20;
        m7 <= px21 * wt21;
        m8 <= px22 * wt22;
    end

    // Stage 2: three partial sums (3 x 16-bit -> 18-bit)
    reg signed [17:0] p0, p1, p2;
    always @(posedge clk) begin
        p0 <= m0 + m1 + m2;
        p1 <= m3 + m4 + m5;
        p2 <= m6 + m7 + m8;
    end

    // Stage 3: final sum (3 x 18-bit -> 20-bit), sign-extend to 32
    reg signed [19:0] s;
    always @(posedge clk) begin
        s <= p0 + p1 + p2;
    end

    assign psum_out = {{12{s[19]}}, s};

endmodule
