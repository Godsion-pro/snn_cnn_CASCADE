`timescale 1ns / 1ps
`include "dronet_params.vh"

// Dual-clock pingpong frame buffer:
//   - Write side: AXI Stream clock domain (wr_clk / wr_rst_n)
//   - Read side : AXI Lite (core) clock domain (rd_clk / rd_rst_n)
//   - True dual-port BRAM is inferred (independent wr_clk / rd_clk)
//   - CDC included for soft_reset (rd->wr) and frame status signals (wr->rd)
module dronet_pingpong_frame_buffer #(
    parameter FRAME_PIXELS = `DRONET_FRAME_PIXELS,
    parameter ADDR_W = `DRONET_FRAME_ADDR_W
) (
    // ---- Write side (AXI Stream domain) ----
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire [7:0]            s_pixel_data,
    input  wire                  s_pixel_valid,
    input  wire                  s_pixel_sof,
    output wire                  s_pixel_ready,

    // ---- Read side (AXI Lite / core domain) ----
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  soft_reset,            // pulse on rd_clk, sync'd to wr_clk inside
    output wire                  frame_done_pulse,      // pulse on rd_clk
    output wire                  completed_buf_sel,     // level on rd_clk
    output wire [15:0]           completed_frame_id,    // level on rd_clk
    output wire                  last_complete_valid,   // level on rd_clk
    input  wire                  cnn_buf_sel,
    input  wire [ADDR_W-1:0]     cnn_rd_addr,
    output wire [7:0]            cnn_rd_data
);

    // True dual-port BRAM: written on wr_clk, read on rd_clk
    (* ram_style = "block" *) reg [7:0] fb0 [0:FRAME_PIXELS-1];
    (* ram_style = "block" *) reg [7:0] fb1 [0:FRAME_PIXELS-1];

    // ============================================================
    // CDC #1: soft_reset (rd_clk -> wr_clk) via toggle-pulse sync
    // ============================================================
    // Toggle a flip-flop on rd_clk for every soft_reset pulse, then
    // edge-detect that toggle in wr_clk domain to regenerate a 1-cycle
    // pulse. This is robust regardless of the clock frequency relation.
    reg soft_reset_toggle_r;  // rd_clk domain
    always @(posedge rd_clk) begin
        if (!rd_rst_n)        soft_reset_toggle_r <= 1'b0;
        else if (soft_reset)  soft_reset_toggle_r <= ~soft_reset_toggle_r;
    end

    (* ASYNC_REG = "TRUE" *) reg [2:0] soft_reset_sync_w;
    always @(posedge wr_clk) begin
        if (!wr_rst_n) soft_reset_sync_w <= 3'b0;
        else           soft_reset_sync_w <= {soft_reset_sync_w[1:0], soft_reset_toggle_r};
    end
    // Edge between FF[1] and FF[2] -> 1-cycle pulse in wr_clk domain
    wire soft_reset_w = soft_reset_sync_w[2] ^ soft_reset_sync_w[1];

    // ============================================================
    // Write side state (wr_clk domain) -- same logic as original
    // ============================================================
    reg                   wr_buf_sel;
    reg [ADDR_W-1:0]      wr_addr;
    reg [15:0]            pixel_count;
    reg [15:0]            frame_counter;

    reg                   frame_done_pulse_w;
    reg                   completed_buf_sel_w;
    reg [15:0]            completed_frame_id_w;
    reg                   last_complete_valid_w;

    wire                  write_fire;
    wire [ADDR_W-1:0]     target_addr;
    wire [15:0]           next_pixel_count;
    wire                  frame_complete_now;

    // s_pixel_ready is always asserted (CNN inference completes within frame period)
    assign s_pixel_ready      = 1'b1;
    assign write_fire         = s_pixel_valid & s_pixel_ready;
    assign target_addr        = s_pixel_sof ? {ADDR_W{1'b0}} : wr_addr;
    assign next_pixel_count   = s_pixel_sof ? 16'd1 : (pixel_count + 16'd1);
    assign frame_complete_now = write_fire && (next_pixel_count == FRAME_PIXELS);

    integer i;
    initial begin
        for (i = 0; i < FRAME_PIXELS; i = i + 1) begin
            fb0[i] = 8'd0;
            fb1[i] = 8'd0;
        end
    end

    always @(posedge wr_clk) begin
        if (!wr_rst_n || soft_reset_w) begin
            wr_buf_sel             <= 1'b0;
            wr_addr                <= {ADDR_W{1'b0}};
            pixel_count            <= 16'd0;
            frame_counter          <= 16'd0;
            frame_done_pulse_w     <= 1'b0;
            completed_buf_sel_w    <= 1'b0;
            completed_frame_id_w   <= 16'd0;
            last_complete_valid_w  <= 1'b0;
        end else begin
            frame_done_pulse_w <= 1'b0;

            if (write_fire) begin
                if (wr_buf_sel) fb1[target_addr] <= s_pixel_data;
                else            fb0[target_addr] <= s_pixel_data;

                if (frame_complete_now) begin
                    completed_buf_sel_w   <= wr_buf_sel;
                    completed_frame_id_w  <= frame_counter;
                    last_complete_valid_w <= 1'b1;
                    frame_done_pulse_w    <= 1'b1;
                    frame_counter         <= frame_counter + 16'd1;
                    wr_buf_sel            <= ~wr_buf_sel;
                    wr_addr               <= {ADDR_W{1'b0}};
                    pixel_count           <= 16'd0;
                end else begin
                    wr_addr     <= target_addr + {{(ADDR_W-1){1'b0}}, 1'b1};
                    pixel_count <= next_pixel_count;
                end
            end
        end
    end

    // ============================================================
    // BRAM read port (rd_clk domain) -- synchronous read for BRAM inference
    //
    // FIX (timing): previously the fb0/fb1 select-mux fed a single
    // registered output, which prevented true-dual-port BRAM inference
    // (fb0/fb1 fell back to LUTRAM → long async-read + MUXF7/F8 chain
    // across the 150MHz→50MHz CDC boundary, causing -0.4ns setup viol).
    //
    // Now each fb is read into its own BRAM output register (fb0_q/fb1_q),
    // and the 2:1 select is a tiny combinational mux AFTER the registers.
    // Read latency stays at 1 cycle, so dronet_compute_engine is unchanged,
    // but the CDC path is now just BRAM clk->Q + a 2:1 mux (short).
    // ============================================================
    reg [7:0] fb0_q, fb1_q;
    always @(posedge rd_clk) begin
        fb0_q <= fb0[cnn_rd_addr];
        fb1_q <= fb1[cnn_rd_addr];
    end
    assign cnn_rd_data = cnn_buf_sel ? fb1_q : fb0_q;

    // ============================================================
    // CDC #2: frame_done_pulse (wr_clk -> rd_clk) via toggle-pulse sync
    // ============================================================
    reg frame_done_toggle_w;
    always @(posedge wr_clk) begin
        if (!wr_rst_n)               frame_done_toggle_w <= 1'b0;
        else if (frame_done_pulse_w) frame_done_toggle_w <= ~frame_done_toggle_w;
    end

    (* ASYNC_REG = "TRUE" *) reg [2:0] frame_done_sync_r;
    always @(posedge rd_clk) begin
        if (!rd_rst_n) frame_done_sync_r <= 3'b0;
        else           frame_done_sync_r <= {frame_done_sync_r[1:0], frame_done_toggle_w};
    end
    wire frame_done_pulse_r = frame_done_sync_r[2] ^ frame_done_sync_r[1];

    // ============================================================
    // CDC #3: metadata (completed_buf_sel, completed_frame_id) wr -> rd
    // ------------------------------------------------------------
    // These signals only change once per frame (~33 ms) and remain stable
    // for the entire frame period, far longer than 2-FF synchronizer
    // settling time. By the time frame_done_pulse_r fires (3-FF latency),
    // the data has already settled (2-FF latency).
    // ============================================================
    (* ASYNC_REG = "TRUE" *) reg        cbs_meta1, cbs_meta2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] cfid_meta1, cfid_meta2;
    always @(posedge rd_clk) begin
        cbs_meta1  <= completed_buf_sel_w;
        cbs_meta2  <= cbs_meta1;
        cfid_meta1 <= completed_frame_id_w;
        cfid_meta2 <= cfid_meta1;
    end

    // ============================================================
    // CDC #4: last_complete_valid (level signal) wr -> rd via 2-FF sync
    // ============================================================
    (* ASYNC_REG = "TRUE" *) reg lcv_meta1, lcv_meta2;
    always @(posedge rd_clk) begin
        if (!rd_rst_n) {lcv_meta2, lcv_meta1} <= 2'b0;
        else           {lcv_meta2, lcv_meta1} <= {lcv_meta1, last_complete_valid_w};
    end

    assign frame_done_pulse    = frame_done_pulse_r;
    assign completed_buf_sel   = cbs_meta2;
    assign completed_frame_id  = cfid_meta2;
    assign last_complete_valid = lcv_meta2;

endmodule
