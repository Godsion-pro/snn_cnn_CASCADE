`timescale 1ns / 1ps
// =====================================================================
// clk_en_gate : glitch-free clock enable (clock gating) using BUFGCE
// ---------------------------------------------------------------------
//  - en   : external enable (e.g. GPIO bit). 1 = pass clock, 0 = gate off.
//           May come from a different clock domain than clk_in; it is
//           2-FF synchronized into the clk_in domain internally.
//  - idle : 1 = the gated block is NOT busy, so it is safe to stop its
//           clock. The clock is ONLY actually stopped when (en==0 AND
//           idle==1); if the block is still busy (idle==0) the clock keeps
//           running so a running operation is never chopped mid-cycle.
//  - clk_out : gated copy of clk_in. When enabled it is the SAME clock
//              (same edges, on the global tree) as clk_in -> downstream
//              logic stays synchronous to neighbours; CDC only matters at
//              the enable/disable transition, which is guarded by `idle`.
//
//  Shared by SNN (idle = snn_idle) and CNN (idle = ~core_busy).
// =====================================================================
module clk_en_gate (
    input  wire clk_in,
    input  wire en,
    input  wire idle,
    output wire clk_out
);
    // Synchronize the (possibly async) enable into the clk_in domain.
    (* ASYNC_REG = "TRUE" *) reg [1:0] en_sync;
    always @(posedge clk_in) begin
        en_sync <= {en_sync[0], en};
    end

    // Only allow the clock to stop when the block is idle.
    wire ce = en_sync[1] | ~idle;

    // BUFGCE: dedicated global clock buffer with clock enable (UNISIM).
    BUFGCE u_bufgce (
        .I (clk_in),
        .CE(ce),
        .O (clk_out)
    );
endmodule
