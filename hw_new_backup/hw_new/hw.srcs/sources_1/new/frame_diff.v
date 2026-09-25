`timescale 1ns / 1ps
//
// ============================================================
// axis_frame_diff
//   Compares two AXI4-Stream grayscale frames after SOF alignment.
//   Non-SOF pixels are discarded until both inputs present tuser=1
//   together.  This prevents a phase-shifted previous-frame stream
//   from being interpreted as full-screen motion.
// ============================================================
module axis_frame_diff #(
    parameter THRESHOLD = 30
)(
    input  wire        aclk,
    input  wire        aresetn,

    // AXI-Stream Slave: current frame pixel
    input  wire [7:0]  s_axis_curr_tdata,
    input  wire        s_axis_curr_tvalid,
    output wire        s_axis_curr_tready,
    input  wire        s_axis_curr_tlast,
    input  wire        s_axis_curr_tuser,

    // AXI-Stream Slave: previous frame pixel
    input  wire [7:0]  s_axis_prev_tdata,
    input  wire        s_axis_prev_tvalid,
    output wire        s_axis_prev_tready,
    input  wire        s_axis_prev_tlast,
    input  wire        s_axis_prev_tuser,

    // AXI-Stream Master: diff result
    output wire [7:0]  m_axis_diff_tdata,
    output wire        m_axis_diff_tvalid,
    input  wire        m_axis_diff_tready,
    output wire        m_axis_diff_tlast,
    output wire        m_axis_diff_tuser,

    // Debug: high when alignment is lost while running
    output wire        sync_error
);

    localparam ST_WAIT_SYNC = 1'b0;
    localparam ST_RUN       = 1'b1;

    reg state;

    wire both_valid     = s_axis_curr_tvalid & s_axis_prev_tvalid;
    wire curr_sof       = s_axis_curr_tvalid & s_axis_curr_tuser;
    wire prev_sof       = s_axis_prev_tvalid & s_axis_prev_tuser;
    wire both_sof       = curr_sof & prev_sof;
    wire sideband_match = (s_axis_curr_tuser == s_axis_prev_tuser) &&
                          (s_axis_curr_tlast == s_axis_prev_tlast);

    wire run_sync_lost = (state == ST_RUN) & both_valid & ~sideband_match;
    wire wait_sync     = (state == ST_WAIT_SYNC) | run_sync_lost;
    wire run_pair      = (state == ST_RUN) & both_valid & sideband_match;
    wire start_pair    = (state == ST_WAIT_SYNC) & both_sof;
    wire emit_pair     = start_pair | run_pair;

    // In WAIT_SYNC, discard non-SOF pixels. If one input reaches SOF
    // first, hold it until the other input also reaches SOF.
    wire curr_ready_wait = curr_sof ? (prev_sof & m_axis_diff_tready) : 1'b1;
    wire prev_ready_wait = prev_sof ? (curr_sof & m_axis_diff_tready) : 1'b1;

    assign s_axis_curr_tready = wait_sync ? curr_ready_wait :
                                (s_axis_prev_tvalid & m_axis_diff_tready);
    assign s_axis_prev_tready = wait_sync ? prev_ready_wait :
                                (s_axis_curr_tvalid & m_axis_diff_tready);

    assign sync_error = run_sync_lost;

    wire [7:0] abs_diff = (s_axis_curr_tdata >= s_axis_prev_tdata) ?
                          (s_axis_curr_tdata - s_axis_prev_tdata) :
                          (s_axis_prev_tdata - s_axis_curr_tdata);

    wire       diff_fire        = emit_pair & m_axis_diff_tready;
    wire [7:0] diff_tdata_comb  = (emit_pair && abs_diff >= THRESHOLD) ? 8'd1 : 8'd0;
    wire       diff_tvalid_comb = emit_pair;
    wire       diff_tlast_comb  = s_axis_curr_tlast;
    wire       diff_tuser_comb  = s_axis_curr_tuser;

    reg [7:0] m_axis_diff_tdata_r;
    reg       m_axis_diff_tvalid_r;
    reg       m_axis_diff_tlast_r;
    reg       m_axis_diff_tuser_r;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state                <= ST_WAIT_SYNC;
            m_axis_diff_tdata_r  <= 8'd0;
            m_axis_diff_tvalid_r <= 1'b0;
            m_axis_diff_tlast_r  <= 1'b0;
            m_axis_diff_tuser_r  <= 1'b0;
        end else if (m_axis_diff_tready) begin
            if (run_sync_lost) begin
                state <= ST_WAIT_SYNC;
            end else if (diff_fire) begin
                state <= ST_RUN;
            end

            m_axis_diff_tdata_r  <= diff_tdata_comb;
            m_axis_diff_tvalid_r <= diff_tvalid_comb;
            m_axis_diff_tlast_r  <= diff_tlast_comb;
            m_axis_diff_tuser_r  <= diff_tuser_comb;
        end
    end

    assign m_axis_diff_tdata  = m_axis_diff_tdata_r;
    assign m_axis_diff_tvalid = m_axis_diff_tvalid_r;
    assign m_axis_diff_tlast  = m_axis_diff_tlast_r;
    assign m_axis_diff_tuser  = m_axis_diff_tuser_r;

endmodule
