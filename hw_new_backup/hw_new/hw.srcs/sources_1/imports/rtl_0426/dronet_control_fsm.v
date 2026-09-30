`timescale 1ns / 1ps

module dronet_control_fsm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        soft_reset,
    input  wire        frame_done_pulse,
    input  wire        frame_done_buf_sel,
    input  wire [15:0] frame_done_frame_id,
    input  wire        frame_valid,
    input  wire        auto_enable,
    input  wire        force_infer,
    input  wire        sw_start_pulse,
    input  wire        flying_obj_flag,
    input  wire        ps_track_set_pulse,
    input  wire        ps_track_clear_pulse,
    input  wire        core_busy,
    output reg         core_start_pulse,
    output reg         core_buf_sel,
    output reg [15:0]  core_frame_id,
    output reg         tracking_active,
    output reg         pending_valid,
    output reg         last_complete_valid,
    output reg         last_complete_buf_sel,
    output reg [15:0]  last_complete_frame_id
);

    reg        pending_buf_sel;
    reg [15:0] pending_frame_id;
    reg        pending_request;
    wire       auto_request;

    assign auto_request = auto_enable && (flying_obj_flag || tracking_active);

    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            core_start_pulse       <= 1'b0;
            core_buf_sel           <= 1'b0;
            core_frame_id          <= 16'd0;
            tracking_active        <= 1'b0;
            pending_valid          <= 1'b0;
            pending_buf_sel        <= 1'b0;
            pending_frame_id       <= 16'd0;
            pending_request        <= 1'b0;
            last_complete_valid    <= 1'b0;
            last_complete_buf_sel  <= 1'b0;
            last_complete_frame_id <= 16'd0;
        end else begin
            core_start_pulse <= 1'b0;

            // FIX: Explicit priority — clear always wins over set
            // Prevents ambiguity if both pulses fire simultaneously
            // (e.g., PS writes 0x3 to ADDR_TRACK)
            if (ps_track_set_pulse && !ps_track_clear_pulse) begin
                tracking_active <= 1'b1;
            end
            if (ps_track_clear_pulse) begin
                tracking_active <= 1'b0;
            end

            if (frame_done_pulse) begin
                last_complete_valid    <= frame_valid;
                last_complete_buf_sel  <= frame_done_buf_sel;
                last_complete_frame_id <= frame_done_frame_id;
                pending_valid          <= frame_valid;
                pending_buf_sel        <= frame_done_buf_sel;
                pending_frame_id       <= frame_done_frame_id;
                pending_request        <= force_infer || auto_request;
            end

            if (sw_start_pulse && !core_busy && last_complete_valid) begin
                core_start_pulse <= 1'b1;
                core_buf_sel     <= last_complete_buf_sel;
                core_frame_id    <= last_complete_frame_id;
            end else if (!core_busy && pending_valid && pending_request) begin
                core_start_pulse <= 1'b1;
                core_buf_sel     <= pending_buf_sel;
                core_frame_id    <= pending_frame_id;
                pending_valid    <= 1'b0;
            end
        end
    end

endmodule
