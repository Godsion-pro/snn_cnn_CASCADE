`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_accel_axi #(
    parameter C_S_AXI_ADDR_WIDTH = 12,
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter integer CONV1_SHIFT = 8,
    parameter integer CONV2_SHIFT = 8,
    parameter integer CONV3_SHIFT = 8,
    parameter integer CONV4_SHIFT = 8,
    parameter integer CONV5_SHIFT = 8,
    parameter integer DET_SHIFT   = 8
) (
    // ---- AXI Lite clock / reset (also used by control_fsm, cnn_core, output_buffer) ----
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,

    // ---- CNN core clock-gating enable (active-high) ----
    //  cnn_clk_en = 1 -> CNN core clocked ; 0 -> gated OFF.
    //  Driven by the snn_cnn_sel node (cnn_clk_en = ~snn_en) so SNN and CNN
    //  are mutually exclusive. Only the clock into u_cnn_core is gated; the
    //  AXI-Lite register file, control_fsm, frame buffer and output buffer
    //  stay on s_axi_aclk so the PS<->PL bus never freezes.
    input  wire                              cnn_clk_en,

    // ---- AXI Stream clock / reset (pixel input domain) ----
    input  wire                              s_axis_aclk,
    input  wire                              s_axis_aresetn,

    // ---- AXI Stream slave (pixel input, on s_axis_aclk) ----
    input  wire [7:0]                        s_pixel_tdata,
    input  wire                              s_pixel_tvalid,
    input  wire                              s_pixel_tuser,
    output wire                              s_pixel_tready,

    // ---- Misc (treated as on s_axi_aclk; if truly async, add CDC at top) ----
    input  wire                              flying_obj_flag,
    output wire                              irq_frame_done,
    output wire                              irq_infer_done,

    // ---- AXI Lite slave (on s_axi_aclk) ----
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire                              s_axi_awvalid,
    output wire                              s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output wire                              s_axi_wready,
    output reg  [1:0]                        s_axi_bresp,
    output reg                               s_axi_bvalid,
    input  wire                              s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire                              s_axi_arvalid,
    output wire                              s_axi_arready,
    output reg  [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output reg  [1:0]                        s_axi_rresp,
    output reg                               s_axi_rvalid,
    input  wire                              s_axi_rready
);

    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_CONTROL = 12'h000;
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_TRACK   = 12'h004;
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_STATUS0 = 12'h008;
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_STATUS1 = 12'h00C;
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_DEBUG0  = 12'h010;

    // Minimal debug register
    //   DEBUG1: raw write sticky flags + last sampled bytes
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_DEBUG1  = 12'h014;

    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_RAW_BASE = 12'h100;
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_RAW_LAST = 12'h624;

    // PS weight/bias load registers
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_W_LOAD   = 12'h800; // [21:8]=addr, [7:0]=data → pulse w_load
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_B_ADDR   = 12'h804; // [6:0]=bias addr (latched)
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_B_DATA   = 12'h808; // [31:0]=bias data → pulse b_load

    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_latched;
    reg [C_S_AXI_ADDR_WIDTH-1:0] araddr_latched;
    reg [C_S_AXI_DATA_WIDTH-1:0] wdata_latched;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb_latched;
    reg aw_pending;
    reg w_pending;

    reg auto_enable_reg;
    reg force_infer_reg;
    reg test_pattern_enable_reg;
    reg sw_start_pulse;
    reg soft_reset_pulse;
    reg ps_track_set_pulse;
    reg ps_track_clear_pulse;
    reg done_latched;
    reg frame_latched;
    reg raw_valid_reg;

    wire frame_done_pulse;
    wire frame_buf_sel;
    wire [15:0] frame_buf_frame_id;
    wire frame_valid;

    wire control_core_start;
    wire control_core_buf_sel;
    wire [15:0] control_core_frame_id;
    wire tracking_active;
    wire pending_valid;
    wire last_complete_valid;
    wire last_complete_buf_sel;
    wire [15:0] last_complete_frame_id;

    wire core_busy;
    wire core_done_pulse;
    wire [15:0] raw_frame_id;
    wire [3:0] current_step;
    wire [23:0] current_cycles_left;
    wire [7:0] dbg_last_frame_byte;

    wire raw_wr_en;
    wire [`DRONET_RAW_ADDR_W-1:0] raw_wr_addr;
    wire signed [7:0] raw_wr_data;

    wire [`DRONET_FRAME_ADDR_W-1:0] frame_rd_addr;
    wire [7:0] frame_rd_data;

    wire [`DRONET_RAW_WORD_ADDR_W-1:0] raw_rd_word_addr;
    wire [31:0] raw_rd_word_data;
    wire [`DRONET_RAW_WORD_ADDR_W-1:0] raw_rd_word_addr_now;

    wire write_commit;

    // Weight/bias load signals
    reg        w_load_pulse;
    reg [13:0] w_load_addr_reg;
    reg signed [7:0] w_load_data_reg;
    reg        b_load_pulse;
    reg [6:0]  b_load_addr_reg;
    reg signed [31:0] b_load_data_reg;
    
    // ============================================================
    // Minimal debug sticky flags in s_axi_aclk domain
    //   dbg_raw_seen    : raw_wr_en occurred at least once after core start
    //   dbg_raw_nz_seen : non-zero raw_wr_data occurred at least once
    //   dbg_raw_last    : last raw write data byte
    //   dbg_raw_last_nz : last non-zero raw write data byte
    //
    // Keep this intentionally tiny. Do not use counters/sums/mark_debug here;
    // those can prevent optimization and explode LUT/F7 mux usage.
    // ============================================================
    reg       dbg_raw_seen;
    reg       dbg_raw_nz_seen;
    reg [7:0] dbg_raw_last;
    reg [7:0] dbg_raw_last_nz;

    // ============================================================
    // CDC: flying_obj_flag (s_axis_aclk -> s_axi_aclk)
    // It is a PULSE (not a long-held level) generated in s_axis_aclk domain.
    // A 2-FF level synchronizer would miss/lose narrow pulses when the
    // source clock is faster than (or close to) the destination clock.
    // We use a toggle-based pulse synchronizer:
    //   1) source side toggles a flop on each input pulse  (holds forever)
    //   2) destination side 2-FF synchronizes the toggle, then edge-detects
    //      to regenerate a 1-cycle pulse in s_axi_aclk domain
    // Caveat: input pulses must be separated by >= ~3 s_axi_aclk cycles
    //         so consecutive toggles are not merged at the destination.
    // ============================================================
    // Source-side (s_axis_aclk): rising-edge detect + toggle generator
    // Rising-edge detect ensures the toggle is robust regardless of input
    // pulse width (1 cycle, multi-cycle, even held-high inputs all become
    // exactly one toggle event per fly detection).
    reg flying_obj_flag_d_axis;
    reg flying_obj_toggle_axis;
    always @(posedge s_axis_aclk or negedge s_axis_aresetn) begin
        if (!s_axis_aresetn) begin
            flying_obj_flag_d_axis <= 1'b0;
            flying_obj_toggle_axis <= 1'b0;
        end else begin
            flying_obj_flag_d_axis <= flying_obj_flag;
            // Toggle only on 0->1 transition
            if (flying_obj_flag && !flying_obj_flag_d_axis) begin
                flying_obj_toggle_axis <= ~flying_obj_toggle_axis;
            end
        end
    end

    // Destination-side (s_axi_aclk) 2-FF synchronizer + edge detect
    (* ASYNC_REG = "TRUE" *) reg [2:0] flying_obj_sync;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) flying_obj_sync <= 3'b0;
        else                flying_obj_sync <= {flying_obj_sync[1:0], flying_obj_toggle_axis};
    end
    // 1-cycle pulse on s_axi_aclk for every input pulse on s_axis_aclk
    wire flying_obj_pulse_sync = flying_obj_sync[2] ^ flying_obj_sync[1];

    // ------------------------------------------------------------
    // Sticky flag: control_fsm samples flying_obj_flag as a LEVEL on the
    // frame_done_pulse cycle. Since the synchronized pulse is only 1 cycle,
    // it would almost never coincide with frame_done_pulse. We latch the
    // pulse into a sticky flag and clear it when frame_done_pulse is taken.
    // Semantics: "If at least one object-detect pulse has occurred since
    //             the last frame_done, request inference for this frame."
    // ------------------------------------------------------------
    reg flying_obj_sticky;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || soft_reset_pulse) begin
            flying_obj_sticky <= 1'b0;
        end else begin
            if (flying_obj_pulse_sync) begin
                flying_obj_sticky <= 1'b1;            // set on incoming pulse
            end
            if (frame_done_pulse) begin
                // clear at frame boundary, but if a pulse arrives in the
                // same cycle, keep it set (pulse wins over clear).
                flying_obj_sticky <= flying_obj_pulse_sync;
            end
        end
    end
    wire flying_obj_flag_sync = flying_obj_sticky;

    assign s_axi_awready = !aw_pending;
    assign s_axi_wready  = !w_pending;
    assign s_axi_arready = !s_axi_rvalid;
    assign write_commit  = aw_pending && w_pending && !s_axi_bvalid;

    assign irq_frame_done = frame_done_pulse;
    assign irq_infer_done = core_done_pulse;

    // ============================================================
    // Frame buffer: spans both clock domains (CDC handled inside)
    //   - Write side : s_axis_aclk / s_axis_aresetn
    //   - Read side  : s_axi_aclk  / s_axi_aresetn
    // ============================================================
    dronet_pingpong_frame_buffer u_pingpong_frame_buffer (
        .wr_clk              (s_axis_aclk),
        .wr_rst_n            (s_axis_aresetn),
        .s_pixel_data        (s_pixel_tdata),
        .s_pixel_valid       (s_pixel_tvalid),
        .s_pixel_sof         (s_pixel_tuser),
        .s_pixel_ready       (s_pixel_tready),

        .rd_clk              (s_axi_aclk),
        .rd_rst_n            (s_axi_aresetn),
        .soft_reset          (soft_reset_pulse),
        .frame_done_pulse    (frame_done_pulse),
        .completed_buf_sel   (frame_buf_sel),
        .completed_frame_id  (frame_buf_frame_id),
        .last_complete_valid (frame_valid),
        .cnn_buf_sel         (control_core_buf_sel),
        .cnn_rd_addr         (frame_rd_addr),
        .cnn_rd_data         (frame_rd_data)
    );

    // All downstream blocks remain in the AXI Lite (s_axi_aclk) domain
    dronet_control_fsm u_control_fsm (
        .clk(s_axi_aclk),
        .rst_n(s_axi_aresetn),
        .soft_reset(soft_reset_pulse),
        .frame_done_pulse(frame_done_pulse),
        .frame_done_buf_sel(frame_buf_sel),
        .frame_done_frame_id(frame_buf_frame_id),
        .frame_valid(frame_valid),
        .auto_enable(auto_enable_reg),
        .force_infer(force_infer_reg),
        .sw_start_pulse(sw_start_pulse),
        .flying_obj_flag(flying_obj_flag_sync),
        .ps_track_set_pulse(ps_track_set_pulse),
        .ps_track_clear_pulse(ps_track_clear_pulse),
        .core_busy(core_busy),
        .core_start_pulse(control_core_start),
        .core_buf_sel(control_core_buf_sel),
        .core_frame_id(control_core_frame_id),
        .tracking_active(tracking_active),
        .pending_valid(pending_valid),
        .last_complete_valid(last_complete_valid),
        .last_complete_buf_sel(last_complete_buf_sel),
        .last_complete_frame_id(last_complete_frame_id)
    );

    // ============================================================
    // Gated clock for the CNN compute core only.
    //   - clk_in : free-running s_axi_aclk
    //   - en     : cnn_clk_en (active-high; = ~snn_en, from snn_cnn_sel)
    //   - idle   : ~core_busy  -> never stop a running inference
    // Everything else in this module remains on the ungated s_axi_aclk.
    // ============================================================
    wire cnn_core_clk;
    clk_en_gate u_cnn_clk_gate (
        .clk_in (s_axi_aclk),
        .en     (cnn_clk_en),
        .idle   (~core_busy),
        .clk_out(cnn_core_clk)
    );

    dronet_cnn_core #(
        .CONV1_SHIFT  (CONV1_SHIFT),
        .CONV2_SHIFT  (CONV2_SHIFT),
        .CONV3_SHIFT  (CONV3_SHIFT),
        .CONV4_SHIFT  (CONV4_SHIFT),
        .CONV5_SHIFT  (CONV5_SHIFT),
        .DET_SHIFT    (DET_SHIFT)
    ) u_cnn_core (
        .clk(cnn_core_clk),
        .rst_n(s_axi_aresetn),
        .soft_reset(soft_reset_pulse),
        .start(control_core_start),
        .test_pattern_enable(test_pattern_enable_reg),
        .frame_id(control_core_frame_id),
        .frame_rd_addr(frame_rd_addr),
        .frame_rd_data(frame_rd_data),
        .raw_wr_en(raw_wr_en),
        .raw_wr_addr(raw_wr_addr),
        .raw_wr_data(raw_wr_data),
        .busy(core_busy),
        .done_pulse(core_done_pulse),
        .raw_frame_id(raw_frame_id),
        .current_step(current_step),
        .cycles_left(current_cycles_left),
        .dbg_last_frame_byte(dbg_last_frame_byte),
        .w_load_en(w_load_pulse),
        .w_load_addr(w_load_addr_reg),
        .w_load_data(w_load_data_reg),
        .b_load_en(b_load_pulse),
        .b_load_addr(b_load_addr_reg),
        .b_load_data(b_load_data_reg)
    );

    dronet_output_buffer u_output_buffer (
        .clk(s_axi_aclk),
        .wr_en(raw_wr_en),
        .wr_addr(raw_wr_addr),
        .wr_data(raw_wr_data),
        .rd_word_addr(raw_rd_word_addr),
        .rd_word_data(raw_rd_word_data)
    );
    
    // ============================================================
    // Minimal debug sticky logic
    // ============================================================
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn || soft_reset_pulse || control_core_start) begin
            dbg_raw_seen    <= 1'b0;
            dbg_raw_nz_seen <= 1'b0;
            dbg_raw_last    <= 8'd0;
            dbg_raw_last_nz <= 8'd0;
        end else begin
            if (raw_wr_en) begin
                dbg_raw_seen <= 1'b1;
                dbg_raw_last <= raw_wr_data[7:0];

                if (raw_wr_data != 8'sd0) begin
                    dbg_raw_nz_seen <= 1'b1;
                    dbg_raw_last_nz <= raw_wr_data[7:0];
                end
            end
        end
    end

    // FIX: Guard against unsigned underflow when araddr < ADDR_RAW_BASE
    assign raw_rd_word_addr_now = (s_axi_araddr >= ADDR_RAW_BASE)
                                 ? (s_axi_araddr - ADDR_RAW_BASE) >> 2
                                 : {`DRONET_RAW_WORD_ADDR_W{1'b0}};
    assign raw_rd_word_addr = raw_rd_word_addr_now;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_pending              <= 1'b0;
            w_pending               <= 1'b0;
            awaddr_latched          <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            wdata_latched           <= {C_S_AXI_DATA_WIDTH{1'b0}};
            wstrb_latched           <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};
            s_axi_bvalid            <= 1'b0;
            s_axi_bresp             <= 2'b00;
            auto_enable_reg         <= 1'b1;
            force_infer_reg         <= 1'b0;
            test_pattern_enable_reg <= 1'b0;
            sw_start_pulse          <= 1'b0;
            soft_reset_pulse        <= 1'b0;
            ps_track_set_pulse      <= 1'b0;
            ps_track_clear_pulse    <= 1'b0;
            done_latched            <= 1'b0;
            frame_latched           <= 1'b0;
            raw_valid_reg           <= 1'b0;
            w_load_pulse            <= 1'b0;
            w_load_addr_reg         <= 14'd0;
            w_load_data_reg         <= 8'sd0;
            b_load_pulse            <= 1'b0;
            b_load_addr_reg         <= 7'd0;
            b_load_data_reg         <= 32'sd0;
        end else begin
            sw_start_pulse       <= 1'b0;
            soft_reset_pulse     <= 1'b0;
            ps_track_set_pulse   <= 1'b0;
            ps_track_clear_pulse <= 1'b0;
            w_load_pulse         <= 1'b0;
            b_load_pulse         <= 1'b0;

            if (!aw_pending && s_axi_awvalid) begin
                aw_pending     <= 1'b1;
                awaddr_latched <= s_axi_awaddr;
            end

            if (!w_pending && s_axi_wvalid) begin
                w_pending     <= 1'b1;
                wdata_latched <= s_axi_wdata;
                wstrb_latched <= s_axi_wstrb;
            end

            if (write_commit) begin
                case (awaddr_latched)
                    ADDR_CONTROL: begin
                        if (wdata_latched[0]) begin
                            sw_start_pulse <= 1'b1;
                        end
                        if (wdata_latched[1]) begin
                            soft_reset_pulse <= 1'b1;
                        end
                        auto_enable_reg         <= wdata_latched[2];
                        force_infer_reg         <= wdata_latched[3];
                        test_pattern_enable_reg <= wdata_latched[4];
                        if (wdata_latched[8]) begin
                            done_latched <= 1'b0;
                        end
                        if (wdata_latched[9]) begin
                            frame_latched <= 1'b0;
                        end
                    end

                    ADDR_TRACK: begin
                        if (wdata_latched[0]) begin
                            ps_track_set_pulse <= 1'b1;
                        end
                        if (wdata_latched[1]) begin
                            ps_track_clear_pulse <= 1'b1;
                        end
                    end

                    ADDR_W_LOAD: begin
                        w_load_addr_reg <= wdata_latched[21:8];
                        w_load_data_reg <= wdata_latched[7:0];
                        w_load_pulse    <= 1'b1;
                    end

                    ADDR_B_ADDR: begin
                        b_load_addr_reg <= wdata_latched[6:0];
                    end

                    ADDR_B_DATA: begin
                        b_load_data_reg <= wdata_latched;
                        b_load_pulse    <= 1'b1;
                    end

                    default: begin
                    end
                endcase

                aw_pending   <= 1'b0;
                w_pending    <= 1'b0;
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (frame_done_pulse) begin
                frame_latched <= 1'b1;
            end

            if (core_done_pulse) begin
                done_latched  <= 1'b1;
                raw_valid_reg <= 1'b1;
            end

            if (soft_reset_pulse) begin
                done_latched  <= 1'b0;
                frame_latched <= 1'b0;
                raw_valid_reg <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            araddr_latched <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            s_axi_rvalid   <= 1'b0;
            s_axi_rdata    <= {C_S_AXI_DATA_WIDTH{1'b0}};
            s_axi_rresp    <= 2'b00;
        end else begin
            if (!s_axi_rvalid && s_axi_arvalid) begin
                araddr_latched <= s_axi_araddr;
                s_axi_rvalid   <= 1'b1;
                s_axi_rresp    <= 2'b00;

                case (s_axi_araddr)
                    ADDR_CONTROL: begin
                        s_axi_rdata <= {
                            27'd0,
                            test_pattern_enable_reg,
                            force_infer_reg,
                            auto_enable_reg,
                            2'd0
                        };
                    end

                    ADDR_TRACK: begin
                        s_axi_rdata <= {
                            31'd0,
                            tracking_active
                        };
                    end

                    ADDR_STATUS0: begin
                        s_axi_rdata <= {
                            24'd0,
                            raw_valid_reg,
                            control_core_buf_sel,
                            last_complete_buf_sel,
                            pending_valid,
                            tracking_active,
                            frame_latched,
                            done_latched,
                            core_busy
                        };
                    end

                    ADDR_STATUS1: begin
                        s_axi_rdata <= {
                            raw_frame_id,
                            last_complete_frame_id
                        };
                    end

                    ADDR_DEBUG0: begin
                        s_axi_rdata <= {
                            4'd0,
                            current_step,
                            current_cycles_left
                        };
                    end
                    
                    ADDR_DEBUG1: begin
                        s_axi_rdata <= {
                            4'hD,               // [31:28] signature
                            current_step,       // [27:24] CNN step
                            raw_wr_en,          // [23]    raw write enable now
                            dbg_raw_seen,       // [22]    raw write occurred
                            dbg_raw_nz_seen,    // [21]    non-zero raw write occurred
                            5'd0,               // [20:16] reserved
                            dbg_raw_last,       // [15:8]  last raw write data
                            dbg_last_frame_byte // [7:0]   last frame byte observed in core
                        };
                    end 

                    default: begin
                        if ((s_axi_araddr >= ADDR_RAW_BASE) && (s_axi_araddr <= ADDR_RAW_LAST)) begin
                            s_axi_rdata <= raw_rd_word_data;
                        end else begin
                            s_axi_rdata <= 32'd0;
                        end
                    end
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule