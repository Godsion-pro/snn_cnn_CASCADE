`timescale 1ns / 1ps

module snn_top (
    // ==== Clocks ====
    input  wire        s_axis_aclk,    // 150MHz : inference
    input  wire        s_axi_aclk,     // 100MHz : weight write

    // ==== Resets ====
    input  wire        s_axis_aresetn,
    input  wire        s_axi_aresetn,

    // ==== Control (s_axis_aclk) ====
    input  wire        start,
    output reg         done,
    output wire        idle,
    output reg         result,
    output reg  [4:0]  spike_count_0,
    output reg  [4:0]  spike_count_1,

    // ==== Threshold (s_axis_aclk) ====
    input  wire signed [15:0] fc1_threshold,
    input  wire signed [15:0] fc2_threshold,

    // ==== Weight write port (s_axi_aclk) ====
    input  wire                w_wr_en,
    input  wire                w_layer,
    input  wire [10:0]         w_addr,
    input  wire signed [7:0]   w_data,

    // ==== Input write port (s_axis_aclk) ====
    input  wire                in_wr_en,
    input  wire [11:0]         in_addr,
    input  wire [2:0]          in_data,

    // ==== 슬라이딩 윈도우 (s_axis_aclk) ====
    input  wire [4:0]          read_offset
);

    // =========================================================
    // Memories
    // =========================================================
    (* ram_style = "block" *)        reg signed [7:0] fc1_w     [0:1151];
    (* ram_style = "block" *)        reg signed [7:0] fc2_w     [0:15];
    (* ram_style = "distributed" *)  reg signed [7:0] input_mem [0:2879];

    // Weight WRITE (s_axi_aclk)
    always @(posedge s_axi_aclk) begin
        if (w_wr_en) begin
            if (!w_layer)
                fc1_w[w_addr] <= w_data;
            else
                fc2_w[w_addr[3:0]] <= w_data;
        end
    end

    // Input WRITE (s_axis_aclk)
    always @(posedge s_axis_aclk) begin
        if (in_wr_en)
            input_mem[in_addr] <= {5'd0, in_data};
    end

    // =========================================================
    // FSM states
    // =========================================================
    localparam [3:0]
        S_IDLE      = 4'd0,
        S_FC1_MAC   = 4'd1,
        S_FC1_DRAIN = 4'd2,
        S_FC1_LIF0  = 4'd3,
        S_FC1_LIF1  = 4'd4,
        S_FC1_LIF2  = 4'd5,
        S_FC1_LIF3  = 4'd6,
        S_FC2_MAC   = 4'd7,
        S_FC2_DRAIN = 4'd8,
        S_FC2_LIF0  = 4'd9,
        S_FC2_LIF1  = 4'd10,
        S_FC2_LIF2  = 4'd11,
        S_FC2_LIF3  = 4'd12,
        S_DONE      = 4'd13;

    reg [3:0] state;

    // idle 출력 (슬라이딩 윈도우용)
    assign idle = (state == S_IDLE);

    // =========================================================
    // Counters / regs
    // =========================================================
    reg [4:0] step_cnt;
    reg [7:0] elem_cnt;
    reg [3:0] neuron_cnt;
    reg [1:0] drain_cnt;

    reg signed [31:0] acc;

    reg signed [31:0] mem1 [0:7];
    reg signed [31:0] mem2 [0:1];

    reg [7:0] spk1_vec;
    reg [4:0] spk_cnt0;
    reg [4:0] spk_cnt1;

    // 슬라이딩 윈도우: offset 래치
    reg [4:0] offset_latch;

    // =========================================================
    // Circular 주소 계산 (슬라이딩 윈도우)
    // =========================================================
    wire [5:0] step_sum    = {1'b0, offset_latch} + {1'b0, step_cnt};
    wire [4:0] actual_step = (step_sum >= 6'd20) ? step_sum[4:0] - 5'd20
                                                  : step_sum[4:0];

    // =========================================================
    // Address calculation
    // =========================================================
    wire [10:0] nc11 = {7'd0, neuron_cnt[2:0]};
    wire [10:0] ec11 = {3'd0, elem_cnt};
    wire [10:0] fc1_rd_addr = (nc11 << 7) + (nc11 << 4) + ec11;

    // 슬라이딩 윈도우: step_cnt 대신 actual_step 사용
    wire [11:0] as12 = {7'd0, actual_step};
    wire [11:0] ec12 = {4'd0, elem_cnt};
    wire [11:0] in_rd_addr = (as12 << 7) + (as12 << 4) + ec12;

    wire [3:0] fc2_rd_addr = {neuron_cnt[0], elem_cnt[2:0]};

    // =========================================================
    // MAC pipeline registers
    // =========================================================
    reg [10:0] fc1_addr_s1;
    reg [11:0] in_addr_s1;
    reg [3:0]  fc2_addr_s1;
    reg [2:0]  spk_idx_s1;
    reg        layer_s1;
    reg        valid_s1;

    reg signed [7:0] op_a_s2;
    reg signed [7:0] op_b_s2;
    reg              valid_s2;

    (* use_dsp = "yes" *)
    reg signed [15:0] prod_s3;
    reg               valid_s3;

    // Stage 1 / 3 / valid_s2
    always @(posedge s_axis_aclk or negedge s_axis_aresetn) begin
        if (!s_axis_aresetn) begin
            fc1_addr_s1 <= 11'd0;
            in_addr_s1  <= 12'd0;
            fc2_addr_s1 <= 4'd0;
            spk_idx_s1  <= 3'd0;
            layer_s1    <= 1'b0;
            valid_s1    <= 1'b0;

            valid_s2    <= 1'b0;

            prod_s3     <= 16'sd0;
            valid_s3    <= 1'b0;
        end else begin
            fc1_addr_s1 <= fc1_rd_addr;
            in_addr_s1  <= in_rd_addr;
            fc2_addr_s1 <= fc2_rd_addr;
            spk_idx_s1  <= elem_cnt[2:0];
            layer_s1    <= (state == S_FC2_MAC);
            valid_s1    <= (state == S_FC1_MAC) || (state == S_FC2_MAC);

            valid_s2    <= valid_s1;

            prod_s3     <= op_a_s2 * op_b_s2;
            valid_s3    <= valid_s2;
        end
    end

    // Stage 2: BRAM/DistRAM read (no reset)
    always @(posedge s_axis_aclk) begin
        if (layer_s1 == 1'b0) begin
            op_a_s2 <= fc1_w[fc1_addr_s1];
            op_b_s2 <= input_mem[in_addr_s1];
        end else begin
            op_a_s2 <= fc2_w[fc2_addr_s1];
            op_b_s2 <= {7'd0, spk1_vec[spk_idx_s1]};
        end
    end

    // =========================================================
    // LIF pipeline
    // =========================================================
    reg signed [31:0] lif_decayed_reg;
    reg signed [31:0] lif_new_reg;
    reg               lif_spike_reg;

    wire signed [31:0] fc1_threshold_ext = {{16{fc1_threshold[15]}}, fc1_threshold};
    wire signed [31:0] fc2_threshold_ext = {{16{fc2_threshold[15]}}, fc2_threshold};

    wire signed [31:0] mem1_decay_calc = (mem1[neuron_cnt[2:0]] * 32'sd7) >>> 3;
    wire signed [31:0] mem2_decay_calc = (mem2[neuron_cnt[0]]   * 32'sd7) >>> 3;

    // =========================================================
    // Main FSM (s_axis_aclk)
    // =========================================================
    integer i;

    always @(posedge s_axis_aclk or negedge s_axis_aresetn) begin
        if (!s_axis_aresetn) begin
            state         <= S_IDLE;
            done          <= 1'b0;
            result        <= 1'b0;
            spike_count_0 <= 5'd0;
            spike_count_1 <= 5'd0;
            step_cnt      <= 5'd0;
            elem_cnt      <= 8'd0;
            neuron_cnt    <= 4'd0;
            drain_cnt     <= 2'd0;
            acc           <= 32'sd0;
            spk1_vec      <= 8'd0;
            spk_cnt0      <= 5'd0;
            spk_cnt1      <= 5'd0;
            offset_latch  <= 5'd0;

            lif_decayed_reg <= 32'sd0;
            lif_new_reg     <= 32'sd0;
            lif_spike_reg   <= 1'b0;

            for (i = 0; i < 8; i = i + 1)
                mem1[i] <= 32'sd0;
            mem2[0] <= 32'sd0;
            mem2[1] <= 32'sd0;
        end else begin
            if (valid_s3)
                acc <= acc + {{16{prod_s3[15]}}, prod_s3};

            case (state)

                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        offset_latch <= read_offset;  // 슬라이딩 윈도우: offset 래치
                        step_cnt     <= 5'd0;
                        elem_cnt     <= 8'd0;
                        neuron_cnt   <= 4'd0;
                        drain_cnt    <= 2'd0;
                        acc          <= 32'sd0;
                        spk1_vec     <= 8'd0;
                        spk_cnt0     <= 5'd0;
                        spk_cnt1     <= 5'd0;

                        lif_decayed_reg <= 32'sd0;
                        lif_new_reg     <= 32'sd0;
                        lif_spike_reg   <= 1'b0;

                        for (i = 0; i < 8; i = i + 1)
                            mem1[i] <= 32'sd0;
                        mem2[0] <= 32'sd0;
                        mem2[1] <= 32'sd0;
                        state <= S_FC1_MAC;
                    end
                end

                S_FC1_MAC: begin
                    if (elem_cnt == 8'd143) begin
                        state     <= S_FC1_DRAIN;
                        drain_cnt <= 2'd0;
                    end else begin
                        elem_cnt <= elem_cnt + 8'd1;
                    end
                end

                S_FC1_DRAIN: begin
                    if (drain_cnt == 2'd2)
                        state <= S_FC1_LIF0;
                    else
                        drain_cnt <= drain_cnt + 2'd1;
                end

                S_FC1_LIF0: begin
                    lif_decayed_reg <= mem1_decay_calc;
                    state           <= S_FC1_LIF1;
                end

                S_FC1_LIF1: begin
                    lif_new_reg <= lif_decayed_reg + acc;
                    state       <= S_FC1_LIF2;
                end

                S_FC1_LIF2: begin
                    lif_spike_reg <= (lif_new_reg >= fc1_threshold_ext);
                    state         <= S_FC1_LIF3;
                end

                S_FC1_LIF3: begin
                    if (lif_spike_reg) begin
                        spk1_vec[neuron_cnt[2:0]] <= 1'b1;
                        mem1[neuron_cnt[2:0]]     <= 32'sd0;
                    end else begin
                        spk1_vec[neuron_cnt[2:0]] <= 1'b0;
                        mem1[neuron_cnt[2:0]]     <= lif_new_reg;
                    end

                    acc      <= 32'sd0;
                    elem_cnt <= 8'd0;

                    if (neuron_cnt == 4'd7) begin
                        neuron_cnt <= 4'd0;
                        state      <= S_FC2_MAC;
                    end else begin
                        neuron_cnt <= neuron_cnt + 4'd1;
                        state      <= S_FC1_MAC;
                    end
                end

                S_FC2_MAC: begin
                    if (elem_cnt == 8'd7) begin
                        state     <= S_FC2_DRAIN;
                        drain_cnt <= 2'd0;
                    end else begin
                        elem_cnt <= elem_cnt + 8'd1;
                    end
                end

                S_FC2_DRAIN: begin
                    if (drain_cnt == 2'd2)
                        state <= S_FC2_LIF0;
                    else
                        drain_cnt <= drain_cnt + 2'd1;
                end

                S_FC2_LIF0: begin
                    lif_decayed_reg <= mem2_decay_calc;
                    state           <= S_FC2_LIF1;
                end

                S_FC2_LIF1: begin
                    lif_new_reg <= lif_decayed_reg + acc;
                    state       <= S_FC2_LIF2;
                end

                S_FC2_LIF2: begin
                    lif_spike_reg <= (lif_new_reg >= fc2_threshold_ext);
                    state         <= S_FC2_LIF3;
                end

                S_FC2_LIF3: begin
                    if (lif_spike_reg) begin
                        mem2[neuron_cnt[0]] <= 32'sd0;
                        if (neuron_cnt[0] == 1'b0)
                            spk_cnt0 <= spk_cnt0 + 5'd1;
                        else
                            spk_cnt1 <= spk_cnt1 + 5'd1;
                    end else begin
                        mem2[neuron_cnt[0]] <= lif_new_reg;
                    end

                    acc      <= 32'sd0;
                    elem_cnt <= 8'd0;

                    if (neuron_cnt == 4'd1) begin
                        if (step_cnt == 5'd19)
                            state <= S_DONE;
                        else begin
                            step_cnt   <= step_cnt + 5'd1;
                            neuron_cnt <= 4'd0;
                            state      <= S_FC1_MAC;
                        end
                    end else begin
                        neuron_cnt <= neuron_cnt + 4'd1;
                        state      <= S_FC2_MAC;
                    end
                end

                S_DONE: begin
                    spike_count_0 <= spk_cnt0;
                    spike_count_1 <= spk_cnt1;
                    result        <= (spk_cnt0 > spk_cnt1) ? 1'b1 : 1'b0;
                    done          <= 1'b1;
                    if (!start)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

    // synthesis translate_off
    wire _unused_s_axi_aresetn = s_axi_aresetn;
    // synthesis translate_on

endmodule