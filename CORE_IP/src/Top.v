`timescale 1ns / 1ps

module Top (
    // ==== Clocks ====
    input  wire        s_axis_aclk,    // 150MHz : AXI-Stream + inference
    input  wire        s_axi_aclk,     // 100MHz : AXI GPIO

    // ==== Resets ====
    input  wire        s_axis_aresetn,
    input  wire        s_axi_aresetn,

    // ==== AXI-Stream (s_axis_aclk) ====
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tuser,

    // ==== GPIO (s_axi_aclk) ====
    input  wire [31:0] gpio_ctrl,
    input  wire [31:0] gpio_threshold,

    // ==== 추론 결과 (s_axis_aclk) ====
    output wire        done,
    output wire        result,
    output wire [4:0]  spike_count_0,
    output wire [4:0]  spike_count_1
);

    // =========================================================
    // AXI-Stream → pixel 변환 (s_axis_aclk)
    // =========================================================
    assign s_axis_tready = 1'b1;

    wire i_pixel_data  = s_axis_tdata[0];
    wire i_pixel_valid = s_axis_tvalid;
    wire i_frame_start = s_axis_tvalid & s_axis_tuser;

    // =========================================================
    // GPIO weight write 신호 (s_axi_aclk)
    // =========================================================
    wire        w_wr_en_g = gpio_ctrl[0];
    wire        w_layer_g = gpio_ctrl[1];
    wire [10:0] w_addr_g  = gpio_ctrl[12:2];
    wire signed [7:0] w_data_g = gpio_ctrl[20:13];

    // =========================================================
    // Threshold CDC (s_axi_aclk → s_axis_aclk)
    // =========================================================
    (* ASYNC_REG = "TRUE" *) reg [31:0] gpio_thr_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] gpio_thr_sync2;

    always @(posedge s_axis_aclk or negedge s_axis_aresetn) begin
        if (!s_axis_aresetn) begin
            gpio_thr_sync1 <= 32'd0;
            gpio_thr_sync2 <= 32'd0;
        end else begin
            gpio_thr_sync1 <= gpio_threshold;
            gpio_thr_sync2 <= gpio_thr_sync1;
        end
    end

    wire signed [15:0] fc1_threshold = gpio_thr_sync2[15:0];
    wire signed [15:0] fc2_threshold = gpio_thr_sync2[31:16];

    // =========================================================
    // 내부 연결선
    // =========================================================
    wire        fe_snn_wr_en;
    wire [11:0] fe_snn_addr;
    wire [2:0]  fe_snn_data;
    wire        fe_snn_start;
    wire [4:0]  fe_read_offset;
    wire        snn_idle;
    wire        snn_done_raw;
    wire        snn_result_raw;

    assign done   = snn_done_raw;
    assign result = snn_done_raw & snn_result_raw;

    // =========================================================
    // feature_extractor (s_axis_aclk)
    // =========================================================
    feature_extractor u_feature_extractor (
        .clk            (s_axis_aclk),
        .rst_n          (s_axis_aresetn),
        .i_frame_start  (i_frame_start),
        .i_pixel_valid  (i_pixel_valid),
        .i_pixel_data   (i_pixel_data),
        .o_snn_wr_en    (fe_snn_wr_en),
        .o_snn_addr     (fe_snn_addr),
        .o_snn_data     (fe_snn_data),
        .o_snn_start    (fe_snn_start),
        .o_read_offset  (fe_read_offset),
        .i_snn_idle     (snn_idle)
    );

    // =========================================================
    // snn_top (dual-clock)
    // =========================================================
    snn_top u_snn_top (
        .s_axis_aclk    (s_axis_aclk),
        .s_axi_aclk     (s_axi_aclk),
        .s_axis_aresetn (s_axis_aresetn),
        .s_axi_aresetn  (s_axi_aresetn),

        .start          (fe_snn_start),
        .done           (snn_done_raw),
        .idle           (snn_idle),
        .result         (snn_result_raw),
        .spike_count_0  (spike_count_0),
        .spike_count_1  (spike_count_1),

        .fc1_threshold  (fc1_threshold),
        .fc2_threshold  (fc2_threshold),

        .w_wr_en        (w_wr_en_g),
        .w_layer        (w_layer_g),
        .w_addr         (w_addr_g),
        .w_data         (w_data_g),

        .in_wr_en       (fe_snn_wr_en),
        .in_addr        (fe_snn_addr),
        .in_data        (fe_snn_data),

        .read_offset    (fe_read_offset)
    );

endmodule
