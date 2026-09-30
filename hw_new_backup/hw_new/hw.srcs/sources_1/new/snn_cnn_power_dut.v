`timescale 1ns / 1ps
// =====================================================================
// snn_cnn_power_dut
// ---------------------------------------------------------------------
//  SAIF/전력 추정용으로 BD에서 SNN + CNN 경로만 떼어낸 합성 가능한 래퍼.
//  실제 BD 와 동일하게:
//    image(gray8) --> axis_frame_diff --> Top(SNN, feature_extractor 포함)
//    image(gray8) --> dronet_accel_axi(CNN)
//  클럭 게이팅(snn_cnn_sel + clk_en_gate/BUFGCE)도 그대로 포함하므로
//  post-synthesis functional sim 시 BUFGCE 토글까지 SAIF 에 잡힌다.
//
//  mode_sel = 1 -> SNN 모드 (SNN clk ON, CNN core clk gated)
//  mode_sel = 0 -> CNN 모드 (CNN core clk ON, SNN clk gated)
//
//  픽셀 입력은 TB 가 3개 스트림으로 구동:
//    - curr  : 현재 프레임 gray8  -> frame_diff current
//    - prev  : 이전 프레임 gray8  -> frame_diff previous
//    - pix   : 현재 프레임 gray8  -> CNN pixel (원본 그대로)
//  frame_diff 의 1-bit 모션 출력이 SNN(Top) 의 AXI-Stream 입력으로 들어간다.
// =====================================================================
module snn_cnn_power_dut #(
    parameter integer FRAME_DIFF_THRESHOLD = 30,
    parameter integer CONV1_SHIFT = 8,
    parameter integer CONV2_SHIFT = 8,
    parameter integer CONV3_SHIFT = 8,
    parameter integer CONV4_SHIFT = 8,
    parameter integer CONV5_SHIFT = 8,
    parameter integer DET_SHIFT   = 8
)(
    // ---- clocks / resets ----
    input  wire        mm_clk_150,      // pixel / SNN stream clock (BD: mm_clk_150)
    input  wire        s_axi_aclk,      // AXI-Lite / CNN-core clock (BD: FCLK_CLK0)
    input  wire        s_axis_aresetn,  // pixel-domain reset (active low)
    input  wire        s_axi_aresetn,   // axi-domain reset  (active low)

    // ---- clock-gating mode select (1 GPIO bit) ----
    input  wire        mode_sel,        // 1=SNN mode, 0=CNN mode

    // ---- frame_diff current-frame stream (gray8) ----
    input  wire [7:0]  curr_tdata,
    input  wire        curr_tvalid,
    output wire        curr_tready,
    input  wire        curr_tlast,
    input  wire        curr_tuser,

    // ---- frame_diff previous-frame stream (gray8) ----
    input  wire [7:0]  prev_tdata,
    input  wire        prev_tvalid,
    output wire        prev_tready,
    input  wire        prev_tlast,
    input  wire        prev_tuser,

    // ---- SNN weight/threshold load (Top gpio_ctrl/gpio_threshold) ----
    input  wire [31:0] gpio_ctrl,
    input  wire [31:0] gpio_threshold,

    // ---- SNN outputs ----
    output wire        snn_done,
    output wire        snn_result,
    output wire [4:0]  snn_spike_count_0,
    output wire [4:0]  snn_spike_count_1,

    // ---- CNN pixel stream (gray8, 원본) ----
    input  wire [7:0]  cnn_pix_tdata,
    input  wire        cnn_pix_tvalid,
    input  wire        cnn_pix_tuser,
    output wire        cnn_pix_tready,

    // ---- CNN misc ----
    input  wire        flying_obj_flag,
    output wire        irq_frame_done,
    output wire        irq_infer_done,

    // ---- CNN AXI-Lite slave ----
    input  wire [11:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [11:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready
);

    // =================================================================
    // 1 GPIO bit -> 상보 enable (snn_en, cnn_clk_en)
    // =================================================================
    wire snn_en;
    wire cnn_clk_en;
    snn_cnn_sel u_sel (
        .mode_sel   (mode_sel),
        .snn_en     (snn_en),
        .cnn_clk_en (cnn_clk_en)
    );

    // =================================================================
    // SNN 클럭 게이트 (mm_clk_150 -> Top.s_axis_aclk)
    //   idle = 1'b1  (즉시 상호배타; 모드 전환 시 SNN 클럭 바로 멈춤)
    // =================================================================
    wire snn_s_axis_aclk;
    clk_en_gate u_snn_clk_gate (
        .clk_in  (mm_clk_150),
        .en      (snn_en),
        .idle    (1'b1),
        .clk_out (snn_s_axis_aclk)
    );

    // =================================================================
    // frame_diff : (curr, prev) gray8 -> 1-bit 모션 스트림
    // =================================================================
    wire [7:0] diff_tdata;
    wire       diff_tvalid;
    wire       diff_tready;
    wire       diff_tlast;
    wire       diff_tuser;

    // frame_diff 는 SNN 트리거 경로 전용 -> SNN 과 같은 게이트 클럭 사용.
    // (CNN 모드(snn_en=0)에서는 frame_diff 클럭도 함께 꺼짐)
    axis_frame_diff #(
        .THRESHOLD (FRAME_DIFF_THRESHOLD)
    ) u_frame_diff (
        .aclk               (snn_s_axis_aclk),
        .aresetn            (s_axis_aresetn),

        .s_axis_curr_tdata  (curr_tdata),
        .s_axis_curr_tvalid (curr_tvalid),
        .s_axis_curr_tready (curr_tready),
        .s_axis_curr_tlast  (curr_tlast),
        .s_axis_curr_tuser  (curr_tuser),

        .s_axis_prev_tdata  (prev_tdata),
        .s_axis_prev_tvalid (prev_tvalid),
        .s_axis_prev_tready (prev_tready),
        .s_axis_prev_tlast  (prev_tlast),
        .s_axis_prev_tuser  (prev_tuser),

        .m_axis_diff_tdata  (diff_tdata),
        .m_axis_diff_tvalid (diff_tvalid),
        .m_axis_diff_tready (diff_tready),
        .m_axis_diff_tlast  (diff_tlast),
        .m_axis_diff_tuser  (diff_tuser),

        .sync_error         ()
    );

    // =================================================================
    // SNN (Top) : frame_diff 1-bit 모션을 AXI-Stream 으로 입력받음
    //   s_axis_aclk = 게이팅된 SNN 클럭
    //   s_axi_aclk  = ungated (GPIO weight 로드용)
    // =================================================================
    Top u_top (
        .s_axis_aclk    (snn_s_axis_aclk),
        .s_axi_aclk     (s_axi_aclk),
        .s_axis_aresetn (s_axis_aresetn),
        .s_axi_aresetn  (s_axi_aresetn),

        .s_axis_tdata   (diff_tdata),
        .s_axis_tvalid  (diff_tvalid),
        .s_axis_tready  (diff_tready),
        .s_axis_tlast   (diff_tlast),
        .s_axis_tuser   (diff_tuser),

        .gpio_ctrl      (gpio_ctrl),
        .gpio_threshold (gpio_threshold),

        .done           (snn_done),
        .result         (snn_result),
        .spike_count_0  (snn_spike_count_0),
        .spike_count_1  (snn_spike_count_1)
    );

    // =================================================================
    // CNN (dronet_accel_axi) : 원본 gray8 입력, cnn_core 클럭은 내부 게이팅
    //   s_axis_aclk = mm_clk_150 (pixel/frame-buffer, ungated)
    //   s_axi_aclk  = ungated (AXI-Lite + cnn_core 의 gated 파생 클럭 소스)
    // =================================================================
    dronet_accel_axi #(
        .C_S_AXI_ADDR_WIDTH (12),
        .C_S_AXI_DATA_WIDTH (32),
        .CONV1_SHIFT (CONV1_SHIFT),
        .CONV2_SHIFT (CONV2_SHIFT),
        .CONV3_SHIFT (CONV3_SHIFT),
        .CONV4_SHIFT (CONV4_SHIFT),
        .CONV5_SHIFT (CONV5_SHIFT),
        .DET_SHIFT   (DET_SHIFT)
    ) u_dronet (
        .s_axi_aclk     (s_axi_aclk),
        .s_axi_aresetn  (s_axi_aresetn),

        .cnn_clk_en     (cnn_clk_en),

        .s_axis_aclk    (mm_clk_150),
        .s_axis_aresetn (s_axis_aresetn),

        .s_pixel_tdata  (cnn_pix_tdata),
        .s_pixel_tvalid (cnn_pix_tvalid),
        .s_pixel_tuser  (cnn_pix_tuser),
        .s_pixel_tready (cnn_pix_tready),

        .flying_obj_flag(flying_obj_flag),
        .irq_frame_done (irq_frame_done),
        .irq_infer_done (irq_infer_done),

        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),
        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready)
    );

endmodule
