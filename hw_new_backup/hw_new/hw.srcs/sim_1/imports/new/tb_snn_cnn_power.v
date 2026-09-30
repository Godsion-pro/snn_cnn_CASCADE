`timescale 1ns / 1ps
// =====================================================================
// tb_snn_cnn_power
// ---------------------------------------------------------------------
//  SAIF/전력 추정용 통합 TB (DUT = snn_cnn_power_dut)
//
//  흐름 (사용자 요청):
//    160x90 image -> frame_diff -> SNN(Top) 누적 추론 -> snn_done
//        -> [모드 전환] CNN ON / SNN OFF
//        -> CNN 이 동일 160x90 원본으로 추론 -> irq_infer_done
//
//  주의:
//   * SNN 은 20프레임 누적해야 done 이 뜬다 (feature_extractor 시간버퍼).
//     -> NUM_INPUT_FRAMES 를 충분히(>=22) 준다.
//   * weight 로딩은 "그 코어 클럭이 켜진 모드"에서 해야 한다.
//       SNN weight  : mode_sel=1 (SNN 모드)
//       CNN weight  : mode_sel=0 (CNN 모드, cnn_core 클럭 ON)
//   * SAIF 측정 구간은 weight 로드를 제외한 "프레임 입력~추론" 구간.
//     아래 $display 의 [SAIF] 시각을 run_power_saif.tcl 에서 사용.
//
//  사용자가 준비할 hex (TB 와 같은 sim 실행 폴더에):
//    input_frames.hex: 연속 프레임 시퀀스. NUM_INPUT_FRAMES x (160x90=14400) byte,
//                      gray8, 한 줄당 1바이트 hex. frame0,frame1,... 순서로 concat.
//                      diff step k 는 prev=frame[k], curr=frame[k+1] (sliding).
//    fc1_weight.hex  : SNN FC1 가중치 1152개 (8bit hex)
//    fc2_weight.hex  : SNN FC2 가중치 16개
//    cnn_w.hex       : CNN flat weight  (CNN_W_TOTAL 개, 8bit)
//    cnn_b.hex       : CNN flat bias    (CNN_B_TOTAL 개, 32bit hex)
// =====================================================================
module tb_snn_cnn_power;

    // ---------- 사용자 조정 파라미터 ----------
    localparam integer FRAME_W      = 160;
    localparam integer FRAME_H      = 90;
    localparam integer FRAME_PIXELS = FRAME_W * FRAME_H;   // 14400
    localparam integer NUM_INPUT_FRAMES = 24;              // 실제 입력 프레임 수 (>=22: SNN 20프레임 + 여유)
                                                           // diff step k: prev=frame[k], curr=frame[k+1] (sliding)

    localparam integer FC1_W_CNT = 1152;   // SNN FC1 (8 x 144)
    localparam integer FC2_W_CNT = 16;     // SNN FC2 (2 x 8)
    localparam signed [15:0] FC1_THR = 16'sd171;
    localparam signed [15:0] FC2_THR = 16'sd146;

    // cnn_weights (1).h 기준
    localparam integer CNN_W_TOTAL = 32'd11144;
    localparam integer CNN_B_TOTAL = 32'd110;

    // CNN CONTROL 비트
    localparam [31:0] CTL_RESET = 32'h0000_0002;
    localparam [31:0] CTL_AUTO  = 32'h0000_0004;
    localparam [31:0] CTL_FORCE = 32'h0000_0008;
    localparam [11:0] ADDR_CONTROL = 12'h000;
    localparam [11:0] ADDR_W_LOAD  = 12'h800;
    localparam [11:0] ADDR_B_ADDR  = 12'h804;
    localparam [11:0] ADDR_B_DATA  = 12'h808;

    // ============================================================
    // Clocks / resets
    // ============================================================
    reg mm_clk_150   = 1'b0;   // 150 MHz pixel/SNN
    reg s_axi_aclk   = 1'b0;   // 100 MHz AXI/CNN
    always #3.333 mm_clk_150 = ~mm_clk_150;
    always #5     s_axi_aclk = ~s_axi_aclk;

    reg s_axis_aresetn = 1'b0;
    reg s_axi_aresetn  = 1'b0;

    // ============================================================
    // DUT I/O
    // ============================================================
    reg         mode_sel = 1'b1;     // 시작은 SNN 모드

    reg  [7:0]  curr_tdata = 0;  reg curr_tvalid = 0;  wire curr_tready;  reg curr_tlast = 0;  reg curr_tuser = 0;
    reg  [7:0]  prev_tdata = 0;  reg prev_tvalid = 0;  wire prev_tready;  reg prev_tlast = 0;  reg prev_tuser = 0;

    reg  [31:0] gpio_ctrl      = 0;
    reg  [31:0] gpio_threshold = 0;

    wire        snn_done, snn_result;
    wire [4:0]  snn_spike_count_0, snn_spike_count_1;

    reg  [7:0]  cnn_pix_tdata = 0;  reg cnn_pix_tvalid = 0;  reg cnn_pix_tuser = 0;  wire cnn_pix_tready;

    reg         flying_obj_flag = 0;
    wire        irq_frame_done, irq_infer_done;

    reg  [11:0] s_axi_awaddr=0;  reg s_axi_awvalid=0; wire s_axi_awready;
    reg  [31:0] s_axi_wdata=0;   reg [3:0] s_axi_wstrb=4'hF; reg s_axi_wvalid=0; wire s_axi_wready;
    wire [1:0]  s_axi_bresp;     wire s_axi_bvalid; reg s_axi_bready=0;
    reg  [11:0] s_axi_araddr=0;  reg s_axi_arvalid=0; wire s_axi_arready;
    wire [31:0] s_axi_rdata;     wire [1:0] s_axi_rresp; wire s_axi_rvalid; reg s_axi_rready=0;

    // ============================================================
    // DUT
    // ============================================================
    snn_cnn_power_dut #(
        .FRAME_DIFF_THRESHOLD (30),
        // cnn_weights (1).h 의 per-layer shift (기본 8 아님!)
        .CONV1_SHIFT (6), .CONV2_SHIFT (7), .CONV3_SHIFT (6),
        .CONV4_SHIFT (8), .CONV5_SHIFT (6), .DET_SHIFT (9)
    ) dut (
        .mm_clk_150     (mm_clk_150),
        .s_axi_aclk     (s_axi_aclk),
        .s_axis_aresetn (s_axis_aresetn),
        .s_axi_aresetn  (s_axi_aresetn),
        .mode_sel       (mode_sel),

        .curr_tdata(curr_tdata), .curr_tvalid(curr_tvalid), .curr_tready(curr_tready),
        .curr_tlast(curr_tlast), .curr_tuser(curr_tuser),
        .prev_tdata(prev_tdata), .prev_tvalid(prev_tvalid), .prev_tready(prev_tready),
        .prev_tlast(prev_tlast), .prev_tuser(prev_tuser),

        .gpio_ctrl(gpio_ctrl), .gpio_threshold(gpio_threshold),
        .snn_done(snn_done), .snn_result(snn_result),
        .snn_spike_count_0(snn_spike_count_0), .snn_spike_count_1(snn_spike_count_1),

        .cnn_pix_tdata(cnn_pix_tdata), .cnn_pix_tvalid(cnn_pix_tvalid),
        .cnn_pix_tuser(cnn_pix_tuser), .cnn_pix_tready(cnn_pix_tready),

        .flying_obj_flag(flying_obj_flag),
        .irq_frame_done(irq_frame_done), .irq_infer_done(irq_infer_done),

        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready)
    );

    // ============================================================
    // 입력 이미지 메모리 + 프레임 모션 생성
    // ============================================================
    reg [7:0] frames [0:NUM_INPUT_FRAMES*FRAME_PIXELS-1];   // 연속 프레임 시퀀스
    reg [7:0] fc1_mem [0:FC1_W_CNT-1];
    reg [7:0] fc2_mem [0:FC2_W_CNT-1];
    // CNN weight/bias (크기 넉넉히; 실제 개수만 사용)
    reg [7:0]  cnn_w [0:262143];
    reg [31:0] cnn_b [0:1023];

    // frame_idx 번째 프레임의 (x,y) 픽셀
    function [7:0] frame_pix;
        input integer frame_idx;
        input integer x;
        input integer y;
        begin
            frame_pix = frames[frame_idx*FRAME_PIXELS + y*FRAME_W + x];
        end
    endfunction

    // ============================================================
    // AXI-Lite write / read task
    // ============================================================
    task axi_write;
        input [11:0] addr; input [31:0] data;
        begin
            @(posedge s_axi_aclk);
            s_axi_awaddr<=addr; s_axi_awvalid<=1; s_axi_wdata<=data; s_axi_wstrb<=4'hF; s_axi_wvalid<=1; s_axi_bready<=1;
            wait (s_axi_awready && s_axi_wready);
            @(posedge s_axi_aclk);
            s_axi_awvalid<=0; s_axi_wvalid<=0;
            wait (s_axi_bvalid);
            @(posedge s_axi_aclk);
            s_axi_bready<=0;
        end
    endtask

    // ============================================================
    // SNN weight 1개 쓰기 (Top gpio_ctrl 경유, s_axi_aclk 도메인)
    //   gpio_ctrl[0]=wr_en, [1]=layer, [12:2]=addr, [20:13]=data
    // ============================================================
    task snn_write_weight;
        input        layer;
        input [10:0] addr;
        input [7:0]  data;
        begin
            @(posedge s_axi_aclk);
            gpio_ctrl <= ({11'd0, data, addr, layer, 1'b1});
            @(posedge s_axi_aclk);
            gpio_ctrl <= 32'd0;
        end
    endtask

    // ============================================================
    // diff step k : prev=frame[k], curr=frame[k+1] (sliding window)
    //   prev/curr -> frame_diff,  curr -> CNN(원본 gray8)
    //   세 스트림 모두 mm_clk_150 도메인, 한 프레임(14400px) 전송
    // ============================================================
    task send_diff_step;
        input integer k;          // prev=frame[k], curr=frame[k+1]
        integer i, x, y;
        begin
            @(posedge mm_clk_150);
            for (i = 0; i < FRAME_PIXELS; i = i + 1) begin
                x = i % FRAME_W;
                y = i / FRAME_W;

                prev_tdata    <= frame_pix(k,     x, y);   // 이전 프레임
                curr_tdata    <= frame_pix(k + 1, x, y);   // 현재 프레임
                cnn_pix_tdata <= frame_pix(k + 1, x, y);   // CNN 원본 = 현재 프레임

                curr_tvalid <= 1'b1;  prev_tvalid <= 1'b1;  cnn_pix_tvalid <= 1'b1;
                curr_tuser  <= (i==0); prev_tuser <= (i==0); cnn_pix_tuser <= (i==0);
                curr_tlast  <= (i==FRAME_PIXELS-1);
                prev_tlast  <= (i==FRAME_PIXELS-1);

                // 세 스트림 모두 ready 일 때 진행
                wait (curr_tready && prev_tready && cnn_pix_tready);
                @(posedge mm_clk_150);
            end
            curr_tvalid<=0; prev_tvalid<=0; cnn_pix_tvalid<=0;
            curr_tuser<=0;  prev_tuser<=0;  cnn_pix_tuser<=0;
            curr_tlast<=0;  prev_tlast<=0;
        end
    endtask

    // ============================================================
    // Watchdog
    // ============================================================
    initial begin
        repeat (60000000) @(posedge s_axi_aclk);
        $display("[TB] TIMEOUT");
        $finish;
    end

    // ============================================================
    // Main
    // ============================================================
    integer f, i;

    // SNN done 래치 (SNN clk 는 게이팅되므로 mm_clk_150 로 샘플)
    reg snn_done_seen = 1'b0;
    always @(posedge mm_clk_150 or negedge s_axis_aresetn) begin
        if (!s_axis_aresetn) snn_done_seen <= 1'b0;
        else if (snn_done)   snn_done_seen <= 1'b1;
    end

    // CNN 추론완료 래치 (s_axi_aclk, ungated). 종료 기준 = 마지막 프레임 추론 끝.
    reg infer_done_seen = 1'b0;
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn)      infer_done_seen <= 1'b0;
        else if (irq_infer_done) infer_done_seen <= 1'b1;
    end

    initial begin
        $display("==== tb_snn_cnn_power start ====");
        // ★ 절대경로: xsim 폴더(.sim/.../xsim)는 시뮬마다 재생성되어 거기 둔 hex 는
        //   사라진다. 안 지워지는 sim_data 폴더를 직접 가리킨다.
        $readmemh("C:/Users/sions/Desktop/jolnon/hw_new_cnnintegration_0602/sim_data/no_moving_3s_moving_3s.hex", frames);
        $readmemh("C:/Users/sions/Desktop/jolnon/hw_new_cnnintegration_0602/sim_data/fc1_weight.hex",  fc1_mem);
        $readmemh("C:/Users/sions/Desktop/jolnon/hw_new_cnnintegration_0602/sim_data/fc2_weight.hex",  fc2_mem);
        if (CNN_W_TOTAL > 0) $readmemh("C:/Users/sions/Desktop/jolnon/hw_new_cnnintegration_0602/sim_data/cnn_w.hex", cnn_w);
        if (CNN_B_TOTAL > 0) $readmemh("C:/Users/sions/Desktop/jolnon/hw_new_cnnintegration_0602/sim_data/cnn_b.hex", cnn_b);

        gpio_threshold = {FC2_THR, FC1_THR};   // Top: [15:0]=fc1, [31:16]=fc2

        // ---- reset ----
        mode_sel       = 1'b1;          // SNN 모드로 시작
        s_axis_aresetn = 1'b0; s_axi_aresetn = 1'b0;
        repeat (40) @(posedge s_axi_aclk);
        s_axis_aresetn = 1'b1; s_axi_aresetn = 1'b1;
        repeat (40) @(posedge s_axi_aclk);
        $display("[TB] reset released @ %0t", $time);

        // ============================================================
        // (1) SNN weight 로드  (SNN 모드: s_axis_aclk ON)
        // ============================================================
        for (i = 0; i < FC1_W_CNT; i = i + 1) snn_write_weight(1'b0, i[10:0], fc1_mem[i]);
        for (i = 0; i < FC2_W_CNT; i = i + 1) snn_write_weight(1'b1, i[10:0], fc2_mem[i]);
        @(posedge s_axi_aclk); gpio_ctrl <= 0;
        $display("[TB] SNN weights loaded @ %0t", $time);

        // ============================================================
        // (2) CNN weight 로드  (CNN 모드 전환: cnn_core 클럭 ON 필요)
        // ============================================================
        mode_sel = 1'b0;
        repeat (10) @(posedge s_axi_aclk);
        axi_write(ADDR_CONTROL, CTL_RESET);
        repeat (20) @(posedge s_axi_aclk);
        axi_write(ADDR_CONTROL, 32'd0);

        for (i = 0; i < CNN_W_TOTAL; i = i + 1)
            axi_write(ADDR_W_LOAD, ((i << 8) & 32'h003F_FF00) | cnn_w[i]);  // [21:8]=addr,[7:0]=data
        for (i = 0; i < CNN_B_TOTAL; i = i + 1) begin
            axi_write(ADDR_B_ADDR, i);
            axi_write(ADDR_B_DATA, cnn_b[i]);
        end
        // AUTO+FORCE: 프레임 있으면 자동 추론
        axi_write(ADDR_CONTROL, CTL_AUTO | CTL_FORCE);
        $display("[TB] CNN weights loaded @ %0t", $time);

        // ============================================================
        // (3) SNN 모드로 복귀 후 프레임 입력 시작
        //     >>> 여기서부터가 SAIF 측정 구간 <<<
        // ============================================================
        mode_sel = 1'b1;
        repeat (20) @(posedge mm_clk_150);
        $display("[SAIF] ===== MEASURE WINDOW START @ %0t =====", $time);
        $stop;   // <-- Tcl 이 여기서 open_saif + log_saif 시작 (weight 로드 제외)

        // sliding 으로 프레임 흘리면서 SNN done 이 latch 되면 CNN 모드로 전환.
        // (fork 레이스 방지: 한 프로세스만 스트림 reg 구동)
        for (f = 0; f < NUM_INPUT_FRAMES - 1; f = f + 1) begin
            send_diff_step(f);       // prev=frame[f], curr=frame[f+1]
            if (snn_done_seen && mode_sel) begin
                $display("[TB] SNN done @ %0t (result=%b spk0=%0d spk1=%0d)",
                         $time, snn_result, snn_spike_count_0, snn_spike_count_1);
                mode_sel = 1'b0;     // CNN ON / SNN OFF
                $display("[TB] -> switched to CNN mode @ %0t", $time);
            end
        end

        // 혹시 아직이면 done 까지 기다렸다 전환
        if (mode_sel) begin
            wait (snn_done_seen);
            mode_sel = 1'b0;
        end

        // CNN 모드에서 프레임 몇 개 더 흘려 CNN 이 캡처/추론하도록 보장
        send_diff_step(NUM_INPUT_FRAMES - 2);
        send_diff_step(NUM_INPUT_FRAMES - 2);

        // 종료 기준: 마지막 프레임의 CNN 추론 완료까지 대기 (bounded — 무한대기 방지)
        $display("[TB] waiting last-frame CNN inference ...");
        i = 0;
        while (!infer_done_seen && i < 5000000) begin
            @(posedge s_axi_aclk); i = i + 1;
        end
        if (infer_done_seen) $display("[TB] CNN inference done @ %0t", $time);
        else                 $display("[TB] WARN: infer_done not seen (timeout)");

        repeat (200) @(posedge s_axi_aclk);   // 추론 후 신호 drain
        $display("[SAIF] ===== MEASURE WINDOW END @ %0t =====", $time);
        $stop;   // <-- Tcl 이 여기서 close_saif

        $display("==== tb_snn_cnn_power end ====");
        #100;
        $finish;
    end

endmodule
