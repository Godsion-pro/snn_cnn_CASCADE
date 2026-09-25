`timescale 1ns / 1ps
//
// ============================================================
// SNN TOP 테스트벤치 (20샘플 연속 추론)
// ============================================================

module snn_top_tb;

    reg        clk;
    reg        rst_n;
    reg        start;
    wire       done;
    wire       result;
    
    wire [4:0] spk0_cnt;  // 0번 뉴런(fly) 스파이크 총합
    wire [4:0] spk1_cnt;  // 1번 뉴런(non-fly) 스파이크 총합
    
    reg signed [15:0] fc1_threshold;
    reg signed [15:0] fc2_threshold;

    reg               w_wr_en;
    reg               w_layer;
    reg [10:0]        w_addr;
    reg signed [7:0]  w_data;

    reg               in_wr_en;
    reg [11:0]        in_addr;
    reg signed [2:0]  in_data;

    // DUT
    snn_top uut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .done          (done),
        .result        (result),
        .spike_count_0 (spk0_cnt),
        .spike_count_1 (spk1_cnt),
        .fc1_threshold (fc1_threshold),
        .fc2_threshold (fc2_threshold),
        .w_wr_en       (w_wr_en),
        .w_layer       (w_layer),
        .w_addr        (w_addr),
        .w_data        (w_data),
        .in_wr_en      (in_wr_en),
        .in_addr       (in_addr),
        .in_data       (in_data)
    );

    // 100MHz 클럭
    always #5 clk = ~clk;

    // ==========================================
    // hex 파일에서 데이터 로드할 메모리 (크기 확장)
    // ==========================================
    reg [7:0] fc1_mem [0:1151];   
    reg [7:0] fc2_mem [0:15];     
    // 2880 * 20개 = 57600
    reg [7:0] inp_mem [0:575999];  
    reg       gt_mem  [0:199];    // 정답지 메모리 (200개)

    integer i, sample_idx;
    integer correct_count = 0;
    integer fly_correct = 0;
    integer nonfly_correct = 0;
  
    initial begin
        // 초기화
        clk      = 0;
        rst_n    = 0;
        start    = 0;
        w_wr_en  = 0;
        w_layer  = 0;
        w_addr   = 0;
        w_data   = 0;
        in_wr_en = 0;
        in_addr  = 0;
        in_data  = 0;
        // params.txt 에서 확인한 값을 여기에 입력!
        fc1_threshold = 16'sd171; 
        fc2_threshold = 16'sd146; 

        // 1. hex 파일 로드
        $readmemh("fc1_weight.hex", fc1_mem);
        $readmemh("fc2_weight.hex", fc2_mem);
        $readmemh("input_data.hex", inp_mem); // 20개의 샘플이 한 번에 로드됨
        $readmemh("labels.hex", gt_mem);
        #100;
        rst_n = 1;
        #50;
        
        w_layer = 1'b0;
        for (i = 0; i < 1152; i = i + 1) begin
            @(posedge clk);
            w_wr_en <= 1'b1;
            w_addr  <= i[10:0];
            w_data  <= fc1_mem[i];
        end
        
        // FC2 가중치 로드 (8 * 2 = 16개)
        w_layer = 1'b1;
        for (i = 0; i < 16; i = i + 1) begin
            @(posedge clk);
            w_wr_en <= 1'b1;
            w_addr  <= i[10:0];
            w_data  <= fc2_mem[i];
        end
        
        @(posedge clk);
        w_wr_en <= 1'b0;
        $display("[TB] Weight Loading Complete!");
        
        
        $display("\n[TB] === 200 샘플 대규모 테스트 시작 ===");

        for (sample_idx = 0; sample_idx < 200; sample_idx = sample_idx + 1) begin
            
            // 데이터 입력 (2880개)
            for (i = 0; i < 2880; i = i + 1) begin
                @(posedge clk);
                in_wr_en <= 1'b1;
                in_addr  <= i[11:0];
                in_data  <= $signed(inp_mem[sample_idx * 2880 + i]);
            end
            @(posedge clk);
            in_wr_en <= 1'b0;

            // 추론 시작
            @(posedge clk);
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;

            wait (done == 1'b1);
            @(posedge clk);
            $display("Sample %03d | TB_Result: %b | spk0: %2d, spk1: %2d | GT: %b", 
                     sample_idx, result, spk0_cnt, spk1_cnt, gt_mem[sample_idx]);
            // 정답 비교 및 통계 계산
            if (result == gt_mem[sample_idx]) begin
                correct_count = correct_count + 1;
                if (gt_mem[sample_idx] == 0) fly_correct = fly_correct + 1;
                else nonfly_correct = nonfly_correct + 1;
            end

            // 10개마다 중간 보고 (너무 많이 출력되면 느리니까)
            if (sample_idx % 10 == 0) 
                $display("[Progress] Sample %3d/200 processed...", sample_idx);
        end
        // === All-zero input test ===
$display("\n[TB] === All-Zero Input Test ===");
for (i = 0; i < 2880; i = i + 1) begin
    @(posedge clk);
    in_wr_en <= 1'b1;
    in_addr  <= i[11:0];
    in_data  <= 8'sd0;
end
@(posedge clk);
in_wr_en <= 1'b0;

@(posedge clk);
start <= 1'b1;
@(posedge clk);
start <= 1'b0;

wait (done == 1'b1);
@(posedge clk);
$display("All-Zero  | Result: %b | spk0: %2d, spk1: %2d", result, spk0_cnt, spk1_cnt);
        // 최종 결과 리포트
        $display("\n===============================================");
        $display("   FINAL SIMULATION RESULT");
        $display("===============================================");
        $display("  Total Accuracy: %d / 200 (%0d%%)", correct_count, (correct_count*100)/200);
        $display("  Fly Accuracy:    %d / 100", fly_correct);
        $display("  Non-Fly Accuracy: %d / 100", nonfly_correct);
        $display("===============================================");
        
        $finish;
    end
endmodule