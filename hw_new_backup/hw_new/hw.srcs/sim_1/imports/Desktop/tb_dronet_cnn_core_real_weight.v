`timescale 1ns / 1ps
`include "dronet_params.vh"

module tb_dronet_cnn_core_real_weight;

    reg clk = 0;
    reg rst_n = 0;
    reg soft_reset = 0;
    reg start = 0;
    reg test_pattern_enable = 0;
    reg [15:0] frame_id = 16'd1;

    wire [`DRONET_FRAME_ADDR_W-1:0] frame_rd_addr;
    reg  [7:0] frame_rd_data;

    wire raw_wr_en;
    wire [`DRONET_RAW_ADDR_W-1:0] raw_wr_addr;
    wire signed [7:0] raw_wr_data;

    wire busy;
    wire done_pulse;
    wire [15:0] raw_frame_id;
    wire [3:0] current_step;
    wire [23:0] cycles_left;
    wire [7:0] dbg_last_frame_byte;

    integer raw_wr_count = 0;
    integer raw_nz_count = 0;
    integer cycle_count = 0;
    integer dump_fd;

    // 출력 채널별 합산 (6채널 x 220셀 = 1320 entries)
    reg signed [31:0] ch_sum [0:5];
    integer ch_idx;

    always #5 clk = ~clk;  // 100 MHz

    always @(*) begin
        frame_rd_data = 8'h80;
    end

    dronet_cnn_core #(
        .CONV1_W_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv1_w.mem"),
        .CONV1_B_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv1_b.mem"),
        .CONV2_W_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv2_w.mem"),
        .CONV2_B_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv2_b.mem"),
        .CONV3_W_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv3_w.mem"),
        .CONV3_B_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv3_b.mem"),
        .CONV4_W_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv4_w.mem"),
        .CONV4_B_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv4_b.mem"),
        .CONV5_W_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv5_w.mem"),
        .CONV5_B_FILE ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/conv5_b.mem"),
        .DET_W_FILE   ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/det_w.mem"),
        .DET_B_FILE   ("C:/Users/OMEN/Desktop/DroneNet/cnn_weights_real/det_b.mem"),
        .CONV1_SHIFT  (8),
        .CONV2_SHIFT  (8),
        .CONV3_SHIFT  (8),
        .CONV4_SHIFT  (8),
        .CONV5_SHIFT  (8),
        .DET_SHIFT    (8)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .soft_reset(soft_reset),
        .start(start),
        .test_pattern_enable(test_pattern_enable),
        .frame_id(frame_id),
        .frame_rd_addr(frame_rd_addr),
        .frame_rd_data(frame_rd_data),
        .raw_wr_en(raw_wr_en),
        .raw_wr_addr(raw_wr_addr),
        .raw_wr_data(raw_wr_data),
        .busy(busy),
        .done_pulse(done_pulse),
        .raw_frame_id(raw_frame_id),
        .current_step(current_step),
        .cycles_left(cycles_left),
        .dbg_last_frame_byte(dbg_last_frame_byte)
    );

    always @(posedge clk) begin
        cycle_count <= cycle_count + 1;

        if (raw_wr_en) begin
            raw_wr_count <= raw_wr_count + 1;
            if (raw_wr_data != 8'sd0) begin
                raw_nz_count <= raw_nz_count + 1;
            end

            // 파일에 모든 raw 출력 기록 (기준값 확보용)
            $fwrite(dump_fd, "%0d %0d\n", raw_wr_addr, raw_wr_data);

            // 채널별 합산
            ch_idx = raw_wr_addr / `DRONET_CELLS;
            if (ch_idx < 6) begin
                ch_sum[ch_idx] <= ch_sum[ch_idx] + {{24{raw_wr_data[7]}}, raw_wr_data};
            end

            if (raw_wr_count < 32 || raw_wr_data != 8'sd0) begin
                $display("[RAW] cycle=%0d addr=%0d data=%0d hex=%02h step=%0d",
                         cycle_count, raw_wr_addr, raw_wr_data, raw_wr_data[7:0], current_step);
            end
        end

        if (done_pulse) begin
            $display("==== DONE at cycle %0d ====", cycle_count);
            $display("raw_wr_count = %0d  (expect 1320)", raw_wr_count);
            $display("raw_nz_count = %0d", raw_nz_count);
            $display("dbg_last_frame_byte = %02h", dbg_last_frame_byte);
            $display("--- channel sums ---");
            for (ch_idx = 0; ch_idx < 6; ch_idx = ch_idx + 1) begin
                $display("  ch[%0d] sum = %0d", ch_idx, ch_sum[ch_idx]);
            end

            if (raw_wr_count != 1320) begin
                $display("RESULT: FAIL - expected 1320 raw writes, got %0d", raw_wr_count);
            end else if (raw_nz_count == 0) begin
                $display("RESULT: FAIL - all 1320 outputs are zero");
            end else begin
                $display("RESULT: PASS - %0d/%0d nonzero outputs", raw_nz_count, raw_wr_count);
            end

            $fclose(dump_fd);
            #100;
            $finish;
        end

        if (cycle_count > 20000000) begin
            $display("TIMEOUT: cycle=%0d step=%0d left=%0d busy=%0d wr=%0d nz=%0d",
                     cycle_count, current_step, cycles_left, busy, raw_wr_count, raw_nz_count);
            $fclose(dump_fd);
            $finish;
        end
    end

    initial begin
        $display("==== tb_dronet_cnn_core_real_weight start ====");

        dump_fd = $fopen("raw_output_dump.txt", "w");
        if (dump_fd == 0) begin
            $display("ERROR: cannot open raw_output_dump.txt");
            $finish;
        end

        for (ch_idx = 0; ch_idx < 6; ch_idx = ch_idx + 1)
            ch_sum[ch_idx] = 0;

        rst_n = 0;
        soft_reset = 0;
        start = 0;
        test_pattern_enable = 0;

        repeat (20) @(posedge clk);
        rst_n = 1;

        repeat (20) @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        $display("[TB] start pulse sent");
    end

endmodule
