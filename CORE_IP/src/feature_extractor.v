`timescale 1ns / 1ps

module feature_extractor (
    input  wire        clk,
    input  wire        rst_n,

    // --- 카메라/이벤트 입력 인터페이스 ---
    input  wire        i_frame_start,
    input  wire        i_pixel_valid,
    input  wire        i_pixel_data,

    // --- SNN Top 모듈로 보내는 인터페이스 ---
    output reg         o_snn_wr_en,
    output reg  [11:0] o_snn_addr,
    output reg  [2:0]  o_snn_data,
    output reg         o_snn_start,
    output reg  [4:0]  o_read_offset,

    // --- snn_top idle 입력 (충돌 방지) ---
    input  wire        i_snn_idle
);

    localparam WIDTH        = 160;
    localparam HEIGHT       = 90;
    localparam TOTAL_PIXELS = WIDTH * HEIGHT;

    localparam IDLE      = 3'd0;
    localparam STREAM    = 3'd1;
    localparam FLUSH_144 = 3'd2;
    localparam WAIT_IDLE = 3'd3;

    reg [2:0] state;

    reg [13:0] pixel_cnt;
    reg [4:0]  frame_ptr;
    reg        buf_full;
    reg [3:0]  x_mod10, y_mod10, blk_x, blk_y;
    reg [6:0]  block_counts [0:143];
    reg [7:0]  flush_cnt;

    // neg slack: shift로 blk_idx 계산
    wire [7:0] blk_idx = {blk_y, 4'b0000} + {4'd0, blk_x};

    wire [4:0] next_ptr = (frame_ptr == 5'd19) ? 5'd0 : frame_ptr + 5'd1;

    // neg slack: shift-add로 주소 계산 (144 = 128 + 16)
    wire [11:0] frame_base_addr =
        ({7'd0, frame_ptr} << 7) + ({7'd0, frame_ptr} << 4);
    wire [11:0] snn_write_addr =
        frame_base_addr + {4'd0, flush_cnt};

    integer i;

    function [2:0] quantize;
        input [6:0] count;
        begin
            if      (count >= 64) quantize = 3'd7;
            else if (count >= 32) quantize = 3'd6;
            else if (count >= 16) quantize = 3'd5;
            else if (count >= 8)  quantize = 3'd4;
            else if (count >= 4)  quantize = 3'd3;
            else if (count >= 2)  quantize = 3'd2;
            else if (count >= 1)  quantize = 3'd1;
            else                  quantize = 3'd0;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            pixel_cnt     <= 14'd0;
            frame_ptr     <= 5'd0;
            buf_full      <= 1'b0;
            flush_cnt     <= 8'd0;
            x_mod10       <= 4'd0;
            y_mod10       <= 4'd0;
            blk_x         <= 4'd0;
            blk_y         <= 4'd0;
            o_snn_wr_en   <= 1'b0;
            o_snn_start   <= 1'b0;
            o_snn_addr    <= 12'd0;
            o_snn_data    <= 3'd0;
            o_read_offset <= 5'd0;
            for (i = 0; i < 144; i = i + 1)
                block_counts[i] <= 7'd0;
        end else begin
            o_snn_wr_en <= 1'b0;
            o_snn_start <= 1'b0;

            case (state)

                IDLE: begin
                    if (i_frame_start) begin
                        pixel_cnt <= 14'd0;
                        flush_cnt <= 8'd0;
                        x_mod10   <= 4'd0;
                        y_mod10   <= 4'd0;
                        blk_x     <= 4'd0;
                        blk_y     <= 4'd0;
                        for (i = 0; i < 144; i = i + 1)
                            block_counts[i] <= 7'd0;
                        state <= STREAM;
                    end
                end

                STREAM: begin
                    if (i_pixel_valid) begin
                        // neg slack: for문 개별 비교 (큰 MUX 방지)
                        if (i_pixel_data) begin
                            for (i = 0; i < 144; i = i + 1) begin
                                if (blk_idx == i[7:0])
                                    block_counts[i] <= block_counts[i] + 7'd1;
                            end
                        end

                        pixel_cnt <= pixel_cnt + 14'd1;

                        if (x_mod10 == 4'd9) begin
                            x_mod10 <= 4'd0;
                            if (blk_x == 4'd15) begin
                                blk_x <= 4'd0;
                                if (y_mod10 == 4'd9) begin
                                    y_mod10 <= 4'd0;
                                    if (blk_y != 4'd8)
                                        blk_y <= blk_y + 4'd1;
                                end else begin
                                    y_mod10 <= y_mod10 + 4'd1;
                                end
                            end else begin
                                blk_x <= blk_x + 4'd1;
                            end
                        end else begin
                            x_mod10 <= x_mod10 + 4'd1;
                        end

                        if (pixel_cnt == TOTAL_PIXELS - 1) begin
                            flush_cnt <= 8'd0;
                            state     <= FLUSH_144;
                        end
                    end
                end

                FLUSH_144: begin
                    o_snn_wr_en <= 1'b1;
                    o_snn_addr  <= snn_write_addr;
                    o_snn_data  <= quantize(block_counts[flush_cnt]);

                    if (flush_cnt == 8'd143) begin
                        if (frame_ptr == 5'd19 && !buf_full) begin
                            buf_full      <= 1'b1;
                            o_read_offset <= next_ptr;
                            o_snn_start   <= 1'b1;
                            frame_ptr     <= next_ptr;
                            state         <= IDLE;
                        end else if (buf_full) begin
                            state <= WAIT_IDLE;
                        end else begin
                            frame_ptr <= next_ptr;
                            state     <= IDLE;
                        end
                    end else begin
                        flush_cnt <= flush_cnt + 8'd1;
                    end
                end

                WAIT_IDLE: begin
                    if (i_snn_idle) begin
                        o_read_offset <= next_ptr;
                        o_snn_start   <= 1'b1;
                        frame_ptr     <= next_ptr;
                        state         <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule