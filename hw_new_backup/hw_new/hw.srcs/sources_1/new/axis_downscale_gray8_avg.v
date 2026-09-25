module axis_downscale_gray8_avg #(
    parameter integer IN_WIDTH   = 1280,
    parameter integer IN_HEIGHT  = 720,
    parameter integer OUT_WIDTH  = 320,
    parameter integer OUT_HEIGHT = 180
)(
    input  wire        aclk,
    input  wire        aresetn,

    // Slave AXI4-Stream input (RGB888)
    input  wire [23:0] s_axis_tdata,
    input  wire [2:0]  s_axis_tkeep,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,   // SOF
    input  wire        s_axis_tlast,   // EOL

    // Master AXI4-Stream output (Gray8)
    output wire [7:0]  m_axis_tdata,
    output wire [0:0]  m_axis_tkeep,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,   // SOF
    output wire        m_axis_tlast    // EOL
);

    localparam integer SCALE_X = IN_WIDTH  / OUT_WIDTH;   // 4
    localparam integer SCALE_Y = IN_HEIGHT / OUT_HEIGHT;  // 4
    localparam integer BLOCK_N = SCALE_X * SCALE_Y;       // 16

    initial begin
        if ((IN_WIDTH % OUT_WIDTH) != 0)
            $error("IN_WIDTH must be divisible by OUT_WIDTH");
        if ((IN_HEIGHT % OUT_HEIGHT) != 0)
            $error("IN_HEIGHT must be divisible by OUT_HEIGHT");
    end

    // 입력 좌표
    reg [$clog2(IN_WIDTH):0]   in_x;
    reg [$clog2(IN_HEIGHT):0]  in_y;

    // 출력 좌표
    reg [$clog2(OUT_WIDTH):0]  out_x;
    reg [$clog2(OUT_HEIGHT):0] out_y;

    // 4x4 블록 누적
    reg [11:0] acc_gray;   // 16 * 255 = 4080, 12bit면 충분
    reg [4:0]  acc_count;  // 0~16

    // 출력 레지스터
    reg [7:0] out_tdata_r;
    reg       out_tvalid_r;
    reg       out_tuser_r;
    reg       out_tlast_r;

    wire out_fire;
    wire in_fire;

    assign out_fire = out_tvalid_r && m_axis_tready;

    // 출력 레지스터가 차 있으면 새 입력을 멈춤
    assign s_axis_tready = aresetn && (!out_tvalid_r || m_axis_tready);
    assign in_fire = s_axis_tvalid && s_axis_tready;

    // grayscale 변환
    // Y ≈ (77R + 150G + 29B) >> 8
    wire [7:0] r = s_axis_tdata[23:16];
    wire [7:0] g = s_axis_tdata[15:8];
    wire [7:0] b = s_axis_tdata[7:0];

    wire [15:0] gray_mul =
        (r * 8'd77) +
        (g * 8'd150) +
        (b * 8'd29);

    wire [7:0] gray8 = gray_mul[15:8];

    // 현재 픽셀이 현재 출력 블록에 포함되는지
    // 사실 1280->320, 720->180 정수배라서 모든 픽셀이 4x4 블록 중 하나에 속함
    // 누적은 그냥 16개 모을 때마다 끊으면 됨
    wire end_of_block_x = (in_x % SCALE_X) == (SCALE_X - 1);
    wire end_of_block_y = (in_y % SCALE_Y) == (SCALE_Y - 1);
    wire end_of_block   = end_of_block_x && end_of_block_y;

    assign m_axis_tdata  = out_tdata_r;
    assign m_axis_tkeep  = 1'b1;
    assign m_axis_tvalid = out_tvalid_r;
    assign m_axis_tuser  = out_tuser_r;
    assign m_axis_tlast  = out_tlast_r;

    always @(posedge aclk) begin
        if (!aresetn) begin
            in_x         <= 0;
            in_y         <= 0;
            out_x        <= 0;
            out_y        <= 0;
            acc_gray     <= 12'd0;
            acc_count    <= 5'd0;
            out_tdata_r  <= 8'd0;
            out_tvalid_r <= 1'b0;
            out_tuser_r  <= 1'b0;
            out_tlast_r  <= 1'b0;
        end else begin
            if (out_fire) begin
                out_tvalid_r <= 1'b0;
            end

            if (in_fire) begin
                // SOF가 오면 프레임 시작으로 정렬
                if (s_axis_tuser) begin
                    in_x      <= 0;
                    in_y      <= 0;
                    out_x     <= 0;
                    out_y     <= 0;
                    acc_gray  <= gray8;
                    acc_count <= 5'd1;
                end else begin
                    acc_gray  <= acc_gray + gray8;
                    acc_count <= acc_count + 1'b1;
                end

                // 4x4 블록 완료 시 평균 출력 준비
                if (end_of_block) begin
                    // 현재 픽셀까지 포함된 합으로 평균
                    if (s_axis_tuser) begin
                        out_tdata_r <= gray8 >> 4; // 사실 첫 픽셀일 뿐이라 예외적, 정상 프레임에선 거의 안 씀
                    end else begin
                        out_tdata_r <= (acc_gray + gray8) >> 4;
                    end

                    out_tvalid_r <= 1'b1;
                    out_tuser_r  <= (out_x == 0 && out_y == 0);
                    out_tlast_r  <= (out_x == OUT_WIDTH - 1);

                    acc_gray     <= 12'd0;
                    acc_count    <= 5'd0;

                    // 출력 좌표 진행
                    if (out_x == OUT_WIDTH - 1) begin
                        out_x <= 0;
                        if (out_y == OUT_HEIGHT - 1)
                            out_y <= 0;
                        else
                            out_y <= out_y + 1'b1;
                    end else begin
                        out_x <= out_x + 1'b1;
                    end
                end

                // 입력 좌표 진행
                if (s_axis_tlast) begin
                    in_x <= 0;
                    if (s_axis_tuser) begin
                        in_y <= 0;
                    end else if (in_y == IN_HEIGHT - 1) begin
                        in_y <= 0;
                    end else begin
                        in_y <= in_y + 1'b1;
                    end
                end else begin
                    if (s_axis_tuser) begin
                        in_x <= 1;
                        in_y <= 0;
                    end else begin
                        in_x <= in_x + 1'b1;
                    end
                end
            end
        end
    end

endmodule