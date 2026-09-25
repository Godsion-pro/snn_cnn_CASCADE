`timescale 1ns / 1ps

module axis_downscale_gray8_avg #
(
    parameter IN_WIDTH   = 1280,
    parameter IN_HEIGHT  = 720,
    parameter OUT_WIDTH  = 320,
    parameter OUT_HEIGHT = 180
)
(
    input  wire        aclk,
    input  wire        aresetn,

    // AXI4-Stream slave input : RGB888
    input  wire [23:0] s_axis_tdata,
    input  wire [2:0]  s_axis_tkeep,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,   // start of frame
    input  wire        s_axis_tlast,   // end of line

    // AXI4-Stream master output : Gray8
    output wire [7:0]  m_axis_tdata,
    output wire [0:0]  m_axis_tkeep,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,   // start of frame
    output wire        m_axis_tlast    // end of line
);

    localparam integer SCALE_X = IN_WIDTH  / OUT_WIDTH;   // 4
    localparam integer SCALE_Y = IN_HEIGHT / OUT_HEIGHT;  // 4

    // 입력 좌표
    reg [15:0] in_x;
    reg [15:0] in_y;

    // 출력 좌표
    reg [15:0] out_x;
    reg [15:0] out_y;

    // 출력 레지스터
    reg [7:0] out_tdata_r;
    reg       out_tvalid_r;
    reg       out_tuser_r;
    reg       out_tlast_r;

    // grayscale 계산용
    wire [7:0] r;
    wire [7:0] g;
    wire [7:0] b;
    wire [9:0] gray_sum;
    wire [7:0] gray8;

    wire in_fire;
    wire out_fire;
    wire sample_en;

    assign r = s_axis_tdata[23:16];
    assign g = s_axis_tdata[15:8];
    assign b = s_axis_tdata[7:0];

    // 단순 평균 grayscale
    assign gray_sum = {2'b00, r} + {2'b00, g} + {2'b00, b};
    //assign gray8    = gray_sum / 3;
    // 근사형으로 바꾸고 싶으면 아래 사용:
    assign gray8 = (gray_sum * 8'd85) >> 8;

    // nearest-neighbor:
    // x,y가 각각 4의 배수일 때만 샘플링
    assign sample_en = ((in_x % SCALE_X) == 0) && ((in_y % SCALE_Y) == 0);

    // 출력 레지스터가 비어 있거나, 현재 출력이 소비되면 다음 입력을 받음
    // nearest라서 sample_en인 픽셀만 막으면 됨. 버릴 픽셀은 그냥 계속 받음.
    assign s_axis_tready = aresetn && (!sample_en || !out_tvalid_r || m_axis_tready);

    assign in_fire  = s_axis_tvalid && s_axis_tready;
    assign out_fire = out_tvalid_r && m_axis_tready;

    assign m_axis_tdata  = out_tdata_r;
    assign m_axis_tkeep  = 1'b1;   // gray8 = 1 byte valid
    assign m_axis_tvalid = out_tvalid_r;
    assign m_axis_tuser  = out_tuser_r;
    assign m_axis_tlast  = out_tlast_r;

    always @(posedge aclk) begin
        if (!aresetn) begin
            in_x         <= 16'd0;
            in_y         <= 16'd0;
            out_x        <= 16'd0;
            out_y        <= 16'd0;
            out_tdata_r  <= 8'd0;
            out_tvalid_r <= 1'b0;
            out_tuser_r  <= 1'b0;
            out_tlast_r  <= 1'b0;
        end else begin
            // 출력이 소비되면 valid 내림
            if (out_fire) begin
                out_tvalid_r <= 1'b0;
            end

            if (in_fire) begin
                // 샘플링할 픽셀만 출력 레지스터에 적재
                if (sample_en) begin
                    out_tdata_r  <= gray8;
                    out_tvalid_r <= 1'b1;
                    out_tuser_r  <= (out_x == 16'd0) && (out_y == 16'd0);
                    out_tlast_r  <= (out_x == (OUT_WIDTH - 1));

                    // 출력 좌표 진행
                    if (out_x == (OUT_WIDTH - 1)) begin
                        out_x <= 16'd0;
                        if (out_y == (OUT_HEIGHT - 1))
                            out_y <= 16'd0;
                        else
                            out_y <= out_y + 16'd1;
                    end else begin
                        out_x <= out_x + 16'd1;
                    end
                end

                // 입력 좌표 진행
                if (s_axis_tlast) begin
                    in_x <= 16'd0;

                    if (s_axis_tuser) begin
                        in_y <= 16'd0;
                    end else if (in_y == (IN_HEIGHT - 1)) begin
                        in_y <= 16'd0;
                    end else begin
                        in_y <= in_y + 16'd1;
                    end
                end else begin
                    if (s_axis_tuser) begin
                        in_x <= 16'd1;
                        in_y <= 16'd0;
                    end else begin
                        in_x <= in_x + 16'd1;
                    end
                end
            end
        end
    end

endmodule