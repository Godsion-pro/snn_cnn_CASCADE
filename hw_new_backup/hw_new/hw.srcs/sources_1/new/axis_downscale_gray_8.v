`timescale 1ns / 1ps

module axis_downscale_gray_8 #
(
    parameter IN_WIDTH   = 1280,
    parameter IN_HEIGHT  = 720,
    parameter OUT_WIDTH  = 160,
    parameter OUT_HEIGHT = 90
)
(
    input  wire        aclk,
    input  wire        aresetn,

    // AXI4-Stream slave input : RGB888
    input  wire [23:0] s_axis_tdata,
    input  wire [2:0]  s_axis_tkeep,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,

    // AXI4-Stream master output : Gray8
    output wire [7:0]  m_axis_tdata,
    output wire [0:0]  m_axis_tkeep,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,
    output wire        m_axis_tlast
);

    // ============================================================
    // 1. Coordinate counters
    // ============================================================
    reg [15:0] in_x;
    reg [15:0] in_y;

    reg [15:0] out_x;
    reg [15:0] out_y;

    // ============================================================
    // 2. Output register
    // ============================================================
    reg [7:0] out_tdata_r;
    reg       out_tvalid_r;
    reg       out_tuser_r;
    reg       out_tlast_r;

    assign m_axis_tdata  = out_tdata_r;
    assign m_axis_tkeep  = 1'b1;
    assign m_axis_tvalid = out_tvalid_r;
    assign m_axis_tuser  = out_tuser_r;
    assign m_axis_tlast  = out_tlast_r;

    // ============================================================
    // 3. Input RGB split
    // ============================================================
    wire [7:0] r;
    wire [7:0] g;
    wire [7:0] b;

    assign r = s_axis_tdata[23:16];
    assign g = s_axis_tdata[15:8];
    assign b = s_axis_tdata[7:0];

    // 1280x720 -> 160x90
    // nearest-neighbor: x,y 둘 다 8의 배수일 때만 샘플링
    wire sample_en;
    // Treat input SOF as the authoritative frame boundary. If output SOF is
    // generated only from stale out_x/out_y counters, VDMA and CNN can both
    // see a vertically wrapped frame after the stream starts mid-frame.
    wire [15:0] eff_in_x;
    wire [15:0] eff_in_y;
    wire [15:0] eff_out_x;
    wire [15:0] eff_out_y;
    wire        sample_en;

    assign eff_in_x  = s_axis_tuser ? 16'd0 : in_x;
    assign eff_in_y  = s_axis_tuser ? 16'd0 : in_y;
    assign eff_out_x = s_axis_tuser ? 16'd0 : out_x;
    assign eff_out_y = s_axis_tuser ? 16'd0 : out_y;
    assign sample_en = (eff_in_x[2:0] == 3'b000) && (eff_in_y[2:0] == 3'b000);

    // ============================================================
    // 4. Pipeline control
    // ============================================================
    wire pipe_ce;
    wire in_fire;

    // output register가 비어 있거나, 현재 output을 downstream이 받을 수 있으면 pipeline 진행
    assign pipe_ce = (!out_tvalid_r) || m_axis_tready;

    // 보수적 backpressure:
    // pipeline이 진행 가능한 경우에만 input을 받는다.
    assign s_axis_tready = aresetn && pipe_ce;

    assign in_fire = s_axis_tvalid && s_axis_tready;

    // ============================================================
    // 5. Grayscale pipeline
    //
    // Original:
    //   gray8 = (77*R + 150*G + 29*B) >> 8
    //
    // Pipeline:
    //   stage 1 : sampled RGB capture
    //   stage 2 : 77*R, 150*G, 29*B
    //   stage 3 : 77*R + 150*G
    //   output  : (77*R + 150*G + 29*B) >> 8
    // ============================================================

    // stage 1: sampled RGB capture
    reg [7:0] r_s1;
    reg [7:0] g_s1;
    reg [7:0] b_s1;
    reg       v_s1;
    reg       user_s1;
    reg       last_s1;

    // stage 2: coefficient multiplication
    reg [15:0] y_r_s2;
    reg [15:0] y_g_s2;
    reg [15:0] y_b_s2;
    reg        v_s2;
    reg        user_s2;
    reg        last_s2;

    // stage 3: partial sum
    reg [15:0] y_rg_s3;
    reg [15:0] y_b_s3;
    reg        v_s3;
    reg        user_s3;
    reg        last_s3;

    wire [15:0] y_sum_s3;
    assign y_sum_s3 = y_rg_s3 + y_b_s3;

    // ============================================================
    // 6. Main sequential logic
    // ============================================================
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

            r_s1         <= 8'd0;
            g_s1         <= 8'd0;
            b_s1         <= 8'd0;
            v_s1         <= 1'b0;
            user_s1      <= 1'b0;
            last_s1      <= 1'b0;

            y_r_s2       <= 16'd0;
            y_g_s2       <= 16'd0;
            y_b_s2       <= 16'd0;
            v_s2         <= 1'b0;
            user_s2      <= 1'b0;
            last_s2      <= 1'b0;

            y_rg_s3      <= 16'd0;
            y_b_s3       <= 16'd0;
            v_s3         <= 1'b0;
            user_s3      <= 1'b0;
            last_s3      <= 1'b0;

        end else begin
            if (pipe_ce) begin
                // ------------------------------------------------
                // stage 3 -> output register
                // ------------------------------------------------
                out_tdata_r  <= y_sum_s3[15:8];
                out_tvalid_r <= v_s3;
                out_tuser_r  <= user_s3;
                out_tlast_r  <= last_s3;

                // ------------------------------------------------
                // stage 2 -> stage 3
                // ------------------------------------------------
                y_rg_s3 <= y_r_s2 + y_g_s2;
                y_b_s3  <= y_b_s2;

                v_s3    <= v_s2;
                user_s3 <= user_s2;
                last_s3 <= last_s2;

                // ------------------------------------------------
                // stage 1 -> stage 2
                // ------------------------------------------------
                y_r_s2 <= {8'd0, r_s1} * 8'd77;
                y_g_s2 <= {8'd0, g_s1} * 8'd150;
                y_b_s2 <= {8'd0, b_s1} * 8'd29;

                v_s2    <= v_s1;
                user_s2 <= user_s1;
                last_s2 <= last_s1;

                // ------------------------------------------------
                // default: no new sampled pixel
                // ------------------------------------------------
                r_s1    <= 8'd0;
                g_s1    <= 8'd0;
                b_s1    <= 8'd0;
                v_s1    <= 1'b0;
                user_s1 <= 1'b0;
                last_s1 <= 1'b0;

                // ------------------------------------------------
                // input accept
                // ------------------------------------------------
                if (in_fire) begin
                    if (sample_en) begin
                        // sampled RGB capture
                        r_s1 <= r;
                        g_s1 <= g;
                        b_s1 <= b;

                        v_s1    <= 1'b1;
                        user_s1 <= s_axis_tuser;
                        last_s1 <= (eff_out_x == (OUT_WIDTH - 1));

                        // output coordinate update
                        if (eff_out_x == (OUT_WIDTH - 1)) begin
                            out_x <= 16'd0;

                            if (eff_out_y == (OUT_HEIGHT - 1))
                                out_y <= 16'd0;
                            else
                                out_y <= eff_out_y + 16'd1;
                        end else begin
                            out_x <= eff_out_x + 16'd1;
                            out_y <= eff_out_y;
                        end
                    end

                    // input coordinate update
                    if (s_axis_tlast) begin
                        in_x <= 16'd0;

                        if (eff_in_y == (IN_HEIGHT - 1))
                            in_y <= 16'd0;
                        else
                            in_y <= eff_in_y + 16'd1;
                    end else begin
                        in_x <= eff_in_x + 16'd1;
                        in_y <= eff_in_y;
                    end
                end
            end
        end
    end

endmodule
