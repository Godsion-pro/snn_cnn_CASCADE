`timescale 1ns / 1ps

module downscale_gray24 (
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Stream Slave input: 1280x720 RGB888
    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,   // SOF
    input  wire        s_axis_tlast,   // EOL

    // AXI4-Stream Master output: 320x180 gray replicated to RGB888
    output wire [23:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,
    output wire        m_axis_tlast
);

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam integer IN_WIDTH   = 1280;
    localparam integer IN_HEIGHT  = 720;
    localparam integer OUT_WIDTH  = 320;
    localparam integer OUT_HEIGHT = 180;

    // ----------------------------------------------------------------
    // Input pixel position counters
    // ----------------------------------------------------------------
    reg [10:0] in_x;
    reg [10:0] in_y;

    // Output x counter (for tlast generation)
    reg [8:0]  out_x;

    // Current input coordinate to evaluate this cycle
    wire [10:0] cur_x = s_axis_tuser ? 11'd0 : in_x;
    wire [10:0] cur_y = s_axis_tuser ? 11'd0 : in_y;

    // 4x downsample: only pass pixels where x,y are multiples of 4
    wire sample_en = (cur_x[1:0] == 2'b00) && (cur_y[1:0] == 2'b00);

    // Handshake:
    // - if this pixel is discarded, always consume it
    // - if this pixel is sampled, only consume when output side is ready
    assign s_axis_tready = (!sample_en) || m_axis_tready;

    // A real accepted input beat
    wire in_fire = s_axis_tvalid && s_axis_tready;

    // A real accepted sampled output beat
    wire out_fire = in_fire && sample_en;

    // ----------------------------------------------------------------
    // RGB888 -> Gray8
    // Simple average approximation: gray ~= (R+G+B)/3
    // ----------------------------------------------------------------
    wire [7:0] r = s_axis_tdata[23:16];
    wire [7:0] g = s_axis_tdata[15:8];
    wire [7:0] b = s_axis_tdata[7:0];

    wire [9:0] sum  = {2'b00, r} + {2'b00, g} + {2'b00, b};
    wire [7:0] gray = (sum * 8'd85) >> 8;  // approx divide by 3

    // Output pixel data: replicate gray to RGB
    assign m_axis_tdata  = {gray, gray, gray};
    assign m_axis_tvalid = s_axis_tvalid && sample_en;

    // First output pixel of frame:
    // input SOF pixel is always (0,0), and sample_en is true there
    assign m_axis_tuser = s_axis_tvalid && sample_en && (cur_x == 11'd0) && (cur_y == 11'd0);

    // Last output pixel of each output line:
    // when output x reaches OUT_WIDTH-1
    assign m_axis_tlast = s_axis_tvalid && sample_en && (out_x == (OUT_WIDTH - 1));

    // ----------------------------------------------------------------
    // Coordinate update
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_x  <= 11'd0;
            in_y  <= 11'd0;
            out_x <= 9'd0;
        end else begin
            if (in_fire) begin
                // ----------------------------
                // Update output x counter only on sampled pixels
                // ----------------------------
                if (sample_en) begin
                    if (out_x == (OUT_WIDTH - 1))
                        out_x <= 9'd0;
                    else
                        out_x <= out_x + 9'd1;
                end

                // ----------------------------
                // Update input x/y counters
                // ----------------------------
                if (s_axis_tuser) begin
                    // Start of frame: force current pixel to (0,0)
                    // Next accepted input pixel becomes x=1, y=0
                    in_x <= 11'd1;
                    in_y <= 11'd0;
                    out_x <= 9'd1; // because (0,0) is sampled and becomes first output pixel
                end
                else if (s_axis_tlast) begin
                    // End of line
                    in_x <= 11'd0;

                    if (in_y == (IN_HEIGHT - 1))
                        in_y <= 11'd0;
                    else
                        in_y <= in_y + 11'd1;

                    // At end of every input line, if sampled pixels were exactly 320,
                    // out_x should already have wrapped to 0 on the last sampled pixel.
                    // So no extra out_x handling needed here.
                end
                else begin
                    in_x <= in_x + 11'd1;
                end
            end
        end
    end

endmodule