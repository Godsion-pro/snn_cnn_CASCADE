module axis_gray8_to_rgb24 (
    input  wire        aclk,
    input  wire        aresetn,

    // Slave AXI4-Stream input (Gray8)
    input  wire [7:0]  s_axis_tdata,
    input  wire [0:0]  s_axis_tkeep,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,

    // Master AXI4-Stream output (RGB24)
    output wire [23:0] m_axis_tdata,
    output wire [2:0]  m_axis_tkeep,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,
    output wire        m_axis_tlast
);

    reg [23:0] out_tdata_r;
    reg        out_tvalid_r;
    reg        out_tuser_r;
    reg        out_tlast_r;

    wire out_fire;
    wire in_fire;

    assign out_fire = out_tvalid_r && m_axis_tready;
    assign s_axis_tready = aresetn && (!out_tvalid_r || m_axis_tready);
    assign in_fire = s_axis_tvalid && s_axis_tready;

    assign m_axis_tdata  = out_tdata_r;
    assign m_axis_tkeep  = 3'b111;
    assign m_axis_tvalid = out_tvalid_r;
    assign m_axis_tuser  = out_tuser_r;
    assign m_axis_tlast  = out_tlast_r;

    always @(posedge aclk) begin
        if (!aresetn) begin
            out_tdata_r  <= 24'd0;
            out_tvalid_r <= 1'b0;
            out_tuser_r  <= 1'b0;
            out_tlast_r  <= 1'b0;
        end else begin
            if (out_fire) begin
                out_tvalid_r <= 1'b0;
            end

            if (in_fire) begin
                out_tdata_r  <= {s_axis_tdata, s_axis_tdata, s_axis_tdata};
                out_tvalid_r <= s_axis_tkeep[0];
                out_tuser_r  <= s_axis_tuser;
                out_tlast_r  <= s_axis_tlast;
            end
        end
    end

endmodule