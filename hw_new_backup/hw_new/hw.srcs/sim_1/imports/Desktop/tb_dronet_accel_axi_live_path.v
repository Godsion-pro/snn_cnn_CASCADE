`timescale 1ns / 1ps
`include "dronet_params.vh"

module tb_dronet_accel_axi_live_path;

    // ============================================================
    // Clock / Reset
    // ============================================================
    reg s_axi_aclk  = 1'b0;   // AXI-Lite / CNN domain
    reg s_axis_aclk = 1'b0;   // pixel stream domain

    reg s_axi_aresetn  = 1'b0;
    reg s_axis_aresetn = 1'b0;

    // 100 MHz AXI/CNN clock
    always #5 s_axi_aclk = ~s_axi_aclk;

    // 150 MHz pixel clock
    always #3.333 s_axis_aclk = ~s_axis_aclk;

    // ============================================================
    // Address map
    // ============================================================
    localparam [11:0] ADDR_CONTROL = 12'h000;
    localparam [11:0] ADDR_TRACK   = 12'h004;
    localparam [11:0] ADDR_STATUS0 = 12'h008;
    localparam [11:0] ADDR_STATUS1 = 12'h00C;
    localparam [11:0] ADDR_DEBUG0  = 12'h010;
    localparam [11:0] ADDR_RAW_BASE = 12'h100;

    localparam integer FRAME_W = 160;
    localparam integer FRAME_H = 90;
    localparam integer FRAME_PIXELS = FRAME_W * FRAME_H;   // 14400

    localparam integer RAW_BYTES = 1320;
    localparam integer RAW_WORDS = RAW_BYTES / 4;          // 330

    // CONTROL bits
    localparam [31:0] CTL_SW_START = 32'h0000_0001;
    localparam [31:0] CTL_RESET    = 32'h0000_0002;
    localparam [31:0] CTL_AUTO     = 32'h0000_0004;
    localparam [31:0] CTL_FORCE    = 32'h0000_0008;
    localparam [31:0] CTL_TESTPAT  = 32'h0000_0010;
    localparam [31:0] CTL_CLRDONE  = 32'h0000_0100;
    localparam [31:0] CTL_CLRFRM   = 32'h0000_0200;

    // ============================================================
    // AXI-Stream pixel input
    // ============================================================
    reg  [7:0] s_pixel_tdata  = 8'd0;
    reg        s_pixel_tvalid = 1'b0;
    reg        s_pixel_tuser  = 1'b0;
    wire       s_pixel_tready;

    // ============================================================
    // Misc
    // ============================================================
    reg  flying_obj_flag = 1'b0;
    wire irq_frame_done;
    wire irq_infer_done;

    // ============================================================
    // AXI-Lite
    // ============================================================
    reg  [11:0] s_axi_awaddr  = 12'd0;
    reg         s_axi_awvalid = 1'b0;
    wire        s_axi_awready;

    reg  [31:0] s_axi_wdata   = 32'd0;
    reg  [3:0]  s_axi_wstrb   = 4'hF;
    reg         s_axi_wvalid  = 1'b0;
    wire        s_axi_wready;

    wire [1:0]  s_axi_bresp;
    wire        s_axi_bvalid;
    reg         s_axi_bready  = 1'b0;

    reg  [11:0] s_axi_araddr  = 12'd0;
    reg         s_axi_arvalid = 1'b0;
    wire        s_axi_arready;

    wire [31:0] s_axi_rdata;
    wire [1:0]  s_axi_rresp;
    wire        s_axi_rvalid;
    reg         s_axi_rready  = 1'b0;

    // ============================================================
    // DUT
    // ============================================================
    dronet_accel_axi #(
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
        .s_axi_aclk     (s_axi_aclk),
        .s_axi_aresetn  (s_axi_aresetn),

        .s_axis_aclk    (s_axis_aclk),
        .s_axis_aresetn (s_axis_aresetn),

        .s_pixel_tdata  (s_pixel_tdata),
        .s_pixel_tvalid (s_pixel_tvalid),
        .s_pixel_tuser  (s_pixel_tuser),
        .s_pixel_tready (s_pixel_tready),

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

    // ============================================================
    // AXI-Lite write task
    // ============================================================
    task axi_write;
        input [11:0] addr;
        input [31:0] data;
        begin
            @(posedge s_axi_aclk);

            s_axi_awaddr  <= addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wstrb   <= 4'hF;
            s_axi_wvalid  <= 1'b1;
            s_axi_bready  <= 1'b1;

            wait (s_axi_awready && s_axi_wready);
            @(posedge s_axi_aclk);

            s_axi_awvalid <= 1'b0;
            s_axi_wvalid  <= 1'b0;

            wait (s_axi_bvalid);
            @(posedge s_axi_aclk);

            s_axi_bready <= 1'b0;
        end
    endtask

    // ============================================================
    // AXI-Lite read task
    // ============================================================
    task axi_read;
        input  [11:0] addr;
        output [31:0] data;
        begin
            @(posedge s_axi_aclk);

            s_axi_araddr  <= addr;
            s_axi_arvalid <= 1'b1;
            s_axi_rready  <= 1'b0;

            wait (s_axi_arready);
            @(posedge s_axi_aclk);

            s_axi_arvalid <= 1'b0;
            s_axi_rready  <= 1'b1;

            wait (s_axi_rvalid);
            data = s_axi_rdata;

            @(posedge s_axi_aclk);
            s_axi_rready <= 1'b0;
        end
    endtask

    // ============================================================
    // Pixel generator
    //   실제 카메라 비슷하게 0이 아닌 160x90 grayscale frame 생성.
    //   첫 픽셀에서만 tuser=1.
    // ============================================================
    task send_one_frame;
        integer i;
        integer x;
        integer y;
        reg [7:0] pix;
        begin
            $display("[TB] sending one 160x90 frame...");

            @(posedge s_axis_aclk);

            for (i = 0; i < FRAME_PIXELS; i = i + 1) begin
                x = i % FRAME_W;
                y = i / FRAME_W;

                // 0만 들어가는 상황을 피하려고 gradient + pattern 사용
                pix = (8'h40 + x[7:0] + (y[7:0] << 1) + (i[7:0] ^ 8'h5A));

                if (pix == 8'h00) begin
                    pix = 8'h7F;
                end

                s_pixel_tdata  <= pix;
                s_pixel_tvalid <= 1'b1;
                s_pixel_tuser  <= (i == 0);

                wait (s_pixel_tready);
                @(posedge s_axis_aclk);
            end

            s_pixel_tdata  <= 8'd0;
            s_pixel_tvalid <= 1'b0;
            s_pixel_tuser  <= 1'b0;

            $display("[TB] frame send done.");
        end
    endtask

    // ============================================================
    // RAW readback checker
    // ============================================================
    task read_raw_and_check;
        integer w;
        integer b;
        integer byte_idx;
        integer nonzero;
        integer total;
        integer min_v;
        integer max_v;
        reg [31:0] word;
        reg [7:0] byte_val;
        begin
            nonzero = 0;
            total   = 0;
            min_v   = 255;
            max_v   = 0;

            $display("[TB] reading RAW buffer through AXI-Lite...");

            for (w = 0; w < RAW_WORDS; w = w + 1) begin
                axi_read(ADDR_RAW_BASE + (w * 4), word);

                if (w < 8) begin
                    $display("[RAW_WORD] addr=0x%03h word=0x%08h",
                             ADDR_RAW_BASE + (w * 4), word);
                end

                for (b = 0; b < 4; b = b + 1) begin
                    byte_idx = w * 4 + b;

                    if (byte_idx < RAW_BYTES) begin
                        byte_val = (word >> (8 * b)) & 8'hFF;

                        total = total + 1;

                        if (byte_val != 8'h00) begin
                            nonzero = nonzero + 1;
                        end

                        if (byte_val < min_v) begin
                            min_v = byte_val;
                        end

                        if (byte_val > max_v) begin
                            max_v = byte_val;
                        end
                    end
                end
            end

            $display("==== RAW READBACK RESULT ====");
            $display("total=%0d nonzero=%0d min=0x%02h max=0x%02h",
                     total, nonzero, min_v[7:0], max_v[7:0]);

            if (nonzero == 0) begin
                $display("RESULT: FAIL - wrapper AXI-Lite RAW readback is all zero");
            end else begin
                $display("RESULT: PASS - wrapper AXI-Lite RAW readback has nonzero data");
            end
        end
    endtask

    // ============================================================
    // Timeout
    // ============================================================
    initial begin
        repeat (20000000) @(posedge s_axi_aclk);
        $display("TIMEOUT");
        $display("STATUS: simulation did not finish in time.");
        $finish;
    end

    // ============================================================
    // Main stimulus
    // ============================================================
    reg [31:0] st0;
    reg [31:0] st1;
    reg [31:0] dbg0;

    initial begin
        $display("==== tb_dronet_accel_axi_live_path start ====");

        // init AXI signals
        s_axi_awaddr  = 12'd0;
        s_axi_awvalid = 1'b0;
        s_axi_wdata   = 32'd0;
        s_axi_wstrb   = 4'hF;
        s_axi_wvalid  = 1'b0;
        s_axi_bready  = 1'b0;

        s_axi_araddr  = 12'd0;
        s_axi_arvalid = 1'b0;
        s_axi_rready  = 1'b0;

        s_pixel_tdata  = 8'd0;
        s_pixel_tvalid = 1'b0;
        s_pixel_tuser  = 1'b0;

        flying_obj_flag = 1'b0;

        // reset
        s_axi_aresetn  = 1'b0;
        s_axis_aresetn = 1'b0;

        repeat (30) @(posedge s_axi_aclk);

        s_axi_aresetn  = 1'b1;
        s_axis_aresetn = 1'b1;

        repeat (30) @(posedge s_axi_aclk);

        $display("[TB] reset released.");

        // Soft reset once
        axi_write(ADDR_CONTROL, CTL_RESET);
        repeat (20) @(posedge s_axi_aclk);

        // Clear reset command, enable AUTO + FORCE, disable TESTPAT
        // AUTO=1, FORCE=1, TESTPAT=0
        axi_write(ADDR_CONTROL, CTL_AUTO | CTL_FORCE);

        axi_read(ADDR_CONTROL, st0);
        $display("[TB] CONTROL readback = 0x%08h", st0);

        // Frame send and frame_done wait
        fork
            begin
                send_one_frame();
            end

            begin
                @(posedge irq_frame_done);
                $display("[TB] irq_frame_done observed.");
            end
        join

        // Wait for CNN inference done
        $display("[TB] waiting irq_infer_done...");
        @(posedge irq_infer_done);
        $display("[TB] irq_infer_done observed.");

        repeat (50) @(posedge s_axi_aclk);

        axi_read(ADDR_STATUS0, st0);
        axi_read(ADDR_STATUS1, st1);
        axi_read(ADDR_DEBUG0,  dbg0);

        $display("[TB] STATUS0=0x%08h STATUS1=0x%08h DEBUG0=0x%08h",
                 st0, st1, dbg0);

        $display("[TB] BUSY=%0d DONE=%0d FRAME=%0d RAW=%0d step=%0d left=%0d",
                 st0[0], st0[1], st0[2], st0[7],
                 dbg0[27:24], dbg0[23:0]);

        read_raw_and_check();

        $display("==== tb_dronet_accel_axi_live_path end ====");
        #100;
        $finish;
    end

endmodule