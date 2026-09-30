`timescale 1ns / 1ps
`include "dronet_params.vh"

module dronet_compute_engine #(
    parameter integer PE_COUNT = 8,
    parameter integer CONV1_SHIFT = 8,
    parameter integer CONV2_SHIFT = 8,
    parameter integer CONV3_SHIFT = 8,
    parameter integer CONV4_SHIFT = 8,
    parameter integer CONV5_SHIFT = 8,
    parameter integer DET_SHIFT   = 8
) (
    input  wire                            clk,
    input  wire                            rst_n,
    input  wire                            soft_reset,
    input  wire                            start,
    input  wire                            test_pattern_enable,
    output reg  [`DRONET_FRAME_ADDR_W-1:0] frame_rd_addr,
    input  wire [7:0]                      frame_rd_data,
    output reg                             raw_wr_en,
    output reg  [`DRONET_RAW_ADDR_W-1:0]   raw_wr_addr,
    output reg  signed [7:0]               raw_wr_data,
    output reg                             busy,
    output reg                             done_pulse,
    output reg  [3:0]                      current_step,
    output reg  [23:0]                     cycles_left,
    output reg  [7:0]                      dbg_last_frame_byte,
    // PS weight/bias load ports
    input  wire                            w_load_en,
    input  wire [13:0]                     w_load_addr,
    input  wire signed [7:0]               w_load_data,
    input  wire                            b_load_en,
    input  wire [6:0]                      b_load_addr,
    input  wire signed [31:0]              b_load_data
);

    localparam [3:0] ST_IDLE         = 4'd0;
    localparam [3:0] ST_SYNTH        = 4'd1;
    localparam [3:0] ST_LOAD         = 4'd2;
    localparam [3:0] ST_LAYER_PREP   = 4'd3;
    localparam [3:0] ST_3X3_ROW_CLR  = 4'd4;
    localparam [3:0] ST_3X3_IC_PREP  = 4'd5;
    localparam [3:0] ST_3X3_LOAD     = 4'd6;
    localparam [3:0] ST_3X3_SLIDE    = 4'd7;
    localparam [3:0] ST_3X3_WRITE    = 4'd8;
    localparam [3:0] ST_1X1_INIT     = 4'd9;
    localparam [3:0] ST_1X1_MAC      = 4'd10;
    localparam [3:0] ST_1X1_WRITE    = 4'd11;
    localparam [3:0] ST_POOL_INIT    = 4'd12;
    localparam [3:0] ST_POOL_ACC     = 4'd13;
    localparam [3:0] ST_POOL_WRITE   = 4'd14;
    localparam [3:0] ST_DONE         = 4'd15;

    // Unified weight ROM layout
    localparam integer W_TOTAL     = 11144;
    localparam integer W_OFF_CONV1 = 0;
    localparam integer W_OFF_CONV2 = 72;
    localparam integer W_OFF_CONV3 = 1224;
    localparam integer W_OFF_CONV4 = 5832;
    localparam integer W_OFF_CONV5 = 6344;
    localparam integer W_OFF_DET   = 10952;

    // Unified bias layout
    localparam integer B_TOTAL     = 110;
    localparam integer B_OFF_CONV1 = 0;
    localparam integer B_OFF_CONV2 = 8;
    localparam integer B_OFF_CONV3 = 24;
    localparam integer B_OFF_CONV4 = 56;
    localparam integer B_OFF_CONV5 = 72;
    localparam integer B_OFF_DET   = 104;

    localparam integer ACT_MEM_DEPTH = 115200;
    localparam integer LB_MAX_WIDTH  = `DRONET_INPUT_W + 2;
    localparam integer TEST_CELL_X   = 10;
    localparam integer TEST_CELL_Y   = 5;
    localparam integer TEST_CELL_IDX = (TEST_CELL_Y * `DRONET_GRID_W) + TEST_CELL_X;

    reg [3:0] state;

    (* ram_style = "block" *) reg signed [7:0] act_a [0:ACT_MEM_DEPTH-1];
    (* ram_style = "block" *) reg signed [7:0] act_b [0:ACT_MEM_DEPTH-1];

    // Unified weight ROM — dual-port BRAM (port A: PS write, port B: engine read)
    (* ram_style = "block" *) reg signed [7:0] w_rom [0:W_TOTAL-1];

    // Unified bias array — small, registers are fine
    reg signed [31:0] b_rom [0:B_TOTAL-1];

    reg signed [31:0] acc_pe  [0:PE_COUNT-1];
    // acc_row is now declared per-PE inside the GEN_PES generate block
    // (distributed RAM). ST_3X3_WRITE reads a variable PE via this MUX:
    reg signed [31:0] acc_row_rd;

    reg [16:0] load_idx;
    reg [10:0] synth_idx;
    reg [5:0]  oc_tile_base;
    reg [5:0]  ic_idx;
    reg [8:0]  ox_idx;
    reg [6:0]  oy_idx;
    reg [3:0]  write_pe_idx;
    reg [8:0]  row_clr_idx;
    reg [8:0]  slide_x_idx;
    reg [8:0]  load_col_idx;
    reg [1:0]  row_load_sel;
    reg [1:0]  pool_idx;
    reg        mem_rd_phase;
    reg [3:0]  preload_pe_idx;
    reg [3:0]  preload_tap_idx;
    reg        init_acc_pending;
    reg [16:0] act_rd_addr;
    reg signed [7:0] act_a_q, act_b_q;
    reg        act_rd_in_bounds;

    reg signed [7:0]  conv1x1_src;
    reg signed [7:0]  lb_load_data;
    reg signed [7:0]  pool_max;
    reg signed [7:0]  pool_val;
    reg signed [7:0]  out_val;
    reg               lb_load_en;

    reg               pe_active [0:PE_COUNT-1];
    reg signed [7:0]  pe1_weight [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w00 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w01 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w02 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w10 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w11 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w12 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w20 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w21 [0:PE_COUNT-1];
    reg signed [7:0]  pe3_w22 [0:PE_COUNT-1];

    wire signed [31:0] pe1_acc_next [0:PE_COUNT-1];
    wire signed [31:0] pe3_psum     [0:PE_COUNT-1];   // 3-cycle pipelined conv output

    // SLIDE pipeline: delay write index/valid to match conv3x3 3-cycle latency
    reg [8:0] slide_x_d1, slide_x_d2, slide_x_d3;
    reg       slide_v_d1, slide_v_d2, slide_v_d3;

    wire [7:0] lb_window_x;
    wire signed [7:0] lb_tap00, lb_tap01, lb_tap02;
    wire signed [7:0] lb_tap10, lb_tap11, lb_tap12;
    wire signed [7:0] lb_tap20, lb_tap21, lb_tap22;

    integer i;
    integer comb_ix, comb_iy, comb_addr_tmp;
    integer seq_addr_tmp;

    // Weight ROM addressing
    reg [13:0] w_base;
    reg [6:0]  b_base;
    reg [2:0]  w_in_ch_shift;
    reg        w_is_conv3x3;
    reg        w_rom_valid;

    wire [5:0]  w_oc;
    wire [13:0] w_oc_ic;
    wire [13:0] w_rom_addr;

    assign w_oc      = oc_tile_base + {2'd0, preload_pe_idx};
    assign w_oc_ic   = ({8'd0, w_oc} << w_in_ch_shift) + {8'd0, ic_idx};
    assign w_rom_addr = w_base + (w_is_conv3x3 ?
                        ((w_oc_ic << 3) + w_oc_ic + {10'd0, preload_tap_idx}) :
                        w_oc_ic);

    reg signed [7:0] w_rom_q;

    // Activation write port (combinational, for BRAM inference)
    reg        act_a_wen, act_b_wen;
    reg [16:0] act_wr_addr;
    reg signed [7:0] act_wr_data;

    assign lb_window_x = slide_x_idx[7:0];

    // ======================== Functions ========================

    function [23:0] cycles_for_step;
        input [3:0] step;
        begin
            case (step)
                `DRONET_STEP_CONV1:  cycles_for_step = 24'd187740;
                `DRONET_STEP_POOL1:  cycles_for_step = 24'd144000;
                `DRONET_STEP_CONV2:  cycles_for_step = 24'd299520;
                `DRONET_STEP_POOL2:  cycles_for_step = 24'd70400;
                `DRONET_STEP_CONV3:  cycles_for_step = 24'd265408;
                `DRONET_STEP_POOL3:  cycles_for_step = 24'd35200;
                `DRONET_STEP_CONV4:  cycles_for_step = 24'd17600;
                `DRONET_STEP_CONV5:  cycles_for_step = 24'd68464;
                `DRONET_STEP_DET:    cycles_for_step = 24'd8800;
                default:             cycles_for_step = 24'd0;
            endcase
        end
    endfunction

    function integer step_in_w;  input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_in_w=160; `DRONET_STEP_POOL1: step_in_w=160;
        `DRONET_STEP_CONV2: step_in_w=80;  `DRONET_STEP_POOL2: step_in_w=80;
        `DRONET_STEP_CONV3: step_in_w=40;  `DRONET_STEP_POOL3: step_in_w=40;
        `DRONET_STEP_CONV4: step_in_w=20;  `DRONET_STEP_CONV5: step_in_w=20;
        `DRONET_STEP_DET:   step_in_w=20;  default: step_in_w=0;
    endcase end endfunction

    function integer step_in_h;  input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_in_h=90;  `DRONET_STEP_POOL1: step_in_h=90;
        `DRONET_STEP_CONV2: step_in_h=45;  `DRONET_STEP_POOL2: step_in_h=45;
        `DRONET_STEP_CONV3: step_in_h=22;  `DRONET_STEP_POOL3: step_in_h=22;
        `DRONET_STEP_CONV4: step_in_h=11;  `DRONET_STEP_CONV5: step_in_h=11;
        `DRONET_STEP_DET:   step_in_h=11;  default: step_in_h=0;
    endcase end endfunction

    function integer step_out_w; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_out_w=160; `DRONET_STEP_POOL1: step_out_w=80;
        `DRONET_STEP_CONV2: step_out_w=80;  `DRONET_STEP_POOL2: step_out_w=40;
        `DRONET_STEP_CONV3: step_out_w=40;  `DRONET_STEP_POOL3: step_out_w=20;
        `DRONET_STEP_CONV4: step_out_w=20;  `DRONET_STEP_CONV5: step_out_w=20;
        `DRONET_STEP_DET:   step_out_w=20;  default: step_out_w=0;
    endcase end endfunction

    function integer step_out_h; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_out_h=90;  `DRONET_STEP_POOL1: step_out_h=45;
        `DRONET_STEP_CONV2: step_out_h=45;  `DRONET_STEP_POOL2: step_out_h=22;
        `DRONET_STEP_CONV3: step_out_h=22;  `DRONET_STEP_POOL3: step_out_h=11;
        `DRONET_STEP_CONV4: step_out_h=11;  `DRONET_STEP_CONV5: step_out_h=11;
        `DRONET_STEP_DET:   step_out_h=11;  default: step_out_h=0;
    endcase end endfunction

    function integer step_in_ch; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_in_ch=1;   `DRONET_STEP_POOL1: step_in_ch=8;
        `DRONET_STEP_CONV2: step_in_ch=8;   `DRONET_STEP_POOL2: step_in_ch=16;
        `DRONET_STEP_CONV3: step_in_ch=16;  `DRONET_STEP_POOL3: step_in_ch=32;
        `DRONET_STEP_CONV4: step_in_ch=32;  `DRONET_STEP_CONV5: step_in_ch=16;
        `DRONET_STEP_DET:   step_in_ch=32;  default: step_in_ch=0;
    endcase end endfunction

    function integer step_out_ch; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1: step_out_ch=8;  `DRONET_STEP_POOL1: step_out_ch=8;
        `DRONET_STEP_CONV2: step_out_ch=16; `DRONET_STEP_POOL2: step_out_ch=16;
        `DRONET_STEP_CONV3: step_out_ch=32; `DRONET_STEP_POOL3: step_out_ch=32;
        `DRONET_STEP_CONV4: step_out_ch=16; `DRONET_STEP_CONV5: step_out_ch=32;
        `DRONET_STEP_DET:   step_out_ch=6;  default: step_out_ch=0;
    endcase end endfunction

    function [0:0] step_is_pool; input [3:0] step; begin case(step)
        `DRONET_STEP_POOL1, `DRONET_STEP_POOL2, `DRONET_STEP_POOL3: step_is_pool=1; default: step_is_pool=0;
    endcase end endfunction

    function [0:0] step_is_conv3; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1, `DRONET_STEP_CONV2, `DRONET_STEP_CONV3, `DRONET_STEP_CONV5: step_is_conv3=1; default: step_is_conv3=0;
    endcase end endfunction

    function [0:0] step_relu_en; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1, `DRONET_STEP_CONV2, `DRONET_STEP_CONV3, `DRONET_STEP_CONV4, `DRONET_STEP_CONV5: step_relu_en=1; default: step_relu_en=0;
    endcase end endfunction

    function [0:0] step_src_a; input [3:0] step; begin case(step)
        `DRONET_STEP_CONV1, `DRONET_STEP_CONV2, `DRONET_STEP_CONV3, `DRONET_STEP_CONV4, `DRONET_STEP_DET: step_src_a=1; default: step_src_a=0;
    endcase end endfunction

    function [0:0] step_dst_a; input [3:0] step; begin case(step)
        `DRONET_STEP_POOL1, `DRONET_STEP_POOL2, `DRONET_STEP_POOL3, `DRONET_STEP_CONV5: step_dst_a=1; default: step_dst_a=0;
    endcase end endfunction

    function integer act_addr;
        input integer ch, y, x, h, w;
        begin act_addr = ((ch * h) + y) * w + x; end
    endfunction

    function integer raw_addr_calc;
        input integer ch, y, x;
        begin raw_addr_calc = (ch * `DRONET_CELLS) + (y * `DRONET_GRID_W) + x; end
    endfunction


    function signed [7:0] quantize_acc;
        input signed [31:0] value; input integer shift; input relu_en;
        reg signed [31:0] tmp;
        begin
            tmp = value;
            if (shift > 0) tmp = tmp >>> shift;
            if (relu_en && (tmp < 0)) tmp = 0;
            if (tmp > 127)       quantize_acc = 8'sd127;
            else if (tmp < -128) quantize_acc = -8'sd128;
            else                 quantize_acc = tmp[7:0];
        end
    endfunction

    // ======================== Combinational: act read address ========================
    always @(*) begin
        conv1x1_src = 8'sd0; lb_load_data = 8'sd0;
        act_rd_addr = 17'd0; act_rd_in_bounds = 1'b0; lb_load_en = 1'b0;
        comb_ix = 0; comb_iy = 0; comb_addr_tmp = 0;

        case (state)
            ST_3X3_LOAD: begin
                comb_ix = load_col_idx - 1;
                comb_iy = oy_idx + row_load_sel - 1;
                if ((comb_ix >= 0) && (comb_ix < step_in_w(current_step)) &&
                    (comb_iy >= 0) && (comb_iy < step_in_h(current_step))) begin
                    act_rd_addr = act_addr(ic_idx, comb_iy, comb_ix, step_in_h(current_step), step_in_w(current_step));
                    act_rd_in_bounds = 1'b1;
                end
                lb_load_en = mem_rd_phase;
            end
            ST_1X1_INIT: begin
                act_rd_addr = act_addr(ic_idx, oy_idx, ox_idx, step_in_h(current_step), step_in_w(current_step));
                act_rd_in_bounds = 1'b1;
            end
            ST_POOL_INIT: begin
                act_rd_addr = act_addr(oc_tile_base, oy_idx<<1, ox_idx<<1, step_in_h(current_step), step_in_w(current_step));
                act_rd_in_bounds = 1'b1;
            end
            ST_POOL_ACC: begin
                if (pool_idx < 2'd3) begin
                    act_rd_addr = act_addr(oc_tile_base, (oy_idx<<1)+(((pool_idx+2'd1)>>1)&32'd1), (ox_idx<<1)+((pool_idx+2'd1)&2'd1), step_in_h(current_step), step_in_w(current_step));
                    act_rd_in_bounds = 1'b1;
                end
            end
            default: ;
        endcase

        if (state == ST_1X1_MAC) begin
            if (step_src_a(current_step)) conv1x1_src = act_a_q;
            else                          conv1x1_src = act_b_q;
        end

        if (state == ST_3X3_LOAD && mem_rd_phase && act_rd_in_bounds) begin
            if (step_src_a(current_step)) lb_load_data = act_a_q;
            else                          lb_load_data = act_b_q;
        end
    end

    // ======================== Combinational: act write port ========================
    always @(*) begin
        act_a_wen = 1'b0; act_b_wen = 1'b0;
        act_wr_addr = 17'd0; act_wr_data = 8'sd0;

        case (state)
            ST_LOAD: if (mem_rd_phase) begin
                act_wr_addr = load_idx;
                act_wr_data = $signed({1'b0, frame_rd_data[7:1]});
                act_a_wen = 1'b1;
            end
            ST_3X3_WRITE:
                if ((write_pe_idx < PE_COUNT) && ((oc_tile_base+write_pe_idx) < step_out_ch(current_step))) begin
                    act_wr_data = quantize_acc(
                        acc_row_rd + b_rom[b_base + oc_tile_base + write_pe_idx],
                        (current_step==`DRONET_STEP_CONV1) ? CONV1_SHIFT :
                        (current_step==`DRONET_STEP_CONV2) ? CONV2_SHIFT :
                        (current_step==`DRONET_STEP_CONV3) ? CONV3_SHIFT : CONV5_SHIFT,
                        step_relu_en(current_step));
                    act_wr_addr = act_addr(oc_tile_base+write_pe_idx, oy_idx, ox_idx, step_out_h(current_step), step_out_w(current_step));
                    if (step_dst_a(current_step)) act_a_wen = 1'b1; else act_b_wen = 1'b1;
                end
            ST_1X1_WRITE:
                if ((write_pe_idx < PE_COUNT) && ((oc_tile_base+write_pe_idx) < step_out_ch(current_step))) begin
                    act_wr_data = quantize_acc(acc_pe[write_pe_idx],
                        (current_step==`DRONET_STEP_CONV4) ? CONV4_SHIFT : DET_SHIFT,
                        step_relu_en(current_step));
                    if (current_step != `DRONET_STEP_DET) begin
                        act_wr_addr = act_addr(oc_tile_base+write_pe_idx, oy_idx, ox_idx, step_out_h(current_step), step_out_w(current_step));
                        if (step_dst_a(current_step)) act_a_wen = 1'b1; else act_b_wen = 1'b1;
                    end
                end
            ST_POOL_WRITE: begin
                act_wr_data = pool_max;
                act_wr_addr = act_addr(oc_tile_base, oy_idx, ox_idx, step_out_h(current_step), step_out_w(current_step));
                if (step_dst_a(current_step)) act_a_wen = 1'b1; else act_b_wen = 1'b1;
            end
        endcase
    end

    // ======================== Modules ========================

    dronet_line_buffer_3x3 #(.MAX_WIDTH(LB_MAX_WIDTH), .ADDR_W(8)) u_line_buffer_3x3 (
        .clk(clk), .load_en(lb_load_en), .load_row_sel(row_load_sel),
        .load_addr(load_col_idx[7:0]), .load_data(lb_load_data), .window_x(lb_window_x),
        .tap00(lb_tap00), .tap01(lb_tap01), .tap02(lb_tap02),
        .tap10(lb_tap10), .tap11(lb_tap11), .tap12(lb_tap12),
        .tap20(lb_tap20), .tap21(lb_tap21), .tap22(lb_tap22)
    );

    genvar g;
    generate
        for (g = 0; g < PE_COUNT; g = g + 1) begin : GEN_PES
            // Per-PE accumulator row — distributed RAM (1 write / 1 read).
            // ROW_CLR and SLIDE writes are time-exclusive → single write port.
            (* ram_style = "distributed" *) reg signed [31:0] acc_row [0:`DRONET_INPUT_W-1];

            // ROW_CLR clears; SLIDE accumulates the 3-cycle-delayed conv result.
            // Write index/valid are delayed (slide_x_d3 / slide_v_d3) to match
            // the conv3x3 pipeline latency. Read-modify-write on acc_row[x_d3].
            always @(posedge clk) begin
                if (state == ST_3X3_ROW_CLR)
                    acc_row[row_clr_idx] <= 32'sd0;
                else if (state == ST_3X3_SLIDE && slide_v_d3 && pe_active[g])
                    acc_row[slide_x_d3] <= acc_row[slide_x_d3] + pe3_psum[g];
            end

            dronet_mac_pe u_mac_pe (
                .in_val(conv1x1_src), .weight(pe1_weight[g]),
                .acc_in(acc_pe[g]), .acc_out(pe1_acc_next[g])
            );
            dronet_conv3x3_pe u_conv3x3_pe (
                .clk(clk),
                .px00(lb_tap00), .px01(lb_tap01), .px02(lb_tap02),
                .px10(lb_tap10), .px11(lb_tap11), .px12(lb_tap12),
                .px20(lb_tap20), .px21(lb_tap21), .px22(lb_tap22),
                .wt00(pe3_w00[g]), .wt01(pe3_w01[g]), .wt02(pe3_w02[g]),
                .wt10(pe3_w10[g]), .wt11(pe3_w11[g]), .wt12(pe3_w12[g]),
                .wt20(pe3_w20[g]), .wt21(pe3_w21[g]), .wt22(pe3_w22[g]),
                .psum_out(pe3_psum[g])
            );
        end
    endgenerate

    // SLIDE write-index / valid pipeline (matches conv3x3 3-cycle latency)
    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            slide_x_d1 <= 9'd0; slide_x_d2 <= 9'd0; slide_x_d3 <= 9'd0;
            slide_v_d1 <= 1'b0; slide_v_d2 <= 1'b0; slide_v_d3 <= 1'b0;
        end else begin
            slide_x_d1 <= slide_x_idx;
            slide_x_d2 <= slide_x_d1;
            slide_x_d3 <= slide_x_d2;
            slide_v_d1 <= (state == ST_3X3_SLIDE) && (slide_x_idx < step_out_w(current_step));
            slide_v_d2 <= slide_v_d1;
            slide_v_d3 <= slide_v_d2;
        end
    end

    // ST_3X3_WRITE variable-PE read MUX (8:1 over per-PE acc_row)
    always @(*) begin
        case (write_pe_idx[2:0])
            3'd0:    acc_row_rd = GEN_PES[0].acc_row[ox_idx];
            3'd1:    acc_row_rd = GEN_PES[1].acc_row[ox_idx];
            3'd2:    acc_row_rd = GEN_PES[2].acc_row[ox_idx];
            3'd3:    acc_row_rd = GEN_PES[3].acc_row[ox_idx];
            3'd4:    acc_row_rd = GEN_PES[4].acc_row[ox_idx];
            3'd5:    acc_row_rd = GEN_PES[5].acc_row[ox_idx];
            3'd6:    acc_row_rd = GEN_PES[6].acc_row[ox_idx];
            default: acc_row_rd = GEN_PES[7].acc_row[ox_idx];
        endcase
    end

    // ======================== BRAM read/write ========================

    // act_a: simple dual-port BRAM
    always @(posedge clk) begin
        if (act_a_wen) act_a[act_wr_addr] <= act_wr_data;
        act_a_q <= act_a[act_rd_addr];
    end

    // act_b: simple dual-port BRAM
    always @(posedge clk) begin
        if (act_b_wen) act_b[act_wr_addr] <= act_wr_data;
        act_b_q <= act_b[act_rd_addr];
    end

    // w_rom: dual-port BRAM — port A: PS write, port B: engine read
    always @(posedge clk) begin
        if (w_load_en) w_rom[w_load_addr] <= w_load_data;
        w_rom_q <= w_rom[w_rom_addr];
    end

    // b_rom: PS writable bias registers
    always @(posedge clk) begin
        if (b_load_en) b_rom[b_load_addr] <= b_load_data;
    end

    // ======================== Main FSM ========================
    always @(posedge clk) begin
        if (!rst_n || soft_reset) begin
            state <= ST_IDLE; frame_rd_addr <= 0;
            raw_wr_en <= 0; raw_wr_addr <= 0; raw_wr_data <= 0;
            busy <= 0; done_pulse <= 0;
            current_step <= `DRONET_STEP_CONV1; cycles_left <= 0;
            dbg_last_frame_byte <= 0;
            load_idx <= 0; synth_idx <= 0;
            oc_tile_base <= 0; ic_idx <= 0; ox_idx <= 0; oy_idx <= 0;
            write_pe_idx <= 0; row_clr_idx <= 0; slide_x_idx <= 0;
            load_col_idx <= 0; row_load_sel <= 0; pool_idx <= 0;
            pool_max <= -8'sd128; mem_rd_phase <= 0;
            preload_pe_idx <= 0; preload_tap_idx <= 0; init_acc_pending <= 0;
            w_base <= 0; b_base <= 0; w_in_ch_shift <= 0; w_is_conv3x3 <= 0; w_rom_valid <= 0;
            for (i = 0; i < PE_COUNT; i = i + 1) begin
                acc_pe[i] <= 0; pe_active[i] <= 0; pe1_weight[i] <= 0;
                pe3_w00[i]<=0; pe3_w01[i]<=0; pe3_w02[i]<=0;
                pe3_w10[i]<=0; pe3_w11[i]<=0; pe3_w12[i]<=0;
                pe3_w20[i]<=0; pe3_w21[i]<=0; pe3_w22[i]<=0;
            end
        end else begin
            raw_wr_en <= 1'b0; done_pulse <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 0; frame_rd_addr <= 0; cycles_left <= 0;
                    if (start) begin
                        busy <= 1; mem_rd_phase <= 0;
                        if (test_pattern_enable) begin
                            state <= ST_SYNTH; synth_idx <= 0;
                            current_step <= `DRONET_STEP_DET; cycles_left <= `DRONET_RAW_BYTES;
                        end else begin
                            state <= ST_LOAD; load_idx <= 0; frame_rd_addr <= 0;
                            current_step <= `DRONET_STEP_CONV1;
                        end
                    end
                end

                ST_SYNTH: begin
                    raw_wr_en <= 1; raw_wr_addr <= synth_idx[`DRONET_RAW_ADDR_W-1:0]; raw_wr_data <= 0;
                    if (synth_idx == (4*`DRONET_CELLS)+TEST_CELL_IDX) raw_wr_data <= 8'sd8;
                    else if (synth_idx == (5*`DRONET_CELLS)+TEST_CELL_IDX) raw_wr_data <= 8'sd8;
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (synth_idx == (`DRONET_RAW_BYTES-1)) state <= ST_DONE;
                    else synth_idx <= synth_idx + 1;
                end

                ST_LOAD: begin
                    if (!mem_rd_phase) begin
                        mem_rd_phase <= 1;
                        frame_rd_addr <= {{(`DRONET_FRAME_ADDR_W-1){1'b0}}, 1'b1};
                    end else begin
                        dbg_last_frame_byte <= frame_rd_data;
                        if (load_idx == (`DRONET_FRAME_PIXELS-1)) begin
                            state <= ST_LAYER_PREP; current_step <= `DRONET_STEP_CONV1; mem_rd_phase <= 0;
                        end else begin
                            load_idx <= load_idx + 1;
                            frame_rd_addr <= frame_rd_addr + {{(`DRONET_FRAME_ADDR_W-1){1'b0}}, 1'b1};
                        end
                    end
                end

                ST_LAYER_PREP: begin
                    oc_tile_base<=0; ic_idx<=0; ox_idx<=0; oy_idx<=0;
                    write_pe_idx<=0; row_clr_idx<=0; slide_x_idx<=0;
                    load_col_idx<=0; row_load_sel<=0; pool_idx<=0;
                    pool_max <= -8'sd128; mem_rd_phase <= 0;
                    cycles_left <= cycles_for_step(current_step);

                    case (current_step)
                        `DRONET_STEP_CONV1: begin w_base<=W_OFF_CONV1; b_base<=B_OFF_CONV1; w_in_ch_shift<=0; w_is_conv3x3<=1; end
                        `DRONET_STEP_CONV2: begin w_base<=W_OFF_CONV2; b_base<=B_OFF_CONV2; w_in_ch_shift<=3; w_is_conv3x3<=1; end
                        `DRONET_STEP_CONV3: begin w_base<=W_OFF_CONV3; b_base<=B_OFF_CONV3; w_in_ch_shift<=4; w_is_conv3x3<=1; end
                        `DRONET_STEP_CONV4: begin w_base<=W_OFF_CONV4; b_base<=B_OFF_CONV4; w_in_ch_shift<=5; w_is_conv3x3<=0; end
                        `DRONET_STEP_CONV5: begin w_base<=W_OFF_CONV5; b_base<=B_OFF_CONV5; w_in_ch_shift<=4; w_is_conv3x3<=1; end
                        `DRONET_STEP_DET:   begin w_base<=W_OFF_DET;   b_base<=B_OFF_DET;   w_in_ch_shift<=5; w_is_conv3x3<=0; end
                        default:            begin w_base<=0; b_base<=0; w_in_ch_shift<=0; w_is_conv3x3<=0; end
                    endcase

                    if (step_is_pool(current_step))       state <= ST_POOL_INIT;
                    else if (step_is_conv3(current_step)) state <= ST_3X3_ROW_CLR;
                    else begin preload_pe_idx<=0; init_acc_pending<=1; w_rom_valid<=0; state<=ST_1X1_INIT; end
                end

                // acc_row[*][row_clr_idx] cleared by per-PE always in GEN_PES
                ST_3X3_ROW_CLR: begin
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (row_clr_idx == (step_out_w(current_step)-1)) begin
                        row_clr_idx<=0; ic_idx<=0; row_load_sel<=0; load_col_idx<=0;
                        preload_pe_idx<=0; preload_tap_idx<=0; w_rom_valid<=0;
                        state <= ST_3X3_IC_PREP;
                    end else row_clr_idx <= row_clr_idx + 1;
                end

                // w_rom_valid toggles per weight: 0=present addr, 1=consume
                ST_3X3_IC_PREP: begin
                    row_load_sel <= 2'd0; load_col_idx <= 9'd0; slide_x_idx <= 9'd0;
                    if (!w_rom_valid) begin
                        w_rom_valid <= 1'b1;
                    end else begin
                        if (preload_tap_idx == 4'd0)
                            pe_active[preload_pe_idx] <= ((oc_tile_base+preload_pe_idx) < step_out_ch(current_step));
                        if ((oc_tile_base+preload_pe_idx) < step_out_ch(current_step)) begin
                            case (preload_tap_idx)
                                4'd0: pe3_w00[preload_pe_idx]<=w_rom_q; 4'd1: pe3_w01[preload_pe_idx]<=w_rom_q;
                                4'd2: pe3_w02[preload_pe_idx]<=w_rom_q; 4'd3: pe3_w10[preload_pe_idx]<=w_rom_q;
                                4'd4: pe3_w11[preload_pe_idx]<=w_rom_q; 4'd5: pe3_w12[preload_pe_idx]<=w_rom_q;
                                4'd6: pe3_w20[preload_pe_idx]<=w_rom_q; 4'd7: pe3_w21[preload_pe_idx]<=w_rom_q;
                                default: pe3_w22[preload_pe_idx]<=w_rom_q;
                            endcase
                        end else begin
                            case (preload_tap_idx)
                                4'd0: pe3_w00[preload_pe_idx]<=0; 4'd1: pe3_w01[preload_pe_idx]<=0;
                                4'd2: pe3_w02[preload_pe_idx]<=0; 4'd3: pe3_w10[preload_pe_idx]<=0;
                                4'd4: pe3_w11[preload_pe_idx]<=0; 4'd5: pe3_w12[preload_pe_idx]<=0;
                                4'd6: pe3_w20[preload_pe_idx]<=0; 4'd7: pe3_w21[preload_pe_idx]<=0;
                                default: pe3_w22[preload_pe_idx]<=0;
                            endcase
                        end
                        w_rom_valid <= 1'b0;
                        if (preload_tap_idx == 4'd8) begin
                            preload_tap_idx <= 0;
                            if (preload_pe_idx == (PE_COUNT-1)) begin preload_pe_idx<=0; state<=ST_3X3_LOAD; end
                            else preload_pe_idx <= preload_pe_idx + 1;
                        end else preload_tap_idx <= preload_tap_idx + 1;
                    end
                end

                ST_3X3_LOAD: begin
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (!mem_rd_phase) mem_rd_phase <= 1;
                    else begin
                        dbg_last_frame_byte <= lb_load_data; mem_rd_phase <= 0;
                        if (load_col_idx == (step_in_w(current_step)+1)) begin
                            load_col_idx <= 0;
                            if (row_load_sel == 2) state <= ST_3X3_SLIDE;
                            else row_load_sel <= row_load_sel + 1;
                        end else load_col_idx <= load_col_idx + 1;
                    end
                end

                // acc_row accumulation done by per-PE always in GEN_PES,
                // using slide_x_d3 / slide_v_d3 (conv3x3 3-cycle latency).
                // slide_x_idx runs to out_w-1 (valid inputs), then +3 extra
                // cycles to flush the pipeline before leaving SLIDE.
                ST_3X3_SLIDE: begin
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (slide_x_idx == (step_out_w(current_step)+2)) begin
                        slide_x_idx <= 0;
                        if (ic_idx == (step_in_ch(current_step)-1)) begin
                            ox_idx<=0; write_pe_idx<=0; state<=ST_3X3_WRITE;
                        end else begin
                            ic_idx<=ic_idx+1; preload_pe_idx<=0; preload_tap_idx<=0;
                            w_rom_valid<=0; state<=ST_3X3_IC_PREP;
                        end
                    end else slide_x_idx <= slide_x_idx + 1;
                end

                ST_3X3_WRITE: begin
                    if (write_pe_idx < PE_COUNT) begin
                        if (cycles_left != 0) cycles_left <= cycles_left - 1;
                        if (write_pe_idx == (PE_COUNT-1)) begin
                            write_pe_idx <= 0;
                            if (ox_idx == (step_out_w(current_step)-1)) begin
                                ox_idx <= 0;
                                if ((oc_tile_base+PE_COUNT) < step_out_ch(current_step)) begin
                                    oc_tile_base <= oc_tile_base + PE_COUNT; state <= ST_3X3_ROW_CLR;
                                end else begin
                                    oc_tile_base <= 0;
                                    if (oy_idx == (step_out_h(current_step)-1)) begin
                                        oy_idx<=0; current_step<=current_step+1; state<=ST_LAYER_PREP;
                                    end else begin oy_idx<=oy_idx+1; state<=ST_3X3_ROW_CLR; end
                                end
                            end else ox_idx <= ox_idx + 1;
                        end else write_pe_idx <= write_pe_idx + 1;
                    end
                end

                // w_rom_valid toggles per PE weight: 0=present addr, 1=consume
                ST_1X1_INIT: begin
                    if (!w_rom_valid) begin
                        if (init_acc_pending) begin
                            for (i=0; i<PE_COUNT; i=i+1)
                                if ((oc_tile_base+i) < step_out_ch(current_step))
                                    acc_pe[i] <= b_rom[b_base + oc_tile_base + i];
                                else acc_pe[i] <= 0;
                            write_pe_idx<=0; init_acc_pending<=0;
                        end
                        w_rom_valid <= 1'b1;
                    end else begin
                        if ((oc_tile_base+preload_pe_idx) < step_out_ch(current_step)) begin
                            pe_active[preload_pe_idx]<=1; pe1_weight[preload_pe_idx]<=w_rom_q;
                        end else begin
                            pe_active[preload_pe_idx]<=0; pe1_weight[preload_pe_idx]<=0;
                        end
                        w_rom_valid <= 1'b0;
                        if (preload_pe_idx==(PE_COUNT-1)) begin preload_pe_idx<=0; state<=ST_1X1_MAC; end
                        else preload_pe_idx <= preload_pe_idx + 1;
                    end
                end

                ST_1X1_MAC: begin
                    dbg_last_frame_byte <= conv1x1_src;
                    for (i=0; i<PE_COUNT; i=i+1) if (pe_active[i]) acc_pe[i] <= pe1_acc_next[i];
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (ic_idx == (step_in_ch(current_step)-1)) begin ic_idx<=0; state<=ST_1X1_WRITE; end
                    else begin ic_idx<=ic_idx+1; preload_pe_idx<=0; init_acc_pending<=0; w_rom_valid<=0; state<=ST_1X1_INIT; end
                end

                ST_1X1_WRITE: begin
                    if (write_pe_idx < PE_COUNT) begin
                        if ((oc_tile_base+write_pe_idx) < step_out_ch(current_step)) begin
                            if (current_step == `DRONET_STEP_DET) begin
                                raw_wr_en<=1; raw_wr_addr<=raw_addr_calc(oc_tile_base+write_pe_idx, oy_idx, ox_idx);
                                raw_wr_data <= act_wr_data;
                            end
                        end
                        if (cycles_left != 0) cycles_left <= cycles_left - 1;
                        if (write_pe_idx == (PE_COUNT-1)) begin
                            write_pe_idx <= 0;
                            if ((oc_tile_base+PE_COUNT) < step_out_ch(current_step)) begin
                                oc_tile_base<=oc_tile_base+PE_COUNT; ic_idx<=0; preload_pe_idx<=0;
                                init_acc_pending<=1; w_rom_valid<=0; state<=ST_1X1_INIT;
                            end else begin
                                oc_tile_base <= 0;
                                if (ox_idx == (step_out_w(current_step)-1)) begin
                                    ox_idx <= 0;
                                    if (oy_idx == (step_out_h(current_step)-1)) begin
                                        oy_idx<=0;
                                        if (current_step==`DRONET_STEP_DET) state<=ST_DONE;
                                        else begin current_step<=current_step+1; state<=ST_LAYER_PREP; end
                                    end else begin oy_idx<=oy_idx+1; ic_idx<=0; preload_pe_idx<=0; init_acc_pending<=1; w_rom_valid<=0; state<=ST_1X1_INIT; end
                                end else begin ox_idx<=ox_idx+1; ic_idx<=0; preload_pe_idx<=0; init_acc_pending<=1; w_rom_valid<=0; state<=ST_1X1_INIT; end
                            end
                        end else write_pe_idx <= write_pe_idx + 1;
                    end
                end

                ST_POOL_INIT: begin pool_idx<=0; pool_max<=-8'sd128; state<=ST_POOL_ACC; end

                ST_POOL_ACC: begin
                    if (step_src_a(current_step)) begin pool_val = act_a_q; end
                    else                          begin pool_val = act_b_q; end
                    if (pool_val > pool_max) pool_max <= pool_val;
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (pool_idx==3) state<=ST_POOL_WRITE; else pool_idx<=pool_idx+1;
                end

                ST_POOL_WRITE: begin
                    if (cycles_left != 0) cycles_left <= cycles_left - 1;
                    if (oc_tile_base == (step_out_ch(current_step)-1)) begin
                        oc_tile_base <= 0;
                        if (ox_idx == (step_out_w(current_step)-1)) begin
                            ox_idx <= 0;
                            if (oy_idx == (step_out_h(current_step)-1)) begin
                                oy_idx<=0; current_step<=current_step+1; state<=ST_LAYER_PREP;
                            end else begin oy_idx<=oy_idx+1; state<=ST_POOL_INIT; end
                        end else begin ox_idx<=ox_idx+1; state<=ST_POOL_INIT; end
                    end else begin oc_tile_base<=oc_tile_base+1; state<=ST_POOL_INIT; end
                end

                ST_DONE: begin busy<=0; done_pulse<=1; state<=ST_IDLE; cycles_left<=0; end
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
