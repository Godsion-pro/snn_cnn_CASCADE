`ifndef DRONET_PARAMS_VH
`define DRONET_PARAMS_VH

`define DRONET_INPUT_W                160
`define DRONET_INPUT_H                90
`define DRONET_FRAME_PIXELS           14400
`define DRONET_FRAME_ADDR_W           14

`define DRONET_GRID_W                 20
`define DRONET_GRID_H                 11
`define DRONET_CELLS                  220
`define DRONET_RAW_CHANNELS           6
`define DRONET_RAW_BYTES              1320
`define DRONET_RAW_ADDR_W             11
`define DRONET_RAW_WORDS              330
`define DRONET_RAW_WORD_ADDR_W        9

`define DRONET_STEP_CONV1             4'd0
`define DRONET_STEP_POOL1             4'd1
`define DRONET_STEP_CONV2             4'd2
`define DRONET_STEP_POOL2             4'd3
`define DRONET_STEP_CONV3             4'd4
`define DRONET_STEP_POOL3             4'd5
`define DRONET_STEP_CONV4             4'd6
`define DRONET_STEP_CONV5             4'd7
`define DRONET_STEP_DET               4'd8
`define DRONET_STEP_COUNT             9

`define DRONET_CORE_IDLE              2'd0
`define DRONET_CORE_RUN               2'd1
`define DRONET_CORE_FLUSH             2'd2

`endif
