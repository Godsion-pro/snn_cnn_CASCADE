#include "xparameters.h"
#include "platform.h"
#include "ov5640/OV5640.h"
#include "ov5640/ScuGicInterruptController.h"
#include "ov5640/PS_GPIO.h"
#include "ov5640/AXI_VDMA.h"
#include "ov5640/PS_IIC.h"
#include "MIPI_D_PHY_RX.h"
#include "MIPI_CSI_2_RX.h"
#include "xuartps.h"
#include <cstdint>
#include <xil_io.h> 
#include <xil_types.h>
#include "snn_weights.h"    //snn 가중치
#include "cnn_weights.h"    //cnn 가중치
#include <math.h>
#include <exception>
#include <stdexcept>
#include "xiicps.h"
#include "xil_cache.h"
#include "xiltimer.h"


// ============================================================
// VDMA1 (gray8 원본 저장 + prev frame 읽기)
// ============================================================
#define VDMA1_BASEADDR      XPAR_AXI_VDMA_1_BASEADDR

// VDMA S2MM (Write) register offsets
#define VDMA_S2MM_DMACR         0x30
#define VDMA_S2MM_DMASR         0x34
#define VDMA_S2MM_VSIZE         0xA0
#define VDMA_S2MM_HSIZE         0xA4
#define VDMA_S2MM_FRMDLY_STRIDE 0xA8
#define VDMA_S2MM_START_ADDR1   0xAC
#define VDMA_S2MM_START_ADDR2   0xB0
#define VDMA_S2MM_START_ADDR3   0xB4

// VDMA MM2S (Read) register offsets
#define VDMA_MM2S_DMACR         0x00
#define VDMA_MM2S_DMASR         0x04
#define VDMA_MM2S_VSIZE         0x50
#define VDMA_MM2S_HSIZE         0x54
#define VDMA_MM2S_FRMDLY_STRIDE 0x58
#define VDMA_MM2S_START_ADDR1   0x5C
#define VDMA_MM2S_START_ADDR2   0x60
#define VDMA_MM2S_START_ADDR3   0x64

//snn 관련
#define GPIO0_BASEADDR  XPAR_AXI_GPIO_0_BASEADDR  // 또는 직접 주소
#define GPIO0_CH1       (GPIO0_BASEADDR + 0x00)     // 채널1 (32bit)
#define GPIO0_CH2       (GPIO0_BASEADDR + 0x08)     // 채널2 (16bit)
#define GPIO1_BASEADDR  XPAR_AXI_GPIO_1_BASEADDR
#define GPIO1_DATA      (GPIO1_BASEADDR + 0x00)

// ===== SNN/CNN clock-gating mode select (axi_gpio_3, 1-bit mode_sel) =====
//  mode_sel = 1 -> SNN mode : SNN clk ON,  CNN core clk gated OFF
//  mode_sel = 0 -> CNN mode : CNN core clk ON, SNN clk gated OFF
//  (snn_cnn_sel.v inverts: snn_en = mode_sel, cnn_clk_en = ~mode_sel)
#ifndef GPIO3_BASEADDR
#define GPIO3_BASEADDR  XPAR_AXI_GPIO_3_BASEADDR
#endif
#define GPIO3_DATA      (GPIO3_BASEADDR + 0x00)   // CH1 data
#define CLK_MODE_SNN    1u
#define CLK_MODE_CNN    0u


// VDMA1 해상도/크기 설정
static constexpr u32 VDMA1_W = 160;
static constexpr u32 VDMA1_H = 90;
static constexpr u32 VDMA1_BPP = 1;
static constexpr u32 VDMA1_HSIZE = VDMA1_W * VDMA1_BPP;
static constexpr u32 VDMA1_STRIDE = VDMA1_HSIZE;
static constexpr u32 VDMA1_FRAME_BYTES = VDMA1_STRIDE * VDMA1_H;
static constexpr u32 VDMA_FRMDLY_SHIFT = 24;
static constexpr u32 VDMA1_PREV_FRAME_DELAY = 1;

// VDMA1 메모리 주소
#define MEM_BASE_ADDR_VDMA1 (XPAR_DDR_MEM_BASEADDR + 0x15000000)
static constexpr u32 VDMA1_FB0 = MEM_BASE_ADDR_VDMA1;
static constexpr u32 VDMA1_FB1 = VDMA1_FB0 + VDMA1_FRAME_BYTES;
static constexpr u32 VDMA1_FB2 = VDMA1_FB1 + VDMA1_FRAME_BYTES;

// ============================================================
// VDMA2 (frame diff 결과 저장)
// ============================================================
#define VDMA2_BASEADDR      0x43020000

static constexpr u32 VDMA2_W = 160;
static constexpr u32 VDMA2_H = 90;
static constexpr u32 VDMA2_BPP = 1;
static constexpr u32 VDMA2_HSIZE = VDMA2_W * VDMA2_BPP;
static constexpr u32 VDMA2_STRIDE = VDMA2_HSIZE;
static constexpr u32 VDMA2_FRAME_BYTES = VDMA2_STRIDE * VDMA2_H;

#define MEM_BASE_ADDR_VDMA2 (XPAR_DDR_MEM_BASEADDR + 0x15100000)
static constexpr u32 VDMA2_FB0 = MEM_BASE_ADDR_VDMA2;
static constexpr u32 VDMA2_FB1 = VDMA2_FB0 + VDMA2_FRAME_BYTES;
static constexpr u32 VDMA2_FB2 = VDMA2_FB1 + VDMA2_FRAME_BYTES;

// ============================================================================
//                    [CNN] dronet_accel_axi 후처리 정의
// ============================================================================
#define DRONET_BASEADDR     XPAR_DRONET_ACCEL_AXI_0_BASEADDR
//   ※ BD에서 IP 이름이 다르면 xparameters.h 기준 매크로명으로 교체

// 레지스터 오프셋 (dronet_accel_axi.v 정의)
#define DRONET_CONTROL      0x000
#define DRONET_TRACK        0x004
#define DRONET_STATUS0      0x008
#define DRONET_STATUS1      0x00C
#define DRONET_DEBUG0       0x010
#define DRONET_RAW_BASE     0x100

// Weight/bias load 주소
#define DRONET_W_LOAD       0x800   // [21:8]=addr, [7:0]=data
#define DRONET_B_ADDR       0x804   // [6:0]=bias addr
#define DRONET_B_DATA       0x808   // [31:0]=bias data

// CONTROL 비트
#define DRONET_CTL_SW_START (1u << 0)
#define DRONET_CTL_RESET    (1u << 1)
#define DRONET_CTL_AUTO     (1u << 2)
#define DRONET_CTL_FORCE    (1u << 3)
#define DRONET_CTL_TESTPAT  (1u << 4)
#define DRONET_CTL_CLRDONE  (1u << 8)
#define DRONET_CTL_CLRFRM   (1u << 9)

// TRACK 비트
#define DRONET_TRK_SET      (1u << 0)
#define DRONET_TRK_CLEAR    (1u << 1)

// STATUS0 비트
#define DRONET_ST0_BUSY     (1u << 0)
#define DRONET_ST0_DONE     (1u << 1)
#define DRONET_ST0_FRAME    (1u << 2)
#define DRONET_ST0_RAWVAL   (1u << 7)


// ===== 운영 모드 선택 =====
#define MODE_A_SW_ONESHOT   1   // PS에서 SW_START를 1회씩 주는 진단 모드
#define MODE_C_ALWAYS_ON    3
#define MODE_B_TRACKING     2

// ===== App demo mode selection =====
//   APP_MODE_SNN_CNN_LINK_ONLY:
//     SNN/Top_0_result drives dronet_accel_axi.flying_obj_flag in RTL.
//     AUTO=1 starts CNN on SNN motion/trigger. Once CNN detects, PS sets
//     tracking_active; RTL then keeps running CNN until PS clears tracking.
//   APP_MODE_SNN_ONLY:
//     Only monitors Top_0/GPIO1. CNN and motor control stay disabled.
//   APP_MODE_CNN_MOTOR_ALWAYS:
//     CNN runs every frame and the best CNN detection drives pan/tilt servos.
//   APP_MODE_SNN_CNN_MOTOR_TRACKING:
//     SNN triggers CNN, CNN keeps tracking while detections continue, and the
//     best CNN detection drives pan/tilt servos.
#define APP_MODE_SNN_ONLY      0
#define APP_MODE_SNN_CNN_LINK_ONLY  1
#define APP_MODE_CNN_MOTOR_ALWAYS   2
#define APP_MODE_SNN_CNN_MOTOR_TRACKING  3

#ifndef APP_DEMO_MODE
#define APP_DEMO_MODE APP_MODE_SNN_CNN_MOTOR_TRACKING
#endif

#define SNN_ONLY_DEMO          (APP_DEMO_MODE == APP_MODE_SNN_ONLY)
#define SNN_CNN_LINK_DEMO      (APP_DEMO_MODE == APP_MODE_SNN_CNN_LINK_ONLY)
#define CNN_MOTOR_ALWAYS_DEMO  (APP_DEMO_MODE == APP_MODE_CNN_MOTOR_ALWAYS)
#define SNN_CNN_MOTOR_DEMO     (APP_DEMO_MODE == APP_MODE_SNN_CNN_MOTOR_TRACKING)
#define SNN_TRIGGERED_CNN_DEMO (SNN_CNN_LINK_DEMO || SNN_CNN_MOTOR_DEMO)
#define MOTOR_CONTROL_DEMO     (CNN_MOTOR_ALWAYS_DEMO || SNN_CNN_MOTOR_DEMO)

#if SNN_ONLY_DEMO
#define DRONET_MODE         MODE_B_TRACKING
#elif SNN_TRIGGERED_CNN_DEMO
#define DRONET_MODE         MODE_B_TRACKING
#elif CNN_MOTOR_ALWAYS_DEMO
#define DRONET_MODE         MODE_C_ALWAYS_ON
#else
    #error "Invalid APP_DEMO_MODE"
#endif

#if MOTOR_CONTROL_DEMO
#ifndef MOTOR_GPIO_BASEADDR
#if defined(XPAR_AXI_GPIO_2_BASEADDR)
#define MOTOR_GPIO_BASEADDR XPAR_AXI_GPIO_2_BASEADDR
#elif defined(XPAR_XGPIO_2_BASEADDR)
#define MOTOR_GPIO_BASEADDR XPAR_XGPIO_2_BASEADDR
#endif
#endif

#ifndef MOTOR_GPIO_BASEADDR
    #error "Motor demo mode requires MOTOR_GPIO_BASEADDR. Update the app platform to the motor-integrated XSA, or define MOTOR_GPIO_BASEADDR manually."
#endif
#endif

// UART live diagnostic throttle.
#define DRONET_LIVE_LOG_ENABLE 1
#define DRONET_LOG_EVERY_N  3
#define SNN_STATUS_LOG_EVERY_LOOPS  2000000u

// UART dump of VDMA1 160x90 grayscale frames for PC-side replay.
// Keep disabled during normal live testing; enable only for PC-side replay dumps.
#define DUMP_VDMA1_FRAME_HEX_ONCE    0
#define DUMP_VDMA1_FRAME_HEX_ALL_FB  1
//   1차 검증: MODE_C_ALWAYS_ON
//   검증 끝나면 MODE_B_TRACKING 으로 바꾸고 재빌드


// ===== 모드 B 전용 파라미터 =====
#define LOST_TIMEOUT_SEC        10u  // tracking 중 N초 연속 miss -> off
#define DRONET_TIMER_COUNTS_PER_SECOND ((u64)(COUNTS_PER_SECOND))

// clock-gating: SNN trigger 로 CNN 모드 진입 후 이 시간 안에 detection 을
// 한 번도 못 잡으면(false trigger) SNN 모드로 복귀.
#define CNN_ACQUIRE_TIMEOUT_SEC 3u


// === Grid / Channel layout (model.py / config.py 기준) ===
#define GRID_COLS           20
#define GRID_ROWS           11
#define NUM_CELLS           (GRID_COLS * GRID_ROWS)
#define NUM_CHANNELS        6
#define RAW_BYTES_TOTAL     1320

#define CH_TX  0
#define CH_TY  1
#define CH_TW  2
#define CH_TH  3
#define CH_OBJ 4
#define CH_CLS 5

#define FRAME_W   160
#define FRAME_H   90
#define STRIDE_X_F  ((float)FRAME_W / GRID_COLS)
#define STRIDE_Y_F  ((float)FRAME_H / GRID_ROWS)

// Pan/tilt servo control. The motor-integrated BD maps this to AXI_GPIO_2:
//   CH1[21:0] = pan PWM ticks, CH1[22] = laser enable, CH2[21:0] = tilt ticks.
#define SERVO_TICKS_AT_0DEG    50000u
#define SERVO_TICKS_AT_180DEG  250000u
#define SERVO_TICKS_PER_DEG    ((SERVO_TICKS_AT_180DEG - SERVO_TICKS_AT_0DEG) / 180u)
#define SERVO_LASER_BIT        (1u << 22)

#define PAN_MIN_DEG      0.0f
#define PAN_MAX_DEG      180.0f
#define TILT_MIN_DEG     0.0f
#define TILT_MAX_DEG     180.0f
#define PAN_START_DEG  90.0f
#define TILT_START_DEG  180.0f
#define SERVO_START_DEG 0.0f

#define TRACK_CENTER_X   (FRAME_W / 2)
#define TRACK_CENTER_Y   (FRAME_H / 2)
#define GAIN_PAN         0.14f
#define GAIN_TILT        0.21f
#define LASER_BBOX_THRESHOLD  70

// config.py: ANCHOR = (5, 11) → (h, w)
#define ANCHOR_H        5.0f
#define ANCHOR_W        11.0f

#define TWH_MAX_CLAMP   5.0f   // model.py: tw.clamp(max=5.0)

// 양자화 스케일 (학습/HW 양자화 일치 시 튜닝)
#ifndef CNN_RAW_SCALE
#define CNN_RAW_SCALE   1.984375f
#endif
#define RAW_SCALE       CNN_RAW_SCALE
#define RAW_SCALE_X1000 ((int)(RAW_SCALE * 1000.0f + 0.5f))

// Quantized live model is less confident than the float model; use a lower
// threshold first, then raise it after checking false positives.
#define CONF_THRESHOLD  0.8f
#define NMS_IOU_THRESH  0.45f

#define MAX_DETECTIONS  16

static void dronet_dump_raw_stats(const char *tag);

// ============================================================================
//          [CLKGATE] SNN/CNN clock-gating mode control
// ============================================================================
//  axi_gpio_3 의 1-bit mode_sel 을 써서 SNN/CNN 클럭을 상호배타로 켠다.
//  HW 게이트(clk_en_gate)가 idle 기준으로 안전하게 끊으므로 PS 는 그냥
//  이 비트만 토글하면 됨. SNN 모드로 돌아갈 때마다 resume 시각을 기록해서
//  (게이팅 동안 얼어붙은 옛 result 로) 곧바로 재트리거되는 것을 막는다.
static u32   g_clk_mode          = CLK_MODE_SNN;
static XTime g_snn_resume_time   = 0;   // SNN 모드 진입 시각 (re-arm 용)
static XTime g_cnn_acquire_start = 0;   // CNN 모드 진입 시각 (획득 타임아웃 용)

static void set_clock_mode(u32 mode) {
    g_clk_mode = mode & 0x1u;
    Xil_Out32(GPIO3_DATA, g_clk_mode);
    if (g_clk_mode == CLK_MODE_SNN) {
        XTime_GetTime(&g_snn_resume_time);
    } else {
        XTime_GetTime(&g_cnn_acquire_start);
    }
}

// SNN 모드로 복귀 직후 settle 구간(여기선 200ms) 동안은 트리거 무시
static int snn_rearm_settled(void) {
    XTime now_t;
    XTime_GetTime(&now_t);
    return (now_t - g_snn_resume_time) >= (DRONET_TIMER_COUNTS_PER_SECOND / 5);
}

// ============================================================================
//          [CNN] sigmoid / exp LUT
// ============================================================================
static float g_sigmoid_lut[256];
static float g_exp_lut[256];

static void init_lookup_tables(void) {
    for (int i = 0; i < 256; i++) {
        int8_t v = (int8_t)i;
        float  x = (float)v / RAW_SCALE;
        g_sigmoid_lut[i] = 1.0f / (1.0f + expf(-x));
        float x_cl = (x > TWH_MAX_CLAMP) ? TWH_MAX_CLAMP : x;
        g_exp_lut[i] = expf(x_cl);
    }
    xil_printf("[POST] LUT init done (RAW_SCALE_x1000=%d)\r\n", RAW_SCALE_X1000);
}

static inline float sigmoid_q(int8_t v) { return g_sigmoid_lut[(uint8_t)v]; }
static inline float exp_q    (int8_t v) { return g_exp_lut    [(uint8_t)v]; }


// ============================================================================
//          [CNN] RAW 1320 byte 읽기
// ============================================================================
static int8_t g_raw[RAW_BYTES_TOTAL];

static void dronet_read_raw(void) {
    u32 *raw_w = (u32 *)g_raw;
    for (u32 i = 0; i < RAW_BYTES_TOTAL / 4; i++) {
        raw_w[i] = Xil_In32(DRONET_BASEADDR + DRONET_RAW_BASE + (i << 2));
    }
}

static inline int8_t raw_at(int ch, int cell) {
    return g_raw[ch * NUM_CELLS + cell];
}


// ============================================================================
//          [CNN] Cell 디코딩 (model.py decode_predictions와 동일 수식)
// ============================================================================
typedef struct {
    int   valid;
    int   cell_idx, row, col;
    float cx, cy, bw, bh;
    int   x1, y1, x2, y2;
    float obj_p, cls_p, score;
} detection_t;

static void decode_cell(int cell, detection_t *d) {
    int row = cell / GRID_COLS;
    int col = cell % GRID_COLS;

    float tx = sigmoid_q(raw_at(CH_TX, cell));
    float ty = sigmoid_q(raw_at(CH_TY, cell));
    float ew = exp_q    (raw_at(CH_TW, cell));
    float eh = exp_q    (raw_at(CH_TH, cell));
    float op = sigmoid_q(raw_at(CH_OBJ, cell));
    float cp = sigmoid_q(raw_at(CH_CLS, cell));

    d->cx = ((float)col + tx) * STRIDE_X_F;
    d->cy = ((float)row + ty) * STRIDE_Y_F;
    d->bw = ANCHOR_W * ew;
    d->bh = ANCHOR_H * eh;

    d->cell_idx = cell; d->row = row; d->col = col;
    d->obj_p = op; d->cls_p = cp; d->score = op * cp;

    int x1 = (int)(d->cx - d->bw * 0.5f);
    int y1 = (int)(d->cy - d->bh * 0.5f);
    int x2 = (int)(d->cx + d->bw * 0.5f);
    int y2 = (int)(d->cy + d->bh * 0.5f);
    if (x1 < 0) x1 = 0; if (x1 > FRAME_W-1) x1 = FRAME_W-1;
    if (x2 < 0) x2 = 0; if (x2 > FRAME_W-1) x2 = FRAME_W-1;
    if (y1 < 0) y1 = 0; if (y1 > FRAME_H-1) y1 = FRAME_H-1;
    if (y2 < 0) y2 = 0; if (y2 > FRAME_H-1) y2 = FRAME_H-1;
    d->x1 = x1; d->y1 = y1; d->x2 = x2; d->y2 = y2;
    d->valid = 1;
}


// ============================================================================
//          [CNN] NMS
// ============================================================================
static float box_iou(const detection_t *a, const detection_t *b) {
    int ix1 = (a->x1 > b->x1) ? a->x1 : b->x1;
    int iy1 = (a->y1 > b->y1) ? a->y1 : b->y1;
    int ix2 = (a->x2 < b->x2) ? a->x2 : b->x2;
    int iy2 = (a->y2 < b->y2) ? a->y2 : b->y2;
    int iw = ix2 - ix1; if (iw < 0) iw = 0;
    int ih = iy2 - iy1; if (ih < 0) ih = 0;
    float inter = (float)(iw * ih);
    float aarea = (float)((a->x2 - a->x1) * (a->y2 - a->y1));
    float barea = (float)((b->x2 - b->x1) * (b->y2 - b->y1));
    float uni   = aarea + barea - inter;
    return (uni > 1e-6f) ? (inter / uni) : 0.0f;
}

static void sort_by_score_desc(detection_t *d, int n) {
    for (int i = 0; i < n - 1; i++) {
        int best = i;
        for (int j = i + 1; j < n; j++)
            if (d[j].score > d[best].score) best = j;
        if (best != i) { detection_t t = d[i]; d[i] = d[best]; d[best] = t; }
    }
}

static int run_nms(detection_t *d, int n) {
    sort_by_score_desc(d, n);
    char sup[MAX_DETECTIONS] = {0};
    int kept = 0;
    for (int i = 0; i < n; i++) {
        if (sup[i]) continue;
        d[kept++] = d[i];
        for (int j = i + 1; j < n; j++) {
            if (sup[j]) continue;
            if (box_iou(&d[i], &d[j]) > NMS_IOU_THRESH) sup[j] = 1;
        }
    }
    return kept;
}


// ============================================================================
//          [CNN] 220 cell 스캔 → threshold → NMS
// ============================================================================
static int decode_and_filter(detection_t *out, int max_out) {
    int n = 0;
    for (int cell = 0; cell < NUM_CELLS && n < max_out; cell++) {
        // Quick reject disabled: let CONF_THRESHOLD decide from decoded score.
        detection_t c;
        decode_cell(cell, &c);
        if (c.score >= CONF_THRESHOLD) out[n++] = c;
    }
    if (n > 1) n = run_nms(out, n);
    return n;
}


// ============================================================================
//          [CNN] 결과 출력
// ============================================================================
static void print_detections(const detection_t *dets, int n) {
    if (n == 0) return;
    for (int i = 0; i < n; i++) {
        const detection_t *d = &dets[i];
        int s_int = (int)(d->score * 100);
        int s_dec = (int)(d->score * 10000) % 100;
        xil_printf("[DRONE][%d/%d] conf=%d.%02d%%  "
                   "center=(%3d,%3d)  bbox=(%3d,%3d)-(%3d,%3d)  size=%dx%d  cell=%d(r%d,c%d)\r\n",
                   i + 1, n, s_int, s_dec,
                   (int)d->cx, (int)d->cy,
                   d->x1, d->y1, d->x2, d->y2,
                   (int)d->bw, (int)d->bh,
                   d->cell_idx, d->row, d->col);
    }
}

// ============================================================================
//          [MOTOR] CNN detection -> pan/tilt servo command
// ============================================================================
#if MOTOR_CONTROL_DEMO
#define MOTOR_GPIO_CH1_DATA  0x00
#define MOTOR_GPIO_CH1_TRI   0x04
#define MOTOR_GPIO_CH2_DATA  0x08
#define MOTOR_GPIO_CH2_TRI   0x0C

static float g_motor_pan_deg = PAN_START_DEG;
static float g_motor_tilt_deg = TILT_START_DEG;

static float clamp_float(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void motor_write_servo(float pan_deg, float tilt_deg, int laser_on) {
    pan_deg = clamp_float(pan_deg, PAN_MIN_DEG, PAN_MAX_DEG);
    tilt_deg = clamp_float(tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG);

    u32 pan_ticks = (u32)(SERVO_TICKS_AT_0DEG + pan_deg * (float)SERVO_TICKS_PER_DEG);
    u32 tilt_ticks = (u32)(SERVO_TICKS_AT_0DEG + tilt_deg * (float)SERVO_TICKS_PER_DEG);

    if (laser_on) {
        pan_ticks |= SERVO_LASER_BIT;
    }

    Xil_Out32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH1_DATA, pan_ticks);
    Xil_Out32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH2_DATA, tilt_ticks);
}

static void motor_init(void) {
    Xil_Out32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH1_TRI, 0x00000000u);
    Xil_Out32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH2_TRI, 0x00000000u);

    g_motor_pan_deg = PAN_START_DEG;
    g_motor_tilt_deg = TILT_START_DEG;
    motor_write_servo(g_motor_pan_deg, g_motor_tilt_deg, 0);

    u32 ch1_data = Xil_In32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH1_DATA);
    u32 ch1_tri  = Xil_In32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH1_TRI);
    u32 ch2_data = Xil_In32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH2_DATA);
    u32 ch2_tri  = Xil_In32(MOTOR_GPIO_BASEADDR + MOTOR_GPIO_CH2_TRI);
    u32 expected_center_ticks = SERVO_TICKS_AT_0DEG +
                                (u32)(SERVO_START_DEG * (float)SERVO_TICKS_PER_DEG);

    xil_printf("[MOTOR] initialized: GPIO=0x%08x pan=%d tilt=%d laser=0\r\n",
               (unsigned int)MOTOR_GPIO_BASEADDR,
               (int)g_motor_pan_deg,
               (int)g_motor_tilt_deg);
    xil_printf("[MOTOR] boot-center expected_ticks=%d CH1_DATA=0x%08x CH1_TRI=0x%08x CH2_DATA=0x%08x CH2_TRI=0x%08x\r\n",
               (int)expected_center_ticks,
               (unsigned int)ch1_data,
               (unsigned int)ch1_tri,
               (unsigned int)ch2_data,
               (unsigned int)ch2_tri);
}

static void motor_hold_no_detection(void) {
    motor_write_servo(g_motor_pan_deg, g_motor_tilt_deg, 0);
}

static void motor_follow_detection(const detection_t *d) {
    int center_x = (int)d->cx;
    int center_y = (int)d->cy;
    int error_x = center_x - TRACK_CENTER_X;
    int error_y = center_y - TRACK_CENTER_Y;
    int bbox_w = d->x2 - d->x1;
    int bbox_h = d->y2 - d->y1;
    int bbox_area = bbox_w * bbox_h;
    int laser_on = (bbox_area >= LASER_BBOX_THRESHOLD) ? 1 : 0;

    g_motor_pan_deg = clamp_float(g_motor_pan_deg - (float)error_x * GAIN_PAN,
                                  PAN_MIN_DEG, PAN_MAX_DEG);
    g_motor_tilt_deg = clamp_float(g_motor_tilt_deg + (float)error_y * GAIN_TILT,
                                   TILT_MIN_DEG, TILT_MAX_DEG);

    motor_write_servo(g_motor_pan_deg, g_motor_tilt_deg, laser_on);

    int conf_x100 = (int)(d->score * 10000.0f + 0.5f);
    xil_printf("[MOTOR] target cell=%d conf=%d.%02d%% center=(%d,%d) err=(%d,%d) area=%d laser=%d pan=%d tilt=%d\r\n",
               d->cell_idx,
               conf_x100 / 100,
               conf_x100 % 100,
               center_x,
               center_y,
               error_x,
               error_y,
               bbox_area,
               laser_on,
               (int)g_motor_pan_deg,
               (int)g_motor_tilt_deg);
}
#endif

// ============================================================================
//          [CNN] DroNet CONTROL helper
// ============================================================================
static u32 dronet_ctrl_base(void) {
#if   (DRONET_MODE == MODE_A_SW_ONESHOT)
    return 0;   // AUTO=0, FORCE=0, TESTPAT=0
#elif (DRONET_MODE == MODE_C_ALWAYS_ON)
    return (DRONET_CTL_AUTO | DRONET_CTL_FORCE);
#elif (DRONET_MODE == MODE_B_TRACKING)
    return (DRONET_CTL_AUTO);
#else
    #error "Invalid DRONET_MODE"
#endif
}

// ============================================================================
//          [CNN] Tracking 상태 관리 (모드 B 전용)
// ============================================================================
static int g_tracking_on = 0;
static u32 g_lost_seconds = 0;
static XTime g_last_detection_time = 0;

static u32 tracking_lost_seconds(void) {
    if (!g_tracking_on || g_last_detection_time == 0) {
        return 0;
    }

    XTime now;
    XTime_GetTime(&now);
    return (u32)((now - g_last_detection_time) / DRONET_TIMER_COUNTS_PER_SECOND);
}

static void tracking_set(void) {
    Xil_Out32(DRONET_BASEADDR + DRONET_TRACK, DRONET_TRK_SET);
    g_tracking_on = 1;
    g_lost_seconds = 0;
    XTime_GetTime(&g_last_detection_time);
    xil_printf("[TRACK] ENABLED (SNN-trigger -> first detection acquired)\r\n");
}

static void tracking_clear(void) {
    Xil_Out32(DRONET_BASEADDR + DRONET_TRACK, DRONET_TRK_CLEAR);
    g_tracking_on = 0;
    g_lost_seconds = 0;
    g_last_detection_time = 0;
    xil_printf("[TRACK] DISABLED (lost >= %d sec -> back to SNN wait)\r\n",
               (int)LOST_TIMEOUT_SEC);
#if SNN_TRIGGERED_CNN_DEMO
    // clock-gating: 추적 종료 -> CNN 클럭 끄고 SNN 모드로 복귀
    set_clock_mode(CLK_MODE_SNN);
    xil_printf("[CLKGATE] tracking lost -> SNN mode (CNN clk gated)\r\n");
#endif
}

// 매 추론마다 호출 (모드 C에선 no-op)
static void update_tracking_state(int n_detections) {
#if (DRONET_MODE == MODE_B_TRACKING)
    if (n_detections > 0) {
        if (!g_tracking_on) tracking_set();
        else {
            g_lost_seconds = 0;
            XTime_GetTime(&g_last_detection_time);
        }
    } else if (g_tracking_on) {
        g_lost_seconds = tracking_lost_seconds();
        if (g_lost_seconds >= LOST_TIMEOUT_SEC) tracking_clear();
    }
#else
    (void)n_detections;
#endif
}


// ============================================================================
//          [CNN] DroNet probe / testpattern / init / infer
// ============================================================================
static void dronet_load_weights(void) {
    xil_printf("[CNN] Loading weights (%d bytes)...\r\n", CNN_W_TOTAL);
    for (int i = 0; i < CNN_W_TOTAL; i++) {
        u32 val = ((u32)(u16)i << 8) | ((u8)cnn_w_rom[i]);
        Xil_Out32(DRONET_BASEADDR + DRONET_W_LOAD, val);
    }
    xil_printf("[CNN] Loading biases (%d entries)...\r\n", CNN_B_TOTAL);
    for (int i = 0; i < CNN_B_TOTAL; i++) {
        Xil_Out32(DRONET_BASEADDR + DRONET_B_ADDR, (u32)i);
        Xil_Out32(DRONET_BASEADDR + DRONET_B_DATA, (u32)cnn_b_rom[i]);
    }
    xil_printf("[CNN] Weight/bias load done.\r\n");
}

static int dronet_probe(void) {
    xil_printf("[POST] DroNet AXI-Lite probe...\r\n");
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_AUTO);
    u32 rb = Xil_In32(DRONET_BASEADDR + DRONET_CONTROL);
    xil_printf("       CONTROL writeback = 0x%08x (expect bit[2]=1)\r\n", rb);

    if ((rb & 0x4) == 0) {
        xil_printf("[POST] !! CONTROL writeback FAILED — BD address mapping 의심\r\n");
        return 0;
    }
    u32 st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
    u32 st1 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);
    xil_printf("       STATUS0=0x%08x  STATUS1=0x%08x\r\n", st0, st1);
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);
    return 1;
}

static void dronet_testpattern_oneshot(void) {
    xil_printf("\r\n[POST] === test_pattern oneshot ===\r\n");

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_RESET);
    for (volatile int i = 0; i < 100000; i++) {}

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL,
              DRONET_CTL_AUTO | DRONET_CTL_FORCE | DRONET_CTL_TESTPAT);

    for (volatile int i = 0; i < 30000000; i++) {}   // ~50ms

    u32 st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
    xil_printf("       STATUS0=0x%08x  (DONE=%d RAW=%d)\r\n",
               st0, (st0 >> 1) & 1, (st0 >> 7) & 1);

    if ((st0 & DRONET_ST0_DONE) && (st0 & DRONET_ST0_RAWVAL)) {
        dronet_read_raw();
        dronet_dump_raw_stats("TESTPAT");
        xil_printf("       raw[obj,cell110]=%d  raw[cls,cell110]=%d  raw[obj,cell0]=%d\r\n",
                   (int)(int8_t)raw_at(CH_OBJ, 110),
                   (int)(int8_t)raw_at(CH_CLS, 110),
                   (int)(int8_t)raw_at(CH_OBJ, 0));

        detection_t dets[MAX_DETECTIONS];
        int n = decode_and_filter(dets, MAX_DETECTIONS);
        xil_printf("       decoded %d detection(s)\r\n", n);
        print_detections(dets, n);

        if (n == 0) {
            xil_printf("       No decoded detections from synthetic RAW pattern.\r\n");
        }
    } else {
        xil_printf("[POST] !! test_pattern inference 미완료 (s_axis_aclk 미공급?)\r\n");
    }

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);
    xil_printf("[POST] === test_pattern oneshot end ===\r\n\r\n");
}

static void dronet_init(void) {
    init_lookup_tables();

    // tracking을 일단 clear 상태에서 시작
    Xil_Out32(DRONET_BASEADDR + DRONET_TRACK, DRONET_TRK_CLEAR);
    g_tracking_on = 0;
    g_lost_seconds = 0;
    g_last_detection_time = 0;

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, dronet_ctrl_base());

#if   (DRONET_MODE == MODE_A_SW_ONESHOT)
    xil_printf("[POST] MODE A: SW_START one-shot live inference\r\n");
#elif (DRONET_MODE == MODE_C_ALWAYS_ON)
    xil_printf("[POST] MODE C: ALWAYS-ON (force_infer=1, every frame)\r\n");
#elif (DRONET_MODE == MODE_B_TRACKING)
    xil_printf("[POST] MODE B: SNN-TRIGGER + TRACKING (lost-timeout=%d sec)\r\n",
               (int)LOST_TIMEOUT_SEC);
#endif
    xil_printf("       anchor=(w%d,h%d)  conf>=%d.%02d  IoU>=%d.%02d  scale_x1000=%d\r\n",
               (int)ANCHOR_W, (int)ANCHOR_H,
               (int)(CONF_THRESHOLD*100), (int)(CONF_THRESHOLD*10000)%100,
               (int)(NMS_IOU_THRESH*100), (int)(NMS_IOU_THRESH*10000)%100,
               RAW_SCALE_X1000);
}

static void print_dronet_status(u32 st0, u32 st1, u32 dbg) // 테스트용
{
    xil_printf("[LIVE] ST0=0x%08x BUSY=%d DONE=%d FRAME=%d TRACK=%d PEND=%d RAW=%d  "
               "ST1=0x%08x last_frame=%d raw_frame=%d  DBG=0x%08x step=%d left=%d\r\n",
               st0,
               (st0 >> 0) & 1,
               (st0 >> 1) & 1,
               (st0 >> 2) & 1,
               (st0 >> 3) & 1,
               (st0 >> 4) & 1,
               (st0 >> 7) & 1,
               st1,
               st1 & 0xFFFF,
               (st1 >> 16) & 0xFFFF,
               dbg,
               (dbg >> 24) & 0xF,
               dbg & 0xFFFFFF);
}

static void dronet_dump_raw_stats(const char *tag)
{
    int total_nonzero = 0;
    int total_min = 127;
    int total_max = -128;

    int ch_nonzero[6] = {0};
    int ch_min[6] = {127, 127, 127, 127, 127, 127};
    int ch_max[6] = {-128, -128, -128, -128, -128, -128};

    for (int ch = 0; ch < 6; ch++) {
        for (int cell = 0; cell < NUM_CELLS; cell++) {
            int v = (int)raw_at(ch, cell);

            if (v != 0) {
                ch_nonzero[ch]++;
                total_nonzero++;
            }

            if (v < ch_min[ch]) ch_min[ch] = v;
            if (v > ch_max[ch]) ch_max[ch] = v;
            if (v < total_min) total_min = v;
            if (v > total_max) total_max = v;
        }
    }

    const char *cn[6] = {"tx ", "ty ", "tw ", "th ", "obj", "cls"};

    xil_printf("[%s] RAW total: nonzero=%d/1320 min=%d max=%d\r\n",
               tag, total_nonzero, total_min, total_max);

    for (int ch = 0; ch < 6; ch++) {
        xil_printf("[%s] ch%d(%s): nonzero=%3d/220 min=%4d max=%4d\r\n",
                   tag, ch, cn[ch], ch_nonzero[ch], ch_min[ch], ch_max[ch]);
    }

    xil_printf("[%s] sample: obj0=%d cls0=%d obj110=%d cls110=%d\r\n",
               tag,
               (int)raw_at(CH_OBJ, 0),
               (int)raw_at(CH_CLS, 0),
               (int)raw_at(CH_OBJ, 110),
               (int)raw_at(CH_CLS, 110));
}

static void dump_vdma1_fb_stats(const char *tag, u32 fb_addr)
{
    Xil_DCacheInvalidateRange((INTPTR)fb_addr, VDMA1_FRAME_BYTES);

    volatile u8 *p = (volatile u8 *)fb_addr;

    int nonzero = 0;
    int sum = 0;
    int minv = 255;
    int maxv = 0;

    for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
        int v = p[i];
        if (v != 0) nonzero++;
        sum += v;
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
    }

    xil_printf("[%s] fb=0x%08x nonzero=%d/%d avg=%d min=%d max=%d\r\n",
               tag,
               (unsigned int)fb_addr,
               nonzero,
               VDMA1_FRAME_BYTES,
               sum / VDMA1_FRAME_BYTES,
               minv,
               maxv);

    xil_printf("[%s] first 4x16:\r\n", tag);
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 16; col++) {
            xil_printf("%02x ", p[row * VDMA1_W + col]);
        }
        xil_printf("\r\n");
    }
}

static int vdma2_diff_nonzero(u32 fb_addr)
{
    Xil_DCacheInvalidateRange((INTPTR)fb_addr, VDMA2_FRAME_BYTES);

    volatile u8 *p = (volatile u8 *)fb_addr;
    int nonzero = 0;

    for (u32 i = 0; i < VDMA2_FRAME_BYTES; i++) {
        if (p[i] != 0) nonzero++;
    }

    return nonzero;
}

static void vdma1_pair_diff_stats(const char *tag, u32 a_addr, u32 b_addr)
{
    Xil_DCacheInvalidateRange((INTPTR)a_addr, VDMA1_FRAME_BYTES);
    Xil_DCacheInvalidateRange((INTPTR)b_addr, VDMA1_FRAME_BYTES);

    volatile u8 *a = (volatile u8 *)a_addr;
    volatile u8 *b = (volatile u8 *)b_addr;

    int ge30 = 0;
    int sum_abs = 0;
    int max_abs = 0;

    for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
        int av = a[i];
        int bv = b[i];
        int d = (av >= bv) ? (av - bv) : (bv - av);
        if (d >= 30) ge30++;
        sum_abs += d;
        if (d > max_abs) max_abs = d;
    }

    xil_printf("[VDMA1_SWDIFF] %s ge30=%d/%d avg_abs=%d max_abs=%d\r\n",
               tag,
               ge30,
               VDMA1_FRAME_BYTES,
               sum_abs / VDMA1_FRAME_BYTES,
               max_abs);
}

static void snn_print_diff_stats(void)
{
    int nz0 = vdma2_diff_nonzero(VDMA2_FB0);
    int nz1 = vdma2_diff_nonzero(VDMA2_FB1);
    int nz2 = vdma2_diff_nonzero(VDMA2_FB2);

    xil_printf("[DIFF] nonzero pixels: fb0=%d/%d fb1=%d/%d fb2=%d/%d\r\n",
               nz0, VDMA2_FRAME_BYTES,
               nz1, VDMA2_FRAME_BYTES,
               nz2, VDMA2_FRAME_BYTES);

    vdma1_pair_diff_stats("fb0-fb1", VDMA1_FB0, VDMA1_FB1);
    vdma1_pair_diff_stats("fb1-fb2", VDMA1_FB1, VDMA1_FB2);
    vdma1_pair_diff_stats("fb2-fb0", VDMA1_FB2, VDMA1_FB0);
}

static void dump_vdma1_fb_hex(const char *tag, u32 fb_addr)
{
#if DUMP_VDMA1_FRAME_HEX_ONCE
    Xil_DCacheInvalidateRange((INTPTR)fb_addr, VDMA1_FRAME_BYTES);

    volatile u8 *p = (volatile u8 *)fb_addr;
    int nonzero = 0;
    int sum = 0;
    int minv = 255;
    int maxv = 0;

    for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
        int v = p[i];
        if (v != 0) nonzero++;
        sum += v;
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
    }

    xil_printf("[FRAME_DUMP_BEGIN] tag=%s w=%d h=%d bytes=%d fb=0x%08x nonzero=%d avg=%d min=%d max=%d\r\n",
               tag,
               VDMA1_W,
               VDMA1_H,
               VDMA1_FRAME_BYTES,
               (unsigned int)fb_addr,
               nonzero,
               sum / VDMA1_FRAME_BYTES,
               minv,
               maxv);

    for (int row = 0; row < (int)VDMA1_H; row++) {
        xil_printf("[FRAME_DUMP_DATA] %s %03d ", tag, row);
        for (int col = 0; col < (int)VDMA1_W; col++) {
            xil_printf("%02x", p[row * VDMA1_STRIDE + col]);
        }
        xil_printf("\r\n");
    }

    xil_printf("[FRAME_DUMP_END] tag=%s\r\n", tag);
#else
    (void)tag;
    (void)fb_addr;
#endif
}

static void dump_dronet_raw_hex(const char *tag)
{
    u32 st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
    u32 st1 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);

    dronet_read_raw();

    xil_printf("[RAW_DUMP_BEGIN] tag=%s channels=%d cells=%d bytes=%d status0=0x%08x status1=0x%08x raw_frame=%d\r\n",
               tag,
               NUM_CHANNELS,
               NUM_CELLS,
               RAW_BYTES_TOTAL,
               st0,
               st1,
               (int)((st1 >> 16) & 0xFFFF));

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        xil_printf("[RAW_DUMP_DATA] tag=%s ch=%d ", tag, ch);
        for (int cell = 0; cell < NUM_CELLS; cell++) {
            xil_printf("%02x", (uint8_t)raw_at(ch, cell));
        }
        xil_printf("\r\n");
    }

    xil_printf("[RAW_DUMP_END] tag=%s\r\n", tag);
}

static void dump_vdma1_all_fb_once_before_swstart(void)
{
    static int dumped = 0;
    if (dumped) return;
    dumped = 1;

    xil_printf("\r\n[VDMA1_DIAG] === before SW_START ===\r\n");
    xil_printf("[VDMA1_DIAG] S2MM_DMACR=0x%08x S2MM_DMASR=0x%08x\r\n",
               Xil_In32(VDMA1_BASEADDR + VDMA_S2MM_DMACR),
               Xil_In32(VDMA1_BASEADDR + VDMA_S2MM_DMASR));

    dump_vdma1_fb_stats("VDMA1_FB0", VDMA1_FB0);
    dump_vdma1_fb_stats("VDMA1_FB1", VDMA1_FB1);
    dump_vdma1_fb_stats("VDMA1_FB2", VDMA1_FB2);

#if DUMP_VDMA1_FRAME_HEX_ONCE
#if DUMP_VDMA1_FRAME_HEX_ALL_FB
    dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
    dump_vdma1_fb_hex("VDMA1_FB1", VDMA1_FB1);
    dump_vdma1_fb_hex("VDMA1_FB2", VDMA1_FB2);
#else
    dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
#endif
#endif

    xil_printf("[VDMA1_DIAG] === end ===\r\n\r\n");
}

static int dronet_infer_once(void) {
    static u32 infer_count = 0;
    static u16 last_used_frame = 0xFFFF;
    static int first = 1;
    u32 dbg1 = 0;
    u32 next_infer_count = infer_count + 1;
#if !DRONET_LIVE_LOG_ENABLE
    int log_this = 0;
#elif (DRONET_LOG_EVERY_N <= 1)
    int log_this = 1;
#else
    int log_this = (next_infer_count == 1) ||
                   ((next_infer_count % DRONET_LOG_EVERY_N) == 0);
#endif

    // ------------------------------------------------------------
    // 0) AUTO / FORCE / TESTPAT 완전히 OFF
    // ------------------------------------------------------------
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);

    // ------------------------------------------------------------
    // 1) 새 live frame 대기
    //    STATUS1[15:0] = last_frame 로 쓰고 있는 코드 기준
    // ------------------------------------------------------------
    u16 target_frame = 0;
    u32 st0 = 0;
    u32 st1 = 0;
    u32 dbg = 0;

    u32 timeout = 5000000;
    while (timeout--) {
        st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
        st1 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);

        u16 last_frame = (u16)(st1 & 0xFFFF);

        if ((st0 & DRONET_ST0_FRAME) && (first || last_frame != last_used_frame)) {
            target_frame = last_frame;
            break;
        }
    }

    if (timeout == 0) {
        dbg = Xil_In32(DRONET_BASEADDR + DRONET_DEBUG0);
        xil_printf("[SW_START] wait frame timeout\r\n");
        print_dronet_status(st0, st1, dbg);
        return 0;
    }

    first = 0;
    last_used_frame = target_frame;

    // 기존 raw_frame 저장
    u32 st1_before = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);
    u16 raw_frame_before = (u16)((st1_before >> 16) & 0xFFFF);

    // ------------------------------------------------------------
    // 2) 이전 DONE/RAW sticky clear
    //    CLRDONE은 pulse처럼 쓰고 바로 0으로 내림
    // ------------------------------------------------------------
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_CLRDONE);
    for (volatile int i = 0; i < 1000; i++) {}
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);
    for (volatile int i = 0; i < 1000; i++) {}

    // ------------------------------------------------------------
    // 3) SW_START 1회 pulse
    //    AUTO=0, FORCE=0, TESTPAT=0 상태에서 start만 1회
    // ------------------------------------------------------------
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_SW_START);
    for (volatile int i = 0; i < 1000; i++) {}
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);

    if (log_this) {
        xil_printf("[SW_START] issued target_frame=%d raw_before=%d\r\n",
                   (int)target_frame, (int)raw_frame_before);
    }

    // ------------------------------------------------------------
    // 4) 이번 SW_START 결과 완료 대기
    //    DONE/RAWVAL뿐 아니라 raw_frame 변화도 같이 확인
    // ------------------------------------------------------------
    timeout = 50000000;
    while (timeout--) {
        st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
        st1 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);

        u16 raw_frame = (u16)((st1 >> 16) & 0xFFFF);

        if ((st0 & DRONET_ST0_DONE) &&
            (st0 & DRONET_ST0_RAWVAL) &&
            (raw_frame != raw_frame_before)) {
            break;
        }
    }

    dbg = Xil_In32(DRONET_BASEADDR + DRONET_DEBUG0);
    dbg1 = Xil_In32(DRONET_BASEADDR + 0x014);
    if (log_this) {
        xil_printf("[DBG1] 0x%08x frame_byte=%d raw_last=%d raw_seen=%d raw_nz=%d\r\n",
                   dbg1, dbg1 & 0xFF, (dbg1>>8)&0xFF, (dbg1>>22)&1, (dbg1>>21)&1);
    }

    if (timeout == 0) {
        xil_printf("[SW_START] inference timeout\r\n");
        print_dronet_status(st0, st1, dbg);

        // DONE/RAW가 켜져 있는데 raw_frame만 안 바뀐 경우도 확인
        if ((st0 & DRONET_ST0_DONE) && (st0 & DRONET_ST0_RAWVAL)) {
            xil_printf("[SW_START] WARN: DONE/RAW set but raw_frame not changed\r\n");
        } else {
            return 0;
        }
    }

    infer_count = next_infer_count;

    // ------------------------------------------------------------
    // 5) RAW read
    // ------------------------------------------------------------
    dronet_read_raw();

    detection_t dets[MAX_DETECTIONS];
    int n = decode_and_filter(dets, MAX_DETECTIONS);

    if (DRONET_LIVE_LOG_ENABLE && (log_this || (n > 0))) {
        print_dronet_status(st0, st1, dbg);
        dronet_dump_raw_stats("LIVE_SW");
        xil_printf("[INF_SW] #%d decoded=%d\r\n", infer_count, n);
    }

    if (DRONET_LIVE_LOG_ENABLE && (n > 0)) {
        print_detections(dets, n);
    } else {
        if (log_this) {
            int best_cell = 0;
            int best_score = -1;
            int best_obj_raw = 0;
            int best_cls_raw = 0;

            for (int cell = 0; cell < NUM_CELLS; cell++) {
                int8_t obj_raw = raw_at(CH_OBJ, cell);
                int8_t cls_raw = raw_at(CH_CLS, cell);

                int obj_p = (int)(sigmoid_q(obj_raw) * 10000.0f);
                int cls_p = (int)(sigmoid_q(cls_raw) * 10000.0f);
                int score = (obj_p * cls_p) / 10000;

                if (score > best_score) {
                    best_score = score;
                    best_cell = cell;
                    best_obj_raw = obj_raw;
                    best_cls_raw = cls_raw;
                }
            }

            xil_printf("[RAW_SW] no detection. best_cell=%d obj_raw=%d cls_raw=%d score=%d.%02d%%\r\n",
                       best_cell,
                       best_obj_raw,
                       best_cls_raw,
                       best_score / 100,
                       best_score % 100);
        }
    }

    update_tracking_state(n);

    // ------------------------------------------------------------
    // 6) 다음 단발 추론을 위해 DONE clear 후 CONTROL=0 유지
    // ------------------------------------------------------------
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_CLRDONE);
    for (volatile int i = 0; i < 1000; i++) {}
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);

    return 1;
}

static void dronet_clear_done_keep_auto(void) {
    u32 ctrl = dronet_ctrl_base();
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, ctrl | DRONET_CTL_CLRDONE);
    for (volatile int i = 0; i < 1000; i++) {}
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, ctrl);
}

static void print_best_raw_score(void) {
    int best_cell = 0;
    int best_score = -1;
    int best_obj_raw = 0;
    int best_cls_raw = 0;

    for (int cell = 0; cell < NUM_CELLS; cell++) {
        int8_t obj_raw = raw_at(CH_OBJ, cell);
        int8_t cls_raw = raw_at(CH_CLS, cell);

        int obj_p = (int)(sigmoid_q(obj_raw) * 10000.0f);
        int cls_p = (int)(sigmoid_q(cls_raw) * 10000.0f);
        int score = (obj_p * cls_p) / 10000;

        if (score > best_score) {
            best_score = score;
            best_cell = cell;
            best_obj_raw = obj_raw;
            best_cls_raw = cls_raw;
        }
    }

    xil_printf("[RAW_AUTO] no detection. best_cell=%d obj_raw=%d cls_raw=%d score=%d.%02d%%\r\n",
               best_cell,
               best_obj_raw,
               best_cls_raw,
               best_score / 100,
               best_score % 100);
}

static int dronet_poll_auto_result(void) {
    static u32 infer_count = 0;
    static u16 last_raw_frame = 0xFFFF;
    static int first = 1;

    u32 st0 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS0);
    u32 st1 = Xil_In32(DRONET_BASEADDR + DRONET_STATUS1);
    u16 raw_frame = (u16)((st1 >> 16) & 0xFFFF);

    if (((st0 & DRONET_ST0_DONE) == 0) || ((st0 & DRONET_ST0_RAWVAL) == 0)) {
        return 0;
    }
    if (!first && raw_frame == last_raw_frame) {
        return 0;
    }

    first = 0;
    last_raw_frame = raw_frame;
    infer_count++;

    dronet_read_raw();

    detection_t dets[MAX_DETECTIONS];
    int n = decode_and_filter(dets, MAX_DETECTIONS);

#if !DRONET_LIVE_LOG_ENABLE
    int log_this = 0;
#elif (DRONET_LOG_EVERY_N <= 1)
    int log_this = 1;
#else
    int log_this = (infer_count == 1) || ((infer_count % DRONET_LOG_EVERY_N) == 0);
#endif

    if (DRONET_LIVE_LOG_ENABLE && (log_this || (n > 0) || g_tracking_on)) {
        u32 dbg = Xil_In32(DRONET_BASEADDR + DRONET_DEBUG0);
        print_dronet_status(st0, st1, dbg);
        dronet_dump_raw_stats("AUTO_SW");
        xil_printf("[INF_AUTO] #%d decoded=%d tracking=%d lost_sec=%d raw_frame=%d\r\n",
                   (int)infer_count,
                   n,
                   g_tracking_on,
                   (int)((n > 0) ? 0 : tracking_lost_seconds()),
                   (int)raw_frame);
    }

    if (DRONET_LIVE_LOG_ENABLE) {
        if (n > 0) {
            print_detections(dets, n);
        } else if (log_this || g_tracking_on) {
            print_best_raw_score();
        }
    }

#if MOTOR_CONTROL_DEMO
    if (n > 0) {
        motor_follow_detection(&dets[0]);
    } else {
        motor_hold_no_detection();
    }
#endif

    update_tracking_state(n);
    dronet_clear_done_keep_auto();
    return 1;
}

// ============================================================
// VDMA1 S2MM (Write) 함수
// ============================================================
static void vdma1_s2mm_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA1_BASEADDR + VDMA_S2MM_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA1-W] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_DMACR, 0x0001408B);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR1, fb0);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR2, fb1);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_START_ADDR3, fb2);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_FRMDLY_STRIDE, VDMA1_STRIDE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_HSIZE, VDMA1_HSIZE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_S2MM_VSIZE, VDMA1_H);

    xil_printf("[VDMA1-W] start done\r\n");
}

// ============================================================
// VDMA1 MM2S (Read) 함수 - prev frame을 DDR에서 읽어서 Frame Diff IP로 전달
// ============================================================
static void vdma1_mm2s_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA1_BASEADDR + VDMA_MM2S_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA1-R] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_DMACR, 0x0001408B);
    // Write와 같은 FB 주소를 읽음 (이전에 저장된 프레임 = prev)
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR1, fb0);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR2, fb1);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_START_ADDR3, fb2);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_FRMDLY_STRIDE,
              (VDMA1_PREV_FRAME_DELAY << VDMA_FRMDLY_SHIFT) | VDMA1_STRIDE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_HSIZE, VDMA1_HSIZE);
    Xil_Out32(VDMA1_BASEADDR + VDMA_MM2S_VSIZE, VDMA1_H);

    xil_printf("[VDMA1-R] start done (frame_delay=%d)\r\n",
               (int)VDMA1_PREV_FRAME_DELAY);
}

// ============================================================
// VDMA2 S2MM (Write) 함수 - diff 결과를 DDR에 저장
// ============================================================
static void vdma2_s2mm_start(u32 fb0, u32 fb1, u32 fb2)
{
    // Reset
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_DMACR, 0x00000004);
    u32 timeout = 10000000;
    while ((Xil_In32(VDMA2_BASEADDR + VDMA_S2MM_DMACR) & 0x00000004) != 0) {
        if (--timeout == 0) {
            xil_printf("[VDMA2] reset timeout!\r\n");
            return;
        }
    }

    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_DMACR, 0x00000003);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR1, fb0);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR2, fb1);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_START_ADDR3, fb2);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_FRMDLY_STRIDE, VDMA2_STRIDE);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_HSIZE, VDMA2_HSIZE);
    Xil_Out32(VDMA2_BASEADDR + VDMA_S2MM_VSIZE, VDMA2_H);

    xil_printf("[VDMA2] start done\r\n");
}
// threshold 설정
void load_threshold() {
    u32 val = ((u8)FC2_THR << 16) | (u8)FC1_THR;
    Xil_Out32(GPIO0_CH2, val);
    xil_printf("Threshold loaded: FC1=%d, FC2=%d\r\n", FC1_THR, FC2_THR);
}

// 가중치 1개 쓰기
void write_weight(u8 layer, u16 addr, int8_t data) {
    u32 val = ((u8)data << 13) | (addr << 2) | (layer << 1) | 1;
    Xil_Out32(GPIO0_CH1, val);
    // w_wr_en 내리기
    Xil_Out32(GPIO0_CH1, 0);
}

// 전체 가중치 로드
void load_weights() {
    xil_printf("Loading FC1 weights...\r\n");
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 144; j++) {
            write_weight(0, i * 144 + j, fc1_weight[i][j]);
        }
    }

    xil_printf("Loading FC2 weights...\r\n");
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            write_weight(1, i * 8 + j, fc2_weight[i][j]);
        }
    }

    xil_printf("All weights loaded!\r\n");
}

// 결과 읽기
typedef struct {
    u32 raw;
    u8  done;
    u8  result;
    u8  spk0;
    u8  spk1;
} snn_result_t;

static snn_result_t snn_read_result(void) {
    snn_result_t s;
    u32 val = Xil_In32(GPIO1_DATA);
    s.raw    = val;
    s.done   = val & 0x01;
    s.result = (val >> 1) & 0x01;
    s.spk0   = (val >> 2) & 0x1F;
    s.spk1   = (val >> 7) & 0x1F;
    return s;
}

static const char *snn_result_label(const snn_result_t *s) {
#if SNN_TRIGGERED_CNN_DEMO
    return s->result ? "TRIGGER" : "IDLE";
#else
    return s->result ? "NON-FLY" : "FLY";
#endif
}

static const char *snn_spike_class_label(const snn_result_t *s) {
    if (s->spk0 > s->spk1) return "FLY";
    if (s->spk1 > s->spk0) return "NON-FLY";
    return "TIE";
}

static int snn_result_is_trigger(const snn_result_t *s) {
    return s->done && (s->result != 0);
}

static void snn_print_result(const char *tag, const snn_result_t *s) {
#if SNN_ONLY_DEMO
    xil_printf("[%s] done=%d result_bit=%d spike_class=%s spk0_fly=%d spk1_nonfly=%d raw=0x%08x\r\n",
               tag,
               s->done,
               s->result,
               snn_spike_class_label(s),
               s->spk0,
               s->spk1,
               (unsigned int)s->raw);
#else
    xil_printf("[%s] done=%d result_bit=%d result=%s spike_class=%s spk0_fly=%d spk1_nonfly=%d raw=0x%08x\r\n",
               tag,
               s->done,
               s->result,
               snn_result_label(s),
               snn_spike_class_label(s),
               s->spk0,
               s->spk1,
               (unsigned int)s->raw);
#endif
}

void read_snn_result() {
    snn_result_t s = snn_read_result();
    snn_print_result("SNN", &s);
}

static void snn_only_monitor_loop(void) {
    xil_printf("[SNN_ONLY] started. CNN/motor disabled.\r\n");
    xil_printf("[SNN_ONLY] class is decoded from spikes: spk0=FLY, spk1=NON-FLY.\r\n");
    xil_printf("[SNN_ONLY] result_bit is the Top_0/result trigger pulse; class is decoded from spikes.\r\n");

    while (1) {
        snn_result_t s = snn_read_result();
        snn_print_result("SNN_ONLY", &s);
        snn_print_diff_stats();
        for (volatile u32 i = 0; i < 5000000u; ++i) {}
    }
}

// ============================================================
// 기존 프로젝트 설정
// ============================================================
#define IRPT_CTL_DEVID      XPAR_XSCUGIC_0_BASEADDR
#define GPIO_DEVID           XPAR_GPIO0_BASEADDR
#define GPIO_IRPT_ID         XPAR_PS7_GPIO_0_INTR
#define CAM_I2C_DEVID        XPAR_I2C0_BASEADDR
#define CAM_I2C_IRPT_ID      XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID           XPAR_AXI_VDMA_0_BASEADDR

#define VDMA_MM2S_IRPT_ID   XPAR_FABRIC_AXI_VDMA_0_INTR
#define VDMA_S2MM_IRPT_ID   XPAR_FABRIC_AXI_VDMA_0_INTR_1
#define CAM_I2C_SCLK_RATE   100000

#define DDR_BASE_ADDR        XPAR_DDR_MEM_BASEADDR
#define MEM_BASE_ADDR        (DDR_BASE_ADDR + 0x0A000000)

#define GAMMA_BASE_ADDR      XPAR_AXI_GAMMACORRECTION_0_BASEADDR

#define DEBUG_EN             0x0
#ifdef STDIN_BASEADDRESS
#define UART_BASEADDR        STDIN_BASEADDRESS
#else
#define UART_BASEADDR        XPAR_XUARTPS_0_BASEADDR
#endif

using namespace digilent;

void dFlushUart();
uint8_t dGetChar();
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV5640& cam,
                          VideoOutput& vid,
                          Resolution res,
                          OV5640_cfg::mode_t mode);

// ============================================================
// main
// ============================================================
static int ov5640_read_reg_direct(u16 reg, u8 *val)
{
    XIicPs Iic;
    XIicPs_Config *Cfg = XIicPs_LookupConfig(CAM_I2C_DEVID);

    if (Cfg == NULL) {
        xil_printf("[I2C] LookupConfig failed\r\n");
        return XST_FAILURE;
    }

    int status = XIicPs_CfgInitialize(&Iic, Cfg, Cfg->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("[I2C] CfgInitialize failed: %d\r\n", status);
        return status;
    }

    XIicPs_SetSClk(&Iic, CAM_I2C_SCLK_RATE);

    u8 regbuf[2];
    regbuf[0] = (u8)((reg >> 8) & 0xFF);
    regbuf[1] = (u8)(reg & 0xFF);

    while (XIicPs_BusIsBusy(&Iic));

    status = XIicPs_MasterSendPolled(&Iic, regbuf, 2, 0x3C);
    if (status != XST_SUCCESS) {
        xil_printf("[I2C] send failed reg=0x%04x status=%d\r\n", reg, status);
        return status;
    }

    while (XIicPs_BusIsBusy(&Iic));

    status = XIicPs_MasterRecvPolled(&Iic, val, 1, 0x3C);
    if (status != XST_SUCCESS) {
        xil_printf("[I2C] recv failed reg=0x%04x status=%d\r\n", reg, status);
        return status;
    }

    while (XIicPs_BusIsBusy(&Iic));

    return XST_SUCCESS;
}

int main() 
{   
    xil_printf("[CHK 0] before irpt_ctl\r\n");
    ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);
    xil_printf("[CHK 1] before gpio_driver\r\n");
    PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
    xil_printf("[CHK 2] before iic_driver\r\n");
    PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, CAM_I2C_SCLK_RATE);
    xil_printf("[CHK 3] before cam\r\n");
    OV5640* cam_ptr = nullptr;
    try {
        xil_printf("[I2C] direct OV5640 ID probe\r\n");
        u8 idh = 0;
        u8 idl = 0;
        int s0 = ov5640_read_reg_direct(0x300A, &idh);
        int s1 = ov5640_read_reg_direct(0x300B, &idl);
        xil_printf("[I2C] 0x300A status=%d value=0x%02x\r\n", s0, idh);
        xil_printf("[I2C] 0x300B status=%d value=0x%02x\r\n", s1, idl);
        cam_ptr = new OV5640(iic_driver, gpio_driver);
        xil_printf("[CHK 4] after cam constructor\r\n");
    }
    catch (const std::exception& e) {
        xil_printf("[ERR] OV5640 constructor failed: %s\r\n", e.what());
        while (1);
    }
    catch (...) {
        xil_printf("[ERR] OV5640 constructor failed: unknown exception\r\n");
        while (1);
    }
    OV5640& cam = *cam_ptr;
    AXI_VDMA<ScuGicInterruptController> vdma_driver(VDMA_DEVID, MEM_BASE_ADDR, irpt_ctl, VDMA_MM2S_IRPT_ID, VDMA_S2MM_IRPT_ID);
    xil_printf("[CHK 5] before vid\r\n");
    VideoOutput vid(XPAR_VTG_BASEADDR, XPAR_VIDEO_DYNCLK_BASEADDR);

    xil_printf("[CHK 6] before pipeline_mode_change\r\n");
    // 카메라 + HDMI 파이프라인 초기화
    pipeline_mode_change(vdma_driver, cam, vid,
        Resolution::R1280_720_60_PP,
        OV5640_cfg::mode_t::MODE_720P_1280_720_60fps);
    
    xil_printf("[CHK 7] after pipeline_mode_change\r\n");
    xil_printf("Video init done.\r\n");

    xil_printf("VDMA0 S2MM DMACR = 0x%08x\r\n", Xil_In32(XPAR_AXI_VDMA_0_BASEADDR + 0x30));
    xil_printf("VDMA0 MM2S DMACR = 0x%08x\r\n", Xil_In32(XPAR_AXI_VDMA_0_BASEADDR + 0x00));

    // clock-gating: SNN 가중치/threshold 로드는 SNN 클럭이 켜져 있어야 한다
    set_clock_mode(CLK_MODE_SNN);
    load_threshold();
    load_weights();
    // VDMA1 Write 시작 (gray8 원본 → DDR)
    xil_printf("Starting VDMA1 Write...\r\n");
    vdma1_s2mm_start(VDMA1_FB0, VDMA1_FB1, VDMA1_FB2);

    // VDMA2 Write 시작 (diff 결과 → DDR)
    xil_printf("Starting VDMA2 Write...\r\n");
    vdma2_s2mm_start(VDMA2_FB0, VDMA2_FB1, VDMA2_FB2);

    // 최소 1프레임 저장 대기
    for (volatile int i = 0; i < 5000000; ++i) { }

    // VDMA1 Read 시작 (prev frame DDR → Frame Diff IP)
    xil_printf("Starting VDMA1 Read...\r\n");
    vdma1_mm2s_start(VDMA1_FB0, VDMA1_FB1, VDMA1_FB2);

    xil_printf("All VDMAs started.\r\n");

#if SNN_ONLY_DEMO
    snn_only_monitor_loop();
#endif

    // ============== CNN 관련 초기화 ==============
    // clock-gating: CNN 가중치 로드/probe/추론은 CNN 코어 클럭이 켜져 있어야 한다
    set_clock_mode(CLK_MODE_CNN);
    if (!dronet_probe()) {
        xil_printf("!! DroNet AXI-Lite probe failed — halt\r\n");
        while (1) {}
    }
    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_RESET);
    for (volatile int i = 0; i < 100000; i++) {}

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);
    for (volatile int i = 0; i < 100000; i++) {}

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_CLRDONE | DRONET_CTL_CLRFRM);
    for (volatile int i = 0; i < 100000; i++) {}

    Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, 0);
    for (volatile int i = 0; i < 100000; i++) {}

    dronet_load_weights();
    dronet_init();
#if MOTOR_CONTROL_DEMO
    motor_init();
#endif

    u32 ctrl = Xil_In32(DRONET_BASEADDR + DRONET_CONTROL);
    xil_printf("[POST] LIVE CONTROL=0x%08x AUTO=%d FORCE=%d TESTPAT=%d\r\n",
            ctrl,
            (ctrl >> 2) & 1,
            (ctrl >> 3) & 1,
            (ctrl >> 4) & 1);
#if DUMP_VDMA1_FRAME_HEX_ONCE
    xil_printf("[UART] capture trigger ready: send 'c' (UART_BASEADDR=0x%08x)\r\n",
            (unsigned int)UART_BASEADDR);
#else
    xil_printf("[UART] live test logging enabled (every %d inference, UART_BASEADDR=0x%08x)\r\n",
            DRONET_LOG_EVERY_N,
            (unsigned int)UART_BASEADDR);
#endif
#if SNN_CNN_LINK_DEMO
    xil_printf("[LINK] SNN->CNN tracking demo enabled: SNN/Top_0_result triggers CNN automatically\r\n");
    xil_printf("[LINK] flow: wait SNN trigger -> CNN detect -> TRACK_SET -> keep CNN until %d sec miss -> TRACK_CLEAR\r\n",
               (int)LOST_TIMEOUT_SEC);
#endif
#if CNN_MOTOR_ALWAYS_DEMO
    xil_printf("[MOTOR] CNN always-on pan/tilt demo enabled\r\n");
    xil_printf("[MOTOR] flow: CNN every frame -> best detection -> pan/tilt servo, UART detection log\r\n");
#endif
#if SNN_CNN_MOTOR_DEMO
    xil_printf("[LINK+MOTOR] SNN->CNN->pan/tilt tracking demo enabled\r\n");
    xil_printf("[LINK+MOTOR] flow: wait SNN trigger -> CNN detect -> motor follow -> keep CNN until %d sec miss\r\n",
               (int)LOST_TIMEOUT_SEC);
#endif

    // ============================================================
    // 메뉴 루프
    // ============================================================
    uint8_t read_char0 = 0;
    uint8_t read_char1 = 0;
    uint8_t read_char2 = 0;
    uint8_t read_char4 = 0;
    uint8_t read_char5 = 0;
    uint16_t reg_addr;
    uint8_t reg_value;

    u32 loop_count = 0; // 너무 빨리 찍히는 걸 방지하기 위한 카운터

    // clock-gating: 운영 시작 모드
    //   SNN-trigger 데모 -> SNN 모드에서 트리거 대기로 시작
    //   CNN 상시 데모     -> CNN 모드 고정
#if SNN_TRIGGERED_CNN_DEMO
    set_clock_mode(CLK_MODE_SNN);
    xil_printf("[CLKGATE] start in SNN mode (waiting for SNN trigger)\r\n");
#elif CNN_MOTOR_ALWAYS_DEMO
    set_clock_mode(CLK_MODE_CNN);
    xil_printf("[CLKGATE] start in CNN mode (always-on)\r\n");
#endif



//     while (1)
//     {
//         xil_printf("\r\n\r\n\r\nPcam 5C MAIN OPTIONS\r\n");
//         xil_printf("\r\nPlease press the key corresponding to the desired option:");
//         xil_printf("\r\n  a. Change Resolution");
//         xil_printf("\r\n  b. Change Liquid Lens Focus");
//         xil_printf("\r\n  d. Change Image Format (Raw or RGB)");
//         xil_printf("\r\n  e. Write a Register Inside the Image Sensor");
//         xil_printf("\r\n  f. Read a Register Inside the Image Sensor");
//         xil_printf("\r\n  g. Change Gamma Correction Factor Value");
//         xil_printf("\r\n  h. Change AWB Settings");
//         xil_printf("\r\n  z. Dump diff image (frame difference)");
//         xil_printf("\r\n  y. Dump gray8 original image\r\n\r\n");

//         read_char0 = getchar(); getchar();
//         xil_printf("Read: %d\r\n", read_char0);

//         switch (read_char0)
//         {
//         case 'a':
//             xil_printf("\r\n  Please press the key corresponding to the desired resolution:");
//             xil_printf("\r\n    1. 1280 x 720, 60fps");
//             xil_printf("\r\n    2. 1920 x 1080, 15fps");
//             xil_printf("\r\n    3. 1920 x 1080, 30fps");

//             read_char1 = getchar(); getchar();
//             xil_printf("\r\nRead: %d", read_char1);

//             switch (read_char1)
//             {
//             case '1':
//                 pipeline_mode_change(vdma_driver, cam, vid,
//                     Resolution::R1280_720_60_PP,
//                     OV5640_cfg::mode_t::MODE_720P_1280_720_60fps);
//                 xil_printf("Resolution change done.\r\n");
//                 break;
//             case '2':
//                 pipeline_mode_change(vdma_driver, cam, vid,
//                     Resolution::R1920_1080_60_PP,
//                     OV5640_cfg::mode_t::MODE_1080P_1920_1080_15fps);
//                 xil_printf("Resolution change done.\r\n");
//                 break;
//             case '3':
//                 pipeline_mode_change(vdma_driver, cam, vid,
//                     Resolution::R1920_1080_60_PP,
//                     OV5640_cfg::mode_t::MODE_1080P_1920_1080_30fps);
//                 xil_printf("Resolution change done.\r\n");
//                 break;
//             default:
//                 xil_printf("\r\n  Selection is outside the available options! Please retry...");
//             }
//             break;

//         case 'b':
//             xil_printf("\r\n\r\nPlease enter value of liquid lens register, in hex, with small letters (2 nibbles): 0x");
//             while (read_char1 < 48) { read_char1 = getchar(); }
//             while (read_char2 < 48) { read_char2 = getchar(); getchar(); }
//             if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
//             if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
//             cam.writeRegLiquid((uint8_t)(16*read_char1 + read_char2));
//             xil_printf("\r\nWrote to liquid lens controller: %x", (uint8_t)(16*read_char1 + read_char2));
//             break;

//         case 'd':
//             xil_printf("\r\n  Please press the key corresponding to the desired setting:");
//             xil_printf("\r\n    1. Select image format to be RGB, output still Raw");
//             xil_printf("\r\n    2. Select image format & output to both be Raw");
//             read_char1 = getchar(); getchar();
//             xil_printf("\r\nRead: %d", read_char1);
//             switch (read_char1)
//             {
//             case '1':
//                 cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RGB);
//                 xil_printf("Settings change done.\r\n");
//                 break;
//             case '2':
//                 cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RAW);
//                 xil_printf("Settings change done.\r\n");
//                 break;
//             default:
//                 xil_printf("\r\n  Selection is outside the available options! Please retry...");
//             }
//             break;

//         case 'e':
//             xil_printf("\r\nPlease enter address of image sensor register, in hex, with small letters (4 nibbles): \r\n");
//             while (read_char1 < 48) { read_char1 = getchar(); }
//             while (read_char2 < 48) { read_char2 = getchar(); }
//             while (read_char4 < 48) { read_char4 = getchar(); }
//             while (read_char5 < 48) { read_char5 = getchar(); getchar(); }
//             if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
//             if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
//             if (read_char4 <= 57) read_char4 -= 48; else read_char4 -= 87;
//             if (read_char5 <= 57) read_char5 -= 48; else read_char5 -= 87;
//             reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
//             xil_printf("Desired Register Address: %x\r\n", reg_addr);

//             read_char1 = 0;
//             read_char2 = 0;
//             xil_printf("\r\nPlease enter value of image sensor register, in hex, with small letters (2 nibbles): \r\n");
//             while (read_char1 < 48) { read_char1 = getchar(); }
//             while (read_char2 < 48) { read_char2 = getchar(); getchar(); }
//             if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
//             if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
//             reg_value = 16*read_char1 + read_char2;
//             xil_printf("Desired Register Value: %x\r\n", reg_value);
//             cam.writeReg(reg_addr, reg_value);
//             xil_printf("Register write done.\r\n");
//             break;

//         case 'f':
//             xil_printf("Please enter address of image sensor register, in hex, with small letters (4 nibbles): \r\n");
//             while (read_char1 < 48) { read_char1 = getchar(); }
//             while (read_char2 < 48) { read_char2 = getchar(); }
//             while (read_char4 < 48) { read_char4 = getchar(); }
//             while (read_char5 < 48) { read_char5 = getchar(); getchar(); }
//             if (read_char1 <= 57) read_char1 -= 48; else read_char1 -= 87;
//             if (read_char2 <= 57) read_char2 -= 48; else read_char2 -= 87;
//             if (read_char4 <= 57) read_char4 -= 48; else read_char4 -= 87;
//             if (read_char5 <= 57) read_char5 -= 48; else read_char5 -= 87;
//             reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
//             xil_printf("Desired Register Address: %x\r\n", reg_addr);
//             cam.readReg(reg_addr, reg_value);
//             xil_printf("Value of Desired Register: %x\r\n", reg_value);
//             break;

//         case 'g':
//             xil_printf("  Please press the key corresponding to the desired Gamma factor:\r\n");
//             xil_printf("    1. Gamma Factor = 1\r\n");
//             xil_printf("    2. Gamma Factor = 1/1.2\r\n");
//             xil_printf("    3. Gamma Factor = 1/1.5\r\n");
//             xil_printf("    4. Gamma Factor = 1/1.8\r\n");
//             xil_printf("    5. Gamma Factor = 1/2.2\r\n");
//             read_char1 = getchar(); getchar();
//             xil_printf("Read: %d\r\n", read_char1);
//             read_char1 = read_char1 - 48;
//             if ((read_char1 > 0) && (read_char1 < 6)) {
//                 Xil_Out32(GAMMA_BASE_ADDR, read_char1-1);
//                 xil_printf("Gamma value changed to option %d.\r\n", read_char1);
//             } else {
//                 xil_printf("  Selection is outside the available options! Please retry...\r\n");
//             }
//             break;

//         case 'h':
//             xil_printf("  Please press the key corresponding to the desired AWB change:\r\n");
//             xil_printf("    1. Enable Advanced AWB\r\n");
//             xil_printf("    2. Enable Simple AWB\r\n");
//             xil_printf("    3. Disable AWB\r\n");
//             read_char1 = getchar(); getchar();
//             xil_printf("Read: %d\r\n", read_char1);
//             switch (read_char1)
//             {
//             case '1': cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED); xil_printf("Enabled Advanced AWB\r\n"); break;
//             case '2': cam.set_awb(OV5640_cfg::awb_t::AWB_SIMPLE);   xil_printf("Enabled Simple AWB\r\n");   break;
//             case '3': cam.set_awb(OV5640_cfg::awb_t::AWB_DISABLED); xil_printf("Disabled AWB\r\n");         break;
//             default:  xil_printf("  Selection is outside the available options! Please retry...\r\n");
//             }
//             break;

//         // ============================================================
//         // Dump: diff 결과 (VDMA2)
//         // ============================================================
//         case 'z':
//         {
//             xil_printf("P5\n160 90\n255\n");
//             Xil_DCacheInvalidateRange((INTPTR)VDMA2_FB0, VDMA2_FRAME_BYTES);
//             u8 *img_ptr = (u8 *)VDMA2_FB0;
//             for (u32 i = 0; i < VDMA2_FRAME_BYTES; i++) {
//                 outbyte(img_ptr[i]);
//             }
//             break;
//         }

//         // ============================================================
//         // Dump: gray8 원본 (VDMA1)
//         // ============================================================
//         case 'y':
//         {
//             xil_printf("P5\n160 90\n255\n");
//             Xil_DCacheInvalidateRange((INTPTR)VDMA1_FB0, VDMA1_FRAME_BYTES);
//             u8 *img_ptr = (u8 *)VDMA1_FB0;
//             for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
//                 outbyte(img_ptr[i]);
//             }
//             break;
//         }
//         case 'r':
//         {
//             read_snn_result();
//             break;
//         }


//         default:
//             xil_printf("  Selection is outside the available options! Please retry...\r\n");
//         }

//         read_char1 = 0;
//         read_char2 = 0;
//         read_char4 = 0;
//         read_char5 = 0;
//     }

// // dronet_init() 뒤, while loop 들어가기 전에 임시 추가
// xil_printf("\r\n[DIAG] === Full RAW dump after 1 forced inference ===\r\n");
// Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL, DRONET_CTL_RESET);
// for (volatile int i = 0; i < 1000000; i++) {}
// Xil_Out32(DRONET_BASEADDR + DRONET_CONTROL,
//           DRONET_CTL_AUTO | DRONET_CTL_FORCE);
// // 충분히 기다림 - 카메라 프레임 들어오고 force_infer로 추론까지
// for (volatile int i = 0; i < 50000000; i++) {}

// dronet_read_raw();

// // 전체 1320 byte 통계
// int total_nonzero = 0;
// int total_min = 127, total_max = -128;
// int ch_nonzero[6] = {0};
// int ch_min[6] = {127,127,127,127,127,127};
// int ch_max[6] = {-128,-128,-128,-128,-128,-128};

// for (int ch = 0; ch < 6; ch++) {
//     for (int cell = 0; cell < NUM_CELLS; cell++) {
//         int8_t v = raw_at(ch, cell);
//         if (v != 0) { ch_nonzero[ch]++; total_nonzero++; }
//         if ((int)v < ch_min[ch]) ch_min[ch] = (int)v;
//         if ((int)v > ch_max[ch]) ch_max[ch] = (int)v;
//         if ((int)v < total_min) total_min = (int)v;
//         if ((int)v > total_max) total_max = (int)v;
//     }
// }

// const char *cn[6] = {"tx ","ty ","tw ","th ","obj","cls"};
// xil_printf("[DIAG] TOTAL: nonzero=%d/1320  min=%d  max=%d\r\n",
//            total_nonzero, total_min, total_max);
// for (int ch = 0; ch < 6; ch++) {
//     xil_printf("[DIAG] ch%d (%s): nonzero=%3d/220  min=%4d  max=%4d\r\n",
//                ch, cn[ch], ch_nonzero[ch], ch_min[ch], ch_max[ch]);
// }

// // 처음 32바이트 hex dump
// xil_printf("[DIAG] first 32 bytes: ");
// for (int i = 0; i < 32; i++) xil_printf("%02x ", (uint8_t)g_raw[i]);
// xil_printf("\r\n");
// xil_printf("[DIAG] === end ===\r\n\r\n");
// // 임시 코드 끝





    while (1)
    {
        int skip_loop_infer = 0;

        // // [1] Non-blocking 키보드 입력 체크
        // // UART 수신 버퍼에 데이터가 있는지 확인 (멈추지 않음)
        // if (XUartPs_IsReceiveData(UART_BASEADDR)) 
        // {
        //     read_char0 = getchar();
        //     // 메뉴 처리 로직 (기존 switch-case 문 그대로 사용)
        //     // 단, 메뉴 진입 시에는 실시간 출력이 잠시 멈춤
        //     handle_menu(read_char0); // 기존 switch문을 별도 함수로 빼거나 여기에 위치
        // }

        // [2] 실시간 SNN 결과 읽기 및 출력
        // 매 루프마다 찍으면 UART 전송 속도가 못 따라가므로 
        // 약 수만 번의 루프마다 한 번씩(또는 특정 주기마다) 출력하도록 조절
        
#if SNN_TRIGGERED_CNN_DEMO || CNN_MOTOR_ALWAYS_DEMO
        (void)skip_loop_infer;
        while (XUartPs_IsReceiveData(UART_BASEADDR)) {
            (void)XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        }

#if SNN_TRIGGERED_CNN_DEMO
        // ===== clock-gating 상호배타 운영 =====
        if (g_clk_mode == CLK_MODE_SNN) {
            // ---- SNN 활성 (CNN 클럭 게이팅됨): SNN 트리거 대기 ----
            snn_result_t s = snn_read_result();
            if (snn_rearm_settled() && snn_result_is_trigger(&s)) {
                // SNN 이 움직임/객체 트리거 -> CNN 모드로 전환
                set_clock_mode(CLK_MODE_CNN);
                dronet_clear_done_keep_auto();   // 게이팅 동안의 stale DONE 정리
                xil_printf("[CLKGATE] SNN trigger -> CNN mode ON (CNN clk enabled)\r\n");
            } else {
                loop_count++;
                if ((loop_count % SNN_STATUS_LOG_EVERY_LOOPS) == 1) {
                    snn_print_result("SNN_WAIT", &s);
                }
            }
        } else {
            // ---- CNN 활성 (SNN 클럭 게이팅됨): 추론/추적 ----
            dronet_poll_auto_result();   // 내부 update_tracking_state ->
                                         // tracking_clear() 가 lost 시 SNN 복귀
            // false trigger 로 들어와 추적을 한 번도 못 잡으면 타임아웃 복귀
            if (!g_tracking_on) {
                XTime now_t;
                XTime_GetTime(&now_t);
                u32 wait_sec = (u32)((now_t - g_cnn_acquire_start)
                                     / DRONET_TIMER_COUNTS_PER_SECOND);
                if (wait_sec >= CNN_ACQUIRE_TIMEOUT_SEC) {
                    set_clock_mode(CLK_MODE_SNN);
                    xil_printf("[CLKGATE] CNN no-acquire %ds -> back to SNN mode\r\n",
                               (int)CNN_ACQUIRE_TIMEOUT_SEC);
                }
            }
        }
#else   /* CNN_MOTOR_ALWAYS_DEMO : CNN 모드 고정, 매 프레임 추론 */
        dronet_poll_auto_result();
#endif
#else
#if DUMP_VDMA1_FRAME_HEX_ONCE
        if (XUartPs_IsReceiveData(UART_BASEADDR)) {
            read_char0 = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
            if ((read_char0 == 'c') || (read_char0 == 'C')) {
#if DUMP_VDMA1_FRAME_HEX_ALL_FB
                dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
                dump_vdma1_fb_hex("VDMA1_FB1", VDMA1_FB1);
                dump_vdma1_fb_hex("VDMA1_FB2", VDMA1_FB2);
#else
                dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
#endif
            } else if ((read_char0 == 'r') || (read_char0 == 'R')) {
                dronet_infer_once();
                dump_dronet_raw_hex("DRONET_RAW");
                skip_loop_infer = 1;
            } else if ((read_char0 == 'b') || (read_char0 == 'B')) {
#if DUMP_VDMA1_FRAME_HEX_ALL_FB
                dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
                dump_vdma1_fb_hex("VDMA1_FB1", VDMA1_FB1);
                dump_vdma1_fb_hex("VDMA1_FB2", VDMA1_FB2);
#else
                dump_vdma1_fb_hex("VDMA1_FB0", VDMA1_FB0);
#endif
                dronet_infer_once();
                dump_dronet_raw_hex("DRONET_RAW");
                skip_loop_infer = 1;
            }
        }
#else
        while (XUartPs_IsReceiveData(UART_BASEADDR)) {
            (void)XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        }
#endif

        if (!skip_loop_infer) {
            dronet_infer_once();
        }
#endif
        
        // if (loop_count++ >= 100000) 
        // {
        //     read_snn_result();
        //     loop_count = 0; // 카운터 초기화
        // }

        // === 임시 진단: 부팅 후 5초 뒤 VDMA1 + s_pixel 양쪽 확인 ===
    static int dumped = 0;
    if (0 && !dumped && loop_count++ > 50000) {
        dumped = 1;
        xil_printf("\r\n=== VDMA1 FB0 dump (first 256 bytes) ===\r\n");
        Xil_DCacheInvalidateRange((INTPTR)VDMA1_FB0, VDMA1_FRAME_BYTES);
        u8 *p = (u8 *)VDMA1_FB0;
        int nonzero = 0, sum = 0, mx = 0;
        for (u32 i = 0; i < VDMA1_FRAME_BYTES; i++) {
            if (p[i] != 0) nonzero++;
            sum += p[i];
            if (p[i] > mx) mx = p[i];
        }
        xil_printf("VDMA1 frame stats: nonzero=%d/%d  avg=%d  max=%d\r\n",
                nonzero, VDMA1_FRAME_BYTES, sum / VDMA1_FRAME_BYTES, mx);
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 16; col++) {
                xil_printf("%02x ", p[row * 160 + col]);
            }
            xil_printf("\r\n");
        }
     }
    }
    

    return 0;
}

// ============================================================
// UART 유틸리티
// ============================================================
void dFlushUart()
{
    while (XUartPs_IsReceiveData(UART_BASEADDR))
        XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
}

uint8_t dGetChar()
{
    uint8_t chRxCh = '0';
    while (XUartPs_IsReceiveData(UART_BASEADDR))
    {
        chRxCh = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        if (chRxCh == '\n') break;
    }
    return chRxCh;
}

// ============================================================
// 카메라 파이프라인 초기화
// ============================================================
// void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
//                           OV5640& cam, VideoOutput& vid,
//                           Resolution res, OV5640_cfg::mode_t mode)
// {
//     vdma_driver.resetWrite();
//     MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
//     MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
// #if (DEBUG_EN == 0x0)
//     cam.reset();
// #endif
//     vdma_driver.configureWrite(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
//     Xil_Out32(GAMMA_BASE_ADDR, 3);
// #if (DEBUG_EN == 0x0)
//     cam.init();
//     vdma_driver.enableWrite();
// #endif
//     MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
//     MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
// #if (DEBUG_EN == 0x0)
//     cam.set_mode(mode);
//     cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED);
// #endif
//     vid.reset();
//     vdma_driver.resetRead();
//     vid.configure(res);
//     vdma_driver.configureRead(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
//     vid.enable();
//     vdma_driver.enableRead();
// }
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV5640& cam, VideoOutput& vid,
                          Resolution res, OV5640_cfg::mode_t mode)
{
    xil_printf("[PIPE 0] resetWrite\r\n");
    vdma_driver.resetWrite();

    xil_printf("[PIPE 1] reset CSI/DPHY\r\n");
    MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
    MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));

#if (DEBUG_EN == 0x0)
    xil_printf("[PIPE 2] cam.reset\r\n");
    cam.reset();
    xil_printf("[PIPE 3] cam.reset done\r\n");
#endif

    xil_printf("[PIPE 4] configureWrite\r\n");
    vdma_driver.configureWrite(timing[static_cast<int>(res)].h_active,
                               timing[static_cast<int>(res)].v_active);

    xil_printf("[PIPE 5] gamma\r\n");
    Xil_Out32(GAMMA_BASE_ADDR, 3);

#if (DEBUG_EN == 0x0)
    xil_printf("[PIPE 6] cam.init\r\n");
    cam.init();
    xil_printf("[PIPE 7] cam.init done\r\n");

    xil_printf("[PIPE 8] enableWrite\r\n");
    vdma_driver.enableWrite();
#endif

    xil_printf("[PIPE 9] enable CSI/DPHY\r\n");
    MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
    MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);

#if (DEBUG_EN == 0x0)
    xil_printf("[PIPE 10] cam.set_mode\r\n");
    cam.set_mode(mode);

    xil_printf("[PIPE 11] cam.set_awb\r\n");
    cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED);
#endif

    xil_printf("[PIPE 12] vid.reset\r\n");
    vid.reset();

    xil_printf("[PIPE 13] resetRead\r\n");
    vdma_driver.resetRead();

    xil_printf("[PIPE 14] vid.configure\r\n");
    vid.configure(res);

    xil_printf("[PIPE 15] configureRead\r\n");
    vdma_driver.configureRead(timing[static_cast<int>(res)].h_active,
                              timing[static_cast<int>(res)].v_active);

    xil_printf("[PIPE 16] vid.enable\r\n");
    vid.enable();

    xil_printf("[PIPE 17] enableRead\r\n");
    vdma_driver.enableRead();

    xil_printf("[PIPE 18] done\r\n");
}
