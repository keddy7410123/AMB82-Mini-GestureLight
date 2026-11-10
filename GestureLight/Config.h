#pragma once
#include <stdint.h>

namespace cfg {
// Fill these in locally. Never publish your Wi-Fi password.
static char SSID[] = "YOUR_WIFI_NAME";
static char PASSWORD[] = "YOUR_WIFI_PASSWORD";

// User-confirmed five-gesture model: IDs 0..4 correspond to gestures 1..5.
constexpr int OPEN_CLASS = 4;
constexpr int CLOSE_CLASS = 0;
constexpr int CONFIDENCE = 60;       // score() uses 0..100
constexpr float DETECT_THRESHOLD = 0.60f; // NN output and gesture trigger both require 60%
// Pixel-aligned ROI shared by SDK input crop, policy, and web overlay.
constexpr int NN_WIDTH = 576, NN_HEIGHT = 320;
constexpr int PREVIEW_FPS = 20; // JPEG capture / browser target; actual rate depends on Wi-Fi
constexpr int INFERENCE_INPUT_FPS = 10; // independent of preview; not measured model FPS
static_assert(PREVIEW_FPS >= 1 && PREVIEW_FPS <= 30 && INFERENCE_INPUT_FPS >= 1 && INFERENCE_INPUT_FPS <= 30, "Invalid camera FPS");
constexpr int ROI_XMIN = 116, ROI_XMAX = 460, ROI_YMIN = 48, ROI_YMAX = 272;
constexpr float ROI_L = float(ROI_XMIN) / NN_WIDTH, ROI_T = float(ROI_YMIN) / NN_HEIGHT;
constexpr float ROI_R = float(ROI_XMAX) / NN_WIDTH, ROI_B = float(ROI_YMAX) / NN_HEIGHT;
constexpr uint32_t HOLD_MS = 500, RELEASE_MS = 500, COOLDOWN_MS = 2000;
constexpr uint32_t MAX_FRAME_GAP_MS = 800, STALE_MS = 1500;
constexpr unsigned MIN_SAMPLES = 3;
constexpr uint32_t RECOGNITION_LED_MS = 1000;

constexpr int SERVO_PIN = 8;         // AMB82 D8 = PF_15, PWM capable
// S35 STD is continuous rotation: these values control speed/direction, NOT angle.
constexpr int STOP_US = 1500;        // must calibrate; may creep at 1500
constexpr int OPEN_US = 1600, CLOSE_US = 1400;
constexpr uint32_t OPEN_MS = 400, CLOSE_MS = 400;
// Each gesture runs only its own direction, then stops; no return stroke.
constexpr uint32_t MAX_PHASE_MS = 500;
// Set true ONLY after neutral, direction and travel are physically tested.
constexpr bool MOTOR_CALIBRATED = true;

static_assert(OPEN_CLASS >= 0 && CLOSE_CLASS >= 0 && OPEN_CLASS != CLOSE_CLASS, "Invalid classes");
static_assert(ROI_L >= 0 && ROI_T >= 0 && ROI_R <= 1 && ROI_B <= 1 && ROI_L < ROI_R && ROI_T < ROI_B, "Invalid ROI");
static_assert(OPEN_MS > 0 && CLOSE_MS > 0 && OPEN_MS <= MAX_PHASE_MS && CLOSE_MS <= MAX_PHASE_MS, "Motor duration exceeds limit");
static_assert(STOP_US >= 1400 && STOP_US <= 1600 && OPEN_US >= 1000 && OPEN_US <= 2000 && CLOSE_US >= 1000 && CLOSE_US <= 2000, "Check PWM pulse width");
static_assert((OPEN_US > STOP_US && CLOSE_US < STOP_US) || (OPEN_US < STOP_US && CLOSE_US > STOP_US), "Open/close must use opposite directions");
}
