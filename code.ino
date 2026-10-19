#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <U8g2lib.h>
#include <FastAccelStepper.h>
#include "driver/i2s_std.h"

#define PIN_STEP      19
#define PIN_DIR       18
#define PIN_EN        23
#define PIN_I2S_BCLK  26
#define PIN_I2S_WS    25
#define PIN_I2S_DOUT  27
#define PIN_I2S_DIN   35
#define PIN_ENC_CLK   13
#define PIN_ENC_DT    14
#define PIN_ENC_SW    4
#define PIN_LIM_START 32
#define PIN_LIM_END   33

#define AUTO_HOME_ON_BOOT     1
#define LIM_ACTIVE            LOW
#define DIR_HIGH_COUNTS_UP    true
#define ENC_INVERT            0
#define ENC_STATES_PER_DETENT 4

constexpr int STEPS_PER_REV = 200;
constexpr int MICROSTEPS = 8;
constexpr float LEAD_MM = 2.0f;
constexpr float TRAVEL_MM = 440.0f;
constexpr float L_MIN_MM = 660.0f;

constexpr float BACKOFF_MM = 3.0f;
constexpr float MARGIN_MM = 1.0f;

constexpr float MOVE_SPEED_MMS = 4.5f;
constexpr float HOME_SPEED_MMS = 2.5f;
constexpr float ACCEL_MMS2 = 4.0f;

constexpr float TUBE_INNER_RADIUS_MM = 22.0f;
constexpr float END_CORR_FACTOR = 0.6f;
constexpr float HORN_EXTRA_MM = 0.0f;
constexpr int FREQ_MIN = 60;
constexpr int FREQ_MAX = 1500;
constexpr int MAX_HARMONIC = 30;
constexpr int MAX_HARM_SEL = 20;
constexpr float SCAN_RANGE_MM = 40.0f;
constexpr float SCAN_STEP_MM = 4.0f;

#define SAMPLE_RATE  16000
#define BLOCK_FRAMES 256
#define GOERTZEL_N   1024
constexpr float SPK_AMPLITUDE = 0.25f;

enum TubeType : uint8_t {
  TUBE_CLOSED_OPEN = 0,
  TUBE_OPEN_OPEN = 1
};

struct Resonance {
  bool ok;
  int n;
  float v;
  float lenMm;
  const char* msg;
};

constexpr float STEPS_PER_MM = (float)STEPS_PER_REV * MICROSTEPS / LEAD_MM;
inline int32_t mmToSteps(float mm) {
  return (int32_t)lroundf(mm * STEPS_PER_MM);
}
inline float stepsToMm(int32_t s) {
  return (float)s / STEPS_PER_MM;
}
inline uint32_t mmsToHz(float mms) {
  return (uint32_t)lroundf(mms * STEPS_PER_MM);
}

volatile float gFreq = 640.0f;
volatile bool gSpkOn = false;
volatile float gToneAmp = 0.0f;
volatile float gMicDb = -120.0f;
volatile uint32_t gFrameCnt = 0;

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;
enum MState : uint8_t {
  M_UNHOMED,
  M_HOMING_SEEK,
  M_HOMING_BACKOFF,
  M_READY,
  M_ERROR
};
volatile MState mState = M_UNHOMED;
volatile bool reqHome = false;
volatile bool reqMove = false;
volatile int32_t reqTarget = 0;
const char* volatile mErr = nullptr;
volatile bool gHomed = false;

U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

float soundSpeed(float tC) {
  return 331.3f * sqrtf(1.0f + tC / 273.15f);
}

float lenFor(int n, float f, float v, TubeType t) {
  float halfWaves = (t == TUBE_OPEN_OPEN) ? (float)n : ((float)n - 0.5f);
  float lEffMm = halfWaves * 0.5f * v / f * 1000.0f;
  float corr = ((t == TUBE_OPEN_OPEN) ? 2.0f : 1.0f) * END_CORR_FACTOR * TUBE_INNER_RADIUS_MM +
               HORN_EXTRA_MM;
  return lEffMm - corr;
}

Resonance computeResonance(float f, int harmSel, TubeType t, float tC) {
  Resonance r;
  r.v = soundSpeed(tC);
  r.ok = false;
  r.n = (harmSel == 0) ? 1 : harmSel;
  r.lenMm = 0;
  r.msg = "";
  const float lo = L_MIN_MM + MARGIN_MM;
  const float hi = L_MIN_MM + TRAVEL_MM - MARGIN_MM;
  if (harmSel == 0) {
    for (int n = 1; n <= MAX_HARMONIC; n++) {
      float L = lenFor(n, f, r.v, t);
      if (L > hi)
        break;
      if (L >= lo) {
        r.ok = true;
        r.n = n;
        r.lenMm = L;
        return r;
      }
    }
    r.msg = "KHONG CO n PHU HOP";
    return r;
  }
  r.lenMm = lenFor(harmSel, f, r.v, t);
  r.ok = (r.lenMm >= lo && r.lenMm <= hi);
  if (!r.ok)
    r.msg = "n NGOAI TAM ONG";
  return r;
}

static i2s_chan_handle_t txH = nullptr, rxH = nullptr;
static float sineTab[2048];
static int32_t txBuf[BLOCK_FRAMES * 2];
static int32_t rxBuf[BLOCK_FRAMES * 2];

void audioInit() {
  for (int i = 0; i < 2048; i++)
    sineTab[i] = sinf(2.0f * PI * i / 2048.0f);

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = BLOCK_FRAMES;
  chanCfg.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(&chanCfg, &txH, &rxH));

  i2s_std_config_t stdCfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = (gpio_num_t)PIN_I2S_BCLK,
              .ws = (gpio_num_t)PIN_I2S_WS,
              .dout = (gpio_num_t)PIN_I2S_DOUT,
              .din = (gpio_num_t)PIN_I2S_DIN,
              .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
          },
  };
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txH, &stdCfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rxH, &stdCfg));
  ESP_ERROR_CHECK(i2s_channel_enable(txH));
  ESP_ERROR_CHECK(i2s_channel_enable(rxH));
}

void audioTask(void*) {
  uint32_t phase = 0;
  float amp = 0.0f;
  const float ramp = SPK_AMPLITUDE / (SAMPLE_RATE * 0.05f);
  float dc = 0.0f;
  float s1 = 0, s2 = 0, lastF = -1;
  int gN = 0;

  for (;;) {
    float f = gFreq;
    uint32_t inc = (uint32_t)((double)f * 4294967296.0 / SAMPLE_RATE);
    float target = gSpkOn ? SPK_AMPLITUDE : 0.0f;

    for (int i = 0; i < BLOCK_FRAMES; i++) {
      if (amp < target) {
        amp += ramp;
        if (amp > target)
          amp = target;
      } else if (amp > target) {
        amp -= ramp;
        if (amp < target)
          amp = target;
      }
      float s = sineTab[phase >> 21] * amp;
      int32_t v = (int32_t)(s * 2147483000.0f);
      txBuf[2 * i] = v;
      txBuf[2 * i + 1] = v;
      phase += inc;
    }
    size_t w = 0, r = 0;
    i2s_channel_write(txH, txBuf, sizeof(txBuf), &w, 1000);
    i2s_channel_read(rxH, rxBuf, sizeof(rxBuf), &r, 1000);

    if (f != lastF) {
      lastF = f;
      s1 = s2 = 0;
      gN = 0;
    }
    float coeff = 2.0f * cosf(2.0f * PI * f / SAMPLE_RATE);
    int frames = (int)(r / 8);
    for (int i = 0; i < frames; i++) {
      float x = (float)(rxBuf[2 * i] >> 8) / 8388608.0f;
      dc += (x - dc) * 0.002f;
      x -= dc;
      float s0 = x + coeff * s1 - s2;
      s2 = s1;
      s1 = s0;
      if (++gN >= GOERTZEL_N) {
        float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        if (p < 0)
          p = 0;
        float a = 2.0f * sqrtf(p) / GOERTZEL_N;
        gToneAmp = a;
        gMicDb = 20.0f * log10f(a + 1e-7f);
        gFrameCnt++;
        s1 = s2 = 0;
        gN = 0;
      }
    }
  }
}

static volatile int encDelta = 0;
static volatile int encAcc = 0;
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encISR() {
  static uint8_t prev = 0;
  prev = ((prev << 2) | (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT)) & 0x0F;
  encAcc += ENC_TABLE[prev];
  if (encAcc >= ENC_STATES_PER_DETENT) {
    encDelta++;
    encAcc = 0;
  } else if (encAcc <= -ENC_STATES_PER_DETENT) {
    encDelta--;
    encAcc = 0;
  }
}

int takeEncoderDelta() {
  portENTER_CRITICAL(&encMux);
  int d = encDelta;
  encDelta = 0;
  portEXIT_CRITICAL(&encMux);
#if ENC_INVERT
  d = -d;
#endif
  return d;
}

static bool filt(uint8_t pin, uint8_t& cnt) {
  if (digitalRead(pin) == LIM_ACTIVE) {
    if (cnt < 5)
      cnt++;
  } else
    cnt = 0;
  return cnt >= 3;
}

static void motionSetNormalProfile() {
  stepper->setSpeedInHz(mmsToHz(MOVE_SPEED_MMS));
  stepper->setAcceleration((int32_t)lroundf(ACCEL_MMS2 * STEPS_PER_MM));
}

static void motionError(const char* msg) {
  stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
  mErr = msg;
  mState = M_ERROR;
}

void motionTask(void*) {
  uint8_t cS = 0, cE = 0;
  uint32_t homeT0 = 0;
  bool endAtHomeStart = false;
  const uint32_t homeTimeout = (uint32_t)((TRAVEL_MM + 80.0f) / HOME_SPEED_MMS * 1000.0f * 1.4f);
  const int32_t backoffSteps = mmToSteps(BACKOFF_MM);
  const int32_t minSteps = mmToSteps(MARGIN_MM);
  const int32_t maxSteps = mmToSteps(TRAVEL_MM - MARGIN_MM);

  for (;;) {
    bool sStart = filt(PIN_LIM_START, cS);
    bool sEnd = filt(PIN_LIM_END, cE);

    if (reqHome) {
      reqHome = false;
      reqMove = false;
      mErr = nullptr;
      gHomed = false;
      endAtHomeStart = sEnd;
      homeT0 = millis();
      stepper->setSpeedInHz(mmsToHz(HOME_SPEED_MMS));
      stepper->setAcceleration((int32_t)lroundf(ACCEL_MMS2 * STEPS_PER_MM));
      mState = M_HOMING_SEEK;
      stepper->runBackward();
    }

    switch (mState) {
      case M_HOMING_SEEK:
        if (sStart) {
          stepper->forceStopAndNewPosition(0);
          stepper->setSpeedInHz(mmsToHz(HOME_SPEED_MMS));
          stepper->moveTo(backoffSteps);
          mState = M_HOMING_BACKOFF;
        } else if (!endAtHomeStart && sEnd) {
          motionError("SAI HUONG? DOI DIR");
        } else if (millis() - homeT0 > homeTimeout) {
          motionError("HOME QUA THOI GIAN");
        }
        break;

      case M_HOMING_BACKOFF:
        if (!stepper->isRunning() && labs(stepper->getCurrentPosition() - backoffSteps) <= 1) {
          motionSetNormalProfile();
          gHomed = true;
          mState = M_READY;
        }
        break;

      case M_READY:
        if (sStart || sEnd) {
          motionError(sStart ? "CHAM CONG TAC DAU" : "CHAM CONG TAC CUOI");
          break;
        }
        if (reqMove) {
          reqMove = false;
          int32_t t = reqTarget;
          if (t < minSteps)
            t = minSteps;
          if (t > maxSteps)
            t = maxSteps;
          stepper->moveTo(t);
        }
        break;

      default:
        break;
    }
    vTaskDelay(2);
  }
}

void requestMove(int32_t steps) {
  reqTarget = steps;
  reqMove = true;
}

int gField = 0;
int gHarmSel = 0;
float gTempC = 28.0f;
TubeType gType = TUBE_CLOSED_OPEN;
Resonance gRes;
bool dirty = true;
uint32_t lastChange = 0;
MState prevMs = M_UNHOMED;

char statusMsg[26] = "";
uint32_t statusUntil = 0;
void setStatus(const char* m, uint32_t ms = 3000) {
  strncpy(statusMsg, m, sizeof(statusMsg) - 1);
  statusMsg[sizeof(statusMsg) - 1] = 0;
  statusUntil = millis() + ms;
}

enum ScanState : uint8_t {
  SC_OFF,
  SC_GOTO,
  SC_SETTLE,
  SC_MEASURE,
  SC_FINISH
};
struct ScanCtx {
  ScanState st = SC_OFF;
  int32_t start, end, pos, best, center;
  float bestAmp, acc;
  int cnt;
  uint32_t t0, lastFrame;
} sc;

void abortScan() {
  sc.st = SC_OFF;
  setStatus("DA HUY QUET");
}

void startScan() {
  if (mState != M_READY || !gRes.ok) {
    setStatus("KHONG QUET DUOC");
    return;
  }
  int32_t minS = mmToSteps(MARGIN_MM), maxS = mmToSteps(TRAVEL_MM - MARGIN_MM);
  sc.center = mmToSteps(gRes.lenMm - L_MIN_MM);
  sc.start = max(minS, sc.center - mmToSteps(SCAN_RANGE_MM));
  sc.end = min(maxS, sc.center + mmToSteps(SCAN_RANGE_MM));
  sc.pos = sc.start;
  sc.best = sc.center;
  sc.bestAmp = -1;
  gSpkOn = true;
  sc.st = SC_GOTO;
  requestMove(sc.pos);
}

void scanUpdate(uint32_t now) {
  if (sc.st == SC_OFF)
    return;
  if (mState != M_READY) {
    sc.st = SC_OFF;
    return;
  }
  bool stopped = !stepper->isRunning();
  int32_t cur = stepper->getCurrentPosition();
  switch (sc.st) {
    case SC_GOTO:
      if (stopped && labs(cur - sc.pos) <= 2) {
        sc.t0 = now;
        sc.st = SC_SETTLE;
      }
      break;
    case SC_SETTLE:
      if (now - sc.t0 > 600) {
        sc.cnt = -1;
        sc.acc = 0;
        sc.lastFrame = gFrameCnt;
        sc.st = SC_MEASURE;
      }
      break;
    case SC_MEASURE:
      if (gFrameCnt != sc.lastFrame) {
        sc.lastFrame = gFrameCnt;
        if (sc.cnt >= 0)
          sc.acc += gToneAmp;
        sc.cnt++;
        if (sc.cnt >= 5) {
          float avg = sc.acc / 5.0f;
          if (avg > sc.bestAmp) {
            sc.bestAmp = avg;
            sc.best = sc.pos;
          }
          sc.pos += mmToSteps(SCAN_STEP_MM);
          if (sc.pos > sc.end) {
            sc.st = SC_FINISH;
            requestMove(sc.best);
          } else {
            sc.st = SC_GOTO;
            requestMove(sc.pos);
          }
        }
      }
      break;
    case SC_FINISH:
      if (stopped && labs(cur - sc.best) <= 2) {
        char b[26];
        float d = stepsToMm(sc.best - sc.center);
        snprintf(b, sizeof(b), "QUET XONG lech %+.1fmm", d);
        setStatus(b, 6000);
        sc.st = SC_OFF;
      }
      break;
    default:
      break;
  }
}

const char* holdHint(uint32_t h) {
  if (h < 700)
    return "Tha: DOI MUC";
  if (h < 2000)
    return "Tha: BAT/TAT LOA";
  if (h < 5000)
    return "Tha: QUET TINH CHINH";
  return "Tha: HOME LAI";
}

void drawUI(uint32_t now, uint32_t holdMs) {
  char b[40];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  auto sel = [&](int i) {
    return gField == i ? '>' : ' ';
  };
  const int Y0 = 7, DY = 8;

  snprintf(b, sizeof(b), "%cTan so : %4d Hz", sel(0), (int)lroundf(gFreq));
  u8g2.drawStr(0, Y0, b);

  if (gHarmSel == 0)
    snprintf(b, sizeof(b), "%cHoa am : Tu dong (n=%d)", sel(1), gRes.n);
  else
    snprintf(b, sizeof(b), "%cHoa am : Thu cong n=%d", sel(1), gHarmSel);
  u8g2.drawStr(0, Y0 + DY, b);

  snprintf(b, sizeof(b), "%cNhiet  : %2.0fC v=%.1fm/s", sel(2), gTempC, gRes.v);
  u8g2.drawStr(0, Y0 + 2 * DY, b);

  snprintf(b, sizeof(b), "%cOng    : %s", sel(3),
           gType == TUBE_CLOSED_OPEN ? "1 kin - 1 ho" : "2 dau ho");
  u8g2.drawStr(0, Y0 + 3 * DY, b);

  if (gRes.ok)
    snprintf(b, sizeof(b), " Muc tieu: %6.2f cm", gRes.lenMm / 10.0f);
  else
    snprintf(b, sizeof(b), " Muc tieu:   ---");
  u8g2.drawStr(0, Y0 + 4 * DY, b);

  if (gHomed)
    snprintf(b, sizeof(b), " Hien tai: %6.2f cm",
             (L_MIN_MM + stepsToMm(stepper->getCurrentPosition())) / 10.0f);
  else
    snprintf(b, sizeof(b), " Hien tai:   ---");
  u8g2.drawStr(0, Y0 + 5 * DY, b);

  snprintf(b, sizeof(b), " Loa:%s  Mic:%4.0fdB", gSpkOn ? "ON " : "OFF", (float)gMicDb);
  u8g2.drawStr(0, Y0 + 6 * DY, b);

  const char* st;
  char tmp[30];
  MState ms = mState;
  if (ms == M_ERROR) {
    snprintf(tmp, sizeof(tmp), "LOI:%s", mErr ? mErr : "?");
    st = tmp;
  } else if (holdMs > 300)
    st = holdHint(holdMs);
  else if (now < statusUntil)
    st = statusMsg;
  else if (sc.st != SC_OFF) {
    int range = max<int32_t>(1, sc.end - sc.start);
    int pct = constrain((int)(100L * (sc.pos - sc.start) / range), 0, 100);
    snprintf(tmp, sizeof(tmp), "QUET %d%%", pct);
    st = tmp;
  } else if (ms == M_HOMING_SEEK || ms == M_HOMING_BACKOFF)
    st = "DANG HOME...";
  else if (ms == M_UNHOMED)
    st = "CHUA HOME (giu >5s)";
  else if (!gRes.ok)
    st = gRes.msg;
  else if (stepper->isRunning())
    st = "DANG DI CHUYEN";
  else
    st = "SAN SANG";
  u8g2.drawBox(0, 56, 128, 8);
  u8g2.setDrawColor(0);
  u8g2.drawStr(1, 63, st);
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

void applyEncoder(int d) {
  switch (gField) {
    case 0: {
      int step = (abs(d) >= 2) ? 5 : 1;
      float f = gFreq + d * step;
      gFreq = constrain(f, (float)FREQ_MIN, (float)FREQ_MAX);
      break;
    }
    case 1:
      gHarmSel = constrain(gHarmSel + d, 0, MAX_HARM_SEL);
      break;
    case 2:
      gTempC = constrain(gTempC + d, -10.0f, 50.0f);
      break;
    case 3:
      gType = (gType == TUBE_CLOSED_OPEN) ? TUBE_OPEN_OPEN : TUBE_CLOSED_OPEN;
      break;
  }
  dirty = true;
  lastChange = millis();
}

void onButtonRelease(uint32_t hold) {
  if (sc.st != SC_OFF) {
    abortScan();
    return;
  }
  if (hold < 700) {
    gField = (gField + 1) % 4;
  } else if (hold < 2000) {
    gSpkOn = !gSpkOn;
    setStatus(gSpkOn ? "LOA BAT" : "LOA TAT", 1500);
  } else if (hold < 5000) {
    startScan();
  } else {
    reqHome = true;
    setStatus("HOME LAI...", 1500);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_LIM_START, INPUT_PULLUP);
  pinMode(PIN_LIM_END, INPUT_PULLUP);

  Wire.begin(21, 22);
  u8g2.begin();
  u8g2.setBusClock(400000);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "Ong cong huong");
  u8g2.drawStr(0, 26, "Dang khoi dong...");
  u8g2.sendBuffer();

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper) {
    Serial.println("LOI: khong tao duoc stepper");
    while (true)
      delay(1000);
  }
  stepper->setDirectionPin(PIN_DIR, DIR_HIGH_COUNTS_UP);
  stepper->setEnablePin(PIN_EN, true);
  stepper->setAutoEnable(true);
  stepper->setDelayToDisable(3000);
  motionSetNormalProfile();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), encISR, CHANGE);

  audioInit();
  xTaskCreatePinnedToCore(audioTask, "audio", 6144, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(motionTask, "motion", 4096, nullptr, 4, nullptr, 0);

#if AUTO_HOME_ON_BOOT
  reqHome = true;
#endif
  Serial.println("San sang.");
}

void loop() {
  uint32_t now = millis();

  int d = takeEncoderDelta();
  if (d) {
    if (sc.st != SC_OFF)
      abortScan();
    else
      applyEncoder(d);
  }

  static bool raw = false, lastRaw = false, stable = false;
  static uint32_t tChange = 0, pressT0 = 0;
  raw = (digitalRead(PIN_ENC_SW) == LOW);
  if (raw != lastRaw) {
    lastRaw = raw;
    tChange = now;
  }
  if (now - tChange > 25 && raw != stable) {
    stable = raw;
    if (stable)
      pressT0 = now;
    else
      onButtonRelease(now - pressT0);
  }
  uint32_t holdMs = stable ? (now - pressT0) : 0;

  gRes = computeResonance(gFreq, gHarmSel, gType, gTempC);

  MState ms = mState;
  if (ms == M_READY && prevMs != M_READY) {
    dirty = true;
    lastChange = now - 1000;
  }
  prevMs = ms;
  if (dirty && ms == M_READY && sc.st == SC_OFF && now - lastChange > 350) {
    if (gRes.ok) {
      requestMove(mmToSteps(gRes.lenMm - L_MIN_MM));
      Serial.printf("f=%.0fHz n=%d v=%.1fm/s L=%.1fmm\n", (float)gFreq, gRes.n, gRes.v, gRes.lenMm);
    }
    dirty = false;
  }

  scanUpdate(now);

  static uint32_t lastDraw = 0;
  if (now - lastDraw >= 100) {
    lastDraw = now;
    drawUI(now, holdMs);
  }

  delay(5);
}
