/*
  Stirring control (ESP32 / Arduino IDE)
  Closed-loop RPM control with Hall feedback, PID (+deadband), stall detection,
  high-frequency PWM via LEDC, and UART telemetry to the ESP32 Node.

  This file implements the functions you provided:
    - setupStirring()
    - loopStirring()
    - getTemperature()          // placeholder (not used by stirring)
    - setTemperatureSetpoint()  // placeholder (not used by stirring)

  Notes
  - Hall sensor: 70 pulses/rev.
  - RPM window: 300 ms (tune 250–500 ms).
  - PWM: 20 kHz on LEDC channel 0.
  - Safety: max-RPM cap + stall timeout → PWM=0 + alarm.
  - Telemetry: JSON lines over Serial2 (UART) to the ESP32 Node.
*/

#include <Arduino.h>
#include <math.h>

// ---------------------------- Pinout (EDIT AS NEEDED) ----------------------------
static const int PIN_HALL      = 34;   // Hall input (GPIO34 is input-only; external pull-up as needed)
static const int PIN_MOTOR_PWM = 25;   // PWM to motor driver
static const int PIN_MOTOR_EN  = -1;   // Optional enable to driver; set -1 if unused
static const int PIN_FAULT_LED = 2;    // On-board LED for simple alarm

// ---------------------------- LEDC (PWM) config ---------------------------------
static const int PWM_CHANNEL  = 0;
static const int PWM_FREQ_HZ  = 20000;          // 20 kHz
static const int PWM_RES_BITS = 12;             // 0..4095
static const int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// ---------------------------- Hall / RPM config ---------------------------------
static const uint16_t HALL_PPR         = 70;    // pulses per revolution
static const uint32_t RPM_SAMPLE_MS    = 300;   // compute RPM every 300 ms
static const uint32_t STALL_TIMEOUT_MS = 600;   // no pulses -> stall

// ---------------------------- Control config ------------------------------------
static float g_KP = 0.50f;   // PID gains (start PI, then tune to PID)
static float g_KI = 0.80f;
static float g_KD = 0.00f;

static const float INTEGRAL_CLAMP   = 400.0f;   // integral limit (maps to duty units)
static const float DEADBAND_RPM     = 6.0f;     // ~2× quantisation step at 300 ms window
static const float RPM_SETPOINT_DEF = 600.0f;   // default setpoint
static const float RPM_MAX_CAP      = 1200.0f;  // shear/safety limit

// ---------------------------- Telemetry (UART to Node) ---------------------------
#define USE_UART_TELEMETRY 1
#if USE_UART_TELEMETRY
  static const uint32_t UART_BAUD   = 115200;
  static const int      UART_TX_PIN = 17;
  static const int      UART_RX_PIN = 16;
  static const uint32_t TELEMETRY_MS = 500;     // ~2 Hz
#endif

// ---------------------------- State variables -----------------------------------
volatile uint32_t g_pulseCount = 0;       // incremented in ISR
volatile uint32_t g_lastPulseMs = 0;      // millis() at last pulse

static float    g_rpm         = 0.0f;
static float    g_rpmSetpoint = RPM_SETPOINT_DEF;
static float    g_errPrev     = 0.0f;
static float    g_errI        = 0.0f;
static uint16_t g_pwmDuty     = 0;        // 0..PWM_MAX_DUTY

// timers
static uint32_t t_now_ms = 0;
static uint32_t t_lastRpmMs = 0;
static uint32_t t_lastCtrlMs = 0;
static uint32_t t_lastTelemMs = 0;

// ---------------------------- ISR ------------------------------------------------
void IRAM_ATTR isr_hall() {
  g_pulseCount++;
  g_lastPulseMs = millis();
}

// ---------------------------- Helpers -------------------------------------------
static inline void motorEnable(bool en) {
  if (PIN_MOTOR_EN >= 0) {
    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_EN, en ? HIGH : LOW);
  }
}

static inline void motorWriteDuty(uint16_t duty) {
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  g_pwmDuty = duty;
  ledcWrite(PWM_CHANNEL, g_pwmDuty);
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float computeRpm(uint32_t pulses, uint32_t windowMs) {
  if (windowMs == 0) return 0.0f;
  // RPM = (pulses / PPR) * (60000 / windowMs)
  return ( (float)pulses / (float)HALL_PPR ) * (60000.0f / (float)windowMs);
}

// ================================================================================
// =                  IMPLEMENTATION OF YOUR SKELETON API                         =
// ================================================================================

void setupStirring() {
  // GPIO
  pinMode(PIN_HALL, INPUT);          // external pull-up or sensor open-collector as designed
  pinMode(PIN_FAULT_LED, OUTPUT);
  digitalWrite(PIN_FAULT_LED, LOW);

  // PWM (LEDC)
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, PWM_CHANNEL);
  motorEnable(true);
  motorWriteDuty(0);

  // Hall interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), isr_hall, RISING);
  g_lastPulseMs = millis();

  // Timers
  t_now_ms = millis();
  t_lastRpmMs = t_now_ms;
  t_lastCtrlMs = t_now_ms;
  t_lastTelemMs = t_now_ms;

  // UART to ESP32 Node
  #if USE_UART_TELEMETRY
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  #endif
}

void loopStirring() {
  t_now_ms = millis();

  // --- RPM calculation every RPM_SAMPLE_MS ---
  if ((t_now_ms - t_lastRpmMs) >= RPM_SAMPLE_MS) {
    noInterrupts();
    uint32_t pulses = g_pulseCount;
    g_pulseCount = 0;
    interrupts();

    g_rpm = computeRpm(pulses, (t_now_ms - t_lastRpmMs));
    t_lastRpmMs = t_now_ms;
  }

  // --- Stall detection ---
  bool stall = (t_now_ms - g_lastPulseMs) > STALL_TIMEOUT_MS;
  if (stall) {
    motorWriteDuty(0);
    digitalWrite(PIN_FAULT_LED, HIGH);
  } else {
    digitalWrite(PIN_FAULT_LED, LOW);
  }

  // --- PID speed control (every 10 ms), skip while stalled ---
  if (!stall && (t_now_ms - t_lastCtrlMs) >= 10) {
    t_lastCtrlMs = t_now_ms;

    // Safety cap on setpoint
    float sp = fminf(g_rpmSetpoint, RPM_MAX_CAP);
    float err = sp - g_rpm;
    float errAbs = fabsf(err);

    // Deadband: hold output & freeze integral in a small range to avoid chatter
    if (errAbs > DEADBAND_RPM) {
      // PI (KD often = 0 for speed control)
      g_errI += err * 0.01f * g_KI;             // dt=10 ms = 0.01 s
      g_errI = clampf(g_errI, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

      float dErr = (err - g_errPrev) / 0.01f;   // derivative term (per second)
      float u = g_KP * err + g_errI + g_KD * dErr;

      // Convert controller output to duty change; start simple and tune scale
      int duty = (int)g_pwmDuty + (int)u;
      duty = (int)clampf((float)duty, 0.0f, (float)PWM_MAX_DUTY);
      motorWriteDuty((uint16_t)duty);

      g_errPrev = err;
    }
    // else in deadband: hold duty, keep integrator as-is (or add a very slow bleed if needed)
  }

  // --- Telemetry (non-blocking) ---
  #if USE_UART_TELEMETRY
  if ((t_now_ms - t_lastTelemMs) >= TELEMETRY_MS) {
    t_lastTelemMs = t_now_ms;
    // Small JSON line for the ESP32 Node
    Serial2.print("{\"rpm\":");     Serial2.print(g_rpm, 1);
    Serial2.print(",\"duty\":");    Serial2.print(g_pwmDuty);
    Serial2.print(",\"sp\":");      Serial2.print(g_rpmSetpoint, 1);
    Serial2.print(",\"stall\":");   Serial2.print(stall ? 1 : 0);
    Serial2.println("}");
  }
  #endif
}

// ---------------------------- Placeholders to match your skeleton ----------------
void getTemperature() {
  // Stirring subsystem does not read temperature directly.
}

void setTemperatureSetpoint() {
  // Not used here (belongs to Heating subsystem).
}

// ---------------------------- Optional helpers for tuning -----------------------
void setStirringSetpoint(float rpm) {
  g_rpmSetpoint = clampf(rpm, 0.0f, RPM_MAX_CAP);
}

void setPidGains(float kp, float ki, float kd) {
  g_KP = kp;
  g_KI = ki;
  g_KD = kd;
}
