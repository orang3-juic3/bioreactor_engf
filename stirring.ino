/*
  Stirring control (ESP32 / Arduino IDE)
  Closed-loop RPM control with Hall feedback, PI(D) with deadband + hysteresis,
  stall detection/recovery, LEDC PWM, and UART telemetry to the ESP32 Node.

  Assumptions:
  - Hall encoder: 70 pulses/rev
  - RPM window: 300 ms (tune 250–500 ms)
  - PWM: 20 kHz, 12-bit
  - UART to Node on Serial2 (115200)
*/

#include <Arduino.h>
#include <math.h>

#if !defined(ESP32)
#error "This code requires ESP32 platform"
#endif

// Pins
static const int PIN_HALL      = 34;
static const int PIN_MOTOR_PWM = 25;
static const int PIN_MOTOR_EN  = -1;
static const int PIN_FAULT_LED = 2;

// LEDC (PWM)
static const int PWM_FREQ_HZ   = 20000;
static const int PWM_RES_BITS  = 12;
static const int PWM_MAX_DUTY  = (1 << PWM_RES_BITS) - 1;

// Hall / RPM
static const uint16_t HALL_PPR         = 70;
static const uint32_t RPM_SAMPLE_MS    = 300;
static const uint32_t STALL_TIMEOUT_MS = 600;

// Control
static float Kp = 0.60f;
static float Ki = 0.80f;
static float Kd = 0.00f;

static const float RPM_SETPOINT_DEF = 600.0f;
static const float RPM_MAX_CAP      = 1200.0f;
static const float DEADBAND_IN_RPM  = 6.0f;
static const float DEADBAND_OUT_RPM = 8.0f;
static const float INTEGRAL_CLAMP   = 400.0f;

// Map controller output (RPM) to duty counts
static const float    DUTY_PER_RPM        = 1.5f;
static const uint16_t STICTION_DUTY       = 180;
static const int      DUTY_SLEW_PER_10MS  = 50;

// Telemetry (UART to ESP32 Node)
#define USE_UART_TELEMETRY 0
#if USE_UART_TELEMETRY
  static const uint32_t UART_BAUD    = 115200;
  static const int      UART_TX_PIN  = 17;
  static const int      UART_RX_PIN  = 16;
  static const uint32_t TELEMETRY_MS = 500;
#endif

// State
volatile uint32_t g_pulseCount = 0;
volatile uint32_t g_lastPulseUs = 0;

static float    g_rpm         = 0.0f;
static float    g_setpointRpm = RPM_SETPOINT_DEF;
static float    g_errPrev     = 0.0f;
static float    g_errI        = 0.0f;
static uint16_t g_pwmDuty     = 0;
static bool     g_inDeadband  = false;

static uint32_t t_now_ms = 0, t_lastRpmMs = 0, t_lastCtrlMs = 0, t_lastTelemMs = 0;

// ISR
void IRAM_ATTR isr_hall() {
  g_pulseCount++;
  g_lastPulseUs = micros();
}

// Helpers
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline void motorEnable(bool en) {
  if (PIN_MOTOR_EN >= 0) {
    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_EN, en ? HIGH : LOW);
  }
}

static inline void motorWriteDuty(uint16_t duty) {
  if ((int)duty < 0) duty = 0;
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  g_pwmDuty = duty;
  ledcWrite(PIN_MOTOR_PWM, g_pwmDuty);
}

static inline float computeRpm(uint32_t pulses, uint32_t windowMs) {
  if (windowMs == 0) return 0.0f;
  return ( (float)pulses / (float)HALL_PPR ) * (60000.0f / (float)windowMs);
}

static inline uint16_t limitDutySlew(uint16_t current, int target, int maxStep) {
  int delta = target - (int)current;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  int out = (int)current + delta;
  if (out < 0) out = 0;
  if (out > PWM_MAX_DUTY) out = PWM_MAX_DUTY;
  return (uint16_t)out;
}

// Public API

void setupStirring() {
  pinMode(PIN_HALL, INPUT);
  pinMode(PIN_FAULT_LED, OUTPUT);
  digitalWrite(PIN_FAULT_LED, LOW);

  ledcAttach(PIN_MOTOR_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
  motorEnable(true);
  motorWriteDuty(0);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL), isr_hall, RISING);
  g_lastPulseUs = micros();

  t_now_ms = millis();
  t_lastRpmMs = t_now_ms;
  t_lastCtrlMs = t_now_ms;
  t_lastTelemMs = t_now_ms;

#if USE_UART_TELEMETRY
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#endif
  Serial.begin(115200);
}

void loopStirring() {
  t_now_ms = millis();

  // RPM estimation
  if ((t_now_ms - t_lastRpmMs) >= RPM_SAMPLE_MS) {
    noInterrupts();
    uint32_t pulses = g_pulseCount;
    g_pulseCount = 0;
    interrupts();

    g_rpm = computeRpm(pulses, (t_now_ms - t_lastRpmMs));
    t_lastRpmMs = t_now_ms;
  }

  // Stall detection
  uint32_t lastPulseMs = g_lastPulseUs / 1000;
  bool stalled = (t_now_ms > lastPulseMs) && ((t_now_ms - lastPulseMs) > STALL_TIMEOUT_MS);
  if (stalled) {
    motorWriteDuty(0);
    digitalWrite(PIN_FAULT_LED, HIGH);
  } else {
    digitalWrite(PIN_FAULT_LED, LOW);
  }

  // Control loop
  if (!stalled && (t_now_ms - t_lastCtrlMs) >= 10) {
    t_lastCtrlMs = t_now_ms;

    float sp   = fminf(g_setpointRpm, RPM_MAX_CAP);
    float err  = sp - g_rpm;
    float eabs = fabsf(err);

    // deadband with hysteresis
    if (!g_inDeadband && eabs <= DEADBAND_IN_RPM)  g_inDeadband = true;
    if ( g_inDeadband && eabs >= DEADBAND_OUT_RPM) g_inDeadband = false;

    int targetDuty = (int)g_pwmDuty;

    if (!g_inDeadband) {
      bool atUpper = (g_pwmDuty >= PWM_MAX_DUTY - 1);
      bool atLower = (g_pwmDuty <= 1);

      // Freeze integral when saturated
      if (!atUpper && !atLower) {
        g_errI += err * 0.01f * Ki;
        g_errI = clampf(g_errI, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
      }

      float dErr    = (err - g_errPrev) / 0.01f;
      float u_rpm   = Kp * err + g_errI + Kd * dErr;
      float u_duty  = u_rpm * DUTY_PER_RPM;

      targetDuty = (int)((float)g_pwmDuty + u_duty);

      // Minimum duty to overcome stiction
      if (sp > 1.0f && targetDuty > 0 && targetDuty < (int)STICTION_DUTY)
        targetDuty = STICTION_DUTY;

      if (targetDuty < 0) targetDuty = 0;
      if (targetDuty > PWM_MAX_DUTY) targetDuty = PWM_MAX_DUTY;

      g_errPrev = err;
    }

    // Slew-limit and apply
    uint16_t nextDuty = limitDutySlew(g_pwmDuty, targetDuty, DUTY_SLEW_PER_10MS);
    motorWriteDuty(nextDuty);
  }

  // Telemetry to ESP32 Node
#if USE_UART_TELEMETRY
  if ((t_now_ms - t_lastTelemMs) >= TELEMETRY_MS) {
    t_lastTelemMs = t_now_ms;
    Serial2.print("{\"rpm\":");   Serial2.print(g_rpm, 1);
    Serial2.print(",\"duty\":");  Serial2.print(g_pwmDuty);
    Serial2.print(",\"sp\":");    Serial2.print(g_setpointRpm, 1);
    Serial2.print(",\"dead\":");  Serial2.print(g_inDeadband ? 1 : 0);
    uint32_t lastPulseMs = g_lastPulseUs / 1000;
    Serial2.print(",\"stall\":"); Serial2.print(((t_now_ms > lastPulseMs) && ((t_now_ms - lastPulseMs) > STALL_TIMEOUT_MS)) ? 1 : 0);
    Serial2.println("}");
  }
#endif

  // Serial tuning: 'u' +50 RPM, 'd' -50 RPM, 'r' reset gains
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'u') g_setpointRpm += 50;
    if (c == 'd') g_setpointRpm -= 50;
    if (c == 'r') { Kp = 0.6f; Ki = 0.8f; Kd = 0.0f; g_errI = 0; }
    g_setpointRpm = clampf(g_setpointRpm, 0.0f, RPM_MAX_CAP);
    Serial.printf("SP=%.1f RPM  Kp=%.2f Ki=%.2f Kd=%.2f\n", g_setpointRpm, Kp, Ki, Kd);
  }
}

// ---------------------------- Placeholders to match your skeleton ----------------
double getRPM() {
  return 1.0;
}

double getPhotoevents() {
  // This should reset the counter!
}

// ---------------------------- Optional helpers for tuning -----------------------
double getRPMSetpoint(){
  return 0.0;
}


// Public setters
void setStirringSetpoint(float rpm) { g_setpointRpm = clampf(rpm, 0.0f, RPM_MAX_CAP); }
void setPidGains(float kp, float ki, float kd) { Kp = kp; Ki = ki; Kd = kd; }
