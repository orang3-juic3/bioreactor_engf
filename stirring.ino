namespace StirringImpl {

/*
  Bioreactor Stirring Control Module

  Purpose: Closed-loop RPM control for bioreactor stirring motor with Hall sensor feedback
  Board: ESP32 (esp32:esp32:esp32)

  Important: This module provides setupStirring() and loopStirring() functions.
  The top-level setup() and loop() are defined in bioreactor.ino and call these functions.

  Hardware:
  - Hall sensor (70 pulses/rev) on GPIO 34
  - Motor PWM output on GPIO 25
  - Fault LED on GPIO 2

  Control:
  - PI controller (Kp=0.60, Ki=0.80)
  - PWM: 20 kHz, 12-bit resolution
  - RPM sampling: 300 ms window
  - Non-blocking timing with millis()
*/

/*
  Hardware verification checklist (if motor does not respond):
  - Hall sensor Vcc must be 3.3V (ESP32 pins are not 5V tolerant)
  - MOSFET gate should have ~10k pulldown and ~100Ω series gate resistor
  - Motor and ESP32 must share common ground
  - Verify Hall sensor wiring and sensor type (open-collector vs active output)
*/

#include <Arduino.h>

#if !defined(ESP32)
#error "This code requires ESP32 platform"
#endif

// -------------------- Pin Configuration --------------------
static const uint8_t PIN_HALL      = 5;
static const uint8_t PIN_MOTOR_PWM = 21;
static const uint8_t PIN_FAULT_LED = LED_BUILTIN;

// -------------------- PWM Configuration --------------------
static const uint8_t  PWM_CHANNEL   = 0;         // LEDC channel (0-15)
static const uint32_t PWM_FREQ_HZ   = 20000;
static const uint8_t  PWM_RES_BITS  = 12;
static const uint32_t PWM_MAX_DUTY  = (1 << PWM_RES_BITS) - 1; // 4095

// -------------------- Hall Sensor / RPM --------------------
static const uint16_t HALL_PPR         = 70;        // pulses per revolution
static const uint32_t RPM_SAMPLE_MS    = 300;       // RPM calculation window
static const uint32_t STALL_TIMEOUT_MS = 600;       // stall detection threshold

// -------------------- Control Parameters --------------------
static float Kp = 0.60f;
static float Ki = 0.80f;
static float Kd = 0.00f;

static const float RPM_SETPOINT_DEF = 600.0f;
static const float RPM_MAX_CAP      = 1200.0f;
static const float DEADBAND_IN_RPM  = 6.0f;
static const float DEADBAND_OUT_RPM = 8.0f;
static const float INTEGRAL_CLAMP   = 400.0f;

// Stiction threshold: scale an 8-bit intuition (180/255) into 12-bit domain
static const uint16_t STICTION_DUTY_8 = 180;
static const uint16_t STICTION_DUTY   = (uint16_t)((STICTION_DUTY_8 / 255.0f) * PWM_MAX_DUTY);
// Simple linear first-pass mapping from RPM to duty. This is conservative and tunable.
static const float DUTY_PER_RPM = (float)PWM_MAX_DUTY / RPM_MAX_CAP;
static const int DUTY_SLEW_PER_10MS = 50;

// -------------------- State Variables --------------------
volatile uint32_t g_pulseCount  = 0;
volatile uint32_t g_lastPulseUs = 0;

static float    g_rpm         = 0.0f;
static float    g_setpointRpm = RPM_SETPOINT_DEF;
static float    g_errPrev     = 0.0f;
static float    g_errI        = 0.0f;
static uint32_t g_pwmDuty     = 0;
static bool     g_inDeadband  = false;

static uint32_t t_lastRpmMs   = 0;
static uint32_t t_lastCtrlMs  = 0;
static uint32_t t_lastDebugMs = 0;

// ISR 
void IRAM_ATTR hallISR() {
  g_pulseCount++;
  g_lastPulseUs = micros();
}

// Helper Functions 
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline void motorSetDuty(uint32_t duty) {
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
  g_pwmDuty = duty;
  ledcWrite(PWM_CHANNEL, duty);
}

static inline float computeRPM(uint32_t pulses, uint32_t windowMs) {
  if (windowMs == 0) return 0.0f;
  return ((float)pulses / (float)HALL_PPR) * (60000.0f / (float)windowMs);
}

static inline uint32_t limitDutySlew(uint32_t current, int target, int maxStep) {
  int delta = target - (int)current;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  int out = (int)current + delta;
  if (out < 0) out = 0;
  if (out > (int)PWM_MAX_DUTY) out = PWM_MAX_DUTY;
  return (uint32_t)out;
}
}

// Getter Functions for Main File 

double getRPM() {
  return (double)StirringImpl::g_rpm;
}

double getRPMSetpoint() {
  return (double)StirringImpl::g_setpointRpm;
}

// Placeholder for photoevents 
double getPhotoevents() {
  return 0.0;
}
double getMotorPWM() {
    // Return normalized PWM value (0.0 to 1.0)
    // PWM is 12-bit (0-4095)
    return (double)StirringImpl::g_pwmDuty / (double)StirringImpl::PWM_MAX_DUTY; // placeholder Make sure you normalise !
}

// Setter Functions

void setStirringSetpoint(float rpm) {
  StirringImpl::g_setpointRpm = StirringImpl::clampf(rpm, 0.0f, StirringImpl::RPM_MAX_CAP);
}

void setPidGains(float kp, float ki, float kd) {
  StirringImpl::Kp = kp;
  StirringImpl::Ki = ki;
  StirringImpl::Kd = kd;
}

void setupStirring() {
  using namespace StirringImpl;
  // Hall sensor: internal pull-up (sensor likely open-collector) and count both edges
  pinMode(PIN_HALL, INPUT_PULLUP);
  pinMode(PIN_FAULT_LED, OUTPUT);
  digitalWrite(PIN_FAULT_LED, LOW);

  // Setup PWM using ledcSetup and ledcAttachPin (Arduino Nano ESP32 compatible)
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, PWM_CHANNEL);
  motorSetDuty(0);

  // Attach Hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, CHANGE);
  g_lastPulseUs = micros();

  t_lastRpmMs  = millis();
  t_lastCtrlMs = millis();
  t_lastDebugMs = millis();

  Serial.println("Stirring subsystem initialized");
}

void loopStirring() {
  using namespace StirringImpl;
  uint32_t t_now_ms = millis();

  // RPM Estimation 
  if ((t_now_ms - t_lastRpmMs) >= RPM_SAMPLE_MS) {
    noInterrupts();
    uint32_t pulses = g_pulseCount;
    g_pulseCount = 0;
    interrupts();

    g_rpm = computeRPM(pulses, t_now_ms - t_lastRpmMs);
    t_lastRpmMs = t_now_ms;
  }

  // Stall Detection - only trigger if motor should be running but isn't
  uint32_t lastPulseMs = g_lastPulseUs / 1000;
  const uint32_t STALL_DUTY_MIN = (uint32_t)(0.05f * PWM_MAX_DUTY); // 5% of full-scale, ~205 for 12-bit
  bool stalled = (g_setpointRpm > 10.0f) && (g_pwmDuty > STALL_DUTY_MIN) &&
                 (t_now_ms > lastPulseMs) && ((t_now_ms - lastPulseMs) > STALL_TIMEOUT_MS);

  if (stalled) {
    motorSetDuty(0);
    digitalWrite(PIN_FAULT_LED, HIGH);
  } else {
    digitalWrite(PIN_FAULT_LED, LOW);
  }

  // Control Loop (10ms interval) 
  if (!stalled && (t_now_ms - t_lastCtrlMs) >= 10) {
    // compute dt in seconds using the actual control period (before updating t_lastCtrlMs)
    float dt = (t_now_ms - t_lastCtrlMs) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;
    t_lastCtrlMs = t_now_ms;

    float sp   = (g_setpointRpm < RPM_MAX_CAP) ? g_setpointRpm : RPM_MAX_CAP;
    float err  = sp - g_rpm;
    float eabs = (err < 0) ? -err : err;

    // Deadband with hysteresis
    if (!g_inDeadband && eabs <= DEADBAND_IN_RPM)  g_inDeadband = true;
    if ( g_inDeadband && eabs >= DEADBAND_OUT_RPM) g_inDeadband = false;

    int targetDuty = (int)g_pwmDuty;

    if (!g_inDeadband) {
      bool atUpper = (g_pwmDuty >= PWM_MAX_DUTY - 1);
      bool atLower = (g_pwmDuty <= 1);

      // anti-windup: integrate only when not saturated
      if (!atUpper && !atLower) {
        g_errI += err * Ki * dt;
        g_errI = clampf(g_errI, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
      }

      float u_rpm  = Kp * err + g_errI;
      float u_duty = u_rpm * DUTY_PER_RPM;

      targetDuty = (int)((float)g_pwmDuty + u_duty);

      // Stiction compensation
      if (sp > 1.0f && targetDuty > 0 && targetDuty < (int)STICTION_DUTY) {
        targetDuty = STICTION_DUTY;
      }

      if (targetDuty < 0) targetDuty = 0;
      if (targetDuty > (int)PWM_MAX_DUTY) targetDuty = PWM_MAX_DUTY;

      g_errPrev = err;
    }

    // Slew-limit and apply
    uint32_t nextDuty = limitDutySlew(g_pwmDuty, targetDuty, DUTY_SLEW_PER_10MS);
    motorSetDuty(nextDuty);
  }

  // Debug telemetry (1s interval)
  if ((t_now_ms - t_lastDebugMs) >= 1000) {
    t_lastDebugMs = t_now_ms;
    Serial.printf("STIR: RPM=%.1f SP=%.1f PWM=%u I=%.2f\n", g_rpm, g_setpointRpm, g_pwmDuty, g_errI);
  }

  // Serial Tuning Commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'u') {
      g_setpointRpm += 50.0f;
      g_inDeadband = false;  // Exit deadband on setpoint change
    }
    if (c == 'd') {
      g_setpointRpm -= 50.0f;
      g_inDeadband = false;  // Exit deadband on setpoint change
    }
    if (c == 'r') {
      Kp = 0.6f;
      Ki = 0.8f;
      Kd = 0.0f;
      g_errI = 0.0f;
      g_inDeadband = false;
    }
    g_setpointRpm = clampf(g_setpointRpm, 0.0f, RPM_MAX_CAP);
    Serial.printf("SP=%.1f RPM  Kp=%.2f Ki=%.2f Kd=%.2f\n", g_setpointRpm, Kp, Ki, Kd);
  }
}