// Heating control parameters and constants
const byte  thermistorpin = A7;
const byte  heaterpin     = 12;

const float Tset  = 35.0;       // Setpoint temperature in °C
const float deltaT = 0.5;       // Hysteresis band in °C
const float Vcc   = 3.3;        // ADC reference voltage
const float R     = 10000.0;    // Series resistor (Ω)
const float Ro    = 10000.0;    // Thermistor nominal resistance at To (Ω)
const float To    = 25.0;       // Reference temperature (°C)
const float beta  = 3700.0;     // Beta coefficient
const float Kadc  = 3.3 / 4095; // ADC LSB (V/count)

float Vadc, T, Rth;
int   currtime, prevtime, T1, T2;
bool  heater, prevheater;

void setupHeating() {
  pinMode(thermistorpin, INPUT);
  pinMode(heaterpin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set PWM frequency and resolution
  ledcSetup(1, 500, 10);
  ledcAttachPin(heaterpin, 1);

  Serial.begin(2000000);

  T1 = millis();
  T2 = T1;
}

void loopHeating() {
  currtime = millis();

  // 100 ms update period
  if (currtime - T1 > 0) {
    prevtime = currtime;
    T1 = T1 + 100;

    Vadc = Kadc * analogRead(thermistorpin);

    // Calculate thermistor resistance from ADC voltage
    Rth = R * Vadc / (Vcc - Vadc);

    // Calculate temperature from thermistor resistance (°C)
    T = (To + 273.0) * beta / (beta + (To + 273.0) * log(Rth / Ro)) - 273.0;

    // Temp correction (TO BE CHANGED FOR FINAL CALIBRATION)
    T = T - 1.5;

    // Bang-bang control with hysteresis
    if (T < Tset - deltaT) {
      heater = 1;
    }
    if (T > Tset + deltaT) {
      heater = 0;
    }

    // Update PWM only when state changes, limit heater power to ~30 W
    if (heater != prevheater) {
      ledcWrite(1, heater * 639);
      prevheater = heater;
    }

    // Live monitoring for Vadc, Rth, and T
    if (currtime - T2 > 0) {
    T2 = T2 + 1000;

    Serial.print("Vadc = ");
    Serial.print(Vadc, 4);

    Serial.print("   Rth = ");
    Serial.print(Rth);

    Serial.print("   Temp = ");
    Serial.print(T, 2);
    Serial.println(" C");
    }
  }
}

double getTemperature() {  // in Celsius
  return T;
}

void setTemperatureSetpoint(double temperature) {
  tSet = temperature;
}

double getTemperatureSetpoint() {
  return tSet;  
}





