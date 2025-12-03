


namespace HeatingImpl {

// Heating control parameters and constants
const byte  thermistorpin = A7;
const byte  heaterpin     = 12;

 float Tset  = 35.0;       // Setpoint temperature in °C
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
const float heaterPower = 30.0;  // Heater power in Watts (from line 76 comment)
unsigned long lastEnergyUpdate = 0;
double totalEnergyWh = 0.0;  // Accumulated energy in Watt-hours
}

void setTemperatureSetpoint(double temperature) {
  HeatingImpl::Tset = temperature;
}

double getTemperature() {  // in Celsius
  return HeatingImpl::T;
}

double getTemperatureSetpoint() {
  return HeatingImpl::Tset;  
}
double getHeaterPWM() {
    // Return normalized PWM value (0. 0 to 1.0)
    // PWM is 10-bit (0-1023), currently set to either 0 or 639
    return HeatingImpl::heater ?  639.0 / 1023.0 : 0.0;
}

double getHeaterEnergyWh() {
    using namespace HeatingImpl;
    
    // Update energy accumulation
    unsigned long currentTime = millis();
    if (lastEnergyUpdate > 0) {
        unsigned long deltaTime = currentTime - lastEnergyUpdate; // milliseconds
        if (heater) {
            // Energy = Power × Time
            // Convert ms to hours: deltaTime / (1000 ms/s × 3600 s/h)
            totalEnergyWh += heaterPower * deltaTime / 3600000.0;
        }
    }
    lastEnergyUpdate = currentTime;
    
    return totalEnergyWh;
}

void resetHeaterEnergy() {
    HeatingImpl::totalEnergyWh = 0.0;
    HeatingImpl::lastEnergyUpdate = millis();
}
void setupHeating() {
  #if HEATING
  using namespace HeatingImpl;
  pinMode(thermistorpin, INPUT);
  pinMode(heaterpin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set PWM frequency and resolution
  ledcSetup(1, 500, 10);
  ledcAttachPin(heaterpin, 1);

  Serial.begin(2000000);

  T1 = millis();
  T2 = T1;
  #endif
}

void loopHeating() {
  #if HEATING
  using namespace HeatingImpl;
  getHeaterEnergyWh();
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
  #endif
}






