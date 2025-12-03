namespace pHImpl {



double pH=0, pHsmoothed=0, sensorValue=0, inputVoltage=0, error=0, integralError = 0;
double goalpH = 7.0;


const byte pHPin=A0, acidPumpPin=3, basePumpPin=9; // pins need to be the same on breadboard
//acidpump is pump B and basepump is A for now
const double KpH = 3.5;
const double k = 5.0/4095.0;
const double offset = -4.0; // depends on calibrated pH value
const double deadband = 0.1;
const double Kp = 150.0, Ki = 0.5; // pi controller constants
const double temp_calib = 25.0 + 273.15;
double temp_curr= 25.0 + 273.15;
unsigned long Timems, T2;
int pwmSignal;


}

void setupPH() {
  using namespace pHImpl;
  pinMode(pHPin,INPUT);
  pinMode(acidPumpPin, OUTPUT);
  pinMode(basePumpPin, OUTPUT);
  Serial.begin(2000000);
  pHsmoothed = analogRead(pHPin) * k * KpH;
}
 
void loopPH() {
  using namespace pHImpl;
  Timems=millis();
  if(Timems-T2>=3000) { // run every 3 seconds
    // measure current pH value

    inputVoltage = analogRead(pHPin);
    sensorValue = k * inputVoltage; // ph circuit value 0-5
    pH = KpH * temp_calib/temp_curr * sensorValue + offset;
    pHsmoothed = 0.9*pHsmoothed + 0.1*pH; // IIR smoothing to reduce noise

    Serial.println(pHsmoothed);

    // PI control starts here

    error = pHsmoothed - goalpH;
    integralError += error * (Timems - T2)/1000.0;


    T2 = Timems;

    if (abs(error) < deadband){
      analogWrite(acidPumpPin, 0);
      analogWrite(basePumpPin, 0);
      integralError = 0;
    }
    else{
      pwmSignal = (int)abs(Kp * error + Ki * integralError); // pi controller calculation
      pwmSignal = constrain(pwmSignal, 0, 255);
      
      if(error > 0){
        analogWrite(acidPumpPin, pwmSignal);
        analogWrite(basePumpPin, 0);
      }
      else{
        analogWrite(basePumpPin, pwmSignal);
        analogWrite(acidPumpPin, 0);
      }
    }
  }
}
double getPH(){
  return pHImpl::pHsmoothed;
}

double getpHSetpoint(){
    return pHImpl::goalpH;
}

void setTargetpH(double targetpH){
    pHImpl::goalpH = targetpH;
}
double getAcidPWM() {
    // Return normalized PWM value (0. 0 to 1.0)
    return pHImpl::error > 0 ? pHImpl::pwmSignal / 255.0 : 0.0;
}

double getBasePWM() {
    // Return normalized PWM value (0.0 to 1.0)
    return pHImpl::error < 0 ? pHImpl::pwmSignal / 255.0 : 0.0;
}

