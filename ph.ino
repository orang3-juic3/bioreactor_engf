const byte pHPin=A0, acidPumpPin=3, basePumpPin=9; // pins need to be the same on breadboard
//acidpump is pump B and basepump is A for now
const float KpH = 3.5;
const float k = 5.0/4095.0;
const float offset = -4.0; // depends on calibrated pH value
float goalpH = 7.0;
const float deadband = 0.1;
const float Kp = 150.0, Ki = 0.5; // pi controller constants
float pH=0, pHsmoothed=0, sensorValue=0, inputVoltage=0, error=0, integralError = 0;
const float temp_calib = 25.0 + 273.15;
float temp_curr= 25.0 + 273.15;
unsigned long Timems, T2;
int pwmSignal;
 
void setupPH() {
  pinMode(pHPin,INPUT);
  pinMode(acidPumpPin, OUTPUT);
  pinMode(basePumpPin, OUTPUT);
  Serial.begin(2000000);
  pHsmoothed = analogRead(pHPin) * k * KpH;
}

float getpH(){
    return pHsmoothed;
}

float getpHSetPoint(){
    return goalpH;
}

void setTargetpH(float targetpH){
    goalpH = targetpH;
}
 
void loopPH() {
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