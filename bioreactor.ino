// ArduinoJson lib req
#include <ArduinoJson.h>
#include <vector>
#include <math.h>

#define HEATING true
#define STIRRING true
#define PH true
void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");
  setupStirring();
  setupPH();
  setupHeating();
  Serial1.begin(115200, SERIAL_8N1, 19, 20);
  Serial.println("Started (Serial1 RX=19, TX=20)");
}


unsigned long last = 0;
int samples = 0;
std::vector<double> temp;
std::vector<double> pHlist;
std::vector<double> rpm;
std::vector<double> acidPwm;
std::vector<double> basePwm;
std::vector<double> heaterPwm;
std::vector<double> motorPwm;

void loop() {
  
  // rxPin = 17 (A3), txPin = 16 (A2)

  loopStirring();
  loopPH();
  loopHeating();
  //do looping things
  temp.push_back(getTemperature());

  pHlist.push_back(getPH());
  rpm.push_back(getRPM());
  acidPwm.push_back(getAcidPWM());
  basePwm.push_back(getBasePWM());
  heaterPwm.push_back(getHeaterPWM());
  motorPwm.push_back(getMotorPWM());

  samples++;
  if ((millis() - last >= 1000 &&
      !temp.empty() &&
      pHlist.size() == temp.size() &&
      rpm.size() == temp.size() )|| temp.size() > 50) {
    
    JsonDocument doc;
    makeReport(doc);
    serializeJson(doc, Serial1);
    Serial1.print("\n");
    last = millis();
    //Serial1.println("heartbeat");
    temp.clear();
    pHlist.clear();
    rpm.clear();
    acidPwm.clear();
    basePwm.clear();
    heaterPwm.clear();
    motorPwm.clear();
    samples = 0;

  }
  /*while (Serial1.available()) {
    Serial.write(Serial1.read()); // show incoming data on USB monitor
  }*/
}
template <typename T> 
class MaxMin {
  private:
    std::vector<T>& data;
  public:
    T min;
    T max;
    MaxMin(T _min, T _max, std::vector<T>& _data)
      : data(_data), min(_min), max(_max)  
    {}
    void adjMin(int i) {
      min = data[i] < min ? data[i] : min;
    }
    void adjMax(int i) {
      max = data[i] > max ? data[i] : max;
    }
    void adjBoth(int i) {
      adjMin(i);
      adjMax(i);
    }
};
template <typename A>
void makeReport(A& doc) {
  unsigned long cTime = millis();
  double min = 1000000.0;
  MaxMin<double> tempMinMax(min ,-min, temp);
  MaxMin<double> phMinMax(min ,-min, pHlist);
  MaxMin<double> rpmMinMax(min ,-min, rpm);
  double tMean = 0;
  double pHmean = 0;
  double rpmMean = 0;

  double acidPwmMean = 0;
  double basePwmMean = 0;
  double heaterPwmMean = 0;
  double motorPwmMean = 0;

  double heaterEnergy = getHeaterEnergyWh();

  for (int i  =0; i < temp.size(); ++i) {
    tempMinMax.adjBoth(i);
    phMinMax.adjBoth(i);
    rpmMinMax.adjBoth(i);
    tMean += temp[i];
    pHmean += pHlist[i];
    rpmMean += rpm[i];
    acidPwmMean += acidPwm[i];
    basePwmMean += basePwm[i];
    heaterPwmMean += heaterPwm[i];
    motorPwmMean += motorPwm[i];
  }
  double n = static_cast<double>(temp.size());
  tMean /= n;
  pHmean /= n;
  rpmMean /= n;
  acidPwmMean /= n;
  basePwmMean /= n;
  heaterPwmMean /= n;
  motorPwmMean /= n;
  doc["window"]["start"] = last;
  doc["window"]["end"] = cTime;
  doc["window"]["seconds"] = (cTime - last) / 1000; // unsafe cast
  doc["window"]["samples"] = samples;
  doc["setpoints"]["temperature_C"] = getTemperatureSetpoint();
  doc["setpoints"]["pH"] = getpHSetpoint();
  doc["setpoints"]["rpm"] = getRPMSetpoint();
  doc["rpm"]["mean"] = rpmMean;
  doc["rpm"]["min"] = rpmMinMax.min;
  doc["rpm"]["max"] = rpmMinMax.max;
  doc["pHlist"]["mean"] = pHmean;
  doc["pHlist"]["min"] = phMinMax.min;
  doc["pHlist"]["max"] = phMinMax.max;
  doc["temperature_C"]["mean"] = tMean;
  doc["temperature_C"]["min"] = tempMinMax.min;
  doc["temperature_C"]["max"] = tempMinMax.max;
  doc["actuators_avg"]["heater_pwm"] = heaterPwmMean;
  doc["actuators_avg"]["acid_pwm"] = acidPwmMean;
  doc["actuators_avg"]["base_pwm"] = basePwmMean;
  doc["actuators_avg"]["motor_pwm"] = motorPwmMean;
  doc["heater_energy_Wh"] = heaterEnergy;
}
 
