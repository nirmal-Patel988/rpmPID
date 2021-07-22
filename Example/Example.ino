#include<rpmPID.h>
Motor m1(22,21);
RPM rpm(&m1,140);
void setup() {
  Serial.begin(115200);
  m1.setEncoder(26,27,-1);
  
  Serial.setTimeout(5);
  rpm.setThreshold(30);
  rpm.setAggTunings(0.3, 0.5, 0);
  rpm.setSoftTunings(0.05, 0.1, 0);
//  m1.setPWM(200);
}
void loop() {
  if (Serial.available()) {    
    rpm.setRPM(Serial.parseInt());
  }
  rpm.compute();

}