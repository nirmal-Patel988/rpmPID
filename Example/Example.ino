#include<rpmPID.h>
motor m1(2, 15 , 27, 12, 14, 4600, -1); //(en1,en2,d1,d2,pwm,cpr)9211,660
RPM rpm(&m1);
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  rpm.setAggTunings(2, 2, 0);
  rpm.setSoftTunings(1, 1, 0);
  rpm.setThreshold(500);
}
void loop() {
  if (Serial.available()) {
    rpm.setRPM(Serial.parseInt());
  }
  rpm.compute();
}
