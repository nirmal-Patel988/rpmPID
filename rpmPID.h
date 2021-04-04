#include<Motor.h>
#include <PID_v1.h>
class RPM {
  public:
    Motor *mtr = new Motor();    //Specify the links and initial tuning parameters
    double aggKp=0.03, aggKi=0, aggKd=0.00;
    double softKp=0.01, softKi=0, softKd=0.00;
    int softThreshold = 0;
    int Pwm, rpm;
    long prevReadings=0;
    double Setpoint = 0, Input=0, Output=0;
    PID *myPID = new PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
    int sampleTime = 0;
    long int st = 0;

    RPM() {
      myPID->SetMode(AUTOMATIC);
      myPID->SetSampleTime(this->sampleTime);
    }

    RPM(Motor *mtr) {
      myPID->SetMode(AUTOMATIC);
      myPID->SetSampleTime(this->sampleTime);
      myPID->SetOutputLimits(-255,255);
      this->mtr = mtr;
    }

    void setAggTunings(double kp, double ki, double kd){
        this->aggKp = kp;
        this->aggKi = ki;
        this->aggKd = kd; 
    }
    
    void setSoftTunings(double kp, double ki, double kd){
        this->softKp = kp;
        this->softKi = ki;
        this->softKd = kd;
    }
    
    void setThreshold(int targetThreshold){
        this->softThreshold = targetThreshold;
    }
    
    long getReadings(){
        return mtr->getReadings();
    }
    
    void setOutputLimits(int min,int max){
        myPID->SetOutputLimits(min,max);
    }

    void setRPM( int Setpoint) {
        this->Setpoint = abs(Setpoint);
    }
    
    void setTunings(double kp, double ki, double kd) {
        myPID->SetTunings(kp, ki, kd);
    }
  long currreading=0;
    void compute() {
      if (millis() - st > 10) {
        // Serial.println(mtr->cpr);
        // rpm = (double)(mtr->getReadings()- prevReadings) * 100 * 60 / mtr->cpr;
        // prevReadings = mtr->getReadings();

        currreading=mtr->getReadings();
         rpm = (double)(currreading- prevReadings) * 100 * 60 / mtr->cpr;
         prevReadings =currreading;
        Input = abs(rpm);   
        // Serial.print("b   b     ");   
        // Serial.println(Input);

        if(abs(Input)<softThreshold){
            this->setTunings(softKp,softKi,softKd);
        }
        
        else{
            this->setTunings(aggKp,aggKi,aggKd);
        }
        
        myPID->Compute();
         Serial.println(String(Setpoint) + "," + String(rpm) + "," + String(map(Output, -255, 255, -100, 100)));
        mtr->setPWM(Output);
        // Serial.println(Output);
        st = millis();
        // delay(50);
      }
    }
};