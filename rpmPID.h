#include<Motor.h>
#include <PID_v1.h>
class RPM {
  public:
    Motor *mtr = new Motor();    //Specify the links and initial tuning parameters
    double aggKp=0.0, aggKi=0, aggKd=0.00;
    double softKp=0.0, softKi=0, softKd=0.00;
    int softThreshold = 0;
    double inputOffset = 0.01;
    int Pwm, rpm,cpr;
    long double prevReadings=0;
    double Setpoint = 0, Input=0, Output=0;
    PID *myPID = new PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
    int sampleTime = 0;
    long int st = 0;
    int interval = 5;
    RPM() {
      myPID->SetMode(AUTOMATIC);
      myPID->SetSampleTime(this->sampleTime);
    }

    RPM(Motor *mtr,int cpr=140) {
      myPID->SetMode(AUTOMATIC);
      myPID->SetSampleTime(this->sampleTime);
      myPID->SetOutputLimits(-255,255);
      this->mtr = mtr;
      this->cpr = cpr;
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
        this->Setpoint = Setpoint;
    }
    
    void setTunings(double kp, double ki, double kd) {
        myPID->SetTunings(kp, ki, kd);
    }
  long currreading=0;
    void compute() {
      if (millis() - st > interval) {
        currreading=mtr->getReadings();
        rpm = currreading - prevReadings;
        Serial.println(((double)Setpoint*interval)/1000.0);
        prevReadings = (double)prevReadings + (((double)Setpoint*interval)/1000.0);
        Input = rpm;   
        Serial.println(currreading);
        if(abs(Input)<softThreshold){
            this->setTunings(softKp,softKi,softKd);
        }
        
        else{
            this->setTunings(aggKp,aggKi,aggKd);
        }
        
        myPID->Compute();
        // Serial.println("");
        // Serial.println("rpm:");
        Serial.println(String(Setpoint) + "," + String(rpm) + "," + Output);
        mtr->setPWM(Output);
        // Serial.println(Output);
        st = millis();
        // delay(50);
      }
    }
};