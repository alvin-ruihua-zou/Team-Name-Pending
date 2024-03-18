#include "Arduino.h"
#include <Encoder.h>
#include "Servo.h"

class MultiServo{
  public:
    Servo& s1;
    Servo& s2;
  public:
    
    MultiServo(Servo& sA, Servo& sB):
    s1(sA), s2(sB)
    {


    }

    void drive_motor(Servo servo, int control){
      if(control > 5){
        if(control > 20){
          servo.fw(control);
        }else{
          servo.fw(20);
        }
      }else if(control < -5){
        if(control < -20){
          servo.bw(-control);
        }else{
          servo.bw(20);
        }
        
      }else{
        servo.mbreak();
      }
    }
    //ramp power output to reduce acceleration
    //assumes s1 and s2 are traveling at similar speed
    int rampedPWM(int terminalPWM, long pos_diff){
      if(pos_diff > 1 * 1836){//change this value to change acceleration rate
        return terminalPWM;
      }
      double base = 0.4;
      double pwm = base + (1.0-base) * double(pos_diff + 1) / 1 * 1836;//change this value to change acceleration rate
      return pwm;
    }
///////////////////////////////////////////////////////////////////////////////////////////////////
//Main Servo Method////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

    void servo_to(long targetPosition1, long targetPosition2, int maxPwm, double p, double d, bool ramped = false){\
      //common variables
      unsigned long lastT = millis();
      unsigned long t = millis();
      unsigned long initialT = millis();
      unsigned long timeout = (_timeout_a * abs(targetPosition1 - s1.currPosition) / 1836 + _timeout_b) * 1000;
      int ogMaxPwm = maxPwm;

      //S1 variables
      long lastPosition1 = s1.currPosition;
      double lastVelocity1 = 0.0;
      double instantVelocity1 = 0.0;
      double velocity1 = 0.0;
      int stopcount1 = 0;
      bool s1Complete = false;

      //S2 variables
      long lastPosition2 = s2.currPosition;
      double lastVelocity2 = 0.0;
      double instantVelocity2 = 0.0;
      double velocity2 = 0.0;
      int stopcount2 = 0;
      bool s2Complete = false;

      

      
      
      while((s1Complete && s2Complete) == false){
        t = millis();
        if(t - initialT > timeout){
          
          break;
        }
        if(ramped){
          maxPwm = rampedPWM(ogMaxPwm, abs(targetPosition1 - s1.currPosition));
        }

        //S1 part////////////////////////////////////////////////////////
        if(s1Complete == false){
          s1.update_position();
          instantVelocity1 = double(s1.currPosition - lastPosition1) / double(t - lastT + 0.1) * 1000;
          velocity1 = 0.6 * velocity1 + 0.4 * instantVelocity1;
          double control1 = -p * 0.3* (s1.currPosition - targetPosition1) - d * 0.05 * velocity1;
          Serial.print("Target:");
          Serial.println(targetPosition1);
          //  Serial.print("curr:");
          // Serial.println(s1.currPosition);
          // Serial.print("control bf");
          // Serial.println(control1);
          double new_acs_reading = analogRead(A0);
          _acs_reading = _acs_reading * 0.92 + 0.08 * new_acs_reading;
          double amp = (_acs_reading - _zero) / 13.5;
          double ma = amp * 1000;
          // if(abs(ma) > _stall_ma && _enable_stall_protection){
          //   break;
          //   Serial.println("STALL DETECTED, ABORT");
          // }
          Serial.print("raw: ");
          Serial.println(new_acs_reading);
          //Serial.println(ma);
          // Serial.print(instantVelocity);
          // Serial.print("  ");
          
          // Serial.print(velocity);
          // Serial.print("  ");
          
          Serial.println(s1.currPosition);
          Serial.print(" <s1");
          // Serial.print(targetPosition1);
          
          if(abs(control1) > maxPwm){
            control1 = maxPwm * (control1 / abs(control1));
          }
          // Serial.print("  ");
          // Serial.println(control1);
          drive_motor(s1, control1);
          // if(_enable_current_limit && abs(ma) > _current_limit){
          //   control1 = chop_current(control1, ma);
          // }
          //Serial.print(control1 * 4);
          //Serial.print(" ");
          
          if(abs(targetPosition1 - s1.currPosition) < 400 && abs(velocity1) < 5){
            stopcount1 += 1;
            s1.mbreak();
          }else{
            stopcount1 = 0;
          }
          if(stopcount1 > 10){
              s1Complete = true;
              s1.mbreak();
            }
          if(abs(targetPosition1 - s1.currPosition) < 33 && abs(velocity1) < 10){
            s1Complete = true;
            Serial.print("Completed! Target:");
          Serial.println(targetPosition1);
           Serial.print("curr:");
          Serial.println(s1.currPosition);
            s1.mbreak();
          }
          lastPosition1 = s1.currPosition;
          lastVelocity1 = velocity1;
        }      
        //S2 part////////////////////////////////////////////////////////
        if(s2Complete == false){
          s2.update_position();
          instantVelocity2 = double(s2.currPosition - lastPosition2) / double(t - lastT + 0.1) * 1000;
          velocity2 = 0.6 * velocity2 + 0.4 * instantVelocity2;
          double control2 = -p * 0.3* (s2.currPosition - targetPosition2) - d * 0.05 * velocity2;
          if(abs(control2) > maxPwm){
            control2 = maxPwm * (control2 / abs(control2));
          }
          drive_motor(s2, control2);
          if(abs(targetPosition2 - s2.currPosition) < 400 && abs(velocity2) < 5){
            stopcount2 += 1;
          }else{
            stopcount2 = 0;
          }
          if(stopcount2 > 10){
              s2Complete = true;
              s2.mbreak();
              Serial.println("160");
            }
          if(abs(targetPosition2 - s2.currPosition) < 33 && abs(velocity2) < 10){
            s2Complete = true;
            s2.mbreak();
            Serial.println("165");
          }
          lastPosition2 = s2.currPosition;
          lastVelocity2 = velocity2;
           Serial.print("curr:");
          Serial.println(s2.currPosition);
        //common
          lastT = t;

        }
      
      }
      Serial.println("done");
      s1.mbreak();
      s2.mbreak();
    }
  private:

    int _en0, _en1, _pwm_pin, _enc0, _enc1;
    long _currPosition  = 0;
    unsigned long _timeout_a = 1;
    unsigned long _timeout_b = 3;

    //Current sensor
    double _acs_reading = 512.0;
    double _zero = 509.3;
    double _stall_ma = 1000;
    double _current_limit = 1000;
    bool _enable_stall_protection = false;
    bool _enable_current_limit = false;
};