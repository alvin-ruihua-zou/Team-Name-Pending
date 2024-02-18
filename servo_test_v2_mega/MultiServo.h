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
        if(control > 25){
          servo.fw(control);
        }else{
          servo.fw(25);
        }
      }else if(control < -5){
        if(control < -25){
          servo.bw(-control);
        }else{
          servo.bw(25);
        }
        
      }else{
        servo.mbreak();
      }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////
//Main Servo Method////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

    void servo_to(long targetPosition1, long targetPosition2, int maxPwm, double p, double d){
      unsigned long lastT = millis();
      unsigned long t = millis();
      unsigned long initialT = millis();
      int timeout = (_timeout_a * (targetPosition1 - s1.currPosition) / 720 + _timeout_b) * 1000;
      long lastPosition1 = s1.currPosition;
      long lastPosition2 = s2.currPosition;
      double lastVelocity1 = 0.0;
      double instantVelocity1 = 0.0;
      double velocity1 = 0.0;
      int stopcount1 = 0;

      double lastVelocity2 = 0.0;
      double instantVelocity2 = 0.0;
      double velocity2 = 0.0;
      int stopcount2 = 0;

      _acs_reading = 512.0;

      bool s1Complete = false;
      bool s2Complete = false;
      //while(abs(targetPosition - _currPosition) > 33 || abs(velocity) > 10){
      while((s1Complete && s2Complete) == false){
        t = millis();
        if(t - initialT > timeout){
          break;
        }

        //S1 part////////////////////////////////////////////////////////
        if(s1Complete == false){
          s1.update_position();
          instantVelocity1 = double(s1.currPosition - lastPosition1) / double(t - lastT + 0.1) * 1000;
          velocity1 = 0.6 * velocity1 + 0.4 * instantVelocity1;
          double control1 = -p * 0.3* (s1.currPosition - targetPosition1) - d * 0.05 * velocity1;
          // Serial.print("Target:");
          // Serial.println(targetPosition1);
          //  Serial.print("curr:");
          // Serial.println(s1.currPosition);
          // Serial.print("control bf");
          // Serial.println(control1);
          // double new_acs_reading = analogRead(A0);
          // _acs_reading = _acs_reading * 0.92 + 0.08 * new_acs_reading;
          // double amp = (_acs_reading - _zero) / 13.5;
          // double ma = amp * 1000;
          // if(abs(ma) > _stall_ma && _enable_stall_protection){
          //   break;
          //   Serial.println("STALL DETECTED, ABORT");
          // }
            
          //Serial.println(ma);
          // Serial.print(instantVelocity);
          // Serial.print("  ");
          
          // Serial.print(velocity);
          // Serial.print("  ");
          
          // Serial.print(s1.currPosition);
          // Serial.print("  ");
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
          
          if(abs(targetPosition1 - s1.currPosition) < 250 && abs(velocity1) < 5){
            stopcount1 += 1;
            Serial.println("plusing");
            Serial.println(stopcount1);
          }else{
            // Serial.print("dif");
            // Serial.println(abs(targetPosition1 - s1.currPosition));
            // Serial.print("velo");
            // Serial.println(velocity1);
            stopcount1 = 0;
          }
          if(stopcount1 > 10){
              s1Complete = true;
            }
          if(abs(targetPosition1 - s1.currPosition) < 33 && abs(velocity1) < 10){
            Serial.println("s1 compelte");
            s1Complete = true;
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
          if(abs(targetPosition2 - s2.currPosition) < 250 && abs(velocity2) < 5){
            stopcount2 += 1;
          }else{
            stopcount2 = 0;
          }
          if(stopcount2 > 10){
              s2Complete = true;
            }
          if(abs(targetPosition2 - s2.currPosition) < 33 && abs(velocity2) < 10){
            s2Complete = true;
          }
          lastPosition2 = s2.currPosition;
          lastVelocity2 = velocity2;
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
    int _timeout_a = 1;
    int _timeout_b = 3;

    //Current sensor
    double _acs_reading = 512.0;
    double _zero = 509.3;
    double _stall_ma = 1000;
    double _current_limit = 1000;
    bool _enable_stall_protection = false;
    bool _enable_current_limit = false;
};
