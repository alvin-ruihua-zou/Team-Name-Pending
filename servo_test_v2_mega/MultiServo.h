#include "Arduino.h"
#include <Encoder.h>
#include "Servo.h"

class MultiServo{
  public:
    Servo& s1;
    Servo& s2;
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
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

    void clear_odo(){
      dx = 0;
      dy = 0;
      dtheta = 0;
    }
    int limit_speed(double over_speed, int control){
      over_speed = min(over_speed, 4);
      double b = 0.96;
      double a = 0.15;
      double ratio = 0.95 - a * (over_speed - 1.0);
      double new_control = ratio * control;
      if(control * new_control < 0){
        return 0;
      }else{
        return new_control;
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

    void servo_to(long targetPosition1, long targetPosition2, int maxPwm, double p, double d, bool enable_odo = false, double max_speed = 9999){
      //common variables
      unsigned long lastT = millis();
      unsigned long t = millis();
      unsigned long initialT = millis();
      unsigned long timeout = (_timeout_a * abs(targetPosition1 - s1.currPosition) / 1836 + _timeout_b) * 1000;
      int ogMaxPwm = maxPwm;
      bool ramped = false;
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
        s1.update_position();
        s2.update_position();
        if(enable_odo){
          double d_s1 = s1.currPosition - lastPosition1;
          // Serial.print("currPostion:");
          // Serial.println(s1.currPosition);
          // Serial.print("lastPosition1:");
          // Serial.println(lastPosition1);

          double d_s2 = -(s2.currPosition - lastPosition2);
          double d_dist = (d_s1 + d_s2)/2 / 792 * 0.0607*3.14;
          double d_rot = (d_s1 - d_s2) * 3.14 *1.2  / 792 * 0.217;//wheel diameter 6.1cm, wheel base 28cm
          dy += d_dist * 1000 * sin(dtheta);
          dx += d_dist * 1000 * cos(dtheta);
          // Serial.print("X:");
          // Serial.println(dx);
          // Serial.print("Y:");
          // Serial.println(dy);
          // Serial.print("ds1: ");
          // Serial.println(d_s1);
          // Serial.print("ds2: ");
          // Serial.println(d_s2);
          // Serial.print("Theta:");
          // Serial.println(dtheta);
          dtheta += d_rot;
        }
        

        //S1 part////////////////////////////////////////////////////////
        if(s1Complete == false){
          
          instantVelocity1 = double(s1.currPosition - lastPosition1) / double(t - lastT + 0.1) * 1000;
          velocity1 = instantVelocity1;
          double control1 = -p * 0.3* (s1.currPosition - targetPosition1) - d * 0.05 * velocity1;
          //Serial.print("s1curr:");
          //Serial.println(s1.currPosition);
          // Serial.print("control bf");
          // Serial.println(control1);
          //Serial.println(ma);
          // Serial.print(instantVelocity);
          // Serial.print("  ");
          
          // Serial.print(velocity);
          // Serial.print("  ");
          
          // Serial.println(s1.currPosition);
          // Serial.print(" <s1");
          // Serial.print(targetPosition1);
          
          if(abs(control1) > maxPwm){
            control1 = maxPwm * (control1 / abs(control1));
          }
          // Serial.print("velocity1:");
          // Serial.println(velocity1);
          // Serial.print("velocity (cm/s):");
          // Serial.print(velocity1 / 792.0 * 19.8);
          // Serial.print(" at time (s):");
          // Serial.println(double(t - initialT) / 1000.0);
          // Serial.print(" control1 before: ");
          // Serial.println(control1);
          if(abs(velocity1) > max_speed && velocity1 * control1 > 0){
            control1 = limit_speed(abs(velocity1)/max_speed, control1);
          }
          // Serial.print(" control1: ");
          // Serial.println(control1);
          
          drive_motor(s1, control1);
          if(abs(targetPosition1 - s1.currPosition) < 500 && abs(velocity1) < 2){
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
          }
          
          lastVelocity1 = velocity1;
        }      
        //S2 part////////////////////////////////////////////////////////
        if(s2Complete == false){
          
          instantVelocity2 = double(s2.currPosition - lastPosition2) / double(t - lastT + 0.1) * 1000;
          velocity2 = instantVelocity2;
          double control2 = -p * 0.3* (s2.currPosition - targetPosition2) - d * 0.05 * velocity2;
          if(abs(control2) > maxPwm){
            control2 = maxPwm * (control2 / abs(control2));
          }
          // Serial.print("velocity2:");
          // Serial.println(velocity2);
          // Serial.print(" control2 before: ");
          // Serial.println(control2);
          if(abs(velocity2) > max_speed && velocity2 * control2 > 0){
            control2 = limit_speed(abs(velocity2)/max_speed, control2);
          }
          // Serial.print(" control2: ");
          // Serial.println(control2);
          drive_motor(s2, control2);
          if(abs(targetPosition2 - s2.currPosition) < 500 && abs(velocity2) < 2){
            stopcount2 += 1;
          }else{
            stopcount2 = 0;
          }
          if(stopcount2 > 10){
              s2Complete = true;
              s2.mbreak();
            }
          if(abs(targetPosition2 - s2.currPosition) < 33 && abs(velocity2) < 10){
            s2Complete = true;
            s2.mbreak();
          }
          
          lastVelocity2 = velocity2;
          //  Serial.print("currs2:");
          // Serial.println(s2.currPosition);
        
          

        }
        //common
        lastT = t;
        lastPosition1 = s1.currPosition;
        lastPosition2 = s2.currPosition;
      
      }
      // Serial.println("done");
      s1.mbreak();
      s2.mbreak();
    }

    void servo_to_no_correction(long targetPosition1, long targetPosition2, int maxPwm, double p, double d, bool detect_stall = false){\
      //common variables
      unsigned long lastT = millis();
      unsigned long t = millis();
      unsigned long initialT = millis();
      unsigned long timeout = (_timeout_a * abs(targetPosition1 - s1.currPosition) / 1836 + _timeout_b) * 1000;
      double max_speed = 120;

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

        //S1 part////////////////////////////////////////////////////////
        if(s1Complete == false){
          s1.update_position();
          instantVelocity1 = double(s1.currPosition - lastPosition1) / double(t - lastT + 0.1) * 1000;
          velocity1 = 0.6 * velocity1 + 0.4 * instantVelocity1;
          double control1 = -p * 0.3* (s1.currPosition - targetPosition1) - d * 0.05 * velocity1;
          
          if(detect_stall){
            double new_acs_reading = analogRead(d1CurrentPin);
            Serial.print("new acsReading:");
            Serial.println(new_acs_reading);
            Serial.print("acsReading:");
            Serial.println(_acs_reading1);
            _acs_reading1 = _acs_reading1 * 0.97 + 0.03 * new_acs_reading;
            double amp = abs((_acs_reading1 - _zero) / 13.5);
            double ma = amp * 1000;
            Serial.println(ma);
            if(ma > _stall_ma){
              s1.mbreak();
              Serial.println("S1 STALLED");
              s1Complete = true;
            }
          }
          
          if(abs(control1) > maxPwm){
            control1 = maxPwm * (control1 / abs(control1));
          }
          if(abs(velocity1) > max_speed && velocity1 * control1 > 0){
            control1 = limit_speed(abs(velocity1)/max_speed, control1);
          }
          drive_motor(s1, control1);

          if(abs(targetPosition1 - s1.currPosition) < 50){
            s1Complete = true;
          //   Serial.print("Completed! Target:");
          // Serial.println(targetPosition1);
          //  Serial.print("curr:");
          // Serial.println(s1.currPosition);
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

          if(detect_stall){
            double new_acs_reading = analogRead(d2CurrentPin);
            _acs_reading2 = _acs_reading2 * 0.97 + 0.03 * new_acs_reading;
            double amp = abs((_acs_reading2 - _zero) / 13.5);
            double ma = amp * 1000;
            if(ma > _stall_ma){
              s2.mbreak();
              Serial.println("S2 STALLED");
              s2Complete = true;
            }
          }
          if(abs(control2) > maxPwm){
            control2 = maxPwm * (control2 / abs(control2));
          }
          if(abs(velocity2) > max_speed && velocity2 * control2 > 0){
            control2 = limit_speed(abs(velocity2)/max_speed, control2);
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
            }
          if(abs(targetPosition2 - s2.currPosition) < 50){
            s2Complete = true;
            s2.mbreak();
          }
          lastPosition2 = s2.currPosition;
          lastVelocity2 = velocity2;
        //common
          lastT = t;

        }
      
      }
      s1.mbreak();
      s2.mbreak();
      _acs_reading1 = 512;
      _acs_reading2 = 512;
    }
  private:

    int _en0, _en1, _pwm_pin, _enc0, _enc1;
    long _currPosition  = 0;
    unsigned long _timeout_a = 1;
    unsigned long _timeout_b = 3;

    int d1CurrentPin = A3;
    int d2CurrentPin = A2;

    //Current sensor
    double _acs_reading1 = 512.0;
    double _acs_reading2 = 512.0;
    double _zero = 512.0;
    double _stall_ma = 720;
    double _current_limit = 1000;
    bool _enable_stall_protection = false;
    bool _enable_current_limit = false;

    
};
