#include "Arduino.h"
#include <Encoder.h>

class Servo{
  public:
    Encoder myEnc;
    long currPosition  = 0;
  public:
    
    Servo(int en0, int en1, int pwm_pin, int enc0, int enc1):
    myEnc(enc0,enc1){
      _en0 = en0;
      _en1 = en1;
      _pwm_pin = pwm_pin;
      _enc0 = enc0;
      _enc1 = enc1;

    }

///////////////////////////////////////////////////////////////////////////////////////////////////
//SetUps////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

    void set_stall_protection(int ma, bool enable){
      _stall_ma = ma;
      _enable_stall_protection = enable;
    }
    void set_current_limiting(int ma, bool enable){
      _current_limit = ma;
      _enable_current_limit = enable;
    }
    void update_position(){
      currPosition = myEnc.read();
    }


///////////////////////////////////////////////////////////////////////////////////////////////////
//Motor Driving////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

    void fw(int pwm){
      // Serial.print("from fw:");
      // Serial.println(pwm);
      digitalWrite(_en0, HIGH);
      digitalWrite(_en1, LOW);
      analogWrite(_pwm_pin, pwm);
      // digitalWrite(3, HIGH);
      // digitalWrite(4, LOW);
      // analogWrite(2, pwm);
    }

    void bw(int pwm){
      // Serial.print("from bw:");
      // Serial.println(pwm);
      digitalWrite(_en0, LOW);
      digitalWrite(_en1, HIGH);
      analogWrite(_pwm_pin, pwm);
    }

    void idle(){
      digitalWrite(_en0, LOW);
      digitalWrite(_en1, LOW);
    }

    void mbreak(){
      digitalWrite(_en0, HIGH);
      digitalWrite(_en1, HIGH);
    }

    double chop_current(double control, double current){
      double over_current = current - _current_limit;
      if (over_current >  _current_limit){
        return 0;
      }else{
        return control * (0.5 * (1.0 - over_current / _current_limit));
      }
    }    

///////////////////////////////////////////////////////////////////////////////////////////////////
//Main Servo Method////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

    void servo_to(long targetPosition, int maxPwm, double p, double d){
      unsigned long lastT = millis();
      unsigned long t = millis();
      unsigned long initialT = millis();
      currPosition = myEnc.read();
      int timeout = (_timeout_a * (targetPosition - currPosition) / 720 + _timeout_b) * 1000;
      Serial.println(timeout);
      Serial.println(targetPosition);
      Serial.println(currPosition);
      long lastPosition = myEnc.read();
      double lastVelocity = 0.0;
      double instantVelocity = 0.0;
      double velocity = 0.0;
      int stopcount = 0;

      _acs_reading = 512.0;
      while(abs(targetPosition - currPosition) > 33 || abs(velocity) > 10){
        
        currPosition = myEnc.read();
        
        t = millis();
        Serial.println(t- initialT);
        if(t - initialT > timeout){
          
          break;
        }
        instantVelocity = double(currPosition - lastPosition) / double(t - lastT + 0.1) * 1000;
        velocity = 0.6 * velocity + 0.4 * instantVelocity;
        double controls = -p * 0.3* (currPosition - targetPosition) - d * 0.05 * velocity;

        double new_acs_reading = analogRead(A0);
        _acs_reading = _acs_reading * 0.92 + 0.08 * new_acs_reading;
        double amp = (_acs_reading - _zero) / 13.5;
        double ma = amp * 1000;
        if(abs(ma) > _stall_ma && _enable_stall_protection){
          break;
          Serial.println("STALL DETECTED, ABORT");
        }
           
        //Serial.println(ma);
        // Serial.print(instantVelocity);
        // Serial.print("  ");
        
        // Serial.print(velocity);
        // Serial.print("  ");
        
        Serial.print(currPosition);
        Serial.print("  ");
        // Serial.print(targetPosition);
        // Serial.print("  ");
        // Serial.println(controls);
        if(abs(controls) > maxPwm){
          controls = maxPwm * (controls / abs(controls));
        }
        if(_enable_current_limit && abs(ma) > _current_limit){
          controls = chop_current(controls, ma);
        }
        //Serial.print(controls * 4);
        //Serial.print(" ");
        if(controls > 10){
          fw(controls);
        }else if(controls < -10){
          bw(-controls);
        }else{
          mbreak();
        }
        if(abs(targetPosition - currPosition) < 180 && abs(velocity) < 1){
          stopcount += 1;
          
        }else{
          stopcount = 0;
        }
        if(stopcount > 10){
            break;
          }
        lastPosition = currPosition;
        lastVelocity = velocity;
        lastT = t;
      }
      mbreak();
    }
  private:
    int _en0, _en1, _pwm_pin, _enc0, _enc1;
    
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
