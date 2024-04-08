// Define pin connections & motor's steps per revolution

const int dirPinM1 = 2;
const int dirPinM2 = 4;
const int stepPinM1 = 3;
const int stepPinM2 = 5;
const int ledPin = 13;
const int stepsPerRevolution = 200;
int stepDelay=1000;
double m1r = 0;
double m2r = 0;
class Steppers{
  public: 
    int dirPinM1 = 2;
    int dirPinM2 = 4;
    int stepPinM1 = 3;
    int stepPinM2 = 5;
  public:
    Steppers(int dirM1,  int stepM1, int dirM2, int stepM2){
      dirPinM1 = dirM1;
      dirPinM2 = dirM2;
      stepPinM1 = stepM1;
      stepPinM2 = stepM2;
    }
    int rampedDelay(int terminalDelay, int ramp_step, int step){
      if(step > ramp_step){
        return terminalDelay;
      }
      double base = 0.1;
      double progress = base + (1.0-base) * double(step + 1) / ramp_step;
      int delay = terminalDelay / sqrt(progress);
      return delay;
    }
    void revStepperSRamp(double rev, int direction, float rps){
      int ramp_steps = 300;
      float stepPS = rps * stepsPerRevolution;
      int terminalStepDelay = (int)(1000000 / stepPS / 2);
      int stepDelay = terminalStepDelay;
      if(true){
        if(direction > 0){
          digitalWrite(dirPinM1, HIGH);
          digitalWrite(dirPinM2, LOW);
        }else{
          digitalWrite(dirPinM1, LOW);
          digitalWrite(dirPinM2, HIGH);
        }
        int steps = (int)(abs(stepsPerRevolution * rev));
        int delay = terminalStepDelay;
        for(int x = 0; x < steps; x++)
        {
          
          if(x < ramp_steps){
            stepDelay = rampedDelay(terminalStepDelay, ramp_steps, x);
            //Serial.println(rampedDelay(terminalStepDelay, ramp_steps, x));
            
          }else if(x > steps-ramp_steps){
            stepDelay = rampedDelay(terminalStepDelay, ramp_steps, steps - x);
            //stepDelay = terminalStepDelay;
          }
          else{
            stepDelay = terminalStepDelay;
          }
          digitalWrite(stepPinM1, HIGH);
          digitalWrite(stepPinM2, HIGH);
          delayMicroseconds(stepDelay);
          digitalWrite(stepPinM1, LOW);
          digitalWrite(stepPinM2, LOW);
          delayMicroseconds(stepDelay);
          //Serial.println(stepDelay);
          //Serial.println(double(1000000)/stepDelay/200/2);
        }
      }
    }
};
// void loop()
// {
//   //clockwise
//   Serial.println("Enter M1 revolution:");
//   while (Serial.available() == 0){
//     delay(10);
//   }
//   String cmd = Serial.readString();
//   Serial.println(cmd);
//   char dir = cmd[0];
//   float rev =cmd.substring(1).toFloat();
//   Serial.println(dir);
//   Serial.println(rev);
//   if(dir == 'w'){
//     revStepperRamp(1, rev, 1, 8);
//   }
//   else if(dir == 's'){
//     revStepperRamp(1, rev, -1, 8);
//   }
//   else if(dir == 'a'){
//     revStepperS2(2, rev, -1, 2);
//   }
//   else if(dir == 'd'){
//     revStepperS(2, rev, 1, 2);
//   }
//   else{
//     Serial.println((String)"Invalid input " + cmd);
//   }

// }

