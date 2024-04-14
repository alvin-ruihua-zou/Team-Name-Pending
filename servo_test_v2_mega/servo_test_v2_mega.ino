
#include <Encoder.h>
//#include "Servo.h"
#include "MultiServo.h"
#include "Steppers.h"
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

// pindef_____________________________________________________________________

const int z1en0 = 27;
const int z1en1 = 26;
const int z1pwm = 7;

const int z2pwm = 6;
const int z2en0 = 23;
const int z2en1 = 22;

const int d1en0 = 17;
const int d1en1 = 16;
const int d1pwm = 5;

const int d2en0 = 24;
const int d2en1 = 25;
const int d2pwm = 4;

const int s1dir = 28;
const int s1step = 29;
const int s2dir = 30;
const int s2step = 31;
const int voltPin = A15;

// end pindef__________________________________________________________________
long currPosition  = 0;
long currPosition2 = 0;
//Encoder myEnc(2, 8);
Servo s1(z1en0, z1en1, z1pwm, 19, 42);
Servo s2(z2en0, z2en1, z2pwm, 18, 43);
Servo d1(d1en0, d1en1, d1pwm, 3, 40);
Servo d2(d2en0, d2en1, d2pwm, 2, 41);
MultiServo zAxis(s1, s2);
MultiServo driveTrain(d1,d2);
Steppers xAxis(s1dir, s1step, s2dir, s2step);
double theta_target = 0;
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  pinMode(z1en0, OUTPUT);
  pinMode(z1en1, OUTPUT);
  pinMode(z1pwm, OUTPUT);

  pinMode(z2en0, OUTPUT);
  pinMode(z2en1, OUTPUT);
  pinMode(z2pwm, OUTPUT);

  pinMode(d1en0, OUTPUT);
  pinMode(d1en1, OUTPUT);
  pinMode(d1pwm, OUTPUT);

  pinMode(d2en0, OUTPUT);
  pinMode(d2en1, OUTPUT);
  pinMode(d2pwm, OUTPUT);


  pinMode(s1dir, OUTPUT);
  pinMode(s1step, OUTPUT);
  pinMode(s2dir, OUTPUT);
  pinMode(s2step, OUTPUT);

  pinMode(A0, INPUT);
  zAxis.set_v_thresh(100);
  driveTrain.set_v_thresh(2);

  //s1.set_current_limiting(400, true);
}

double get_voltage(){
  double voltage = (double)analogRead(voltPin) / 1023 * 55.7;
  return voltage;
}

// void correct_theta(){
//   double theta = driveTrain.dtheta;
  
// }
void drive_till_edge(int ospeed, int max_range){
  double max_speed = 200.0;
  double correction = 18.0;
  long initPos1 = d1.myEnc.read();
  long initPos2 = d2.myEnc.read();
  long currPos1 = d1.myEnc.read();
  long currPos2 = d2.myEnc.read();
  int thresh = 150;
  // Serial.println(analogRead(A9));
  // Serial.println(abs(currPos - initPos));
  // Serial.println(analogRead(A9) < thresh);
  // Serial.println(abs(currPos - initPos) < max_range);
  long lastPosition1 = d1.myEnc.read();
  long lastPosition2 = d2.myEnc.read();
  long lastT = millis();
  long t = millis();
  double velocity1 = 0;
  double velocity2 = 0;
  int control1 = ospeed;
  int control2 = ospeed;
  while((analogRead(A9) > thresh) && (abs(currPos1 - initPos1) < max_range)){
    control1 = ospeed;
    control2 = ospeed;
    t = millis();
    Serial.println(analogRead(A9));

    currPos1 = d1.myEnc.read();
    velocity1 = double(currPos1 - lastPosition1) / double(t - lastT + 0.1) * 1000;
    currPos2 = d2.myEnc.read();
    velocity2 = double(currPos2 - lastPosition2) / double(t - lastT + 0.1) * 1000;

    

    if(abs(velocity1) > max_speed){
      control1 = driveTrain.limit_speed(abs(velocity1)/max_speed, control1);
    }
    if(abs(velocity2) > max_speed){
      control2 = driveTrain.limit_speed(abs(velocity2)/max_speed, control2);
    }

    double rot = (abs(currPos1 - initPos1) - abs(currPos2 - initPos2)) / 792 /4.6 ;//28/6.1 = 4.6
    control2 *= (1.0 + rot * correction);
  
    // Serial.print("velocity:");
    // Serial.println(velocity1);
    // Serial.print("control1");
    // Serial.println(control1);
    // Serial.print("control2");
    // Serial.println(control2);
    if(control1 < 0){
      d1.bw(-control1);
    }else{
      d1.fw(control1);
    }
    if(control2 < 0){
      d2.fw(-control2);
    }else{
      
      d2.bw(control2);
    }
    lastT = t;
    lastPosition1 = currPos1;
    lastPosition2 = currPos2;
    
  }
  d1.mbreak();
  d2.mbreak();
}
void multi_command()
{
  Serial.println("Enter CMD sequence:");
  while (Serial.available() == 0)
  {
    delay(10);
  }
  String cmd = Serial.readString();
  Serial.println(cmd);
  int curr_idx = 0;
  int idx = 0;
  while (true)
  {
    idx = cmd.indexOf(':', curr_idx);

    String command = cmd.substring(curr_idx, idx);
    Serial.print("executed:");
    Serial.println(command);
    cmd_servo_multi(command);
    delay(500);
    curr_idx = idx + 1;
    if (cmd.indexOf(':', curr_idx) == -1)
    {
      idx = cmd.indexOf(':', curr_idx);
      String command = cmd.substring(curr_idx, idx);
      Serial.print("last executed:");
      cmd_servo_multi(command);
      Serial.println(command);
      break;
    }
  }
  Serial.println("finished");
}



void cmd_servo_multi(String cmd)
{
  // Serial.println("Enter CMD:");
  // while (Serial.available() == 0){
  //   delay(10);
  // }
  // String cmd = Serial.readString();
  
  int index = 0;
  for(int i = 0; i < 10 && i < cmd.length(); i++){
    if(isDigit(cmd[i]) or cmd[i] == '-'){
      index = i;
      break;
    }
  }
  if(index == 0){
    index = cmd.length();
  }
  String dir = cmd.substring(0,index);
  double rev =cmd.substring(index).toFloat();
  Serial.print("cmd is:");
  Serial.println(dir);
  Serial.print("rev is:");
  Serial.println(rev);
  
  // if(dir == 'w'){
  //   s1.servo_to(tick1, 160, 0.2, 0.1);
  // }
  if(dir == "w"){
    
    long currPosition1 = s1.myEnc.read();
    long currPosition2 = s2.myEnc.read();
    long tick1 = -rev * 1836 + currPosition1;
    long tick2 = rev * 1836 + currPosition2;
    zAxis.servo_to(tick1, tick2, 140, 0.6, 0.2);
  }
  if(dir == "t"){
    theta_target += rev * 1.34; //1.28 rad per rev
    double rev_c = (theta_target - driveTrain.dtheta) / 1.34;
    Serial.print("revc");
    Serial.println(rev_c);
    long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = rev_c * 792 + currPosition1;
    long tick2 = rev_c * 792 + currPosition2;
    
    driveTrain.servo_to(tick1, tick2, 150, 3.6, 0.20, true,200);
    Serial.println("complete");
    Serial.println(rev_c);
    delay(400);    
  }
  if(dir == "tu"){
    // bool correct_theta = false;
    // if(correct_theta){

    //   double corr_rev = drivedtheta / 1.34; //77degree per rev, hence 1.34 rad per rev
    //   rev += corr_rev;
    // }
    long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = rev * 792 + currPosition1;
    long tick2 = rev * 792 + currPosition2;
    
    driveTrain.servo_to(tick1, tick2, 150, 2.4, 0.1, true,300);
    Serial.println("complete");
  }
  if(dir == "fw"){
    long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = rev * 792 + currPosition1;
    long tick2 = -rev * 792 + currPosition2;
    driveTrain.servo_to(tick1, tick2, 140, 0.5, 0.2, true, 600);
    Serial.println("complete");
  }
  if(dir == "l"){
    long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = 0.4 * 1836 + currPosition1;
    long tick2 = -0.4 * 1836 + currPosition2;
    driveTrain.servo_to_no_correction(tick1, tick2, 180, 0.8, 0.1, false);
    delay(500);
    tick1 = 1.0 * 1836 + currPosition1;
    tick2 = -1.0 * 1836 + currPosition2;
    driveTrain.servo_to_no_correction(tick1, tick2, 120, 0.4, 0.1, true);
  }
  if(dir == "a"){
    xAxis.revStepperSRamp(abs(rev), 1, 7 );
  }
  if(dir == "d"){
    xAxis.revStepperSRamp(abs(rev), -1, 7 );
  }
  if(dir == "z"){
    s1.myEnc.write(0);
    s2.myEnc.write(0);
    driveTrain.clear_odo();
    theta_target = 0;
  }
   if(dir == "p"){
    Serial.print("s1: ");
    Serial.println(s1.myEnc.read());
    Serial.print("s1: ");
    Serial.println(s2.myEnc.read());
  }
  if(dir == "v"){
    zAxis.servo_to(4.3 * 1836, -4.3* 1836, 80, 0.4, 0.1);
  }
  if(dir == "b"){
    xAxis.revStepperSRamp(6.4, -1, 6 );
  }
  if(dir == "n"){
    zAxis.servo_to(-0.4 * 1836, 0.4 * 1836, 80, 0.4, 0.1);
  }
  if(dir == "m"){
    xAxis.revStepperSRamp(6.4, 1, 6 );
  }
  if(dir == "cr"){
long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    // long tick1 = 0.38 * 792 + currPosition1;
    // long tick2 = -0.38 * 792 + currPosition2;
    // driveTrain.servo_to_no_correction(tick1, tick2, 200, 2.9, 0.1, false);
    // delay(200);
    long tick1 = 3.3 * 792 + currPosition1;
    long tick2 = -3.3 * 792 + currPosition2;
    driveTrain.servo_to_no_correction(tick1, tick2, 160, 1.8, 0.25, true);
    delay(200);    
  }
  if(dir == "i"){
    for(int i = 0; i < int(rev); i++){
      zAxis.servo_to(4.3 * 1836, -4.3* 1836, 100, 0.6, 0.1);
    delay(200);
    xAxis.revStepperSRamp(6.4, -1, 6 );
    delay(200);

    
    
    zAxis.servo_to(-0.2 * 1836, 0.2 * 1836, 100, 0.6, 0.1);
    delay(200);
    xAxis.revStepperSRamp(6.4, 1, 6 );
    delay(200);

    // long currPosition1 = d1.myEnc.read();
    // long currPosition2 = d2.myEnc.read();
    // // long tick1 = 0.38 * 792 + currPosition1;
    // // long tick2 = -0.38 * 792 + currPosition2;
    // // driveTrain.servo_to_no_correction(tick1, tick2, 200, 2.9, 0.1, false);
    // // delay(200);
    // long tick1 = 3.6 * 792 + currPosition1;
    // long tick2 = -3.6 * 792 + currPosition2;
    // driveTrain.servo_to_no_correction(tick1, tick2, 160, 1.9, 0.25, true);
    // delay(200);
    }
    
  }
  if(dir == "di"){
    zAxis.servo_to(-0.1 * 1836, 0.1 * 1836, 100, 0.5, 0.17);
    delay(200);
    for(int i = 0; i < int(rev); i++){
      drive_till_edge(-130, 3000);
      delay(500);                                                                                                                                                                         
      long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = -0.24 * 792 + currPosition1;
    long tick2 = 0.24 * 792 + currPosition2;
      driveTrain.servo_to(tick1, tick2, 130, 2.3, 0.25, false,200);
      delay(800);
      xAxis.revStepperSRamp(6.1, -1, 4);//frame back
      delay(400);

      zAxis.servo_to(4.5 * 1836, -4.5* 1836, 100, 0.5, 0.17);//frame down
      delay(800);

      xAxis.revStepperSRamp(6.4, 1, 4 );//bot back
      delay(800);

      zAxis.servo_to(-0.2 * 1836, 0.2 * 1836, 100, 0.5, 0.17);//bot down
      delay(800);
      if(i % 3 == 2){
        currPosition1 = d1.myEnc.read();
      currPosition2 = d2.myEnc.read();
      tick1 = 2.0 * 792 + currPosition1;
      tick2 = -2.0 * 792 + currPosition2;
      driveTrain.servo_to_no_correction(tick1, tick2, 140, 0.9, 0.1, true);//forward to square
      delay(200);
      }
      
    }

    
  }
  if(dir == "?"){
    Serial.print("Votage is: ");
    Serial.println(get_voltage());
  }
  if(dir == "dte"){
    drive_till_edge(rev, 2000);
    delay(200);
    long currPosition1 = d1.myEnc.read();
    long currPosition2 = d2.myEnc.read();
    long tick1 = -0.18 * 792 + currPosition1;
    long tick2 = 0.18 * 792 + currPosition2;
    driveTrain.servo_to(tick1, tick2, 130, 5.5, 0.3, false,100);
  }
  if(dir == "odo"){
    Serial.println("x, y, theta");
    Serial.println(driveTrain.dx);
    Serial.println(driveTrain.dy);
    Serial.println(driveTrain.dtheta);
  }
}

// void cmd_servo(){
//   Serial.println("Enter M1 revolution:");
//   while (Serial.available() == 0){
//     delay(10);
//   }
//   String cmd = Serial.readString();
//   Serial.println(cmd);
//   char dir = cmd[0];
//   double rev =cmd.substring(1).toFloat();
  
//   currPosition = s1.myEnc.read();
//   int tick = rev * 792 + currPosition;
//   if(dir == 'w'){
//     s1.servo_to(tick, 160, 0.2, 0.1);
//   }

//   currPosition2 = s2.myEnc.read();
//   int tick2 = rev * 792 + currPosition2;
//   if(dir == 'w'){
//     s2.servo_to(tick2, 160, 0.2, 0.1);
//   }
// }
void loop() {
  // cmd_servo_multi();
  multi_command();


}
