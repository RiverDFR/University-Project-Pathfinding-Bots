/* Appendix D: MONA Code (Loaded onto the MONA Education Robot)
[27] – Roughly 80 percent of the code is from the referenced individual’s work */

#include <SPI.h>
#include <MsTimer2.h>
#include <Wire.h>
#include<SoftwareSerial.h>
int LED1 = 13;        //D13 RED LED on top
int LED2 = 12;        //D12 bottom front LED
#define ON   1
#define OFF  0
//motors IOs  ---------------------------
float Motor_right_PWM = 10;  //   PIN 10
float Motor_right_direction = 5;  //   PIN 5
float Motor_left_PWM = 9;    //   PIN 9 
float Motor_left_direction = 6;   //   PIN 6
//#define Forward 0              // Direction code
//#define Reverse 1              // Direction code
float directl=0;
float directr=0;
float directll=1;
float directrr=1;
float directlreal=1;
float directrreal=1;
const byte Left_interruptPin = 2;
const byte Right_interruptPin = 3;
char order = '0';

float Left_forward_speed=0;
float Right_forward_speed=0;
float Kp = 0.15;
float Ki = 3;
float move_distance = 0;
float angle = 0;

// Used to control whether this node is sending or receiving

//---------------------------------------------------
void forward(){
  analogWrite(Motor_right_PWM,Right_forward_speed); // right motor
  digitalWrite(Motor_right_direction,directr); //right
  analogWrite(Motor_left_PWM,Left_forward_speed); // left motor
  digitalWrite(Motor_left_direction,directl); //left
}

long double Left_counter, Right_counter;

//---------------------------------------------------
void Left_int_counter(){
  Left_counter++;
}
//---------------------------------------------------
void Right_int_counter(){
  Right_counter++;
}
//---------------------------------------------------

float Left_compensatory=1,Right_compensatory=1;
float Desiredr = 0;   // number of pulses which is desired
float Desiredl = 0;   // number of pulses which is desired
float run_exp=0;

float sum_Right_compensatory=0,sum_Left_compensatory=0;

//---------------------------------------------------
void Timer_overflow() {   // every 400 ms is called
  run_exp++;
  Right_compensatory =  directrr*Desiredr - directrreal* Right_counter;
  Left_compensatory  =  directll*Desiredl -  directlreal* Left_counter;  
  sum_Right_compensatory +=  Right_compensatory;
  sum_Left_compensatory += Left_compensatory;

  // a basic proportional control (disable these two lines for Open-loop control)
  //Right_forward_speed +=  Kp * Right_compensatory;
  //Left_forward_speed  +=  Kp *  Left_compensatory;
  Right_forward_speed =  Kp * Right_compensatory+Ki*sum_Right_compensatory*0.05;
  Left_forward_speed  =  Kp * Left_compensatory+Ki*sum_Left_compensatory*0.05;

  if ( Right_forward_speed<0){
 // Right_forward_speed=abs(Right_forward_speed);
    directrreal=-1;
   directr=1;
  }
  else{ 
    directrreal=1;
    directr=0;
  }

  if ( Left_forward_speed<0){
  // Left_forward_speed=abs(Left_forward_speed);
   directlreal=-1;
   directl=1;
  }
  else{
    directlreal=1;
    directl=0;
  }
  forward();  // update PWM set-point 
  //reset counters 
  Left_counter=0;
  Right_counter=0;
}
//---------------------------------------------------
//---------------------------------------------------
void setup() {

  Wire.begin(9); // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600); // start serial for output
  
  // set encoder counter to 0
  Left_counter=0;
  Right_counter=0;
  // init INT0 and INT1 for left and right motors encoders 
  pinMode(Left_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Left_interruptPin), Left_int_counter, CHANGE);
  pinMode(Right_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Right_interruptPin), Right_int_counter, CHANGE);
  // setup Timer2 for motor control
  MsTimer2::set(50, Timer_overflow); // 400ms period
  MsTimer2::start();
  //set PWM set-point
  forward();
}
//---------------------------------------------------

void receiveEvent(int bytes) {
 order = Wire.read(); // waits to recieve the order from the main arduino UNO
 Serial.print(order);
}

//---------------------------------------------------

void line_straight(float move_distance){ //going straight code
  float move_speed=50; // mm/s
  float move_time=move_distance*100/move_speed; // *100ms
  for (float i=0;i<move_time;i++){
         Desiredl = move_speed*1.8; 
         Desiredr = move_speed*1.8;  
         directl=1;
         directr=1;  
         directll=-1;
         directrr=-1;
         delay(100);
    }
    Desiredl = 0; 
    Desiredr = 0;   
}
//---------------------------------------------------

void turn_angle_right(float angle){ //turning right code
  float move_speed=50; // mm/s
  float move_time=angle/90*5.97*100/move_speed; // *100ms
  for (float i=0;i<move_time;i++){
         Desiredl = move_speed*1.8; 
         Desiredr = move_speed*1.8;  
         directl=1;
         directr=0;  
         directll=-1;
         directrr=1;
         delay(100);
    }
    Desiredl = 0;
    Desiredr = 0;   
}
//---------------------------------------------------

void turn_angle_left(float angle){ //turning left code
  float move_speed=50; // mm/s
  float move_time=angle/90*5.97*100/move_speed; // *100ms
  for (float i=0;i<move_time;i++){
         Desiredl = move_speed*1.8; 
         Desiredr = move_speed*1.8;  
         directl=0;
         directr=1;  
         directll=1;
         directrr=-1;
         delay(100);
    }
    Desiredl = 0;
    Desiredr = 0;   
}
//---------------------------------------------------
void stop_motors(){
    MsTimer2::stop();
    analogWrite(Motor_right_PWM,0); // right motor
    digitalWrite(Motor_right_direction,0); //right
    analogWrite(Motor_left_PWM,0); // left motor
    digitalWrite(Motor_left_direction,0); //left
    while(1){
      delay(100);
    }
}
//---------------------------------------------------

// the loop function runs over and over again forever
void loop() {
  
   if(order == '1'){ //order 1 is the "forward" command from the arduino uno
      line_straight(12);
      order = '0';
    }
    
    else if(order == '2'){ //order 2 if the "back" command from the arduino uno
      turn_angle_right(97);
      turn_angle_right(97);
      order = '0';
    }

    else if(order == '3'){ //order 3 is the "left" command from the arduino uno
      turn_angle_left(97);
      order = '0';
    }

    else if(order == '4'){ //mode 4 is the "right" command from the arduino uno
      turn_angle_right(97);
      order = '0';
    }
    else if(order == '5'){ //mode 5 is the "stop" command from the arduino uno
      stop_motors();
    }
}
