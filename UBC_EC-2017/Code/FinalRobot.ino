/*
  This code was used to control our submission for the 2017 UBC Engineering Competition.
  Our submission placed 2nd out of 7 teams.
  Team members: Megan Nantel, Candice Ip, Adam Schonewille and Doug Matthews
*/

#include <Servo.h>]
#include <QueueArray.h>


int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

//DC Servo 1 (BASE)
Servo servo1;  // create servo object to control a servo
int pot1 = 3;
int motor1 = 9;
float dSum1 = 0;
float lastError1;
float kp1 = -3;
float kd1 = 0.00001;
QueueArray<float> dQueue1;

//DC SERVO 2 (BOTTOM PIVOT)
Servo servo2;  // create servo object to control a servo
int pot2 = A4;
int motor2 = 10;
float dSum2 = 0;
float lastError2;
float kp2 = -3;
float kd2 = 0.00001;
//QueueArray<float> dQueue2;

//DC SERVO 3 (MIDDLE PIVOT)
Servo servo3;  // create servo object to control a servo
int pot3 = A5;
int motor3 = 11;
float dSum3 = 0;
float lastError3;
float kp3 = -3;
float kd3 = 0.00001;
//QueueArray<float> dQueue3;

int NUM_DERIVATIVE_STEPS = 3;

QueueArray<float> AdjustDCServo(Servo servo, int potNum, int goalAngle, QueueArray<float> queue, float *derivativeSum, float *lastError, float kp, float kd);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

// Use these to control the mode of the robot
bool simple = true; // Allow the user to control the robot with 3 potentiometers
bool push1 = false; // Perform the push test
bool position1 = false; // Move the object from one position to the other
bool height = false; // Lift the object to a given height

void setup() {
  servo1.attach(motor1);  
  servo2.attach(motor2);  
  servo3.attach(motor3);  
  Serial.begin(9600);
  delay(1000);

  // setup Queues to compute derivatives 
  for(int count = 0; count < NUM_DERIVATIVE_STEPS; count++){
    dQueue1.push(0.0);
//    Multiple quese where causing problems. Given the short time frame the 
//    other motors where switched to just proportional gain instead of debuging problem
//    dQueue2.push(0.0);
//    dQueue3.push(0.0);
  }

  // Initalize the "Last Errors" this is used to calculate the derivative.
  lastError1 = 0;
  lastError2 = 0;
  lastError3 = 0;

  // Set the inital positions for the autonomus modes
  if(push1){
    for( int count; count < 400; count++){
      SimpleDCServo(87, 1);
      SimpleDCServo(180, 2);
      SimpleDCServo(147, 3);
      delay(15);
    }
  }
  else if (position1){
    for( int count; count < 400; count++){
      SimpleDCServo(87, 1);
      SimpleDCServo(180, 2);
      SimpleDCServo(147, 3);
      delay(15);
    }
  }

  else if(height){
    for( int count; count < 400; count++){
      SimpleDCServo(87, 1);
      SimpleDCServo(180, 2);
      SimpleDCServo(100, 3);
      delay(15);
    }
  }
  
}

void loop() {
 
  if(simple){
    
    // Read Values of Potentiometers
    int val1 = analogRead(0);            // reads the value of the potentiometer (value between 0 and 1023)
    val1 = map(val1, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    int val2 = analogRead(1);            // reads the value of the potentiometer (value between 0 and 1023)
    val2= map(val2, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    int val3 = analogRead(2);            // reads the value of the potentiometer (value between 0 and 1023)
    val3 = map(val3, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    
    SimpleDCServo(val1, 1);
    SimpleDCServo(val2, 2);
    SimpleDCServo(val3, 3);
    
    Serial.print("Motor1");
    Serial.print(val1);
    Serial.print("Motor2");
    Serial.print(val2);
    Serial.print("Motor3");
    Serial.println(val3);
  }
  else if (push1){
    SimpleDCServo(87, 1);
    SimpleDCServo(20, 2);
    SimpleDCServo(40, 3);
  }
  else if (position1){
    SimpleDCServo(120, 1);
    SimpleDCServo(100, 2);
    SimpleDCServo(40, 3);
  } else if (height){
    SimpleDCServo(87, 1);
    SimpleDCServo(180, 2);
    SimpleDCServo(100, 3);
  }
  
  delay(15);                         
}

// Corect the angle of the motor using PD control
QueueArray<float> AdjustDCServo(Servo servo, int potNum, int goalAngle, QueueArray<float> queue,  float *derivativeSum, float *lastError, float kp, float kd){
  int goalVal = map(goalAngle, 0, 180, 50, 950); 

  int readVal = analogRead(potNum);

  int error = goalVal - readVal;

  float errorChange = (float)error - *lastError;
  *lastError = error;
  
  float popVal = queue.pop();

  // Compute the derivatve using the sum of the past NUM_DERIVATIVE_STEPS errorChanges.
  // This is an approximation of the derivative using a moving average.
  *derivativeSum = *derivativeSum + errorChange - popVal;
  queue.push(errorChange);
  float dError = mapf(*derivativeSum/NUM_DERIVATIVE_STEPS, -250.0, 250.0, -50, 50)*abs(error);

  int gain = map(kp*error,-1023, 1023, 0, 180)+ kd*dError; 

  servo.write(gain);

//  Use these lines for debugging
//  Serial.print("Goal Val:");
//  Serial.print(goalAngle);
//
//  Serial.print("Read Val:");
//  Serial.print(readVal);
//  
//  Serial.print(", pop Val:");
//  Serial.print(popVal);
//  
//  Serial.print(", derivative Val:");
//  Serial.print(dError);
//
//  
//  Serial.print(", error Val:");
//  Serial.print(error);
//
//  Serial.print(", gain:");
//  Serial.println(gain);

  return queue;
}

// Use proportional gain to correct the angle of the motor.
void  ExtraSimple(Servo servo, int potNum, int goalAngle, float kp){
  int goalVal = map(goalAngle, 0, 180, 50, 900); 

  int readVal = analogRead(potNum);
  int error = goalVal - readVal;

  int gain = map(kp*error,-1023, 1023, 0, 180); 

  servo.write(gain);
}

// Use this function to easily change the angle of the motor.
void SimpleDCServo(int angle, int motorNum){
  if(motorNum == 1){
      dQueue1 = AdjustDCServo(servo1, pot1, angle, dQueue1, &dSum1, &lastError1, kp1,kd1);
  }
  else if (motorNum == 2){
      ExtraSimple(servo2, pot2, angle, kp2);
  }
  else{
      ExtraSimple(servo3, pot3, angle, kp3);
  }
}

// Equivalent to the Arduino map function but for float values. 
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
     float result;
     result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
     return result;
}

