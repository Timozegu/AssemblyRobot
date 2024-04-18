// Include the library InverseK.h
#include <InverseK.h>
#include <Servo.h>

#define GRIPP  A2  // Arduino pin connected to VRY pin of joystick, used for the gripper
#define VRX_PIN  A3 // Arduino pin connected to VRX pin
#define VRY_PIN  A5 // Arduino pin connected to VRY pin
#define VRZ_PIN  A4 // Arduino pin connected to VRZ pin (corresponds to the VRX of another joystick)

int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis
int zValue = 0; // To store value of the Z axis
int Grippadd = 0; // To store value of the Gripper position
int x =200;
int y =100;
int z = 100;
int Gripp = 0;

Servo joint1;  // create servo objects to control a servo 
Servo joint2;
Servo joint3;
Servo joint4;
Servo Gripper;


float prevang1 = 100;
int val;
  
float a0, a1, a2, a3;


void setup() {

  joint2.attach(9);  //attach the motors to different digital pins
  joint3.attach(10);
  joint4.attach(11);
  Gripper.attach(5);
  
  // Setup the lengths and rotation limits for each link
  Link base, upperarm, forearm, hand;
  Serial.begin(9600);
  base.init(105.6, b2a(0.0), b2a(180.0));  //105.6 mm based on Fusion Structure
  upperarm.init(143.923, b2a(0), b2a(180.0)); //143.923 mmbased on Fusion Structure
  forearm.init(91.703, b2a(0.0), b2a(180.0)); //91.703mm based on Fusion Structure
  hand.init(116.969, b2a(0.0), b2a(180.0));  //116.969mm based on Fusion Structure
  
  // Attach the links to the inverse kinematic model
  InverseK.attach(base, upperarm, forearm, hand);
}

void loop() {

  joint1.attach(6); //For Continuous motor reatach every time.
  
  xValue = analogRead(VRX_PIN);  // read joystick value
  yValue = analogRead(VRY_PIN); // read joystick value
  zValue = analogRead(VRZ_PIN); // read joystick value
  Grippadd = analogRead(GRIPP); // read joystick value
  xValue = map(xValue,220,760,3,-3); //map value from analog to [-3:3] value for integration 
  yValue = map(yValue,220,760,3,-3); //map value from analog to [-3:3] value for integration 
  zValue = map(zValue,220,760,3,-3); //map value from analog to [-3:3] value for integration 
  Grippadd = map(Grippadd,220,760,30,-30); //map value from analog to [-30:30] value for integration 
  x = x + xValue;   //update x coordinate
  y = y - yValue;   //update y coordinate
  z = z + zValue;   //update z coordinate
  Gripp = Gripp- Grippadd; //update Gripper position
  // print data to Serial Monitor on Arduino IDE
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(", y = ");
  Serial.print(y);
  Serial.print(" z = ");
  Serial.println(z);
  Serial.print(" Gripp = ");
  Serial.println(Gripp);
  delay(200);


  // Calculates the angles using inverse kinematics
   if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {
    Serial.print('a');Serial.print('0');Serial.print('=');Serial.print(a2b(a0)); Serial.print('\n');
    Serial.print('a');Serial.print('1');Serial.print('=');Serial.print(a2b(a1));Serial.print('\n');
    Serial.print('a');Serial.print('2');Serial.print('=');Serial.print(a2b(a2));Serial.print('\n');
    Serial.print('a');Serial.print('3');Serial.print('=');Serial.println(a2b(a3));
  } else {
    Serial.println("No solution found1!");
  }


  
  float angle1 = a2b(a0);
  float angle2 = a2b(a1);
  float angle3 = a2b(a2);
  float angle4 = a2b(a3);



  if (prevang1 < angle1) { // to know which direction the first continuous motor has to run 
    val = 180;
  }
  if (prevang1> angle1){
    val = 0;
  }
  if (abs(prevang1- angle1) <0.5){ // stop the motor if there is not much change
    val = 94;
  }

prevang1 = angle1;
angle1 = a2b(a0)*0.83; //apply angular velocity to learn time during which motor 1 has to turn
 
 joint1.write(val);  
 delay(angle1);   // send time needed to reach angle value for joint 1
 joint1.write(94);
 joint1.detach();//CT
 delay(100);
 joint2.write(angle2); // send angle value for joint 1
 delay(100);// not CT
 joint3.write(abs(180 -angle3));  // send angle value for joint 1
 delay(100);
 joint4.write(angle4);  // send angle value for joint 1
 delay(100);
  Gripper.write(Gripp);  // send Gripper value value for gripper motor
  
// Quick conversion from the Braccio angle system to radians
float b2a(float b){
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}
