
// INDIVIDUAL JOINT DIRECTION PROGRAM
#include <Servo.h>


Servo joint1;  // create servo objects to control a servo
Servo joint2;
Servo joint3;
Servo joint4;
Servo gripper;





void setup() {
  joint2.attach(9);  //attach the motors to different digital pins
  joint3.attach(10);
  joint4.attach(11);
  gripper.attach(5);
  
/* FOR 1 ITERATION
//gripper.write(180);
gripper.write(0);
delay(1000);
 

 joint2.write(115);
 delay(1000);// not CT
 joint3.write(abs(180-55));
 delay(1000);
 joint4.write(80);
 delay(1000);

 
 gripper.write(180);
 delay(1000);
 
joint2.write(130);
 delay(1000);// not CT
 joint3.write(abs(180-75));
 delay(1000);
 joint4.write(60);
 delay(1000);

 joint1.attach(6);
 joint1.write(0);
 delay(400);
 joint1.detach();

 joint2.write(115);
 delay(1000);// not CT
 joint3.write(abs(180-55));
 delay(1000);
 joint4.write(80);
 delay(1000);

gripper.write(0);
 delay(1000);

joint2.write(130);
 delay(1000);// not CT
 joint3.write(abs(180-75));
 delay(1000);
 joint4.write(60);
 delay(1000);


joint1.attach(6);
 joint1.write(180);
 delay(400);
 joint1.detach();

 joint2.write(170);
 delay(1000);// not CT
 joint3.write(abs(180-90)); 
 delay(1000);
 joint4.write(60);
 delay(1000);


 gripper.write(180);
 delay(1000);
 */
}

void loop() {
  
gripper.write(0);

// FOR MULTIPLE ITERATIONS

// send robot to cylinder position
 joint2.write(115);
 delay(100);// not CT
 joint3.write(abs(180-55));
 delay(100);
 joint4.write(80);
 delay(100);


//Grab cylinder
 gripper.write(180);


//lift cylinder
joint2.write(130);
 delay(100);// not CT
 joint3.write(abs(180-75));
 delay(100);
 joint4.write(60);
 delay(100);


//Move the robot to another place
 joint1.attach(6);
 joint1.write(0);
 delay(600);
 joint1.detach();


 //put the cylinder down
 joint2.write(115);
 delay(100);// not CT
 joint3.write(abs(180-55));
 delay(100);
 joint4.write(80);
 delay(100);

//release the cylinder
 gripper.write(0);


//move the robot up
joint2.write(130);
 delay(100);// not CT
 joint3.write(abs(180-75));
 delay(100);
 joint4.write(60);
 delay(100);

//move robot to another place
joint1.attach(6);
 joint1.write(180);
 delay(600);
 joint1.detach();

//put robot in rest ppsition
 joint2.write(170);
 delay(100);// not CT
 joint3.write(abs(180-90)); 
 delay(100);
 joint4.write(60);
 delay(100);

}
