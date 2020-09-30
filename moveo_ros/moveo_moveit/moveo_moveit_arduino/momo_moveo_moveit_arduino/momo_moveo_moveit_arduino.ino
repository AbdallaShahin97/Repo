/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the
   BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.
   Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
      1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
         the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
      2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached

   Publishing to the following ROS topics: joint_steps_feedback
      1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
         accurately

   Author: Jesse Weisberg
*/
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>

#include <moveo_moveit/ArmJointState.h>
#include <Servo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Int64.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <std_msgs/UInt16.h>
//#include <std_msgs/UInt32.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

int count = 0;

//J1
#define STEP1              2
#define DIR1               32

// Joint 1 //J2
#define E1_STEP_PIN        3
#define E1_DIR_PIN         34


//J3
#define Z_STEP_PIN         4
#define Z_DIR_PIN          36


//j4
#define Y_STEP_PIN         5
#define Y_DIR_PIN          38


//j5
#define X_STEP_PIN         6
#define X_DIR_PIN          40

//j6
#define STEP6              7
#define DIR6               42

//j7
#define E0_STEP_PIN        8
#define E0_DIR_PIN         44


AccelStepper joint1(1, STEP1, DIR1);
AccelStepper joint2(1, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint3(1, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint4(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint5(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint6(1, STEP6, DIR6);
AccelStepper joint7(1, E0_STEP_PIN, E0_DIR_PIN);


Servo gripper;
MultiStepper steppers;
int i = 0;
int x = 0;
float m;
float* despose[4];
float joint_step[7];
int joint_status = 0;

ros::NodeHandle nh;
//std_msgs::Int64 msgfee;
//std_msgs::Int32 msg;

std_msgs::UInt16 msg;
//std_msgs::Float64 msg;


//instantiate publisher (for debugging purposes)
ros::Publisher steps("feedback", &msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps) {
  // joint_status = 1;
  joint_status = 1;
  x += 1;

  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6;
  joint_step[6] = arm_steps.position7;
  //joint_step[7] = arm_steps.position8; //gripper position <0-180>
  despose[i] = joint_step;
  i += 1;

}

void gripper_cb( const std_msgs::String& cmd_msg) {
  String s;
  s = cmd_msg.data;
  if (s == "open") {
    gripper.write(0);
    delay(1000);
  }
  else if (s == "grip") {
    gripper.write(73);
    delay(1000);
  }

  //  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180
  //  delay(1000);
  //  int gripperang=cmd_msg.data; // Set servo angle, should be from 0-180
  digitalWrite(13, HIGH - digitalRead(13)); // Toggle led
}

//instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps", arm_cb); //subscribes to joint_steps on arm
ros::Subscriber<std_msgs::String> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>
//ros::Publisher pub("gripper_angle", &msg);

void setup() {
  //put your setup code here, to run once:

  //pinMode(13, OUTPUT);
  // joint_status = 1;
  joint_status = 0;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(12000);
  joint2.setMaxSpeed(10000);
  joint3.setMaxSpeed(10000);   /// try to add acceleration
  joint4.setMaxSpeed(10000);
  joint5.setMaxSpeed(10000);
  joint6.setMaxSpeed(10000);
  joint7.setMaxSpeed(10000);

  joint1.setAcceleration(100000);
  joint2.setAcceleration(100000);
  joint3.setAcceleration(100000);
  joint4.setAcceleration(100000);
  joint5.setAcceleration(100000);
  joint6.setAcceleration(100000);
  joint7.setAcceleration(100000);


  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);
  steppers.addStepper(joint7);

  // Configure gripper servo
  gripper.attach(9);

  digitalWrite(13, 1); //toggle led
}

void loop() {
  //if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command

  if (x == 4) // If command callback (arm_cb) is being called, execute stepper command

  {
    for (int i = 0; i < 4; i = i + 1) {
      despose[i];



      long positions[7];  // Array of desired stepper positions must be long
      positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
      positions[1] = -joint_step[1];
      positions[2] = -joint_step[2];
      positions[3] = joint_step[3];
      positions[4] = -joint_step[4];
      positions[5] = joint_step[5];
      positions[6] = -joint_step[6];


      msg.data = 1;

      //    Serial.println(positions[0]);
      //    Serial.println(-joint_step[0]*640);

      //    Publish back to ros to check if everything's correct
      //   msg.data=positions[0];
      steps.publish(&msg);

      steppers.moveTo(positions);
      //nh.spinOnce();
      steppers.runSpeedToPosition(); // Blocks until all are in position

      //gripper.write(joint_step[6]);  // move gripper after manipulator reaches goal

      //    pub.publish(&msg);
      //    delay(1000);
    }
  }

  //digitalWrite(13, HIGH - digitalRead(13)); //toggle led
  x = 0;
  //gripper.write(gripperang);
  //delay(20);
  nh.spinOnce();
  delay(1);

}
