

///////////////////////////////////  Sends motor_steps once  ///////////////////////////////////////////////////

// #include "ros/ros.h"
// #include "sensor_msgs/JointState.h"
// #include "moveo_moveit/ArmJointState.h"
// #include "math.h"

// moveo_moveit::ArmJointState arm_steps;
// moveo_moveit::ArmJointState total;

// int stepsPerRevolution[7] = {160,6400,6400,6400,6400,6400,6400}; // microsteps/revolution (using 16ths) from observation, for each motor
// int joint_status = 0;
// double cur_angle[7];
// int joint_step[7];
// double prev_angle[7] = {0,0,0,0,0,0,0}; 
// double prevv_angle[7] = {0,0,0,0,0,0,0}; 
// double init_angle[7] = {0,0,0,0,0,0,0};
// double total_steps[7] = {0,0,0,0,0,0,0};
// int count = 0;
// int momo = 0;


// //keep a running sum of all the step counts and use that as the final step to send to arduino accelstepper

// // int angle_to_steps(double x)
// // {
// //   float steps;
// //   steps=((x / M_PI)*stepsPerRevolution)+0.5; // (radians)*(1 revolution/PI radians)*(200 steps/revolution)
// //   return steps;
// // }

// //command callback (for position) function 
// void cmd_cb(const sensor_msgs::JointState& cmd_arm)
// {
//   if (count==0){
//     prev_angle[0] = cmd_arm.position[0];
//     prev_angle[1] = cmd_arm.position[1];
//     prev_angle[2] = cmd_arm.position[2];
//     prev_angle[3] = cmd_arm.position[3];
//     prev_angle[4] = cmd_arm.position[4];
//     prev_angle[5] = cmd_arm.position[5];
//     prev_angle[6] = cmd_arm.position[6];


//     init_angle[0] = cmd_arm.position[0];
//     init_angle[1] = cmd_arm.position[1];
//     init_angle[2] = cmd_arm.position[2];
//     init_angle[3] = cmd_arm.position[3];
//     init_angle[4] = cmd_arm.position[4];
//     init_angle[5] = cmd_arm.position[5];
//     init_angle[6] = cmd_arm.position[6];

//   }

//   // ros::NodeHandle nh;
//   // ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
//   // ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps",50);
//   ROS_INFO_STREAM("Received /move_group/fake_controller_joint_states");
    
//   // arm_steps.position1 = (cmd_arm.position[0]*stepsPerRevolution[0]/M_PI+0.5)-prev_angle[0];
//   // arm_steps.position2 = (cmd_arm.position[1]*stepsPerRevolution[1]/M_PI+0.5)-prev_angle[1];
//   // arm_steps.position3 = (cmd_arm.position[2]*stepsPerRevolution[2]/M_PI+0.5)-prev_angle[2];
//   // arm_steps.position4 = (cmd_arm.position[3]*stepsPerRevolution[3]/M_PI+0.5)-prev_angle[3];
//   // arm_steps.position5 = (cmd_arm.position[4]*stepsPerRevolution[4]/M_PI+0.5)-prev_angle[4];
//   // arm_steps.position6 = (cmd_arm.position[5]*stepsPerRevolution[5]/M_PI+0.5)-prev_angle[5];

//   //compute relative step count to move each joint-- only works if all joint_angles start at 0
//   //otherwise, we need to set the current command to the initial joint_angles
//   //ROS_INFO_NAMED("test", "cmd_arm.position[4]: %f, prev_angle[4]: %f, init_angle[4]: %f", cmd_arm.position[4], prev_angle[4], init_angle[4]);
//   //ROS_INFO_NAMED("test", "arm_steps.position5 #1: %f", (cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/M_PI);

//   arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]*(640));
//   arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]*(8)/(2*M_PI));
//   arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]*(60)/(2*M_PI));
//   arm_steps.position4 = (int)((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]*(20)/(2*M_PI));
//   arm_steps.position5 = (int)((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]*(10)/(2*M_PI));
//   arm_steps.position6 = (int)((cmd_arm.position[5]-prev_angle[5])*stepsPerRevolution[5]*(10)/(2*M_PI));

//   ROS_INFO_NAMED("test", "arm_steps.position5 #2: %d", arm_steps.position5);

//   if (prev_angle[0,1,2,3,4,5,6] == cmd_arm.position[0,1,2,3,4,5,6] && prev_angle[0,1,2,3,4,5,6] != prevv_angle[0,1,2,3,4,5,6]){

//   arm_steps.position1 = (int)((cmd_arm.position[0]-init_angle[0])*stepsPerRevolution[0]*(4.25)*1000);     //max rail distance=0.791367 m   ,   max steps for this value is = 17833
//   arm_steps.position2 = (int)((cmd_arm.position[1]-init_angle[1])*stepsPerRevolution[1]*(4.25*2)/(2*M_PI));
//   arm_steps.position3 = (int)((cmd_arm.position[2]-init_angle[2])*stepsPerRevolution[2]*(15.3*4)/(2*M_PI));  
//   arm_steps.position4 = (int)((cmd_arm.position[3]-init_angle[3])*stepsPerRevolution[3]*(5.18*4)/(2*M_PI));
//   arm_steps.position5 = (int)((cmd_arm.position[4]-init_angle[4])*stepsPerRevolution[4]*(5.18*2)/(2*M_PI));
//   arm_steps.position6 = (int)((cmd_arm.position[5]-init_angle[5])*stepsPerRevolution[5]*(5.18*2)/(2*M_PI));
//   arm_steps.position7 = (int)((cmd_arm.position[6]-init_angle[6])*stepsPerRevolution[6]*(5.18*2)/(2*M_PI));

//   prevv_angle[0] = prev_angle[0];
//   prevv_angle[1] = prev_angle[1];
//   prevv_angle[2] = prev_angle[2];
//   prevv_angle[3] = prev_angle[3];
//   prevv_angle[4] = prev_angle[4];
//   prevv_angle[5] = prev_angle[5];
//   prevv_angle[6] = prev_angle[6];

//   momo=1;

//   }

//   if (count!=0){
//     prev_angle[0] = cmd_arm.position[0];
//     prev_angle[1] = cmd_arm.position[1];
//     prev_angle[2] = cmd_arm.position[2];
//     prev_angle[3] = cmd_arm.position[3];
//     prev_angle[4] = cmd_arm.position[4];
//     prev_angle[5] = cmd_arm.position[5];
//     prev_angle[6] = cmd_arm.position[6];

//   }


// /*  ROS_INFO_NAMED("test", "total_steps[4]: %f, total: %d", total_steps[4], total.position5);*/
//   ROS_INFO_NAMED("test", "cmd_arm.position[0] %f", cmd_arm.position[0]);
//   ROS_INFO_NAMED("test", "init_angle[0] %f", init_angle[0]);
// //  ROS_INFO_NAMED("test", "cmd_arm.position[1] %d", arm_steps.position2);

//   ROS_INFO_STREAM("Done conversion to /joint_steps");
//   joint_status = 1;
//   count=1;
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "moveo_moveit");
//   ros::NodeHandle nh;
//   ROS_INFO_STREAM("In main function");
//   ros::Subscriber sub = nh.subscribe("/joint_states",1000,cmd_cb);
//   ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("/joint_steps",100);

//   // ros::Publisher sub = nh.advertise<moveo_moveit::ArmJointState>("/joint_stepsss",500); /// ZIZOOO

//   //ros::Rate loop_rate(20); // Joe opened it

//   while (ros::ok())
//   {

//      if(momo == 1)
//     {      
//       momo = 0;
//       // pub.publish(arm_steps.position1);
//       // sleep(3);
//       pub.publish(arm_steps);

//       ROS_INFO_STREAM("Ourcodes  moveo_ros momom ");
// //      pub.publish(total);
//       ROS_INFO_STREAM("Published to /joint_steps");
//     }


//     ros::spinOnce();
//     //loop_rate.sleep();  // Joe opened it
//    }
//   ros::spin();
//   return 0;
//}

///////////////////////////////////  Sends motor_steps continously  ///////////////////////////////////////////////////

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "moveo_moveit/ArmJointState.h"
#include "math.h"

moveo_moveit::ArmJointState arm_steps;
moveo_moveit::ArmJointState total;

int stepsPerRevolution[7] = {160,6400,6400,6400,6400,6400,6400}; // microsteps/revolution (using 16ths) from observation, for each motor
int joint_status = 0;
double cur_angle[7];
int joint_step[7];
double prev_angle[7] = {0,0,0,0,0,0,0}; 
double init_angle[7] = {0,0,0,0,0,0,0};
double total_steps[7] = {0,0,0,0,0,0,0};
int count = 0;
int momo = 0;


//keep a running sum of all the step counts and use that as the final step to send to arduino accelstepper

// int angle_to_steps(double x)
// {
//   float steps;
//   steps=((x / M_PI)*stepsPerRevolution)+0.5; // (radians)*(1 revolution/PI radians)*(200 steps/revolution)
//   return steps;
// }

//command callback (for position) function 
void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
  if (count==0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];


    init_angle[0] = cmd_arm.position[0];
    init_angle[1] = cmd_arm.position[1];
    init_angle[2] = cmd_arm.position[2];
    init_angle[3] = cmd_arm.position[3];
    init_angle[4] = cmd_arm.position[4];
    init_angle[5] = cmd_arm.position[5];
    init_angle[6] = cmd_arm.position[6];

  }

  // ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
  // ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps",50);
  ROS_INFO_STREAM("Received /move_group/fake_controller_joint_states");
    
  // arm_steps.position1 = (cmd_arm.position[0]*stepsPerRevolution[0]/M_PI+0.5)-prev_angle[0];
  // arm_steps.position2 = (cmd_arm.position[1]*stepsPerRevolution[1]/M_PI+0.5)-prev_angle[1];
  // arm_steps.position3 = (cmd_arm.position[2]*stepsPerRevolution[2]/M_PI+0.5)-prev_angle[2];
  // arm_steps.position4 = (cmd_arm.position[3]*stepsPerRevolution[3]/M_PI+0.5)-prev_angle[3];
  // arm_steps.position5 = (cmd_arm.position[4]*stepsPerRevolution[4]/M_PI+0.5)-prev_angle[4];
  // arm_steps.position6 = (cmd_arm.position[5]*stepsPerRevolution[5]/M_PI+0.5)-prev_angle[5];

  //compute relative step count to move each joint-- only works if all joint_angles start at 0
  //otherwise, we need to set the current command to the initial joint_angles
  //ROS_INFO_NAMED("test", "cmd_arm.position[4]: %f, prev_angle[4]: %f, init_angle[4]: %f", cmd_arm.position[4], prev_angle[4], init_angle[4]);
  //ROS_INFO_NAMED("test", "arm_steps.position5 #1: %f", (cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/M_PI);

  /*arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]*(640));
  arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]*(8)/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]*(60)/(2*M_PI));
  arm_steps.position4 = (int)((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]*(20)/(2*M_PI));
  arm_steps.position5 = (int)((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]*(10)/(2*M_PI));
  arm_steps.position6 = (int)((cmd_arm.position[5]-prev_angle[5])*stepsPerRevolution[5]*(10)/(2*M_PI));
*/
  ROS_INFO_NAMED("test", "arm_steps.position5 #2: %d", arm_steps.position5);

  if (prev_angle[0,1,2,3,4,5,6] == cmd_arm.position[0,1,2,3,4,5,6]){

  arm_steps.position1 = (int)((cmd_arm.position[0]-init_angle[0])*stepsPerRevolution[0]*(4.25)*1000);     //max rail distance=0.791367 m   ,   max steps for this value is = 17833
  arm_steps.position2 = (int)((cmd_arm.position[1]-init_angle[1])*stepsPerRevolution[1]*(4.25*2)/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-init_angle[2])*stepsPerRevolution[2]*(15.3*4)/(2*M_PI));
  arm_steps.position4 = (int)((cmd_arm.position[3]-init_angle[3] + 0)*stepsPerRevolution[3]*(5.18*4)/(2*M_PI));
  arm_steps.position5 = (int)((cmd_arm.position[4]-init_angle[4])*stepsPerRevolution[4]*(5.18*2)/(2*M_PI));
  arm_steps.position6 = (int)((cmd_arm.position[5]-init_angle[5])*stepsPerRevolution[5]*(5.18*2)/(2*M_PI));
  arm_steps.position7 = (int)((cmd_arm.position[6]-init_angle[6])*stepsPerRevolution[6]*(5.18*2)/(2*M_PI));


/*
  if (count!=0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];

  }

  //total steps taken to get to goal
  // total_steps[0]+=arm_steps.position1;
  // total_steps[1]+=arm_steps.position2;
  // total_steps[2]+=arm_steps.position3;
  // total_steps[3]+=arm_steps.position4;
  // total_steps[4]+=arm_steps.position5;

  total.position1 += arm_steps.position1;
  total.position2 += arm_steps.position2;
  total.position3 += arm_steps.position3;
  total.position4 += arm_steps.position4;
  total.position5 += arm_steps.position5;
  total.position6 += arm_steps.position6;
  total.position7 += arm_steps.position7;


  ROS_INFO_NAMED("test", "total_steps[4]: %f, total: %d", total_steps[4], total.position5);
  ROS_INFO_NAMED("test", "arm_steps.position5 #3: %d", arm_steps.position5);

  ROS_INFO_STREAM("Done conversion to /joint_steps");
  joint_status = 1;
  count=1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveo_moveit");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("In main function");
  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
  ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps",50);
  
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    if(joint_status==1)
    {
      joint_status = 0;
      pub.publish(arm_steps);
      //pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
    }
    ros::spinOnce();
    loop_rate.sleep();  
  }

  //ros::spin();
  return 0;
}

*/
    momo=1;
    }

  if (count!=0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];

  }

  //total steps taken to get to goal
  /*total_steps[0]+=arm_steps.position1;
  total_steps[1]+=arm_steps.positionon2;
  total_steps[2]+=arm_steps.position3;
  total_steps[3]+=arm_steps.position4;
  total_steps[4]+=arm_steps.position5;
 */
/*  total.position1 += arm_steps.position1;
  total.position2 += arm_steps.position2;
  total.position3 += arm_steps.position3;
  total.position4 += arm_steps.position4;
  total.position5 += arm_steps.position5;
  total.position6 += arm_steps.position6;

/*  ROS_INFO_NAMED("test", "total_steps[4]: %f, total: %d", total_steps[4], total.position5);*/
  ROS_INFO_NAMED("test", "cmd_arm.position[0] %f", cmd_arm.position[0]);
  ROS_INFO_NAMED("test", "init_angle[0] %f", init_angle[0]);
//  ROS_INFO_NAMED("test", "cmd_arm.position[1] %d", arm_steps.position2);

  ROS_INFO_STREAM("Done conversion to /joint_steps");
  joint_status = 1;
  count=1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveo_moveit");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("In main function");
  ros::Subscriber sub = nh.subscribe("/joint_states",1000,cmd_cb);
  ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("/joint_steps",500);

  // ros::Publisher sub = nh.advertise<moveo_moveit::ArmJointState>("/joint_stepsss",500); /// ZIZOOO

  ros::Rate loop_rate(20);


  // pub.publish(arms);
  // ros::Duration(1).sleep();
  // pub.publish(arm_steps);

  while (ros::ok())
  {
   /* if(joint_status==1)
    {
      joint_status = 0;
      //pub.publish(arm_steps);
      pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
    }
*/
     if(momo == 1)
    {      
      momo = 0;
      // pub.publish(arm_steps.position1);
      // sleep(3);
      pub.publish(arm_steps);

      ROS_INFO_STREAM("Ourcodes  moveo_ros ");
//      pub.publish(total);
      ROS_INFO_STREAM("Published to /joint_steps");
    }


    ros::spinOnce();
//     loop_rate.sleep();  
   }
  ros::spin();
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////