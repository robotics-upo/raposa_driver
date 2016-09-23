/***********************************************************************/
/**                                                                    */
/** raposa_teleop_joy.h                                                */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include "raposa_driver/raposa_config.h"
#include <stdlib.h>

///////////////////////////////////////////////////
// Default values for buttons and other values   //
// These values can be modified via rosparam     //
// NOTE: Axes inputs range [0,1] in ROS          //
// NOTE: button inputs are 0 or 1 (pressed) (int)//
// It has been configured to control with the   ///
// Logitech wireless pad controller              //

#define STOP_EXIT	      1
#define FREQ                100 
#define PANIC_FREQ          300
#define LINEAR_VELOCITY_AXIS  1
#define ANGULAR_VELOCITY_AXIS 0
#define NORM_ARM_AXIS         3
#define INC_ARM_BUTTON        3
#define DEC_ARM_BUTTON        1
#define SLOW_BUTTON   	      5
#define MAX_VELOCITY_BUTTON   7
#define PANIC_BUTTON          2
#define START_BUTTON          9
#define BACK_BUTTON           8
#define REVERSE_BUTTON        0
#define MAX_LINEAR_VELOCITY   1.0
#define MAX_ANGULAR_VELOCITY  1.5707963

#define MIN_VEL_VARIATION     0.001
#define MIN_ARM_VARIATION     0.01
#define ARM_INCREMENT         2.0
#define SLOW_MULTIPLIER       0.2

#define MAX_JOY_TIME          8.0

////////////////////////////////////////////////////
// Variables that store the actual values        ///
// The ros-parameters have the same name         ///

RaposaConfig raposa_config;

// General inputs
int panicButton;
int startButton;
int backButton;
int reverseButton; // For enabling backwards mode

// Velocity command related inputs
double maxLinearVelocity;
double maxAngularVelocity;
double slow_multiplier;
int linearVelocityAxis;
int angularVelocityAxis;
int maxVelocityButton;
int slowButton;

// ARM related inputs
double curr_arm;
double norm_arm;
int inc_arm_button, dec_arm_button;
double arm_increment; // Value that will be added to the curr_arm
int norm_arm_axis; // For normalized arm mode
DeadZone<> norm_arm_dead_zone(MIN_ARM_VARIATION);

//////////////////////////////////

double currentLinearVelocity  = 0.0;
double currentAngularVelocity = 0.0;

//////////////////////////////////
// Status flags

bool publishArm     = false;
bool publishArmNorm = false;
bool panic          = false;
bool startPressed   = false;
bool backPressed   = false;
bool backwards = false;
bool ant_reverse_but = false;
bool slow_mode = false;
bool publishSlow = false;

ros::Time last_joy_time;
ros::Publisher reverse_pub;
ros::Publisher slow_pub;

void joyReceived(const sensor_msgs::Joy::ConstPtr& joy)
{
  // First of all, panic mode: if pressed --> the panic mode is activated (TODO: deactivate panic)
  last_joy_time = ros::Time::now();
  startPressed = joy->buttons[startButton] == 1;
  panic = panic | (joy->buttons[panicButton] == 1);
  backPressed = joy->buttons[backButton] == 1;
  if (!ant_reverse_but && joy->buttons[reverseButton] == 1) {
    backwards = !backwards;
    std_msgs::Bool msg;
    msg.data = backwards;
    reverse_pub.publish(msg);
  }
  if (panic)
  {
    currentLinearVelocity = 0.0;
    currentAngularVelocity = 0.0;
  } 
  else
  {
    // If the max velocity button is not pressed --> velocity commands are attenuated
    double multiplier = (joy->buttons[maxVelocityButton] == 0)?0.5:1.0;
    if (slow_mode) {
      multiplier *= slow_multiplier;
    }
    
    // A positive angular velocity will rotate the reference frame to the left
    // While a positive axes value means that the stick is to the right --> change sign
    currentAngularVelocity =-maxAngularVelocity * multiplier * joy->axes[angularVelocityAxis];
    currentLinearVelocity = maxLinearVelocity * multiplier * joy->axes[linearVelocityAxis];
    currentLinearVelocity *= backwards?-1.0:1.0; // Only the linear velocity should change when going backwards
    
    // Incremental arm mode
    if (joy->buttons[inc_arm_button]) {
      curr_arm += arm_increment;
      curr_arm = raposa_config.arm_sat.apply(curr_arm);
      publishArm = true;
    } else if (joy->buttons[dec_arm_button]) {
      curr_arm -= arm_increment;
      curr_arm = raposa_config.arm_sat.apply(curr_arm);
      publishArm = true;
    } else {
      publishArm = false;
      publishArmNorm = false;
      if (fabs(joy->axes[norm_arm_axis] - norm_arm) > MIN_ARM_VARIATION) {
	// Normalized arm mode
	publishArmNorm = true;
	norm_arm = norm_arm_dead_zone.apply(joy->axes[norm_arm_axis]);
      }
    }
    if (joy->buttons[slowButton]) {
      slow_mode = !slow_mode;
      publishSlow = true;
    }
  }
}


void sendCmdVel(double linearVelocity, double angularVelocity, ros::Publisher& vel_pub)
{
  geometry_msgs::Twist vel;
  vel.angular.z = angularVelocity;
  vel.linear.x = linearVelocity;
  vel.linear.z = 0.0;
  vel.linear.y = 0.0;
  vel_pub.publish(vel);
}

void sendArmCmd(double arm_pos, ros::Publisher& arm_pub) {
  std_msgs::Float32 arm;
  arm.data = arm_pos;
  arm_pub.publish(arm);
}

void sendNormArmCmd(double arm_pos, ros::Publisher &pub) {
  std_msgs::Float32 arm;
  arm.data = norm_arm;
  pub.publish(arm);
}

void sendSlowCmd(bool slow_mode, ros::Publisher &pub) {
  std_msgs::Bool msg;
  msg.data = slow_mode;
  slow_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RaposaTeleopJoy");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  bool stopExit;
  pn.param<bool>("stop_on_exit",stopExit, STOP_EXIT);
  
  double freq;
  pn.param<double>("freq",freq,FREQ);
  double panicFreq;
  pn.param<double>("panic_freq",panicFreq,PANIC_FREQ);
  
  pn.param<int>("panic_button", panicButton, PANIC_BUTTON);
  pn.param<int>("start_button", startButton, START_BUTTON);
  pn.param<int>("back_button", backButton, BACK_BUTTON);
  pn.param<int>("reverse_button", reverseButton, REVERSE_BUTTON);
  
  pn.param<int>("linear_velocity_axis",linearVelocityAxis,LINEAR_VELOCITY_AXIS);
  pn.param<int>("angular_velocity_axis",angularVelocityAxis,ANGULAR_VELOCITY_AXIS);
  pn.param<int>("norm_arm_axis", norm_arm_axis, NORM_ARM_AXIS);
  
  pn.param<int>("slow_button", slowButton, SLOW_BUTTON);
  pn.param<int>("turbo_button", maxVelocityButton, MAX_VELOCITY_BUTTON);
  
  pn.param<int>("inc_arm_button", inc_arm_button, INC_ARM_BUTTON);
  pn.param<int>("dec_arm_button", dec_arm_button, DEC_ARM_BUTTON);
  pn.param<double>("arm_increment", arm_increment, ARM_INCREMENT);
  
  pn.param<double>("max_linear_velocity",maxLinearVelocity,MAX_LINEAR_VELOCITY);
  pn.param<double>("max_angular_velocity",maxAngularVelocity,MAX_ANGULAR_VELOCITY);
  
  double max_joy_time = 2.8;
  pn.param<double>("max_joy_time", max_joy_time, MAX_JOY_TIME);
  
  pn.param<double>("slow_multipier", slow_multiplier, SLOW_MULTIPLIER);

  ros::Publisher vel_pub = pn.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher arm_pub = pn.advertise<std_msgs::Float32>("/cmd_arm", 1);
  reverse_pub = pn.advertise<std_msgs::Bool>("/reverse", 1);
  slow_pub = pn.advertise<std_msgs::Bool>("/slow_motion", 1);
  
  ros::Publisher arm_norm_pub = pn.advertise<std_msgs::Float32>("/cmd_arm_norm", 1);
  ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 5, joyReceived);		

  ros::Rate rate(freq);
  ros::Rate panicRate(panicFreq);
  bool panic_showed = false;
  bool start = false;
  bool first_start = true;
  ROS_INFO("Raposa teleop node. Press START to have fun. Press BACK at any moment to exit.");
  while (n.ok() && !backPressed) {
    if (!start) 
    {
      sendCmdVel(0.0, 0.0, vel_pub);
      start = startPressed;
      rate.sleep();
    } 
    else if (panic) 
    {
      sendCmdVel(0.0, 0.0, vel_pub);
      panicRate.sleep();
      if (!panic_showed) 
      {
	panic_showed = true;
	ROS_ERROR("Panic mode ON. Press START to exit panic mode.");
      }
      if (startPressed) 
      {
	panic = false;
	panic_showed = false; // Turn it off for future PANICs
	ROS_INFO("Panic mode OFF. Entering normal mode (already started)"); 
      }
    } 
    else 
    {
      if (first_start) 
      {
	first_start = false;
	ROS_INFO("The show has started, please have fun.");
      }
      if ((ros::Time::now() - last_joy_time).toSec() > max_joy_time ) {
        currentAngularVelocity *= 0.95;
        currentLinearVelocity *= 0.95;
      } 
      sendCmdVel(currentLinearVelocity, currentAngularVelocity, vel_pub);
      
      if (publishArm) 
      {
	sendArmCmd(curr_arm, arm_pub);
// 	publishArm = false;
      }
      if (publishArmNorm) 
      {
	sendNormArmCmd(norm_arm, arm_norm_pub);
// 	publishArmNorm = false;
      }
     
      if (publishSlow) {
        sendSlowCmd(slow_mode, slow_pub);
        publishSlow = false;
      }
      rate.sleep();
    }
    ros::spinOnce();
  }
  if (backPressed) {
    ROS_INFO("Back button pressed --> stopping Raposa and exiting the program. ");
    // Before exiting --> stop Raposa
    sendCmdVel(0.0, 0.0, vel_pub);
    if (stopExit) {
      ROS_INFO("Killing all ros nodes.");
      system("rosnode kill -a");
    }
  }
  
  // Before exiting --> stop Raposa
  sendCmdVel(0.0, 0.0, vel_pub);
  
  return 0;
}
