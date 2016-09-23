 /***********************************************************************/
/**                                                                    */
/** raposa_node.h                                                      */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
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
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <boost/bind.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include "raposa_driver/raposa_manager.hpp"


#define FREQ		 20.0
#define FREQ_IMU	150.0
#define ARM_TIMEOUT 	  2.0
#define VEL_TIMEOUT 	  0.5

RaposaManager *raposa = NULL;

ros::Time cmd_vel_time;
ros::Time cmd_arm_time;
ros::Time cmd_norm_arm_time;
double arm_timeout;
double vel_timeout;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  cmd_vel_time = ros::Time::now();
  raposa->setVelocity(cmd_vel->linear.x,cmd_vel->angular.z);
}

void cmdArmReceived(const std_msgs::Float32ConstPtr & arm_value) 
{
  cmd_arm_time = ros::Time::now();
  raposa->setArm(arm_value->data);
}

void cmdArmNormReceived(const std_msgs::Float32ConstPtr & norm_arm_value) 
{
  // Priorize cmdarm over cmd_arm_norm
  cmd_norm_arm_time = ros::Time::now();
  if ( (cmd_norm_arm_time - cmd_arm_time).toSec() > arm_timeout) {
    raposa->setArmNorm(norm_arm_value->data);
  }
}

int main(int argc, char** argv)
{
  double dt;

  // Pointers to be freed, here!
//   robot_state_publisher::RobotStatePublisher *raposa_pub = NULL;
  try
  {
    ros::init(argc, argv, "RaposaNode");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    std::string raposa_port;

    pn.param<std::string>("raposa_device", raposa_port, "/dev/serial/by-id/usb-FTDI_MM232R_USB_MODULE_FTB3LEN7-if00-port0");

    std::string base_frame_id;
    std::string odom_frame_id;
    std::string base_ticks_id;
    std::string odom_ticks_id;
    bool use_imu;

    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    pn.param<bool>("use_imu", use_imu, false);
    pn.param<double>("arm_timeout", arm_timeout, ARM_TIMEOUT);
    pn.param<double>("vel_timeout", vel_timeout, VEL_TIMEOUT);
    
    bool publish_tf;
    pn.param<bool>("publish_tf", publish_tf, false);
    

    double freq, freq_imu;
    pn.param<double>("freq",freq,FREQ);
    pn.param<double>("freq_imu",freq_imu,FREQ_IMU);
    
    dt = 1.0 / freq; // Do not forget the dt :)
    ros::Publisher odom_pub = pn.advertise<nav_msgs::Odometry>(odom_frame_id, 5);
    ros::Publisher odom_ticks_pub = pn.advertise<nav_msgs::Odometry>(odom_ticks_id, 5);
    ros::Publisher raw_arm_pub = pn.advertise<std_msgs::Int16>("/arm_pos", 1);
    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,cmdVelReceived);
    ros::Subscriber cmd_arm_sub = n.subscribe<std_msgs::Float32>("/cmd_arm",1,cmdArmReceived);
    ros::Subscriber cmd_arm_norm_sub = n.subscribe<std_msgs::Float32>("/cmd_arm_norm",1,cmdArmNormReceived);
    ros::Time current_time,last_time;
    last_time = ros::Time::now();
    ros::Rate r(freq);
    tf::TransformBroadcaster tf_broadcaster;
    
    RaposaConfig raposa_config;
    try {
      raposa = new RaposaManager(raposa_port, raposa_config);
      ROS_INFO("Connected to Raposa OK");
      if (use_imu) {
	raposa->setIMU(freq_imu);
      }
    } catch (std::exception &e) {
      ROS_ERROR("Exception thrown while connecting to Raposa. Content: %s", e.what());
      return -1;
    }
    
    int raw_arm_pos;
    std_msgs::Int16 raw_arm_msg;
    
    raposa->setVelocity(0.0 ,0.0);
    RaposaState raposa_state;
    
    cmd_vel_time = ros::Time::now() - ros::Duration(vel_timeout);
    cmd_arm_time = ros::Time::now() - ros::Duration(arm_timeout);

    // Save the proper fields
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::Imu imu_msg;
    bool first_msg = true;
    while (n.ok()) {
      // Sleep and actualize
      r.sleep();	
      ros::spinOnce();
     
      
      current_time = ros::Time::now();
      double cmd_vel_sec = (current_time - cmd_vel_time).toSec();

      if (cmd_vel_sec >= vel_timeout) {
	raposa->setVelocity(0.0 ,0.0);
      } 

      //       // Get and publish raw Arm value
      raposa->getRawArm(raw_arm_pos);
      raw_arm_msg.data = raw_arm_pos;
      raw_arm_pub.publish(raw_arm_msg);

      // Odometry: call a external function
      
      raposa->update();
      dt = (current_time - last_time).toSec();
      last_time = current_time;
      
      raposa_state = raposa->getState();
      
      // ******************************************************************************************
      // first , we'll publish the odometry message over ROS
      nav_msgs::Odometry odom = raposa_state.odom;
      odom.header.frame_id = odom_frame_id;
      odom_pub.publish(odom);
		
      // then , we'll publish the transforms over tf
      odom_trans.transform.rotation = raposa_state.odom.pose.pose.orientation;
      odom_trans.transform.translation.x = raposa_state.odom.pose.pose.position.x;
      odom_trans.transform.translation.y = raposa_state.odom.pose.pose.position.y;
      odom_trans.transform.translation.z = raposa_state.odom.pose.pose.position.z;
      odom_trans.header.frame_id = odom_frame_id;
      odom_trans.child_frame_id = base_frame_id;
      odom_trans.header.stamp = ros::Time::now();
      
      // Publish the odometry TF and odom message
      if (publish_tf) {
        tf_broadcaster.sendTransform(odom_trans);
      }
    }
  }
  catch(RaposaManagerException e) {
    ROS_WARN("%s",e.what());
//     ROS_BREAK();
  }
  
  // Free memory
  delete raposa;
//   delete raposa_pub;
  
  return 0;
}
