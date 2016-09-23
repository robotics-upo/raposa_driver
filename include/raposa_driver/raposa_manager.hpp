/***********************************************************************/
/**                                                                    */
/** raposa_manager.h                                                   */
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


// TODO: All conversion giraff--> raposa

#ifndef _RAPOSA_MANAGER_HPP_
#define _RAPOSA_MANAGER_HPP_

// Activate this to show some debug information
// #define _RAPOSA_MANAGER_DEBUG_

#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <exception>
#include "raposa_serial.hpp"
#include "raposa_config.h"
#include "arduimu_v3/imu.hpp"
#include <nav_msgs/Odometry.h>	

//-- HEADERS ----------------------------------------------------------------------------

//! @class RaposaManagerException
//! @brief An exception to throw in case of fatal error
class RaposaManagerException:public std::exception
{
  public:
  RaposaManagerException(const std::string& message) : message(message) {}
  ~RaposaManagerException() throw () {}
  virtual const char* what() const throw() {return message.c_str();}
	  
  private:
  std::string message;
};

//! \struct RaposaState Gather all relevant state variables of the Raposa
struct RaposaState
{
  bool is_stopped;
  double arm_value_deg;
  double linear_velocity, x, y;
  nav_msgs::Odometry odom;
  
  int64_t total_left_ticks, total_right_ticks;
  
  timespec last_update;
};

//! @class RaposaManager                                                          */
//! @brief This manages the Raposa controller. Includes velocity and arm commands

class RaposaManager
{
  public:
  //! @brief Recommended constructor
  //! @param device Device name where the Raposa is linkedsuch as "/dev/ttyUSB0"
  //! @param params Configuration parameters of the Raposa
  //!  Comments:
  //!     The serial connections will be stablished in the constructor, if some
  //!     error happens, a RaposaManagerException will be thrown.
  RaposaManager(const std::string& device, const RaposaConfig &config);
  
  //! @brief Destructor. Stops the robot and closes all open connections
  ~RaposaManager();
  
  //! @brief Gets the state of raposa (cmd_vel, measured arm_pos, odometry position...)
  //! Comment: This function should be called in a loop
  bool update();
  
  //! @brief Sets the linear and angular velocity of the raposa
  //! @param linear (IN) -- linear velocity in m/s
  //! @param angular (IN) -- angular velocity in rad/s
  //! @retval true Could establish the velocity
  //! @retval false Errors found while estabilishing the velocity
  bool setVelocity(double linear, double angular);

  //! @brief Gets the incremental move distance (IMDL,IMDR) from enconder increments
  //! @param IMDL (OUT) The IMD for left wheel in meters
  //! @param IMDR (OUT) The IMD for right wheel in meters
  bool getIMD(double& imdl, double& imdr);

  //! @brief Sets the arm to the specified value.
  //! @param arm_deg Desired arm angle, in degrees.
  bool setArm(double arm_deg);
  
  //! @brief Sets the arm to the specified value.
  //! @param arm_deg Normalized value between [-1,1] (0 == 0 deg)
  bool setArmNorm(double arm_deg);

  //! @brief Gets the arm position
  //! @return The current arm angle, in radians.
  bool getArm(double &arm);
  
  //! @brief Determines if the robot is stopped
  //! @retval true The robot is stopped
  //! @retval false The robot is moving	
  bool isStopped();
  
  inline double resetOdometry() 
  {
    // Initialize positions
    state.odom.pose.pose.position.x = state.odom.pose.pose.position.y = state.odom.pose.pose.position.z = 0.0;
    state.total_left_ticks = state.total_right_ticks = 0;
    
    // Initialize orientations
    state.odom.pose.pose.orientation.x = state.odom.pose.pose.orientation.y = state.odom.pose.pose.orientation.z = 0.0;
    state.odom.pose.pose.orientation.w = 1.0;
    
    // Discard the first measure and raise the flag
    first_odometry = true;
    update();
  }
  

  //! @brief accessor to the state of Raposa
  inline const RaposaState &getState() const {return state;}
  
  
  //! --------------------- RAW commands for calibration purposes
  
  //! @brief Gets the raw arm position
  //! @return The current arm angle.
  bool getRawArm(int &arm);
  
  //! @brief Sets the raw velocities of each motor
  //! @param left Left velocity
  //! @param right Right velocity
  //! @retval true Success
  //! @retval false Error
  bool setRawVelocity(int16_t left, int16_t right);
  
  //! @brief Inits the IMU from the arduimu_v3 package
  //! @param freq Operating frequency of the IMU
  //! @retval true Success
  //! @retval false Error
  bool setIMU(double freq);
  
  private:
  RaposaSerial raposa_serial;
  RaposaConfig _config;
  RaposaState state;
  
  bool first_odometry;
  
  IMU *imu_;
  
  //! @brief Calculates the increments of the position and orientation variables of the robot by integrating odometry measures
  //! @retval true Got ticks from the encoders and performed the calculations
  //! @retval false Some error while retrieving the ticks
  bool calculateOdometry();
  
};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)

/***********************************/
/** RaposaManager implementation   */
/***********************************/

inline bool RaposaManager::setIMU(double freq)
{
  if (imu_) {
    delete imu_;
  }
  imu_ = new IMU(freq);
  
  ros::Rate r(0.5);
  while (!imu_->isInit() && ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  if (ros::ok()) {
    ROS_INFO("IMU initialized ok");
  }
  
  return true;
}


inline RaposaManager::RaposaManager(const std::string& serial_device, const RaposaConfig &config) :
raposa_serial(serial_device), _config(config),first_odometry(true),imu_(NULL)
{
  state.is_stopped = true;
  
  #ifdef _RAPOSA_MANAGER_DEBUG_
  _printTime();
  std::cout << "Connecting to Raposa"<<std::endl;
  #endif
  
  if (!raposa_serial.open()) {
    
#ifdef _RAPOSA_MANAGER_DEBUG_
    std::cerr << "Could not connect to raposa.\n";
#endif
    
    throw RaposaManagerException(raposa_serial.getLastError());
  } else {
#ifdef _RAPOSA_MANAGER_DEBUG_
    _printTime();
    std::cout << "Connected to Raposa"<<std::endl;
#endif
  }
  // Stop robot
  setVelocity(0.0f, 0.0f);
  resetOdometry();
}

inline RaposaManager::~RaposaManager() {
  if (raposa_serial.isOpen()) {
    // If the connection was open --> stop Raposa and... 
    setVelocity(0.0f, 0.0f);
    // TODO: move the arm to 0.0?
  }
}

// TODO: check it!
inline bool RaposaManager::getIMD(double& imdl, double& imdr)
{
  unsigned char command[1];
  command[0] = _config.get_enc;
  
  bool ret_val = raposa_serial.write(command, 1);
  unsigned char buffer[8];
  
#ifdef _RAPOSA_MANAGER_DEBUG_
//   if (!ret_val) {
//     std::cout << "Error in write" << std::endl;
//   } else {
//     std::cout << "Write OK" << std::endl;
//   }
#endif
  
  int16_t right, left;

  if (ret_val && raposa_serial.getResponse(buffer, 8) && !first_odometry) {
    right = (int16_t)(((unsigned char)buffer[1] << 8) + (unsigned char)buffer[2]);
    // The raposa control board returns directly the increment. But the left encoder is in the opposite direction
    left =  -(int16_t)(((unsigned char)buffer[3] << 8) + (unsigned char)buffer[4]);
  }
  
#ifdef _RAPOSA_MANAGER_DEBUG_
  if  (right != 0 || left != 0 && !first_odometry) 
  {
    std::cout << "right = " << right << "\t left = " << left << std::endl;
  }
#endif

// 	
  if (left == 0 && right == 0) {
    state.is_stopped = true;
    imdl = 0.0;
    imdr = 0.0;
  }
  else
  {
    state.is_stopped = false;
    
    if (left > 0) {
      imdl = (double)left * _config.meters_tick_l;
    } else {
      imdl = (double)left * _config.meters_tick_l_b;
    }
    if (right > 0) {
      imdr = (double)right * _config.meters_tick_r;
    } else {
      imdr = (double)right * _config.meters_tick_r_b;
    }
    state.total_left_ticks += left;
    state.total_right_ticks += right;
  }
  
  return ret_val;
}

inline bool RaposaManager::isStopped()
{
  return state.is_stopped;
}

inline bool RaposaManager::setArm(double arm_deg)
{
  unsigned char command[3];
  command[0] = _config.set_arm;
  bool ret_val = true;
  
  // TODO: conversion between degrees and int?
  arm_deg = _config.arm_sat.apply(arm_deg);
  int arm_pos = ( (arm_deg - _config.arm_sat._low) / (_config.arm_sat._high - _config.arm_sat._low) ) * ((double)(_config.arm_int_sat._high - _config.arm_int_sat._low));
  arm_pos += _config.arm_int_sat._low;
  
#ifdef _RAPOSA_MANAGER_DEBUG_
  std::cout << "RaposaManager::setArm --> arm_deg = " << arm_deg << "\t arm_pos = " << arm_pos << std::endl;
#endif
  
  char aux1, aux2;
  aux1 = (unsigned char)(arm_pos >> 8);
  aux2 = (unsigned char)(arm_pos & 0xFF);
  command[1] = aux1;
  command[2] = aux2;
  
  ret_val = raposa_serial.write(command, 3);
  
  // Wait for a response (checksum
  unsigned char response[4];
  ret_val = raposa_serial.getResponse(response, 4);
  
  return ret_val;
}

inline bool RaposaManager::getArm(double &arm_value)
{
  unsigned char command[1];
  command[0] = _config.get_arm;
  
  bool ret_val;
  
  ret_val = raposa_serial.write(command, 1);
  
  if (ret_val) {
    unsigned char buf[6];
    ret_val = raposa_serial.getResponse(buf, 6);
    
    // TODO: from int to double
    arm_value = (int)buf[1]*256 + (int)buf[2];
  }
  
  return ret_val;
}

inline bool RaposaManager::getRawArm(int &arm_value)
{
  unsigned char command[1];
  command[0] = _config.get_arm;
  bool ret_val;
  
  ret_val = raposa_serial.write(command, 1);
  
  if (ret_val) {
    unsigned char buf[6];
    ret_val = raposa_serial.getResponse(buf, 6);
    
    arm_value = (int)buf[1]*256 + (int)buf[2];
  }
  
  return ret_val;
}

bool RaposaManager::setArmNorm(double arm)
{
  double new_val = arm * _config.arm_sat._high;
  
  return setArm(new_val);
}

// Updates all but the odometry? --> TODO: integrate IMD, and last velocity
inline bool RaposaManager::update()
{
  bool ret_val = true;
  
  getArm(state.arm_value_deg);
  
  calculateOdometry();
  
  return ret_val;
}


inline bool RaposaManager::setRawVelocity(int16_t left, int16_t right) {
  bool ret_val = true;
  
  // get the right command
  unsigned char command[5];
  command[0] = _config.set_vel;
  command[1] = (unsigned char)(right >> 8);
  command[2] = (unsigned char)(right & 0xFF);
  command[3] = (unsigned char)(left >> 8);
  command[4] = (unsigned char)(left & 0xFF);
  
  // Send it
  ret_val = raposa_serial.write(command, 5);
  
  // Wait for response
  unsigned char response[4];
  ret_val = raposa_serial.getResponse(response, 4);
  
  return ret_val;
}

inline bool RaposaManager::setVelocity(double linear, double angular)
{
  unsigned char command[5];
  bool ret_val = true;
  
  command[0] = _config.set_vel;
  
  linear = _config.velocity_sat.apply(linear);
  angular = _config.velocity_sat.apply(angular);

  // Following idMind driver --> v_r = (linear - L * angular)    ; v_l = -(linear + L * angular) 
  double left_speed = -(linear + _config.half_axis_distance * angular);
  double right_speed = (linear - _config.half_axis_distance * angular);
  
  left_speed = _config.velocity_sat.apply(left_speed);
  right_speed = _config.velocity_sat.apply(right_speed);
  
  // Convert from left and right speed velocities to [-1100, 1100]
  // TODO: this is deprecated
  int v_left = left_speed * ((double)_config.vel_int_sat._high) / (_config.velocity_sat._high * _config.peri_wheel);
  int v_right = right_speed * ((double)_config.vel_int_sat._high) / (_config.velocity_sat._high * _config.peri_wheel);
  
  // TODO: Use the empirical relationship
//    v_left = -( linear * (linear > 0.0)? _config.v_m_s_to_v_raw : _config.v_m_s_to_v_raw_b + angular * (angular > 0.0)?_config.ang_vel_to_raw_l:_config.ang_vel_to_raw_r);
//    v_right = linear * (linear > 0.0)? _config.v_m_s_to_v_raw : _config.v_m_s_to_v_raw_b - angular * (angular > 0.0)?_config.ang_vel_to_raw_l:_config.ang_vel_to_raw_r;
  
  #ifdef _RAPOSA_MANAGER_DEBUG_
  if (linear > 0.01 || angular > 0.01) {
    std::cout << "RaposaManager::setVelocity --> v = " << linear << "\t w = " << angular << "\t ";
    std::cout << "\t l_speed = " << left_speed << "\t r_speed = " << right_speed << "\t ";
    std::cout << "\t l_int = " << v_left << "\t r_int = " << v_left << std::endl;
  }
#endif
  
  // get the right command
  command[1] = (unsigned char)(v_right >> 8);
  command[2] = (unsigned char)(v_right & 0xFF);
  command[3] = (unsigned char)(v_left >> 8);
  command[4] = (unsigned char)(v_left & 0xFF);

  ret_val = raposa_serial.write(command, 5);
  
  // Wait for response
  unsigned char response[4];
  ret_val = raposa_serial.getResponse(response, 4);
  
  return ret_val;
}

bool RaposaManager::calculateOdometry()
{
  double dl, dr;
  struct timespec curr_time;
  clock_gettime(CLOCK_REALTIME, &curr_time);
  bool ret_val = getIMD(dl, dr);
  
  if (ret_val) {
    double dist = ((dl + dr) * 0.5);
    dist = _config.encoders_dead.apply(dist);
    
    double d_theta = (dr - dl) / _config.estimated_diag;	 	//rad
    
    if (first_odometry) {
      first_odometry = false;
      clock_gettime(CLOCK_REALTIME, &state.last_update);
    } else {
      // Actualize the linear and angular speeds
      double d_t = curr_time.tv_sec - state.last_update.tv_sec + (curr_time.tv_nsec - state.last_update.tv_nsec)*1e-9;
      state.linear_velocity = dist / d_t;
      double angular_rate = d_theta / d_t;
      double yaw = tf::getYaw(state.odom.pose.pose.orientation);
      state.odom.header.stamp = ros::Time(curr_time.tv_sec, curr_time.tv_nsec);
      // Correct the angles with the IMU, if available
      if (imu_) {
	sensor_msgs::Imu angles = imu_->getFilteredAngles(angular_rate);
	state.odom.pose.pose.orientation = angles.orientation;
	state.odom.twist.twist.angular = angles.angular_velocity;
      } else {
	state.odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw + d_theta);
	state.odom.twist.twist.angular.z = angular_rate;
      }
      double eff_angle = yaw + state.odom.twist.twist.angular.z * d_t * 0.5;
      // We use the odometry model in Thurn, Burgard, Fox; considering that the two turns have equal value (half of the total)
      state.odom.twist.twist.linear.x = state.linear_velocity * cos(eff_angle);
      state.odom.twist.twist.linear.y = state.linear_velocity * sin(eff_angle);
      state.odom.twist.twist.linear.z = 0.0; // TODO: 3D odometry?
      state.odom.pose.pose.position.x += dist * cos(eff_angle);
      state.odom.pose.pose.position.y += dist * sin(eff_angle);
      state.odom.pose.pose.position.z = 0.0; // TODO: 3D odometry?
      
      state.last_update = curr_time;
    }
  }
  
  return ret_val;
}

//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
