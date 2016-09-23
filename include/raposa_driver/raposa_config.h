/***********************************************************************/
/**                                                                    */
/** raposa_config.h                                                   */
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

#ifndef __RAPOSA_CONFIG_H__
#define __RAPOSA_CONFIG_H__

#include "raposa_functions.hpp"
#include <stdint.h>

class RaposaConfig {
public:
  // Robot constraints
  Saturate<> velocity_sat; // Saturation in velocity (m/s)
  Saturate<> angular_sat; // Saturation in angular velocity (rad/s)
  DeadZone<> velocity_dead; // Dead zone of the velocity (m/s)
  Saturate<> arm_sat; // Saturation of the arm
  Saturate<int> arm_int_sat, vel_int_sat; // Two integer saturators that will help to make the conversion
  DeadZone<> encoders_dead;
  
  // Relationships
  
  double half_axis_distance, radius, meters_tick, peri_wheel, ticks_revolution, diag_wheels; // In meters
  
  // Empirical relationships
  double meters_tick_l, meters_tick_r; // Odometry calibration
  double meters_tick_l_b, meters_tick_r_b; // Odometry calibration
  double v_m_s_to_v_raw, v_m_s_to_v_raw_b; // Linear speed to raw commands
//   double ang_vel_to_raw_l, ang_vel_to_raw_r; // Relates angular speed to raw commands
  double estimated_diag;
  
  // Robot commands
  unsigned char set_vel, get_enc, set_arm, get_arm;
  
  RaposaConfig(const std::string &filename = "");
  
  void setDefaultConfig();
  
  bool saveConfig(const std::string &filename);
};

RaposaConfig::RaposaConfig(const std::string &filename)
{
  if (filename == "") {
    setDefaultConfig();
  } // TODO: load parameters from file (yaml)
}

// TODO: implement
bool RaposaConfig::saveConfig(const std::string& filename)
{
  return true;
}

void RaposaConfig::setDefaultConfig()
{
  half_axis_distance = 0.2;
  diag_wheels = 0.7;
  radius = 0.085; // 0.085 is the distance from the robot center to the ground
  ticks_revolution = 32250; // See the document API_summary_RaposaNG_motors_2015.pdf
  peri_wheel = 2 * M_PI * radius; // Perimeter of the wheel
  meters_tick = peri_wheel / ticks_revolution; // meters / tick radius
  meters_tick_l = meters_tick;
  meters_tick_r = meters_tick;
  
  // From calibration
  meters_tick_l = 1 / 236413.9;
  meters_tick_r = 1 / 234142.9;
  meters_tick_l_b = 1 / 222666.7;
  meters_tick_r_b = 1 / 217015.8;
  v_m_s_to_v_raw = 1574.482576;
  v_m_s_to_v_raw_b = 1488.852036;
  estimated_diag = 0.55;
  
  // Debug
  std::cerr << "DEBUG: Meters / ticks L = " << meters_tick_l << std::endl;
  std::cerr << "DEBUG: Meters / ticks R = " << meters_tick_r << std::endl;
  std::cerr << "DEBUG: Meters / ticks BL = " << meters_tick_l_b << std::endl;
  std::cerr << "DEBUG: Meters / ticks BR = " << meters_tick_r_b << std::endl;
  
  velocity_dead = DeadZone<>(0.01);
  encoders_dead = DeadZone<>(0.0003);
  
  velocity_sat = Saturate<>(1.12);
  angular_sat = Saturate<>(0.3);
  arm_sat = Saturate<>(-27.0, 40.0);

  
  arm_int_sat = Saturate<int>(444, 690);
  vel_int_sat = Saturate<int>(1100);
  
  set_vel = (char)0x56; // See raposa manual
  get_enc = (char)0x4A;
  get_arm = (char)0x48;
  set_arm = (char)0x42;
}

#endif