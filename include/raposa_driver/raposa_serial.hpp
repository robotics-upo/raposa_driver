/***********************************************************************/
/**                                                                    */
/** giraff_serial.h                                                    */
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

//! TODO: All (see what is necessary)

#ifndef _RAPOSA_SERIAL_H_
#define _RAPOSA_SERIAL_H_

// Activate this to show some debug information about the communications Linux PC <--> Giraff PC
// #define _RAPOSA_SERIAL_DEBUG_

#include "serial_interface.hpp"
#include <sstream>
#include <string>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <iomanip>


//-- HEADERS ----------------------------------------------------------------------------

/*************************************************************************************/
/**                                                                                  */
/** class Action                                                                     */
/**                                                                                  */
/** An abstract class in order to implement the Strategy Design Pattern              */
/** http://en.wikipedia.org/wiki/Strategy_pattern                                    */
/**                                                                                  */
/*************************************************************************************/

class Action
{
	public:
	Action() {}
	virtual ~Action() {}
	virtual bool doAction() {return true;};
} defaultAction;


/************************************************************************************/
/**                                                                                 */
/** class RaposaSerial                                                              */
/**                                                                                 */
/** A class implementing methods for communicating with the Raposa controller       */           
/**                                                                                 */
/** The methods of this class should be called by the main program in this fashion: */
/**                                                                                 */
/**  1.-   open                                                                     */
/**  2.-   LOOP:                                                                    */
/**  2.1.-  checkDSR                                                                */
/**  2.2.-  readCommand                                                             */
/**  2.3.-  ... process command, maybe you want to ask the Giraff AVR...            */
/**  2.4.-  writeResponse                                                           */
/**  3.-   close                                                                    */
/**                                                                                 */
/**                                                                                 */
/************************************************************************************/

class RaposaSerial : public SerialInterface
{
  public:
    
  //! @brief Default constructor*------------------------------------------------- Constructor -----
  //! @param devicename (IN) -- The serial port to the Giraff PC, as "/dev/ttyUSB1"
  //!  Comments:
  //!     The communication will be initiated when the program calls the open
  //!     method. There is a state "initiated" (communications are allowed)
  //!     or "not initiated" (communications are not allowed). 
  //!     At the beginning, it's "not initiated".
  RaposaSerial(const std::string& devicename);
  
  //! @brief Destructor
  virtual ~RaposaSerial() {}
  
  //! @brief Open: connects to Raposa at 115200 and 8N1. 
  //! @retval true if the Raposa has been connected successfully,
  //! @retval false if some error has occurred
  virtual bool open();
  
  //! \brief Waits for the response 
  //! @param response (OUT) The content of the response
  //! @param size Size of the response
  //! @retval true No errors were found
  //! @retval false Some error occurred
  bool getResponse(unsigned char *response, int size); 
  
  //! @brief Gets the state of the communication
  //! @retval true The communications have been initiated
  //! @retval false The communications have not been initiated
  bool isInitiated();

  private:
  bool initiated;
  
  
  static const uint32_t baud_rate;
  static const uint8_t open_mode;
};

//-- END OF HEADERS -----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ---------------------------------------------------------------------------

// --- General purpose functions ----------------------------------------------------------------

inline void _printTime() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  char day[4], mon[4];
  int wday, hh, mm, ss, year;
  sscanf(ctime((time_t*) &(ts.tv_sec)), "%s %s %d %d:%d:%d %d",day, mon, &wday, &hh, &mm, &ss, &year);
  std::cout << std::setw(2) << std::setfill(' ') << wday << ' ' << mon << ' ' << year << ' ' << std::setfill('0') << hh << ':' << mm << ':' << ss << '.' << ts.tv_nsec<< ' ';
}

/**********************************/
/** RaposaSerial implementation   */
/**********************************/

//! Communication parameters
const uint32_t RaposaSerial::baud_rate = B57600;
const uint8_t RaposaSerial::open_mode = O_RDWR | O_NOCTTY | O_NDELAY;

inline RaposaSerial::RaposaSerial(const std::string& devicename) :
SerialInterface(devicename, false),
initiated(false)
{}


inline bool RaposaSerial::open()
{
  return initiated = SerialInterface::open(baud_rate, open_mode);
}

inline bool RaposaSerial::getResponse(unsigned char *response, int size)
{
  bool ret_val = true;
  int bytes;
  
  if (!initiated) {
    last_error = "Communications not initiated";
    return false;
  }
  
  bytes = SerialInterface::read_stream(response, size);
  
  if (bytes == -1) {
    last_error = "Error found when waiting for response";
    return false;
  }
  
  ret_val = bytes == size;
    
  #ifdef _RAPOSA_SERIAL_DEBUG_ 
    if (bytes > 0) {
      _printTime(); 
      std::cout << "Response of Raposa (ints): ";
      for(int i = 0; i < size; i++) {
	std::cout << (int)response[i] << " ";
      }
      std::cout << std::endl;
    }
  #endif
  
  return ret_val;
}

inline bool RaposaSerial::isInitiated()
{
  return initiated;
}

//-- END OF INLINE FUNCTIONS ---------------------------------------
#endif
