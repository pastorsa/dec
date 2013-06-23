/********************************************************************************
 * Copyright (c) 2013, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Majenko Technologies.
 ********************************************************************************/

#ifndef _INTER_CHIP_SERIAL_COMMUNICATION_H
#define _INTER_CHIP_SERIAL_COMMUNICATION_H

#include <unistd.h>
#include <termios.h>

// include config
#include <ICSC_config.h>

namespace dec_icsc
{

/*! @brief This class is used to communicate over the RS-485 bus over USB
 */
class InterChipSerialCommunication
{
  // Format of command callback functions
  typedef void (*callback_function)(unsigned char, char, unsigned char, char *);

public:
  InterChipSerialCommunication();
  virtual ~InterChipSerialCommunication() {};

  /*! Initialize the communication
   * @param station the id of this station
   * @param baud baudrate. For example B115200
   * @param device name of the device. For example /dev/ttyUSB0
   * @return True on success, otherwise False
   */
  bool begin(unsigned char station, unsigned int baud, char *device);
  void stop();

  unsigned char send(unsigned char station, unsigned char command, unsigned char len, char *data);
  void process();
  void register_command(char command, callback_function function);
  void unregister_command(char command);

private:

  // Structure to store command code / function pairs
  typedef struct
  {
    char commandCode;
    callback_function callback;
  } command;

  struct icsc_header
  {
    unsigned char soh;
    unsigned char dst;
    unsigned char src;
    char command;
    unsigned char len;
    unsigned char stx;
  };

  struct icsc_footer
  {
    unsigned char etx;
    unsigned char checksum;
    unsigned char eot;
  };

  int serial_;
  unsigned char station_;
  command commands_[MAX_COMMANDS];
};

}

#endif // _INTER_CHIP_SERIAL_COMMUNICATION_H
