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

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <assert.h>

#include <dec_icsc/icsc_over_usb.h>

namespace dec_icsc
{

InterChipSerialCommunication::InterChipSerialCommunication() : serial_(0), station_(0)
{
  printf("Initializing ICSC for >%i< message types.\n", MAX_COMMANDS);
  for (int i = 0; i < MAX_COMMANDS; ++i)
  {
    commands_[i].commandCode = 0;
    commands_[i].callback = NULL;
  }
};

bool InterChipSerialCommunication::begin(unsigned char station, unsigned int baud, char *device)
{
  struct termios tty;

  station_ = station;
  serial_ = open(device, O_RDWR | O_EXCL | O_NONBLOCK);
  if (!serial_)
  {
    printf("Could not open device >%s<.\n", device);
    errno = ENODEV;
    return false;
  }
  tcgetattr(serial_, &tty);

  cfsetspeed(&tty, baud);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tcsetattr(serial_, TCSANOW, &tty);
  return true;
}

void InterChipSerialCommunication::stop()
{
  if(serial_ > 0)
    close(serial_);
  else
  {
    printf("ICSC not initialized correctly. Not stopping.\n");
  }
}

unsigned char InterChipSerialCommunication::send(unsigned char station, unsigned char command, unsigned char len, char *data)
{
  assert(len < 255);

  unsigned char i;
  struct icsc_header header;
  struct icsc_footer footer;
#define NUM_SERIAL_START_BYTES 4
  char fill[NUM_SERIAL_START_BYTES] = {SOH, SOH, SOH, SOH};

  header.soh = SOH;
  header.dst = station;
  header.src = station_;
  header.command = command;
  header.len = len;
  header.stx = STX;

  footer.etx = ETX;
  footer.eot = EOT;
  footer.checksum = 0;
  footer.checksum += station;
  footer.checksum += station_;
  footer.checksum += command;
  footer.checksum += len;
  for (i = 0; i < len; i++)
  {
    footer.checksum += (unsigned char)data[i];
  }

  int num_bytes_written;
  unsigned char num_payload_bytes_written;
  num_bytes_written = write(serial_, &fill, NUM_SERIAL_START_BYTES);
  if(num_bytes_written != NUM_SERIAL_START_BYTES)
  {
    printf("Warning : only wrote %i bytes instead of %i when writing the filling bytes.\n", (int)num_bytes_written, (int)NUM_SERIAL_START_BYTES);
  }
  num_bytes_written = write(serial_, &header, sizeof(struct icsc_header));
  if(num_bytes_written != (int)sizeof(struct icsc_header))
  {
    printf("Warning : only wrote %i bytes instead of %i when writing the header.\n", (int)num_bytes_written, (int)sizeof(struct icsc_header));
  }
  num_bytes_written = write(serial_, data, len);
  if (num_bytes_written >= 0 && num_bytes_written < 255)
  {
    num_payload_bytes_written = num_bytes_written;
  }
  else
  {
    num_payload_bytes_written = 0;
  }
  if(num_payload_bytes_written != (int)len)
  {
    printf("Warning : only wrote %i bytes instead of %i when writing the payload.\n", (int)num_payload_bytes_written, (int)len);
  }
  num_bytes_written = write(serial_, &footer, sizeof(struct icsc_footer));
  if(num_bytes_written != (int)sizeof(struct icsc_footer))
  {
    printf("Warning : only wrote %i bytes instead of %i when writing the footer.\n", (int)num_bytes_written, (int)sizeof(struct icsc_footer));
  }

  fsync(serial_);

  return num_payload_bytes_written;
}

void InterChipSerialCommunication::process()
{
  static unsigned char phase = 0;
  static unsigned char len = 0;
  static unsigned char pos = 0;
  static unsigned char command = 0;
  static unsigned char sender = 0;
  static unsigned char destination = 0;
  static unsigned char header[6];
  static char data[256];
  static unsigned char calcCS = 0;
  int i;
  char inch;

  while (read(serial_, &inch, 1) == 1)
  {
    switch (phase)
    {
      case 0:
      {
        // printf("Phase %i.\n", phase);
        header[0] = header[1];
        header[1] = header[2];
        header[2] = header[3];
        header[3] = header[4];
        header[4] = header[5];
        header[5] = inch;
        if ((header[0] == SOH) && (header[5] == STX))
        {
          if (header[1] == station_)
          {
            destination = header[1];
            sender = header[2];
            command = header[3];
            len = header[4];
            phase = 1;
            pos = 0;
            len = 0;

            calcCS = 0;
            calcCS += destination;
            calcCS += sender;
            calcCS += (unsigned char)command;
            calcCS += len;
            if (len == 0)
            {
              phase = 2;
            }
          }
        }
        break;
      }
      case 1:
      {
        // printf("Phase %i.\n", phase);
        data[pos++] = inch;
        calcCS += (unsigned char)inch;
        if (pos == len)
        {
          phase = 2;
        }
        break;
      }
      case 2:
      {
        // printf("Phase %i.\n", phase);
        if (inch == ETX)
        {
          phase = 3;
        }
        else
        {
          pos = 0;
          len = 0;
          command = 0;
          sender = 0;
          destination = 0;
          len = 0;
          calcCS = 0;
          phase = 0;
        }
        break;
      }
      case 3:
      {
        // printf("Phase %i.\n", phase);
        if (calcCS != (unsigned char)inch)
        {
          pos = 0;
          len = 0;
          command = 0;
          sender = 0;
          destination = 0;
          len = 0;
          calcCS = 0;
          phase = 0;
        }
        else
        {
          phase = 4;
        }
        break;
      }
      case 4:
      {
        // printf("Phase %i.\n", phase);
        if ((unsigned char)inch != EOT)
        {
          pos = 0;
          len = 0;
          command = 0;
          sender = 0;
          destination = 0;
          len = 0;
          calcCS = 0;
          phase = 0;
        }
        else
        {
          // printf("Ready to call callbacks.\n");
          for (i = 0; i < MAX_COMMANDS; i++)
          {
            // printf("Checking to call callback >%i<.\n", i);
            if (commands_[i].commandCode == command)
            {
              // printf("Found >%i<.\n", i);
              if (commands_[i].callback)
              {
                // printf("Calling callback >%i<.\n", i);
                commands_[i].callback(sender, command, len, data);
              }
            }
          }
          pos = 0;
          len = 0;
          command = 0;
          sender = 0;
          destination = 0;
          len = 0;
          calcCS = 0;
          phase = 0;
          phase = 0;
        }
        break;
      }
    }
  }
  usleep(20);
}

void InterChipSerialCommunication::registerCommand(char command, callback_function function)
{
  for (int i = 0; i < MAX_COMMANDS; i++)
  {
    if (commands_[i].commandCode == 0)
    {
      printf("Registering callback >%i< to command >%u<.\n", i, command);
      commands_[i].commandCode = command;
      commands_[i].callback = function;
      return;
    }
  }
}

void InterChipSerialCommunication::unregisterCommand(char command)
{
  for (int i = 0; i < MAX_COMMANDS; i++)
  {
    if (commands_[i].commandCode == command)
    {
      commands_[i].commandCode = 0;
      commands_[i].callback = NULL;
    }
  }
}

}
