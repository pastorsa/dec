/*
 * test_icsc_send_led.cpp
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include <ros/ros.h>
#include <string>

#include "DEC.h"
#include <dec_icsc/icsc_over_usb.h>

using namespace std;

const unsigned char DEC_NODE_ID = 0;

/*! Communication interface used to generate/parse messages
 */
DECInterface dec_interface;
dec_icsc::InterChipSerialCommunication icsc;

// Local variables
bool error = false;
bool msg_received = false;

class TestICSCDEC
{
public:
  TestICSCDEC() {};
  virtual ~TestICSCDEC() {};

  bool setup();
  void run();

private:

};

// This function is called whenever a broadcast was send
void receive(unsigned char source, char command, unsigned char length, char *data)
{
  ROS_INFO("Received >%i< bytes from >%u< from >%c<.", (int)length, source, command);
  if (command == DEC_SENSOR_DATA)
  {
    // parse data into local memory
    dec_interface.parseSensorData(source, data);

    // check whether it's for me
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {
      // continue the sensor data loop
      dec_interface.generateRequest(DEC_NODE_ID); // token = 0
      icsc.send(ICSC_BROADCAST, DEC_SENSOR_DATA, dec_interface.length_, dec_interface.data_);
    }
  }
  else if (command == DEC_LIGHT_DATA)
  {
    // parse data into local memory
    dec_interface.parseLightData(source, data);

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {
      // not implemented yet
    }
  }
  msg_received = true;
}

bool TestICSCDEC::setup()
{
  std::string device_name = "/dev/ttyUSB0";
  ROS_INFO("Trying to open device >%s<.", device_name.c_str());
  if(!icsc.begin(DEC_CONTROLLER_ID, const_cast<char*>(device_name.c_str())))
  {
    ROS_INFO("Problems when setting up device.");
    return false;
  }
  ROS_INFO("Setup complete. Opened >%s<.", device_name.c_str());

  icsc.registerCommand(DEC_SETUP_DATA, &receive);
  icsc.registerCommand(DEC_SENSOR_DATA, &receive);
  icsc.registerCommand(DEC_LIGHT_DATA, &receive);

  dec_interface.loadSetupData();
  if(dec_interface.generateSetupData(DEC_NODE_ID)) // token = 0
  {
    ROS_WARN("Sending setup broadcast of size >%u<.", dec_interface.length_);
    icsc.send(ICSC_BROADCAST, DEC_SETUP_DATA, dec_interface.length_, dec_interface.data_);
  }
  return true;
}

void TestICSCDEC::run()
{
  while (ros::ok())
  {
    icsc.process();

    ROS_INFO("ok");
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestICSCDEC");
  ros::NodeHandle node_handle;

  TestICSCDEC test_icsc_receive_led;
  if(!test_icsc_receive_led.setup())
  {
    ROS_ERROR("Test failed.");
    return -1;
  }

  test_icsc_receive_led.run();
  return 0;
}
