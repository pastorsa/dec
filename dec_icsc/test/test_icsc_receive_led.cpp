/*
 * test_icsc_send_led.cpp
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include <ros/ros.h>
#include <string>

#include <dec_icsc/icsc_over_usb.h>

using namespace std;

unsigned char remote_station_id = 2;
const unsigned char my_station_id = 50;

class TestICSCReceiveLED
{
public:
  TestICSCReceiveLED() {};
  virtual ~TestICSCReceiveLED() {};

  bool setup();
  void run();

private:
  dec_icsc::InterChipSerialCommunication icsc_;

};

bool TestICSCReceiveLED::setup()
{
  std::string device_name = "/dev/ttyUSB0";
  ROS_INFO("Trying to open device >%s<.", device_name.c_str());
  if(!icsc_.begin(my_station_id, B115200, const_cast<char*>(device_name.c_str())))
  {
    ROS_INFO("Problems when setting up device.");
    return false;
  }
  ROS_INFO("Setup complete. Opened >%s<.", device_name.c_str());
  return true;
}


void receive(unsigned char source, char command, unsigned char length, char *data)
{
  ROS_WARN("Received >%i< bytes from >%u< from >%c<.", (int)length, source, command);
}

void TestICSCReceiveLED::run()
{
  icsc_.register_command('P', &receive);
  icsc_.register_command('R', &receive);
  while (ros::ok())
  {
    icsc_.process();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestICSCReceiveLED");
  ros::NodeHandle node_handle;

  TestICSCReceiveLED test_icsc_receive_led;
  if(!test_icsc_receive_led.setup())
  {
    ROS_ERROR("Test failed.");
    return -1;
  }

  test_icsc_receive_led.run();
  return 0;
}
