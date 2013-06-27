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

const unsigned char DEC_CONTROLLER_ID = 254;

class TestICSCSendLED
{
public:
  TestICSCSendLED() {};
  virtual ~TestICSCSendLED() {};

  bool setup();
  void run();

private:
  dec_icsc::InterChipSerialCommunication icsc_;

};

bool TestICSCSendLED::setup()
{
  std::string device_name = "/dev/ttyUSB0";
  ROS_INFO("Trying to open device >%s<.", device_name.c_str());
  if(!icsc_.begin(DEC_CONTROLLER_ID, const_cast<char*>(device_name.c_str())))
  {
    ROS_INFO("Problems when setting up device.");
    return false;
  }
  ROS_INFO("Setup complete. Opened >%s<.", device_name.c_str());
  return true;
}

void TestICSCSendLED::run()
{
  ros::Time now = ros::Time::now();
  ros::Time previous = now;
  bool flag = false;

  ROS_INFO("Start loop.");
  while (ros::ok())
  {
    now = ros::Time::now();
    if (now > previous + ros::Duration(0.5))
    {
      if (flag)
      {
        ROS_INFO("Sending P.");
        icsc_.send(ICSC_BROADCAST, 'P', 0, NULL);
        // icsc_.send(remote_station_id, 'P', 0, NULL);
      }
      else
      {
        ROS_INFO("Sending R.");
        icsc_.send(ICSC_BROADCAST, 'R', 0, NULL);
        // icsc_.send(remote_station_id, 'R', 0, NULL);
      }
      flag = !flag;
      previous = now;
    }
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestICSCSendLED");
  ros::NodeHandle node_handle;

  TestICSCSendLED test_icsc_send_led;
  if(!test_icsc_send_led.setup())
  {
    ROS_ERROR("Test failed.");
    return -1;
  }

  test_icsc_send_led.run();
  return 0;
}
