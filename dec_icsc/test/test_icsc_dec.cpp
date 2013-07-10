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

#include "DEC_structure.h"

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

  /*! Sets dec_interface.setup_data from the generated header files.
   * @param node_id The arduino id for which the setup data needs to be set
   */
  void loadSetupData(const int& node_id);

  bool setup();
  void run();

private:

};

// This function is called whenever a broadcast was send
void receive(unsigned char source, char command, unsigned char length, char *data)
{
  ROS_INFO("Received >%i< bytes from >%u< from >%c<.", (int)length, source, command);
  if (command == DEC_SETUP_DATA)
  {
    // parse data into local memory
    dec_interface.parseSetupData(data);

    // check whether it's for me
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {
      // continue the sensor data loop
      dec_interface.generateRequest(DEC_NODE_ID); // token = 0
      icsc.send(ICSC_BROADCAST, DEC_SENSOR_DATA, dec_interface.length_, dec_interface.data_);
    }
  }
  else if (command == DEC_SENSOR_DATA)
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
  else
  {
    ROS_ERROR("Invalid message.");
  }
  //  else if (command == DEC_LIGHT_DATA)
  //  {
  //    // parse data into local memory
  //    dec_interface.parseLightData(source, data);
  //
  //    // check whether it's my turn to broadcast my data
  //    if (dec_interface.token_ == DEC_CONTROLLER_ID)
  //    {
  //      // not implemented yet
  //    }
  //  }
  msg_received = true;
}

/*! Sets setup_data_t_ from the generated header files.
 * @param node_id which setup data need to be loaded ?
 */
void TestICSCDEC::loadSetupData(const int& node_id)
{
  uint8_t num_leds_index = 0;
  dec_interface.setup_data_.num_led_strips = NUM_LED_STRIPS_PER_ARDUINO[node_id];
  uint8_t io_pin_index = 0;
  for (uint8_t i = 0; i < dec_interface.setup_data_.num_led_strips; ++i)
  {
    dec_interface.setup_data_.led_strips[i].num_leds = NUM_LEDS_OF_EACH_LIGHT[num_leds_index];
    num_leds_index++;
    dec_interface.setup_data_.led_strips[i].pin = IO_PIN_ORDERING[io_pin_index];
    io_pin_index++;
  }

  dec_interface.setup_data_.num_sensors = NUM_SENSORS_PER_ARDUINO[node_id];
  for (uint8_t i = 0; i < dec_interface.setup_data_.num_sensors; ++i)
  {
    dec_interface.setup_data_.sensors[i].pin = IO_PIN_ORDERING[io_pin_index];
    io_pin_index++;
  }
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
  // icsc.registerCommand(DEC_LIGHT_DATA, &receive);

  loadSetupData(DEC_NODE_ID);
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

//    ROS_INFO("ok");
//    ros::Duration(1.0).sleep();
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
