/*
 * dec_interface.cpp
 *
 *  Created on: Jul 11, 2013
 *      Author: pastor
 */

#include <ros/ros.h>

#include <dec_udp/dec_interface.h>

namespace dec_udp
{

DecInterface::DecInterface()
 : received_data_length_(0)
{
  udp_socket_.reset(new UDPSocket(SERVER_IP_ADDRESS, SERVER_PORT));
  ROS_INFO("Created socket at %s:%i", udp_socket_->getLocalAddress().c_str(), udp_socket_->getLocalPort());

  // initialize data
  reset(&_setup_data, _sensor_data, _light_data);

}

void DecInterface::print(const setup_data_t& setup_data)
{
  ROS_INFO("Setup Data:");
  ROS_INFO(" Number of LED strips is >%i<", (int)setup_data.num_led_strips);
  for (uint8_t i = 0; i < setup_data.num_led_strips; ++i)
    ROS_INFO("  Strip >%i< : #LEDs is >%i< at pin >%i<.", (int)i, (int)setup_data.led_strips[i].num_leds, (int)setup_data.led_strips[i].pin);

  ROS_INFO(" Number of sensors is >%i<", (int)setup_data.num_sensors);
  for (uint8_t i = 0; i < setup_data.num_sensors; ++i)
    ROS_INFO("  Sensor >%i< is at pin >%i<.", (int)i, (int)setup_data.sensors[i].pin);
}

void DecInterface::print(const sensor_data_t& sensor_data)
{
  ROS_INFO("Sensor Data:");
}

void DecInterface::print(const light_data_t& light_data)
{
  ROS_INFO("Light Data:");
}

bool DecInterface::sendSetupData(const uint8_t node_id)
{

  ROS_INFO("Loading setup data for node >%i<.", node_id);
  loadSetupData(node_id);

  print(_setup_data);

  ROS_INFO("Generating setup data for node >%i<.", node_id);
  if(!generateSetupData(node_id, _rx_buffer, &_length))
  {
    ROS_ERROR("Failed to generate setup data for node >%i<.", node_id);
    return false;
  }

  // std::string foreign_address = BASE_IP_ADDRESS + boost::lexical_cast<std::string>((int)node_id);
  std::string foreign_address = BROADCAST_IP_ADDRESS;

  ROS_INFO("Sending >%i< bytes of setup data for node id >%i< to >%s:%i<.", (int)_length, node_id, foreign_address.c_str(), (int)FOREIGN_PORT);

  // Send the string to the server
  if(udp_socket_->sendTo((void*)_rx_buffer, (int)_length, foreign_address, FOREIGN_PORT) != _length)
  {
    ROS_ERROR("Problems when sending to node id >%i< to >%s:%i<.", node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
    return false;
  }

  ROS_INFO("Waiting to receive answer.");
  received_data_length_ = udp_socket_->recv((void*)_rx_buffer, BUFFER_SIZE);

  ROS_INFO("Received >%i< bytes.", received_data_length_);

  parseSetupData(_rx_buffer);
  print(_setup_data);

  ROS_INFO("Done.");
  return true;
}

}
