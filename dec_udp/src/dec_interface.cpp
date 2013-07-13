/*
 * dec_interface.cpp
 *
 *  Created on: Jul 11, 2013
 *      Author: pastor
 */

#include <stdio.h>
#include <dec_udp/dec_interface.h>

#include <boost/lexical_cast.hpp>

namespace dec_udp
{

DecInterface::DecInterface()
{
  udp_socket_.reset(new UDPSocket(SERVER_IP_ADDRESS, SERVER_PORT));
  // printf("Created socket at %s:%i\n", udp_socket_->getLocalAddress().c_str(), udp_socket_->getLocalPort());

  // initialize data
  resetData();
}

void DecInterface::print(const setup_data_t& setup_data)
{
  printf("Setup Data:\n");
  printf(" Number of LED nodes is >%u<.\n", _setup_data.num_led_nodes);
  for (uint8_t i = 0; i < _setup_data.num_led_nodes; ++i)
    printf("  Node >%u< : #LEDs is >%u< at pin >%u<.\n", i, _setup_data.led_nodes[i].num_leds, _setup_data.led_nodes[i].pin);
  printf(" Number of LED beams is >%u<.\n", _setup_data.num_led_beams);
  for (uint8_t i = 0; i < _setup_data.num_led_beams; ++i)
    printf("  Beam >%u< : #LEDs is >%u< at pin >%u<.\n", i, _setup_data.led_beams[i].num_leds, _setup_data.led_beams[i].pin);
  printf(" Number of sensors is >%u<.\n", setup_data.num_sensors);
  for (uint8_t i = 0; i < setup_data.num_sensors; ++i)
    printf("  Sensor >%u< is at pin >%u<.\n", i, setup_data.sensors[i].pin);
}

void DecInterface::print(const sensor_data_t& sensor_data)
{
  printf("Sensor Data:\n");
}

void DecInterface::print(const light_data_t& light_data)
{
  printf("Light Data:\n");
}

bool DecInterface::sendSetupData(const uint8_t node_id)
{
  printf("Loading setup data for node >%i<.\n", node_id);
  loadSetupData(node_id);
  print(_setup_data);

  printf("Generating setup data for node >%i<.\n", node_id);
  generateSetupData(_rx_buffer);
  printData();

  const int IP_FROM_NODE = node_id + 1;
  std::string foreign_address = BASE_IP_ADDRESS + boost::lexical_cast<std::string>((int)IP_FROM_NODE);

  printf("Sending >%i< bytes of setup data for node id >%i< to >%s:%i<.\n", (int)_rx_buffer_length, node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
  if(udp_socket_->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, foreign_address, FOREIGN_PORT) != _rx_buffer_length)
  {
    printf("Problems when sending setup data to node id >%i< to >%s:%i<.\n", node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
    return false;
  }

  printf("Waiting to receive setup data answer.\n");
  _rx_buffer_length = udp_socket_->recv((void*)_rx_buffer, BUFFER_SIZE);

  printData();

  parseSetupData(_rx_buffer);
  print(_setup_data);

  printf("Done with setup.\n");
  return true;
}

bool DecInterface::sendLightData(const int node_id, const light_data_t& light_data)
{
  loadSetupData(node_id);

  // printf("Generating light data for node >%i<.\n", node_id);
  generateLightData(_rx_buffer, &light_data);
  // printf("Generated >%u< bytes of light data.\n", _rx_buffer_length);

  const int IP_FROM_NODE = node_id + 1;
  std::string foreign_address = BASE_IP_ADDRESS + boost::lexical_cast<std::string>((int)IP_FROM_NODE);

  // printf("Sending >%i< bytes of light data for node id >%i< to >%s:%i<.\n", (int)_rx_buffer_length, node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
  if(udp_socket_->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, foreign_address, FOREIGN_PORT) != _rx_buffer_length)
  {
    printf("Problems when sending light data to node id >%i< to >%s:%i<.\n", node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
    return false;
  }

  // printf("Waiting to receive light data answer.\n");
  _rx_buffer_length = udp_socket_->recv((void*)_rx_buffer, BUFFER_SIZE);

  // printf("Done communicating light data.\n");
  return true;
}

}
