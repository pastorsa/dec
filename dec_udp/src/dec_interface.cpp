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

DecInterface::DecInterface(const uint8_t num_sockets)
{
  for (uint8_t i = 0; i < num_sockets; ++i )
  {
    boost::shared_ptr<UDPSocket> udp_socket(new UDPSocket(SERVER_IP_ADDRESS, SERVER_PORT + i));
    assert(udp_socket->setNonBlocking());
    printf("Created socket at %s:%i\n", udp_socket->getLocalAddress().c_str(), udp_socket->getLocalPort());
    udp_sockets_.push_back(udp_socket);
  }

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
  if(udp_sockets_[node_id]->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, foreign_address, FOREIGN_PORT) != _rx_buffer_length)
  {
    printf("Problems when sending setup data to node id >%i< to >%s:%i<.\n", node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
    return false;
  }

  printf("Waiting to receive setup data answer.\n");
  _rx_buffer_length = 0;
  unsigned long int TIMEOUT_SETUP_IN_MICROSECONDS = 1000000;
  std::string source_address;
  unsigned int source_port;
  int return_code = udp_sockets_[node_id]->recvFromNonBlocking((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port, TIMEOUT_SETUP_IN_MICROSECONDS);
  if (return_code < 0)
  {
    printf("Missed setup packet from node with id >%i< : return code was >%i<.\n", node_id, return_code);
    return false;
  }

  if(source_address.compare(foreign_address) != 0)
  {
    printf("Missed sensor packet from node with id >%i<.\n", node_id);
    return false;
  }

  _rx_buffer_length = (uint16_t)return_code;
  // _rx_buffer_length = udp_sockets[node_id]_->recv((void*)_rx_buffer, BUFFER_SIZE);

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
  if(udp_sockets_[node_id]->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, foreign_address, FOREIGN_PORT) != _rx_buffer_length)
  {
    printf("Problems when sending light data to node id >%i< to >%s:%i<.\n", node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
    return false;
  }

  _rx_buffer_length = 0;
  unsigned long int TIMEOUT_LIGHT_IN_MICROSECONDS = 2000 + 100000;
  std::string source_address;
  unsigned int source_port;
  int return_code = udp_sockets_[node_id]->recvFromNonBlocking((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port, TIMEOUT_LIGHT_IN_MICROSECONDS);
  if (return_code < 0)
  {
    printf("Missed sensor packet from node with id >%i<.\n", node_id);
  }
  else
  {
    if(source_address.compare(foreign_address) == 0)
      _rx_buffer_length = (uint16_t)return_code;
    else
      printf("Missed sensor packet from node with id >%i<. Got package from >%s<.\n", node_id, source_address.c_str());
  }

  // printf("Waiting to receive light data answer.\n");
  // _rx_buffer_length = udp_socket_->recv((void*)_rx_buffer, BUFFER_SIZE);

  // printf("Done communicating light data.\n");
  return true;
}

}
