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
  // to store received sensor data
  received_sensor_data_.resize(num_sockets);
  for (unsigned int i = 0; i < received_sensor_data_.size(); ++i)
  {
    resetSensorData(&(received_sensor_data_[i]));
  }
  // initialize data
  resetData();
}

void DecInterface::print(const setup_data_t& setup_data)
{
  printf("Setup Data:\n");
  printf(" Number of block LEDs is >%u<.\n", setup_data.num_block_leds);
  for (uint8_t i = 0; i < setup_data.num_block_leds; ++i)
    printf("  Block >%u< : Block LED >%u< starts at index >%u< and has >%u< LEDs.\n", i,
           setup_data.block_leds[i].pin, setup_data.block_leds[i].index, setup_data.block_leds[i].num_blocks);
  printf(" Number of pixel LEDs is >%u<.\n", setup_data.num_pixel_leds);
  for (uint8_t i = 0; i < setup_data.num_pixel_leds; ++i)
    printf("  Beam >%u< : Pixel LED >%u< starts at index >%u< and has >%u< LEDs.\n", i,
           setup_data.pixel_leds[i].pin, setup_data.pixel_leds[i].index, setup_data.pixel_leds[i].num_pixels);
  printf(" Number of light strips used is >%u<.\n", setup_data.num_strips_used);
  for (uint8_t i = 0; i < setup_data.num_strips_used; ++i)
    printf("  Strip >%u< has >%u< LEDs.\n", i, setup_data.strip_setup[i].total_num_leds_at_strip);

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

//setup_data_t DecInterface::getSetupData(const uint8_t node_id)
//{
//  loadSetupData(node_id);
//  return _setup_data;
//}

bool DecInterface::sendSetupData(const uint8_t node_id, const setup_data_t& setup_data)
{
  printf("Loading setup data for node >%i<.\n", node_id);
  // loadSetupData(node_id);
  _setup_data = setup_data;
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
  printf("Generating light data for node >%i<.\n", node_id);
  // loadSetupData(node_id);
  generateLightData(_rx_buffer, &light_data);
  printData();

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
  // printf("Waiting to receive sensor data answer.\n");
  int return_code = udp_sockets_[node_id]->recvFromNonBlocking((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port, TIMEOUT_LIGHT_IN_MICROSECONDS);
  if (return_code < 0)
  {
    printf("Missed sensor packet from node with id >%i<.\n", node_id);
    return false;
  }
  else
  {
    if(source_address.compare(foreign_address) == 0)
    {
      _rx_buffer_length = (uint16_t)return_code;
      parseSensorData(_rx_buffer);
      received_sensor_data_[node_id] = _sensor_data;
      printf("Received data for node %i.\n", node_id);
      printData();
    }
    else
    {
      printf("Missed sensor packet from node with id >%i<. Got package from >%s<.\n", node_id, source_address.c_str());
      return false;
    }
  }

  // printf("Done communicating light data and receiving sensor data.\n");
  return true;
}

}
