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

DecInterface::DecInterface(const uint8_t num_sockets, std::vector<uint8_t>& nets)
{
  assert((unsigned int)num_sockets == nets.size());

  for (uint8_t i = 0; i < num_sockets; ++i)
  {
    std::string base_ip = BASE + boost::lexical_cast<std::string>((int)nets[i]) + "." + boost::lexical_cast<std::string>((int)i + 1);
    printf("base_ip %s\n", base_ip.c_str());
    base_ip_addresses_.push_back(base_ip);
    std::string server_ip = BASE + boost::lexical_cast<std::string>((int)nets[i]) + "." + SERVER;
    server_ip_addresses_.push_back(server_ip);
    printf("server_ip %s\n", server_ip.c_str());

    boost::shared_ptr<UDPSocket> udp_socket(new UDPSocket(server_ip, SERVER_PORT + i));
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
    printf("  Block >%u< : Block LED at pin >%u< starts at index >%u< and has >%u< LEDs.\n", i,
           setup_data.block_leds[i].pin, setup_data.block_leds[i].index, setup_data.block_leds[i].num_blocks);
  printf(" Number of pixel LEDs is >%u<.\n", setup_data.num_pixel_leds);
  for (uint8_t i = 0; i < setup_data.num_pixel_leds; ++i)
    printf("  Beam >%u< : Pixel LED at pin >%u< starts at index >%u< and has >%u< LEDs.\n", i,
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

bool DecInterface::sendSetupData(const uint8_t node_id, const setup_data_t& setup_data)
{
  printf("Loading setup data for node >%i<.\n", node_id);
  _setup_data = setup_data;
  print(_setup_data);

  printf("Generating setup data for node >%i<.\n", node_id);
  generateSetupData(_rx_buffer);
  printData();

  printf("Sending >%i< bytes of setup data for node id >%i< to >%s:%i<.\n", (int)_rx_buffer_length, node_id, base_ip_addresses_[node_id].c_str(), (int)FOREIGN_PORT);
  if(udp_sockets_[node_id]->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, base_ip_addresses_[node_id], FOREIGN_PORT) != _rx_buffer_length)
  {
    printf("Problems when sending setup data to node id >%i< to >%s:%i<.\n", node_id, base_ip_addresses_[node_id].c_str(), (int)FOREIGN_PORT);
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

  if(source_address.compare(base_ip_addresses_[node_id]) != 0)
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

bool DecInterface::sendLightData(const std::vector<bool>& send_flags,
                                 const std::vector<light_data_t>& light_data,
                                 const std::vector<setup_data_t>& setup_data)
{
  assert(send_flags.size() == setup_data.size());
  assert(send_flags.size() == light_data.size());

  // send all
  for (unsigned int i = 0; i < send_flags.size(); ++i)
  {
    if (send_flags[i])
    {
      // printf("Generating light data for node >%i<.\n", node_id);
      generateLightData(_rx_buffer, &(light_data[i]));
      // printData();
      // printf("Generated >%u< bytes of light data.\n", _rx_buffer_length);
      // printf("Sending >%i< bytes of light data for node id >%i< to >%s:%i<.\n", (int)_rx_buffer_length, node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
      if(udp_sockets_[i]->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, base_ip_addresses_[i], FOREIGN_PORT) != _rx_buffer_length)
      {
        printf("Problems when sending light data to node id >%i< to >%s:%i<.\n", (int)i+1, base_ip_addresses_[i].c_str(), (int)FOREIGN_PORT);
        return false;
      }
    }
  }

  // receive all
  for (unsigned int i = 0; i < send_flags.size(); ++i)
  {
    if (send_flags[i])
    {
      _rx_buffer_length = 0;
      std::string source_address;
      unsigned int source_port;
      // printf("Waiting to receive sensor data answer for node >%i<.\n", (int)i+1);
      // unsigned long int TIMEOUT_LIGHT_IN_MICROSECONDS = 2000 + 100000;
      // int return_code = udp_sockets_[i]->recvFromNonBlocking((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port, TIMEOUT_LIGHT_IN_MICROSECONDS);
      int return_code = udp_sockets_[i]->recvFrom((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port);
      if (return_code < 0)
      {
        printf("Missed sensor packet from node with id >%i<.\n", (int)i+1);
        return false;
      }
      else
      {
        if(source_address.compare(base_ip_addresses_[i]) == 0)
        {
          // printf("Return code of node %i is %i.\n", node_id, return_code);
          _rx_buffer_length = (uint16_t)return_code;
          // print(setup_data);
          parseSensorData(_rx_buffer, &(setup_data[i]));
          received_sensor_data_[i] = _sensor_data;
          // printf("Received data for node >%i<.\n", (int)i+1);
          // printData();
        }
        else
        {
          printf("Missed sensor packet from node with id >%i<. Got package from >%s<.\n", (int)i+1, source_address.c_str());
          return false;
        }
      }
    }
  }
  // printf("Done communicating light data and receiving sensor data.\n");
  return true;
}

bool DecInterface::sendLightData(const int node_id, const light_data_t& light_data, const setup_data_t& setup_data)
{
  return false;
//  // printf("Generating light data for node >%i<.\n", node_id);
//  generateLightData(_rx_buffer, &light_data);
//  // printData();
//  // printf("Generated >%u< bytes of light data.\n", _rx_buffer_length);
//  // printf("Sending >%i< bytes of light data for node id >%i< to >%s:%i<.\n", (int)_rx_buffer_length, node_id, foreign_address.c_str(), (int)FOREIGN_PORT);
//  if(udp_sockets_[node_id]->sendTo((void*)_rx_buffer, (int)_rx_buffer_length, base_ip_addresses_[node_id], FOREIGN_PORT) != _rx_buffer_length)
//  {
//    printf("Problems when sending light data to node id >%i< to >%s:%i<.\n", node_id, base_ip_addresses_[node_id].c_str(), (int)FOREIGN_PORT);
//    return false;
//  }
//
//  _rx_buffer_length = 0;
//  unsigned long int TIMEOUT_LIGHT_IN_MICROSECONDS = 2000 + 100000;
//  std::string source_address;
//  unsigned int source_port;
//  // printf("Waiting to receive sensor data answer.\n");
//
//	// TODO: set recvFrom
//
//  int return_code = udp_sockets_[node_id]->recvFromNonBlocking((void*)_rx_buffer, BUFFER_SIZE, source_address, source_port, TIMEOUT_LIGHT_IN_MICROSECONDS);
//  if (return_code < 0)
//  {
//    printf("Missed sensor packet from node with id >%i<.\n", node_id);
//    return false;
//  }
//  else
//  {
//    if(source_address.compare(base_ip_addresses_[node_id]) == 0)
//    {
//      // printf("Return code of node %i is %i.\n", node_id, return_code);
//      _rx_buffer_length = (uint16_t)return_code;
//      // print(setup_data);
//      parseSensorData(_rx_buffer, &setup_data);
//      received_sensor_data_[node_id] = _sensor_data;
//      // printf("Received data for node >%i< \n", node_id);
//      // printData();
//    }
//    else
//    {
//      printf("Missed sensor packet from node with id >%i<. Got package from >%s<.\n", node_id, source_address.c_str());
//      return false;
//    }
//  }
//
//  // printf("Done communicating light data and receiving sensor data.\n");
//  return true;
}

}
