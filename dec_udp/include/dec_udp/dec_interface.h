/*
 * dec_interface.h
 *
 *  Created on: Jul 11, 2013
 *      Author: pastor
 */

#ifndef DEC_INTERFACE_H_
#define DEC_INTERFACE_H_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <dec_communication/dec_communication.h>

// local includes
#include <dec_udp/PracticalSocket.h>

namespace dec_udp
{

static const std::string BASE_IP_ADDRESS = "10.0.0.";
static const std::string SERVER_IP_ADDRESS = BASE_IP_ADDRESS + "100";
static const std::string BROADCAST_IP_ADDRESS = BASE_IP_ADDRESS + "255";
static const unsigned int SERVER_PORT = 1501;
static const unsigned int FOREIGN_PORT = 1500;

class DecInterface
{

public:

  DecInterface(const uint8_t num_sockets);
  virtual ~DecInterface() {};

  /*!
   * @param setup_data
   */
  void print(const setup_data_t& setup_data);
  /*!
   * @param setup_data
   */
  void print(const sensor_data_t& light_data);
  /*!
   * @param setup_data
   */
  void print(const light_data_t& sensor_data);

  /*!
   * @param node_id
   * @return True on success, otherwise False
   */
  bool sendSetupData(const uint8_t node_id, const setup_data_t& setup_data);

  /*!
   * @param node_id
   * @param light_data
   * @return True on success, otherwise False
   */
  bool sendLightData(const int node_id, const light_data_t& light_data, const setup_data_t& setup_data);

  /*!
   */
  std::vector<sensor_data_t> received_sensor_data_;

private:

  std::vector<boost::shared_ptr<UDPSocket> > udp_sockets_;
};

}

#endif /* DEC_INTERFACE_H_ */
