/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#ifndef _DEC_COMMUNICATION_H
#define _DEC_COMMUNICATION_H

#include <stdlib.h>
#include <stdint.h>

#include <dec_communication/DEC_config.h>
#include <dec_communication/DEC_structure.h>

// Fixed constants
#define DEC_CONTROLLER_ID 50
// Note station id 0xFD == 253 is used for broadcasting

/* Remember:
 * uint8_t     = [0..255]
 * uint16_t    = [0..65535]
 * uint32_t    = [0..4billion]
 *
 * int8_t      = [-127..128]
 * int16_t     = [-32768..32767]
 * int32_t     = [-2billion..2billion]
 *
 * On ATMEGA boards
 * float is the same as double
 * float is a 32bit real number
 */

/*
Message Definition
Byte 0 is always the id of the arduino with the NEXT token.
if the token matches with DEC_NODE_ID or DEC_CONTROLLER_ID
then we can send a broadcast

The message types can be DEC_SETUP_DATA, DEC_SENSOR_DATA, and DEC_LIGHT_DATA
The format corresponds to the structs setup_data_t, sensor_data_t, and light_data_t.
*/

// Message types:
// Message is broadcasted for setup
#define DEC_SETUP_DATA 0x01
// Message contains sensor information
#define DEC_SENSOR_DATA 0x02
// Message contains light information
// #define DEC_LIGHT_DATA 0x03

#define BUFFER_SIZE (uint16_t)900

/*check if the compiler is of C++*/
#ifdef __cplusplus
#ifndef __CDT_PARSER__
extern "C" {
#endif
#endif

/*!
 */
uint8_t _token;
uint16_t _length;
uint8_t _rx_buffer[BUFFER_SIZE];

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SETUP_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
typedef struct
{
  uint8_t pin;
} sensor_setup_t;
typedef struct
{
  uint16_t num_leds;
  uint8_t pin;
} led_strip_setup_t;
/*! Setup data.
 */
typedef struct
{
  uint8_t num_led_strips;
  led_strip_setup_t led_strips[DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE];
  uint8_t num_sensors;
  sensor_setup_t sensors[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE];
} setup_data_t;

/*! Data type to which the "data" gets parsed and which gets generated
 * by the functions below based on the
 */
setup_data_t _setup_data;

/*! Parse the received data into the this->setup_data_ structure.
 * The index into this->setup_data_ is obtained from the first byte in
 * the received this->data_;
 * @param data : Received data being parsed into this->setup_data_
 */
void parseSetupData(uint8_t* data);

/*! Prepares the data_ and length_ member variables from the setup_data_ structure.
 * Fill desired values into setup_data_ at index "token" before calling this function
 * @param token   : Token of the arduino that is receiving the setup data
 *                  Note: this is also the "index" into the setup_data_ that
 *                  is being used to generate/fill the "data" message
 * @return 1 on success, otherwise 0 (due to wrong input)
 */
uint8_t generateSetupData(uint8_t token, uint8_t* buffer, uint16_t* length);
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SETUP_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SENSOR_DATA >>>>>>>>>>>>>>>>>>>>>>>>> */
/*! Sensor data being broadcasted by the nodes and received by everyone
 */
typedef struct
{
  uint8_t sensors[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE];
} sensor_data_t;

/*! Data type to which the data gets parsed and which gets generated
 * by the functions below.
 */
sensor_data_t _sensor_data[DEC_NUMBER_OF_ARDUINOS];

/*! Parse the received "data" into the the sensor_data_ structure.
 * @param source  : ID of the node that send this message used at
 *                  index into this->sensor_data_
 * @param data    : Received data being parsed into this->sensor_data_
 */
void parseSensorData(uint8_t source, char* data);

/*! Prepares the data_ and length_ member variables from the sensor_data_ structure.
 * Fill desired values into sensor_data_ at index "token" before calling this function
 * @param token   : Token of the node that is receiving the sensor data
 *                  Note: this is also the "index" into the sensor_data_ that
 *                  is being used to generate/fill the "data" message
 */
void generateSensorData(uint8_t token, uint8_t* buffer, uint16_t* length);

/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SENSOR_DATA <<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_LIGHT_DATA >>>>>>>>>>>>>>>>>>>>>>>>>> */

/*! Light data being broadcasted by the controller and received by each node
 */
typedef struct
{
  uint32_t colors[DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP];
} led_data_t;

typedef struct
{
  led_data_t led_strips[DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE];
} light_data_t;

/*! Data type to which the "data" gets parsed and which gets generated
 * by the functions below.
 */
light_data_t _light_data[DEC_NUMBER_OF_ARDUINOS];

/*! Parse the received data into the the light_data_ structure.
 * @param source  : ID of the node that send this message used at
 *                  index into this->light_data_
 * @param data    : Received data being parsed into this->light_data_
 */
void parseLightData(uint8_t source, char* data);

 /*! Prepares the data_ and length_ member variables from the light_data_ structure.
  * Fill desired values into sensor_data_ at index "token" before calling this function
  * @param token   : Token of the node that is receiving the light data
  *                  Note: this is also the "index" into the light_data_ that
  *                  is being used to generate/fill the "data" message
  */
void generateLightData(uint8_t token);

/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_LIGHT_DATA <<<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*! Prepares an empty message to be send out by the master
* @param token : Token of the >next< node
*/
void generateRequest(uint8_t token, uint8_t* buffer, uint16_t* length);

/*!
 * @param node_id
 */
void loadSetupData(const uint8_t node_id);

/*! Update token in this->data_
* @param token : Token for the next station
*/
void setToken(uint8_t token, uint8_t* buffer);
/*! Update token in this->data_ to be DEC_CONTROLLER_ID
*/
void setTokenToController(uint8_t* buffer);

/*!
 */
// void dec_init(void);

/*!
 * @param setup_data
 * @param sensor_data
 * @param light_data
 */
void reset(setup_data_t* setup_data, sensor_data_t* sensor_data, light_data_t* light_data);

/*check if the compiler is of C++ */
#ifdef __cplusplus
}
#endif


#endif // _DEC_COMMUNICATION_H
