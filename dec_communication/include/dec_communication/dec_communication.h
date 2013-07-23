/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#ifndef _DEC_COMMUNICATION_H
#define _DEC_COMMUNICATION_H

#include <stdlib.h>
#include <stdint.h>

// generated files
#include "DEC_config.h"
#include "DEC_structure.h"

// Fixed constants
// #define DEC_CONTROLLER_ID 50

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
Byte 0 is the message type. The message types can be DEC_SETUP_DATA, DEC_SENSOR_DATA, and DEC_LIGHT_DATA
The format corresponds to the structs setup_data_t, sensor_data_t, and light_data_t.
*/

// Message types:
// Message is broadcasted for setup
#define DEC_SETUP_DATA (uint8_t)1
// Message contains sensor information
#define DEC_SENSOR_DATA (uint8_t)2
// Message contains light information
#define DEC_LIGHT_DATA (uint8_t)3

// update this
#define BUFFER_SIZE (uint16_t)1600

/*check if the compiler is of C++*/
#ifdef __cplusplus
#ifndef __CDT_PARSER__
extern "C" {
#endif
#endif

/*!
 */
extern uint16_t _rx_buffer_length;
extern uint8_t _rx_buffer[BUFFER_SIZE];

extern uint8_t _light_buffer[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE + (uint8_t)1];

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SETUP_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
typedef struct
{
  uint8_t pin;
} sensor_setup_t;
typedef struct
{
  uint8_t index;
  uint8_t num_leds;
  uint8_t pin;
} led_strip_setup_t;
/*! Setup data.
 */
typedef struct
{
  uint8_t num_block_leds;
  led_strip_setup_t block_leds[DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY];
  uint8_t num_pixel_leds;
  led_strip_setup_t pixel_leds[DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY];
  uint8_t num_sensors;
  sensor_setup_t sensors[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE];
} setup_data_t;

/*! Data type to which the "data" gets parsed and which gets generated
 * by the functions below based on the
 */
extern setup_data_t _setup_data;

/*! Parse the received data into the _setup_data structure.
 * Note ! This function calls allocatePixelData(_setup_data) afterwards
 * @param data : Received data being parsed
 */
void parseSetupData(uint8_t* data);

/*! Prepares the provided buffer and _length member variables from the _setup_data structure.
 * Fill desired values into _setup_data before calling this function
 * @param buffer
 */
void generateSetupData(uint8_t* buffer);
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SETUP_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SENSOR_DATA >>>>>>>>>>>>>>>>>>>>>>>>> */
/*! Sensor data being broadcasted by the nodes and received by everyone
 */
typedef struct
{
  uint8_t sensor_value[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE];
} sensor_data_t;

/*! Data type to which the data gets parsed and which gets generated
 * by the functions below.
 */
extern sensor_data_t _sensor_data;

/*! Parse the received data into the the _sensor_data structure.
 * @param buffer  : Received data being parsed
 */
void parseSensorData(uint8_t* buffer);

/*! Prepares the provided buffer and _length member variables from the _sensor_data structure.
 * Fill desired values into _sensor_data before calling this function
 * @param buffer
 * @param sensor_data
 */
void generateSensorData(uint8_t* buffer, const sensor_data_t* sensor_data);

/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SENSOR_DATA <<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*! >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_LIGHT_DATA >>>>>>>>>>>>>>>>>>>>>>>>>> */

/*! Light data being broadcasted by the controller and received by each node
 */
typedef struct
{
  uint8_t red[DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP];
  uint8_t green[DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP];
  uint8_t blue[DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP];
  uint8_t brightness[DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP];
} led_block_data_t;

typedef struct
{
  // uint8_t red[DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP];
  // uint8_t green[DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP];
  // uint8_t blue[DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP];
  // uint8_t brightness[DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP];
//  uint8_t* red;
//  uint8_t* green;
//  uint8_t* blue;
//  uint8_t* brightness;
  uint8_t* red;
  uint8_t* green;
  uint8_t* blue;
  uint8_t* brightness;
} led_pixel_data_t;

typedef struct
{
  led_block_data_t block_leds[DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE];
  led_pixel_data_t pixel_leds[DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE];
  uint8_t pixel_memory_allocated;
} light_data_t;

/*! Data type to which the "data" gets parsed and which gets generated
 * by the functions below.
 */
extern light_data_t _light_data;

/*! Parse the received data into the the _light_data structure.
 * @param buffer  : Received data being parsed
 */
void parseLightData(uint8_t* buffer);

/*! Prepares provided buffer and _length member variables from the _light_data structure.
 * Fill desired values into _light_data before calling this function
 * @param buffer
 * @param light_data
 */
void generateLightData(uint8_t* buffer, const light_data_t* light_data);

/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_LIGHT_DATA <<<<<<<<<<<<<<<<<<<<<<<<<< */
/*! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/*!
 * @param node_id
 */
void loadSetupData(const uint8_t node_id);

/*!
 * @param buffer
 * @return 1 if buffer contains setup data
 */
uint8_t isSetupData(uint8_t* buffer);
/*!
 * @param buffer
 * @return 1 if buffer contains sensor data
 */
uint8_t isSensorData(uint8_t* buffer);
/*!
 * @param buffer
 * @return 1 if buffer contains light data
 */
uint8_t isLightData(uint8_t* buffer);

/*! Zeros all data
 */
void resetData(void);

/*! Zero sensor data
 */
void resetSetupData(setup_data_t* setup_data);
void resetSensorData(sensor_data_t* sensor_data);
void resetLightData(light_data_t* light_data);

/*!
 * @param setup_data
 */
void allocatePixelData(setup_data_t* setup_data, light_data_t* light_data);

/*! Prints internal data
 */
void printData(void);

/*check if the compiler is of C++ */
#ifdef __cplusplus
}
#endif

#endif // _DEC_COMMUNICATION_H
