/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "dec_communication.h"

uint16_t _rx_buffer_length;
uint8_t _rx_buffer[BUFFER_SIZE];
uint8_t _light_buffer[DEC_MAX_NUMBER_OF_SENSORS_PER_NODE + (uint8_t)1];

setup_data_t _setup_data;
sensor_data_t _sensor_data;
light_data_t _light_data;

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SETUP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void parseSetupData(uint8_t* buffer)
{
  uint8_t i = 0;
  _rx_buffer_length = (uint16_t)0;
  // skip message type
  _rx_buffer_length++;

  _setup_data.num_block_leds = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
    _setup_data.block_leds[i].index = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.block_leds[i].num_leds = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.block_leds[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  _setup_data.num_pixel_leds = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
    _setup_data.pixel_leds[i].index = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.pixel_leds[i].num_leds = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.pixel_leds[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  _setup_data.num_sensors = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    _setup_data.sensors[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }
}

void generateSetupData(uint8_t* buffer)
{
  uint8_t i = 0;
  _rx_buffer_length = (uint16_t)0;
  buffer[_rx_buffer_length] = DEC_SETUP_DATA;
  _rx_buffer_length++;

  buffer[_rx_buffer_length] = _setup_data.num_block_leds;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.block_leds[i].index;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.block_leds[i].num_leds;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.block_leds[i].pin;
    _rx_buffer_length++;
  }

  buffer[_rx_buffer_length] = _setup_data.num_pixel_leds;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.pixel_leds[i].index;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.pixel_leds[i].num_leds;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.pixel_leds[i].pin;
    _rx_buffer_length++;
  }

  buffer[_rx_buffer_length] = _setup_data.num_sensors;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.sensors[i].pin;
    _rx_buffer_length++;
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SENSOR_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void parseSensorData(uint8_t* buffer)
{
  uint8_t i = 0;
  _rx_buffer_length = 0;
  // skip message type
  _rx_buffer_length++;

  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    _sensor_data.sensor_value[i] = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }
}

void generateSensorData(uint8_t* buffer, const sensor_data_t* sensor_data)
{
  uint8_t i = 0;
  _rx_buffer_length = 0;
  buffer[_rx_buffer_length] = DEC_SENSOR_DATA;
  _rx_buffer_length++;

  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)sensor_data->sensor_value[i];
    _rx_buffer_length++;
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SENSOR_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_LIGHT_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void parseLightData(uint8_t* buffer)
{
  uint8_t i = 0;
  uint8_t j = 0;
  _rx_buffer_length = (uint8_t)0;
  // skip message type
  _rx_buffer_length++;

  // first set light nodes
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
    for (j = 0; j < _setup_data.block_leds[i].num_leds; ++j)
    {
      _light_data.block_leds[i].red[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.block_leds[i].green[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.block_leds[i].blue[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.block_leds[i].brightness[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
    }
  }

  // second set light beams
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
    for (j = 0; j < _setup_data.pixel_leds[i].num_leds; ++j)
    {
      _light_data.pixel_leds[i].red[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].green[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].blue[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].brightness[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
    }
  }
}

void generateLightData(uint8_t* buffer, const light_data_t* light_data)
{
  uint8_t i = 0;
  uint8_t j = 0;
  _rx_buffer_length = 0;
  buffer[_rx_buffer_length] = DEC_LIGHT_DATA;
  _rx_buffer_length++;

  // first set blocks
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
    for (j = 0; j < _setup_data.block_leds[i].num_leds; ++j)
    {
      buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].red[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].green[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].blue[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].brightness[j];
      _rx_buffer_length++;
    }
  }

  // second set pixels
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
    for (j = 0; j < _setup_data.pixel_leds[i].num_leds; ++j)
    {
      buffer[_rx_buffer_length] = (uint8_t)light_data->pixel_leds[i].red[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->pixel_leds[i].green[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->pixel_leds[i].blue[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->pixel_leds[i].brightness[j];
      _rx_buffer_length++;
    }
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_LIGHT_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void loadSetupData(const uint8_t node_id)
{
//  uint8_t i = 0;
//  _setup_data.num_block_leds = NUM_BLOCK_LEDS_PER_TEENSY[node_id];
//  for (i = 0; i < _setup_data.num_block_leds; ++i)
//  {
//    _setup_data.block_leds[i].start_index = BLOCK_LEDS_START_INDEX[i];
//    _setup_data.block_leds[i].end_index = BLOCK_LEDS_END_INDEX[i];
//    _setup_data.block_leds[i].num_leds = _setup_data.block_leds[i].end_index - _setup_data.block_leds[i].start_index;
//    _setup_data.block_leds[i].pin = BLOCK_LED_PINS[i];
//  }
//
//  _setup_data.num_pixel_leds = NUM_PIXEL_LEDS_PER_TEENSY[node_id];
//  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
//  {
//    _setup_data.pixel_leds[i].start_index = PIXEL_LEDS_START_INDEX[i];
//    _setup_data.pixel_leds[i].end_index = PIXEL_LEDS_END_INDEX[i];
//    _setup_data.pixel_leds[i].num_leds = _setup_data.pixel_leds[i].end_index - _setup_data.pixel_leds[i].start_index;
//    _setup_data.pixel_leds[i].pin = PIXEL_LED_PINS[i];
//  }

//  _setup_data.num_sensors = NUM_SENSORS_PER_TEENSY[node_id];
//  for (i = 0; i < _setup_data.num_sensors; ++i)
//  {
//    _setup_data.sensors[i].pin = SENSOR_PIN_ORDERING[i];
//  }
}

uint8_t isSetupData(uint8_t* buffer)
{
  if (DEC_SETUP_DATA == (uint8_t)buffer[0])
    return 1;
  return 0;
}
uint8_t isSensorData(uint8_t* buffer)
{
  if (DEC_SENSOR_DATA == (uint8_t)buffer[0])
    return 1;
  return 0;
}
uint8_t isLightData(uint8_t* buffer)
{
  if (DEC_LIGHT_DATA == (uint8_t)buffer[0])
    return 1;
  return 0;
}

void resetData(void)
{
  resetSetupData(&_setup_data);
  resetSensorData(&_sensor_data);
  resetLightData(&_light_data);
}

void resetSetupData(setup_data_t* setup_data)
{
  uint8_t i = 0;
  setup_data->num_block_leds = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    setup_data->block_leds[i].index = (uint16_t)0;
    setup_data->block_leds[i].num_leds = (uint16_t)0;
    setup_data->block_leds[i].pin = (uint8_t)0;
  }

  setup_data->num_pixel_leds = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    setup_data->pixel_leds[i].index = (uint16_t)0;
    setup_data->pixel_leds[i].num_leds = (uint16_t)0;
    setup_data->pixel_leds[i].pin = (uint8_t)0;
  }

  setup_data->num_sensors = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++i)
  {
    setup_data->sensors[i].pin = (uint8_t)0;
  }
}

void resetSensorData(sensor_data_t* sensor_data)
{
  uint8_t i = 0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++i)
  {
    sensor_data->sensor_value[i] = (uint8_t)0;
  }
}

void resetLightData(light_data_t* light_data)
{
  uint8_t i = 0;
  uint8_t j = 0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    for (j = 0; j < DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP; ++j)
    {
      light_data->block_leds[i].red[j] = (uint8_t)0;
      light_data->block_leds[i].blue[j] = (uint8_t)0;
      light_data->block_leds[i].green[j] = (uint8_t)0;
      light_data->block_leds[i].brightness[j] = (uint8_t)0;
    }
  }

  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    for (j = 0; j < DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP; ++j)
    {
      light_data->pixel_leds[i].red[j] = (uint8_t)0;
      light_data->pixel_leds[i].blue[j] = (uint8_t)0;
      light_data->pixel_leds[i].green[j] = (uint8_t)0;
      light_data->pixel_leds[i].brightness[j] = (uint8_t)0;
    }
  }
}

void printData(void)
{
  uint8_t i = 0;
  printf("Data: %u bytes : ", _rx_buffer_length);
  for (i = 0; i < _rx_buffer_length; ++i)
  {
    printf("%u ", _rx_buffer[i]);
  }
  printf("\n");
}
