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
    _setup_data.block_leds[i].num_blocks = (uint8_t)buffer[_rx_buffer_length];
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
    _setup_data.pixel_leds[i].num_pixels = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.pixel_leds[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  _setup_data.num_strips_used = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_strips_used; ++i)
  {
    _setup_data.strip_setup[i].total_num_leds_at_strip = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  _setup_data.num_sensors = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    _setup_data.sensors[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  allocatePixelData(&_setup_data, &_light_data);
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
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.block_leds[i].num_blocks;
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
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.pixel_leds[i].num_pixels;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.pixel_leds[i].pin;
    _rx_buffer_length++;
  }

  buffer[_rx_buffer_length] = _setup_data.num_strips_used;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_strips_used; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.strip_setup[i].total_num_leds_at_strip;
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
  _rx_buffer_length = (uint16_t)0;
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
  _rx_buffer_length = (uint16_t)0;
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
  _rx_buffer_length = (uint16_t)0;
  // skip message type
  _rx_buffer_length++;

  // first set light nodes
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
    _light_data.block_leds[i].red = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.block_leds[i].green = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.block_leds[i].blue = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.block_leds[i].brightness = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  // second set light beams
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
    for (j = 0; j < _setup_data.pixel_leds[i].num_pixels; ++j)
    {
      _light_data.pixel_leds[i].red[_setup_data.pixel_leds[i].index + j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].green[_setup_data.pixel_leds[i].index + j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].blue[_setup_data.pixel_leds[i].index + j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.pixel_leds[i].brightness[_setup_data.pixel_leds[i].index + j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
    }
  }
}

void generateLightData(uint8_t* buffer, const light_data_t* light_data)
{
  uint8_t i = 0;
  uint8_t j = 0;
  _rx_buffer_length = (uint16_t)0;
  buffer[_rx_buffer_length] = DEC_LIGHT_DATA;
  _rx_buffer_length++;

  // first set blocks
//  printf("Block Light: %u blocks.\n", _setup_data.num_block_leds);
  for (i = 0; i < _setup_data.num_block_leds; ++i)
  {
//    printf("  >%u< - >%u< >%u< >%u< >%u<\n", i,
//           (uint8_t)light_data->block_leds[i].red,
//           (uint8_t)light_data->block_leds[i].green,
//           (uint8_t)light_data->block_leds[i].blue,
//           (uint8_t)light_data->block_leds[i].brightness);

    buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].red;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].green;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].blue;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->block_leds[i].brightness;
    _rx_buffer_length++;
  }

  // second set pixels
//  printf("Pixel Light: %u pixels.\n", _setup_data.num_pixel_leds);
  for (i = 0; i < _setup_data.num_pixel_leds; ++i)
  {
//    printf("Pixel %i has index %i num_pixels %i and pin %i.\n", i, _setup_data.pixel_leds[i].index,
//           _setup_data.pixel_leds[i].num_pixels, _setup_data.pixel_leds[i].pin);

    for (j = 0; j < _setup_data.pixel_leds[i].num_pixels; ++j)
    {
      // uint8_t index = _setup_data.pixel_leds[i].index + j;
      uint8_t index = (uint8_t)0;
//      printf("   >%u< of >%u< num pixels\n", j, _setup_data.pixel_leds[i].num_pixels);
//      printf("   >%u< index >%u< >%u< >%u< >%u<\n", index, (uint8_t)light_data->pixel_leds[i].red[j],
//             (uint8_t)light_data->pixel_leds[i].green[j], (uint8_t)light_data->pixel_leds[i].blue[j],
//             (uint8_t)light_data->pixel_leds[i].brightness[j]);

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
  for (i = 0; i < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY; ++i)
  {
    setup_data->block_leds[i].index = (uint8_t)0;
    setup_data->block_leds[i].num_blocks = (uint8_t)0;
    setup_data->block_leds[i].pin = (uint8_t)0;
  }

  setup_data->num_pixel_leds = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY; ++i)
  {
    setup_data->pixel_leds[i].index = (uint8_t)0;
    setup_data->pixel_leds[i].num_pixels = (uint8_t)0;
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
  for (i = 0; i < DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY; ++i)
  {
    light_data->block_leds[i].red = (uint8_t)0;
    light_data->block_leds[i].blue = (uint8_t)0;
    light_data->block_leds[i].green = (uint8_t)0;
    light_data->block_leds[i].brightness = (uint8_t)0;
  }
  _light_data.pixel_memory_allocated = (uint8_t)0;

  //  for (i = 0; i < DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP; ++i)
  //  {
  //    for (j = 0; j < DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP; ++j)
  //    {
  //      light_data->pixel_leds[i].red[j] = (uint8_t)0;
  //      light_data->pixel_leds[i].blue[j] = (uint8_t)0;
  //      light_data->pixel_leds[i].green[j] = (uint8_t)0;
  //      light_data->pixel_leds[i].brightness[j] = (uint8_t)0;
  //    }
  //  }
}

void allocatePixelData(setup_data_t* setup_data, light_data_t* light_data)
{
  uint8_t i = 0;
  if (light_data->pixel_memory_allocated)
  {
    for (i = 0; i < setup_data->num_pixel_leds; ++i)
    {
      free(light_data->pixel_leds[i].red);
      free(light_data->pixel_leds[i].green);
      free(light_data->pixel_leds[i].blue);
      free(light_data->pixel_leds[i].brightness);
    }
  }
  // printf("Allocating %i bytes.\n", (int)4 * (int)sizeof(uint8_t)* DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
  for (i = 0; i < setup_data->num_pixel_leds; ++i)
  {
    //printf("0 %i.\n", (int)i);
    light_data->pixel_leds[i].red = (uint8_t*) malloc(sizeof(uint8_t) * DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    //printf("1 %i.\n", (int)i);
    light_data->pixel_leds[i].green = (uint8_t*) malloc(sizeof(uint8_t) * DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    //printf("2 %i.\n", (int)i);
    light_data->pixel_leds[i].blue = (uint8_t*) malloc(sizeof(uint8_t) * DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
    //printf("3 %i.\n", (int)i);
    light_data->pixel_leds[i].brightness = (uint8_t*) malloc(sizeof(uint8_t) * DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP);
  }
  //printf("Done.. allocating.\n");
  light_data->pixel_memory_allocated = (uint8_t)1;
}

void printData(void)
{
  uint16_t i = 0;
  printf("Data: %i bytes : ", (int)_rx_buffer_length);
  for (i = 0; i < _rx_buffer_length; ++i)
  {
    printf("%u ", _rx_buffer[i]);
  }
  printf("\n");
}
