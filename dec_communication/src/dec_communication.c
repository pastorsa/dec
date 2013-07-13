/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// File generated from dec_visualization/config/config.yaml
#include "dec_communication.h"
// #include <dec_communication/dec_communication.h>

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SETUP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void parseSetupData(uint8_t* buffer)
{
  uint8_t i = 0;
  _rx_buffer_length = (uint16_t)0;
  // skip message type
  _rx_buffer_length++;

  _setup_data.num_led_nodes = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_led_nodes; ++i)
  {
    _setup_data.led_nodes[i].num_leds = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.led_nodes[i].pin = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  _setup_data.num_led_beams = (uint8_t)buffer[_rx_buffer_length];
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_led_beams; ++i)
  {
    _setup_data.led_beams[i].num_leds = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _setup_data.led_beams[i].pin = (uint8_t)buffer[_rx_buffer_length];
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

  buffer[_rx_buffer_length] = _setup_data.num_led_nodes;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_led_nodes; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.led_nodes[i].num_leds;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.led_nodes[i].pin;
    _rx_buffer_length++;
  }

  buffer[_rx_buffer_length] = _setup_data.num_led_beams;
  _rx_buffer_length++;
  for (i = 0; i < _setup_data.num_led_beams; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.led_beams[i].num_leds;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)_setup_data.led_beams[i].pin;
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
  for (i = 0; i < _setup_data.num_led_nodes; ++i)
  {
    _light_data.led_nodes[i].red = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.led_nodes[i].green = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.led_nodes[i].blue = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
    _light_data.led_nodes[i].brightness = (uint8_t)buffer[_rx_buffer_length];
    _rx_buffer_length++;
  }

  // second set light beams
  for (i = 0; i < _setup_data.num_led_beams; ++i)
  {
    for (j = 0; j < _setup_data.led_beams[i].num_leds; ++j)
    {
      _light_data.led_beams[i].red[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.led_beams[i].green[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.led_beams[i].blue[j] = (uint8_t)buffer[_rx_buffer_length];
      _rx_buffer_length++;
      _light_data.led_beams[i].brightness[j] = (uint8_t)buffer[_rx_buffer_length];
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

  // first set light nodes
  for (i = 0; i < _setup_data.num_led_nodes; ++i)
  {
    buffer[_rx_buffer_length] = (uint8_t)light_data->led_nodes[i].red;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->led_nodes[i].green;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->led_nodes[i].blue;
    _rx_buffer_length++;
    buffer[_rx_buffer_length] = (uint8_t)light_data->led_nodes[i].brightness;
    _rx_buffer_length++;
  }

  // second set light beams
  for (i = 0; i < _setup_data.num_led_beams; ++i)
  {
    for (j = 0; j < _setup_data.led_beams[i].num_leds; ++j)
    {
      buffer[_rx_buffer_length] = (uint8_t)light_data->led_beams[i].red[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->led_beams[i].green[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->led_beams[i].blue[j];
      _rx_buffer_length++;
      buffer[_rx_buffer_length] = (uint8_t)light_data->led_beams[i].brightness[j];
      _rx_buffer_length++;
    }
  }
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_LIGHT_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void loadSetupData(const uint8_t node_id)
{
  uint8_t i = 0;
  uint8_t index = 0;

  _setup_data.num_led_nodes = NUM_LED_NODES_PER_ARDUINO[node_id];
  for (i = 0; i < _setup_data.num_led_nodes; ++i)
  {
    _setup_data.led_nodes[i].num_leds = NUM_LEDS_OF_EACH_LIGHT[i];
    _setup_data.led_nodes[i].pin = LIGHT_PIN_ORDERING[i];
  }

  index = _setup_data.num_led_nodes;
  _setup_data.num_led_beams = NUM_LED_NODES_PER_ARDUINO[node_id];
  for (i = 0; i < _setup_data.num_led_beams; ++i)
  {
    _setup_data.led_beams[i].num_leds = NUM_LEDS_OF_EACH_LIGHT[index];
    _setup_data.led_beams[i].pin = LIGHT_PIN_ORDERING[index];
    index++;
  }

  _setup_data.num_sensors = NUM_SENSORS_PER_ARDUINO[node_id];
  for (i = 0; i < _setup_data.num_sensors; ++i)
  {
    _setup_data.sensors[i].pin = SENSOR_PIN_ORDERING[i];
  }
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
  uint8_t i = 0;
  uint8_t j = 0;
  // DEC_SETUP_DATA
  _setup_data.num_led_nodes = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    _setup_data.led_nodes[i].num_leds = (uint16_t)0;
    _setup_data.led_nodes[i].pin = (uint8_t)0;
  }

  _setup_data.num_led_beams = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    _setup_data.led_beams[i].num_leds = (uint16_t)0;
    _setup_data.led_beams[i].pin = (uint8_t)0;
  }

  _setup_data.num_sensors = (uint8_t)0;
  for (i = 0; i < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++i)
  {
    _setup_data.sensors[i].pin = (uint8_t)0;
  }

  // DEC_SENSOR_DATA
  for (i = 0; i < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++i)
  {
    _sensor_data.sensor_value[i] = (uint8_t)0;
  }

  // DEC_LIGHT_DATA
  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    _light_data.led_nodes[i].red = (uint8_t)0;
    _light_data.led_nodes[i].blue = (uint8_t)0;
    _light_data.led_nodes[i].green = (uint8_t)0;
    _light_data.led_nodes[i].brightness = (uint8_t)0;
  }

  for (i = 0; i < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++i)
  {
    for (j = 0; j < DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP; ++j)
    {
      _light_data.led_beams[i].red[j] = (uint8_t)0;
      _light_data.led_beams[i].blue[j] = (uint8_t)0;
      _light_data.led_beams[i].green[j] = (uint8_t)0;
      _light_data.led_beams[i].brightness[j] = (uint8_t)0;
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
