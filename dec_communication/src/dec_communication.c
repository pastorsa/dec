/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// File generated from dec_visualization/config/config.yaml
#include <dec_communication/dec_communication.h>

void loadSetupData(const uint8_t node_id)
{
  uint8_t num_leds_index = 0;
  _setup_data.num_led_strips = NUM_LED_STRIPS_PER_ARDUINO[node_id];
  uint8_t io_pin_index = 0;
  for (uint8_t i = 0; i < _setup_data.num_led_strips; ++i)
  {
    _setup_data.led_strips[i].num_leds = NUM_LEDS_OF_EACH_LIGHT[num_leds_index];
    num_leds_index++;
    _setup_data.led_strips[i].pin = IO_PIN_ORDERING[io_pin_index];
    io_pin_index++;
  }

  _setup_data.num_sensors = NUM_SENSORS_PER_ARDUINO[node_id];
  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
  {
    _setup_data.sensors[i].pin = IO_PIN_ORDERING[io_pin_index];
    io_pin_index++;
  }
}

void setToken(uint8_t token, uint8_t* buffer)
{
 buffer[0] = token;
}
/*! Update token in this->data_ to be DEC_CONTROLLER_ID
*/
void setTokenToController(uint8_t* buffer)
{
 setToken(DEC_CONTROLLER_ID, buffer);
}

//void dec_init(void)
//{
////  if ((sizeof(_setup_data) + sizeof(_sensor_data_) + sizeof(_rx_buffer)) > 1500)
////  {
////    printf("Too much memory being used to store data >%i< bytes.\n", (int)(sizeof(_setup_data) + sizeof(sensor_data_) + sizeof(data_)));
////    printf("Can't deal with that.\n");
////  }
//
////  printf("=================================================\n");
////  printf("Using >%i< bytes for setup data.\n", (int)sizeof(_setup_data));
////  printf("Using >%i< bytes for sensor data.\n", (int)sizeof(sensor_data_));
////  printf("Using >%i< bytes for message data.\n", (int)(sizeof(data_)));
////  printf("Using a total of >%i< bytes in SRAM.\n", (int)(sizeof(data_) + sizeof(_setup_data) + sizeof(sensor_data_)));
////  printf("=================================================\n");
//
//  // The controller is the boss
//  _token = DEC_CONTROLLER_ID;
//
//  // Local member variables used for sending/receiving messages
//  // data_ = (char*)malloc(MAX_MESSAGE * sizeof(char));
//  // length_ = 0;
//
//  // Zero out everything
//  reset(&_setup_data, _sensor_data, _light_data);
//}

void reset(setup_data_t* setup_data, sensor_data_t* sensor_data, light_data_t* light_data)
{
  // DEC_SETUP_DATA
  setup_data->num_led_strips = 0;
  for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++j)
  {
    setup_data->led_strips[j].num_leds = 0;
    setup_data->led_strips[j].pin = 0;
  }
  setup_data->num_sensors = 0;
  for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++j)
  {
    setup_data->sensors[j].pin = 0;
  }

  // DEC_SENSOR_DATA
  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  {
    for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++j)
    {
      sensor_data[i].sensors[j] = 0;
    }
  }

  //  // DEC_LIGHT_DATA
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++j)
  //    {
  //      for (uint8_t k = 0; k < DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP; ++k)
  //      {
  //        light_data_[i].led_strips[j].colors[k] = 0;
  //      }
  //    }
  //  }
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SETUP >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void parseSetupData(uint8_t* data)
{
  uint16_t byte_count = 0;
  _token = data[byte_count];
  byte_count++;

  _setup_data.num_led_strips = data[byte_count];
  byte_count++;
  for (uint8_t i = 0; i < _setup_data.num_led_strips; ++i)
  {
    memcpy(&(_setup_data.led_strips[i]), &(data[byte_count]), sizeof(led_strip_setup_t));
    byte_count = byte_count + sizeof(led_strip_setup_t);
  }

  _setup_data.num_sensors = data[byte_count];
  byte_count++;
  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
  {
    memcpy(&(_setup_data.sensors[i]), &(data[byte_count]), sizeof(sensor_setup_t));
    byte_count = byte_count + sizeof(sensor_setup_t);
  }

  // Note: length has not been used and is assumed to be of size >byte_count< by now
}

// Generate this->data_ message from this->_setup_data at index >token< to be send to nodes
uint8_t generateSetupData(uint8_t token, uint8_t* buffer, uint16_t* length)
{
  // error checking
  if (_setup_data.num_led_strips > DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE)
    return 0;
  for (uint8_t i = 0; i < _setup_data.num_led_strips; ++i)
    if (_setup_data.led_strips[i].num_leds > DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP)
      return 0;
  if (_setup_data.num_sensors > DEC_MAX_NUMBER_OF_SENSORS_PER_NODE)
    return 0;

  uint16_t byte_count = 0;
  buffer[byte_count] = token;
  byte_count++;

  buffer[byte_count] = _setup_data.num_led_strips;
  byte_count++;
  for (uint8_t i = 0; i < _setup_data.num_led_strips; ++i)
  {
    memcpy(&(buffer[byte_count]), &(_setup_data.led_strips[i]), sizeof(led_strip_setup_t));
    byte_count = byte_count + sizeof(led_strip_setup_t);
  }

  buffer[byte_count] = _setup_data.num_sensors;
  byte_count++;
  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
  {
    memcpy(&(buffer[byte_count]), &(_setup_data.sensors[i]), sizeof(sensor_setup_t));
    byte_count = byte_count + sizeof(sensor_setup_t);
  }

  *length = byte_count;
  return 1;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SENSOR_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Parse this->data_ message into this->sensor_data_ at index this->token_
void parseSensorData(uint8_t source, char* data)
{
  uint8_t byte_count = 0;
  _token = data[byte_count];
  byte_count++;

  // copy the entire structure
  memcpy(&(_sensor_data[source]), &(data[byte_count]), sizeof(sensor_data_t));
  byte_count = byte_count + sizeof(sensor_data_t);

  // Note: length has not been used and is assumed to be of size >byte_count< by now
}

// Generate this->data_ message from this->sensor_data_ at index >token< to be send to nodes
void generateSensorData(uint8_t token, uint8_t* buffer, uint16_t* length)
{
  uint16_t byte_count = 0;
  buffer[byte_count] = token;
  byte_count++;

  memcpy(&(buffer[byte_count]), &(_sensor_data[token]), sizeof(sensor_data_t));
  byte_count = byte_count + sizeof(sensor_data_t);

  *length = byte_count;
}

void generateRequest(uint8_t token, uint8_t* buffer, uint16_t* length)
{
  uint16_t byte_count = 0;
  buffer[byte_count] = token;
  byte_count++;
  *length = byte_count;
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SENSOR_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_LIGHT_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//
// // Parse this->data_ message into this->light_data_ at index this->token_
// void DECInterface::parseLightData(uint8_t source, char* data)
// {
//  uint8_t byte_count = 0;
//  token_ = data[byte_count];
//  byte_count++;
//
//  // copy the entire structure
//  memcpy(&(light_data_[source]), &(data[byte_count]), sizeof(light_data_t));
//  byte_count = byte_count + sizeof(light_data_t);
//
//  // Note: length has not been used and is assumed to be of size >byte_count< by now
// }
//
// // Generate this->data_ message from this->light_data_ at index >token< to be send to nodes
// void DECInterface::generateLightData(uint8_t token)
// {
//  uint8_t byte_count = 0;
//  data_[byte_count] = token;
//  byte_count++;
//
//  // copy the entire structure
//  memcpy(&(data_[byte_count]), &(light_data_[token]), sizeof(light_data_t));
//  byte_count = byte_count + sizeof(light_data_t);
//
//  length_ = byte_count;
// }

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_LIGHT_DATA <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
