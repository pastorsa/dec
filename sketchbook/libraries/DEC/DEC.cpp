/*! This file implements the communication protocol for the dec structure
 *
 *  Created on: Jun 22, 2013
 *      Author: pastor
 */

#include "DEC.h"
#include <stdio.h>

#ifndef ARDUINO
#include <string.h>
using namespace std;
#endif

// Constructor of the class
DECInterface::DECInterface()
{
  // Allocate memory (do this separately to have things aligned in memory
  //  setup_data_ = (setup_data_t*)malloc(DEC_NUMBER_OF_ARDUINOS * sizeof(setup_data_t));
  //  sensor_data_ = (sensor_data_t*)malloc(DEC_NUMBER_OF_ARDUINOS * sizeof(sensor_data_t));
  //  light_data_ = (light_data_t*)malloc(DEC_NUMBER_OF_ARDUINOS * sizeof(light_data_t));
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    setup_data_[i] = (setup_data_t*)malloc(sizeof(setup_data_t));
  //  }
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    sensor_data_[i] = (sensor_data_t*)malloc(sizeof(sensor_data_t));
  //  }
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    light_data_[i] = (light_data_t*)malloc(sizeof(light_data_t));
  //  }

  if ((sizeof(setup_data_) + sizeof(sensor_data_) + sizeof(data_)) > 1500)
  {
    printf("Too much memory being used to store data >%i< bytes.\n", (int)(sizeof(setup_data_) + sizeof(sensor_data_) + sizeof(data_)));
    printf("Can't deal with that.\n");
  }

  printf("=================================================\n");
  printf("Using >%i< bytes for setup data.\n", (int)sizeof(setup_data_));
  printf("Using >%i< bytes for sensor data.\n", (int)sizeof(sensor_data_));
  printf("Using >%i< bytes for message data.\n", (int)(sizeof(data_)));
  printf("Using a total of >%i< bytes in SRAM.\n", (int)(sizeof(data_) + sizeof(setup_data_) + sizeof(sensor_data_)));
  printf("=================================================\n");

  // The controller is the boss
  token_ = DEC_CONTROLLER_ID;

  // Local member variables used for sending/receiving messages
  // data_ = (char*)malloc(MAX_MESSAGE * sizeof(char));
  length_ = 0;

  // Zero out everything
  reset();
}

// Destructor of the class
DECInterface::~DECInterface()
{
  // Free memory
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    free(setup_data_[i]);
  //  }
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    free(sensor_data_[i]);
  //  }
  //  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  //  {
  //    free(light_data_[i]);
  //  }
  //  free(setup_data_);
  //  free(sensor_data_);
  //  free(light_data_);
  //  free(data_);
}

void DECInterface::reset()
{
  // DEC_SETUP_DATA
  setup_data_.num_led_strips = 0;
  for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE; ++j)
  {
    setup_data_.led_strips[j].num_leds = 0;
    setup_data_.led_strips[j].pin = 0;
  }
  setup_data_.num_sensors = 0;
  for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++j)
  {
    setup_data_.sensors[j].pin = 0;
  }

  // DEC_SENSOR_DATA
  for(uint8_t i = 0; i < DEC_NUMBER_OF_ARDUINOS; ++i)
  {
    sensor_data_[i].level = 0;
    for (uint8_t j = 0; j < DEC_MAX_NUMBER_OF_SENSORS_PER_NODE; ++j)
    {
      sensor_data_[i].sensors[j] = 0;
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

// Parse this->data_ message into this->setup_data_ at index token taken from the data message
void DECInterface::parseSetupData(char* data)
{
  uint8_t byte_count = 0;
  token_ = data[byte_count];
  byte_count++;

  setup_data_.num_led_strips = data[byte_count];
  byte_count++;
  for (uint8_t i = 0; i < setup_data_.num_led_strips; ++i)
  {
    memcpy(&(setup_data_.led_strips[i]), &(data[byte_count]), sizeof(led_strip_setup_t));
    byte_count = byte_count + sizeof(led_strip_setup_t);
  }

  setup_data_.num_sensors = data[byte_count];
  byte_count++;
  for (uint8_t i = 0; i < setup_data_.num_sensors; ++i)
  {
    memcpy(&(setup_data_.sensors[i]), &(data[byte_count]), sizeof(sensor_setup_t));
    byte_count = byte_count + sizeof(sensor_setup_t);
  }

  // Note: length has not been used and is assumed to be of size >byte_count< by now
}

// Generate this->data_ message from this->setup_data_ at index >token< to be send to nodes
boolean DECInterface::generateSetupData(uint8_t token)
{
  // error checking
  if (setup_data_.num_led_strips > DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE)
    return false;
  for (uint8_t i = 0; i < setup_data_.num_led_strips; ++i)
    if (setup_data_.led_strips[i].num_leds > DEC_MAX_NUMBER_OF_LEDS_PER_LIGHT_STRIP)
      return false;
  if (setup_data_.num_sensors > DEC_MAX_NUMBER_OF_SENSORS_PER_NODE)
    return false;

  uint16_t byte_count = 0;
  data_[byte_count] = token;
  byte_count++;

  data_[byte_count] = setup_data_.num_led_strips;
  byte_count++;
  for (uint8_t i = 0; i < setup_data_.num_led_strips; ++i)
  {
    memcpy(&(data_[byte_count]), &(setup_data_.led_strips[i]), sizeof(led_strip_setup_t));
    byte_count = byte_count + sizeof(led_strip_setup_t);
  }

  data_[byte_count] = setup_data_.num_sensors;
  byte_count++;
  for (uint8_t i = 0; i < setup_data_.num_sensors; ++i)
  {
    memcpy(&(data_[byte_count]), &(setup_data_.sensors[i]), sizeof(sensor_setup_t));
    byte_count = byte_count + sizeof(sensor_setup_t);
  }

  length_ = byte_count;
  return true;
}
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< DEC_SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DEC_SENSOR_DATA >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Parse this->data_ message into this->sensor_data_ at index this->token_
void DECInterface::parseSensorData(uint8_t source, char* data)
{
  uint8_t byte_count = 0;
  token_ = data[byte_count];
  byte_count++;

  // copy the entire structure
  memcpy(&(sensor_data_[source]), &(data[byte_count]), sizeof(sensor_data_t));
  byte_count = byte_count + sizeof(sensor_data_t);

  // Note: length has not been used and is assumed to be of size >byte_count< by now
}

// Generate this->data_ message from this->sensor_data_ at index >token< to be send to nodes
void DECInterface::generateSensorData(uint8_t token)
{
  uint16_t byte_count = 0;
  data_[byte_count] = token;
  byte_count++;

  memcpy(&(data_[byte_count]), &(sensor_data_[token]), sizeof(sensor_data_t));
  byte_count = byte_count + sizeof(sensor_data_t);

  length_ = byte_count;
}

void DECInterface::generateRequest(uint8_t token)
{
  uint16_t byte_count = 0;
  data_[byte_count] = token;
  byte_count++;
  length_ = byte_count;
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
