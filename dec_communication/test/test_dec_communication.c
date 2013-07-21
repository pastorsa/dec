/*
 * test_dec_communication.c
 *
 *  Created on: Jul 11, 2013
 *      Author: pastor
 */

#include <stdlib.h>
#include <stdio.h>
#include <dec_communication/dec_communication.h>

void printData();

void printSetupData()
{
  printData();

//  printf("Setup Data:\n");
//  printf(" Number of LED nodes is >%u<\n", _setup_data.num_led_nodes);
//  for (uint8_t i = 0; i < _setup_data.num_led_nodes; ++i)
//    printf("  Node >%u< : #LEDs is >%u< at pin >%u<.\n", i, _setup_data.led_nodes[i].num_leds, _setup_data.led_nodes[i].pin);
//  printf(" Number of LED beams is >%u<\n", _setup_data.num_led_beams);
//  for (uint8_t i = 0; i < _setup_data.num_led_beams; ++i)
//    printf("  Beam >%u< : #LEDs is >%u< at pin >%u<.\n", i, _setup_data.led_beams[i].num_leds, _setup_data.led_beams[i].pin);

  printf(" Number of sensors is >%u<\n", _setup_data.num_sensors);
  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
    printf("  Sensor >%u< is at pin >%u<.\n", i, _setup_data.sensors[i].pin);
  printf("====================================================================\n");
}

void printData()
{
  printf("====================================================================\nData: %u bytes : ", _rx_buffer_length);
  for (uint8_t i = 0; i < _rx_buffer_length; ++i)
  {
    printf("%u ", _rx_buffer[i]);
  }
  printf("\n");
}

int main()
{
  resetData();
  printSetupData();

  uint8_t node_id = 1;
  loadSetupData(node_id);

  generateSetupData(_rx_buffer);
  printSetupData();

  resetData();
  printSetupData();

  parseSetupData(_rx_buffer);
  printSetupData();

  node_id = 0;
  loadSetupData(node_id);
  generateSetupData(_rx_buffer);

  parseSetupData(_rx_buffer);
  printSetupData();

  return 0;
}
