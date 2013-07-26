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

  printf("Setup Data:\n");
  printf(" Number of block LEDs is >%u<.\n", _setup_data.num_block_leds);
  for (uint8_t i = 0; i < _setup_data.num_block_leds; ++i)
    printf("  Node >%u< : Block LED at pin >%u< starts at index >%u< and has >%u< blocks.\n", i,
           _setup_data.block_leds[i].pin, _setup_data.block_leds[i].index, _setup_data.block_leds[i].num_blocks);
  printf(" Number of pixel LEDs is >%u<.\n", _setup_data.num_pixel_leds);
  for (uint8_t i = 0; i < _setup_data.num_pixel_leds; ++i)
    printf("  Beam >%u< : Pixel LED at pin >%u< starts at index >%u< and has >%u< pixels.\n", i,
           _setup_data.pixel_leds[i].pin, _setup_data.pixel_leds[i].index, _setup_data.pixel_leds[i].num_pixels);
  printf(" Number of sensors is >%u<.\n", _setup_data.num_sensors);
  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
    printf("  Sensor >%u< is at pin >%u<.\n", i, _setup_data.sensors[i].pin);
  printf("====================================================================\n");
}

void printData()
{
  printf("====================================================================\nData: %u bytes : ", (int)_rx_buffer_length);
  for (int i = 0; i < (int)_rx_buffer_length; ++i)
  {
    printf("%i ", (int)_rx_buffer[i]);
  }
  printf("\n");
}

int main()
{
	return 0;

  resetData();
  // printSetupData();

  uint8_t node_id = 0;
	// loadSetupData(node_id);

  for (unsigned int i = 0; i < 100; ++i)
  {

    generateSetupData(_rx_buffer);
    printSetupData();

    // resetData();
    // printSetupData();

    parseSetupData(_rx_buffer);
    printSetupData();

  }

//  node_id = 1;
//  loadSetupData(node_id);
//  generateSetupData(_rx_buffer);
//
//  parseSetupData(_rx_buffer);
//  printSetupData();

  return 0;
}
