// This configuration file is generated from dec_visualization/config/structure.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_STRUCTURE_H
#define _DEC_STRUCTURE_H

static const uint8_t LIGHT_PIN_ORDERING[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27};
static const uint8_t SENSOR_PIN_ORDERING[10] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 17};

static const uint8_t NUM_SENSORS_PER_TEENSY[1] = {3};

static const uint8_t NUM_BLOCK_LEDS_PER_TEENSY[1] = {11};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t BLOCK_LEDS_START_INDEX[1][10] = {{0, 21, 43, 0, 21, 0, 21, 43, 0, 10}};
static const uint8_t NUM_PIXEL_LEDS_PER_TEENSY[1] = {6};


#endif // _DEC_STRUCTURE_H
