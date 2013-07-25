// This configuration file is generated from dec_visualization/config/structure.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_STRUCTURE_H
#define _DEC_STRUCTURE_H

static const uint8_t LIGHT_PIN_ORDERING[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27};
static const uint8_t SENSOR_PIN_ORDERING[26] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22, 23, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

static const uint8_t NUM_SENSORS_PER_TEENSY[1] = {1};

static const uint8_t NUM_BLOCK_LEDS_PER_TEENSY[1] = {3};
static const uint8_t BLOCK_LEDS_START_INDEX[][20] = {{0, 5, 10}};
static const uint8_t BLOCK_LEDS_NUMBER[][20] = {{5, 5, 5}};
static const uint8_t BLOCK_LEDS_PINS[][20] = {{0, 0, 0}};

static const uint8_t NUM_PIXEL_LEDS_PER_TEENSY[1] = {1};
static const uint8_t PIXEL_LEDS_START_INDEX[][150] = {{0}};
static const uint8_t PIXEL_LEDS_NUMBER[][150] = {{1}};
static const uint8_t PIXEL_LEDS_PINS[][150] = {{1}};

static const uint8_t NUM_STRIPS_USED[1] = {2};
static const uint8_t TOTAL_NUM_LEDS_AT_STRIP[][9] = {{15, 1, 0, 0, 0, 0, 0, 0, 0}};

#endif // _DEC_STRUCTURE_H
