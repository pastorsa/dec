// This configuration file is generated from dec_visualization/config/structure.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_STRUCTURE_H
#define _DEC_STRUCTURE_H

static const uint8_t IO_PIN_ORDERING[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

static const uint8_t NUM_LED_STRIPS_PER_ARDUINO[10] = {2, 2, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t NUM_LEDS_OF_EACH_LIGHT[4] = {4, 3, 2, 5};
static const uint8_t NUM_SENSORS_PER_ARDUINO[10] = {1, 1, 0, 0, 0, 0, 0, 0, 0, 0};

static const uint8_t LIGHT_BEAM_CONNECTIONS[2] = {0, 1};
static const uint8_t LIGHT_NODE_CONNECTIONS[2] = {0, 1};
static const uint8_t SENSOR_CONNECTIONS[2] = {0, 1};
static const uint8_t LIGHT_BEAM_INDEX_COUNTER[2] = {0, 0};
static const uint8_t LIGHT_NODE_INDEX_COUNTER[2] = {0, 0};
static const uint8_t SENSOR_INDEX_COUNTER[2] = {0, 0};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_BEAM_MAP[10][10] = {{0, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {1, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_NODE_MAP[10][10] = {{0, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {1, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_SENSOR_MAP[10][10] = {{0, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {1, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}, {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}};

#endif // _DEC_STRUCTURE_H
