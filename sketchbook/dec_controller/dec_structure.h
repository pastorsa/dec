// This configuration file is generated from dec_visualization/config/structure.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_STRUCTURE_H
#define _DEC_STRUCTURE_H

static const uint8_t CONTROLLER_ICSC_DE_PIN = 7;
static const uint8_t NODE_ICSC_DE_PINS[5] = {7, 7, 8, 8, 8};
static const uint8_t IO_PIN_ORDERING[17] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

static const uint8_t NUM_LED_STRIPS_PER_ARDUINO[5] = {54, 1, 1, 0, 0};
static const uint8_t NUM_LEDS_OF_EACH_LIGHT[56] = {1, 2, 3, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static const uint8_t NUM_SENSORS_PER_ARDUINO[5] = {1, 0, 0, 0, 0};

static const uint8_t LIGHT_BEAM_CONNECTIONS[4] = {0, 2, 1, 0};
static const uint8_t LIGHT_NODE_CONNECTIONS[52] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t SENSOR_CONNECTIONS[1] = {0};
static const uint8_t LIGHT_BEAM_INDEX_COUNTER[4] = {0, 0, 0, 1};
static const uint8_t LIGHT_NODE_INDEX_COUNTER[52] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};
static const uint8_t SENSOR_INDEX_COUNTER[1] = {0};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_BEAM_MAP[5][5] = {{0, 0, 255, 255, 255}, {1, 255, 255, 255, 255}, {2, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_NODE_MAP[5][5] = {{0, 0, 0, 0, 0}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_SENSOR_MAP[5][5] = {{0, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}, {255, 255, 255, 255, 255}};

#endif // _DEC_STRUCTURE_H
