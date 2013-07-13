// This configuration file is generated from dec_visualization/config/structure.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_STRUCTURE_H
#define _DEC_STRUCTURE_H

static const uint8_t LIGHT_PIN_ORDERING[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27};
static const uint8_t SENSOR_PIN_ORDERING[3] = {5, 6, 7};

static const uint8_t NUM_LED_NODES_PER_ARDUINO[1] = {1};
static const uint8_t NUM_LED_BEAMS_PER_ARDUINO[1] = {1};
static const uint8_t NUM_LEDS_OF_EACH_LIGHT[2] = {14, 15};
static const uint8_t NUM_SENSORS_PER_ARDUINO[1] = {2};

static const uint8_t LIGHT_BEAM_CONNECTIONS[1] = {0};
static const uint8_t LIGHT_NODE_CONNECTIONS[1] = {0};
static const uint8_t SENSOR_CONNECTIONS[2] = {0, 0};
static const uint8_t LIGHT_BEAM_INDEX_COUNTER[1] = {0};
static const uint8_t LIGHT_NODE_INDEX_COUNTER[1] = {0};
static const uint8_t SENSOR_INDEX_COUNTER[2] = {0, 1};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_BEAM_MAP[1][1] = {{0}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_LIGHT_NODE_MAP[1][1] = {{0}};
// A value of 255 is assigned to invalidate the entry.
static const uint8_t ARDUINO_TO_SENSOR_MAP[1][1] = {{0}};

#endif // _DEC_STRUCTURE_H
