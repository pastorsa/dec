// This configuration file is generated from dec_visualization/config/config.yaml.
// Please do not edit.
// Instead edit dec_visualization/config/structure.yaml and regenerate this file.

#ifndef _DEC_CONFIG_H
#define _DEC_CONFIG_H

// Number of teensys in the structure.
#define DEC_NUMBER_OF_TEENSYS (uint8_t)1

// Maximum number of sensors, light strips, and LEDs per light strip. (To statically allocate memory)

#define DEC_MAX_NUMBER_OF_SENSORS_PER_NODE (uint8_t)26
#define DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE (uint8_t)9

#define DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY (uint8_t)90
#define DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY (uint8_t)90

#define DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP (uint8_t)150

static const uint8_t LIGHT_PIN_ORDERING[9] = {0, 1, 14, 15, 16, 24, 25, 26, 27};
static const uint8_t SENSOR_PIN_ORDERING[26] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22, 23, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

#endif // _DEC_CONFIG_H
