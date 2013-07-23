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

#define DEC_MAX_NUMBER_OF_BLOCKS_PER_TEENSY (uint8_t)40
#define DEC_MAX_NUMBER_OF_PIXELS_PER_TEENSY (uint8_t)255

#define DEC_MAX_NUMBER_OF_BLOCKS_PER_LIGHT_STRIP (uint8_t)20
#define DEC_MAX_NUMBER_OF_PIXELS_PER_LIGHT_STRIP (uint8_t)150

#endif // _DEC_CONFIG_H
