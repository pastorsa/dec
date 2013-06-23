/*
 * ICSC_config.h
 *
 *  Created on: Jun 23, 2013
 *      Author: pastor
 */

#ifndef ICSC_CONFIG_H_
#define ICSC_CONFIG_H_

// TODO: play with this
#define DEC_BAUD_RATE 115200

// Uncomment the definition of ICSC_DYNAMIC if you want to use
// dynamic memory allocation
//#define ICSC_DYNAMIC

// Uncomment the definition of ICSC_NO_STATS if you don't need stats
// and need to save space
#define ICSC_NO_STATS

// Uncomment the ICSC_COLLISION_DETECTION if collision detection is needed
// On Arduino the transmit and receive are isolated and therefore not needed
// #define ICSC_COLLISION_DETECTION

#ifndef ICSC_DYNAMIC
// This defines the maximum size of any message that can be sent
// between chips.  Reduce this to conserve memory at the cost
// of smaller packet sizes.  Note that this only affects the
// size of packets that can be received - you can always send
// up to 255 bytes.  If the remote end can't receive them all
// the packet will be silently discarded.
#define MAX_MESSAGE 50

// The maximum number of registered commands.  If you're not
// going to have many commands reducing this can save memory.
// If you want lots of commands you may need to increase this
// value.  Note that this only affects the commands registerable
// at the receiving end
// TODO: keep this in mind and revisit
#define MAX_COMMANDS 10
#endif

// If this StationId is used during send(broadcast)
// the message will be picked up by all devices
#define ICSC_BROADCAST  0xFD

#define ICSC_CMD_SYS    0x1F

// Packet wrapping characters, defined in standard ASCII table
#define SOH 1
#define STX 2
#define ETX 3
#define EOT 4

#endif /* ICSC_CONFIG_H_ */
