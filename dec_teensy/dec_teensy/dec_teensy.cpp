/*
 Copyright (C) 2012 RedoX <dev@redox.ws>
 This file is part of TeenC.

 TeenC is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 TeenC is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with TeenC.  If not, see <http://www.gnu.org/licenses/>.

 Taken from http://svn.redox.ws/listing.php?repname=TeenC
 */

// for memcpy
#include <string.h>

// ENC28J60 related includes
#include "common.h"
#include "spi.h"
#include "enc28j60.h"
#include "ustack.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

// for input/output
#include "Arduino.h"
#include <avr/io.h>

// for the LEDs
#include "Adafruit_NeoPixel.h"

// DEC includes
#include "dec_communication.h"
//uint16_t _rx_buffer_length;
//#define BUFFER_SIZE (uint16_t)1600
//uint8_t _rx_buffer[BUFFER_SIZE];

// =======================================================================================
// NOTE: Change both, NODE_ID and NODE_ID_HEX !!
#define NODE_ID 1
#define NODE_ID_HEX 0x01
// =======================================================================================
// =======================================================================================

static const uint8_t IPAddr[4] = {10, 0, 0, NODE_ID};
static uint8_t MACAddr[6] = {0x0, 0x0, 0x0, 0x0, 0x0, NODE_ID_HEX};
static const uint8_t node_id = (uint8_t)NODE_ID - 1;

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define ADC_Prescaler (_BV(ADPS2) | _BV(ADPS1))
#define ADC_Mux 0

#define TLed PORTD ^= _BV(6)

// Neopixel
#define NEO_PIXEL_TYPE (NEO_GRB + NEO_KHZ800)
Adafruit_NeoPixel light_strips[DEC_MAX_NUMBER_OF_LED_STRIPS_PER_NODE];

/*!
 */
void setupPins(const setup_data_t* setup_data)
{
  // Serial.println("test");
  for (uint8_t i = 0; i < setup_data->num_strips_used; ++i)
  {
    light_strips[i].setup((uint16_t)setup_data->strip_setup[i].total_num_leds_at_strip,
                                    (uint8_t)LIGHT_PIN_ORDERING[i], NEO_PIXEL_TYPE);
    // Initialize all pixels to 'off'
    for (uint16_t j = 0; j < light_strips[i].numPixels(); ++j)
    {
      light_strips[i].setPixelColor(j, (uint8_t)255, (uint8_t)0, (uint8_t)0);
    }
  }

  for (uint8_t i = 0; i < setup_data->num_strips_used; ++i)
  {
    light_strips[i].begin();
    light_strips[i].show(); // Initialize all pixels to 'off'
  }

  for (uint8_t i = 0; i < setup_data->num_sensors; ++i)
  {
    pinMode(setup_data->sensors[i].pin, INPUT);
  }
}

//int led_state = LOW;             // ledState used to set the LED
//const uint8_t led_pin = 6; // Teensy2.0 uses pin 11

//void blink(void)
//{
//  if (led_state == LOW)
//    led_state = HIGH;
//  else
//    led_state = LOW;
//  digitalWrite(led_pin, led_state);
//}

uint16_t packet_length;
uint16_t protocol_type;
uint8_t *buffer_ptr;
uint8_t ip_header_length;
uint8_t aux_init;
uint8_t light_buffer[1 + sizeof(sensor_data_t)];

uint8_t is_setup;

void setup()
{
  // Serial.begin(9600);

  // init
  resetData();
  // dec_init();

  CPU_PRESCALE(0); /* Run at 16 MHz */
  DDRD = _BV(DDD6); /* Teensy onboard Led Enable */
  /* ADC */
  DDRF = ~_BV(DDF0); /* F0 as Input */
  //PORTF = _BV(PORTF0);
  /* Enable Pullup */
  DIDR0 = _BV(ADC0D); /* Disable Digital Pin */

  // initialize the digital pin as an output to blink the LED.
  // pinMode(led_pin, OUTPUT);

  dbg_init
  ; /* If Debug State On, Init USB */
  SPI_Init(SPI_DP2, SPI_PRESC_1); /* AVR SPI Init */
  do
  {
    ENC_Init(MACAddr); /* ENC28J60 Init */
    _delay_ms(500);
    aux_init = ENC_RevID;
  } while (!aux_init);

  ENC_LEDInit; /* ENC Leds Init */
  is_setup = 0;
}

void loop()
{
  // TLed;
  // _delay_ms(300);
  // dbg_c(ENC_LinkSt?'~':'_');
  if (ENC_LinkSt)
  {
    if (ENC_hasRxd)
    {
      packet_length = ENC_PckRx(_rx_buffer, BUFFER_SIZE);
      if (packet_length > 0)
      {
        // Return 1 if packet was ARP, and was handeld
        if (!ifARP_Reply(_rx_buffer, packet_length))
        {
          // Return IPHLen if valid IP Packet, else 0
          ip_header_length = IP_Check(_rx_buffer, packet_length);
          if (ip_header_length > 0)
          {
            protocol_type = IP_gProtocol(_rx_buffer);
            if (protocol_type == IP_pICMP)
            {
              // dbg_s("ICMP\n");
              ICMP_Reply(_rx_buffer, ip_header_length, packet_length);
            }
            else if (protocol_type == IP_pUDP)
            {
             // Serial.println("Got UDP package");
              // dbg_s("UDP\n");
              _rx_buffer_length = UDP_Recv(_rx_buffer, ip_header_length, packet_length);
              if (_rx_buffer_length > 0)
              {
                buffer_ptr = _rx_buffer + IP_Start + ip_header_length + UDP_HeaderLen;

                if (isSetupData(buffer_ptr))
                {
                  // parse setup data and setup teensy
                  parseSetupData(buffer_ptr);
                  setupPins(&_setup_data);
                  UDP_Send(_rx_buffer, ip_header_length, _rx_buffer_length, ModeReply);
                  is_setup = 1;
                }
                else if (is_setup && isLightData(buffer_ptr))
                {
                  // quickly store received light data to immediately send out sensor data
                  memcpy(light_buffer, buffer_ptr, sizeof(light_buffer));
                  // get sensor data and send it out
                  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
                  {
                    if (digitalRead(_setup_data.sensors[i].pin))
                    {
                      _sensor_data.sensor_value[i] = (uint8_t)0;
                    }
                    else
                    {
                      _sensor_data.sensor_value[i] = (uint8_t)1;
                    }
                  }
                  generateSensorData(buffer_ptr, &_sensor_data);
                  UDP_Send(_rx_buffer, ip_header_length, _rx_buffer_length, ModeReply);

                  // overwrite the rx_buffer again and parse light data and set it
                  memcpy(buffer_ptr, light_buffer, sizeof(light_buffer));
                  parseLightData(buffer_ptr);

                  for (uint8_t i = 0; i < _setup_data.num_block_leds; ++i)
                  {
                    uint8_t index = _setup_data.block_leds[i].index;
                    uint8_t num = index + _setup_data.block_leds[i].num_blocks;
                    for (uint8_t j = index; j < num; ++j)
                    {
                      uint8_t pin = _setup_data.block_leds[i].pin;
                      // light_strips[BLOCK_LEDS_PINS[node_id][i]].setPixelColor((uint16_t)j,
                      light_strips[pin].setPixelColor((uint16_t)j,
                                                      _light_data.block_leds[i].red,
                                                      _light_data.block_leds[i].green,
                                                      _light_data.block_leds[i].blue);
                    }
                  }

//                  for (uint8_t i = 0; i < _setup_data.num_block_leds; ++i)
//                  {
//                    uint8_t num = _setup_data.block_leds[i].index + _setup_data.block_leds[i].num_blocks;
//                    for (uint8_t j = _setup_data.block_leds[i].index; j < num; ++j)
//                    {
//                      uint8_t index = _setup_data.block_leds[i].index;
//                      light_strips[BLOCK_LEDS_PINS[node_id][i]].setPixelColor((uint16_t)j,
//                                                                     _light_data.block_leds[i].red[index],
//                                                                     _light_data.block_leds[i].green[index],
//                                                                     _light_data.block_leds[i].blue[index]);
//
//                      light_strips[BLOCK_LEDS_PINS[node_id][i]].show();
//                    }
//                  }

//                  for (uint8_t i = 0; i < _setup_data.num_pixel_leds; ++i)
//                  {
//                    uint8_t num = _setup_data.pixel_leds[i].index + _setup_data.pixel_leds[i].num_pixels;
//                    for (uint8_t j = _setup_data.pixel_leds[i].index; j < num; ++j)
//                    {
//                      light_strips[PIXEL_LEDS_PINS[node_id][i]].setPixelColor((uint16_t)j,
//                                                              _light_data.pixel_leds[i].red[j],
//                                                              _light_data.pixel_leds[i].green[j],
//                                                              _light_data.pixel_leds[i].blue[j]);
//                    }
//                  }
                  // let there be light
                  for (uint8_t i = 0; i < _setup_data.num_strips_used; ++i)
                  {
                    light_strips[i].show();
                  }
                }
              }
            } // IP Packet Type
          } // else dbg_s("\nUnhandeld IP Packet.");
        } // else dbg_s("\nARP"); // IP or ARP //d bg_c('\n');
      } //  GotEth Packet
    } // Packet Available
  } // Link is Up

}

/* Input Strings, return 0 if they're the same, else -1 */
//signed char SameStrPM(uint8_t *is, char *is2, signed char l)
//{
//  while (l-- > 0)
//    if (*(is++) != pgm_read_byte(is2++))
//      l = -2;
//  return l + 1;
//}
// Return 0xFF if str found
//uint8_t FindStr(uint8_t *p, uint16_t pl, char *s, uint8_t l)
//{
//  uint8_t j;
//  do
//  {
//    for (j = 0; j < l; j++)
//      if (*(p + j) != pgm_read_byte(s + j))
//        j = l << 2;
//    p++;
//    if (j < (l << 1))
//      l = 0xFF;
//  } while (l != 0xFF && (pl--) > l);
//  return l;
//}
/* Make ADC Conversion, return 16bits value */
//uint16_t ADC_Read(uint8_t ch)
//{
//  ADCSRA = _BV(ADEN) | /* Enable ADC */
//  ADC_Prescaler; /* Set ADC Prescaler */
//  ADCSRB = _BV(ADHSM) | /*  High Speed Mode */
//  (ch & 0x20); /* Pin Mux - Last Bit */
//  ADMUX = _BV(REFS0) | /* Analog Reference: Vcc */
//  (ch & 0x1F); /* Pin Mux - 4 First bits */
//  ADCSRA |= _BV(ADSC); /* Start the conversion */
//  while (ADCSRA & _BV(ADSC))
//    ; /* Wait for result */
//  return ADC;
//}

// spi.c
void SPI_Init(unsigned char hof, unsigned char presc)
{
  /* Set MOSI, SCK and CS output all others input */
  SPI_Port = _BV(SPI_MOSI) | _BV(SPI_SCLK) | _BV(SPI_CS);
  SPI_PortD = _BV(SPI_MOSI) | _BV(SPI_SCLK) | _BV(SPI_CS);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = _BV(SPE) | _BV(MSTR) | (presc);
  if (hof)
    SPSR |= _BV(SPI2X); /* Fosc/2 if no prescaler */
}

void SPI_Tx(unsigned char c)
{
  //dbgf_c('w'); dbgf_n(c); dbgf_c(' ');
  SPDR = c; /* Start transmission */
  while (!(SPSR & (1 << SPIF)))
    ; /* Wait for Tx complete */
}

void SPI_Send(unsigned char c)
{
  ChS; /* Chip Select */
  SPI_Tx(c); /* Send Byte */
  ChD; /* Chip Deselect */
}

unsigned char SPI_Rx(void)
{
  SPDR = 0; /* Clear Rx Value */
  while (!(SPSR & (1 << SPIF)))
    ; /* Wait for Rx complete */
  //dbgf_c('r'); dbgf_n(SPDR); dbgf_c(' ');
  return SPDR; /* Return Value */
}

unsigned char SPI_Recv(void)
{
  ChS; /* Chip Select */
  SPI_Rx(); /* SPI Rx */
  ChD; /* Chip Deselect */
  return SPDR; /* Return Value */
}

// ustack
uint16_t ENC_PckRx(uint8_t *b, uint16_t l)
{
  uint16_t pl, ps; /* RxD Packet Length, Packet Status */
  if (ENC_hasRxd)
  {
    ENC_RegWrite(ERDPTL, (ENC_RxNxt) & 0xFF); /* Set the Read Pointer to ENC_RxNxt Address */
    ENC_RegWrite(ERDPTH, (ENC_RxNxt) >> 8);
    ENC_RxNxt = ENC_Read(ENC_RBM, 0); /* Update ENC_RxNxt with the next Packet Address */
    ENC_RxNxt |= ENC_Read(ENC_RBM, 0) << 8;
    pl = ENC_Read(ENC_RBM, 0) - 4; /* Get (Packet-CRC) Length, 2 bytes */
    pl |= ENC_Read(ENC_RBM, 0) << 8;
    ps = ENC_Read(ENC_RBM, 0); /* Get Packet Status, 2 bytes */
    ps |= ENC_Read(ENC_RBM, 0) << 8;
    //dbg_s("\nPckLen: ");dbg_n16(pl);dbg_c('\n');
    //dbg_s("\nPckStat: ");dbg_n16(ps);dbg_c('\n');
    if (ps & 0x80)
    { /* Check in Status if Received Ok (CRC and no symbol error) */
      if (pl + 1 > l) /* Adjust PacketLen to Buffer Len */
        pl = l;
      ENC_BMRead(b, pl); /* Fill b with Packet Data */
      l = (ENC_RxNxt && ENC_RxStart) ? ENC_RxStop : ENC_RxNxt - 1; /* Adjust Address */
      //dbg_n((l)>>8); dbg_n((l)&0xFF);
      ENC_RegWrite(ERXRDPTL, (l) & 0xFF); /* Free Memory in ENC RAM  - Update Read Pointer */
      ENC_RegWrite(ERXRDPTH, (l) >> 8);
    }
    else
      /* Discard Packet */
      pl = 0;
    ENC_Write(ENC_BfS, ECON2, ECON2_PKTDEC); /* Decrement Packet Counter */
  }
  else
    /* No Packet Available */
    pl = 0;
  return pl; /* Return Packet Length */
}

void ENC_PckTx(uint8_t *b, uint16_t l)
{
  while (ENC_RegRead(ECON1) & ECON1_TXRTS) /* Wait for Previous Tx Completion */
    /* Errata Rev7 - Transmit Logic
     * Wait for Tx Error Interrupt, then Reset TxLogic
     */
    if (ENC_RegRead(EIR) & EIR_TXERIF)
    {
      ENC_Write(ENC_BfS, ECON1, ECON1_TXRST); /* Start Reset */
      ENC_Write(ENC_BfC, ECON1, ECON1_TXRST); /* End Reset */
      ENC_Write(ENC_BfC, EIR, EIR_TXERIF); /* Clear Interrupt Flag */
    }

  ENC_RegWrite(EWRPTL, ENC_TxStart&0xFF); /* Set Write Pointer to TxStart */
  ENC_RegWrite(EWRPTH, ENC_TxStart>>8);
  /* Update TxEnd Addr just after Data, let space for TxVector */
  ENC_RegWrite(ETXNDL, (ENC_TxStart+l)&0xFF);
  ENC_RegWrite(ETXNDH, (ENC_TxStart+l)>>8);
  ENC_Write(ENC_WBM, 0, 0); /* Write Control Byte to 0x0, make the ENC use MACON3 settings */
  ENC_BMWrite(b, l); /* Write Data in Memory Buffer */
  ENC_Write(ENC_BfC, EIR, EIR_TXERIF | EIR_TXIF); /* Clear Interrupts Flags */
  ENC_Write(ENC_BfS, ECON1, ECON1_TXRTS); /* Send Packet - Enable Transmit - */
  while (!(ENC_RegRead(EIR) & (EIR_TXERIF | EIR_TXIF)))
    ; /* Wait for Tx Completion or Error */
  ENC_Write(ENC_BfC, ECON1, ECON1_TXRTS); /* Disable Transmit */
  /* *** BEWARE not to Set ERDPT to an even Address *** */
  //~ ENC_RegWrite(ERDPTL,(ENC_TxStart+l+3)&0xFF);         /* Set Read Pointer to ETXND+3 to get Status */
  //~ ENC_RegWrite(ERDPTL,(ENC_TxStart+l+3)>>8);
  //~  = ENC_Read(ENC_RBM,0);                                                        /* Low Bit for truncated Status Vector */
  //~ if(l&0x80) {                                                                                                          /* Check if Tx Error, retry */
  //~     ENC_Write(ENC_BfS,ECON1,ECON1_TXRST);                                                                    /* TxReset Start */
  //~     ENC_Write(ENC_BfC,ECON1,ECON1_TXRST);                                                                      /* TxReset End */
  //~     ENC_Write(ENC_BfC,EIR,EIR_TXERIF|EIR_TXIF);                                             /* Clear Interrupts Flags */
  //~     ENC_Write(ENC_BfS,ECON1,ECON1_TXRTS);                              /* Send Packet - Enable Transmit - */
  //~     //dbg_s("Err : "); dbg_n(st); dbg_c('\n');
  //~ }
}

void ENC_PhyWrite(uint8_t Addr, uint16_t d)
{
  ENC_RegWrite(MIREGADR, Addr); /* Set Addr */
  ENC_RegWrite(MIWRL, d & 0xFF); /* Write Data Low */
  ENC_RegWrite(MIWRH, d >> 8); /* Write Data High */
  while (ENC_RegRead(MISTAT) & MISTAT_BUSY)
    ; /* Wait until PHY completed */
}

uint8_t ENC_PhyReadH(uint8_t Addr)
{
  ENC_RegWrite(MIREGADR, Addr); /* Set Addr */
  ENC_RegWrite(MICMD, MICMD_MIIRD); /* Start Read Op */
  while (ENC_RegRead(MISTAT) & MISTAT_BUSY) /* Wait until PHY completed */
    _delay_us(15);
  ENC_RegWrite(MICMD, 0); /* Reset Read Bit */
  return ENC_RegRead(MIRDH); /* Return High Byte */
}

void ENC_Write(uint8_t Op, uint8_t Addr, uint8_t c)
{
  ChS; /* Chip Select */
  SPI_Tx(Op | (AddrMsk & Addr)); /* SPI Send Op and Addr */
  SPI_Tx(c); /* SPI Send Data */
  ChD; /* Chip Deselect */
}

uint8_t ENC_Read(uint8_t Op, uint8_t Addr)
{
  ENC_SBank((Addr & BankMsk) >> 5); /* Change Bank if Needed */
  ChS; /* Chip Select */
  SPI_Tx(Op | (AddrMsk & Addr)); /* SPI Send Read Operation */
  if (Addr & 0x80) /* SPI Dummy Read if Needed for PHY and MAC */
    SPI_Rx();
  Addr = SPI_Rx(); /* SPI Data Read */
  ChD; /* Chip Deselect */
  return Addr; /* Return Read Value */
}

void ENC_BMWrite(uint8_t *b, uint16_t l)
{
  ChS; /* Chip Select */
  SPI_Tx(ENC_WBM); /* SPI Send Buffer Write Op */
  while (l--) /* For each byte of the Buffer */
    SPI_Tx(*(b++)); /* SPI Send Byte in BM */
  ChD; /* Chip Deselect */
}

void ENC_BMRead(uint8_t *b, uint16_t l)
{
  ChS; /* Chip Select */
  SPI_Tx(ENC_RBM); /* SPI Send Buffer Read Op */
  while (l--) /* While Buffer's not full */
    *(b++) = SPI_Rx(); /* Fill it with Rx Byte */
  ChD; /* Chip Deselect */
}

void ENC_SBank(uint8_t b)
{
  if (b != ENC_Bank)
  {
    //dbg_s("\nGo Bank "); dbg_n(b); dbg_c('\n');
    ENC_Bank = b; /* Update current Bank */
    ENC_Write(ENC_BfC, ECON1, (ECON1_BSEL1 | ECON1_BSEL0)); /* Clear Bank Bits */
    ENC_Write(ENC_BfS, ECON1, b); /* Set Bank Bits */
  }
}

void ENC_Init(uint8_t Mac[6])
{
  ENC_Bank = 4; /* Init Bank Number to an impossible Value*/
  ENC_SoftRst; /* ENC28J60 Soft Reset */
  _delay_ms(10); /* Reset Delay - cf Errata */

  /* Bank 0 - Set Buffer Memory Registers */
  ENC_RegWrite(ERXSTL, ENC_RxStart&0xFF); /* Rx Start - End */
  ENC_RegWrite(ERXSTH, ENC_RxStart>>8);
  ENC_RegWrite(ERXNDL, ENC_RxStop&0xFF);
  ENC_RegWrite(ERXNDH, ENC_RxStop>>8);
  ENC_RegWrite(ERXRDPTL, ENC_RxStart&0xFF); /* RxRead Pointer Addr */
  ENC_RegWrite(ERXRDPTH, ENC_RxStart>>8);
  ENC_RegWrite(ETXSTL, ENC_TxStart&0xFF); /* Tx Start - End */
  ENC_RegWrite(ETXSTH, ENC_TxStart>>8);
  ENC_RegWrite(ETXNDL, ENC_TxStop&0xFF);
  ENC_RegWrite(ETXNDH, ENC_TxStop>>8);
  /* Bank 1 - Packet Filtering -
   * Reject Bad CRC
   * For broadcast packets we allow only ARP packtets
   * All other packets should be unicast for our mac (MAADR)
   *
   * The pattern to match on is therefore
   * Type     ETH.DST
   * ARP      BROADCAST
   * 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
   * in binary these poitions are:11 0000 0011 1111
   * This is hex 303F->EPMM0=0x3f,EPMM1=0x30
   */
  ENC_RegWrite(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
  ENC_RegWrite(EPMM0, 0x3f);
  ENC_RegWrite(EPMM1, 0x30);
  ENC_RegWrite(EPMCSL, 0xf9);
  ENC_RegWrite(EPMCSH, 0xf7);
  /* Bank 2 - MAC Related Stuff -
   * Enable MAC receive, Full Duplex IEEE flow control possible
   * Automatic 60 bytes Padding, CRC Operations
   * IEEE 803.2 Standard conformity - Half Duplex -
   * Maximum Packet Size accepted
   * Back-to-Back Inter-Packet Gap : 0x12->Half Duplex, 0x15->Full
   * Non-Back-to-Back Inter-Packet Gap Low/High to 0x12 and 0xC
   */
  ENC_RegWrite(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
  ENC_RegWrite(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
  ENC_RegWrite(MACON4, MACON4_DEFER);
  ENC_RegWrite(MAMXFLL, ENC_MaxFrameLen&0xFF);
  ENC_RegWrite(MAMXFLH, ENC_MaxFrameLen>>8);
  ENC_RegWrite(MABBIPG, 0x12);
  ENC_RegWrite(MAIPGL, 0x12);
  ENC_RegWrite(MAIPGH, 0xC);
  /* Bank 3 - More MAC Related Stuff -
   * Set MAC Address
   */
  ENC_RegWrite(MAADR6, Mac[5]);
  ENC_RegWrite(MAADR5, Mac[4]);
  ENC_RegWrite(MAADR4, Mac[3]);
  ENC_RegWrite(MAADR3, Mac[2]);
  ENC_RegWrite(MAADR2, Mac[1]);
  ENC_RegWrite(MAADR1, Mac[0]);
  /* PHY Init
   * No Loopback
   */
  ENC_PhyWrite(PHCON2, PHCON2_HDLDIS);
  ENC_Write(ENC_BfS, EIE, EIE_INTIE | EIE_PKTIE); /* Enable Interrupts and Packets Reception */
  ENC_Write(ENC_BfS, ECON1, ECON1_RXEN);
}

// ustack

/* Return Length of Written Data */
uint8_t BuffWrite(uint8_t *b, char *s)
{
  char c = 0x1;
  uint8_t l = 0;
  while (c)
  {
    c = *(s++);
    *(b++) = c;
    l++;
  }
  return l;
}

/* Return Length of Written Data */
uint8_t BuffWritePM(uint8_t *b, char *s)
{
  char c = 0x1;
  uint8_t l = 0;
  while (c)
  {
    c = pgm_read_byte(s++);
    *(b++) = c;
    l++;
  }
  return l;
}

void IP_Send(uint8_t *b, uint8_t hl, uint16_t dl, uint8_t m)
{
  *(b + IP_Start) = (IP_Vers << 4) | (hl >> 2); /* Update Header Length */
  *(b + IP_Total_Len_H) = (hl + dl) >> 8; /* Update Total Length */
  *(b + IP_Total_Len_L) = (hl + dl) & 0xFF;
  if (m & ModeReply)
  { /* If it's a Reply */
    EthH_Reply(b); /* Switch MAC Addresses */
    /* We dont need dl anymore, reuse it */
    for (dl = 0; dl < EthIPLen; dl++)
    { /* Update Dest/Src Ip */
      *(b + IP_DestIP + dl) = *(b + IP_SrcIP + dl);
      *(b + IP_SrcIP + dl) = IPAddr[dl];
    }
  }
  else
  {
    *(b + IP_Checksum_H) = 0; /* Erase Checksum */
    *(b + IP_Checksum_L) = 0;
  }
  *(b + IP_Flags_H) = IP_FlagDF; /* Don't Fragment Flag ON */
  *(b + IP_Flags_L) = 0x0;
  *(b + IP_TTL) = IP_TTL_v; /* Set TTL */
  dl = IP_Checksum(b + IP_Start, hl); /* Compute Checksum */
  *(b + IP_Checksum_H) = dl >> 8; /* Update Checksum */
  *(b + IP_Checksum_L) = dl & 0xFF;
}

void TCP_Send(uint8_t *b, uint8_t hl, TCP_Data *tcp_s, uint8_t m)
{
  uint16_t chks;
  IP_Send(b, hl, tcp_s->HLen + tcp_s->DLen, m); /* Make IP Header */
  b += IP_Start + hl;
  if (m & ModeReply)
  { /* If it's a Reply */
    m = *(b + TCP_SrcPort_H); /* Switch Ports */
    *(b + TCP_SrcPort_H) = *(b + TCP_DestPort_H);
    *(b + TCP_DestPort_H) = m;
    m = *(b + TCP_SrcPort_L);
    *(b + TCP_SrcPort_L) = *(b + TCP_DestPort_L);
    *(b + TCP_DestPort_L) = m;
    for (m = 0; m < 4; m++)
    { /* Update Seq/Ack */
      chks = *(b + TCP_AckN + m); /* Save Ack */
      *(b + TCP_AckN + m) = *(b + TCP_SeqN + m); /* Mv Seq To Ack */
      *(b + TCP_SeqN + m) = chks; /* Mv Acq to Seq */
    }
    /* Should Use uint32_t but assume we don't receive more than 255 bytes at a time */
    *(b + TCP_AckN + 0x3) += tcp_s->Ss; /* Update Seq Number */
    *(b + TCP_SeqN + 0x3) += tcp_s->As;
    *(b + TCP_Flags) = tcp_s->Flags; /* Update Flags */
  }
  else
  {
    *(b + TCP_Flags) = TCP_fPSH | TCP_fACK; /* Update Flags to PSH/ACK*/
    if (m & ModeSingle)
      *(b + TCP_Flags) |= TCP_fFIN; /* If single Packet, send FIN */
  }
  *(b + TCP_HLen) = (tcp_s->HLen >> 2) << 4; /* Update HeaderLen */
  *(b + TCP_Checksum_H) = 0x0; /* Clear Checksum */
  *(b + TCP_Checksum_L) = 0x0;
  chks = UDP_TCP_Checksum(b - IP_Start - hl, hl); /* Compute Checksum */
  *(b + TCP_Checksum_H) = chks >> 8; /* Update */
  *(b + TCP_Checksum_L) = chks & 0xFF;
  b -= IP_Start + hl;
  //dbg_s("\nSend:\n");
  //for(chks = 0;chks<EthLen+hl+tcp_s->HLen+tcp_s->DLen;chks++) {
  //dbg_n(*(b+chks)); dbg_c(' ');
  //}
  Send_Pck(b, EthLen + hl + tcp_s->HLen + tcp_s->DLen); /* Send Packet */
}

/* Return 1 if it's an valid IP Packet, else 0 */
uint8_t TCP_Recv(uint8_t *b, uint8_t hl, uint16_t l, TCP_Data *tcp_s)
{
  uint16_t chks_p = 0;
  b += IP_Start + hl;
#ifdef _UDP_TCP_DCheck
  chks_p = *(b+TCP_Checksum_H)<<8; /* Get Checksum from Packet */
  chks_p |= *(b+TCP_Checksum_L);
  //dbg_s("\nPacket Checksum: ");dbg_n16(chks_p);
#endif
  *(b + TCP_Checksum_H) = 0x0; /* Erase Checksum */
  *(b + TCP_Checksum_L) = 0x0;
  //dbg_s("\nComputed Checksum: ");dbg_n16(UDP_TCP_Checksum(b,hl));
#ifdef _UDP_TCP_DCheck
  if(UDP_TCP_Checksum(b-(IP_Start+hl),hl) == chks_p)
  { /* Check if checksum is valid */
#endif
  tcp_s->Flags = *(b + TCP_Flags); /* Get TCP Flags */
  tcp_s->HLen = (*(b + TCP_HLen) >> 4) * 4; /* Get TCP Data Offset */
  b -= (IP_Start + hl);

  chks_p = IP_GetDataLen(b, hl); /* Get TCP Data Length */

  if (chks_p > l - IP_Start - hl - tcp_s->HLen) /* If packet is truncated, avoid overflow  */
    chks_p = l - IP_Start - hl - tcp_s->HLen;
  tcp_s->DLen = chks_p; /* Update Data Len */
  chks_p = 1;
#ifdef _UDP_TCP_DCheck
}
else
chks_p = 0;
#endif
  return chks_p; /* Return Status */
}

void UDP_Send(uint8_t *b, uint8_t hl, uint16_t dl, uint8_t m)
{
  uint16_t chks;
  IP_Send(b, hl, UDP_HeaderLen + dl, m); /* Update IP Header */
  b += IP_Start + hl;
  if (m & ModeReply)
  { /* If it's a Reply */
    chks = *(b + UDP_SrcPort_H); /* Switch Ports */
    *(b + UDP_SrcPort_H) = *(b + UDP_DestPort_H);
    *(b + UDP_DestPort_H) = chks;
    chks = *(b + UDP_SrcPort_L);
    *(b + UDP_SrcPort_L) = *(b + UDP_DestPort_L);
    *(b + UDP_DestPort_L) = chks;
  }

  *(b + UDP_LenH) = (UDP_HeaderLen + dl) >> 8; /* Update UDP Len */
  *(b + UDP_LenL) = (UDP_HeaderLen + dl) & 0xFF;
  chks = UDP_TCP_Checksum(b - (IP_Start + hl), hl); /* Compute Checksum */
  *(b + UDP_Checksum_H) = chks >> 8; /* Add It */
  *(b + UDP_Checksum_L) = chks & 0xFF;
  b -= IP_Start + hl;
  /* for(i = 0;i<EthLen+hl+UDP_HeaderLen+dl;i++) { dbg_n(*(b+i)); dbg_c(' '); }*/
  Send_Pck(b, EthLen + UDP_HeaderLen + hl + dl); /* Send Packet */
}

/* Return Pseudo Checksum */
uint16_t UDP_TCP_Checksum(uint8_t *b, uint8_t hl)
{
  uint32_t s = 0;
  uint16_t i;
  b += IP_SrcIP;
  for (i = 0; i < EthIPLen; i++)
  { /* Add Src/Dest IP */
    s += ((uint16_t)(*b) << 8) | *(b + 1);
    b += 2;
  }
  b -= IP_SrcIP + 2 * EthIPLen; /* Go back to the beggining of the buffer */
  s += *(b + IP_Protocol); /* Add IP protocol */
  i = (uint16_t)IP_GetDataLen(b, hl); /* Get UDP/TCP Length */
  s += i; /* Add to Sum */
  /* dbg_s("\nDataLen: ");dbg_n16(IP_GetDataLen(b,hl));dbg_c('\n'); */
  b += IP_Start + hl; /* Go to UDP/TCP Header Start Address */
  while (i > 1)
  { /* Compute Checksum of UDP/TCP Header + Data */
    s += (((uint16_t)(*b)) << 8) | *(b + 1);
    i -= 2;
    b += 2;
  }
  if (i > 0)
    s += ((uint16_t)(*b)) << 8;
  while (s >> 16)
    s = (s & 0xFFFF) + (s >> 16);
  return ~((uint16_t)s); /* Return Checksum */
}

/* Return UDP Data Length */
uint16_t UDP_Recv(uint8_t *b, uint8_t hl, uint16_t l)
{
  uint16_t chks_p;
#ifdef _UDP_TCP_DCheck
  chks_p = *(b+IP_Start+hl+UDP_Checksum_H)<<8; /* Get Checksum from Packet */
  chks_p |= *(b+IP_Start+hl+UDP_Checksum_L);
  //dbg_s("\nPacket Checksum: ");dbg_n16(chks_p);
#endif
  *(b + IP_Start + hl + UDP_Checksum_H) = 0x0; /* Erase UDP Checksum */
  *(b + IP_Start + hl + UDP_Checksum_L) = 0x0;
#ifdef _UDP_TCP_DCheck
  //dbg_s("\nComputed Checksum: ");dbg_n16(UDP_TCP_Checksum(b,hl));
  if(UDP_TCP_Checksum(b,hl) == chks_p)
  { /* Check if checksum is valid */
#endif
  chks_p = IP_GetDataLen(b, hl) - UDP_HeaderLen; /* Get UDP Data Len */
  if (chks_p > l - IP_Start - hl - UDP_HeaderLen)/* If packet is truncated, avoid overflow  */
    chks_p = l - IP_Start - hl - UDP_HeaderLen;
#ifdef _UDP_TCP_DCheck
}
else
chks_p =0;
#endif
  return chks_p; /* Return Data Len */
}

/* Return IP"Data Length */
uint16_t IP_GetDataLen(uint8_t *b, uint8_t hl)
{
  return (uint16_t)(((*(b + IP_Total_Len_H)<<8)|(*(b+IP_Total_Len_L)))- hl);
}

void ICMP_Reply(uint8_t *b, uint8_t hl, uint16_t l)
{
  uint16_t chks_p, dl;
  if (*(b + IP_Start + hl) == ICMP_tReq)
  { /* If ICMP Request (Echo), accept */
    dl = IP_GetDataLen(b, hl);
    if (dl > l - EthLen - IP_HeaderLen) /* Avoid Buffer Overflow if packet has been truncated */
      dl = l - EthLen - IP_HeaderLen;
    chks_p = *(b + IP_Start + hl + ICMP_Checksum_H) << 8; /* Get ICMP Checksum */
    chks_p |= *(b + IP_Start + hl + ICMP_Checksum_L);
    *(b + IP_Start + hl + ICMP_Checksum_H) = 0; /* Erase checksum */
    *(b + IP_Start + hl + ICMP_Checksum_L) = 0;
    if (chks_p == IP_Checksum(b + IP_Start + hl, dl))
    {
      IP_Send(b, hl, dl, ModeReply); /* Checksum Ok, Echo Reply */
      *(b + ICMP_Start) = ICMP_tRep; /* Set type to Echo-reply */
      *(b + ICMP_Start + 1) = 0; /* Reset ICMP Code */
      chks_p = IP_Checksum(b + ICMP_Start, dl); /* Compute Checksum */
      //dbg_s(" Computed Checksum: "); dbg_n16(chks_p); dbg_c('\n');
      *(b + ICMP_Start + ICMP_Checksum_H) = chks_p >> 8; /* Update Checksum */
      *(b + ICMP_Start + ICMP_Checksum_L) = chks_p & 0xFF;
      Send_Pck(b, EthLen + IP_HeaderLen + dl); /* Send Reply */
    }
  }
}

/* Return IP Checksum */
uint16_t IP_Checksum(uint8_t *b, uint16_t l)
{
  uint32_t s = 0;
  while (l > 1)
  {
    s += (((uint16_t)(*b)) << 8) | *(b + 1);
    l -= 2;
    b += 2;
  }
  if (l > 0)
    s += ((uint16_t) * b) << 8;
  while (s >> 16)
    s = (s & 0xFFFF) + (s >> 16);
  return ~((uint16_t)s);
}

/* Return 0 if Invalid Packet, or IP Header Length if it's a valid one */
uint8_t IP_Check(uint8_t *b, uint16_t l)
{
  uint16_t chks_p;
  uint8_t hlen;
  hlen = (*(b + IP_Start) & 0xF) * 4; /* Header Len in bytes */
  chks_p = *(b + IP_Protocol); /* Check if Protocol supported */
  if (chks_p == IP_pICMP || chks_p == IP_pUDP || chks_p == IP_pTCP)
  {
    chks_p = (*(b + IP_Checksum_H)) << 8; /* Get Checksum */
    chks_p |= *(b + IP_Checksum_L);
    *(b + IP_Checksum_H) = 0; /* Erase from Packet */
    *(b + IP_Checksum_L) = 0;
    if (chks_p != IP_Checksum((b + IP_Start), hlen)) /* If checksum Invalid, discard packet*/
      hlen = 0;
  }
  else
    hlen = 0; /* Discard Packet */
  return hlen;
}

/* Return 1 if packet was arp, else 0 */
uint8_t ifARP_Reply(uint8_t *b, uint16_t l)
{
  uint8_t i;
  for (i = 0; i < EthIPLen; i++) /* Is for this IP ? */
    if (*(b + ARP_DestIP + i) != IPAddr[i])
      i = 0xFE;

  if (i != 0xFF)
  {
    if (*(b + ARP_Op_H)== ARP_Op_Req_H && *(b+ARP_Op_L) == ARP_Op_Req_L){ /* Is ARP Request */
EthH_Reply    (b); /* Well, make ARP Answer */

    *(b+ARP_Op_L) = ARP_Op_Rep_L; /* Update ARP Op */
    for(i=0;i<EthMacLen;i++)
    { /* Update ARP Dest/Src MAC */
      *(b+ARP_DestMac+i) = *(b+i);
      *(b+ARP_SrcMac+i) = MACAddr[i];
    }
    for(i=0;i<EthIPLen;i++)
    { /* Update ARP Dest/Src IP */
      *(b+ARP_DestIP+i) = *(b+ARP_SrcIP+i);
      *(b+ARP_SrcIP+i) = IPAddr[i];
    }
    Send_Pck(b,l); /* Send Reply on the Network */
    i=1; /* Update Status - Packet Handeld */
  }
}
else
i++; /* Set Status to 0 */
  return i; /* Return Status */
}

void EthH_Reply(uint8_t *b)
{
  uint8_t i;
  for (i = 0; i < EthMacLen; i++)
  { /* Update Ethernet Header for reply (switch Src/Dest MAC) */
    *(b + i) = *(b + i + EthMacLen);
    *(b + EthMacLen + i) = MACAddr[i];
  }
}
