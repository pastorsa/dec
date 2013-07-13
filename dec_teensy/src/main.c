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

// This file needs to be adjusted for each upload
#include "address.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

// for input/output
#include "Arduino.h"
#include <avr/io.h>

// DEC includes
#include <dec_communication/dec_communication.h>

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define ADC_Prescaler (_BV(ADPS2) | _BV(ADPS1))
#define ADC_Mux 0

#define TLed PORTD ^= _BV(6)

// spi.c
void SPI_Init(unsigned char hof,unsigned char presc) {
        /* Set MOSI, SCK and CS output all others input */
        SPI_Port = _BV(SPI_MOSI)|_BV(SPI_SCLK)|_BV(SPI_CS);
        SPI_PortD = _BV(SPI_MOSI)|_BV(SPI_SCLK)|_BV(SPI_CS);
        /* Enable SPI, Master, set clock rate fck/16 */
        SPCR = _BV(SPE)|_BV(MSTR)|(presc);
        if(hof)
                SPSR |= _BV(SPI2X);                                     /* Fosc/2 if no prescaler */
}

void SPI_Tx(unsigned char c) {
        //dbgf_c('w'); dbgf_n(c); dbgf_c(' ');
        SPDR = c;                                                                                        /* Start transmission */
        while(!(SPSR & (1<<SPIF)));                      /* Wait for Tx complete */
}

void SPI_Send(unsigned char c) {
        ChS;                                                                                                                            /* Chip Select */
        SPI_Tx(c);                                                                                                                        /* Send Byte */
        ChD;                                                                                                                    /* Chip Deselect */
}

unsigned char SPI_Rx(void) {
        SPDR=0;                                                                                                   /* Clear Rx Value */
        while(!(SPSR & (1<<SPIF)));                     /* Wait for Rx complete */
        //dbgf_c('r'); dbgf_n(SPDR); dbgf_c(' ');
        return SPDR;                                                                                             /* Return Value */
}

unsigned char SPI_Recv(void) {
        ChS;                                                                                                                               /* Chip Select */
        SPI_Rx();                                                                                                                                  /* SPI Rx */
        ChD;                                                                                                                    /* Chip Deselect */
        return SPDR;                                                                                         /* Return Value */
}

// ustack











/*!
 */
void setupPins(const setup_data_t* setup_data)
{
  for (uint8_t i = 0; i < setup_data->num_led_nodes; ++i)
  {
    pinMode(setup_data->led_nodes[0].pin, OUTPUT);
    digitalWrite(setup_data->led_nodes[0].pin, LOW);
  }

  for (uint8_t i = 0; i < setup_data->num_led_beams; ++i)
  {
    pinMode(setup_data->led_beams[i].pin, OUTPUT);
    digitalWrite(setup_data->led_beams[i].pin, LOW);
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

void setup()
{
  // init
  resetData();
  // dec_init();

  /*

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

  */
}

void loop()
{
  /*
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
                }
                else if (isLightData(buffer_ptr))
                {
                  // quickly store received light data to immediately send out sensor data
                  memcpy(light_buffer, buffer_ptr, sizeof(light_buffer));
                  // get sensor data and send it out
                  for (uint8_t i = 0; i < _setup_data.num_sensors; ++i)
                  {
                    if (digitalRead(_setup_data.sensors[i].pin))
                    {
                      _sensor_data.sensor_value[i] = (uint8_t)1;
                    }
                    else
                    {
                      _sensor_data.sensor_value[i] = (uint8_t)0;
                    }
                  }
                  generateSensorData(buffer_ptr, &_sensor_data);
                  UDP_Send(_rx_buffer, ip_header_length, _rx_buffer_length, ModeReply);

                  // overwrite the rx_buffer again and parse light data and set it
                  memcpy(buffer_ptr, light_buffer, sizeof(light_buffer));
                  parseLightData(buffer_ptr);
                }
              }
            } // IP Packet Type
          } // else dbg_s("\nUnhandeld IP Packet.");
        } // else dbg_s("\nARP"); // IP or ARP //d bg_c('\n');
      } //  GotEth Packet
    } // Packet Available
  } // Link is Up
  */
}

//int main(void)
//{
//
//  while (1)
//  {
//  } // Main Loop
//}

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
