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
 */

#include "common.h"
#include "main.h"
#include "spi.h"
#include "enc28j60.h"
#include "ustack.h"

int main(void)
{
  uint16_t PLen, DLen;
  uint8_t RxB[BFSize ], *Ptr, IPHLen, k;
  char Str[4];
  TCP_Data Tcp;

  CPU_PRESCALE(0); /* Run at 16 MHz */
  DDRD = _BV(DDD6); /* Teensy Onboad Led Enable */
  /* ADC */
  DDRF = ~_BV(DDF0); /* F0 as Input */
  //PORTF = _BV(PORTF0);														 /* Enable Pullup */
  DIDR0 = _BV(ADC0D); /* Disable Digital Pin */

  dbg_init
  ; /* If Debug State On, Init USB */
  SPI_Init(SPI_DP2, SPI_PRESC_1); /* AVR SPI Init */
  do
  {
    ENC_Init(MACAddr); /* ENC28J60 Init */
    _delay_ms(500);
    k = ENC_RevID;
  } while (!k);

  ENC_LEDInit; /* ENC Leds Init */
  while (1)
  {
    //TLed;
    //_delay_ms(300);
    //dbg_c(ENC_LinkSt?'~':'_');
    if (ENC_LinkSt)
    {
      if (ENC_hasRxd)
      {
        // return Packet Size
        PLen = ENC_PckRx(RxB, BFSize);
        if (PLen > 0)
        {
          /*dbg_s("\nGot Packet:\n");
           for(i=0;i<l;i++) {
           dbg_n(RxB[i]); dbg_c(' ');
           } */
          // Return 1 if packet was ARP, and was handeld
          if (!ifARP_Reply(RxB, PLen))
          {
            // Return IPHLen if valid IP Packet, else 0
            IPHLen = IP_Check(RxB, PLen);
            if (IPHLen > 0)
            {
              dbg_s("\nIP - ");
              // Return Protocol
              DLen = IP_gProtocol(RxB);
              if (DLen == IP_pICMP)
              {
                dbg_s("ICMP");
                ICMP_Reply(RxB, IPHLen, PLen);
              }
              else if (DLen == IP_pUDP)
              {
                dbg_s("UDP");
                // Return Data Len
                DLen = UDP_Recv(RxB, IPHLen, PLen);
                if (DLen > 0)
                {
                  //dbg_s(" on Port ");
                  //dbg_n(RxB[IP_Start+j+UDP_DestPort_H]);
                  //dbg_n(RxB[IP_Start+j+UDP_DestPort_L]);
                  //dbg_s(" Got Message: ");
                  //dbg_c(' ');
                  //dbg_n(i);
                  //for(l=0;l<i;l++)
                  //dbg_c(RxB[IP_Start+j+UDP_HeaderLen+l]);
                  Ptr = RxB + IP_Start + IPHLen + UDP_HeaderLen;
                  if (SameStrPM(Ptr, PSTR("Ts:sT"), 5))
                    DLen = BuffWritePM(Ptr, PSTR("Not Auth"));
                  else
                  {
                    if (!SameStrPM(Ptr + 6, PSTR("TLed"), 4))
                      TLed;
                    DLen = BuffWritePM(Ptr, PSTR("Granted"));
                  }
                  DLen += BuffWritePM(Ptr + DLen, PSTR("\n>> "));
                  //for(l=0;l<i-1;l++)
                  //RxB[IP_Start+j+UDP_HeaderLen+l]='0';
                  UDP_Send(RxB, IPHLen, DLen, ModeReply);
                  //UDP_Send(RxB,j,i,ModeSend);
                }
              }
              else
              {
                dbg_s("TCP");
                // Return 1 if accepted
                if (TCP_Recv(RxB, IPHLen, PLen, &Tcp))
                {
                  /*dbg_s(" on Port ");
                   dbg_n(RxB[IP_Start+j+TCP_DestPort_H]);
                   dbg_n(RxB[IP_Start+j+TCP_DestPort_L]);
                   dbg_s(" Packet Ok: Flags: ");
                   dbg_n16(Tcp.Flags);
                   dbg_s(" - HLen: ");
                   dbg_n16(Tcp.HLen);
                   dbg_s(" - DLen: ");
                   dbg_n16(Tcp.DLen);
                   if(Tcp.DLen > 0) {
                   dbg_s(" - Message: ");
                   dbg_c('\n');
                   for(i=0;i<Tcp.DLen;i++)
                   dbg_c(RxB[IP_Start+IPHLen+Tcp.HLen+i]);
                   } */
                  if (Tcp.Flags == TCP_fSYN)
                  {
                    //dbg_s(" - Got SYN, do SYN_ACK");
                    Tcp.Ss = 1;
                    Tcp.As = 0;
                    Tcp.Flags = TCP_fSYN | TCP_fACK;
                    TCP_Send(RxB, IPHLen, &Tcp, ModeReply);
                    k = 0;
                  }
                  else if (Tcp.Flags == TCP_fACK)
                  {
                    //dbg_s(" - Got Ack");
                    if (k == 1)
                    {
                      Tcp.As = 0;
                      Tcp.Ss = 0;
                      Tcp.DLen = 0;
                      Tcp.Flags = TCP_fACK | TCP_fFIN;
                      TCP_Send(RxB, IPHLen, &Tcp, ModeReply);
                      k = 2;
                    }
                  }
                  else if (Tcp.Flags == (TCP_fACK | TCP_fPSH))
                  {
                    //dbg_s(" - Got Push-Ack, do Ack");
                    Tcp.Ss = Tcp.DLen;
                    Tcp.As = 0;
                    Tcp.Flags = TCP_fACK;
                    DLen = Tcp.DLen;
                    Tcp.DLen = 0;
                    TCP_Send(RxB, IPHLen, &Tcp, ModeReply);
                    Ptr = RxB + IP_Start + IPHLen + Tcp.HLen;
                    if (Get16B(Ptr-Tcp.HLen+TCP_SrcPort_H) == 0x50)
                    {
                      if (!SameStrPM(Ptr, PSTR("GET /"), 4))
                      {
                        Tcp.Ss = 1;
                        // Return 0xFF if str found
                        // If Non auth, deny access
                        if (FindStr(Ptr, 0x200, PSTR("dGVzdDp0ZXN0"), 12) != 0xFF)
                          Tcp.DLen = BuffWritePM(Ptr, PSTR("HTTP/1.1 401 NoAuth\r\n"
                                                           "WWW-Authenticate: Basic realm=\"TeenSee\"\r\n"
                                                           "Content-Type: text/html\r\n\r\n"
                                                           "<html>\n\t<head><title>TeenC</title></head>"
                                                           "<body><h1>Auth Required</h1></body></html>"));
                        else
                        { // Else, allow access
                          // If page is not root: / -> 404
                          if (*(Ptr + 5) != ' ' && *(Ptr + 5) != '?')
                            Tcp.DLen = BuffWritePM(Ptr, PSTR("HTTP/1.1 404 Not Found\r\n"
                                                             "Content-Type: text/html\r\n\r\n"
                                                             "<html><head><title>TeenC</title></head>"
                                                             "<body>"
                                                             "<h1>404 - I think you're lost...</h1>"
                                                             "</body></html>"));
                          else
                          {
                            if (*(Ptr + 6) == 'L' && *(Ptr + 7) == 'T')
                            {
                              TLed;
                              Tcp.DLen = BuffWritePM(Ptr, PSTR("HTTP/1.1 303 Ok\r\n"
                                                               "Location: /\r\n\r\n"));
                            }
                            else
                            {
                              Tcp.DLen = BuffWritePM(Ptr, PSTR("HTTP/1.1 200 Ok\r\n"
                                                               "Content-Type: text/html\r\n\r\n"
                                                               "<html><head><title>TeenC</title></head>"
                                                               "<body><h1>It works!</h1><p>Led is O"));
                              Tcp.DLen += BuffWritePM(Ptr + Tcp.DLen, (PORTD & _BV(6) ? PSTR("n") : PSTR("ff")));
                              Tcp.DLen += BuffWritePM(Ptr + Tcp.DLen, PSTR(" - <a href=\"?LT\">Toggle</a>"
                                                                           "<br />ADC Pin Value: "));
                              Tcp.DLen += BuffWrite(Ptr + Tcp.DLen, itoa(ADC_Read(ADC_Mux), Str, 10));
                              Tcp.DLen += BuffWritePM(Ptr + Tcp.DLen, PSTR("</p></body></html>"));
                            }
                          }
                        }
                        /* Send Back Data */
                        TCP_Send(RxB, IPHLen, &Tcp, ModeSingleSend);
                        k = 2;
                      }
                    }
                    else
                    {
                      if (!SameStrPM(Ptr, PSTR("TL"), 2))
                        TLed;
                      Tcp.DLen = BuffWritePM(Ptr, PSTR(">> "));
                      /* Send Back Data */
                      TCP_Send(RxB, IPHLen, &Tcp, ModeSend);
                      k = 0;
                    }
                  }
                  else if (Tcp.Flags == (TCP_fACK | TCP_fFIN))
                  {
                    //dbg_s("End\n");
                    Tcp.As = 0;
                    Tcp.DLen = 0;
                    if (k != 2)
                    {
                      Tcp.Ss = 0;
                      Tcp.Flags = TCP_fACK | TCP_fFIN;
                    }
                    else
                    {
                      Tcp.Ss = 1;
                      Tcp.Flags = TCP_fACK;
                    }
                    TCP_Send(RxB, IPHLen, &Tcp, ModeReply);
                    k = 2;
                  } //else  dbg_s("Unknown Flag "); // Flags
                } // TCP Packet Accepted
              } // IP Packet Type
            } //else dbg_s("\nUnhandeld IP Packet.");
          } //else dbg_s("\nARP");					// IP or ARP
          //dbg_c('\n');
        } //  GotEth Packet
      } // Packet Available
    } // Link is Up
  } // Main Loop
}

/* Input Strings, return 0 if they're the same, else -1 */
signed char SameStrPM(uint8_t *is, char *is2, signed char l)
{
  while (l-- > 0)
    if (*(is++) != pgm_read_byte(is2++))
      l = -2;
  return l + 1;
}

// Return 0xFF if str found
uint8_t FindStr(uint8_t *p, uint16_t pl, char *s, uint8_t l)
{
  uint8_t j;
  do
  {
    for (j = 0; j < l; j++)
      if (*(p + j) != pgm_read_byte(s + j))
        j = l << 2;
    p++;
    if (j < (l << 1))
      l = 0xFF;
  } while (l != 0xFF && (pl--) > l);
  return l;
}

/* Make ADC Conversion, return 16bits value */
uint16_t ADC_Read(uint8_t ch)
{
  ADCSRA = _BV(ADEN) | /* Enable ADC */
  ADC_Prescaler; /* Set ADC Prescaler */
  ADCSRB = _BV(ADHSM) | /*  High Speed Mode */
  (ch & 0x20); /* Pin Mux - Last Bit */
  ADMUX = _BV(REFS0) | /* Analog Reference: Vcc */
  (ch & 0x1F); /* Pin Mux - 4 First bits */
  ADCSRA |= _BV(ADSC); /* Start the conversion */
  while (ADCSRA & _BV(ADSC))
    ; /* Wait for result */
  return ADC;
}