// Present a "Will be back soon web page", as stand-in webserver.
// 2011-01-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
 
#include <EtherCard.h>
#include <DEC.h>


// Fixed constants need to be adjusted for each arduino eventually
static const uint8_t DEC_NODE_ID = 1;

// ethernet mac address - must be unique on your network
static const uint8_t mymac[] = { 0x11,0x22,0x33,0x44,0x55,0x01 };
// static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

static const uint8_t PORT = 11311;

// ethernet interface ip address
static const uint8_t myip[] = { 10,0,0,2 };
// gateway ip address
static const uint8_t gwip[] = { 10,0,0,1 };

static const uint16_t buffer_size = 500;
uint8_t Ethernet::buffer[buffer_size]; // tcp/ip send and receive buffer
// BufferFiller bfill;

/*! Communication interface used to generate/parse messages
 */
// DECInterface dec_interface;

// callback to handle the received message
/*
 * raw: message length is NUM_LED * 3
 * cmd:start,dir,i1,i2,dLen,<data>
 * Art-Net.....
 */
void handleMessage(uint16_t port, uint8_t ip[4], const char *data, uint16_t len)
{
  // IPAddress src(ip[0], ip[1], ip[2], ip[3]);
  // Serial.println(src);
  // Serial.println(port);
  // Serial.println(len);

  Serial.println("Received message");

  ether.printIp("Received message from ", ip);
  Serial.print("port=");
  Serial.print((int) port);
  Serial.print("  length=");
  Serial.println((int) len);
}

void setup()
{
  Serial.begin(57600);
  Serial.print("Node >");
  Serial.print(DEC_NODE_ID);
  Serial.println("< setup:");

  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
  {
    Serial.println( "Failed to access Ethernet controller");
  }
  if(!ether.staticSetup(myip, gwip))
  {
    Serial.println( "Failed to setup Ethernet controller");
  }

  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);

  ether.udpServerListenOnPort(&handleMessage, PORT);
}

void loop()
{
  // this must be called for ethercard functions to work.
  ether.packetLoop(ether.packetReceive());
  
  static unsigned long ts = millis();
  if (millis() - ts >= 2000)
  {
    ts = millis();
    Serial.println("running");
  }
}
