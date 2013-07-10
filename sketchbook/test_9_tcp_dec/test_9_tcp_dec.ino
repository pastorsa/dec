// Present a "Will be back soon web page", as stand-in webserver.
// 2011-01-30 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
 
#include <EtherCard.h>
#include <DEC.h>

// Fixed constants need to be adjusted for each arduino eventually
const uint8_t DEC_NODE_ID = 1;

// ethernet mac address - must be unique on your network
static uint8_t mymac[] = { 0x11,0x22,0x33,0x44,0x55,0x01 };
// static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };

// ethernet interface ip address
static uint8_t myip[] = { 10,0,0,2 };
// gateway ip address
static uint8_t gwip[] = { 10,0,0,1 };

static const uint16_t buffer_size = 1500;
uint8_t Ethernet::buffer[buffer_size]; // tcp/ip send and receive buffer
BufferFiller bfill;

/*! Communication interface used to generate/parse messages
 */
DECInterface dec_interface;

void setup ()
{
  Serial.begin(57600);
  if (ether.begin(buffer_size, mymac) == 0)
    Serial.println( "Failed to access Ethernet controller");
  ether.staticSetup(myip,gwip);
}

static uint16_t homePage() {
  long t = millis() / 1000;
  word h = t / 3600;
  byte m = (t / 60) % 60;
  byte s = t % 60;
  bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Pragma: no-cache\r\n"
    "\r\n"
    "<meta http-equiv='refresh' content='1'/>"
    "<title>RBBB server</title>"
    "<h1>$D$D:$D$D:$D$D</h1>"),
      h/10, h%10, m/10, m%10, s/10, s%10);
  return bfill.position();
}

/*
static uint16_t getSensorData()
{
  // dec_interface.generateSensorData(DEC_NODE_ID);
  // send back for now
  // dec_interface.setTokenToController();

  bfill = ether.tcpOffset();

  dec_interface.data_[0] = 0;
  dec_interface.data_[1] = 1;
  dec_interface.data_[2] = 2;
  dec_interface.data_[3] = 3;
  dec_interface.length_ = 4;

  bfill.emit_raw(dec_interface.data_, dec_interface.length_);
  return bfill.position();
}
*/

void loop ()
{
  uint16_t len = ether.packetReceive();
  uint16_t pos = ether.packetLoop(len);

//  if (pos)  // check if valid tcp data is received
//    ether.httpServerReply(getSensorData()); // send web page data
  if (pos)  // check if valid tcp data is received
  {
    Serial.println('receove');
    ether.httpServerReply(homePage()); // send web page data
  }
  else
  {
    
  }
