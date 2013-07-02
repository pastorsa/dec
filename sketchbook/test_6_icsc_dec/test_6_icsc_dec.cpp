#include <ICSC.h>
#include <DEC.h>

// Fixed constants need to be adjusted for each arduino eventually
const uint8_t DEC_NODE_ID = 1;
const uint8_t ICSC_DE_PIN = 7;

// Local variables
boolean msg_received = false;

/*! Communication interface used to generate/parse messages
 */
DECInterface dec_interface;

// Local variables for debugging
const int led_pin = 13;         // the number of the LED pin
int led_state = LOW;             // ledState used to set the LED
long previous_millis = 0;        // will store last time LED was updated
unsigned long interval = 100;   // interval at which to blink (milliseconds)
static unsigned long ts = millis();
void blink()
{
  unsigned long current_millis = millis();
  if(current_millis - previous_millis > interval)
  {
    if (led_state == LOW)
      led_state = HIGH;
    else
      led_state = LOW;
    digitalWrite(led_pin, led_state);
  }
}

/*
// This function is called whenever a broadcast was send
void receive(unsigned char source, char command, unsigned char length, char *data)
{
  msg_received = true;
  if (command == DEC_SETUP_DATA)
  {
    // parse data into local memory
    dec_interface.parseSetupData(data);
    // check whether it's my turn to broadcast my data
    // if (dec_interface.token_ == DEC_NODE_ID)
    {
      // setup the node
      // Adafruit_NeoPixel test(13);

      // acknowledge
      dec_interface.generateSetupData(DEC_NODE_ID);
      // send back for now
      dec_interface.setTokenToController();
      ICSC.broadcast(command, dec_interface.length_, dec_interface.data_);
    }
  }
  else if (command == DEC_SENSOR_DATA)
  {
    // parse data into local memory
    dec_interface.parseSensorData(source, data);

    // update

    // check whether it's my turn to broadcast my data
    //if (dec_interface.token_ == DEC_NODE_ID)
    {
      dec_interface.generateSensorData(DEC_NODE_ID);
      // send back for now
      dec_interface.setTokenToController();
      ICSC.broadcast(command, dec_interface.length_, dec_interface.data_);
    }
  }
  //  else if (command == DEC_LIGHT_DATA)
  //  {
  //    // parse data into local memory
  //    dec_interface.parseLightData(source, data);
  //    //
  //
  //    // check whether it's my turn to broadcast my data
  //    if (dec_interface.token_ == DEC_NODE_ID)
  //    {
  //      dec_interface.generateLightData(DEC_NODE_ID);
  //      // send back for now
  //      dec_interface.setTokenToController();
  //      ICSC.broadcast(command, dec_interface.length_, dec_interface.data_);
  //    }
  //  }
}
*/

void setup()
{
  ICSC.begin(DEC_NODE_ID, DEC_BAUD_RATE, ICSC_DE_PIN);
 // ICSC.registerCommand(DEC_SETUP_DATA, &receive);
 // ICSC.registerCommand(DEC_SENSOR_DATA, &receive);
  // ICSC.registerCommand(DEC_LIGHT_DATA, &receive);

  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, led_state);

  // enable the RS-485 bus
  pinMode(7, OUTPUT); // DE
  pinMode(8, OUTPUT); // NOT_RE

  digitalWrite(7, LOW);  // DE
  digitalWrite(8, LOW);   // NOT_RE
}

void loop()
{
  ICSC.process();
  
  dec_interface.loadSetupData();
  
  static unsigned long ts = millis();
  if (/*!msg_received &&*/ millis() - ts >= 100)
  {
    ts = millis();
    blink();
  }
  if (msg_received)
  {
    msg_received = false;
    if (led_state == LOW)
      led_state = HIGH;
    else
      led_state = LOW;
    digitalWrite(led_pin, led_state);
  }
}



