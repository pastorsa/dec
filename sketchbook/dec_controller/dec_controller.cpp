#include <ICSC.h>
#include <DEC.h>

// include generated file
#include "dec_structure.h"

/*! Communication interface used to generate/parse messages
 */
DECInterface dec_interface;

// Fixed constants need to be adjusted for each arduino eventually
const uint8_t NODE_ID = 1;
const uint8_t CONTROLLER_ICSC_DE_PIN = 7;

// Local variables
boolean error = false;
boolean msg_received = false;

// Local variables for debugging
const int led_pin = 13;         // the number of the LED pin
int led_state = LOW;             // ledState used to set the LED
long previous_millis = 0;        // will store last time LED was updated
unsigned long interval = 500;   // interval at which to blink (milliseconds)
void blink()
{
  unsigned long current_millis = millis();
  if(current_millis - previous_millis > interval)
  {
    previous_millis = current_millis;
    if (led_state == LOW)
      led_state = HIGH;
    else
      led_state = LOW;
    digitalWrite(led_pin, led_state);
  }
}

// This function is called whenever a broadcast was send
void receive_broadcast(unsigned char source, char command, unsigned char length, char *data)
{
  blink();

  if (command == DEC_SETUP_DATA)
  {
    // parse data into local memory
    dec_interface.parseSetupData(data);
    // check whether it's for me
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {


      // start the sensor data loop
      // dec_interface.generateRequest(0); // token = 0
      // ICSC.broadcast(DEC_SETUP_DATA, dec_interface.length_, dec_interface.data_);
    }
  }

  else if (command == DEC_SENSOR_DATA)
  {
    // parse data into local memory
    dec_interface.parseSensorData(source, data);

    // check whether it's for me
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {
      // continue the sensor data loop
      dec_interface.generateRequest(0); // token = 0
      ICSC.broadcast(DEC_SETUP_DATA, dec_interface.length_, dec_interface.data_);
    }
  }

  else if (command == DEC_LIGHT_DATA)
  {
    // parse data into local memory
    dec_interface.parseLightData(source, data);

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_CONTROLLER_ID)
    {
      // not implemented yet
    }
  }

  msg_received = true;
}


/*!
 */
void setup()
{
  // setup led blinking for debugging
  pinMode(led_pin, OUTPUT);

  // setup communication
  ICSC.begin(DEC_CONTROLLER_ID, DEC_BAUD_RATE, CONTROLLER_ICSC_DE_PIN);
  ICSC.registerCommand(ICSC_BROADCAST, &receive_broadcast);

  dec_interface.loadSetupData();
  if(dec_interface.generateSetupData(0)) // token = 0
  {
    ICSC.broadcast(0, dec_interface.length_, dec_interface.data_);
  }
  // else
  // {
  //   error = true;
  // }
}

/*!
 */
void loop()
{
  ICSC.process();

  static unsigned long ts = millis();
  if (millis() - ts >= 1000)
  {
    ts = millis();
    if(dec_interface.generateSetupData(0)) // token = 0
    {
      ICSC.broadcast(0, dec_interface.length_, dec_interface.data_);
    }
  }

  if(!error)
  {
    if(msg_received)
    {
      msg_received = false;
      ts = millis();
    }
  }
}

// Only used for development in eclipse
#ifdef ECLIPSE_ONLY
int main(void)
{
  return 0;
}
#endif
