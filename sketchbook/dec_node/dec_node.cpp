#include <ICSC.h>
#include <DEC.h>
#include <Adafruit_NeoPixel.h>

/*! Custom variable
 * DEC_NODE_ID must be within [1, DEC_NUM_NODES]
 */
static const uint8_t DEC_NODE_ID = 0;
static const uint8_t DEC_ICSC_DE_PIN = 7;

/*! Communication interface used to generate/parse messages
 */
DECInterface dec_interface;

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
    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
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

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
    {
      dec_interface.generateSensorData(DEC_NODE_ID);
      // send back for now
      dec_interface.setTokenToController();
      ICSC.broadcast(command, dec_interface.length_, dec_interface.data_);
    }
  }

  else if (command == DEC_LIGHT_DATA)
  {
    // parse data into local memory
    dec_interface.parseLightData(source, data);
    //

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
    {
      dec_interface.generateLightData(DEC_NODE_ID);
      // send back for now
      dec_interface.setTokenToController();
      ICSC.broadcast(command, dec_interface.length_, dec_interface.data_);
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
  ICSC.begin(DEC_NODE_ID, DEC_BAUD_RATE, DEC_ICSC_DE_PIN);
  ICSC.registerCommand(ICSC_BROADCAST, &receive_broadcast);

  blink();
}

/*!
 */
void loop()
{
  ICSC.process();

  if (!error)
  {
    // only react
    if(msg_received)
    {
      msg_received = false;
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
