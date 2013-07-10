#include <DEC.h>

// Fixed constants need to be adjusted for each arduino eventually
const uint8_t DEC_NODE_ID = 0;

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

void setup()
{
  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, led_state);

}

void loop()
{
  dec_interface.loadSetupData();
  
  static unsigned long ts = millis();
  if (/*!msg_received &&*/ millis() - ts >= 200)
  {
    ts = millis();
    // blink();
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



