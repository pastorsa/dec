#include <DEC.h>
#include <ICSC.h>

const uint8_t DEC_NODE_ID = 1;
const uint8_t ICSC_DE_PIN = 7;

boolean msg_received = false;

// Local variables for debugging
const int led_pin = 13;         // the number of the LED pin
int led_state = LOW;             // ledState used to set the LED
long previous_millis = 0;        // will store last time LED was updated
unsigned long interval = 1000;   // interval at which to blink (milliseconds)
static unsigned long ts = millis();
void blink()
{
  unsigned long current_millis = millis();
  previous_millis = current_millis;
  if (led_state == LOW)
    led_state = HIGH;
  else
    led_state = LOW;
  digitalWrite(led_pin, led_state);
}

void setup()
{
  ICSC.begin(DEC_NODE_ID, DEC_BAUD_RATE, ICSC_DE_PIN);
  ICSC.registerCommand('P', &pressed);
  ICSC.registerCommand('R', &released);

  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // enable the RS-485 bus
  pinMode(7, OUTPUT); // DE
  pinMode(8, OUTPUT); // NOT_RE

  digitalWrite(7, LOW);  // DE
  digitalWrite(8, LOW);   // NOT_RE
}

void loop()
{
  ICSC.process();
}

void pressed(unsigned char src, char command, unsigned char len, char *data)
{
  msg_received = true;
  digitalWrite(13, HIGH);
}

void released(unsigned char src, char command, unsigned char len, char *data)
{
  msg_received = true;
  digitalWrite(13, LOW);
}


