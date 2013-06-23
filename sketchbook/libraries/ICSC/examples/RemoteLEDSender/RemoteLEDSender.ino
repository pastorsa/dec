#include <ICSC.h>

const uint8_t my_station_id = 2;
const uint8_t remote_station_id = 50;

boolean flag = false;

// Local variables for debugging
const int led_pin = 13;         // the number of the LED pin
int led_state = LOW;             // ledState used to set the LED
long previous_millis = 0;        // will store last time LED was updated
unsigned long interval = 500;   // interval at which to blink (milliseconds)
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
  ICSC.begin(my_station_id, 115200);

  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // enable the RS-485 bus
  pinMode(7, OUTPUT); // DE
  pinMode(8, OUTPUT); // NOT_RE

  digitalWrite(7, HIGH);  // DE
  digitalWrite(8, LOW);   // NOT_RE
}

void loop()
{
  if ((millis() - ts) >= interval)
  {
    blink();
    ts = millis();
    if (flag)
    {
      ICSC.send(ICSC_BROADCAST, 'P', 0, NULL);
      // ICSC.send(remote_station_id, 'P', 0, NULL);      
    } 
    else
    {
      ICSC.send(ICSC_BROADCAST, 'R', 0, NULL);
      // ICSC.send(remote_station_id, 'R', 0, NULL);      
    }
    flag = !flag;
  }
}
