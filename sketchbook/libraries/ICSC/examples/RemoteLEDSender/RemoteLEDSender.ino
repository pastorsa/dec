#include <ICSC.h>

const uint8_t my_station_id = 2;
const uint8_t remote_station_id = 50;

boolean flag = false;

const uint8_t DE_PIN = 7;
const uint8_t RE_PIN = 8;


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
  ICSC.begin(my_station_id, 115200, DE_PIN);

  // LED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // enable the RS-485 bus
  pinMode(DE_PIN, OUTPUT); // DE
  pinMode(RE_PIN, OUTPUT); // NOT_RE

  digitalWrite(DE_PIN, HIGH);  // DE
  digitalWrite(RE_PIN, LOW);   // NOT_RE  
}

void loop()
{
  if ((millis() - ts) >= interval)
  {
    ts = millis();
    if (flag)
    {
      // ICSC.send(ICSC_BROADCAST, 'P', 0, NULL);
      ICSC.send(remote_station_id, 'P', 0, NULL);      
    } 
    else
    {
      // ICSC.send(ICSC_BROADCAST, 'R', 0, NULL);
      ICSC.send(remote_station_id, 'R', 0, NULL);      
      blink();
    }
    flag = !flag;
  }
  ICSC.process();
}
