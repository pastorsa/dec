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

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop()
{
  static unsigned long ts = millis();
  if (millis() - ts >= 200)
  {
    ts = millis();
    blink();
  }
}

