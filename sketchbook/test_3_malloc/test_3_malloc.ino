/* Blink without Delay
 
 Turns on and off a light emitting diode(LED) connected to a digital  
 pin, without using the delay() function.  This means that other code
 can run at the same time without being interrupted by the LED code.
 
 The circuit:
 * LED attached from pin 13 to ground.
 * Note: on most Arduinos, there is already an LED on the board
 that's attached to pin 13, so no hardware is needed for this example.
 
 
 created 2005
 by David A. Mellis
 modified 8 Feb 2010
 by Paul Stoffregen
 
 This example code is in the public domain.

 
 http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
 */

// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 1000;           // interval at which to blink (milliseconds)

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  // here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the 
  // difference between the current time and last time you blinked 
  // the LED is bigger than the interval at which you want to 
  // blink the LED.
  unsigned long currentMillis = millis();
  
  #define MAX_MESSAGE 50
  char* data;
  data = (char *)malloc(sizeof(char) * MAX_MESSAGE);
  
  uint8_t uint8_data_in = 255;
  data[0] = uint8_data_in;  
  uint8_t uint8_data_out = data[0];  
  if (uint8_data_in != uint8_data_out)
  {
    Serial.println("Invalid uint8_t communicated");
    Serial.println(uint8_data_in);
    Serial.println(uint8_data_out);
    Serial.println("");
  }
  
  uint16_t uint16_data_in = 65535;
  data[1] = highByte(uint16_data_in);
  data[2] = lowByte(uint16_data_in);
  uint16_t uint16_data_out = word(data[1], data[2]);  
  if (uint16_data_in != uint16_data_out)
  {
    Serial.println("Invalid uint16_t communicated");
    Serial.println(uint16_data_in);
    Serial.println(uint16_data_out);
    Serial.println("");
  }
  
  
  typedef struct
  {
    uint8_t uint8_data;
    uint16_t uint16_data;
  } struct_t;
  struct_t struct_data_in;
  struct_data_in.uint8_data = 255;
  struct_data_in.uint16_data = 65535;
  memcpy(&(data[4]), &struct_data_in, sizeof(struct_t));
  struct_t struct_data_out;
  memcpy(&struct_data_out, &(data[4]), sizeof(struct_t));
  if ((struct_data_in.uint8_data != struct_data_out.uint8_data)
      || (struct_data_in.uint16_data != struct_data_out.uint16_data))
  {
    Serial.println("Invalid struct_t communicated");
    Serial.println(struct_data_in.uint8_data);
    Serial.println(struct_data_out.uint8_data);
    Serial.println(struct_data_in.uint16_data);
    Serial.println(struct_data_out.uint16_data);
    Serial.println("");
  }

  
  free(data);
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}

