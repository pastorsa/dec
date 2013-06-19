#include <ICSC.h>
#include <DEC.h>

#include <FreeMemory.h>

/*! Custom variable
 * DEC_NODE_ID must be within [1, DEC_NUM_NODES]
 */
static const uint8_t DEC_NODE_ID = 1;

static const uint8_t num_led_strips = 4;
static const uint8_t num_sensors = 4;
static const uint64_t level = 7238374;

/*!
 */
DECInterface dec_interface;

// constants won't change. Used here to
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long interval = 1000;           // interval at which to blink (milliseconds)

boolean test_setup_data()
{
  boolean error = false;
  if (!error && dec_interface.token_ != DEC_NODE_ID)
    error = true;
  if (!error && dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips != num_led_strips)
    error = true;
  for (uint8_t i = 0; !error && i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    if (!error && dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds != uint16_t(1 + i))
      error = true;
    if (!error && dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].pin != 1 + i)
      error = true;
  }
  if (!error && dec_interface.setup_data_[DEC_NODE_ID]->num_sensors != num_sensors)
    error = true;
  for (uint8_t i = 0; !error && i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    if (!error && dec_interface.setup_data_[DEC_NODE_ID]->sensors[i].pin != 1 + i)
      error = true;
  }
  return error;
}

void print_setup_data()
{
  Serial.println("Setup data.");
  Serial.print("Token: ");
  Serial.println(dec_interface.token_);
  Serial.print("num_led_strips: ");
  Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips);
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    Serial.print(dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds);
    Serial.print(" ");
    Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].pin);
  }
  Serial.print("num_sensors: ");
  Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->num_sensors);
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->sensors[i].pin);
  }
}

void run_setup_data_test()
{
  // fill setup data
  dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips = num_led_strips;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds = 1 + i;
    dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].pin = 1 + i;
  }
  dec_interface.setup_data_[DEC_NODE_ID]->num_sensors = num_sensors;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    dec_interface.setup_data_[DEC_NODE_ID]->sensors[i].pin = 1 + i;
  }

  // generate message
  if(!dec_interface.generateSetupData(DEC_NODE_ID))
  {
    Serial.println("FAIL: Problem when generating setup data. Wrong input.");
    print_setup_data();
  }

  // message being send... and received...

  // zero out local memory for testing
  dec_interface.token_ = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips = 0;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds =0;
    dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].pin = 0;
  }
  dec_interface.setup_data_[DEC_NODE_ID]->num_sensors = 0;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    dec_interface.setup_data_[DEC_NODE_ID]->sensors[i].pin = 0;
  }

  // parse message
  dec_interface.parseSetupData(dec_interface.data_);

  // test
  if (test_setup_data())
  {
    Serial.println("Wrong setup message received.");
    print_setup_data();
  }
}

boolean test_sensor_data()
{
  boolean error = false;
  if (!error && dec_interface.token_ != DEC_NODE_ID)
    error = true;
  if (!error && dec_interface.sensor_data_[DEC_NODE_ID]->level != level)
    error = true;
  for (uint8_t i = 0; !error && i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    if (!error && dec_interface.sensor_data_[DEC_NODE_ID]->sensors[i] != uint16_t(1 + i))
      error = true;
  }
  return error;
}

void print_sensor_data()
{
  Serial.println("Sensor data.");
  Serial.print("Token: ");
  Serial.println(dec_interface.token_);
  Serial.print("num_sensors: ");
  Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->num_sensors);
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    Serial.println(dec_interface.sensor_data_[DEC_NODE_ID]->sensors[i]);
  }
}

void run_sensor_data_test()
{
  // fill setup data
  dec_interface.sensor_data_[DEC_NODE_ID]->level = level;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    dec_interface.sensor_data_[DEC_NODE_ID]->sensors[i] = uint16_t(1 + i);
  }

  // generate message
  dec_interface.generateSensorData(DEC_NODE_ID);
  // message being send... and received...

  // zero out local memory for testing
  dec_interface.token_ = 0;
  dec_interface.sensor_data_[DEC_NODE_ID]->level = 0;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_sensors; ++i)
  {
    dec_interface.sensor_data_[DEC_NODE_ID]->sensors[i] = 0;
  }

  // parse message
  dec_interface.parseSensorData(DEC_NODE_ID, dec_interface.data_);

  // test
  if (test_sensor_data())
  {
    Serial.println("Wrong sensor message received.");
    print_sensor_data();
  }
}

boolean test_light_data()
{
  boolean error = false;
  uint32_t color = 0;
  for (uint8_t i = 0; !error && i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    for (uint16_t j = 0; !error && j < dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds; ++j)
    {
      if (!error && dec_interface.light_data_[DEC_NODE_ID]->led_strips[i].colors[j] != color++)
        error = true;
    }
  }
  return error;
}

void print_light_data()
{
  Serial.println("Light data.");
  Serial.print("Token: ");
  Serial.println(dec_interface.token_);
  Serial.print("num_led_strips: ");
  Serial.println(dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips);

  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    for (uint16_t j = 0; j < dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds; ++j)
    {
      Serial.print(dec_interface.light_data_[DEC_NODE_ID]->led_strips[i].colors[j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

void run_light_data_test()
{
  // fill setup data
  uint32_t color = 0;
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    for (uint16_t j = 0; j < dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds; ++j)
    {
      dec_interface.light_data_[DEC_NODE_ID]->led_strips[i].colors[j] = color++;
    }
  }

  // generate message
  dec_interface.generateLightData(DEC_NODE_ID);
  // message being send... and received...

  // zero out local memory for testing
  for (uint8_t i = 0; i < dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips; ++i)
  {
    for (uint16_t j = 0; j < dec_interface.setup_data_[DEC_NODE_ID]->led_strips[i].num_leds; ++j)
    {
      dec_interface.light_data_[DEC_NODE_ID]->led_strips[i].colors[j] = 0;
    }
  }

  // parse message
  dec_interface.parseLightData(DEC_NODE_ID, dec_interface.data_);

  // test
  if (test_light_data())
  {
    Serial.println("Wrong light message received.");
    print_light_data();
  }
}

// this function will return the number of bytes currently free in RAM
int memoryTest() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }

  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}

/*!
 */
void setup()
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
// }

//void setup()
//{
//  ////// set up and blink the status LED once////////
//  pinMode(13,OUTPUT); // set up the status LED
//  digitalWrite(13, HIGH); // turn on the status LED
//  delay(20); // wait so the blink is visitlbe
//  digitalWrite(13, LOW); // turn off the status LED
//  //////////////////////////////////////////////////

}

/*!
 */
void loop()
{
  /*
  Serial.println("freeMemory()");
  Serial.println(freeMemory());

  // run the memory test function and print the results to the serial port
  int result = memoryTest();
  Serial.print("Memory test results: ");
  Serial.print(result,DEC);
  Serial.println(" bytes free.");
  */

  run_setup_data_test();
  // print_setup_data();

  if(!test_setup_data())
  {
    run_sensor_data_test();
    run_light_data_test();
  }
  else
  {
    Serial.println("FAIL: test_setup_data FAILED");
  }

  unsigned long currentMillis = millis();
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


// Only used for development in eclipse
#ifdef ECLIPSE_ONLY
int main(void)
{
  return 0;
}
#endif
