#include <ICSC.h>
#include <DEC.h>

// include generated file
#include "dec_structure.h"

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

void setSetupData(uint8_t node_id, uint8_t num_led_strips,
                  uint16_t* num_leds_for_each_strip,
                  uint8_t* led_pins,
                  uint8_t num_sensors,
                  uint8_t* sensor_pins)
{
  dec_interface.setup_data_[node_id]->num_led_strips = num_led_strips;
  for (uint8_t i = 0; i < dec_interface.setup_data_[node_id]->num_led_strips; ++i)
  {
    dec_interface.setup_data_[node_id]->led_strips[i].num_leds = num_leds_for_each_strip[i];
    dec_interface.setup_data_[node_id]->led_strips[i].pin = led_pins[i];
  }

  dec_interface.setup_data_[node_id]->num_sensors = num_sensors;
  for (uint8_t i = 0; i < dec_interface.setup_data_[node_id]->num_sensors; ++i)
  {
    dec_interface.setup_data_[node_id]->sensors[i].pin = sensor_pins[i];
  }
}

void loadSetupData()
{
  // for now all nodes are configured in the same way
  uint8_t num_led_strips = 3;
  uint16_t num_leds_for_each_strip[] = {1, 2, 1};
  uint8_t led_pins[] = {1, 2, 3};

  uint8_t num_sensors = 3;
  uint8_t sensor_pins[] = {4, 5, 6};

  for (uint8_t node_id = 0; node_id < DEC_NUM_NODES; ++node_id)
  {
    setSetupData(node_id, num_led_strips, num_leds_for_each_strip, led_pins, num_sensors, sensor_pins);
  }
}

/*!
 */
void setup()
{
  // setup led blinking for debugging
  pinMode(led_pin, OUTPUT);

  // setup communication
  ICSC.begin(DEC_CONTROLLER_ID, DEC_BAUD_RATE);
  ICSC.registerCommand(ICSC_BROADCAST, &receive_broadcast);

  loadSetupData();
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
