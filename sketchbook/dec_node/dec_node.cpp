#include <ICSC.h>
#include <DEC.h>
#include <Adafruit_NeoPixel.h>

/*! Custom variable
 * DEC_NODE_ID must be within [1, DEC_NUM_NODES]
 */
static const uint8_t DEC_NODE_ID = 1;

/*!
 */
DECInterface dec_interface;

/*!
 * @param source Sender id
 * @param command
 * @param length
 * @param data
 */
void receive_broadcast(unsigned char source, char command, unsigned char length, char *data)
{
  if (command == DEC_SETUP_DATA)
  {
    // parse data into local memory
    dec_interface.parseSetupData(data);
    //

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
    {
      // dec_interface.getData(dec_interface.length_, dec_interface.data_entries_);
      // ICSC.broadcast(DEC_DATA, dec_interface.length_, dec_interface.data_entries_);
    }

    // Adafruit_NeoPixel test(13);
  }

  else if (command == DEC_SENSOR_DATA)
  {
    // parse data into local memory
    dec_interface.parseSensorData(source, data);
    //

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
    {
      // dec_interface.getData(dec_interface.length_, dec_interface.data_entries_);
      // ICSC.broadcast(DEC_DATA, dec_interface.length_, dec_interface.data_entries_);
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
      // dec_interface.getData(dec_interface.length_, dec_interface.data_entries_);
      // ICSC.broadcast(DEC_DATA, dec_interface.length_, dec_interface.data_entries_);
    }
  }
}

void run_test()
{

  // fill setup data
  dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips = 3;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].num_leds = 10;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].pin = 1;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].num_leds = 11;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].pin = 2;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].num_leds = 12;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].pin = 3;
  dec_interface.setup_data_[DEC_NODE_ID]->num_sensors = 3;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[0].pin = 4;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[1].pin = 5;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[2].pin = 6;

  // generate message
  dec_interface.generateSetupData(DEC_NODE_ID);
  // message being send... and received...

  // zero out local memory for testing
  dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].num_leds = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].pin = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].num_leds = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].pin = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].num_leds = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].pin = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->num_sensors = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[0].pin = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[1].pin = 0;
  dec_interface.setup_data_[DEC_NODE_ID]->sensors[2].pin = 0;

  // parse message
  dec_interface.parseSetupData(dec_interface.data_);

  // test
  if ((dec_interface.setup_data_[DEC_NODE_ID]->num_led_strips != 3)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].num_leds != 10)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[0].pin != 1)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].num_leds != 11)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[1].pin != 2)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].num_leds != 12)
   || (dec_interface.setup_data_[DEC_NODE_ID]->led_strips[2].pin != 3)
   || (dec_interface.setup_data_[DEC_NODE_ID]->num_sensors != 3)
   || (dec_interface.setup_data_[DEC_NODE_ID]->sensors[0].pin != 4)
   || (dec_interface.setup_data_[DEC_NODE_ID]->sensors[1].pin != 5)
   || (dec_interface.setup_data_[DEC_NODE_ID]->sensors[2].pin != 6))
  {
    Serial.prin
  }

}


/*!
 */
void setup()
{
  /*
  ICSC.begin(DEC_NODE_ID, DEC_BAUD_RATE);
  // Make sure that the size of our message obeys the limits
  if(sizeof(DECInterface::data_entry_t) > MAX_MESSAGE)
  {
    // BREAK OR SOMTHING
  }
  ICSC.registerCommand(ICSC_BROADCAST, &receive_broadcast);
  */
}

/*!
 */
void loop()
{
  // ICSC.process();
  run_test();

}


// Only used for development in eclipse
#ifdef ECLIPSE_ONLY
int main(void)
{
  return 0;
}
#endif
