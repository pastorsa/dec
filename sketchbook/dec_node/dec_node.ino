#include <ICSC.h>
#include <DEC.h>
#include <Adafruit_NeoPixel.h>

/*! Custom variable
 * DEC_NODE_ID must be within [1, DEC_NUM_NODES]
 */
#define DEC_NODE_ID 1

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
  if (command == DEC_INITIALIZATION)
  {

  }
  else if (command == DEC_SETUP)
  {

  }
  else if (command == DEC_DATA)
  {
    // parse data into local memory
    dec_interface.parseDataEntry(source, command, length, data);
    //

    // check whether it's my turn to broadcast my data
    if (dec_interface.token_ == DEC_NODE_ID)
    {
      // dec_interface.getData(dec_interface.length_, dec_interface.data_entries_);
      // ICSC.broadcast(DEC_DATA, dec_interface.length_, dec_interface.data_entries_);
    }
  }
}

/*!
 */
void setup()
{
  ICSC.begin(DEC_NODE_ID, DEC_BAUD_RATE);

  // Make sure that the size of our message obeys the limits
  if(sizeof(DECInterface::data_entry_t) > MAX_MESSAGE)
  {
    // BREAK OR SOMTHING
  }

  ICSC.registerCommand(ICSC_BROADCAST, &receive_broadcast);


}

/*!
 */
void loop()
{
  ICSC.process();



}




// Only used for development in eclipse
#ifdef ECLIPSE_ONLY
int main(void)
{
  return 0;
}
#endif
