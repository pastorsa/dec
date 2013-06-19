#include <ICSC.h>
#include <DEC.h>

DECInterface dec_interface;


/*!
 * @param source
 * @param command
 * @param length
 * @param data
 */
void receive_broadcast(unsigned char source, char command, unsigned char length, char *data)
{

  if (dec_interface.parse(source, command, length, data) != 0)
  {

  }
}

/*!
 */
void setup()
{

  ICSC.begin(DEC_CONTROLLER_ID, DEC_BAUD_RATE);
  ICSC.registerCommand('T', &receive_broadcast);


}

/*!
 */
void loop()
{
  // ICSC.send(2, ICSC_SYS_PING, 5, "PING");
  ICSC.process();
}




// Only used for development in eclipse
#ifdef ECLIPSE_ONLY
int main(void)
{
  return 0;
}
#endif
