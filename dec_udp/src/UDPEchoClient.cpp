/*
 *   C++ sockets on Unix and Windows
 *   Copyright (C) 2002
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <dec_udp/PracticalSocket.h>      // For UDPSocket and SocketException
#include <iostream>               // For cout and cerr
#include <cstdlib>                // For atoi()
#include <string.h>

#include <sys/time.h>
#include <ctime>

using namespace std;

const int ECHOMAX = 1400; // Longest string to echo

int main(int argc, char *argv[])
{
  if ((argc < 3) || (argc > 4))
  { // Test for correct number of arguments
    cerr << "Usage: " << argv[0] << " <Server> <Echo String> [<Server Port>]\n";
    exit(1);
  }

  string servAddress = argv[1]; // First arg: server address
  char* echoString = argv[2]; // Second arg: string to echo
  int echoStringLen = strlen(echoString); // Length of string to echo
  if (echoStringLen > ECHOMAX)
  { // Check input length
    cerr << "Echo string too long" << endl;
    exit(1);
  }
  unsigned short echoServPort = Socket::resolveService((argc == 4) ? argv[3] : "echo", "udp");

  try
  {
    UDPSocket sock(1501);
    cout << "Created socket on port " << sock.getLocalPort() << endl;
    int respStringLen = 0; // Length of received response
    unsigned int num_packages = 100;

    cout << "start..." << endl;
    clock_t begin = clock();
		int counter = 0;
    for (unsigned int i = 0; i < num_packages; ++i)
    {
      // Send the string to the server
      sock.sendTo(echoString, echoStringLen, servAddress, echoServPort);

      // Receive a response
      char echoBuffer[ECHOMAX + 1]; // Buffer for echoed string + \0

      respStringLen = sock.recv(echoBuffer, ECHOMAX);
			counter++;

      echoBuffer[respStringLen] = '\0'; // Terminate the string!
      // cout << "Received " << respStringLen << " bytes : " << echoBuffer << endl; // Print the echoed arg
    }
    clock_t end = clock();
    cout << "...end" << endl;
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    cout << "Transmitting " << num_packages << " of size " << respStringLen << " took " << elapsed_secs << " seconds." << endl;
		cout << "Received " << counter << " packages." << endl;
    // Destructor closes the socket

  }
  catch (SocketException &e)
  {
    cerr << e.what() << endl;
    exit(1);
  }

  return 0;
}
