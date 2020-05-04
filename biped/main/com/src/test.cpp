/* Test program for sending and receiving of data between the computer and microcontroller
 * also will parse and interpret the data RX. from the microcontroller
 *
 * NOTE: This is using UART Serial protocol at a baud rate of 115200 with default
 *       8N1 format, which is 8 data bits, no parity, 1 stop bit
 *
 * NOTE: data form for motor speed:
 *                 l(+/-)###r(+/-)###
 */

#include "Com.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "rs232.h"

int main(int argc, char** argv){
  std::cout << "Initiating coms..." << std::endl;
  Com comms;

  std::cout << "Sending dummy data to mega..." << std::endl;
  comms.TXData(50, -50);

  std::cout << "Got left encoder val: " << comms.getLeftEncoder() << std::endl;
  std::cout << "Got right encoder val:" << comms.getRightEncoder() << std::endl;
  std::cout << "Got usec time: " << comms.getTimeUsec() << std::endl;
  std::cout << "Got sec time: " << comms.getTimeSec() << std::endl;

  return 0;
}
