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

#define BUF_SIZE 128
#define PORT_NUM 24 // NOTE: using ttyACM0 -> port number 24
#define BAUDRATE 115200


int main(int argc, char** argv){
  Com comms(BUF_SIZE, PORT_NUM, BAUDRATE);
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit
  char str_send[1][BUF_SIZE];
  unsigned char str_recv[BUF_SIZE];

  if (RS232_OpenComport(PORT_NUM, BAUDRATE, mode, 0)){ // 0 is no flowctrl
    printf("Cannot open port\n");
    return 0;
  }

  usleep(500000); // usec -> 500ms for stable condition

  for(int j = 0; j < 10; j++){
    // dummy velocity data
    comms.setSpd((rand() % (255 * 2 + 1) + (-255)), (rand() % (255 * 2 + 1) + (-255)));
    comms.formatData();

    // then put it into str_send
    strcpy(str_send[0], comms.data.c_str());

    RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
    //printf("Sent to mega: '%s' \n", str_send[0]);
    usleep(100000); // waits for reply 100ms

    // gets chars from serial port (if any)
    int n = RS232_PollComport(PORT_NUM, str_recv, (int)BUF_SIZE);
    if(n > 0){
      str_recv[n] = 0; // always put a "null" at the end of a string
      std::string RX_data((char *)str_recv);
      comms.interpretRXData(RX_data);
    }
    usleep(100000); // sleep for 100ms
  }

  RS232_CloseComport(PORT_NUM);

  std::cout << "\n\nGot left encoder val: " << comms.getLeftEncoder() << std::endl;
  std::cout << "Got right encoder val:" << comms.getRightEncoder() << std::endl;
  std::cout << "Got usec time: " << comms.getTimeUsec() << std::endl;
  std::cout << "Got sec time: " << comms.getTimeSec() << std::endl;

  return 0;
}
