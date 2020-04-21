/* Test program for serial communication from pi4 to arduino mega and vice versa
 * TODO: double check if the mega can/should handle floating point calculations
 *
 * NOTE: This is using UART Serial protocol at a baud rate of 115200 with default
 *       8N1 format, which is 8 data bits, no parity, 1 stop bit
 *
 */

#include <iostream>
#include <stdio.h>
#include <cmath>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "rs232.h"


#define BUF_SIZE 128
#define PORT_NUM 24 // NOTE: using ttyACM0 -> port number 24
#define BAUDRATE 115200


int main(int argc, char** argv){ // don't forget that argc stands for argument count & argv stands for argument vector lol

  int i = 0;

  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit
  char str_send[2][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer

  strcpy(str_send[0], "This is a test string");
  strcpy(str_send[1], "This is a another test string");

  if(RS232_OpenComport(PORT_NUM, BAUDRATE, mode, 0)){ // 0 is no flowctrl
    printf("Cannot open port\n");
    return 0;
  }

  usleep(2000000); // usec -> 2000ms for stable condition

  for(int j = 0; j < 10; j++){
    RS232_cputs(PORT_NUM, str_send[i]); // sends string on serial
    printf("Sent to Arduino: '%s' \n", str_send[i]);
    usleep(500000); // waits for reply 500ms
    int n = RS232_PollComport(PORT_NUM, str_recv, (int)BUF_SIZE);
    if(n > 0){
      str_recv[n] = 0; // always put a "null" at the end of a string
      printf("Received %i bytes: '%s'\n", n, (char *)str_recv);
    }
    i++;
    i %= 2;
    usleep(100000); // sleep for 1000ms
  }

  RS232_CloseComport(PORT_NUM);

  return 0;
}
