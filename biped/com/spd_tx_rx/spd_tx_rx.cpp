/* Test program for serial communication from pi4 to arduino mega and vice versa
 * This program is mainly to test the sending and receiving of data between the
 * two devices along with proper organization and sorting of the data for onboard
 * calculations
 *
 * NOTE: This is using UART Serial protocol at a baud rate of 115200 with default
 *       8N1 format, which is 8 data bits, no parity, 1 stop bit
 *
 * NOTE: data form for motor speed:
 *                 l(+/-)###r(+/-)###
 */

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

// function to count digits
int countDigit(int num){
  int count = 0;
  while(num != 0){
    num = num / 10;
    count++;
  }
  return count;
}

// function to add 0's in so the resulting string will be the right size
// l+###;r-###
std::string formatIntToString(int input_spd){
  int spd = abs(input_spd);
  std::string spd_str;
  int digits = countDigit(spd);

  if (digits == 3){spd_str = std::to_string(spd);}
  else if (digits == 2){spd_str = "0" + std::to_string(spd);}
  else if (digits == 1){spd_str = "00" + std::to_string(spd);}

  if (input_spd < 0){spd_str = "-" + spd_str;}

  return spd_str; // should be a three digit number
}


int main(int argc, char** argv){
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit
  char str_send[1][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer

  int l_spd = 0;
  int r_spd = 0;
  std::string data = "";

  if (RS232_OpenComport(PORT_NUM, BAUDRATE, mode, 0)){ // 0 is no flowctrl
    printf("Cannot open port\n");
    return 0;
  }

  usleep(1000000); // usec -> 1000ms for stable condition TODO: check how short this can be

  for(int j = 0; j < 50; j++){

    data = ""; // clear up the data

    // psuedo testing as if speed was updated (0 - 255 due to PWM analog output)
    l_spd = rand() % (255 * 2 + 1) + (-255); // this is to include -255 to 255
    r_spd = rand() % (255 * 2 + 1) + (-255);

    if(l_spd > 0){
      data = data + "l" + "+" + formatIntToString(l_spd);
    }
    else{
      data = data + "l" + formatIntToString(l_spd);
    }

    //delimiter
    data = data + ";";

    if(r_spd > 0){
      data = data + "r" + "+" + formatIntToString(r_spd);
    }
    else{
      data = data + "r" + formatIntToString(r_spd);
    }

    // make sure you're sending the data with \n since that's how the mega differentiates each msg
    data = data + "\n";

    // then put it into str_send
    strcpy(str_send[0], data.c_str()); //.str() since strcpy only accepts const char*

    RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
    printf("Sent to mega: '%s' \n", str_send[0]);
    usleep(500000); // waits for reply 500ms

    // gets chars from serial port (if any)
    int n = RS232_PollComport(PORT_NUM, str_recv, (int)BUF_SIZE);
    if(n > 0){
      str_recv[n] = 0; // always put a "null" at the end of a string
      printf("Received %i bytes: '%s'\n", n, (char *)str_recv);

      // TODO: Figure out how to parse the data from the mega
    }

    usleep(500000); // sleep for 500ms
  }

  RS232_CloseComport(PORT_NUM);

  return 0;
}
