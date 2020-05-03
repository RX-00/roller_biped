/* Class for sending and receiving of data between the computer and microcontroller
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

// constructor
Com::Com(){
  buf_size = BUF_SIZE;
  port_num = PORT_NUM;
  baud_rate = BAUDRATE;
  delim = ";";
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit
  char str_send[1][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  std::string data = "";
}

// count how many digits are in num
int Com::countDigit(int num){
  int count = 0;
  while (num != 0){
    num = num / 10;
    count++;
  }
  return count;
}

// format the input into the form -### for the data transfer
std::string Com::formatIntToString(int input_spd){
  int spd = abs(input_spd);
  std::string spd_str;
  int digits = countDigit(spd);

  if (digits == 3){
    spd_str = std::to_string(spd);
  }
  else if (digits == 2){
    spd_str = "0" + std::to_string(spd);
  }
  else if (digits == 1){
    spd_str = "00" + std::to_string(spd);
  }

  if (input_spd < 0){
    spd_str = "-" + spd_str;
  }

  return spd_str; // should be a three digit number
}

// format the parts into the proper form w/ delimiter
std::string Com::formatData(std::string l_spd, std::string r_spd){
  std::string data = "";
  data = l_spd + delim + r_spd;
  return data;
}

void Com::TXData(std::string data){
  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port\n");
  }

  usleep(250000); // wait for 250 milliseconds for stable connection NOTE: can it be shorter?

  
}

void Com::RXData(){

}

long Com::interpretTime(){
  
}

long Com::interpretEncoder(){
  
}
