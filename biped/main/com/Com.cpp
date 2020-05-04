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

// destructor
Com::~Com(){
  RS232_CloseComport(port_num);'
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
std::string Com::formatIntToString(const int &input_spd){
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
std::string Com::formatData(const int &l_spd, const int &r_spd){
  if(l_spd > 0){
    data = data + "l" + "+" + Com::formatIntToString(l_spd);
  } else{
    data = data + "l" + Com::formatIntToString(l_spd);
  }

  // delimiter
  data = data + ";";

  if(r_spd > 0){
    data = data + "r" + "+" + Com::formatIntToString(r_spd);
  } else{
    data = data + "r" + Com::formatIntToString(r_spd);
  }

  //  \n is how the mega differentiates each msg
  data = data + "\n";
  return data;
}

// transmit data
void Com::TXData(int l_spd, int r_spd){
  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port for TX\n");
  }

  usleep(250000); // wait for 250 milliseconds for stable connection NOTE: can it be shorter?
  data = Com::formatData(int l_spd, int r_spd); // set new data to send
  // then put it into str_send
  strcpy(str_send[0], data.c_str()); //.str( ) since strcpy only accepts const char*
  RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
  printf("Sent to mega: '%s' \n", str_send[0]);
  usleep(25000); // waits for 25ms
  RS232_CloseComport(port_num);
}

// receive and interpret data
void Com::RXData(){
  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port for RX\n");
  }

  // get chars from serial port (if any)
  int n = RS232_PollComport(port_num, str_recv, (int)buf_size);

  if (n > 0){
    str_recv[n] = 0; // always put a "null" at the end of a string
    printf("RX %i bytes: '%s'\n", n, (char *)str_recv);
    cout << "Test: \n" << (char *)str_recv << endl; // TODO: figure out if the str_recv buffer can be formatted into std::string for parsing

    if (){
      Com::interpretTime();
    }
    if (){
      Com::interpretEncoder();
    }
  }

  usleep(150000); // 150 ms, shortest is ~100 ms according to doc
  RS232_CloseComport(port_num);
}

// parse and interpret time data
void Com::interpretTime(std::string line){
  last_update_us = ;
  sec_since_last_update = ;
}

// parse and interpret encoder data
void Com::interpretEncoder(std::string line){
  l_encoder = ;
  r_encoder = ;
}
