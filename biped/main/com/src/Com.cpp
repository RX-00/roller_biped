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

// constructor
Com::Com(){
  buf_size = 128;
  port_num = 24; // default ttyACM0
  baud_rate = 115200;
  delim = ';';
  char str_send[1][buf_size]; // send data buffer
  unsigned char str_recv[buf_size]; // recv data buffer
  l_spd = 0;
  r_spd = 0;
  data = "";
  l_encoder = 0;
  r_encoder = 0;
  last_update_us = 0;
  sec_since_last_update = 0;
}

// constructor
Com::Com(int BUF_SIZE, int PORT_NUM, int BAUDRATE){
  buf_size = BUF_SIZE;
  port_num = PORT_NUM;
  baud_rate = BAUDRATE;
  delim = ';';
  char str_send[1][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  l_spd = 0;
  r_spd = 0;
  data = "";
  l_encoder = 0;
  r_encoder = 0;
  last_update_us = 0;
  sec_since_last_update = 0;
}

// destructor
Com::~Com(){
  //RS232_CloseComport(port_num);
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

// transmit data NOTE: broken
void Com::TXData(int l_spd, int r_spd){
  //NOTE: single flexible member arrays from C are tricky, that's why I declare it each time it's needed
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit

  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port for TX\n");
  }

  usleep(150000); // wait for 100 milliseconds for stable connection
  data = Com::formatData(l_spd, r_spd); // set new data to send
  // then put it into str_send
  strcpy(str_send[0], data.c_str()); //.str( ) since strcpy only accepts const char*
  RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
  printf("Sent to mega: '%s' \n", str_send[0]);
  usleep(150000);
  RS232_CloseComport(port_num);
}

// receive and interpret data NOTE: broken
void Com::RXData(){
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit

  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port for RX\n");
  }

  usleep(500000);
  // get chars from serial port (if any)
  int n = RS232_PollComport(port_num, str_recv, (int)buf_size);

  if (n > 0){
    str_recv[n] = 0; // always put a "null" at the end of a string
    printf("RX %i bytes: '%s'\n", n, (char *)str_recv);
    std::cout << "Test: \n" << (char *)str_recv << std::endl; // TODO: figure out if the str_recv buffer can be formatted into std::string for parsing

    std::string place_holder = "test";

    if (true){
      Com::interpretTime(place_holder);
    }
    if (true){
      Com::interpretEncoder(place_holder);
    }
  }

  usleep(500000); // shortest (safe) wait time is ~100 ms according to doc
  RS232_CloseComport(port_num);
}

// transmit and then read in data
void Com::TX_RX_Data(int l_spd, int r_spd){
  // transmit data
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit

  if (RS232_OpenComport(port_num, baud_rate, mode, 0)){ // 0 is no flowctrl
    printf("ERROR: Cannot open port for TX\n");
  }

  usleep(500000); // wait for 500 milliseconds for stable connection
  data = Com::formatData(l_spd, r_spd); // set new data to send
  // then put it into str_send
  strcpy(str_send[0], data.c_str()); //.str( ) since strcpy only accepts const char*
  RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
  printf("Sent to mega: '%s' \n", str_send[0]);
  usleep(500000);

  // receive data
  int n = RS232_PollComport(port_num, str_recv, (int)buf_size);

  if (n > 0){
    str_recv[n] = 0; // always put a "null" at the end of a string
    printf("RX %i bytes: '%s'\n", n, (char *)str_recv);
  }

  std::string place_holder = "test";
  if (true){
    Com::interpretTime(place_holder);
  }
  if (true){
    Com::interpretEncoder(place_holder);
  }
  usleep(500000); // shortest (safe) wait time is ~100 ms according to doc
  RS232_CloseComport(port_num);
}

// parse and interpret time data
void Com::interpretTime(std::string line){
  last_update_us = 0;
  sec_since_last_update = 0;
}

// parse and interpret encoder data
void Com::interpretEncoder(std::string line){
  l_encoder = 0;
  r_encoder = 0;
}
