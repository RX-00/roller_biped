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
#include <sstream>
#include <regex>
#include <algorithm>
#include <iterator>

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
  delim_line = "\n";
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
std::string Com::formatData(){
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

void Com::setSpd(int lspd, int rspd){
  l_spd = lspd;
  r_spd = rspd;
}

void Com::interpretRXData(std::string RX_data){
  //NOTE: be careful about how fast this refresh really is...
  //printf("RX data: \n%s \n", RX_data.c_str());

  data_RX_vec.clear();

  std::regex pattern(delim_line);
  std::copy(std::sregex_token_iterator(RX_data.begin(), RX_data.end(), pattern, -1),
            std::sregex_token_iterator(), std::back_inserter(data_RX_vec));


  for (std::string line: data_RX_vec){
    std::cout << line;
    // if line starts with an e then interpretEncoder()
    // if line starts with an t then interpretTime()
  }
  //std::cout << "Processed Data: \n" << data_RX_vec[1] << std::endl;

  // interpret data inside
  interpretTime(RX_data);
  interpretEncoder(RX_data);

}
