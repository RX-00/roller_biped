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

void Com::setSpd(int lspd, int rspd){
  l_spd = lspd;
  r_spd = rspd;
}

// parse and interpret time data
void Com::interpretTime(std::string line, char type){
  int us_prev = last_update_us;
  int sec_prev = sec_since_last_update;
  try{
    if (type == 'u') // set last update microseconds
      last_update_us = std::stoi(line);
    else if (type == 's') // set secs since last update
      sec_since_last_update = std::stoi(line);
  }
  catch(const std::invalid_argument& e){
    printf("ERR: RX data invalid arg\n");
  }
  catch(const std::out_of_range& e){
    printf("ERR: RX data out of range\n");
  }
}

// parse and interpret encoder data
void Com::interpretEncoder(std::string line, char pos){
  int l_enc_prev = l_encoder;
  int r_enc_prev = r_encoder;
  try {
    if (pos == 'l') // set left encoder (can be negative!)
      l_encoder = std::stoi(line);
    else if (pos == 'r') // set right encoder
      r_encoder = std::stoi(line);
  }
  catch (const std::invalid_argument& e){
    printf("ERR: RX data invalid arg\n");
  }
  catch (const std::out_of_range& e){
    printf("ERR: RX data out of range\n");
  }
}

void Com::interpretRXData(std::string RX_data){
  //NOTE: be careful about how fast this refresh really is...
  data_RX_vec.clear();

  std::stringstream ss(RX_data);
  std::string tmp_str;

  while(getline(ss, tmp_str, '\n')){
    data_RX_vec.push_back(tmp_str);
  }

  for (std::string line: data_RX_vec){
    if (line[0] == 'e'){
      // tokenize encoder data to [left and right]
      std::stringstream ss(line); // convert line into a string stream
      std::string tmp_str;
      std::vector<std::string> tokens;

      while(getline(ss, tmp_str, ';')){tokens.push_back(tmp_str);}

      //NOTE: check if these indices in the vector even exist first
      if (tokens.size() > 2){
        interpretEncoder(tokens[1], 'l');
        interpretEncoder(tokens[2], 'r');
      }
    }
    if (line[0] == 't'){
      // tokenize time data to [lastUpdateMicrosecs and secsSinceLastUpdate]
      std::stringstream ss(line); // convert line into a string stream
      std::string tmp_str;
      std::vector<std::string> tokens;

      while(getline(ss, tmp_str, ';')){tokens.push_back(tmp_str);}

      //NOTE: check if these indices in the vector even exist first
      if (tokens.size() > 2){
        interpretTime(tokens[1], 'u');
        interpretTime(tokens[2], 's');
      }
    }
  }
}
