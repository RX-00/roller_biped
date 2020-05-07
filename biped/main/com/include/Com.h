#ifndef COM_H
#define COM_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <vector>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "rs232.h"

class Com{
 private:
  int buf_size;
  int port_num;
  int baud_rate;
  char delim;
  std::string delim_line;
  int l_spd;
  int r_spd;
  int l_encoder;
  int r_encoder;
  long last_update_us;
  long sec_since_last_update;

 public:
  Com();
  Com(int BUF_SIZE, int PORT_NUM, int BAUDRATE);
  ~Com();

  std::string data;
  std::vector<std::string> data_RX_vec;

  int countDigit(int num);

  std::string formatIntToString(const int &input_spd);
  std::string formatData();

  void interpretTime(std::string line);
  void interpretEncoder(std::string line);
  void setSpd(int lspd, int rspd);
  void interpretRXData(std::string RX_data);

  int getLeftEncoder() const {return l_encoder;};
  int getRightEncoder() const {return r_encoder;};
  long getTimeUsec() const {return last_update_us;};
  long getTimeSec() const {return sec_since_last_update;};
  int getLSpd() const {return l_spd;};
  int getRSpd() const {return r_spd;};
};


#endif
