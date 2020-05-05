#ifndef COM_H
#define COM_H

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

class Com{
 private:
  int buf_size;
  int port_num;
  int baud_rate;
  char delim;
  char str_send[1][BUF_SIZE];
  unsigned char str_recv[BUF_SIZE];
  int l_spd;
  int r_spd;
  std::string data;
  int l_encoder;
  int r_encoder;
  long last_update_us;
  long sec_since_last_update;

 public:
  Com();
  Com(int BUF_SIZE, int PORT_NUM, int BAUDRATE);
  ~Com();

  int countDigit(int num);

  std::string formatIntToString(const int &input_spd);
  std::string formatData(const int &l_spd, const int &r_spd);

  void TXData(int l_spd, int r_spd);
  void RXData();
  void TX_RX_Data(int l_spd, int r_spd);

  void interpretTime(std::string line);
  void interpretEncoder(std::string line);

  int getLeftEncoder() const {return l_encoder;};
  int getRightEncoder() const {return r_encoder;};
  long getTimeUsec() const {return last_update_us;};
  long getTimeSec() const {return sec_since_last_update;};
  int getLSpd() const {return l_spd;};
  int getRSpd() const {return r_spd;};
};


#endif
