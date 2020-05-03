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

class Com{
 private:
  int buf_size;
  int port_num;
  int baud_rate;
  char delim;
  char mode[];
  char str_send[][];
  unsigned char str_recv[];
  std::string data;

 public:
  Com();

  int countDigit(int num);

  std::string formatIntToString(int input_spd);
  std::string formatData(std::string l_spd, std::string r_spd);

  void TXData(std::string data);
  void RXData();

  long interpretTime();
  long interpretEncoder();
  //int interpretSpeed();

};


#endif
