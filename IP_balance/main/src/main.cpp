/* Test program for sending and receiving of data between the computer and microcontroller
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
#include <chrono>
#include <thread>

#include "rs232.h"              // serial TX RX
#include "RTIMULib.h"           // IMU
#include "RPMSerialInterface.h" // servos

#define BUF_SIZE 128
#define PORT_NUM 24 // NOTE: using ttyACM0 -> port number 24
#define BAUDRATE 115200
#define TX_TIME    1500 // usec -> 1.5 sec for stable condition
#define RX_TIME    100  // waits for reply 100ms
#define CYCLE_TIME 100  // sleep for 100ms



int main(int argc, char** argv){

  // Setup IMU
  int sampleCount = 0;
  int sampleRate = 0;
  uint64_t rateTimer;
  uint64_t displayTimer;
  uint64_t now;
  RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
  RTIMU *imu = RTIMU::createIMU(settings);

  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
    printf("No IMU found\n");
    exit(1);
  }

  imu->IMUInit();
  // this is a convenient place to change fusion parameters
  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);
  // set up for rate timer
  rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();


  // Setup comms
  Com comms(BUF_SIZE, PORT_NUM, BAUDRATE);
  char mode[] = {'8', 'N', '1', 0}; // 8 data bits, no parity, 1 stop bit
  char str_send[1][BUF_SIZE];
  unsigned char str_recv[BUF_SIZE];

  if (RS232_OpenComport(PORT_NUM, BAUDRATE, mode, 0)){ // 0 is no flowctrl
    printf("Cannot open port\n");
    return 0;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(TX_TIME));


  // Main loop
  for(int j = 0; j < 10; j++){
    // dummy velocity data
    comms.setSpd((rand() % (255 * 2 + 1) + (-255)), (rand() % (255 * 2 + 1) + (-255)));
    comms.formatData();
    strcpy(str_send[0], comms.data.c_str());
    RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial
    printf("Sent to mega: %s \n", str_send[0]);

    std::this_thread::sleep_for(std::chrono::milliseconds(RX_TIME));

    int n = RS232_PollComport(PORT_NUM, str_recv, (int)BUF_SIZE);
    if(n > 0){
      str_recv[n] = 0; // always put a "null" at the end of a string
      printf("RX: %s\n", (char *)str_recv);
      std::string RX_data((char *)str_recv);
      comms.interpretRXData(RX_data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(CYCLE_TIME));
  }

  RS232_CloseComport(PORT_NUM);

  std::cout << "\n\nGot left encoder val: " << comms.getLeftEncoder() << std::endl;
  std::cout << "Got right encoder val:" << comms.getRightEncoder() << std::endl;
  std::cout << "Got usec time: " << comms.getTimeUsec() << std::endl;
  std::cout << "Got sec time: " << comms.getTimeSec() << std::endl;
  std::cout << "Sent left speed: " << comms.getLSpd() << std::endl;
  std::cout << "RX left speed: " << comms.getLSpdRX() << std::endl;

  return 0;
}
