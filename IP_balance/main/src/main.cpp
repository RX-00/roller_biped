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
#include "Utils.h"              // servo utility class for sleep & time methods

#define BUF_SIZE   128
#define PORT_NUM   24 // NOTE: using ttyACM0 -> port number 24
#define BAUDRATE   115200
#define TX_TIME    1500 // usec -> 1.5 sec for stable condition
#define RX_TIME    100  // waits for reply 100ms
#define CYCLE_TIME 100  // sleep for 100ms

//NOTE: modified the RPM library to allow for greater servo range
#define SRVO_MAX   10000
#define SRVO_MIN   2000
#define LEFT_HIP   1
#define RIGHT_HIP  0
#define LEFT_KNEE  3
#define RIGHT_KNEE 2


//TODO: test that class for delaying the serial interface

// default crouching position for the robot
void default_pos(RPM::SerialInterface *serialInterface){
  std::cout << "Moving into default position in 1 seconds..." << std::endl;
  Utils::sleep(1000);
  serialInterface -> setTargetCP(LEFT_HIP, 3000);  // LOWER  (back)
  serialInterface -> setTargetCP(RIGHT_HIP, 9000); // HIGHER (back)
  serialInterface -> setTargetCP(LEFT_KNEE, SRVO_MIN);
  serialInterface -> setTargetCP(RIGHT_KNEE, SRVO_MAX);
  Utils::sleep(1500);
}

void servo_test(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  std::cout << "Testing servo number: " << 11 << std::endl;
  for (int i = 0; i < 5; i++){
    std::cout << "min position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MIN);
    Utils::sleep(1000);
    std::cout << "middle position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, 6000);
    Utils::sleep(1000);
    std::cout << "max position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MAX);
    Utils::sleep(1000);
  }
  exit(1);
}

// function to test device over serial w/ sinusoidal signals
void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber){
  // Generate a sinusoid signal to send to the PololuInterface
  std::cout << "Sending sinusoidal signal to device to test device..." << std::endl;
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = SRVO_MIN;
	const unsigned int channelMaxValue = SRVO_MAX;
	const unsigned int channelValueRange = channelMaxValue - channelMinValue;
	const unsigned int signalPeriodInMs = 2000;
	unsigned int time0 = Utils::getTimeAsMilliseconds();
	unsigned int timeSinceStart = 0;
	while ( timeSinceStart < 5000 ){
    float k = sin( (pi*2)/signalPeriodInMs * timeSinceStart ) * (float)(channelValueRange/2);
    float channelValue = (float)channelMinValue + (float)channelValueRange/2 + k;
    printf("\rchannelValue=%d", (unsigned int)channelValue );
    serialInterface->setTargetCP( channelNumber, (unsigned short)channelValue );
    timeSinceStart = Utils::getTimeAsMilliseconds() - time0;
    Utils::sleep(5);
  }
  printf("\n");
}

// function to create serial interface for the maestro servo controller
RPM::SerialInterface * serialInterfaceInit(unsigned char deviceNumber, unsigned char channelNumber, std::string portName){
  // create the interface for the maestro
  std::cout << "Serial interface init..." << std::endl;
	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
    printf("Failed to create serial interface. %s\n", errorMessage.c_str());
    std::cout << "Terminating program..." << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << "Serial interface initiated\n" << std::endl;
  return serialInterface;
}


int main(int argc, char** argv){
  // Serial servo interface
  unsigned char deviceNumber = 12;
	unsigned char channelNumber = 11;
  std::string portName = "/dev/ttyACM1";
  RPM::SerialInterface *servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);
  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;

  //sinusoid_signal(servosInterface, channelNumber);
  //servo_test(servosInterface, channelNumber);

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
  std::cout << "Opening serial port...\n\n" << std::endl;
  if (RS232_OpenComport(PORT_NUM, BAUDRATE, mode, 0)){ // 0 is no flowctrl
    printf("Cannot open port\n");
    return 0;
  }
  usleep(500000 * 2); // usec -> 500ms for stable condition
  //std::this_thread::sleep_for(std::chrono::milliseconds(TX_TIME));

  //setup default leg position
  default_pos(servosInterface);

  // Main loop
  for(int j = 0; j < 10; j++){
    // dummy velocity data
    comms.setSpd((rand() % (255 * 2 + 1) + (-255)), (rand() % (255 * 2 + 1) + (-255)));
    comms.formatData();
    strcpy(str_send[0], comms.data.c_str());
    RS232_cputs(PORT_NUM, str_send[0]); // sends string on serial

    usleep(100000); // waits for reply 100ms
    //std::this_thread::sleep_for(std::chrono::milliseconds(RX_TIME));

    int n = RS232_PollComport(PORT_NUM, str_recv, (int)BUF_SIZE);
    if(n > 0){
      str_recv[n] = 0; // always put a "null" at the end of a string
      //printf("RX: %s\n", (char *)str_recv);
      std::string RX_data((char *)str_recv);
      comms.interpretRXData(RX_data);
    }
    usleep(100000 * 5); // sleep for 100ms
    //std::this_thread::sleep_for(std::chrono::milliseconds(CYCLE_TIME));
  }

  RS232_CloseComport(PORT_NUM);
  delete servosInterface;
  servosInterface = NULL;

  std::cout << "\n\nGot left encoder val: " << comms.getLeftEncoder() << std::endl;
  std::cout << "Got right encoder val:" << comms.getRightEncoder() << std::endl;
  std::cout << "Got usec time: " << comms.getTimeUsec() << std::endl;
  std::cout << "Got sec time: " << comms.getTimeSec() << std::endl;
  std::cout << "Sent left speed: " << comms.getLSpd() << std::endl;
  std::cout << "RX left speed: " << comms.getLSpdRX() << std::endl;

  return 0;
}
