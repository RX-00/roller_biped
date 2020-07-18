/*
 * This program is to test how fast the servos will
 * respond with this library vs. the python
 * implementation.
 */

#include <iostream>
#include <stdio.h>
#include <cmath>

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include <sys/time.h>

#include "RPMSerialInterface.h"

using namespace std;


// A utility class to provide cross-platform sleep and simple time methods
class Utils{
public:
	static void sleep( unsigned int _Milliseconds );
	static unsigned long long int getTickFrequency();
	static unsigned long long int getTimeAsTicks();
	static unsigned int getTimeAsMilliseconds();

private:
	static unsigned long long int mInitialTickCount;
};

// function to test device over serial w/ sinusoidal signals
void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber);

// Utils class implementation
void Utils::sleep( unsigned int _Milliseconds ){
#if _WIN32
	::Sleep( _Milliseconds );
#else
	struct timespec l_TimeSpec;
	l_TimeSpec.tv_sec = _Milliseconds / 1000;
	l_TimeSpec.tv_nsec = (_Milliseconds % 1000) * 1000000;
	struct timespec l_Ret;
	nanosleep(&l_TimeSpec,&l_Ret);
#endif
}

unsigned long long int Utils::getTickFrequency(){
#if _WIN32
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);
	return frequency.QuadPart;
#else
	// The gettimeofday function returns the time in microseconds. So it's frequency is 1,000,000.
	return 1000000;
#endif
}

unsigned long long int Utils::getTimeAsTicks(){
	unsigned long long int tickCount;
#if _WIN32
	LARGE_INTEGER l;
	QueryPerformanceCounter(&l);
	tickCount = l.QuadPart;
#else
	struct timeval p;
	gettimeofday(&p, NULL);	// Gets the time since the Epoch (00:00:00 UTC, January 1, 1970) in sec, and microsec
	tickCount = (p.tv_sec * 1000LL * 1000LL) + p.tv_usec;
#endif
	if ( mInitialTickCount==0xffffffffffffffffUL )
		mInitialTickCount = tickCount;
	tickCount -= mInitialTickCount;
	return tickCount;
}

unsigned int Utils::getTimeAsMilliseconds(){
	unsigned int millecondsTime = static_cast<unsigned int>( (getTimeAsTicks() * 1000) / getTickFrequency() );
	return millecondsTime;
}

unsigned long long int Utils::mInitialTickCount = 0xffffffffffffffffUL;


void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber){
  // Generate a sinusoid signal to send to the PololuInterface
  cout << "Sending sinusoidal signal to device to test device..." << endl;
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = 4000;
	const unsigned int channelMaxValue = 8000;
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
}


int main(int argc, char** argv){
  // create the interface for the maestro
  cout << "Serial interface init..." << endl;
  unsigned char deviceNumber = 12;
	unsigned char channelNumber = 1;

	std::string portName = "/dev/ttyACM0";

	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
      printf("Failed to create serial interface. %s\n", errorMessage.c_str());
      cout << "Terminating program..." << endl;
      return -1;
  }
  cout << "Serial interface initiated" << endl;

  sinusoid_signal(serialInterface, channelNumber);

  // TODO: FIND OUT THE RANGE OF THE SERVOS IN THIS INTERFACE
  int min = 4000;
  int max = 8000;
  int pos;
  // test each servo port at the "same time"
  printf("\n\nTesting all ports from min to max... \n");

  for (int i = 0; i < 5; i++){
    serialInterface -> setTargetCP(2, min);
    Utils::sleep(2000);
    serialInterface -> setTargetCP(2, max);
    Utils::sleep(2000);
  }

  for (int i = 0; i < 2; i++){
    if (i == 0)
      pos = min;
    else
      pos = max;
    serialInterface -> setTargetCP(0, pos);
    serialInterface -> setTargetCP(1, pos);
    serialInterface -> setTargetCP(2, pos);
    serialInterface -> setTargetCP(3, pos);
    serialInterface -> setTargetCP(4, pos);
    serialInterface -> setTargetCP(5, pos);
    serialInterface -> setTargetCP(6, pos);
    serialInterface -> setTargetCP(7, pos);
    serialInterface -> setTargetCP(8, pos);
    serialInterface -> setTargetCP(9, pos);
    serialInterface -> setTargetCP(10, pos);
    serialInterface -> setTargetCP(11, pos);
    serialInterface -> setTargetCP(12, pos);
    serialInterface -> setTargetCP(13, pos);
    serialInterface -> setTargetCP(14, pos);
    serialInterface -> setTargetCP(15, pos);
    serialInterface -> setTargetCP(16, pos);
    serialInterface -> setTargetCP(17, pos);

    Utils::sleep(1500);
  }

  Utils::sleep(1500);

  // test each servo port in sequence
  printf("Testing each servo port in sequence... \n");
  for (int port_num = 0; port_num < 18; port_num++){
    for (int i = 4000; i <= 8000; i = i + 100){
      printf("Testing port %d at pos: %d \n", port_num, i);
      serialInterface -> setTargetCP(port_num, i);
      Utils::sleep(300);
    }
  }

  // delete the interface
  cout << "Terminating serial interface..." << endl;
  delete serialInterface;
  serialInterface = NULL;

  return 0;
}
