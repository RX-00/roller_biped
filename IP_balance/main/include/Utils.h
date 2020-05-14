#ifndef UTILS_H
#define UTILS_H

class Utils{
 public:
  static void sleep(unsigned int _Milliseconds);
	static unsigned long long int getTickFrequency();
	static unsigned long long int getTimeAsTicks();
	static unsigned int getTimeAsMilliseconds();
 private:
  static unsigned long long int mInitialTickCount;
};
#endif
