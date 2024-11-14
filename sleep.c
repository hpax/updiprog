#include "sleep.h"

void msleep(uint32_t msec)
{
  #ifdef _WIN32
	SleepEx(msec, false);
  #endif // _WIN32
  #ifdef __unix__
  usleep(msec*1000);
  #endif // __unix__
}

