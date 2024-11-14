#include "sleep.h"

void msleep(uint32_t msec)
{
  #ifdef _WIN32
	SleepEx(msec, false);
  #endif // _WIN32
  #if defined(__APPLE__) || defined(__linux)
  usleep(msec*1000);
  #endif // __linux
}

