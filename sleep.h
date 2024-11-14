#ifndef SLEEP_H
#define SLEEP_H

#include <stdint.h>
#include <stdbool.h>
#ifdef _WIN32
#include <windows.h>
#include <unistd.h>
#endif // _WIN32
#ifdef __unix__
#include <unistd.h>
#endif // __unix__

void msleep(uint32_t usec);

#endif
