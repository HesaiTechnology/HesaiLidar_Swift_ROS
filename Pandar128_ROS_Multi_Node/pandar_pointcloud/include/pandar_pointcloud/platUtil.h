#ifndef _PLAT_UTIL_H_
#define _PLAT_UTIL_H_

#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>

#define SHED_FIFO_PRIORITY_HIGH      99
#define SHED_FIFO_PRIORITY_MEDIUM    70
#define SHED_FIFO_PRIORITY_LOW        1

extern void ShowThreadPriorityMaxMin (int policy);
extern void SetThreadPriority (int policy, int priority);

extern unsigned int GetTickCount();

extern unsigned int GetMicroTickCount();

extern uint64_t     GetMicroTickCountU64();

#endif  //_PLAT_UTIL_H_
