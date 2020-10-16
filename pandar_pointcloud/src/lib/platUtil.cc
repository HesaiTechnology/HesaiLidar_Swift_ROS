#include <pandar_pointcloud/platUtil.h>
#include <unistd.h>
#include <sys/syscall.h>

#define gettid() syscall(SYS_gettid)

void ShowThreadPriorityMaxMin (int policy)
{   
    int priority = sched_get_priority_max (policy);
    printf ("policy %d max_priority = %d\n", policy, priority);
    priority = sched_get_priority_min (policy);
    printf ("policy %d, min_priority = %d\n", policy, priority);
}

void SetThreadPriority (int policy, int priority)
{ 
    printf("set thread %lu, tid %ld, policy %d and priority %d\n", pthread_self(), gettid(), policy, priority);
    sched_param param;
    param.sched_priority = priority;
    pthread_setschedparam(pthread_self(), policy, &param);

    int ret_policy;
    pthread_getschedparam(pthread_self(), &ret_policy, &param);
    printf("get thead %lu, tid %ld, policy %d and priority %d\n", pthread_self(), gettid(), ret_policy, param.sched_priority);

}


unsigned int GetTickCount() {
  unsigned int ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000000 + time.tv_sec * 1000;
  }
  return ret;
}

unsigned int GetMicroTickCount() {
  unsigned int ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
  return ret;
}

uint64_t GetMicroTickCountU64() {
  uint64_t ret = 0;
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
  return ret;
}
