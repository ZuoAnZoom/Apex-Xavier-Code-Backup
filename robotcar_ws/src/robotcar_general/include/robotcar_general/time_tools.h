#ifndef TIME_TOOLS
#define TIME_TOOLS

#include <stdio.h>
#include <sys/time.h>

class TimeCounter
{
public:
  TimeCounter() {};

  inline void start()
  {
    gettimeofday(&start_, NULL);
  }

  inline void end()
  {
    gettimeofday(&end_, NULL);
  }

  inline float getDT()
  {
    float time_use = (end_.tv_sec - start_.tv_sec) * 1000000 + (end_.tv_usec - start_.tv_usec);
    return time_use / 1.0e6;
  }

private:
  struct timeval start_, end_;
};

#endif /* TIME_TOOLS */
