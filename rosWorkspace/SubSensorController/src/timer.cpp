/*
 * timer.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: bholdaway
 */

#include <sys/time.h>
#include "timer.h"

void timer::start(time_t sec, suseconds_t usec)
{
  gettimeofday(&m_endTime, &m_timeZone);
  m_endTime.tv_sec += sec;
  m_endTime.tv_sec += usec;
}

bool timer::isTimeout()
{
  bool ret = false;
  struct timeval tmp;
  struct timezone tz;
  unsigned long timeLeft, curTime;

  gettimeofday(&tmp, &tz);
  curTime = (tmp.tv_sec * 1000000) + tmp.tv_usec;
  timeLeft = (m_endTime.tv_sec * 1000000) + m_endTime.tv_usec;

  if (curTime >= timeLeft)
  {
    ret = true;
  }

  return ret;
}
