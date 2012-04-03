/*
 * timer.h
 *
 */
#include <sys/time.h>

#ifndef TIMER_H_
#define TIMER_H_

class timer {
    private:
        struct timeval m_endTime;
        struct timezone m_timeZone;

    public:
        void start(time_t sec, suseconds_t usec);

        bool isTimeout();
};

#endif /* TIMER_H_ */
