#ifndef NAEX_TIMER_H
#define NAEX_TIMER_H

#include <boost/chrono.hpp>

namespace naex {

class Timer
{
private:
    typedef boost::chrono::high_resolution_clock Clock;
    typedef Clock::time_point Time;
    typedef boost::chrono::duration<double> Duration;
    Time start;
public:
    Timer(): start(Clock::now()) {}
    Timer(const Time &s): start(s) {}
    Timer(const Timer &s): start(s.start) {}
    void reset() { start = Clock::now(); }
    double seconds_elapsed() const
    {
        return boost::chrono::duration_cast<Duration>(Clock::now() - start).count();
    }
};

}

#endif //NAEX_TIMER_H
