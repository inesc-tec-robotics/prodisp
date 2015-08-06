#include <iostream>
//#include <sys/timeb.h>

#include "timer_ms.h"

using boost::timer::cpu_timer;
using boost::timer::cpu_times;
using boost::timer::nanosecond_type;

double TimerMs::getTime(void)
{
	return (double)timer_.elapsed().wall/1000000.0;
}

double TimerMs::getTimeRestart(void)
{
	double time = (double)timer_.elapsed().wall/1000000.0;
	restart();
	return time;
}

void TimerMs::restart(void)
{
	timer_.stop();
	timer_.start();
}
