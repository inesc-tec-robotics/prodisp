#ifndef TIMERMS_HPP
#define TIMERMS_HPP

//=================================
// Forward declared dependencies
//=================================


//=================================
// Included dependencies
//=================================
//#include <boost/timer/timer.hpp>




class TimerMs
{
public:
	TimerMs(void) { restart(); }

	double getTime(void);
	double getTimeRestart(void);
	void restart(void);

private:
//	int getMilliCount(void);
//	int getMilliSpan(int nTimeStart);

//	boost::timer::cpu_timer timer_;
};


#endif // TIMERMS_HPP
