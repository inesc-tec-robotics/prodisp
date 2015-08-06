#ifndef Runnable_H__
#define Runnable_H__

#include "prerequisites.h"

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

/** Abstract class for easily creating active classes, i.e.\ classes internally running a thread
 *  of execution. */
class Runnable
{
public:
	virtual ~Runnable();

	void start();

protected:
	/** Thread entry point, implement this in derived classes. */
	virtual void run() = 0;

	void terminate();

protected:
	boost::scoped_ptr<boost::thread> mThread;
};

#endif // Runnable_H__
