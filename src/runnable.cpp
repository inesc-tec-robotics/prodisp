#include "runnable.h"

#include <boost/bind.hpp>

Runnable::~Runnable()
{
	// NB: The destructor of the most derived class must call terminate()
	//     or wait for the thread to terminate in another way. Otherwise
	//     derived classes may be destroyed before execution of the thread
	//     has finished, and bad things will happen.
	
	// Check that the thread has been terminated (i.e. is not joinable)
	if (mThread)
		assert( !mThread->joinable() && "thread not terminated" );
}

/** Starts execution of the Runnable::run function.
 *  @remarks
 *      Must be called only once. */
void Runnable::start()
{
	assert( !mThread && "only call start once" );

	// Create and start thread which executes run()
	mThread.reset(new boost::thread(boost::bind(&Runnable::run, this)));
}

/** Interrupts the thread and waits for it to terminate.
 *  @remarks
 *      Call this function in the destructor of the most derived class to
 *      ensure that the thread is terminated correctly before the object is
 *      destroyed. Or establish some other method for proper termination. */
void Runnable::terminate()
{
	if (mThread)
	{
		// Request the thread to terminate
		mThread->interrupt();

		// Wait for thread to complete execution
		mThread->join();
	}
}
