#ifndef Singleton_H__
#define Singleton_H__

#include "prerequisites.h"

#include <boost/noncopyable.hpp>

template <class T>
class Singleton : boost::noncopyable
{
public:
	Singleton()
	{
		assert( !msInstance );
		msInstance = static_cast<T*>(this);
	}

	~Singleton()
	{
		assert( msInstance );
		msInstance = 0;
	}

	static T& getInstance()    { assert( msInstance ); return *msInstance; }
	static T* getInstancePtr() { return msInstance; }

protected:
	static T* msInstance;
};

#endif // Singleton_H__
