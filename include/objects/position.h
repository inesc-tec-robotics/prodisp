#ifndef POSITION_H
#define POSITION_H

#include "object.h"

class Position : public Object
{
public:
	Position() : Object() {}
	virtual ~Position() {}

	void setColour(const Colour& colour) {}

protected:
	bool drawVertices() { return true; }
};

#endif // POSITION_H
