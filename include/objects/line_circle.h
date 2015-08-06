#ifndef LINE_CIRCLE_H__
#define LINE_CIRCLE_H__

#include "objects/primitive_line.h"

class LineCircle : public PrimitiveLine
{
public:
	LineCircle();
	void setNumPoints(int num_points) { num_points_ = num_points; }
	virtual void createPrimitive();

private:
	int num_points_;
};


#endif // LINE_CIRCLE_H__
