#include "objects/line_circle.h"

LineCircle::LineCircle() :
	num_points_(20)
{

}

void LineCircle::createPrimitive()
{
	if (colours_.size() == 0)
		addColour(Colour::WHITE.c);

	float step_size = 2*M_PI / (float)num_points_;
	for (int i = 0; i < num_points_; i++)
	{
		float angle = (float)i*step_size;
		addVertex(cos(angle), sin(angle), 0);

		if (i > 0)
		{
			addLine(i-1, i, 0, 0);
		}
	}
	addLine(num_points_-1, 0, 0, 0);
}
