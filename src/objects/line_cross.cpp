#include "objects/line_cross.h"

LineCross::LineCross()
{

}

void LineCross::createPrimitive()
{
	if (colours_.size() == 0)
		addColour(Colour::WHITE.c);

	addVertex(-1, -1, 0);
	addVertex(-1,  1, 0);
	addVertex( 1, -1, 0);
	addVertex( 1,  1, 0);

	addLine(0, 3, 0, 0);
	addLine(1, 2, 0, 0);
}
