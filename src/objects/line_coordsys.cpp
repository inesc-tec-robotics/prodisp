#include "objects/line_coordsys.h"

void LineCoordsys::createPrimitive()
{
	addVertex(0, 0, 0);
	addVertex(1, 0, 0);
	addVertex(0, 1, 0);
	addVertex(0, 0, 1);

	addColour(Colour::WHITE.c);
	addColour(Colour::RED.c);
	addColour(Colour::GREEN.c);
	addColour(Colour::BLUE.c);

	// X
	addLine(0, 1, 0, 1);	// x
	addLine(0, 2, 0, 2);	// y
	addLine(0, 3, 0, 3);	// z
}
