#include "objects/object_composer.h"
#include "objects/line_circle.h"
#include "objects/position.h"
#include "objects/plane.h"
#include "objects/line_cross.h"
#include "objects/cube.h"

// Singleton instance
template<> ObjectComposer* Singleton<ObjectComposer>::msInstance = 0;

//Object* ObjectComposer::getStud(void)
//{
//	LineCross* stud = new LineCross;
//	stud->setColour(Colour::RED);
//	stud->createPrimitive();
//	stud->scaleVertices(0.025);
//	return stud;
//}

Object* ObjectComposer::getRing(void)
{
	LineCircle* main = new LineCircle;
	main->setColour(Colour::YELLOW);
	main->setNumPoints(40);
	main->createPrimitive();
	main->scaleVertices(0.05);

	// X
	LineCircle* mini = new LineCircle;
	mini->setColour(Colour::RED);
	main->setNumPoints(40);
	mini->createPrimitive();
	mini->scaleVertices(0.01);
	mini->setParent(main);
	mini->getTf().setTranslation(0.05,0,0);

	// Y
	mini = new LineCircle(*mini);
	mini->setColour(Colour::GREEN);
	mini->getTf().setTranslation(0,0.05,0);

	return main;
}

Object* ObjectComposer::getColourCube(float x, float y, float z)
{
	float trans_x = x/2;
	float trans_y = y/2;
	float trans_z = z/2;

	Position* center = new Position;

	Plane* top = new Plane(x, z, 1);
	top->setColour(Colour::RED);
	top->setParent(center);
	top->getTf().setTranslation(0, trans_y, 0);
	top->createPrimitive();

	Plane* bottom = new Plane(*top);
	bottom->setColour(Colour::BLUE);
	bottom->getTf().setTranslation(0, -trans_y, 0);

	Plane* left = new Plane(y, z, 1);
	left->setColour(Colour::CYAN);
	left->setParent(center);
	left->getTf().setTranslation(-trans_x, 0, 0);
	left->getTf().setRotation(0, 0, 90);
	left->createPrimitive();

	Plane* right = new Plane(*left);
	right->setColour(Colour::GREEN);
	right->getTf().setTranslation(trans_x, 0, 0);

	Plane* front = new Plane(x, y, 1);
	front->setColour(Colour::MAGENTA);
	front->setParent(center);
	front->getTf().setTranslation(0, 0, -trans_z);
	front->getTf().setRotation(90, 0, 0);
	front->createPrimitive();

	Plane* back = new Plane(*front);
	back->setColour(Colour::YELLOW);
	back->getTf().setTranslation(0, 0, trans_z);

	return center;
}

Object* ObjectComposer::getCoordBlocks(float lengths)
{
	Position* center = new Position;

	float hlength = 0.50 * lengths;
	float hwidth  = 0.06 * lengths;

	Cube* x_axis = new Cube;
	x_axis->createPrimitive();
	x_axis->scaleVerticesAlong(hlength, hwidth, hwidth);
	x_axis->getTf().setTranslation(hlength, 0, 0);
	x_axis->setColour(Colour::RED);
	x_axis->setParent(center);

	Cube* y_axis = new Cube;
	y_axis->createPrimitive();
	y_axis->scaleVerticesAlong(hwidth, hlength, hwidth);
	y_axis->getTf().setTranslation(0, hlength, 0);
	y_axis->setColour(Colour::GREEN);
	y_axis->setParent(center);

	Cube* z_axis = new Cube;
	z_axis->createPrimitive();
	z_axis->scaleVerticesAlong(hwidth, hwidth, hlength);
	z_axis->getTf().setTranslation(0, 0, hlength);
	z_axis->setColour(Colour::BLUE);
	z_axis->setParent(center);

	return center;
}





