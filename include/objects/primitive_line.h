#ifndef PRIMITIVE_LINE_H__
#define PRIMITIVE_LINE_H__

#include "objects/object.h"

#include <vector>

struct Line
{
public:
	Vector3* vertices_[2];
	Colour* colours_[2];

	Line(Vector3* v0, Vector3* v1,
		  Colour* c0, Colour* c1)
	{
		vertices_[0] = v0;
		vertices_[1] = v1;

		colours_[0] = c0;
		colours_[1] = c1;
	}
};

class PrimitiveLine : public Object
{
public:
	PrimitiveLine();
	PrimitiveLine(const PrimitiveLine& src);	// Copy constructor
	virtual ~PrimitiveLine();

	void setLineWidth(float line_width) { line_width_ = line_width; }

	virtual void createPrimitive(void) = 0;
	void setColour(const Colour& colour);

protected:
	int addColour(const float c[]);
	void addLine(int v0, int v1,
					 int c0, int c1);

private:
	bool drawVertices();
	void drawLine(Line* face);

protected:
	std::vector<Colour*> colours_;

private:
	float line_width_;
	std::vector<Line*> lines_;
};


#endif // PRIMITIVE_LINE_H__
