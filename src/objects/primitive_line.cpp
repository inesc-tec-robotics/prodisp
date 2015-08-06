#include "objects/primitive_line.h"

using namespace std;

PrimitiveLine::PrimitiveLine() : Object(),
	line_width_(5.0)
{
}

PrimitiveLine::PrimitiveLine(const PrimitiveLine& src) : Object(src),
	line_width_(src.line_width_)
{
	// Copy colours:
	for (int i = 0; i < (int)src.colours_.size(); ++i)
		addColour(src.colours_[i]->c);

	// Copy colours:
	for (int l = 0; l < (int)src.lines_.size(); ++l)
	{
		int dest_v_i[2];
		int dest_c_i[2];

		for (int p = 0; p < 2; ++p)
		{
			Vector3* src_v = src.lines_[l]->vertices_[p];
			Colour* src_c = src.lines_[l]->colours_[p];

			// Find the same vertex in this:
			for (int i = 0; i < (int)vertices_.size(); ++i)
			{
				Vector3* dest_v = vertices_[i];
				if (*src_v == *dest_v)
				{
					// Found!
					dest_v_i[p] = i;
					break;
				}
			}

			// Find the same colour in this:
			for (int i = 0; i < (int)colours_.size(); ++i)
			{
				Colour* dest_c = colours_[i];
				if (*src_c == *dest_c)
				{
					// Found!
					dest_c_i[p] = i;
					break;
				}
			}

		}
		// Add new face:
		addLine(dest_v_i[0], dest_v_i[1],
				  dest_c_i[0], dest_c_i[1]);
	}
}

PrimitiveLine::~PrimitiveLine()
{
	for (int i = 0; i < (int)colours_.size(); ++i)
		delete colours_[i];

	for (int i = 0; i < (int)lines_.size(); ++i)
		delete lines_[i];
}

int PrimitiveLine::addColour(const float c[4])
{
	colours_.push_back( new Colour(c[0], c[1], c[2], c[3]) );

	return ((int)colours_.size()-1);
}

void PrimitiveLine::addLine(int v0, int v1,
									 int c0, int c1)
{
	lines_.push_back( new Line(vertices_[v0], vertices_[v1],
										colours_[c0], colours_[c1]) );
}

bool PrimitiveLine::drawVertices()
{
	for (int i = 0; i < (int)lines_.size(); i++)
	{
		drawLine( lines_[i] );
	}
	return true;
}

void PrimitiveLine::drawLine(Line* line)
{
	glDisable(GL_LIGHTING);
	glLineWidth(line_width_);

	glBegin(GL_LINES);
	for (int i = 0; i < 2; i++)
	{
		glColor3fv(line->colours_[i]->c);
		glVertex3fv(line->vertices_[i]->c);
	}
	glEnd();

	glEnable(GL_LIGHTING);
}

void PrimitiveLine::setColour(const Colour& colour)
{
	if (colours_.size() == 0)
	{
		addColour(colour.c);
	}
	else
	{
		for (int i = 0; i < (int)colours_.size(); i++)
		{
			*colours_[i] = colour;
		}
	}
}

