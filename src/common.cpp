#include "common.h"

#include <time.h>

// Colour constants
const Colour Colour::WHITE   = Colour(1.0f, 1.0f, 1.0f);
const Colour Colour::BLACK   = Colour(0.0f, 0.0f, 0.0f);
const Colour Colour::RED     = Colour(1.0f, 0.0f, 0.0f);
const Colour Colour::GREEN   = Colour(0.0f, 1.0f, 0.0f);
const Colour Colour::ORANGE  = Colour(1.0f, 0.5f, 0.0f);
const Colour Colour::BLUE    = Colour(0.0f, 0.0f, 1.0f);
const Colour Colour::CYAN    = Colour(0.0f, 1.0f, 1.0f);
const Colour Colour::MAGENTA = Colour(1.0f, 0.0f, 1.0f);
const Colour Colour::YELLOW  = Colour(1.0f, 1.0f, 0.0f);

// Rect constants
template <> const Rectf Rectf::UNIT = Rectf(0.0f, 0.0f, 1.0f, 1.0f);

// Returns time value in sec.
double getTime(void)
{
	static time_t start_sec = 0;

	timespec tp;
	clock_gettime (0, &tp);

	if (!start_sec)
		start_sec = tp.tv_sec;

	return static_cast<double>(tp.tv_sec - start_sec)
		 + static_cast<double>(tp.tv_nsec*0.000000001);
}

// Generates timevarying angle with a precition of 0.1 degree.
GLfloat generateAngle(float degPrSec)
{
	return ((unsigned int)(getTime()*degPrSec/0.1)
					% 3600) * 0.1;
}

Matrix4::Matrix4(const cv::Mat &m)
{
	assert(m.cols == 4);
	assert(m.rows == 4);

	for(unsigned int row=0; row<4; ++row)
	{
		for(unsigned int col=0; col<4; ++col)
		{
			c[row*4+col] = m.at<double>(row, col);
		}
	}
}

void Matrix4::transpose()
{
	Matrix4 old = *this;
	for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			c[row*4+col] = old.c[row+col*4];
		}
	}
}

std::ostream& operator<<(std::ostream& os, const Matrix4& m)
{
	os << "[";
	for (int r = 0; r < 4; r++)
	{
		for (int c = 0; c < 4; c++)
		{
			if (c > 0) os << " ";
			os << m.c[r*4+c];
			if (c < 3) os << ", ";
		}
		if (r < 3) os << std::endl;
	}
	os << "]" << std::endl;

	return os;
}

