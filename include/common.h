#ifndef Common_H__
#define Common_H__

#include "prerequisites.h"

#include "MathOperations.hpp"
#include <iostream>
#include <cmath>
#include <cstring>
#include <GL/gl.h>
#include <opencv2/core/core.hpp>
#include <boost/scoped_ptr.hpp>

#define EPSILON 0.00001f

//class MatGL : MathOp::Transform
//{
//	float* getGl(void);	// 16 length
//};

// Colour
struct Colour
{
	union
	{
		struct { float r, g, b, a; };
		float c[4];
	};

	Colour() {}
	Colour(float r, float g, float b, float a = 1.0f) : r(r), g(g), b(b), a(a) {}

	bool operator==(const Colour& c) const;
	Colour operator*(const float x) const;

	static const Colour WHITE;
	static const Colour BLACK;
	static const Colour RED;
	static const Colour GREEN;
	static const Colour ORANGE;
	static const Colour BLUE;
	static const Colour CYAN;
	static const Colour MAGENTA;
	static const Colour YELLOW;
};

inline bool Colour::operator==(const Colour& c) const
{
	return (c.r == r) && (c.g == g) && (c.b == b) && (c.a == a);
}

inline Colour Colour::operator*(const float x) const
{
	return Colour(r*x,g*x,b*x);
}

// Point
template <typename T>
struct Point
{
	union
	{
		struct { T x, y; };
		T c[2];
	};

	Point() {}
	Point(T x, T y) : x(x), y(y) {}
};

typedef Point<int> Pointi;
typedef Point<float> Pointf;

// Rect
template <typename T>
struct Rect
{
	union
	{
		struct { T left, top, right, bottom; };
		T c[4];
	};

	Rect() {}

	Rect(T left, T top, T right, T bottom)
		: left(left), top(top), right(right), bottom(bottom)
	{}

	Rect(const Point<T>& a, const Point<T>& b)
		: left(a.x), top(a.y), right(b.x), bottom(b.y)
	{}

	Rect(const cv::Point_<T>& a, const cv::Point_<T>& b)
	{
		left  = std::min(a.x,b.x);
		top   = std::min(a.y,b.y);
		right = std::max(a.x,b.x);
		bottom= std::max(a.y,b.y);
	}

	T width() const { return (right - left); }
	T height() const { return (top - bottom); }
	
	void resize(const Point<T>& a, const Point<T>& b)
	{
		left = a.x; top = a.y; right = b.x; bottom = b.y;
	}

//	void merge(const Point<T>& p)
//	{
//		// NB: Positive y-direction is up here
//		left   = std::min(left, p.x);
//		top    = std::max(top, p.y);
//		right  = std::max(right, p.x);
//		bottom = std::min(bottom, p.y);
//	}

	static const Rect UNIT;
};

typedef Rect<int> Recti;
typedef Rect<float> Rectf;

// Vector2
struct Vector2
{
	union
	{
		struct { float x, y; };
		float c[2];
	};

	Vector2() {}
	Vector2(float x, float y) : x(x), y(y) {}
	Vector2(const float* v) : x(v[0]), y(v[1]) {}

	// Operations
	float dot(const Vector2& rhs) const { return x*rhs.x + y*rhs.y; }
	Vector2 perpendicular() const { return Vector2(-y, x); }
	float length() const { return sqrtf(lengthSqr()); }
	float lengthSqr() const { return dot(*this); }
	void normalize();
	Vector2 normalized() const;
	Vector2 scaled(const Vector2& s) const { return Vector2(x*s.x, y*s.y); }

	// Operators
	bool operator==(const Vector2& v) const;

	Vector2& operator+=(const Vector2& rhs) { x += rhs.x; y += rhs.y; return *this; }
	Vector2& operator-=(const Vector2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
	Vector2& operator*=(float rhs) { x *= rhs; y *= rhs; return *this; }
	Vector2& operator/=(float rhs) { x /= rhs; y /= rhs; return *this; }

	Vector2 operator-() const { return Vector2(-x, -y); }

	Vector2 operator+(const Vector2& rhs) const { return Vector2(x+rhs.x, y+rhs.y); }
	Vector2 operator-(const Vector2& rhs) const { return Vector2(x-rhs.x, y-rhs.y); }
	Vector2 operator*(float rhs) const { return Vector2(x*rhs, y*rhs);  }
	Vector2 operator/(float rhs) const { return Vector2(x/rhs, y/rhs);  }
};

inline void Vector2::normalize()
{
	float l = length();
	if (l != 0.0f)
		*this /= l;
}

inline Vector2 Vector2::normalized() const
{
	Vector2 temp(*this);
	temp.normalize();
	return temp;
}

inline bool Vector2::operator==(const Vector2& v) const
{
	return (v.x == x) && (v.y == y);
}

inline float dot(const Vector2& a, const Vector2& b)
{
	return a.dot(b);
}

inline Vector2 operator*(float lhs, const Vector2& rhs)
{
	return rhs*lhs;
}

// Vector3
struct Vector3
{
	float c[3];
//		union
//		{
//			struct { float x, y, z; };
//			float c[3];
//		};
//	union
//	{
//		struct { float x, y, z; };
//		float c[3];
//	};

	Vector3()									{ c[0] = 0;		c[1] = 0;	 c[2] = 0; }
	Vector3(float x, float y, float z)	{ c[0] = x;		c[1] = y;	 c[2] = z; }
	Vector3(const float* v)					{ c[0] = v[0]; c[1] = v[1]; c[2] = v[2]; }
	Vector3(const double* v)				{ c[0] = v[0]; c[1] = v[1]; c[2] = v[2]; }
	Vector3(const CvPoint3D32f v)			{ c[0] = v.x;	c[1] = v.y;	 c[2] = v.z; }
	Vector3(const cv::Vec3d v)				{ c[0] = v[0]; c[1] = v[1]; c[2] = v[2]; }
	Vector3(const cv::Point3d v)			{ c[0] = v.x;	c[1] = v.y;	 c[2] = v.z; }

	// Get
	float x(void) const { return c[0]; }
	float y(void) const { return c[1]; }
	float z(void) const { return c[2]; }

	// Operations
	float dot(const Vector3& rhs) const { return c[0]*rhs.c[0] + c[1]*rhs.c[1] + c[2]*rhs.c[2]; }
	Vector3 cross(const Vector3& rhs) const;
	float length() const { return sqrtf(lengthSqr()); }
	float lengthSqr() const { return dot(*this); }
	void normalize();
	Vector3 normalized() const;
	Vector3& projectToPlane(const Vector3& plane_n);

	// Operators
	bool operator==(const Vector3& v) const;

	Vector3& operator+=(const Vector3& rhs) { c[0] += rhs.c[0]; c[1] += rhs.c[1]; c[2] += rhs.c[2]; return *this; }
	Vector3& operator-=(const Vector3& rhs) { c[0] -= rhs.c[0]; c[1] -= rhs.c[1]; c[2] -= rhs.c[2]; return *this; }
	Vector3& operator*=(float rhs) { c[0] *= rhs; c[1] *= rhs; c[2] *= rhs; return *this; }
	Vector3& operator/=(float rhs) { c[0] /= rhs; c[1] /= rhs; c[2] /= rhs; return *this; }

	Vector3 operator-() const { return Vector3(-c[0], -c[1], -c[2]); }

	Vector3 operator+(const Vector3& rhs) const { return Vector3(c[0]+rhs.c[0], c[1]+rhs.c[1], c[2]+rhs.c[2]); }
	Vector3 operator-(const Vector3& rhs) const { return Vector3(c[0]-rhs.c[0], c[1]-rhs.c[1], c[2]-rhs.c[2]); }
	Vector3 operator*(float rhs) const { return Vector3(c[0]*rhs, c[1]*rhs, c[2]*rhs);  }
	Vector3 operator/(float rhs) const { return Vector3(c[0]/rhs, c[1]/rhs, c[2]/rhs);  }
};

//typedef boost::scoped_ptr<Vector3> Vector3ScPtr;

inline Vector3 Vector3::cross(const Vector3& rhs) const
{
	return Vector3(
			c[1]*rhs.c[2] - c[2]*rhs.c[1],
			c[2]*rhs.c[0] - c[0]*rhs.c[2],
			c[0]*rhs.c[1] - c[1]*rhs.c[0]);
}

inline void Vector3::normalize()
{
	float l = length();
	if (l != 0.0f)
		*this /= l;
}

inline Vector3 Vector3::normalized() const
{
	Vector3 temp(*this);
	temp.normalize();
	return temp;
}

inline Vector3& Vector3::projectToPlane(const Vector3& plane_n)
{
	double temp = dot(plane_n)/plane_n.lengthSqr();
	*this = (*this) - plane_n * temp;

	return *this;
}

inline bool Vector3::operator==(const Vector3& v) const
{
	return (v.c[0] == c[0]) && (v.c[1] == c[1]) && (v.c[2] == c[2]);
}

inline float dot(const Vector3& a, const Vector3& b)
{
	return a.dot(b);
}

inline Vector3 cross(const Vector3& a, const Vector3& b)
{
	return a.cross(b);
}

inline Vector3 operator*(float lhs, const Vector3& rhs)
{
	return rhs*lhs;
}

// Matrix4
struct Matrix4
{
	// Column-major matrix layout (pre-multiply vectors with matrices)
	// (See 9.005 on http://www.opengl.org/resources/faq/technical/transformations.htm)
	//
	//   [ 0   4   8  12 ]
	//   [ 1   5   9  13 ]
	//   [ 2   6  10  14 ]
	//   [ 3   7  11  15 ]

	float c[16];

	Matrix4() {}
	Matrix4(const float* m) { memcpy(c, m, sizeof(c)); }
	Matrix4(const cv::Mat& m);

	// Operations
	Matrix4 setIdentity();
	void setTranslation(const Vector3& v);
	void setRotation(const Vector3& u, float angle);
	void setPerspective(float fovy, float aspect/*, float zNear, float zFar*/);
	void transpose();

	Vector3 transform(const Vector3& v) const;
	Vector3 transformAffine(const Vector3& v) const;

	// Operators
	Matrix4& operator*=(const Matrix4& rhs);		// Assumes column-major
	Matrix4 operator*(const Matrix4& rhs) const;	// Assumes column-major
	Vector3 operator*(const Vector3& rhs) const;	// Assumes row-major
	// operator<< cannot be member because Matrix is right-side argument:
	friend std::ostream& operator<<(std::ostream& os, const Matrix4& m);

	// Creation
	static Matrix4 translation(const Vector3& v);
	static Matrix4 rotation(const Vector3& u, float angle);
};

inline Matrix4 Matrix4::setIdentity()
{
	c[1] = c[2] = c[3] = c[4] = c[6] = c[7] = c[8] = c[9] = c[11] = c[12] = c[13] = c[14] = 0.0f;
	c[0] = c[5] = c[10] = c[15] = 1.0f;
	return *this;
}

inline void Matrix4::setTranslation(const Vector3& v)
{
	c[1] = c[2] = c[3] = c[4] = c[6] = c[7] = c[8] = c[9] = c[11] = 0.0f;
	c[0] = c[5] = c[10] = c[15] = 1.0f;
	c[12] = v.c[0];
	c[13] = v.c[1];
	c[14] = v.c[2];
}

inline void Matrix4::setRotation(const Vector3& u, float angle)
{
	// http://en.wikipedia.org/wiki/Rotation_matrix

	angle *= M_PI/180.0f;

	// The axis u is assumed to be normalized
	float co = cosf(angle);
	float si = sinf(angle);

	c[ 0] = u.c[0]*u.c[0]+(1.0f-u.c[0]*u.c[0])*co;
	c[ 1] = u.c[0]*u.c[1]*(1.0f-co)+u.c[2]*si;
	c[ 2] = u.c[0]*u.c[2]*(1.0f-co)-u.c[1]*si;

	c[ 4] = u.c[0]*u.c[1]*(1.0f-co)-u.c[0]*si;
	c[ 5] = u.c[1]*u.c[1]+(1.0f-u.c[1]*u.c[1])*co;
	c[ 6] = u.c[1]*u.c[2]*(1.0f-co)+u.c[0]*si;

	c[ 8] = u.c[0]*u.c[2]*(1.0f-co)+u.c[1]*si;
	c[ 9] = u.c[1]*u.c[2]*(1.0f-co)-u.c[0]*si;
	c[10] = u.c[2]*u.c[2]+(1.0f-u.c[2]*u.c[2])*co;

	c[ 3] = c[ 7] = c[11] = c[12] = c[13] = c[14] = 0.0f;
	c[15] = 1.0f;
}

inline void Matrix4::setPerspective(float fovy, float aspect/*, float zNear, float zFar*/)
{
	// http://www.opengl.org/sdk/docs/man/xhtml/gluPerspective.xml
	// (modified to store z directly in z')

	fovy *= 0.5f * (M_PI/180.0f);
	float f = cosf(fovy)/sinf(fovy);

	c[1] = c[ 2] = c[ 3] = c[ 4] = c[ 6] = c[7] = c[8] = c[9] = c[11]
		 = c[12] = c[13] = c[14] = c[15] = 0.0f;

	c[ 0] = f/aspect;
	c[ 5] = f;
	c[10] = 1.0f;
}

inline Vector3 Matrix4::transform(const Vector3& v) const
{
	return Vector3(
			c[0]*v.c[0] + c[4]*v.c[1] + c[ 8]*v.c[2],
			c[1]*v.c[0] + c[5]*v.c[1] + c[ 9]*v.c[2],
			c[2]*v.c[0] + c[6]*v.c[1] + c[10]*v.c[2]);
}

inline Vector3 Matrix4::transformAffine(const Vector3& v) const
{
	return Vector3(
			c[0]*v.c[0] + c[4]*v.c[1] + c[ 8]*v.c[2] + c[12],
			c[1]*v.c[0] + c[5]*v.c[1] + c[ 9]*v.c[2] + c[13],
			c[2]*v.c[0] + c[6]*v.c[1] + c[10]*v.c[2] + c[14]);
}

inline Matrix4& Matrix4::operator*=(const Matrix4& rhs)
{
	Matrix4 temp;

	temp.c[ 0] = c[ 0]*rhs.c[ 0] + c[ 4]*rhs.c[ 1] + c[ 8]*rhs.c[ 2] + c[12]*rhs.c[ 3];
	temp.c[ 1] = c[ 1]*rhs.c[ 0] + c[ 5]*rhs.c[ 1] + c[ 9]*rhs.c[ 2] + c[13]*rhs.c[ 3];
	temp.c[ 2] = c[ 2]*rhs.c[ 0] + c[ 6]*rhs.c[ 1] + c[10]*rhs.c[ 2] + c[14]*rhs.c[ 3];
	temp.c[ 3] = c[ 3]*rhs.c[ 0] + c[ 7]*rhs.c[ 1] + c[11]*rhs.c[ 2] + c[15]*rhs.c[ 3];

	temp.c[ 4] = c[ 0]*rhs.c[ 4] + c[ 4]*rhs.c[ 5] + c[ 8]*rhs.c[ 6] + c[12]*rhs.c[ 7];
	temp.c[ 5] = c[ 1]*rhs.c[ 4] + c[ 5]*rhs.c[ 5] + c[ 9]*rhs.c[ 6] + c[13]*rhs.c[ 7];
	temp.c[ 6] = c[ 2]*rhs.c[ 4] + c[ 6]*rhs.c[ 5] + c[10]*rhs.c[ 6] + c[14]*rhs.c[ 7];
	temp.c[ 7] = c[ 3]*rhs.c[ 4] + c[ 7]*rhs.c[ 5] + c[11]*rhs.c[ 6] + c[15]*rhs.c[ 7];

	temp.c[ 8] = c[ 0]*rhs.c[ 8] + c[ 4]*rhs.c[ 9] + c[ 8]*rhs.c[10] + c[12]*rhs.c[11];
	temp.c[ 9] = c[ 1]*rhs.c[ 8] + c[ 5]*rhs.c[ 9] + c[ 9]*rhs.c[10] + c[13]*rhs.c[11];
	temp.c[10] = c[ 2]*rhs.c[ 8] + c[ 6]*rhs.c[ 9] + c[10]*rhs.c[10] + c[14]*rhs.c[11];
	temp.c[11] = c[ 3]*rhs.c[ 8] + c[ 7]*rhs.c[ 9] + c[11]*rhs.c[10] + c[15]*rhs.c[11];

	temp.c[12] = c[ 0]*rhs.c[12] + c[ 4]*rhs.c[13] + c[ 8]*rhs.c[14] + c[12]*rhs.c[15];
	temp.c[13] = c[ 1]*rhs.c[12] + c[ 5]*rhs.c[13] + c[ 9]*rhs.c[14] + c[13]*rhs.c[15];
	temp.c[14] = c[ 2]*rhs.c[12] + c[ 6]*rhs.c[13] + c[10]*rhs.c[14] + c[14]*rhs.c[15];
	temp.c[15] = c[ 3]*rhs.c[12] + c[ 7]*rhs.c[13] + c[11]*rhs.c[14] + c[15]*rhs.c[15];

	return *this = temp;
}

inline Matrix4 Matrix4::operator*(const Matrix4& rhs) const
{
	Matrix4 temp(*this);
	temp *= rhs;
	return temp;
}

inline Vector3 Matrix4::operator*(const Vector3& rhs) const	// Assumes row-major
{
	Vector3 temp;
	temp.c[0] = c[0]*rhs.c[0] + c[1]*rhs.c[1] + c[2]*rhs.c[2];
	temp.c[1] = c[4]*rhs.c[0] + c[5]*rhs.c[1] + c[6]*rhs.c[2];
	temp.c[2] = c[8]*rhs.c[0] + c[9]*rhs.c[1] + c[10]*rhs.c[2];
	return temp;
}

inline Matrix4 Matrix4::translation(const Vector3& v)
{
	Matrix4 m;
	m.setTranslation(v);
	return m;
}

inline Matrix4 Matrix4::rotation(const Vector3& u, float angle)
{
	Matrix4 m;
	m.setRotation(u, angle);
	return m;
}

/** Perspective correct interpolation. */
inline float interpolatePerspective(float s, float z1, float z2)
{
	// http://www.comp.nus.edu.sg/~lowkl/publications/lowk_persp_interp_techrep.pdf

	if (z1 == 0.0f || z2 == 0.0f)
		return 0.0f; // fail gracefully

	float rz1 = 1.0f/z1;
	float rz2 = 1.0f/z2;

	return 1.0f/(rz1 + s*(rz2 - rz1));
}

/** Tests for intersection between two 2D lines, and returns whether they intersect. If the
 *  lines intersect, the fractions of the intersections are stored in s and t. If s and t are
 *  both between 0 and 1, the intersection lies on the segments. */
inline bool lineIntersection(const Vector3 p[4], float& s, float& t)
{
	// http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/

	float den = (p[3].y() - p[2].y())*(p[1].x() - p[0].x()) - (p[3].x() - p[2].x())*(p[1].y() - p[0].y());

	if (fabs(den) < EPSILON)
		return false; // segments are parallel

	s = ((p[3].x() - p[2].x())*(p[0].y() - p[2].y()) - (p[3].y() - p[2].y())*(p[0].x() - p[2].x()))/den;
	t = ((p[1].x() - p[0].x())*(p[0].y() - p[2].y()) - (p[1].y() - p[0].y())*(p[0].x() - p[2].x()))/den;

	return true; // lines intersect (or are coincident)
}

// Returns time value in sec.
double getTime(void);

// Generates timevarying angle with a precition of 0.1 degree.
GLfloat generateAngle(float degPrSec);

#endif // Common_H__
