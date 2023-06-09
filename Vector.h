#ifndef _VECTOR_H_
#define _VECTOR_H_
#include "Point.h"

class Vector : public Point {
	public:
	Vector();
	Vector(double a, double b):Point(a,b){};
	Vector&  operator+(const Vector& another){
		x += another.x;
		y += another.y;	
		return *this;
	}
	Point operator+(const Point& aPoint){
		x += aPoint.x;
		y += aPoint.y;
		return (Point)(*this);
	}
	friend Vector operator-(const Vector& a, const Vector& b);
};
Vector operator-(const Vector& a, const Vector& b)
{
	return Vector(a.x-b.x,a.y-b.y);
}
#endif
