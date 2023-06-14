#ifndef _VECTOR_H_
#define _VECTOR_H_
#include "BasicFun.h"
#include "Point.h"
class Vector : public Point {
	public:
		Vector();
		Vector(double const& a, double const & b):Point(a,b){};
		Vector(const Vector& another):Point(another.x,another.y){};
		Vector(const Point& a, const Point& b) { x = b.x - a.x; y = b.y - a.y;}
		Vector operator=(const Vector& another) { return Vector(another);}
	        double len()const;
		double len2()const;
		Vector& rotate(const double& angle) {
			double ox = x, oy = y;
			x = ox * cos(angle) - oy * sin(angle);
			y = ox * sin(angle) + oy * cos(angle);
			return *this;
		}
		Vector rotate(const double& angle) const {
			return Vector(x*cos(angle)-y*sin(angle),x*sin(angle) + y*cos(angle));
		}	
		Vector& operator*(const double& scale) { x *= scale; y *= scale; return *this;} 
		Vector& operator+(const Vector& another) { x += another.x; y += another.y; return *this; }
		Vector& operator-(const Vector& another) { x -= another.x; y -= another.y; return *this; } 
		Vector& operator/(const double& scale) { x /= scale; y /= scale; return *this;}
		bool operator==(const Vector& another) {return ((x== another.x) && (y == another.y));}
		friend Vector operator*(const Vector& a,const double& scale);
		friend Vector operator*(const double& scale, const Vector& a);
		friend Vector operator/(const Vector& a, const double& scale);
		friend Vector operator+(Vector& a, Vector& b);
		friend Vector operator-(Vector& a, Vector& b);
		friend Point operator+(const Point& a, const  Vector& b);
		friend Point operator+(const Vector& a, const Point& b);
		friend double dotproduct(const Vector& a, const Vector& b);
		friend double crossproduct(const Vector& a, const Vector& b);
		friend double areaparallelogram(const Point& a, const Point& b, const Point& c);
		friend double areatriangle(const Point& a, const Point& b, const Point& c);
		friend double angle(const Vector& a, const Vector& b);
		friend bool parallel(const Vector& a, const Vector& b);
};
double Vector::len2()const
{
	return dotproduct(*this,*this);
}
double Vector::len() const
{
	return sqrt(len2());
}
double angle(const Vector& a, const Vector& b)
{
	return acos( dotproduct(a,b) / a.len() / b.len() );
}
Vector operator*(const Vector& a,const double& scale)
{
	return Vector(a.x * scale,a.y * scale);
}
Vector operator*(const double& scale, const Vector& a)
{
	return Vector(a.x * scale, a.y * scale);
}
Vector operator/(const Vector& a, const double& scale)
{
	return Vector(a.x/scale,a.y/scale);
}
Vector operator+(Vector& a, Vector& b)
{
	return Vector(a.x + b.x, a.y + b.y);
}
Vector operator-(Vector& a, Vector& b)
{
	return Vector(a.x - b.x, a.y - b.y);
}
Point operator+(const Point& a, const Vector& b)
{
	return Point(a.x + b.x, a.y + b.y);
}
Point operator+(const Vector& a, const Point& b)
{
	return Point(a.x + b.x, a.y + b.y);
}	
double dotproduct(const Vector& a, const Vector& b)
{
	return a.x * b.x + a.y * b.y;
}
double crossproduct(const Vector& a, const Vector& b)
{
	return a.x * b.y - a.y * b.x;
}
double areaparallelogram(const Point& a, const Point& b, const Point& c)
{
	Vector ba(a,b), bc(c,b);
	return crossproduct(ba,bc);
}
double areatriangle(const Point& a, const Point& b, const Point& c)
{
	return areaparallelogram(a,b,c)/2.0;
}
bool parallel(const Vector& a, const Vector& b)
{
	return (sgn(crossproduct(a,b)) == 0);
}
#endif

