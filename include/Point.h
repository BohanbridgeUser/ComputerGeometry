#ifndef _POINT_H_
#define _POINT_H_
#include <cmath>
class Point{
	public:
	double x,y;
	Point(){};
	Point(double i, double j):x(i),y(j){};
	Point(const Point& another);
	Point& operator-(const Point& another) {
		x -= another.x;
		y -= another.y;
		return *this;
	}
	Point& operator+(const Point& another) {
		x += another.x;
		y += another.y;
		return *this;
	}
	Point operator=(const Point& another) {
		return Point(another.x,another.y);
	}
	friend double distance(const Point& a, const Point& b);
	friend double distance2(const Point& a, const Point& b);
	friend Point& operator-(const Point& a, const Point& b);
};
Point::Point(const Point& another)
{
	x = another.x;
	y = another.y;
}
Point& operator-(const Point& a, const Point& b)
{
	return a-b;
}
double distance(const Point& a, const Point& b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(pow(dx,2)+pow(dy,2));
}
double distance2(const Point& a, const Point& b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return pow(dx,2) + pow(dy,2);
}
#endif
