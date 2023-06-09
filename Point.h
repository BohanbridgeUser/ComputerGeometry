#ifndef _POINT_H_
#define _POINT_H_
class Point{
	public:
	double x,y;
	Point(){};
	Point(double i, double j):x(i),y(j){};
	Point&  operator-(const Point& another) {
		x -= another.x;
		y -= another.y;
		return *this;
	}
	friend Point& operator-(const Point& a, const Point& b); 
};
Point& operator-(const Point& a, const Point& b)
{
	return a-b;
}
#endif
