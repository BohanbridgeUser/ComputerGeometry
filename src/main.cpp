#include <iostream>
#include "Point.h"
#include "Vector.h"

int main(int argc, char** argv)
{
	using std::cout;
	using std::endl;
	Point A(10.0,20.0);
	Point B(30.0,40.0);
	Vector V((A-B).x,(A-B).y);
	cout << V.x << " " << V.y << endl;
	return 0;
}
