#include <iostream>
#include <cstdlib>
#include <cmath>
const double pi = acos(-1.0);
const double eps = 1e-8;
int sgn(double x)
{
	if(fabs(x) < eps) return 00;
	else return x<0? -1:1;
}
int dcmp(double x, double y)
{
	if(fabs(x-y) < eps) return 0;
	else return x<y? -1:1;
}

