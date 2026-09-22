#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__
#include "plane.h"

class Triangle : public Plane{
public:
   double thirdX;
   double n1x, n1y, n1z, n2x, n2y, n2z, invD;
   Triangle(Vector c, Vector b, Vector a, Texture* t);
   double getIntersection(Ray ray);
   bool getLightIntersection(Ray ray, double* fill);
};

#endif