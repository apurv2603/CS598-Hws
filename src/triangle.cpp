#include "triangle.h"

Triangle::Triangle(Vector c, Vector b, Vector a, Texture* t):Plane(Vector(0,0,0), t, 0., 0., 0., 0., 0.){
   center = c;
   Vector righta = (b-c);
   textureX = righta.mag();
   right = righta/textureX;
   vect = right.cross(b-a).normalize();

   xsin = -right.z;
   if(xsin<-1.)xsin = -1;
   else if (xsin>1.)xsin=1.;
   yaw = asin(xsin);
   xcos = sqrt(1.-xsin*xsin);

   zcos = right.x/xcos;
   zsin = -right.y/xcos;
   if(zsin<-1.)zsin = -1;
   else if (zsin>1.)zsin=1.;
   if(zcos<-1.)zcos = -1;
   else if (zcos>1.)zcos=1.;
   roll = asin(zsin);

   ycos = vect.z/xcos;
   if(ycos<-1.)ycos = -1;
   else if (ycos>1.)ycos=1.;
   pitch = acos(ycos);
   ysin = sqrt(1-ycos*ycos);

   up.x = -xsin*ysin*zcos+ycos*zsin;
   up.y = ycos*zcos+xsin*ysin*zsin;
   up.z = -xcos*ysin;
   Vector temp = vect.cross(right);
   Vector np = solveScalers(right, up, vect, a-c);
   textureY = np.y;
   thirdX = np.x;

   d = -vect.dot(center);
   Vector n1 = up.cross(vect);
   Vector n2 = vect.cross(right);
   invD = 1.0 / right.dot(n1);
   n1x = n1.x; n1y = n1.y; n1z = n1.z;
   n2x = n2.x; n2y = n2.y; n2z = n2.z;
}

double Triangle::getIntersection(Ray ray){
   double time = Plane::getIntersection(ray);
   if(time==inf)
      return time;
   Vector C = ray.point+ray.vector*time-center;
   double dx = (C.x*n1x + C.y*n1y + C.z*n1z) * invD;
   double dy = (C.x*n2x + C.y*n2y + C.z*n2z) * invD;
   unsigned char tmp = (thirdX - dx) * textureY + (thirdX-textureX) * (dy - textureY) < 0.0;
   return((tmp!=(textureX * dy < 0.0)) || (tmp != (dx * textureY - thirdX * dy < 0.0)))?inf:time;
}


bool Triangle::getLightIntersection(Ray ray, double* fill){
   const double t = ray.vector.dot(vect);
   const double norm = vect.dot(ray.point)+d;
   const double r = -norm/t;
   if(r<=0. || r>=1.) return false;
   Vector C = ray.point+ray.vector*r-center;
   double dx = (C.x*n1x + C.y*n1y + C.z*n1z) * invD;
   double dy = (C.x*n2x + C.y*n2y + C.z*n2z) * invD;

   unsigned char tmp = (thirdX - dx) * textureY + (thirdX-textureX) * (dy - textureY) < 0.0;
   if ((tmp!=(textureX * dy < 0.0)) || (tmp != (dx * textureY - thirdX * dy < 0.0))) return false;

   if(texture->opacity>1-1E-6) return true;
   unsigned char temp[4];
   double amb, op, ref;
   texture->getColor(temp, &amb, &op, &ref,fix(dx/textureX-.5), fix(dy/textureY-.5));
   if(op>1-1E-6) return true;
   fill[0]*=temp[0]/255.;
   fill[1]*=temp[1]/255.;
   fill[2]*=temp[2]/255.;
   return false;
}
