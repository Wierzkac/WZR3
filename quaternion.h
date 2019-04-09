
#ifndef _QUATMATHVECT__H
#define _QUATMATHVECT__H

#define M_PI 3.14159265358979323846  
#define M_RAD 180/M_PI

#include <math.h>

#ifndef _WEKTOR__H 
  #include "vector3D.h"
#endif


struct kwaternion
{
       float x,y,z,w;

       kwaternion(float x1,float y1,float z1,float w1);
       kwaternion();

       kwaternion AsixAngle();        // zamiana kwaterniona na reprezentacjê k¹towo-osiow¹
       Wektor3 obroc_wektor(Wektor3 w);      // obrót wektora z u¿yciem kwaterniona obrotu

      kwaternion operator*(kwaternion q); // iloczyn vectorowy
      kwaternion operator~ ();
      kwaternion operator+=(kwaternion q);  // dodanie vec+vec
      kwaternion operator+ (kwaternion q);  // dodanie vec+vec
      kwaternion operator- (kwaternion q);  // odejmowanie vec-vec
      kwaternion n(); // licz normalna z obecnego kwaternionu 
      float l(); // dlugosc kwateriona 
      kwaternion operator* (float value); // mnozenie razy skalar
      kwaternion operator/ (float value); // dzielenie przez skalar

  
};

kwaternion AsixToQuat(Wektor3 v,float angle);

#endif
