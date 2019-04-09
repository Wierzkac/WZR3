/************************************************************************************************** 
    Obs³uga kwaternionów 
   
    Aby zrealizowaæ obrót nale¿y pomno¿yæ kwaternion orientacji przez kwaterion obrotu
    z lewej strony:
            
    qOrient = qRot*qOrient
    
    Uwaga! Zak³adam, ¿e kwaterniony qRot i qOrient s¹ d³ugoœci 1, inaczej trzeba jeszcze je normalizowaæ            
    
    Kwaternion mo¿e tez reprezentowaæ orientacjê obiektu, jeœli za³o¿ymy jak¹œ orientacjê
    pocz¹tkow¹ np. obiekt skierowany w kierunku osi 0x, z normaln¹ zgodn¹ z 0y. Wówczas 
    kwaternion odpowiada obrotowi od po³o¿enia poc¿¹tkowego. 
****************************************************************************************************/
#include <stdlib.h>
//#include "vector3D.h"
#include "quaternion.h"


kwaternion::kwaternion(float x,float y,float z,float w)

{
	this->x=x;
	this->y=y;
	this->z=z;
	this->w=w;
}


kwaternion::kwaternion()
{
	x=0;
	y=0;
	z=0;
	w=1;
}

// mno¿enie dwóch kwaternionów
kwaternion kwaternion::operator*(kwaternion q)
{
    double rx, ry, rz, rw;		// temp result	


	
	rx	= x*q.w + w*q.x - z*q.y + y*q.z;
	ry	= y*q.w + w*q.y - x*q.z + z*q.x;
	rz	= z*q.w + w*q.z - y*q.x + x*q.y;
	
	rw	= w*q.w - x*q.x - y*q.y - z*q.z;

	return kwaternion((float)rx,(float)ry,(float)rz,(float)rw);
}

// zamiana kwaterniona na reprezentacjê k¹towo-osiow¹
// dla wygody oœ i k¹t obrotu umieszczane s¹ w strukturze typu kwaternion 
kwaternion kwaternion::AsixAngle()
{
    Wektor3 v(x,y,z);

    if (v.dlugosc()==0) return kwaternion(0,0,0,1);
    else 
    {
        v=v.znorm();
		    float kat_obrotu = (w <= -1 ? 0 : (w >= 1 ? 0 : 2.0*acos(w)));
        return kwaternion(v.x,v.y,v.z,kat_obrotu);
    }
}

kwaternion kwaternion::operator~ ()
{
    return kwaternion(-x,-y,-z,w);
}
  
kwaternion kwaternion:: operator+= (kwaternion q)	// operator+= is used to add another Vector3D to this Vector3D.
{
	x += q.x;
	y += q.y;
	z += q.z;
	w += q.w;
    return *this;
}

kwaternion kwaternion:: operator+ (kwaternion q)	// operator+= is used to add another Vector3D to this Vector3D.
{
        return kwaternion(x+q.x,y+q.y,z+q.z,w+q.w);
}
  
kwaternion kwaternion:: operator- (kwaternion q)	// operator+= is used to add another Vector3D to this Vector3D.
{
        return kwaternion(x-q.x,y-q.y,z-q.z,w-q.w);
}  
  
kwaternion kwaternion::n() // licz normalna z obecnego wektora
{
float length;

	length=sqrt(x*x+y*y+z*z+w*w);
	if (length==0) return kwaternion(0,0,0,0);
	    else return kwaternion(x/length,y/length,z/length,w/length);
}

float kwaternion::l() // d³ugoœæ 
{
  return sqrt(x*x+y*y+z*z+w*w);
}	

kwaternion kwaternion::operator* (float value)  // mnozenie razy skalar
{
	return kwaternion(x*value,y*value,z*value,w*value);
}

kwaternion kwaternion::operator/ (float value)  // dzielenie przez skalar
{
	return kwaternion(x/value,y/value,z/value,w/value);
}
	


/*  obrót wektora z u¿yciem kwaterniona obrotu - do sprawdzenia
macierz obrotu:
M =  [ w^2 + x^2 - y^2 - z^2       2xy - 2wz                   2xz + 2wy
       2xy + 2wz                   w^2 - x^2 + y^2 - z^2       2yz - 2wx
       2xz - 2wy                   2yz + 2wx                   w^2 - x^2 - y^2 + z^2 ]
*/
Wektor3 kwaternion::obroc_wektor(Wektor3 V)       
{ 
    Wektor3 Vo;

    kwaternion p = kwaternion(V.x,V.y,V.z,0);
    kwaternion q = kwaternion(x,y,z,w);
    kwaternion pp = q*p*~q;
    Vo.x = pp.x;Vo.y = pp.y; Vo.z = pp.z;

    return Vo;    
}

// zamiana obrotu wyra¿onego reprezentacj¹ k¹towo-osiow¹ na reprezentacjê kwaternionow¹  
kwaternion AsixToQuat(Wektor3 v, float angle)
{
    float x,y,z,w;			// temp vars of vector
    double rad, scale;		// temp vars

	if (v.dlugosc()==0) return kwaternion();
	
	v = v.znorm();
	rad = angle;
	scale = sin(rad/2.0);


	w = (float)cos(rad/2.0);

	x = float(v.x * scale);
	y = float(v.y * scale);
	z = float(v.z * scale);

	return kwaternion(x,y,z,w);
}
