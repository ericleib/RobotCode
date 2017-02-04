/*
  Vector.h - Library for handling 3D vectors.
  Created by Eric Leibenguth, 18 Feb 2016.
  Released into the public domain.
*/
#include <StandardCplusplus.h>
#include <serstream>

#ifndef Vector_h
#define Vector_h


class Vector
{
  public:
    double x;
	double y;
	double z;
	
    Vector(double x, double y, double z);
	
    void set(double x, double y, double z);
	
    Vector add(const Vector &v) const;
    Vector sub(const Vector &v) const;
    Vector mult(const double d) const;
    Vector div(const double d) const;
	Vector cross(const Vector &v) const;
	Vector normalize() const;
	Vector lerp(const Vector &v, double t) const;
	double dist(const Vector &v) const;
	double dot(const Vector &v) const;
	double mag() const;
	double angle(const Vector &v) const;
	double signedAngle(const Vector &v, const Vector &ref) const;
	
    void add_(const Vector &v);
    void sub_(const Vector &v);
    void mult_(const double d);
    void div_(const double d);
	void cross_(const Vector &v);
	void normalize_();
	void lerp_(const Vector &v, double t);
	void set_mag(const double d);
		
	static const Vector EX, EY, EZ;
	
};

bool operator == (const Vector &v1, const Vector &v2);
bool operator != (const Vector &v1, const Vector &v2);
Vector operator + (const Vector &v1, const Vector &v2);
Vector operator - (const Vector &v1, const Vector &v2);
Vector operator - (const Vector &v);
double operator * (const Vector &v1, const Vector &v2);
Vector operator * (const Vector &v, const double d);
Vector operator * (const double d, const Vector &v);
Vector operator / (const Vector &v, const double d);
std::ostream &operator << (std::ostream &os, const Vector &v);


class Rotation
{
  public:
    Rotation(double q0, double q1, double q2, double q3);
    Rotation(const Vector &axis, double angle);
    void set(double q0, double q1, double q2, double q3);
    Vector rotate(const Vector &v) const;
    Rotation compose(const Rotation &r) const;
  
  private:
    double q0, q1, q2, q3;
};

#endif