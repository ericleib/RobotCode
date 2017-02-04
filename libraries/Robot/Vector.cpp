/*
  Vector.cpp - Library for handling 3D vectors.
  Created by Eric Leibenguth, 18 Feb 2016.
  Released into the public domain.
*/

#include <StandardCplusplus.h>
#include <serstream>
#include <math.h>

#include "Vector.h"


// Constructors

Vector::Vector(double x, double y, double z)
{
  set(x, y, z);
}

void Vector::set(double _x, double _y, double _z)
{
  x = _x;
  y = _y;
  z = _z;
}



// Operations

Vector Vector::add(const Vector &v) const
{
  Vector result = Vector(*this);
  result.add_(v);
  return result;
}

Vector Vector::sub(const Vector &v) const
{
  Vector result = Vector(*this);
  result.sub_(v);
  return result;
}

Vector Vector::mult(const double d) const
{
  Vector result = Vector(*this);
  result.mult_(d);
  return result;
}

Vector Vector::div(const double d) const
{
  Vector result = Vector(*this);
  result.div_(d);
  return result;
}

Vector Vector::cross(const Vector &v) const
{
  Vector result = Vector(*this);
  result.cross_(v);
  return result;
}

Vector Vector::normalize() const
{
  Vector result = Vector(*this);
  result.normalize_();
  return result;
}

Vector Vector::lerp(const Vector &v, double t) const
{
  Vector result = Vector(*this);
  result.lerp_(v, t);
  return result;
}

double Vector::dist(const Vector &v) const
{
  return sub(v).mag();
}

double Vector::dot(const Vector &v) const
{
  return x*v.x + y*v.y + z*v.z;
}

double Vector::mag() const{
  return sqrt(x*x + y*y + z*z);
}

double Vector::angle(const Vector &v) const{
  return acos(dot(v) / (mag() * v.mag()));
}

double Vector::signedAngle(const Vector &v, const Vector &ref) const{
  double d = cross(v).dot(ref);
  return angle(v) * (d>=0? 1 : -1);
}


// Operations in place

void Vector::add_(const Vector &v)
{
  set(x+v.x, y+v.y, z+v.z);
}

void Vector::sub_(const Vector &v)
{
  set(x-v.x, y-v.y, z-v.z);
}

void Vector::mult_(const double d)
{
  set(x*d, y*d, z*d);
}

void Vector::div_(const double d)
{
  mult_(1.0/d);
}

void Vector::cross_(const Vector &v)
{
  set(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}

void Vector::normalize_()
{
  div_(mag());
}

void Vector::lerp_(const Vector &v, double t)
{
  add_(t * (v - *this));
}

void Vector::set_mag(const double d){
  mult_(d / mag());
}


// Operator overloading

bool operator == (const Vector &v1, const Vector &v2){
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

bool operator != (const Vector &v1, const Vector &v2){
	return v1.x != v2.x || v1.y != v2.y || v1.z != v2.z;
}

Vector operator + (const Vector &v1, const Vector &v2){
	return v1.add(v2);
}

Vector operator - (const Vector &v1, const Vector &v2){
	return v1.sub(v2);
}

Vector operator - (const Vector &v){
	return v.mult(-1.0);
}

double operator * (const Vector &v1, const Vector &v2){
	return v1.dot(v2);
}

Vector operator * (const Vector &v, const double d){
	return v.mult(d);
}

Vector operator * (const double d, const Vector &v){
	return v.mult(d);
}

Vector operator / (const Vector &v, const double d){
	return v.div(d);
}

std::ostream &operator<<(std::ostream &os, const Vector &v)
{
    os << '[' << v.x << ',' << v.y << ',' << v.z << ']';
    return os;
}

const Vector Vector::EX = Vector(1.0, 0.0, 0.0);
const Vector Vector::EY = Vector(0.0, 1.0, 0.0);
const Vector Vector::EZ = Vector(0.0, 0.0, 1.0);


// Rotation class (quaternion-based composable rotation class for vectors)

Rotation::Rotation(double q0, double q1, double q2, double q3)
{
  set(q0, q1, q2, q3);
}

Rotation::Rotation(const Vector &axis, double angle)
{
  double coeff = sin(-0.5 * angle) / axis.mag();
  set(cos(-0.5 * angle), coeff * axis.x, coeff * axis.y, coeff * axis.z);
}

void Rotation::set(double q0_, double q1_, double q2_, double q3_){
  q0 = q0_;
  q1 = q1_;
  q2 = q2_;
  q3 = q3_;
}

Vector Rotation::rotate(const Vector &v) const{
  double s = q1 * v.x + q2 * v.y + q3 * v.z;
  Vector res(2 * (q0 * (v.x * q0 - (q2 * v.z - q3 * v.y)) + s * q1) - v.x,
             2 * (q0 * (v.y * q0 - (q3 * v.x - q1 * v.z)) + s * q2) - v.y,
             2 * (q0 * (v.z * q0 - (q1 * v.y - q2 * v.x)) + s * q3) - v.z);
  return res;
}

Rotation Rotation::compose(const Rotation &r) const{
  Rotation res(r.q0 * q0 - (r.q1 * q1 + r.q2 * q2 + r.q3 * q3),
               r.q1 * q0 + r.q0 * q1 + (r.q2 * q3 - r.q3 * q2),
               r.q2 * q0 + r.q0 * q2 + (r.q3 * q1 - r.q1 * q3),
               r.q3 * q0 + r.q0 * q3 + (r.q1 * q2 - r.q2 * q1));
  return res;
}