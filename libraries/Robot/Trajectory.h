/*
  Trajectory.h - Library for handling Trajectories.
  Created by Eric Leibenguth, 18 Feb 2016.
  Released into the public domain.
*/

#include <StandardCplusplus.h>
#include <serstream>
#include <vector>
#include "smart_ptr.h"

#include "Vector.h"

#ifndef Trajectory_h
#define Trajectory_h


// Segment

class Segment {
  public:	
	double length, phase;
	bool ground;
	
	virtual Vector pointLin(double t) const = 0;
	virtual void to_string(std::ostream &os) const;
};

std::ostream &operator << (std::ostream &os, const Segment &s);

typedef smart_ptr<Segment> Segment_;


// Bezier

const int STEPS = 10;

class Bezier2 : public Segment
{
  public:
	Bezier2(const Vector &p1, const Vector &cp, const Vector &p2, bool ground);
	virtual Vector point(double t) const;
	virtual Vector dpoint(double t) const;
	Vector pointLin(double t) const;
	void to_string(std::ostream &os) const;
	
	Vector p1, cp, p2;
	Vector v1, v2;
	double lengths[STEPS+1];
    double compute_length();

};

class Bezier : public Bezier2
{
  public:
	Bezier(const Vector &p1, const Vector &cp1, const Vector &cp2, const Vector &p2, bool ground);
	Vector point(double t) const;
	Vector dpoint(double t) const;
	void to_string(std::ostream &os) const;
	
  private:
	Vector cp2, v3;

};


// Point

class Point : public Segment
{
  public:
    Point(const Vector &p, bool ground);
    Vector pointLin(double t) const;
	void to_string(std::ostream &os) const;
  
  private:
    Vector p;  
};


// Line

class Line : public Segment
{
  public:
    Line(const Vector &v1, const Vector &v2, bool ground);
    Vector pointLin(double t) const;
	void to_string(std::ostream &os) const;
	
  private:
    Vector p1, p2;  
};


// Oscillation

class Oscillation : public Segment
{
  public:
    Oscillation(const Vector &p1, const Vector &p2, double dphase, double freq);
    Oscillation(const Vector &p1, const Vector &p2, double phase);
    Oscillation(const Vector &p1, const Vector &p2);
    Vector pointLin(double phase) const;
    Vector speed(double phase) const;
	void to_string(std::ostream &os) const;
  
  private:
    Vector p1, p2, v;
    double dphase, freq;
};



// Trajectory

class Trajectory {
  public:
    double length_grd, length;
	Trajectory();
	void addSegment(const Vector &p1, const Vector &cp1, const Vector &cp2, const Vector &p2, bool ground);
	void addSegment(const Vector &p1, const Vector &cp, const Vector &p2, bool ground);
	void addSegment(const Vector &p1, const Vector &p2, bool ground);
	void addSegment(const Vector &p, bool ground);
	void addSegment(const Segment_ &b);
	void setGroundRatio(double k);
	Vector point(double phase) const;
	friend std::ostream &operator << (std::ostream &os, const Trajectory &b);
  
  private:
	std::vector<Segment_> segments;	// Pointers are used for polymorphism
};


#endif