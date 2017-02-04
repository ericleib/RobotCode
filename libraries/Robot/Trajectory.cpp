/*
  Trajectory.cpp - Library for handling Trajectories.
  Created by Eric Leibenguth, 18 Feb 2016.
  Released into the public domain.
*/

#include <StandardCplusplus.h>
#include <serstream>
#include <vector>
#include <math.h>

#include "Vector.h"
#include "Trajectory.h"


// Segment

void Segment::to_string(std::ostream &os) const{
  os << "<Segment>";
}

std::ostream &operator << (std::ostream &os, const Segment &s)
{
  s.to_string(os);
  return os;
}



// Bezier curves

Bezier2::Bezier2(const Vector &p1_, const Vector &cp_, const Vector &p2_, bool ground_):
	p1(p1_), 
	cp(cp_),
	p2(p2_), 
	v1(2 * p1 - 4 * cp + 2 * p2), 
	v2(-2 * p1 + 2 * cp)
{
  //std::cout << "New Bezier: " << p1_ << " to " << p2_ << std::endl;
  ground = ground_;
  phase = 0.0;
  length = compute_length();
}

Vector Bezier2::point(double t) const{
  return v1 * (0.5*t*t) + v2*(t) + p1;
}

Vector Bezier2::dpoint(double t) const{
  return v1 * t + v2;
}

Vector Bezier2::pointLin(double t) const{
  double l = t * length;   // length that must be walked on the bezier
  double l1 = lengths[0];
  for(int i=1; i<STEPS+1; i++){
	double l2 = lengths[i];
	if(l1<=l && l<=l2){
	  return point( (i-1 + (l-l1)/(l2-l1)) / STEPS );
	}
	l1 = l2;
  }
  return Vector(sqrt(-1),sqrt(-1),sqrt(-1));
}

double Bezier2::compute_length(){
  lengths[0] = 0.0;
  for(int i=0; i<STEPS; i++){
    lengths[i+1] = lengths[i] + dpoint(((double)i)/STEPS).mag() / STEPS;  // Discretize the trajectory
  }
  return lengths[STEPS];
}

void Bezier2::to_string(std::ostream &os) const{
  os << "<Bezier quad: " << p1 << ", " << cp << ", " << p2 << " (length: " << length << ", phase: " << phase << ")>";
}



Bezier::Bezier(const Vector &p1, const Vector &cp1, const Vector &cp2_, const Vector &p2, bool ground_):
	Bezier2(p1, cp1, p2, ground_),
	cp2(cp2_),	
	v3(-3 * p1 + 3 * cp1)
{
  //std::cout << "New Bezier: " << p1_ << " to " << p2_ << std::endl;
  v1 = -1 * p1 + 3 * cp - 3 * cp2 + p2; 
  v2 = 3 * p1 -6 * cp + 3 * cp2;
  phase = 0.0;
  length = compute_length();
}

Vector Bezier::point(double t) const{
  return v1 * (t*t*t) + v2*(t*t) + v3 * t + p1;
}

Vector Bezier::dpoint(double t) const{
  return v1 * (3*t*t) + v2*(2*t) + v3;
}

void Bezier::to_string(std::ostream &os) const{
  os << "<Bezier cub: " << p1 << ", " << cp << ", " << cp2 << ", " << p2 << " (length: " << length << ", phase: " << phase << ")>";
}



// Line

Line::Line(const Vector &v1, const Vector &v2, bool ground_) : p1(v1),  p2(v2) {
  ground = ground_;
  length = v1.dist(v2);
}

Vector Line::pointLin(double t) const{
  return p1.lerp(p2,t);
}

void Line::to_string(std::ostream &os) const{
  os << "<Line: " << p1 << ", " << p2 << ")>";
}



// Point

Point::Point(const Vector &p_, bool ground_) : p(p_) {
  ground = ground_;
  length = 0;
}

Vector Point::pointLin(double t) const{
  return p * 1;
}

void Point::to_string(std::ostream &os) const{
  os << "<Point: " << p << ")>";
}



// Oscillation

Oscillation::Oscillation(const Vector &p1_, const Vector &p2_, double dphase_, double freq_) : p1(p1_), p2(p2_), v(p2_-p1_), dphase(dphase_), freq(freq_){}

Oscillation::Oscillation(const Vector &p1_, const Vector &p2_, double dphase) : Oscillation(p1_, p2_, dphase, 1.0){}

Oscillation::Oscillation(const Vector &p1_, const Vector &p2_) : Oscillation(p1_, p2_, 0.0){}

Vector Oscillation::pointLin(double ph) const{
  return p1 + v * (0.5 - 0.5 * cos((dphase + freq*ph) * 2 * M_PI));
}

Vector Oscillation::speed(double ph) const {
  return v * (-M_PI * freq * sin((dphase + freq*ph) * 2 * M_PI));
}

void Oscillation::to_string(std::ostream &os) const{
  os << "<Oscillation: " << p1 << ", " << p2 << ")>";
}



// Trajectory

Trajectory::Trajectory(): length(0.0), length_grd(0.0) {}

void Trajectory::addSegment(const Vector &p1, const Vector &cp1, const Vector &cp2, const Vector &p2, bool ground){
  addSegment(Segment_(new Bezier(p1, cp1, cp2, p2, ground)));
}

void Trajectory::addSegment(const Vector &p1, const Vector &cp, const Vector &p2, bool ground){
  addSegment(Segment_(new Bezier2(p1, cp, p2, ground)));
}

void Trajectory::addSegment(const Vector &p1, const Vector &p2, bool ground){
  addSegment(Segment_(new Line(p1, p2, ground)));
}

void Trajectory::addSegment(const Vector &p, bool ground){
  addSegment(Segment_(new Point(p, ground)));
}

void Trajectory::addSegment(const Segment_ &s){
  segments.push_back(s);
  length += s->length;
  if(s->ground) length_grd += s->length;
}

void Trajectory::setGroundRatio(double k){
  if(length==0){
	segments[0]->phase = 1.0;    // Case of degenerate trajectory
  }else if(length_grd==0){  			// Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
	for(std::vector<Segment_>::iterator it = segments.begin(); it != segments.end(); ++it) {
      Segment_ & s = *it;
      s->phase = s->ground? k : (1 - k) * s->length / length;
    }
  }else{  			// Set the phase for each segment, in function of the ratio (speed is assumed constant on respectively air and ground)
	for(std::vector<Segment_>::iterator it = segments.begin(); it != segments.end(); ++it) {
      Segment_ & s = *it;
      s->phase = s->length * (s->ground? k / length_grd : (1 - k) / (length - length_grd));
    }
  }
}

Vector Trajectory::point(double phase) const {
  double p = 0.0;
  for(std::vector<Segment_>::iterator it = segments.begin(); it != segments.end(); ++it) {
    Segment_ & s = *it;
    p += s->phase;
    if(phase <= p)
      return s->pointLin(1-(p-phase)/ s->phase);
  }
  return Vector(0,0,0);
}

std::ostream &operator << (std::ostream &os, const Trajectory &t)
{
  os << "Trajectory (length:" << t.length << ", length_grd:" << t.length_grd << "):" << std::endl;
  for(std::vector<Segment_>::iterator it = t.segments.begin(); it != t.segments.end(); ++it) {
    Segment_ & s = *it;
    os << *s << std::endl;
  }
  return os;
}
