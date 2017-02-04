
#include <StandardCplusplus.h>
#include <serstream>
#include "smart_ptr.h"

#include "Arduino.h"
#include "Vector.h"
#include "Trajectory.h"
#include <Servo.h>

#ifndef Leg_h
#define Leg_h

struct Robot;	// Forward declaration, to break circular dependency

class Leg {
  public:

	Vector foot, knee, shoulder, slot, foot_ref;
	double theta, phi, psi;
	
	Leg(Robot &robot, const Vector &slot, bool right);
	void calib(double coef_1_0, double coef_1_1, double coef_2_0, double coef_2_1, double coef_3_0, double coef_3_1);
	void attach(int pin1, int pin2, int pin3);
	void resolve();
	
	int right_();
	
  private:
    const Robot & robot;
	const bool right;         // Right or left leg
	const Servo lateral, contact, pendulum;
	double coef_1_0, coef_1_1, coef_2_0; // Coefficients of calibration for servos
	double coef_2_1, coef_3_0, coef_3_1; // Coefficients of calibration for servos

	void computeShoulder();
	void computeKnee();
	void computeAngles();
	void command();
	
  friend std::ostream &operator << (std::ostream &os, const Leg &l);
};


struct Frame {
  double width, length;
  Vector ref;	  // Reference point for speed vector
  Rotation attitude;
  Vector xf, yf, zf;
  
  Frame();
  void setAttitude(double h, double pitch, double roll);
  Vector getSlotPosition(int i);
  Vector getPoint(double x, double y, double z);  

};

class Move {	// Abstract class for movements
  
  public:
    Move(Robot & robot);
	virtual void to_string(std::ostream &os) const;
	virtual double getPeriod(double phase) const = 0;
	virtual Vector getSpeed(double phase) const = 0;
	virtual double getRotation(double phase) const = 0;
	virtual double getHeight(double phase) const = 0;
	virtual double getPitch(double phase) const = 0;
	virtual double getRoll(double phase) const = 0;
	virtual Vector getFootPosition(int i, double phase) const = 0;
    virtual void update();
    
	static Trajectory makeStep(const Vector &foot, double dx, double dr, double h, double k);
  
    Vector getSpeed(double phase, const Vector &point) const;
   
    Robot & robot; // Reference point for Speed
	
	double phase0, duration; // start phase and duration (in nb of periods)
  
	virtual void set(int i, double value);
	double get(int i) const;
	
	std::vector<double> parameters;
  
};

std::ostream &operator << (std::ostream &os, const Move &m);

typedef smart_ptr<Move> Move_;

struct Robot {
	
  Frame frame;
  double shoulderWidth;   // Shoulder lateral shift
  double upperLegLength;  // Upper segment length
  double lowerLegLength;  // Lower segment length
  std::vector<Leg> legs;  // Leg container
  
  void init();
  void apply(Move_ move, double phase);
};

std::ostream &operator << (std::ostream &os, const Robot &r);

#endif