
#include <StandardCplusplus.h>
#include <serstream>
#include "smart_ptr.h"

#include "Vector.h"
#include "Trajectory.h"
#include "Components.h"

#ifndef Move_h
#define Move_h


const double RAD = M_PI / 180.0;
const double DEG = 1.0 / RAD;
const int SPEED=0, SHIFT_X=1, SHIFT_Y=2, DX=3, DY=4, DZ=5, DR=6, K_GROUND=7, ROTATION=8, PHASE_OSC=9, AMPL_OSC=10, PHASE1=11, PHASE2=12, PHASE3=13, PHASE4=14, PITCH=15, ROLL=16, HEIGHT=17;

class Walk : public Move {

  public:
    Walk(Robot & robot);
    Walk(Robot & robot, double rotation);
    Walk(Robot & robot, double rotation, double dr);
  
	virtual void to_string(std::ostream &os) const;
	virtual double getPeriod(double phase) const;
	Vector getSpeed(const Vector & point) const;
	virtual Vector getSpeed(double phase) const;
	Vector getRawSpeed() const;
	virtual double getRotation(double phase) const;
	virtual double getHeight(double phase) const;
	virtual double getPitch(double phase) const;
	virtual double getRoll(double phase) const;
	virtual Vector getFootPosition(int i, double phase) const;
    void update();
		
  private:
	Trajectory t[4];
	Oscillation osc;
	double dphase(int i) const;
    
};

class Stand : public Walk {
  public:
    Stand(Robot & robot);
};

class WalkTransient : public Walk {
  
  public:
    WalkTransient(Robot & robot, const Move_ & move1, const Move_ & move2);
	
	void to_string(std::ostream &os) const;
	double getPeriod(double phase) const;
	Vector getSpeed(double phase) const;
	double getRotation(double phase) const;
	double getHeight(double phase) const;
	double getPitch(double phase) const;
	double getRoll(double phase) const;
	Vector getFootPosition(int i, double phase) const;
    virtual void setParameters(double phase);    

    Move_ move1, move2;
	
  private:
    double phase_ = -1.0;
};

class PilotedMove : public WalkTransient {
  
  public:
	PilotedMove(Robot & robot);
  
	void to_string(std::ostream &os) const;
    void setParameters(double phase);  // Overrides WalkTransient's method
  
	void set(int i, double value);	// Overrides Move's method
  
  private:
	double lastphase_;
};

class MovePlanner {
  
  private:
    std::vector<Move_> moves;   // List of planned moves
  
  public:
    void addMove(const Move_ & move, double duration);
    void addMove(const Move_ & move);
    void addMoveWithTransient(const Move_ & move, double transientDuration, double duration);
    Move_ getMove(double phase) const;
	
  friend std::ostream &operator << (std::ostream &os, const MovePlanner &m);
};

#endif