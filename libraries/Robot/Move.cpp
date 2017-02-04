

#include <StandardCplusplus.h>
#include <serstream>
#include "smart_ptr.h"
#include <math.h>

#include "Vector.h"
#include "Trajectory.h"
#include "Components.h"

#include "Move.h"


// Abstract move class

Move::Move(Robot & r) : robot(r) {}

void Move::update(){}

Trajectory Move::makeStep(const Vector &foot, double dx, double dr, double dz, double k)
{
  Trajectory t;
  if(dx!=0 || dz !=0){
	double dy = dx * tan(dr);
    t.addSegment(foot + Vector(dx, dy, 0),   foot + Vector(-dx, -dy, 0), true);
    t.addSegment(foot + Vector(-dx, -dy, 0), foot + Vector(-dx, -dy, dz), foot + Vector(0, 0, dz), false);
    t.addSegment(foot + Vector(0, 0, dz),    foot + Vector(dx, dy, dz),   foot + Vector(dx, dy, 0), false);
  }else{
    t.addSegment(foot, true);
  }
  t.setGroundRatio(k);
  return t;
}

Vector Move::getSpeed(double phase, const Vector &point) const {
  return getSpeed(phase) + (robot.frame.ref - point).cross(getRotation(phase) * Vector::EZ);  // Speed of the point = Speed(CG) + point->cg ^ Rotation
}

void Move::to_string(std::ostream &os) const{
  os << "<Abstract Move>";
}

void Move::set(int i, double value){
  if(i >= parameters.size())
	parameters.resize(i+1);
  parameters[i] = value;
}

double Move::get(int i) const {
  return parameters[i];
}

std::ostream &operator << (std::ostream &os, const Move &m)
{
  m.to_string(os);
  return os;
}



// Walk move

Walk::Walk(Robot & robot) : Walk(robot, 0.0){}
Walk::Walk(Robot & robot, double rotation): Walk(robot, rotation, 0.0){}
Walk::Walk(Robot & robot, double rotation, double dr) : 
Move(robot), osc(Oscillation(Vector(0,0,0), Vector(0,0,0)))
{
  set(ROTATION, rotation);
  set(DR, dr);
  set(SPEED, 40.0);
  set(HEIGHT, 85.0);
  set(SHIFT_X, -3.0);
  set(SHIFT_Y, -2.0);
  set(DX, 30.0);
  set(DY, 20.0);
  set(DZ, 20.0);
  set(K_GROUND, 0.9);
  set(PHASE_OSC, 0.18);
  set(AMPL_OSC, 10.0);
  set(PITCH, 0.0);
  set(ROLL, 0.0);
  set(PHASE1, 0.25);
  set(PHASE2, 1.00);
  set(PHASE3, 0.75);
  set(PHASE4, 0.50);
}

void Walk::to_string(std::ostream &os) const{
  if(get(ROTATION)==0.0){
	if(get(SPEED)==0.0){
      os << "Standing";
	}else if(get(DR)==0.0){
      os << "Straight walk";
	}else{
      os << "Crab walk";
	}
  }else{
	os << "Turn " << get(ROTATION) << "deg/s";
  }
  //os << std::endl << t[0];
}

double Walk::getPeriod(double phase) const {  // Period of the movement
  if(get(DX)==0.0) return 1.0; // Case of degenerate trajectory
  return get(DX) / (0.5 * get(K_GROUND) * fabs(get(SPEED)));
}

Vector Walk::getRawSpeed() const { return Vector(get(SPEED), get(SPEED)*tan(RAD * get(DR)), 0.0); }
  
Vector Walk::getSpeed(const Vector &point) const{    
  return getRawSpeed() + (robot.frame.ref - point).cross(Vector(0, 0, RAD * get(ROTATION)));  // Speed of the point = Speed(CG) + point->cg ^ Rotation
}

Vector Walk::getSpeed(double phase) const{ return getRawSpeed(); } // Not needed: .add(osc.speed(p).mult(1.0/getPeriod(p)))
  
double Walk::getRotation(double phase) const{ return RAD * get(ROTATION); }

double Walk::getHeight(double phase) const{ return get(HEIGHT); }

double Walk::getPitch(double phase) const{ return RAD * get(PITCH); }

double Walk::getRoll(double phase) const{ return RAD * get(ROLL); }

Vector Walk::getFootPosition(int i, double phase) const {
  if(get(SPEED)<0){
    double ph = 1.0 - fmod(phase - dphase(i), 1.0);
    return t[i].point(ph) + osc.pointLin(1.0-phase);
  }else{
    double ph = fmod(phase + dphase(i), 1.0);
    return t[i].point(ph) + osc.pointLin(phase);
  }
}

double Walk::dphase(int i) const {
  return i==0? get(PHASE1) : i==1? get(PHASE2) : i==2? get(PHASE3) : get(PHASE4);
}
  
void Walk::update(){
  osc = Oscillation(Vector(0,0,0), Vector(0,0,0)); // No oscillation to define the nominal trajectories of the legs (for turns)
  for(int i=0; i<robot.legs.size(); i++){      // for each leg
    Vector foot_ref = robot.legs[i].foot_ref + Vector(get(SHIFT_X), get(SHIFT_Y) + get(DY) * robot.legs[i].right_(), 0);
    Vector v = getSpeed(foot_ref);  // Local speed of the leg : Problem: this includes the oscillations...
    double dx = get(DX) * (get(ROTATION)==0.0 ? 1.0 : v.x / get(SPEED));
    double dr = get(ROTATION)==0.0 ? RAD * get(DR) : atan(v.y/v.x); // warning: some particular cases where rotation!=0 and v.x = 0
    t[i] = makeStep(foot_ref, dx, dr, get(DZ), get(K_GROUND));  // Left forward
  }
  osc = Oscillation(Vector(0,-get(AMPL_OSC),0), Vector(0,get(AMPL_OSC),0), get(PHASE_OSC), 1.0);  // Now we add oscillations
}




// Standing

Stand::Stand(Robot & robot) : Walk(robot) {
  set(SPEED, 0.0);
  set(DX, 0.0);
  set(DZ, 0.0);
  set(AMPL_OSC, 0.0);
}
  
  
  
// Transient move  
  
WalkTransient::WalkTransient(Robot & robot, const Move_ & m1, const Move_ & m2):
Walk(robot), move1(m1), move2(m2){}


void WalkTransient::to_string(std::ostream &os) const{
  move1->to_string(os);
  os << " > ";
  move2->to_string(os);
}

void WalkTransient::setParameters(double phase){
  if(phase_ != phase){
    phase_ = phase;
    double phase_in = (phase-phase0)/duration;
    for(int i=0; i<parameters.size(); i++)
	  Walk::set(i, (1.0-phase_in) * move1->parameters[i] + phase_in * move2->parameters[i]);
    update(); // Recompute move's trajectories
  }
}

double WalkTransient::getPeriod(double phase) const{
  setParameters(phase);
  return Walk::getPeriod(phase);
}

Vector WalkTransient::getSpeed(double phase) const{
  setParameters(phase);
  return Walk::getSpeed(phase);
}

double WalkTransient::getRotation(double phase) const{
  setParameters(phase);
  return Walk::getRotation(phase);
}

double WalkTransient::getHeight(double phase) const{
  setParameters(phase);
  return Walk::getHeight(phase);
}

double WalkTransient::getPitch(double phase) const{
  setParameters(phase);
  return Walk::getPitch(phase);
}

double WalkTransient::getRoll(double phase) const{
  setParameters(phase);
  return Walk::getRoll(phase);
}

Vector WalkTransient::getFootPosition(int i, double phase) const{
  setParameters(phase);
  return Walk::getFootPosition(i, phase);
}


// Piloted move

PilotedMove::PilotedMove(Robot & robot): WalkTransient(robot, Move_(new Stand(robot)), Move_(new Stand(robot))) {}

void PilotedMove::to_string(std::ostream &os) const{
  os << "Piloted Move (";
  WalkTransient::to_string(os);
  os << ")";
  //os << std::endl;
  //Walk::to_string(os);
  //os << std::endl;
}

void PilotedMove::setParameters(double phase){
  lastphase_ = phase;	// Keep the phase in memory in case of a set()
  WalkTransient::setParameters(min(phase, phase0 + duration));
}
  
void PilotedMove::set(int i, double value){
  move1->parameters = parameters;
  move2->parameters[i] = value;
  //std::cout << "SET " << i << " " << value << std::endl<< std::endl;
  phase0 = lastphase_; // phase_;	// Does not work, as we do not allow phase_ to exceed phase0+duration...
}


// Move planner

void MovePlanner::addMove(const Move_ & move, double duration) {  // Moves are added at the beginning
  move->duration = duration;
  move->phase0 = moves.size() == 0 ? 0.0 : moves.back()->phase0 + moves.back()->duration;
  move->update();  // init trajectories, taking start phase into account
  moves.push_back(move);
}

void MovePlanner::addMove(const Move_ & move){ addMove(move, 0.0); }

void MovePlanner::addMoveWithTransient(const Move_ & move, double transientDuration, double duration) {  // Moves are added at the beginning
  Move_ last = moves.back();
  Move_ trans(new WalkTransient(move->robot, last, move));
  addMove(trans, transientDuration);
  addMove(move, duration);
}

Move_ MovePlanner::getMove(double phase) const{  // Moves are retrieved in function of phase
  Move_ move = moves[0];
  for(std::vector<Move_>::iterator it = moves.begin(); it != moves.end(); ++it) {
    Move_ & m = *it;
    if(m->phase0 > phase)
      break;
    move = m;
  }
  return move;
}

std::ostream &operator << (std::ostream &os, const MovePlanner &m)
{
  os << "Move planning:" << std::endl;
  for(std::vector<Move_>::iterator it = m.moves.begin(); it != m.moves.end(); ++it) {
    Move_ & move = *it;
    move->to_string(os);
	os << " from " << move->phase0 << " to " << (move->phase0+move->duration) << std::endl;
  }
}


