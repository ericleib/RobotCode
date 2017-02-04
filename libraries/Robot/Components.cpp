
#include <StandardCplusplus.h>
#include <serstream>
#include <math.h>

#include "Arduino.h"
#include "Vector.h"
#include "Trajectory.h"
#include <Servo.h>
#include "Components.h"


Leg::Leg(Robot &bot, const Vector &s, bool r):
robot(bot), foot_ref(s), slot(Vector(0,0,0)), knee(Vector(0,0,0)), shoulder(Vector(0,0,0)), foot(Vector(0,0,0)), right(r)
{}

void Leg::calib(double c1, double c2, double c3, double c4, double c5, double c6){
  coef_1_0 = c1;
  coef_1_1 = c2;
  coef_2_0 = c3;
  coef_2_1 = c4;
  coef_3_0 = c5;
  coef_3_1 = c6;
}
	
void Leg::attach(int pin1, int pin2, int pin3){
  lateral.attach(pin1) ;  // Lateral
  pendulum.attach(pin2) ;  // Pendulum
  contact.attach(pin3) ;  // Contact
}

void Leg::resolve(){
  computeShoulder();
  computeKnee();
  computeAngles();
  command();
}


void Leg::computeShoulder(){
  Vector slot_foot = foot - slot;  // Vector from slot to foot
  slot_foot = slot_foot + robot.frame.xf * (-slot_foot.dot(robot.frame.xf));  //Projection on plane of rotation of shoulder
  double a1 = acos(robot.shoulderWidth / slot_foot.mag()); // always positive : angle between slot->foot and slot->shoulder
  slot_foot.set_mag(robot.shoulderWidth);
  Vector slot_shoulder = Rotation(robot.frame.xf, a1*right_()).rotate(slot_foot);  // Rotate by a1 and resize to transform slot->foot into slot->shoulder
  shoulder = slot + slot_shoulder;
  //std::cout <<  slot << " " << shoulder << std::endl;
}

void Leg::computeKnee(){  // Computes the intersection of two circles
  double d = foot.dist(shoulder);
  double a = (pow(robot.lowerLegLength,2) - pow(robot.upperLegLength,2) + d*d) / (2 * d);
  double h = sqrt(pow(robot.lowerLegLength,2) - a*a);
  //std::cout << foot << " " << shoulder << std::endl;
  //std::cout << a << " " << d << " " << h << std::endl;
  //std::cout << foot << " " << (shoulder - foot) << " " << (a/d) << " " << (shoulder - foot) * (a/d) << " " << foot + (shoulder - foot) * (a/d) << std::endl;
  Vector pt = foot + (shoulder - foot) * (a/d);
  //std::cout << (shoulder - slot) << " " << (shoulder - slot).normalize() << " " << (shoulder - slot).normalize() * (right? 1 : -1) << " " << right << std::endl;
  Vector normal = (shoulder - slot).normalize() * right_();
  knee = pt + (shoulder - foot).cross(normal) * (h/d);
  //std::cout << pt << " " << normal << " " << knee << std::endl;
}

void Leg::computeAngles(){
  Vector yf = robot.frame.yf * right_();
  theta = yf.signedAngle(shoulder - slot, robot.frame.xf);
  phi = (-robot.frame.xf).angle(knee - shoulder);
  psi = (knee - shoulder).angle(foot - knee);
}

void Leg::command(){
  // Convert into pulse
  double mtheta = coef_1_0 * degrees(theta) + coef_1_1 ;
  double mphi =   coef_2_0 * degrees(phi)   + coef_2_1 ;
  double mpsi =   coef_3_0 * degrees(psi)   + coef_3_1 ;
  //std::cout << mtheta << " " << mphi << " " << mpsi << std::endl;
  lateral.writeMicroseconds(mtheta);
  pendulum.writeMicroseconds(mphi);
  contact.writeMicroseconds(mpsi);
}

int Leg::right_(){
  return right ? 1 : -1;
}

std::ostream &operator << (std::ostream &os, const Leg &l)
{
  os << "Leg (right:" << l.right << ", slot:" << l.foot_ref << ")";
  return os;
}




Frame::Frame() : ref(Vector(0,0,0)), xf(Vector::EX), yf(Vector::EY), zf(Vector::EZ), attitude(Rotation(Vector::EX, 0.0)) {}

void Frame::setAttitude(double h, double pitch, double roll){
  ref = Vector(0.5*length, 0.5*width, h);
  attitude = Rotation(Vector::EY, pitch).compose(Rotation(Vector::EX, roll));
  xf = attitude.rotate(Vector::EX);
  yf = attitude.rotate(Vector::EY);
  zf = attitude.rotate(Vector::EZ);
}

Vector Frame::getSlotPosition(int i){
  switch(i){
    case 0: return getPoint(-0.5*length, -0.5*width, 0);
    case 1: return getPoint( 0.5*length, -0.5*width, 0);
    case 2: return getPoint(-0.5*length,  0.5*width, 0);
    default: return getPoint(0.5*length,  0.5*width, 0);
  }
}

Vector Frame::getPoint(double x, double y, double z){
  return ref + attitude.rotate(Vector(x,y,z));
}
  
  
  

void Robot::init(){
  frame.setAttitude(85, 0, 0);	// Note: unfortunate that we have to initialize the robot with a given height
  legs.push_back(Leg(*this, Vector(0, 			 0, 		  0), false));
  legs.push_back(Leg(*this, Vector(frame.length, 0, 		  0), false));
  legs.push_back(Leg(*this, Vector(0, 			 frame.width, 0), true));
  legs.push_back(Leg(*this, Vector(frame.length, frame.width, 0), true));
}

void Robot::apply(Move_ move, double phase){
  frame.setAttitude(move->getHeight(phase), move->getPitch(phase), move->getRoll(phase));
  for(int i=0; i<4; i++){  // For each leg
    legs[i].slot = frame.getSlotPosition(i);
    legs[i].foot = move->getFootPosition(i, phase);  // Trajectory planning
    legs[i].resolve();  // Inverse kinematics & CG
  }
}
  
std::ostream &operator << (std::ostream &os, const Robot &robot)
{
  os << "Robot: " << std::endl;
  os << "Ref: " << robot.frame.ref << std::endl;
  os << robot.legs[0] << std::endl << robot.legs[1] << std::endl << robot.legs[2] << std::endl << robot.legs[3] << std::endl;
  return os;
}
