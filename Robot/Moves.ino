
const int DX_LEG=18, DY_LEG=19, DZ_LEG=20;

class SlideLeg : public Stand {
  
  Trajectory slide;
  
  public:
  
  SlideLeg(Robot & robot) : Stand(robot) {
    set(DX_LEG, 30.0);
    set(DY_LEG, 30.0);
    set(DZ_LEG, 10.0);
  }
  
  Vector getFootPosition(int i, double phase) const{
    if(i==3) return slide.point(fmod(phase-phase0, 1.0));
    return Walk::getFootPosition(i, phase);
  }
  
  double getPeriod(double phase) const{
    return 3.0;
  }
  
  void update() {
    Vector foot_ref = robot.legs[3].foot_ref + Vector(get(SHIFT_X), get(SHIFT_Y) + get(DY) * robot.legs[3].right_(), 0);
    Vector foot_up = foot_ref + Vector(0,0,get(DZ_LEG));
    Vector foot_up2 = foot_up + Vector(get(DX_LEG),0,0);
    Vector target = foot_up2 + Vector(0,get(DY_LEG),0);
  
    slide.addSegment(foot_ref, true);
    slide.addSegment(foot_ref, foot_up, foot_up2, target, false);
    slide.addSegment(target, foot_up2, foot_up, foot_ref, false);
    slide.setGroundRatio(0.1);
    Walk::update();
  }
};

void setSlideLegMoves(){

  Move_ stand(new Stand(robot));
  planner.addMove(stand, 5.0);

  Move_ stand2(new Stand(robot));
  stand2->set(SHIFT_X, 10);
  stand2->set(SHIFT_Y, 10);
  planner.addMoveWithTransient(stand2, 1.0, 0.5);

  Move_ slide(new SlideLeg(robot));
  slide->set(SHIFT_X, 10);
  slide->set(SHIFT_Y, 10);
  planner.addMove(slide, 1.0);

  Move_ stand3(new Stand(robot));
  stand3->set(SHIFT_X, 10);
  stand3->set(SHIFT_Y, 10);
  planner.addMove(stand3, 1.0);
  
  Move_ slide2(new SlideLeg(robot));
  slide2->set(SHIFT_X, 10);
  slide2->set(SHIFT_Y, 10);
  slide2->set(DX_LEG, 30);
  slide2->set(DY_LEG, 40);
  planner.addMove(slide2, 1.0);
  
  Move_ stand4(new Stand(robot));
  stand4->set(SHIFT_X, 10);
  stand4->set(SHIFT_Y, 10);
  stand4->set(HEIGHT, 70.0);
  stand4->set(PITCH, 5);
  stand4->set(ROLL, -5);
  planner.addMoveWithTransient(stand4, 2.0, 0.1);
  
  Move_ slide3(new SlideLeg(robot));
  slide3->set(SHIFT_X, 10);
  slide3->set(SHIFT_Y, 10);
  slide3->set(HEIGHT, 70.0);
  slide3->set(PITCH, 5);
  slide3->set(ROLL, -5);
  slide3->set(DX_LEG, 35);
  slide3->set(DY_LEG, 65);
  //slide3->set(DZ_LEG, 40);
  planner.addMove(slide3, 1.0);
  
  Move_ stand5(new Stand(robot));
  planner.addMoveWithTransient(stand5, 1.0, 1.0);

  move = stand;
}

