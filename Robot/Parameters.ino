
// Geometrical parameters, hardware parameters, calibration of the robot
void initRobot(){

  robot.frame.width = 43.75 ;        // Width of frame
  robot.frame.length = 121.5 ;       // Length
  robot.shoulderWidth = 25.0;  // Shoulder lateral shift
  robot.upperLegLength = 53.0; // Upper segment length
  robot.lowerLegLength = 53.0; // Lower segment length
  robot.init();                // Initialize legs & reference
  
  robot.legs[0].calib(11.21131038, 1501.586231, -11.18267531,  2146.039388, 10.61884392, 519.9931446);
  robot.legs[1].calib(-11.44937739, 1481.462664, -11.49389072,  2187.343735, 11.27957227, 510.298769);
  robot.legs[2].calib(11.31410927, 1441.657104, 11.5150545,  714.9550326, -11.29913691,  2416.266045);
  robot.legs[3].calib(-11.17394284, 1483.149462, 11.768816, 747.1102078, -11.63080146,  2433.721161);
  
  robot.legs[0].attach(8, 9, 10);
  robot.legs[1].attach(2, 3, 4);
  robot.legs[2].attach(11, 12, 13);
  robot.legs[3].attach(5, 6, 7);
  
}



// Move planning of the robot
void initPlanner(){

  setPiloted();

}


void setPiloted(){
  
  Move_ piloted(new PilotedMove(robot));
  planner.addMove(piloted, 2.5); // Piloted move with interpolation period of 2.5*T

  move = piloted;
  
}

void setForwardToBackward(){

  Move_ walk(new Walk(robot));
  walk->set(DX, 40.0);
  walk->set(SPEED, 30.0);
  planner.addMove(walk, 2.0);

  Move_ walk2(new Walk(robot));
  walk2->set(DX, 40.0);
  walk2->set(SPEED, -30.0);
  planner.addMoveWithTransient(walk2, 3.0, 5.0);

  move = walk ;
  
}

void setTrot(){

  Move_ stand(new Stand(robot));
  stand->set(PHASE1, 0.0);
  stand->set(PHASE2, 0.5);
  stand->set(PHASE3, 0.5);
  stand->set(PHASE4, 0.0);
  planner.addMove(stand, 3.0);

  Move_ trot(new Walk(robot));
  trot->set(SPEED, 150.0); // 200
  trot->set(DX, 40.0);
  trot->set(DY, 35.0);
  trot->set(AMPL_OSC, 0.0);
  trot->set(SHIFT_X, -2.0);
  trot->set(SHIFT_Y, 2.0);
  trot->set(K_GROUND, 0.8); // 0.65
  trot->set(PHASE1, 0.0);
  trot->set(PHASE2, 0.5);
  trot->set(PHASE3, 0.5);
  trot->set(PHASE4, 0.0);
  planner.addMoveWithTransient(trot, 3.0, 5.0); // transitoire 3s, mouvement 5s
  
  move = stand ;
}

void setWalkTrotWalk(){
  
  Move_ stand(new Stand(robot));
  planner.addMove(stand, 5.0);
  
  Move_ walk(new Walk(robot));
  walk->set(DX, 50.0);
  walk->set(SPEED, 60.0);
  planner.addMoveWithTransient(walk, 2.0, 2.0);

  Move_ walk2(new Walk(robot));
  walk2->set(DX, 50.0);
  walk2->set(SPEED, 80.0);
  walk2->set(AMPL_OSC, 0.0);
  walk2->set(K_GROUND, 0.87); // 0.65
  walk2->set(PHASE1, 0.5);
  walk2->set(PHASE2, 1.0);
  walk2->set(PHASE3, 1.0);
  walk2->set(PHASE4, 0.5);
  planner.addMoveWithTransient(walk2, 1.0, 2.0);

  Move_ walk3(new Walk(robot));
  walk3->set(DX, 50.0);
  walk3->set(SPEED, 60.0);
  planner.addMoveWithTransient(walk3, 1.0, 2.0);
  
  move = stand ;
}

void setDemoMove(){

  Move_ slow(new Walk(robot));
  slow->set(SHIFT_X, -7.0);
  slow->set(SHIFT_Y, 2.0);
  planner.addMoveWithTransient(slow, 3.0,5.0);
  
  Move_ walk_crab(new Walk(robot, 0.0, 25.0));  // Rotation = 0, dr = 25deg
  walk_crab->set(SHIFT_X, -7.0);
  walk_crab->set(SHIFT_Y, 2.0);
  walk_crab->set(SPEED, 50);
  planner.addMoveWithTransient(walk_crab, 2.0, 5.0);
  
  Move_ turn(new Walk(robot, -8.0, 0.0));
  turn->set(SHIFT_X, -7.0);
  turn->set(SHIFT_Y, 2.0);
  turn->set(SPEED, 50);
  planner.addMoveWithTransient(turn, 3.0, 10.0);
  
  Move_ turn2(new Walk(robot, -10.0, 0.0));
  turn2->set(SHIFT_X, -7.0);
  turn2->set(SHIFT_Y, 2.0);
  turn2->set(SPEED, 5.0);
  turn2->set(DX, 5.0);
  turn2->set(DY, 40.0);
  planner.addMoveWithTransient(turn2, 3.0, 10.0);
  
  Move_ walk2(new Walk(robot));
  walk2->set(SHIFT_X, -7.0);
  walk2->set(SHIFT_Y, 2.0);
  planner.addMoveWithTransient(walk2, 3.0, 5.0);
  
  Move_ stand2(new Stand(robot));
  planner.addMoveWithTransient(stand2, 5.0, 0.0);
  
  move = slow ;
}

