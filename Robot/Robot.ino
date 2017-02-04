
/*
 Controlling a leg movement
*/
#include <StandardCplusplus.h>
#include <serstream>

#include <Move.h>
#include <Components.h>
#include <Time.h>

using namespace std;

namespace std
{
  ohserialstream cout(Serial);  // Directs the standard output to the Serial port
}

Time time_keeper;
MovePlanner planner;
Move_ move;
Robot robot;
double phase = 0.0;

void setup() {
  Serial.begin(57600);
  delay(1000) ;
  Serial.print("Hello World") ;
  cout << endl << endl << endl << endl << endl << endl << "Setup" << endl;

  initRobot();

  cout << robot << endl;

  initPlanner();

  cout << planner << endl;
  
  time_keeper.init(micros());
}


void loop() {
  phase = time_keeper.update(micros(), move->getPeriod(phase), print_state);
  
  move = planner.getMove(phase);
  
  while (Serial.available() > 0) {
    int i = Serial.parseInt();
    float f = Serial.parseFloat();
    Serial.read();  // Read the end character to empty the buffer

    cout << "setting parameter " << i << "= " << f << endl;

    //if(i==ROTATION || i==SPEED)
    move->set(i, f);
  }

  robot.apply(move, phase);  
}

void print_state(int time_s, double phase, double period, int freq){
  cout << "time: " << time_s << "s / phase: " << phase << " / Period: " << period << "s / Speed: " << move->getSpeed(phase).x << "mm/s / " << freq << "Hz / ";
  move->to_string(cout);
  //cout << endl << "   Speed: " << move->getSpeed(phase).x;
  cout << endl;
  //cout << robot.legs[0].foot.x << " " << robot.legs[0].phi << endl;
  cout << "   Feet state: " << analogRead(A4) << " / " << analogRead(A5) << " / " << analogRead(A6) << " / " << analogRead(A7) << endl;
  Serial.flush();
}



