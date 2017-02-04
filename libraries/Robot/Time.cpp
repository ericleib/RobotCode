
#include "Time.h"


void Time::init(unsigned long micros){
  micros_0 = micros;
}

double Time::update(long micros, double period, void (*f)(int, double, double, int)){
  double dt = (micros - micros_0) * SEC - time;
  time += dt;
  phase += dt / period;
  if(time_s < (int) time){  // Jump to next second
	(*f)(time_s, phase, period, loop_freq);
    loop_freq = 0;
  }
  time_s = (int) time;
  loop_freq++;
  return phase;
}

