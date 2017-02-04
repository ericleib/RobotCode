
#ifndef Time_h
#define Time_h

const double SEC = 1.0e-6;

class Time {
  public:	
    void init(unsigned long micros);
	double update(long micros, double period, void (*f)(int, double, double, int));
	
	double phase = 0.0;       // Phase of the movement
	int loop_freq = 0;        // Frequency of loop
	
  private:	
	unsigned long micros_0;   // Origin of time
	int time_s = 0;           // Time as int
	double time = 0.0;        // Time (s)

};

#endif