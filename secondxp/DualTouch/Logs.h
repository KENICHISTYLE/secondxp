
#include <cstdio>
#include <ctime>
#include <iostream>

class Logger 
{

public:
	Logger();
	~Logger(void); 

	void init();
	void setBeginTime();
	void setEndTime();
	void printElepsedTime();
	double getElepsedTime();

private:	
	time_t m_beginTime;
	time_t m_endTime;

};