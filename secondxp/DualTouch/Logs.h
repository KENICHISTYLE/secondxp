
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

using namespace std;

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
	void createLogFile();
	void saveToLogFile(const char* log);
	void saveTrajectory();	
	void saveSession(FILE* session);
	void loadSession();
	void saveBeginInfo();
	void saveEndInfo();
	void saveTimeDate(FILE* file, struct tm* datetime);
	void printOptions(FILE* file);
	void setOption(char option,bool value);
	char* printBool(bool value);	

private:	

	int m_Session_id;
	int m_game_id;	
	string m_CurrentLogFile;

	bool m_traj_visible;
	bool m_feedback;
	
	time_t m_SessionBeginTime;
	time_t m_SessionEndTime;

	void endOffile(FILE* file);
};