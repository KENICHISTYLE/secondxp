#pragma once

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>
#include <BulletDynamics\Dynamics\btRigidBody.h>
#include "Consts.h"

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
	void startLogWrite();
	void writeToLog(const char* log);
	void endLogWrite();
	void saveTrajectory(vector <btVector3*>* trajectory,unsigned int Max_points);	
	void saveSession(FILE* session);
	void loadSession();
	void saveBeginInfo();
	void saveEndInfo();
	void saveTimeDate(FILE* file, struct tm* datetime);
	
	void dynamicBegin();
	void dynamicWrite(const char* log);
	void dynamicEnd();

	void printOptions(FILE* file);
	void setOption(char option,bool value);
	char* printBool(bool value);	

private:	

	int m_Session_id;
	int m_game_id;	

	string m_CurrentLogFile;
	FILE* m_open_file;

	string m_dynamic_logname;
	FILE* m_dynamic_log;

	bool m_traj_visible;
	bool m_feedback;
	
	time_t m_SessionBeginTime;
	time_t m_SessionEndTime;

	void endOffile(FILE* file);
};