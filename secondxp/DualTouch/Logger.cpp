
#include "Logs.h"

using namespace std;

Logger::Logger(){	
}

void Logger::init(){
	time(&m_beginTime);
	time(&m_endTime);
}


Logger::~Logger(void){
}

void Logger::setBeginTime(){
	time(&m_beginTime);
}

void Logger::setEndTime(){
	time(&m_endTime);
}

void Logger::printElepsedTime(){
	m_endTime = time(NULL);
	double temp = difftime(m_endTime, m_beginTime);
	cout << " Elepsed time "<< temp <<": sconds " << endl;
}

double Logger::getElepsedTime(){
	m_endTime = time(NULL);
	return  difftime(m_endTime, m_beginTime);
}