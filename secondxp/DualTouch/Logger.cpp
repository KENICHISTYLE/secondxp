
#include "Logs.h"
#include <iostream>
#include <windows.h>
#include <direct.h>

const char* Log_directory = "Logs\\";
const char* sessionsFileName = "Logs\\Sessions.ses"; 
const char* log_name = "Game_log_";
const char* dyn_log_name = "Game_dynamic_log";
const char* ext = ".log";
const int max_line_length = 200;
const int crac_par_val = 10;

Logger::Logger(){	
}

void Logger::init(){

	m_Session_id = 0;
	m_game_id = 0;
	m_CurrentLogFile = "";
	m_dynamic_logname = "";

	time(&m_SessionBeginTime);
	time(&m_SessionEndTime);

	FILE* dir = NULL;
	fopen_s(&dir,Log_directory,"r");
		
	if(dir == NULL)
		mkdir(Log_directory);
	else
		fclose(dir);

	loadSession();
}


Logger::~Logger(void){
}

void Logger::setBeginTime(){
	time(&m_SessionBeginTime);
}

void Logger::setEndTime(){
	time(&m_SessionEndTime);
}

void Logger::printElepsedTime(){
	time_t t;
	t = time(NULL);
	double temp = difftime(t, m_SessionBeginTime);
	cout << " Elepsed time "<< temp <<": sconds " << endl;
}

double Logger::getElepsedTime(){
	time_t t = time(NULL);
	return  difftime(t, m_SessionBeginTime);
}

void Logger::dynamicBegin(){
	if(m_dynamic_logname != ""){

		FILE* file = NULL;

		fopen_s(&file, m_dynamic_logname.c_str(), "r+");

		if(file != NULL){
			endOffile(file);
			fprintf_s(file,"########  Status report at\t");

			struct myTime t = getCurrent();

			fprintf_s(file," %d:%d:%d:%d ------------------ ##### \n",t.hh,t.mm,t.ss,t.mil);
		}


		m_dynamic_log = file;
	}
}

struct myTime Logger::getCurrent(){
	SYSTEMTIME stime;
	GetSystemTime(&stime);
	WORD millis = stime.wMilliseconds;

	struct tm datetime;
	time_t now;

	time(&now);

	localtime_s(&datetime, &now);

	struct myTime t;

	t.hh = datetime.tm_hour ; 
	t.mm = datetime.tm_min;
	t.ss = datetime.tm_sec;
	t.mil = millis;

	return t;
}

void Logger::dynamicWrite(const char* log){
	if(m_dynamic_log != NULL){		
		fprintf_s(m_dynamic_log,"%s",log);
	}
}

void Logger::dynamicEnd(){
	if(m_dynamic_log != NULL)
		fclose(m_dynamic_log);
	m_dynamic_log = NULL;
}

void Logger::createLogFile(){
	
	ostringstream oss;
	oss << (int) m_Session_id;

	string fileName = Log_directory;
	fileName += log_name + oss.str() + ext;

	m_CurrentLogFile = fileName;

	string dynName = Log_directory;
	dynName += dyn_log_name + oss.str() + ext;

	m_dynamic_logname = dynName;

	FILE* file = NULL;

	fopen_s(&file,fileName.c_str(),"w");

	fprintf_s(file,"# Game Session ID \n");
	fprintf_s(file,"%d \n",m_Session_id);
	fprintf_s(file,"### Session Start ### \n");

	fclose(file);

	file = NULL;

	fopen_s(&file,dynName.c_str(),"w");

	fprintf_s(file,"# Game Session ID \n");
	fprintf_s(file,"%d \n",m_Session_id);
	fprintf_s(file,"### Session Start ### \n");

	fclose(file);
}

char* Logger::printBool(bool value){
	if(value)
		return " True ";
	else
		return " False ";
}

void Logger::printOptions(FILE* file){

	fprintf(file,"# With Fead back \n");
	fprintf(file,"%s \n",printBool(m_feedback));

	fprintf(file,"# With Visible Trajectory \n");
	fprintf(file,"%s \n",printBool(m_traj_visible));

}

void Logger::setOption(char option,bool value){
	
	switch(option){
		case 'f': m_feedback = value;
			      break;
		case 't': m_traj_visible = value;
			      break;
		default:
			      printf(" Option inconue");
	}

}

void Logger::saveBeginInfo(){
	
	FILE* file = NULL;
	
    fopen_s(&file,m_CurrentLogFile.c_str(),"r+");

	endOffile(file);

	fprintf_s(file,"# Game ID \n");

	fprintf_s(file,"%d \n",m_game_id);	

	fprintf_s(file,"# Begin play Date and Time \n");

	struct tm Today;
	time_t now;

	time(&now);

	localtime_s(&Today, &now);

	saveTimeDate(file,&Today);	

	printOptions(file);

	fprintf_s(file, "### Game start ### \n ");

	fclose(file);

	m_game_id++;
}
	
void Logger::saveTimeDate(FILE* file, struct tm* datetime){

	fprintf_s(file," %d / %d / %d    %d:%d:%d \n",datetime->tm_mday,(datetime->tm_mon + 1),(
		datetime->tm_year + 1900),datetime->tm_hour,datetime->tm_min,datetime->tm_sec);

}

void Logger::saveEndInfo(){
	
	FILE* file = NULL;

	fopen_s(&file, m_CurrentLogFile.c_str(),"r+");

	endOffile(file);

	fprintf_s(file,"# Game End Date and Time \n");

	struct tm Today;
	time_t now;

	time(&now);

	localtime_s(&Today, &now);

	saveTimeDate(file,&Today);

	fprintf_s(file,"# ---------------------------------------------------------- # \n\n ");

	fclose(file);

}

// Use when log one info
// same as startLogWrite();writeToLog(log);void endLogWrite();
void Logger::saveToLogFile(const char* log){

	if(m_CurrentLogFile != ""){
		FILE* file = NULL;

		fopen_s(&file, m_CurrentLogFile.c_str(), "r+");

		if(file != NULL){
			endOffile(file);
			fprintf_s(file,"%s",log);
			fclose(file);
		}
	}

}

// Use to log multiple info 
void Logger::startLogWrite(){
	if(m_CurrentLogFile != ""){
		FILE* file = NULL;

		fopen_s(&file, m_CurrentLogFile.c_str(), "r+");

		if(file != NULL){
			endOffile(file);
		}

		m_open_file = file;
	}
}

void Logger::writeToLog(const char* log){
	if(m_open_file != NULL)
		fprintf_s(m_open_file,"%s",log);
}

void Logger::endLogWrite(){
	if(m_open_file != NULL)
		fclose(m_open_file);
	m_open_file = NULL;
}

void Logger::saveTrajectory(vector <btVector3*>* trajectory,unsigned int Max_points){

	FILE* file = NULL;
	
    fopen_s(&file,m_CurrentLogFile.c_str(),"r+");

	endOffile(file);

	fprintf_s(file,"# Trajectory par ball \n");

	for(int i =0 ; i <ThronNumber ; i++){
			fprintf_s(file," Ball %d ",i);	
			for(int j = 0; j< 3*crac_par_val; j++)
				fputc(' ',file);
			fputc('\t',file);
		}

	fprintf_s(file,"\n");

	for(int i =0 ; i <ThronNumber ; i++){		    
			fputc(' X',file);
			for(int j = 0; j< crac_par_val; j++)
				fputc(' ',file);	
			fputc('Y',file);
			for(int j = 0; j< crac_par_val; j++)
				fputc(' ',file);
			fputc('Z',file);
			for(int j = 0; j< crac_par_val; j++)
				fputc(' ',file);
			fputc('\t',file);
		}

	fprintf_s(file,"\n");

	//unsigned int pmax = trajectory[i].size()

	for(unsigned int k = 0; k< Max_points; k++){

		for(int i =0 ; i <ThronNumber ; i++){
			
			unsigned s = trajectory[i].size();

			if(s > 0 && s > k)
			{
				fprintf_s(file," %lf %lf %lf ",(trajectory[i][k])->x(),(trajectory[i][k])->y(),(trajectory[i][k])->z());
			}
			else
				fprintf_s(file,"                    ");
			fputc('\t',file);
		}

		fprintf_s(file,"\n");
	}

	fclose(file);
}

void Logger::saveSession(FILE* session){

	fprintf(session,"## Session ID \n");

	fprintf(session,"%d \n",m_Session_id);
	fprintf(session,"# Session Date and Time \n");

	struct tm Today;
	time_t now;

	time(&now);
	
	localtime_s(&Today, &now);

	saveTimeDate(session, &Today);
	
	fprintf(session,"\n");
}

void Logger::loadSession(){

	FILE* session = NULL;

	fopen_s(&session, sessionsFileName,"r+");

	// load data to create a new session
	if(session != NULL){
		char line[max_line_length] ="";
		int temp_id = 0;

		while(fgets(line,max_line_length,session) != NULL){
			if(line[0] == '#' && line[1] == '#'){
				fscanf(session,"%d",&temp_id);
			}
		}

		m_Session_id = temp_id+1;

		saveSession(session);		
	}
	else // create the file where to store data about sessions
	{
		fopen_s(&session, sessionsFileName, "w");
		saveSession(session);
	}

	fclose(session);
}

void Logger::endOffile(FILE* file){

	char line[max_line_length] ="";
	while(fgets(line,max_line_length,file) != NULL);

}