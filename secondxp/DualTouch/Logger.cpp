
#include "Logs.h"


const char* sessionsFileName = "Sessions.ses"; 
const char* log_name = " Game_log_";
const char* ext = ".log";
const int max_line_length = 200;

Logger::Logger(){	
}

void Logger::init(){

	m_Session_id = 0;
	m_game_id = 0;
	m_CurrentLogFile = "";

	time(&m_SessionBeginTime);
	time(&m_SessionEndTime);

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

void Logger::createLogFile(){
	
	ostringstream oss;
	oss << (int) m_Session_id;

	string fileName = log_name + oss.str() + ext;

	m_CurrentLogFile = fileName;

	FILE* file = NULL;

	fopen_s(&file,fileName.c_str(),"w");

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

	fprintf_s(file,"# Joueur ID \n");

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

	fprintf_s(file," %d / %d / %d    %d:%d:%d \n",datetime->tm_mday,datetime->tm_mon,
		datetime->tm_year,datetime->tm_hour,datetime->tm_min,datetime->tm_sec);

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

	fprintf_s(file,"# ---------------------------------------------------------- #");

	fclose(file);

}

void Logger::saveToLogFile(const char* log){

	if(m_CurrentLogFile != ""){
		FILE* file = NULL;

		fopen_s(&file, m_CurrentLogFile.c_str(), "r+");

		if(file != NULL){
			endOffile(file);
			fprintf_s(file,log);
			fclose(file);
		}
	}

}

void Logger::saveTrajectory(){
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