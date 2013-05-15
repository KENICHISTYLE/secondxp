#pragma once

#include "Renderer.h"
#include "Camera.h"
#include "Physic.h"
#include "HapticDevice.h"
#include <iostream>
#include "Logs.h"
#include "Consts.h"
#include <Windows.h>

const btScalar Ball_Size = 0.35;
const btScalar Effector_Mass = 4.2;
const btScalar Effector_Size = 0.17;
const btScalar m_timeSpeed = 0.01;
const int canonNbr = 8;
const float Gut_vz = 5.0f;
const float Gut_vy = 20.0f;
const double Time = 3 ;
const double LearnTime = 90 ;
const int ThrowV = 6;
const int Nbr_launch_game = 5*ThrowV;
const unsigned int max_calculated_points = 600;
const btScalar max_canon_dep = 1;
const int PauseLenght = 30;

using namespace std;

class DualTouch
{

public:
	DualTouch(void);
	~DualTouch(void);

	struct ball{
		struct ball(void){			
			isBall = false;
		};
		btVector3 pos;
		int score;
		bool isBall;
	};

	
	struct log_dyn{
		struct log_dyn(void){
			caught = false;
			phase = false;			
		};
		struct ball throwed[ThronNumber];
		struct myTime currentTime;
		btVector3 effcPosition;
		btScalar vitesseLancer;		
		bool caught;
		bool phase;
		int phaseNbr;
		int caughtValue;
		int score;
		int catchNbr;
		int goodCatchNbr;
		int badCatchNbr;
		int okCatchNbr;
	};

	void reshape(int width, int height);
	void display1();
	void display2();
	void idle();


	void keyboard1(unsigned char key, int x, int y);
	void keyboardUp1(unsigned char key, int x, int y);
	void mouse1(int button, int state, int x, int y);
	void motion1(int x, int y);

	void keyboard2(unsigned char key, int x, int y);
	void keyboardUp2(unsigned char key, int x, int y);
	void mouse2(int button, int state, int x, int y);
	void motion2(int x, int y);

	void init1();
	void createScene();
	void createCursor(unsigned int deviceId);
	void addLauncher();
	void start();
	bool testIfCanThrow();

	void randomMoveCanons();

	void getFinalPos(btTransform* target,unsigned int targetIndex, btScalar init_vx,btScalar x_dec);
	
	void setVelocityTarget(btScalar time,btRigidBody* target,btScalar x);
		
	void throwMultiObject(btScalar Onumber, float canonpos, int index);
	void setHapticParam();
	void waitFeadBack();
	void setAfterColideCoord();
	void deleteThrowedObjects();
	//callbacks
	static void newConstraint(void * ptr,btRigidBody * body,unsigned int id);
	static void deleteConstraint(void * ptr,btRigidBody * body,unsigned int id);
	static void tickCallback(btDynamicsWorld *world, btScalar timeStep);

	void evaluateScore();
	void setTheTargetFinalPos(btRigidBody* target,btScalar initial_x);

	void init2();

	void reset();

	void shuffleV();
	void gameStatus();	
	void reportScoreInfo();
	const char* ballValue(Object* ball);
	void changeParam();
	void inPauseMode();

	void logDynamic();
	void saveLogDynamic();
	string stringFromBtvector(btVector3* vec);
	string colorFromScore(int score);
	

	Logger m_log;
	Physic m_physic;
	Renderer m_renderer;
	Camera m_camera1;
	Camera m_camera2;
	HapticDevice m_hds;
	
	btRigidBody * cursors[NB_DEVICES_MAX];
	btVector3 m_cursorColors[NB_DEVICES_MAX];
	btVector3 m_impactPossible[ThronNumber];

	vector <struct log_dyn*> m_logsVec;
	vector <btScalar> m_throwed_xv;
	vector <float> m_throwed_x;
	vector <Object *> m_throwed_object_list;
	vector <btRigidBody *> m_throwed_rigid_list;
	vector <btTransform*> m_throwed_transform;	
	vector <btVector3*> m_trajectory[ThronNumber];
	btScalar m_throwVelocity[ThrowV];
	int m_score;
	int m_lancerNbr;
	int m_catchs;
	int m_good_catchs;
	int m_bad_catchs;
	int m_ok_catchs;

	btRigidBody* m_curentThrowed;	
	Object* m_curentObject;
	btScalar m_velocityY;
	btScalar m_velocityZ;	
	bool m_moveTarget;
	btScalar m_theta;
	btScalar m_lunch_z;
	btScalar m_lunch_y;	
	btScalar m_impactY;	
	Object* m_canons[canonNbr];
	btCollisionShape* m_canonShape;	
	btScalar m_CanonPos[canonNbr];	
	time_t m_time;
	time_t m_log_time;
	time_t m_adaptationTime; 
	string m_note;
	int m_throw_at_once;
	bool m_eval;
	bool m_withTraj;
	bool m_feed;
	int m_left_to_launch;
	bool m_startLog;
	bool m_timerBegan;
	bool m_Fin_jeu;
	bool m_wait;
	time_t m_waitTime;
	int m_passe;
	btScalar m_actualV;
	int m_vIndex;
	WORD m_log_milis;
	btScalar m_canon_step;
	Object* m_effObj;
};
