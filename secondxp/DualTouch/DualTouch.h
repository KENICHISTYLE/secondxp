#pragma once

#include "Renderer.h"
#include "Camera.h"
#include "Physic.h"
#include "HapticDevice.h"
#include <iostream>
#include "Logs.h"

const btScalar Ball_Size = 0.3;
const btScalar Effector_Mass = 4;
const btScalar Effector_Size = 0.08f;
const btScalar m_timeSpeed = 0.01;
const int canonNbr = 4;
const float Gut_vz = 5.0f;
const float Gut_vy = 13.2f;
const double Time = 10 ;
const int Nbr_launch_game = 5;

using namespace std;

class DualTouch
{

public:
	DualTouch(void);
	~DualTouch(void);

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
	void rotateCanon(btVector3* rotate);
	void moveCanonLeft(btScalar x);
	void moveCanonRight(btScalar x);
	void teleportX(Object* canon,btScalar x);

	void getFinalPos(btTransform* target,int targetIndex, btScalar init_vx,btScalar x_dec);
	
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

	void gameStatus();
	
	Logger m_log;
	Physic m_physic;
	Renderer m_renderer;
	Camera m_camera1;
	Camera m_camera2;
	HapticDevice m_hds;
	
	btRigidBody * cursors[NB_DEVICES_MAX];
	btVector3 m_cursorColors[NB_DEVICES_MAX];
	btVector3 m_impactPossible[ThronNumber];	

	vector <btScalar> m_throwed_xv;
	vector <float> m_throwed_x;
	vector <Object *> m_throwed_object_list;
	vector <btRigidBody *> m_throwed_rigid_list;
	vector <btTransform*> m_throwed_transform;	
	vector <btVector3*> m_trajectory[ThronNumber];
	int m_goodToCatch[ThronNumber];
	int m_score;
	int m_lancerNbr;
	int m_catchs;

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
	int m_CanonPos[canonNbr];	
	time_t m_time;
	string m_note;
	bool m_eval;
	bool m_withTraj;
	bool m_feed;
	int m_left_to_launch;
	Object* m_effObj;
};
