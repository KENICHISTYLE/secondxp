#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduVector.h>
#include <cstdio>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <vector>
#include "Object.h"
#include <HDU/hduError.h>
#include <limits>

#pragma comment (lib,"hd.lib")
#pragma comment (lib,"hdu.lib")

#pragma once

#define STIFFNESS 0.30
#define EPSILON 0.00001 /* zero, for purposes of calculating distances. */
#define SCALE_DEVICE_TO_WORLD 0.1f /* scaling distance */
#define SCALE_WORLD_TO_DEVICE 1.0f/SCALE_DEVICE_TO_WORLD /* scaling distance */
#define OFFSET_TO_CAMERA 90
#define NB_DEVICES_MAX 1 //2
#define MAXSCLAR 9999.9
#define MAXSMOOTH_LOOP 50
#define MXFORCE 10
#define MNFORCE -MXFORCE
#define MXDISTANCE 65
#define MNDISTANCE -MXDISTANCE
#define VARIATION_MAX 0.2

const btScalar Time = 100 ;
const HDdouble Quick_Dicplacement = 16;
const HDdouble Quick_Distance_max = 35.0;
const HDdouble Slow_Distance_max = 20.0;

class HapticData
{
	public:
	HapticData(HHD id){
		m_id=id;m_ready=false;m_nbCollision=0;
		m_near = new btVector3(MAXSCLAR,MAXSCLAR,MAXSCLAR);
		m_smother =MAXSMOOTH_LOOP;m_currentThrown = NULL;m_done = false;		
		m_oldPosition.set(0,0,0);
	};
	HapticData(){m_id=-1;m_ready=false;m_nbCollision=0;m_currentThrown = NULL;};
	HHD m_id;
	hduVector3Dd m_position;
	hduVector3Dd m_oldPosition;
	hduVector3Dd m_atThrowPos;
	hduVector3Dd m_force;
	HDdouble m_transform[16];
	hduVector3Dd m_realPosition;	
	HDint m_buttons;
	bool m_ready;
	int m_nbCollision;
	btVector3* m_near;	
	btRigidBody * m_currentThrown;	
	int m_smother;
	bool m_done;
	
	btVector3* m_impactPos;
	
};

class HapticSynchronizer
{
	public:

	HapticSynchronizer(){m_data=NULL;};
	~HapticSynchronizer(){if(m_data!=NULL){delete m_data;}};
	void setData(HapticData *data){m_data=data;}
	void setThrownList(std::vector <btRigidBody *> thrown);
	void setThrown(btRigidBody * thrown);
	void setImpactPos(btVector3* m_impactPos);	
	void setNear(btVector3* pos);
	void setFeadBack();
	HapticData * m_data;
	HapticData m_free;	
	btTransform m_effectors;
	btTransform* m_cameraViews;
	btGeneric6DofConstraint* m_constraints;
	btScalar m_timeK;
};

class HapticDevice
{
public:
	HapticDevice();
	~HapticDevice(void);
	void addDevice(char * name, btTransform &cameraView);
	void init();
	void run();
	void feedback(btDynamicsWorld &dynamic);
	void setConstraint(unsigned int devicesID, btGeneric6DofConstraint* constraint);
	btTypedConstraint*  createConstraint(btRigidBody &myBody,btRigidBody &itsBody);

	static HDCallbackCode HDCALLBACK aSchedule(void *pUserData);
	static HDCallbackCode HDCALLBACK sScheduleOut(void *pUserData);
	static HDCallbackCode HDCALLBACK sScheduleIn(void *pUserData);

	btRigidBody * getConstraintedBody(unsigned int devicesID);
	unsigned int getNbDevices(){return m_nbDevices;}

	btTransform m_effectors[NB_DEVICES_MAX];
	static btTransform transform(HapticSynchronizer* hdd,HapticData* data);
	static hduVector3Dd invertTransform(btVector3* trans,btTransform* invertCamera);
	static hduVector3Dd ComputeForce(hduVector3Dd* effector, hduVector3Dd* target, hduVector3Dd* velocity);
	static hduVector3Dd ForecToImpact(hduVector3Dd* effector,hduVector3Dd* impactpos);
	btVector3 getEffectorPosition();

	HDdouble betweenTwoPoints(hduVector3Dd point1, hduVector3Dd point2);
	void(*m_newConstraint)(void * ptr,btRigidBody *,unsigned int );
	void(*m_deleteConstraint)(void * ptr,btRigidBody *,unsigned int );

	void * m_ptr;

	void setDThrownList(std::vector <btRigidBody *>* thrown);
	void setThrown(btRigidBody * thrown);
	void setDThrownObject(std::vector <Object *>* thrown_object);
	void setDThrownObject(Object * thrown);
	void setImpactPos(btVector3* pos);
	void setPossibleImpactPoints(btVector3* impact);
	void clearPossibleImpactPoints();
	hduVector3Dd trajectoryLine(hduVector3Dd point1,hduVector3Dd point2);
	HDdouble distanceToPath(hduVector3Dd path,hduVector3Dd point);
	HDdouble dott(hduVector3Dd v1,hduVector3Dd v2);
	void setTrajectory(std::vector<btVector3*>* points, int index,unsigned int stopIndex);

	int getCaughtIndex();
	void showTarget(unsigned int pos);
	void showTarget(btRigidBody * target);
	bool isReadyLaunch();
	void setWaitLunch();
	void Lunch();
	bool isTargetChosen();
	void waitTargetChoice();
	btRigidBody* getTarget();
	void activateMove();
	void deactivateMove();
	void resetThrow();
	bool isCaught();

	void clearTrajectory();
	HDdouble distanceToTrajectory(hduVector3Dd effector,int trajectoryIndex,btTransform* invertCamera);
	static void truncate(HDdouble* x,HDdouble* y,HDdouble* z);
	static bool inrange(HDdouble x,HDdouble y,HDdouble z);
	void setGround(btRigidBody* ground);

private:

	std::string m_name;
	unsigned int m_nbDevices;
	HapticSynchronizer m_hss[NB_DEVICES_MAX];
	HDSchedulerHandle m_updateDeviceCallback;
	btGeneric6DofConstraint* m_constraints[NB_DEVICES_MAX];
	btTypedConstraint* m_itsConstraints[NB_DEVICES_MAX];
	btTransform  * m_cameraViews[NB_DEVICES_MAX];
	HDint m_oldButtons[NB_DEVICES_MAX];	
	btVector3* m_impactPos;
	btVector3* m_hipoPos;
	std::vector <btVector3*> m_possibleImpact;
	btScalar m_variator; 
	Object* m_ThrownObject;
	btRigidBody* m_Thrown;
	btRigidBody* m_caught;
	std::vector <Object *>* m_thrownObjects;
	std::vector <btRigidBody *>* m_thrownRigids;
	std::vector <btVector3*> m_trajectory[ThronNumber];
	bool m_canLaunch;
	bool m_posSet;
	bool m_targetChoosen;
	bool m_coll;
	bool m_Feedback;
	bool m_quick;
	btRigidBody* m_ground;	
};


