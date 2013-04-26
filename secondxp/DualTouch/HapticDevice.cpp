#include "HapticDevice.h"
#include <iostream>

HapticDevice::HapticDevice()
{
	m_updateDeviceCallback = HD_INVALID_HANDLE;
	for(int i=0;i<NB_DEVICES_MAX;i++)
	{
		m_constraints[i] = NULL;
		m_itsConstraints[i] = NULL;
		m_oldButtons[i] = HD_INVALID_HANDLE;			
	}
	m_nbDevices = 0;
	m_coll = false;	

	cleanHistory();
	//m_time = 0;
}


HapticDevice::~HapticDevice(void)
{
	if(m_updateDeviceCallback != HD_INVALID_HANDLE)
	{
		hdStopScheduler();
		hdUnschedule(m_updateDeviceCallback);
	}

	for(unsigned int i=0;i<m_nbDevices;i++)
	{
		if(m_constraints[i] != NULL)
			delete m_constraints[i];

		if(m_hss[i].m_data!=NULL)
		{
			if (m_hss[i].m_data->m_id != HD_INVALID_HANDLE)
			{
				hdDisableDevice(m_hss[i].m_data->m_id);
				m_hss[i].m_data->m_id = HD_INVALID_HANDLE;
			}
		}
	}
}

HDCallbackCode HDCALLBACK HapticDevice::aSchedule(void *pUserData)
{
	HapticSynchronizer * hs = (HapticSynchronizer *) pUserData;

	for(unsigned int i=0;i<NB_DEVICES_MAX;i++)
	{
		if(hs[i].m_data!=NULL)
		{
			HapticData * data = hs[i].m_data;

			hduVector3Dd force( 0, 0, 0 );				
			hdBeginFrame(hs[i].m_data->m_id);		

			HDdouble forceClamp;
			hduVector3Dd distance = data->m_realPosition - data->m_position;

			force = data->m_force;
			/* if we are nearly at the center don't recalculate anything,
				since the center is a singular point. */
			if(data->m_nbCollision > 0)
			{
				if(distance.magnitude() > EPSILON && data->m_ready)
				{   

					/* force is calculated from F=kx; k is the stiffness
						and x is the vector magnitude and direction.  In this case,
						k = STIFFNESS, and x is the penetration vector from 
						the actual position to the desired position. */

					force += STIFFNESS*distance;			
					
				} 
			}
			
			// Check if we need to clamp the force. 
			hdGetDoublev(HD_NOMINAL_MAX_FORCE, &forceClamp);
			if (hduVecMagnitude(force) > forceClamp)
			{
				hduVecNormalizeInPlace(force);
				hduVecScaleInPlace(force, forceClamp);
			}
			

			hdSetDoublev(HD_CURRENT_FORCE, force);
			
			hdEndFrame(data->m_id);
		}
		
	}

	
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
       // hduPrintError(stderr, &error, "Error during scheduler callback");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
		}
    }

	return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK HapticDevice::sScheduleOut(void *pUserData)
{
	HapticSynchronizer * hs = (HapticSynchronizer *) pUserData;

	for(unsigned int i=0;i<NB_DEVICES_MAX;i++)
	{
		if(hs[i].m_data!=NULL)
		{
			hdMakeCurrentDevice(hs[i].m_data->m_id);
			hdBeginFrame(hs[i].m_data->m_id);
			hdGetDoublev(HD_CURRENT_POSITION, hs[i].m_data->m_position);
			hdGetDoublev(HD_CURRENT_FORCE, hs[i].m_data->m_force);
			hdGetDoublev(HD_CURRENT_TRANSFORM, hs[i].m_data->m_transform);
			hdGetIntegerv(HD_CURRENT_BUTTONS, &hs[i].m_data->m_buttons);
			hdEndFrame(hs[i].m_data->m_id);

			hs[i].m_free.m_position = hs[i].m_data->m_position;
			for(int j=0;j<16;j++)
				hs[i].m_free.m_transform[j] = hs[i].m_data->m_transform[j];
			
			

			hs[i].m_free.m_buttons=hs[i].m_data->m_buttons;
			hs[i].m_free.m_force = hs[i].m_data->m_force;
		}
	}

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK HapticDevice::sScheduleIn(void *pUserData)
{
	HapticSynchronizer * hs = (HapticSynchronizer *) pUserData;

	for(unsigned int i=0;i<NB_DEVICES_MAX;i++)
	{
		if(hs[i].m_data!=NULL)
		{
			hs[i].m_data->m_realPosition = hs[i].m_free.m_realPosition;
			hs[i].m_data->m_nbCollision = hs[i].m_free.m_nbCollision;
			hs[i].m_data->m_force = hs[i].m_free.m_force;
			hs[i].m_data->m_ready = true;
		}
	}

    return HD_CALLBACK_DONE;
}

//name: identifier of the haptic device (exemple : "PHANToM 1")
//cameraView : camera matrix linked with the haptic device
//constraint : physical constraint/object linked with the haptic device
void HapticDevice::addDevice(char * name, btTransform & cameraView)
{
	if(NB_DEVICES_MAX>=m_nbDevices+1)
	{
		HDErrorInfo error;
		m_name = name;
		//HapticData * data = new HapticData(hdInitDevice(name));                             <-- name
		HapticData * data = new HapticData(hdInitDevice(HD_DEFAULT_DEVICE));
		m_hss[m_nbDevices].setData(data);
		
		m_hss[m_nbDevices].m_effectors = m_effectors[m_nbDevices];
		m_hss[m_nbDevices].m_constraints = m_constraints[m_nbDevices];

		if (HD_DEVICE_ERROR(error = hdGetError())) 
		{
			printf("Failed to initialize haptic device %i\n",data->m_id);
			return ;
		}
		else
		{
			printf("Succed to initialize haptic device %i\n",data->m_id);
		}

		hdEnable(HD_FORCE_OUTPUT);
		
		m_hss[m_nbDevices].m_cameraViews = &cameraView;
		m_cameraViews[m_nbDevices] = &cameraView;
		m_nbDevices++;
	}
}

void HapticDevice::setDThrownList(std::vector <btRigidBody *>* thrown){
	m_thrownRigids = thrown;	
	m_hss[0].setFeadBack();
}

void HapticDevice::setDThrownObject(std::vector <Object *>* thrown){
	m_thrownObjects = thrown;
}

void HapticDevice::setDThrownObject(Object * thrown){
	//m_ThrownObject = thrown;
}

void HapticDevice::setFeadBack(bool WithOrWithout){
	m_Feedback = WithOrWithout;
}

void HapticDevice::setThrown(btRigidBody * thrown){
	for(unsigned int i=0; i< NB_DEVICES_MAX;i++)
		m_hss[i].setThrown(thrown);
	//m_time = 0;
	//std::cout<<"set hap"<<std::endl;
}

void HapticDevice::resetThrow(){
	m_Thrown = NULL;
	m_thrownObjects = NULL;
	m_thrownRigids = NULL;
	m_caught = NULL;
	clearTrajectory();
	m_selectedDistance = Distance_max*2;
}

void HapticDevice::setConstraint(unsigned int devicesID, btGeneric6DofConstraint* constraint)
{
	if(devicesID<m_nbDevices)
	{
		m_constraints[devicesID] = constraint;
	}
}

void HapticDevice::init()
{
	if(m_nbDevices>0)
	{
		HDErrorInfo error;

		m_updateDeviceCallback = hdScheduleAsynchronous( aSchedule, &m_hss, HD_MAX_SCHEDULER_PRIORITY);

		hdStartScheduler();
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			printf("Failed to start the scheduler\n");
		}
		else
		{
			printf("Succed to start the scheduler\n");
		}
		m_canLaunch = false;
		m_posSet = false;
		m_targetChoosen = false;
		m_Feedback = true; 
		m_devine = true;
	}
}

btTransform HapticDevice::transform(HapticSynchronizer* hs,HapticData* data){
	
			btTransform myTrans;
			btVector3 origin((HDfloat)data->m_position[0],(HDfloat)data->m_position[1],(HDfloat)data->m_position[2]-OFFSET_TO_CAMERA);
			origin*=SCALE_DEVICE_TO_WORLD;
			//hs->m_cameraViews->setBasis(hs->m_cameraViews->getBasis().transpose());

			btMatrix3x3 basis;
			btScalar m[16];
			for(int j=0;j<16;j++)
				m[j] = (float) data->m_transform[j];
			basis.setFromOpenGLSubMatrix(m);

			//put device position/orientation into  camera/world referencial
			myTrans.setOrigin(origin); 
			myTrans.setBasis(basis);

			btTransform* tempcam = new btTransform(*(hs->m_cameraViews));
			tempcam->setBasis(hs->m_cameraViews->getBasis().transpose());
			
			myTrans.mult(*tempcam,myTrans);
			//hs->m_cameraViews->setBasis(hs->m_cameraViews->getBasis().transpose());
			
			return myTrans;
}

hduVector3Dd  HapticDevice::invertTransform(btVector3* trans,btTransform* invertCamera){
			
			btTransform camInv =btTransform(*invertCamera);
		       
			btVector3 pos(*trans);
			pos =camInv(pos);
			pos*=SCALE_WORLD_TO_DEVICE;					
			
			return hduVector3Dd(pos.getX(),	pos.getY(),	pos.getZ()+OFFSET_TO_CAMERA) ;
}

void HapticDevice::run()
{
	hdScheduleSynchronous(sScheduleOut, &m_hss, HD_DEFAULT_SCHEDULER_PRIORITY);

	for(unsigned int i=0;i<m_nbDevices;i++)
	{
		if(m_constraints[i] != NULL)
		{
			btTransform myTrans;
			btVector3 origin((HDfloat)m_hss[i].m_free.m_position[0],(HDfloat)m_hss[i].m_free.m_position[1],(HDfloat)m_hss[i].m_free.m_position[2]-OFFSET_TO_CAMERA);
			origin*=SCALE_DEVICE_TO_WORLD;
			m_cameraViews[i]->setBasis(m_cameraViews[i]->getBasis().transpose());

			btMatrix3x3 basis;
			btScalar m[16];
			for(int j=0;j<16;j++)
				m[j] = (float) m_hss[i].m_free.m_transform[j];
			basis.setFromOpenGLSubMatrix(m);

			//put device position/orientation into  camera/world referencial
			myTrans.setOrigin(origin); 
			myTrans.setBasis(basis);

			myTrans.mult(*(m_cameraViews[i]),myTrans);

			btTransform offset(btMatrix3x3::getIdentity(),btVector3(0,0,0.5));
			m_effectors[i].setOrigin(origin);
			m_effectors[i].setBasis(basis);
			m_effectors[i].mult(m_effectors[i],offset);
			m_effectors[i].mult(*m_cameraViews[i],m_effectors[i]);

			//correction cursor orientation 
			m_effectors[i].getBasis()*=btMatrix3x3(1,0,0, 0,-1,0, 0,0,-1);

			m_constraints[i]->getFrameOffsetA() = myTrans;

			//if(m_itsConstraints[i]!=NULL)
			//	((btGeneric6DofConstraint*)m_itsConstraints[i])->getFrameOffsetA() = myTrans;
		}
	}
}

void  HapticDevice::feedback(btDynamicsWorld &dynamic)
{
	for(unsigned int i=0;i<m_nbDevices;i++)
	{
		// free move
		if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) != 0 )// (m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_2))
		{
			m_hss[i].m_free.m_done = true;	
			m_hss[i].m_free.m_force = hduVector3Dd(0,0,0);
		}		
		
		if(m_constraints[i] != NULL)
		{
			btRigidBody * myBody = &m_constraints[i]->getRigidBodyB();
		
			btTransform myTrans = myBody->getWorldTransform();			

			//Check collision  
			if(m_constraints[i]->getUserConstraintPtr() != NULL)
			{  
				//std::cout<< " se cas la " <<std::endl;
				m_hss[i].m_free.m_nbCollision = 1;
				//if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) != 0 && (m_oldButtons[i] & HD_DEVICE_BUTTON_1) == 0)
				btCollisionObject * object = static_cast<btCollisionObject *>(m_constraints[i]->getUserConstraintPtr());
				
				if(object->getInternalType()== btCollisionObject::CO_RIGID_BODY)
				{
					btRigidBody * collideBody = static_cast<btRigidBody *>(object);	
					if(collideBody->getInvMass()!=0 && collideBody != m_ground)
					{
							if(m_itsConstraints[i] == NULL )
							{

								// catch it if colide with it
								if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) == 0 )
								{						
											//create constraint
											btTransform bodyTrans = collideBody->getWorldTransform();
											m_itsConstraints[i]   = createConstraint(*myBody,*collideBody);
											dynamic.addConstraint(m_itsConstraints[i],true);
											m_newConstraint(m_ptr,collideBody,i);
											m_caught = collideBody;
											m_coll = true;	
											m_devine = false;
											showTarget(collideBody);
											deactivateMove();
								}
							}else
								// realise it when button 1 pressed
								if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) != 0)
								{
						
										//remove constraint
										dynamic.removeConstraint(m_itsConstraints[i]);
										delete m_itsConstraints[i];
										m_itsConstraints[i]=NULL;
										m_deleteConstraint(m_ptr,collideBody,i);
										//m_hss[i].setThrown(NULL);								
										m_hss[i].m_free.m_done = true;
										showTarget(collideBody);
										m_variator = 0;
										m_coll = false;										
										deactivateMove();
								}
							
					}
					
				}
				

			}
			else
			{
				m_hss[i].m_free.m_nbCollision = 0;
				//if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) != 0 && (m_oldButtons[i] & HD_DEVICE_BUTTON_1) == 0)
				if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_1) != 0)
				{
					if(m_itsConstraints[i] != NULL )
					{
						//remove constraint
						m_deleteConstraint(m_ptr,&m_itsConstraints[i]->getRigidBodyB(),i);
						dynamic.removeConstraint(m_itsConstraints[i]);
						delete m_itsConstraints[i];
						m_itsConstraints[i]=NULL;
						m_hss[i].m_free.m_done = true;
						if(m_caught != NULL)
							showTarget(m_caught);	
						m_coll = false;						
						deactivateMove();
					}						
					
				}
				
			}

			if(m_itsConstraints[i]!=NULL)
				m_hss[i].m_free.m_nbCollision = 1;			
			else
				// launch an other target
				if((m_hss[i].m_free.m_buttons & HD_DEVICE_BUTTON_2) != 0 && (m_oldButtons[i] & HD_DEVICE_BUTTON_2) == 0)
				{				
				m_canLaunch = true;				
				}				

			m_oldButtons[i]=m_hss[i].m_free.m_buttons;

			//put back cursor world position into device referencial                         <<<<< ------------------------------
			btTransform offset(btMatrix3x3::getIdentity(),btVector3(0,0,-0.5));
			m_effectors[i].setOrigin(myTrans.getOrigin());
			m_effectors[i].mult(m_effectors[i],offset);
			btVector3 pos = m_cameraViews[i]->inverse()(myTrans.getOrigin());
			pos*=SCALE_WORLD_TO_DEVICE;
			m_hss[i].m_free.m_realPosition.set(pos.getX(),pos.getY(),pos.getZ()+OFFSET_TO_CAMERA); 
			

			}
			
			btTransform camInv = m_cameraViews[i]->inverse();
			// detecte the direction
			HDdouble deplacement = betweenTwoPoints(m_hss[i].m_free.m_atThrowPos,m_hss[i].m_free.m_position);
			HDdouble distanceMax = Distance_max;
			
			hduVector3Dd move = m_hss[i].m_free.m_oldPosition - m_hss[i].m_free.m_position;
			//if(move.magnitude() >= Quick_Dicplacement){
			//	//distanceMax = Slow_Distance_max;	
			//	//activateMove();
			//    //deactivateMove();
			//}else             
			//{				
			//	//deactivateMove();
			//	//distanceMax = Slow_Distance_max; 
			//}
			
			HDdouble selectedDistance = 0;
			m_hss[i].m_free.m_force = hduVector3Dd(0,0,0);
			if( m_Feedback && m_devine && m_thrownRigids != NULL && m_thrownRigids->size()>0 && deplacement > distanceMax && m_sible == 0 /* && !m_targetChoosen */){
				//btTransform trans;				
				//hduVector3Dd line = trajectoryLine(m_hss[i].m_free.m_position, m_hss[i].m_free.m_oldPosition);
				HDdouble distance = Slow_Distance_max;
				if(move.magnitude() >= Quick_Dicplacement)
					distance = Quick_Distance_max;
				//unsigned int targ = 0;
				for(unsigned int j = 0; j<m_thrownRigids->size(); j++){
					//trans = (*m_thrownRigids)[j]->getWorldTransform();
					//btVector3 vec,v2;
					//v2 =btVector3(trans.getOrigin());
					//if(m_possibleImpact.size()> 0)
						//vec = btVector3(*m_possibleImpact[j]);
				
					//hduVector3Dd target = invertTransform(&v2, &camInv);
					
					//hduVector3Dd target2 = invertTransform(&vec, &camInv);
					
					//HDdouble d = distanceToPath(line,target) * 0.2 ;
					HDdouble d = distanceToTrajectory(m_hss[i].m_free.m_position,j,&camInv);
					
					//HDdouble d = betweenTwoPoints(target,m_hss[i].m_free.m_position);
					//HDdouble d2 = betweenTwoPoints(target2,m_hss[i].m_free.m_position);
				//	hduVector3Dd imp = invertTransform(m_possibleImpact[j], &camInv);					
				//	HDdouble sc = move.dotProduct(imp);
				//	if(sc >= 0)
						if((d)<distance){
							if(m_Thrown != (*m_thrownRigids)[j]){
								m_hss[i].m_free.m_currentThrown = (*m_thrownRigids)[j];
								distance = d;
								m_Thrown = (*m_thrownRigids)[j];		
								m_impactPos = m_possibleImpact[j];
								m_targetChoosen = true;							
								activateMove();
								selectedDistance = d;	
								m_sible = 6;
								//deactivateMove();
								//targ = i;
							}
						}
					
				}
				if(m_targetChoosen){
					showTarget(m_Thrown);
					addPrevious(m_Thrown);
				}
				else
					m_sible = 0;
				//m_time = Time;
			}
			if(m_canLaunch){
				//m_time = Time;
				m_hss[i].m_free.m_done = true;				
				m_variator = 0.001;	
				m_sible = 0;
				m_devine = true;
				deactivateMove();
				cleanHistory();
				m_hss[i].m_free.m_atThrowPos = m_hss[i].m_free.m_position;
			}
			m_hss[i].m_free.m_oldPosition = m_hss[i].m_free.m_position ;
			if(m_hss[i].m_free.m_currentThrown != NULL && m_posSet && m_Feedback ){
												
					hduVector3Dd impact = invertTransform(m_impactPos, &camInv);
					hduVector3Dd pos(m_hss[i].m_free.m_position); 
					
					
					if(!m_hss[i].m_free.m_done){	
					
						hduVector3Dd helpForce = ForceToImpact(&pos,&impact);	
						hduVector3Dd temp = impact - pos; 
						HDdouble d = temp.magnitude();
						//std::cout<< " D " << d << std::endl;

						if(!helpForce.isZero(EPSILON)){						 	    
							
							hduVector3Dd force = 0.8 * helpForce;												

							m_hss[i].m_free.m_force = force;
							m_Force = force;	

						}else 
							{
							m_hss[i].m_free.m_force = m_Force;							
						    }
				
					}
				
			}			
			hdScheduleSynchronous(sScheduleIn, &m_hss, HD_DEFAULT_SCHEDULER_PRIORITY);
						
			m_selectedDistance = selectedDistance;
			if(m_sible > 0)
				m_sible--;
	}
}

btTypedConstraint* HapticDevice::createConstraint(btRigidBody &myBody,btRigidBody &itsBody)
{
	btTransform myTrans = myBody.getWorldTransform();
	btTransform itsTrans = itsBody.getWorldTransform().inverse();
	btVector3 itsPos = itsTrans * myTrans.getOrigin();
	itsTrans *= myTrans;

	btPoint2PointConstraint * constraint = new btPoint2PointConstraint(myBody,itsBody,btVector3(0,0,0),itsPos);
	constraint->m_setting.m_impulseClamp = 100;
	constraint->m_setting.m_tau = 0.001f;		

	return constraint;

	/*itsBody.setMassProps(0.0001f,btVector3(16,16,16));
	btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(myBody,itsBody,btTransform(btMatrix3x3::getIdentity()),itsTrans,true);
	
	//btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(itsBody,btTransform(btMatrix3x3(1,0,0, 0,1,0, 0,0,1),itsPos),false);
	dof6->setBreakingImpulseThreshold(10000);

	dof6->setLinearLowerLimit(btVector3(0,0,0));
	dof6->setLinearUpperLimit(btVector3(0,0,0));  
	dof6->setAngularLowerLimit(btVector3(0,0,0));
	dof6->setAngularUpperLimit(btVector3(0,0,0));

	for(int axe=0;axe<6;axe++)
	{
		dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,axe);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.8f,axe);
	}
	return dof6;*/
}


btRigidBody * HapticDevice::getConstraintedBody(unsigned int devicesID)
{
	if(devicesID<m_nbDevices)
	{
		if(m_itsConstraints[devicesID]!=NULL)
		{
			return &m_itsConstraints[devicesID]->getRigidBodyB();
		}
	}

	return NULL;
}

bool HapticDevice::isCaught(){
	return m_coll;
}

void HapticSynchronizer::setThrown(btRigidBody * thrown){
	m_free.m_currentThrown = thrown;
	m_free.m_done = false;
	//std::cout<<" set "<<std::endl;
}

void HapticSynchronizer::setFeadBack(){	
	m_free.m_done = false;
	//std::cout<<" set "<<std::endl;
}

bool HapticDevice::isReadyLaunch(){
	return m_canLaunch;
}
void HapticDevice::Lunch(){
	m_canLaunch = true;
}

btRigidBody* HapticDevice::getTarget(){
	return m_Thrown;
}

bool HapticDevice::isTargetChosen(){
	return m_targetChoosen;
}

void HapticDevice::waitTargetChoice(){
	m_targetChoosen = false;
}

void HapticDevice::activateMove(){
	m_posSet = true;
    //m_Feedback = true; 
}

void HapticDevice::deactivateMove(){
	m_posSet = false;
	//m_Feedback = false; 
}

HDdouble HapticDevice::dott(hduVector3Dd v, hduVector3Dd u){
	return (v[0]*u[0] + v[1]*u[1] + v[2]*u[2]); 
}

void HapticDevice::setWaitLunch(){
	m_canLaunch = false;
}

void HapticDevice::showTarget(unsigned int pos){

		for(unsigned int i= 0;i< m_thrownObjects->size();i++)
			{				
				if(i == pos){
					(*m_thrownObjects)[i]->setColor(green);
					m_ThrownObject = (*m_thrownObjects)[i];
				}else
					(*m_thrownObjects)[i]->setColor(light_Grey);
			};
	

}

void HapticDevice::showTarget(btRigidBody* target){

		for(unsigned int i= 0 ;i< m_thrownObjects->size();i++)
			{				
				if((*m_thrownObjects)[i]->getshape() == target->getCollisionShape()){
					//(*m_thrownObjects)[i]->setColor(green);
					m_ThrownObject = (*m_thrownObjects)[i];
					m_ThrownObject->setShow(true);
				}else{
					//(*m_thrownObjects)[i]->setColor(light_Grey);
					(*m_thrownObjects)[i]->setShow(false);
				}
			};
	

}

int HapticDevice::getCaughtScore(){
	for(unsigned int i= 0 ;i< m_thrownObjects->size();i++)
			{				
				if((*m_thrownObjects)[i]->getshape() == m_caught->getCollisionShape()){
					return ((*m_thrownObjects)[i])->getScore();
				}
			};

	return 0;
}

void HapticDevice::truncate(HDdouble* x,HDdouble* y,HDdouble* z){
	
	if(*x>MXFORCE) *x= MXFORCE;
	if(*x<MNFORCE) *x= MXFORCE;

	if(*y>MXFORCE) *y= MXFORCE;
	if(*y<MNFORCE) *y= MXFORCE;

	if(*z>MXFORCE) *z= MXFORCE;
	if(*z<MNFORCE) *z= MXFORCE;
}

bool HapticDevice::inrange(HDdouble x,HDdouble y,HDdouble z){
	
	if(x>MXDISTANCE ) return false;
	if(x<MNDISTANCE ) return false;

	if(y>MXDISTANCE) return false;
	if(y<MNDISTANCE) return false;

	if(z>0) return false;
	//if(z<MNDISTANCE) return false;

	return true;
}

hduVector3Dd HapticDevice::ComputeForce(hduVector3Dd* effector, hduVector3Dd* target, hduVector3Dd* velocity)
{
	
	hduVector3Dd objectif(*target);
	hduVector3Dd inbetween = *target - *effector;
	const HDdouble stepSize = 1.0;					
	if (inbetween.magnitude() > stepSize)
		{
		inbetween.normalize();
		objectif += inbetween*1.0;
		}
	else
		{
		objectif = *effector;
		}
	inbetween = objectif - *effector;
	HDdouble arrivaltime = inbetween[2]/(*velocity[2]);
	HDdouble yy = /*- g/2.0 **/  pow(arrivaltime,2) + (*velocity[1]) * arrivaltime + *target[1];	
	inbetween += (*velocity *0.005);
	//inbetween[1] = yy;
	inbetween[2] = 2;
	return inbetween;
}

hduVector3Dd HapticDevice::ForceToImpact(hduVector3Dd* effector,hduVector3Dd* impactpos){
	    // use Yamada and al force model
		hduVector3Dd inbetween = *impactpos - *effector ;
		hduVector3Dd force = hduVector3Dd(0,0.1,0.0);
		HDdouble phi = 2.5;
		HDdouble sigma = 0.091;

		HDdouble div = 1.0;

		HDdouble s2 = pow(sigma, 2);

		HDdouble x = inbetween.magnitude();
						
		HDdouble f = phi * ( x/ s2 ) * std::exp((s2 - pow(x,2))/2*s2);		

		inbetween.normalize();		

		HDdouble some =0;
		int l = 0;
		for(int k = 0;k<Nbr_factor;k++){
				if( m_lastFactor[k] != 0){
					some += f;
					++l; 
				}
	    	}
				
		f = (some + f)/ (l+1);

		if(f >= div && (x > 4 )){
			force +=  (div - (div/f)) * inbetween;
			//std::cout<< " if " << std::endl;
		}else{//std::cout<< " else " << std::endl;
			force += 0.2 * (inbetween + m_predForce);
		}
		//if(checkPrevious())
		//	force +=  0.03 * inbetween;

		//std::cout<< " x " << x << " f " << f  << " " << force[0] << "  " << force[1] << " " << force[2] << std::endl;
		//std::cout<<" " << inbetween[0] << "  " << inbetween[1] << " " << inbetween[2] << std::endl;

		m_lastFactor[m_factor_index] = f;		
		m_factor_index = (m_factor_index + 1) % Nbr_factor;
		m_predForce = force;
		
	return force ;
}


void HapticDevice::cleanHistory(){

	m_factor_index = 0;
	m_selected_index = 0;
	m_predForce.set(0.0,0.0,0.0);

	for(int k = 0;k<Nbr_factor;k++)
		m_lastFactor[k] = 0;

	for(int k = 0;k<Nbr_previous;k++)
		m_lastSelected[k] = false;

	m_previous = NULL;
}
void HapticDevice::addPrevious(btRigidBody* target){
	if(m_previous != NULL){
		if(m_previous == target)
			m_lastSelected[m_selected_index] = true;

		m_selected_index = (m_selected_index + 1) % Nbr_previous;
	}
	m_previous = target;
}

bool HapticDevice::checkPrevious(){
	int count =0;

	for(int k = 0;k<Nbr_previous;k++)
		if(m_lastSelected[k] == false)
			count++;
	
	return (count < Nbr_previous/2);
}

void HapticDevice::setImpactPos(btVector3* pos){
	m_impactPos = new btVector3(*pos); 
	m_hss[0].setImpactPos(new btVector3(*pos));
}

void HapticSynchronizer::setImpactPos(btVector3* m_impactPos){
	m_data->m_impactPos = m_impactPos;
}

btVector3 HapticDevice::getEffectorPosition(){
	return m_effectors[0].getOrigin();
}

void HapticDevice::setGround(btRigidBody* ground){
	m_ground = ground;
}

hduVector3Dd HapticDevice::trajectoryLine(hduVector3Dd p1,hduVector3Dd p2){
	
	HDdouble a =  (p2[1] - p1[1])/(p2[0] - p1[0]);
	HDdouble b = p1[1] - a * p1[0];
	//std::cout<<"  a"<<a<<" b"<<b<<std::endl;
	return hduVector3Dd(a,b,0);
}

HDdouble HapticDevice::betweenTwoPoints(hduVector3Dd p1, hduVector3Dd p2){
	
	return sqrt(pow((p1[0] - p2[0]) , 2)+
				pow((p1[1] - p2[1]) , 2)+
				pow((p1[2] - p2[2]) , 2));	
}

HDdouble HapticDevice::distanceToPath(hduVector3Dd path,hduVector3Dd p){
	return abs(path[0] * p[0] + path[1] * p[1]) / sqrt(pow(path[0],2)+pow(path[1],2)); 
}

void HapticDevice::setPossibleImpactPoints(btVector3* impact){
	for(int i =0 ; i < m_thrownRigids->size(); i++)
	{
		m_possibleImpact.push_back(&impact[i]);		
	}
}

void HapticDevice::clearPossibleImpactPoints(){
	m_possibleImpact.clear();	
}



void HapticDevice::setTrajectory(std::vector<btVector3*>* points, int index,unsigned int stop){
	//std::cout << stop << std::endl;
	for(unsigned int i = 0; i < stop; i++)
	{
		if(i > stop) return;
		if(i < (stop/2) ) continue;
		m_trajectory[index].push_back((*points)[i]);		
		
	}
		
}

void HapticDevice::FeadBack(){
	m_Feedback = !m_Feedback;
}
HDdouble HapticDevice::distanceToTrajectory(hduVector3Dd eff,int index,btTransform* invCam){
	HDdouble d = DBL_MAX;
	if(m_trajectory[index].size() > 0)
		for(int i = 0; i < m_trajectory[index].size(); i++){			
			btVector3* temp = (m_trajectory[index])[i];		
			hduVector3Dd p = invertTransform(temp, invCam);
			HDdouble d1 = betweenTwoPoints(eff,p);
			if(d1 < d){
				//m_hipoPos = (m_trajectory[index])[i];
				m_possibleImpact[index] = (m_trajectory[index])[i];
				d = d1;
			}
			if(temp->y() <= getEffectorPosition().y())
				return d;			
		}

	return d;
}

void HapticDevice::clearTrajectory(){
	for(int i =0 ; i < ThronNumber;i++)
		m_trajectory[i].clear();
}

int HapticDevice::getCaughtIndex(){
	int j = 0;
	if(m_caught != NULL)
		for(unsigned int i= 0;i< m_thrownRigids->size();i++)
			{	
				j++;
				if((*m_thrownRigids)[i] == m_caught)
					return j;
			}
	
    return -1;
}