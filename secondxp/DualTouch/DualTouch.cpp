#include "DualTouch.h"

DualTouch::DualTouch(void)
{
	m_cursorColors[0]=red;
	//m_cursorColors[1]=blue;
}

DualTouch::~DualTouch(void)
{ 
	delete m_canonShape;	
	m_physic.exit();
}

void DualTouch::init1()
{
	m_velocityY = Gut_vy;
	m_velocityZ = Gut_vz;
	m_impactY = -2;
	m_moveTarget = false;	
	m_theta = 45;
	m_lunch_z = 3.5;
	m_lunch_y = 30;
	m_time = time(NULL);	
	m_catchs = 0;
	m_lancerNbr = 0;
	m_note = "";
	m_eval = false;
	m_withTraj = true;
	m_feed = true;
	//m_leftToTrhow = ThronNumber;
	m_renderer.init();
	m_renderer.setRepaire(m_lunch_y);
	m_physic.init();
	m_log.init();
	m_camera1.moveTo(btVector3(0.5,-10,3));
	//m_camera2.moveTo(btVector3(-0.5,-10,3));

	m_hds.addDevice("PHANToM",m_camera1.m_view);
	//m_hds.addDevice("PHANToM 2",m_camera2.m_view);
	m_hds.init();

	m_hds.setFeadBack(m_feed);
	m_hds.m_ptr=this;
	m_hds.m_newConstraint = newConstraint;
	m_hds.m_deleteConstraint = deleteConstraint;
	//m_hds.setDThrownList(m_throwed_rigid_list);

	createScene();

	createCursor(0);
	//createCursor(1);
	//m_physic.m_dynamicsWorld->setInternalTickCallback(tickCallback,this);
}

void DualTouch::init2()
{
	//m_hd2.init(std::string("PHANToM 2"));
	//m_hd2.setCameraView(&m_camera2.m_view);
	m_renderer.init();	
}

void DualTouch::createScene()
{


	//ground
	const btScalar halfSize = 0.5f;
	btTransform * t = new btTransform(btQuaternion(),btVector3(0,0,-halfSize*2)); 
	btCollisionShape * shape = new btBoxShape (btVector3(100,halfSize*2,50));
	
	btRigidBody * body = m_physic.addRigidBody(0,t,shape);
	m_hds.setGround(body);
	//body->setHitFraction(0);
	//body->setRestitution(0);
	//body->setFriction(0);
	//body->setRollingFriction(0);
	//body->setCollisionFlags( body->getCollisionFlags() | 
	//	btCollisionObject::CF_KINEMATIC_OBJECT); 
	//body->setActivationState(DISABLE_DEACTIVATION); 
	m_renderer.addObject(new Object(shape,t,neutral));

	addLauncher();

}

void DualTouch::addLauncher(){
	//
	btScalar hx = 0.3;
	btScalar hy = 0.7;
	btScalar hz = 0.3;
	btTransform * t = new btTransform(btQuaternion(),btVector3(0,m_lunch_y,hy+0.2)); 

	btCollisionShape * shape = new btBoxShape (btVector3(hx+hx,hy,40));
	
	btRigidBody * body = m_physic.addRigidBody(0,t,shape);
	m_renderer.addObject(new Object(shape,t,orange));

	body->setCollisionFlags( body->getCollisionFlags() | 
	btCollisionObject::CF_KINEMATIC_OBJECT); 
	body->setActivationState(DISABLE_DEACTIVATION);
	
	m_canonShape = new btCylinderShape(btVector3(hx,hy*2,hz));	
	for(int i = 0; i <canonNbr; i++){
		m_CanonPos[i] = ((i+1)-canonNbr/2)*3;
		t = new btTransform(btQuaternion(btVector3(1,0,0),-0.55),btVector3(m_CanonPos[i],m_lunch_y+hy,hy+1.5));
		m_canons[i] = m_renderer.addObject(new Object(m_canonShape,t,dark_Grey));			
	}  
	
}


void DualTouch::moveCanonLeft(btScalar x){
	btTransform trans;	
	//m_canon->getMotionState()->getWorldTransform(trans);
	btVector3 t = trans.getOrigin();
	trans.setOrigin(btVector3(t.x()-x,t.y(),t.z()));
	//m_canon->getMotionState()->setWorldTransform(trans);
}

void DualTouch::moveCanonRight(btScalar x){
	btTransform trans;	
	//m_canon->getMotionState()->getWorldTransform(trans);
	btVector3 t = trans.getOrigin();
	trans.setOrigin(btVector3(t.x()+x,t.y(),t.z()));
	//m_canon->getMotionState()->setWorldTransform(trans);
}

void DualTouch::teleportX(Object* canon,btScalar x){
	btTransform trans;		
	trans = *(canon->getTransform());
	btVector3 t = trans.getOrigin();
	trans.setOrigin(btVector3(x,t.y(),t.z()));	
	canon->setTransform(trans);
}


void DualTouch::rotateCanon(btVector3* rotate){
	btTransform myTrans = btTransform();
//	m_canon->getMotionState()->getWorldTransform(myTrans);
//	btQuaternion rot = myTrans.getRotation();
//	btScalar alpha = asin(rotate->x()/rotate->z())*(PI/180);
//	rot.setX(rotate->x());
//	myTrans.setRotation(rot);
//	m_canon->getMotionState()->setWorldTransform(myTrans);
}

void DualTouch::createCursor(unsigned int deviceId)
{
	btCollisionShape * shape = new btSphereShape(Effector_Size);
	btTransform *t = new btTransform(btQuaternion(),btVector3(0,0,0));

	btRigidBody *rigidBody = m_physic.addRigidBody(Effector_Mass,t,shape);  // < -------------------------------------------------
	cursors[deviceId]=rigidBody;
	rigidBody->setActivationState(DISABLE_DEACTIVATION);

	btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(* rigidBody, *t,false);
	dof6->setLinearLowerLimit(btVector3(0,0,0));
	dof6->setLinearUpperLimit(btVector3(0,0,0));
	dof6->setAngularLowerLimit(btVector3(0,0,0));
	dof6->setAngularUpperLimit(btVector3(0,0,0));

	for(int axe=0;axe<6;axe++)
	{
		dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,axe);
		dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.2f,axe);		
	}

	dof6->setUserConstraintPtr(NULL);

	m_hds.setConstraint(deviceId,dof6);

	m_physic.m_dynamicsWorld->addConstraint(dof6);  // << ------------------------------------------------------------
	m_renderer.addObject(new Object(shape,t,neutral));

	shape = new btConeShapeZ(0.25f,1);
	m_renderer.addObject(new Object(shape,&(m_hds.m_effectors[deviceId]),m_cursorColors[deviceId]));

}



void DualTouch::throwMultiObject(btScalar Onumber, float canonPos, int index){
	   	float dec = 0.0f;		

		int r = Onumber;

		int z = (rand() % 10) + 1 ;
		int y = (rand() % 3) ;
		m_goodToCatch[index] = y;
		m_velocityY = Gut_vy + y;
		m_velocityZ = Gut_vz + (z/10);

		btVector3 targetpos1 = btVector3(canonPos+dec,m_lunch_y,m_lunch_z);
		btVector3 targetpos2 = btVector3(canonPos-dec,m_lunch_y,m_lunch_z);

		btTransform* t ; 		
		btCollisionShape * shape ;	

		btRigidBody* body;
		Object* obj;

		m_impactY = m_hds.getEffectorPosition().y() ;		
		
		for(unsigned int i = 0;i<Onumber;i++)
		{			  
		  int f = (rand() % 100) + 0.1;
		  int initial_x;
		  if (index % 2 == 0){
			initial_x = (f/1000) -0.05;
			t = new btTransform(btQuaternion(),targetpos1); 
			m_throwed_x.push_back(canonPos+dec);		
			
		  }else{
			initial_x= (-f/1000) +0.05;
			t = new btTransform(btQuaternion(),targetpos2);
			m_throwed_x.push_back(canonPos-dec);		
			
		  }

		  shape = new btSphereShape (Ball_Size);		  
		  body = m_physic.addRigidBody(BALL_MASS,t,shape);		
		  switch(y){
		  case 0 :   obj = new Object(shape,t,yellow);
					 obj->setScore(-10);
					 break;
		  case 1 :   obj = new Object(shape,t,light_red);
			         obj->setScore(20);
					 break;
		  case 2 :   obj = new Object(shape,t,blue);
			         obj->setScore(10);
					 break;
		  default:  
			         obj->setScore(0);
					 obj = new Object(shape,t,gray);
					 break;
		  }
		  setVelocityTarget(m_timeSpeed,body,initial_x); 
		  m_renderer.addObject(obj);		 
		  m_throwed_rigid_list.push_back(body);
		  m_throwed_object_list.push_back(obj);
		  m_throwed_transform.push_back(new btTransform(*t));
		  getFinalPos(m_throwed_transform[index],index,initial_x,canonPos);
		}		
		// a verifier	
			
}

void DualTouch::setHapticParam(){
	m_physic.setThrown(&m_throwed_rigid_list);
	m_hds.setDThrownList(&m_throwed_rigid_list);		
	m_hds.setDThrownObject(&m_throwed_object_list);	
	m_hds.setPossibleImpactPoints(m_impactPossible);
	m_hds.waitTargetChoice();
	m_hds.deactivateMove();
	m_hds.setFeadBack(m_feed);
}


void DualTouch::setTheTargetFinalPos(btRigidBody* target,btScalar initial_x){

		//btVector3 final = getFinalPos(&target->getWorldTransform(),initial_x);
		//m_hds.setImpactPos(&final);
}		

// calculate trajectory and save final pos
void DualTouch::getFinalPos(btTransform* target,int targetIndex, btScalar vx,  btScalar x_dec){
	float time = 0;	
	int i = 0, j = 0;
	bool s = true;
	unsigned int stop = 0;
	btVector3 gravity = m_physic.m_dynamicsWorld->getGravity();
	btScalar y = m_lunch_y;
	btScalar z = 0;
	btScalar x = 0;
	btVector3* p;
	while(y>m_impactY - m_lunch_y && j< 150){
		++i;
		y = m_velocityY * cos(m_theta) * time;
		y = m_lunch_y - y;
		z = (gravity.z()/2  * pow(time,2) ) +( m_velocityZ * sin(m_theta)*time ) +  m_lunch_z ;
		x =vx*(time-m_timeSpeed)+x_dec;
		time += m_timeSpeed; 
		if( i % 8 == 0){
			j++;
			p = new btVector3(x,y,z);
			m_trajectory[targetIndex].push_back(p);
		}
		if(y <= m_impactY  && s)
		{
			m_impactPossible[targetIndex] = btVector3(x,y+0.1,z);
			stop = j;			
			s = false;
		}
	}
	
	if(m_withTraj)
		m_renderer.setPoints(&(m_trajectory[targetIndex]), targetIndex);
	m_hds.setTrajectory(&(m_trajectory[targetIndex]), targetIndex, j);	
}


void DualTouch::setVelocityTarget(btScalar time,btRigidBody* target,btScalar x){
	
	btVector3 gravity = m_physic.m_dynamicsWorld->getGravity();
	
	btScalar z = (gravity.z()/2 * pow(time,2)) +( m_velocityZ * sin(m_theta)*time) +  m_lunch_z;
		
	btScalar vy = -m_velocityY * cos(m_theta);
	btScalar vz = gravity.z()*time  +  m_velocityZ * sin(m_theta);
	target->setLinearVelocity(btVector3(x,vy,vz));
	
}

void DualTouch::setAfterColideCoord(){
	
	btVector3 gravity = m_physic.m_dynamicsWorld->getGravity();
	
	m_impactY = m_hds.getEffectorPosition().y() ;	
	m_physic.clearColide();
	// clear trajectory

	for (unsigned int i = 0; i < ThronNumber; i++)
	{
		for(unsigned int j = 0; j < m_trajectory[i].size(); j++)
			delete (m_trajectory[i])[j];
		m_trajectory[i].clear();		
	}

	m_hds.clearPossibleImpactPoints();
	m_renderer.clearPoints();

	for(unsigned int targ = 0; targ <m_throwed_rigid_list.size(); targ++){
		btTransform myTrans = m_throwed_rigid_list[targ]->getWorldTransform();
		btVector3 deb = myTrans.getOrigin();
		btVector3 velocity = m_throwed_rigid_list[targ]->getLinearVelocity();
		
		btScalar y = deb.y();
		btScalar z = 0;
		btScalar x = 0;
		float time = 0;	
		btScalar vy = - velocity.y()/cos(m_theta);
	    btScalar vz = -(gravity.z()*time)/sin(m_theta);

		int i = 0, j = 0, index = 0 ;
		bool s = true;
		unsigned int stop = 0;
		btVector3* p;
		while( y>m_impactY - deb.y() && j<300){
			++i;
			y = vy * cos(m_theta) * time;
			y = deb.y() - y;
			z = (gravity.z()/2  * pow(time,2) ) +( vz * sin(m_theta)*time ) +  deb.z() ;
			x = velocity.x()*(time-m_timeSpeed);
			time += m_timeSpeed; 
			if( i % 8 == 0){
				j++;
				p = new btVector3(x,y,z);
				m_trajectory[index].push_back(p);
			}
			if(y <= m_impactY  && s)
			{
				m_impactPossible[index] = btVector3(x,y+0.1,z);
				stop = j;			
				s = false;
			}
		}
			
		m_renderer.setPoints(&(m_trajectory[index]), index);
		m_hds.setTrajectory(&(m_trajectory[index]), index, j);	
		++index;
	}
}

void DualTouch::deleteThrowedObjects(){

	if(m_throwed_rigid_list.size() > 0){
	 // clear rigd bodies		
	m_physic.delthrown(&m_throwed_rigid_list);	 
	m_physic.clearColide();		
	
    // clear graphics
	for (unsigned int i = 0; i < m_throwed_object_list.size(); i++)
    {		
		m_renderer.delObject(m_throwed_object_list[i]);

		delete m_throwed_transform[i];		
    }

	// clear trajectory

	for (unsigned int i = 0; i < ThronNumber; i++)
	{
		for(unsigned int j = 0; j < m_trajectory[i].size(); j++)
			delete (m_trajectory[i])[j];
		m_trajectory[i].clear();		
	}
	
	for (int i = 0; i < ThronNumber; i++)
		m_goodToCatch[i] = -1;

	  m_throwed_rigid_list.clear();
	  m_throwed_object_list.clear();	
	  m_throwed_transform.clear();	
	  m_throwed_x.clear();

	  m_hds.clearPossibleImpactPoints();
	  m_renderer.clearPoints();
	  m_hds.resetThrow();
	  //cout<<"objects " << m_throwed_object_list.size()<<endl;
	  //cout<<"rigid " << m_throwed_rigid_list.size()<<endl;


	}
}

void DualTouch::reset()
{
	m_physic.reset();
	for(int i=0;i<NB_DEVICES_MAX;i++)
	{
		cursors[i]->setActivationState(DISABLE_DEACTIVATION);
		
		cursors[i]->setWorldTransform(m_hds.m_effectors[i]);
	}
	
}

void DualTouch::reshape(int width, int height)
{
	glViewport(0,0,width,height); 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();  
	gluPerspective(45,(width/(float)height),0.01,1000); 
	glMatrixMode(GL_MODELVIEW); 
}

void DualTouch::display1(){  
	m_renderer.setClearColor(gray[0],gray[1],gray[2]);
	m_camera1.lookAt();
	m_renderer.display();
	glDisable(GL_LIGHTING);
	m_physic.render();
}

void DualTouch::display2()
{
	m_renderer.setClearColor(blue[0],blue[1],blue[2]);
	m_camera2.lookAt();
	m_renderer.display();	
	glDisable(GL_LIGHTING);
	m_physic.render();
}

void DualTouch::evaluateScore(){
	m_lancerNbr++;
	if(m_score <= 0) m_score = 0;
	int score = m_hds.getCaughtScore();
	if(score != 0)	{
		m_score += score;
		}
		m_catchs++;
		//cout << " index " << index  << " val " << m_goodToCatch[index] << " catch " << m_catchs << " lanced " << m_lancerNbr << endl;
    m_note = "";
	if( m_score >= 200)
		m_note = " Genial !!!! ";
	//cout << " index " << index  << " val " << m_goodToCatch[index] << endl;
}

void DualTouch::waitFeadBack(){	
	if(m_hds.isReadyLaunch()){
		//m_log.printElepsedTime();
		m_time = time(NULL);
		deleteThrowedObjects();			
		for (int i=0; i<canonNbr; i++) { // shuffle
			int r = i + (rand() % ( canonNbr - i)); 
		    int temp = m_CanonPos[i];
			m_CanonPos[i] = m_CanonPos[r];
			m_CanonPos[r] = temp;
        }

		for(int i = 0; i< ThronNumber; i++){
			throwMultiObject(1, m_CanonPos[i],i);
		}
			
		setHapticParam();
		m_hds.waitTargetChoice();
		m_hds.setWaitLunch();		
		m_eval = true;
	}
	
	/*if(m_hds.isTargetChosen()){
		for(unsigned int i = 0; i<m_throwed_rigid_list.size(); i++){
			btRigidBody* t= m_hds.getTarget();
			if(m_throwed_rigid_list[i] == t){
				btVector3 final =  m_impactPossible[i]; 
				m_hds.setImpactPos(&final);				
				m_hds.activateMove();
			}
		}

	}*/

	if(m_physic.collHapend())
		setAfterColideCoord();

	// timed launch
	time_t now = time(NULL);
    time_t time = now - m_time;

	if(!m_hds.isCaught()){
		if(time > Time){
			m_time = now;					
			m_hds.Lunch();
			m_hds.deactivateMove();
		}

	}else
		m_time = now ;	

	if(m_hds.isCaught() && m_eval){
	    m_eval = false;
		evaluateScore();
	}

	ostringstream oss;
	oss << (int) (Time - time);
	string* s = new string();
	*s = " Temps restant avant lancer ... " + oss.str() + " secondes  ! " ;
	oss.str("");	
	if(m_score > 0){
		oss << m_score;
		*s += " Votre score : " + oss.str() + ". " + m_note;
	}

	m_renderer.setText(s);
}

void DualTouch::idle()
{	
	m_hds.run();	
	m_physic.run();
	m_physic.tick();
	m_hds.feedback(*m_physic.m_dynamicsWorld);	
	
	waitFeadBack();
}

void DualTouch::tickCallback(btDynamicsWorld *world, btScalar timeStep)
{
	DualTouch * dt = (DualTouch *) world->getWorldUserInfo();
	dt->m_hds.run();
	dt->m_physic.tick();
	dt->m_hds.feedback(*dt->m_physic.m_dynamicsWorld);
}

void DualTouch::newConstraint(void *ptr ,btRigidBody * body,unsigned int id)
{
	DualTouch * my = (DualTouch *)ptr;
	if(body!=NULL)
	{
		Object * object =my->m_renderer.getObject(body->getCollisionShape());
		if(object->m_color==neutral)
		{
			//object->m_color = my->m_cursorColors[id];
		}
		else
		{
			object->m_texture =true;
			//object->m_color = neutral;
		}
	}
}

void DualTouch::deleteConstraint(void * ptr,btRigidBody * body,unsigned int id)
{
	DualTouch * my = (DualTouch *)ptr;
	if(body!=NULL)
	{
		Object * object =my->m_renderer.getObject(body->getCollisionShape());
		if(object->m_texture)
		{
			object->m_texture =false;
			/*if(id==0)
				object->m_color = my->m_cursorColors[1];
			else
				object->m_color = my->m_cursorColors[0];*/
		}
		else
		{
			//object->m_color = neutral;
		}
	}
}

void DualTouch::keyboard1(unsigned char key, int x, int y)
{
	//m_camera1.m_key = key;
	switch(key){
		case('t'):  m_withTraj = !m_withTraj;
			        if(m_withTraj)
						cout << " Activate Trajectory " << endl;
					else
						cout << " Deactivate Trajectory " << endl;
					break;
		case('h'):  m_feed = !m_feed;
					if(m_feed)
						cout << " Activate Feedback " << endl;
					else
						cout << " Deactivate  Feedback " << endl;
					break;
		case('+'):
		case('p'): m_velocityY += 0.01f;
				   m_velocityZ -= 0.0001f;
				   cout<<" velocity Y "<<m_velocityY<<endl;
				break;
		case('-'):
		case('o'): m_velocityY -= 0.01f;
				   m_velocityZ += 0.0001f;
				   cout<<" velocity Y "<<m_velocityY<<endl;
				break;
		default:
			m_camera1.keyboardDown(key);
	};

	
}

void DualTouch::keyboard2(unsigned char key, int x, int y)
{
	m_camera2.keyboardDown(key);
}

void DualTouch::keyboardUp1(unsigned char key, int x, int y)
{
	m_camera1.keyboardUp(key);
}

void DualTouch::keyboardUp2(unsigned char key, int x, int y)
{
	m_camera2.keyboardUp(key);
}

void DualTouch::mouse1(int button, int state, int x, int y)
{
	m_camera1.mouseDown(button,x,y);
	
}

void DualTouch::mouse2(int button, int state, int x, int y)
{
	m_camera2.mouseDown(button,x,y);
}

void DualTouch::motion1(int x, int y)
{
	m_camera1.mouseMotion(x,y);
}

void DualTouch::motion2(int x, int y)
{
	m_camera2.mouseMotion(x,y);
}