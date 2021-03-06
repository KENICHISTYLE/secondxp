#include "DualTouch.h"

DualTouch::DualTouch(void)
{
	m_cursorColors[0]=red;
	//m_cursorColors[1]=blue;
}

DualTouch::~DualTouch(void)
{ 
	printf(" Saving logs please wait ....... \n");
	saveLogDynamic();
	deleteThrowedObjects();
	m_effObj->m_transform = NULL;
	//m_hds.m_ptr = NULL;
	//delete m_canonShape;	
	printf(" Dual Touch Done . \n");
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
	m_log_time = time(NULL);
	m_adaptationTime = time(NULL);
	m_catchs = 0;
	m_lancerNbr = 0;
	m_note = "";
	m_eval = false;
	m_withTraj = false;
	m_feed = true;
	m_catchs = 0;
	m_good_catchs = 0;
	m_bad_catchs = 0;
	m_ok_catchs = 0;
	m_throw_at_once = 3;
	m_startLog = false;
	m_actualV = 0;
	m_vIndex = 0;
	m_Fin_jeu = false;
	m_passe = 0;
	m_log_milis = 0;
	m_wait = true;
	//m_leftToTrhow = ThronNumber;
	m_renderer.init();
	m_renderer.setRepaire(m_lunch_y);
	m_physic.init();
	m_log.init();
	m_log.setOption('f',m_feed);
	m_log.setOption('t',m_withTraj);
	m_renderer.showFeedBack(m_feed);
	m_camera1.moveTo(btVector3(0.0,-35,6));
	m_left_to_launch = Nbr_launch_game;
	m_timerBegan = false;
	m_waitTime = time(NULL);
	//m_camera2.moveTo(btVector3(-0.5,-10,3));
	changeParam();
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
	//m_renderer.init();	
}

void DualTouch::createScene()
{	
	//ground
	const btScalar halfSize = 0.5f;
	btTransform * t = new btTransform(btQuaternion(),btVector3(0,0,-halfSize*2)); 
	btCollisionShape * shape = new btBoxShape (btVector3(100,halfSize*2,50));
	
	btRigidBody * body = m_physic.addRigidBody(0,t,shape);
	m_hds.setGround(body);
	m_renderer.addObject(new Object(shape,t,neutral));

	addLauncher();

	m_log.createLogFile();
	m_log.saveBeginInfo();	
}

void DualTouch::addLauncher(){
	//
	btScalar hx = 0.25;
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
		m_CanonPos[i] = ((i+1)-canonNbr/2)*4;
		t = new btTransform(btQuaternion(btVector3(1,0,0),-0.55),btVector3(m_CanonPos[i],m_lunch_y+hy,hy+1.5));
		m_canons[i] = m_renderer.addObject(new Object(m_canonShape,t,dark_Grey));			
	}  
	
	 m_canon_step = 0.05;

	 for(int i = 0; i< ThrowV; i++)
		 m_throwVelocity[i] = (i+1) * 6;

	 shuffleV();
}

void DualTouch::shuffleV(){
	//shuffle velocities
	for(int i = 0; i< ThrowV; i++){
		 int r = i + (rand() % (ThrowV-i)); // Random remaining position.
		 btScalar temp = m_throwVelocity[i];
		 m_throwVelocity[i] = m_throwVelocity[r];
		 m_throwVelocity[r] = temp;
	}
}

void DualTouch::randomMoveCanons(){
	
	
	btScalar center = ((1)-canonNbr/2)*4;
	if(abs(m_CanonPos[0]+ m_canon_step -center) > max_canon_dep)
		m_canon_step = -m_canon_step;

	for(int i = 0; i < canonNbr ; i++){

		if( i % 2 == 0  ){
			m_CanonPos[i] += m_canon_step;
			m_canons[i]->getTransform()->getOrigin().setX(m_CanonPos[i]);
		}else{
			m_CanonPos[i] -= m_canon_step;
			m_canons[i]->getTransform()->getOrigin().setX(m_CanonPos[i]);
		}
	}
}

void DualTouch::changeParam(){
	if(m_feed){
		m_feed = false;
		m_withTraj = true;
	}else{
		m_feed = true;
		m_withTraj = false;
	}
	m_log.setOption('f',m_feed);
	m_log.setOption('t',m_withTraj);	
	m_renderer.showFeedBack(m_feed);
};

void DualTouch::gameStatus(){
	if(m_left_to_launch > 0){
		m_left_to_launch--;

		if(m_left_to_launch % ThrowV == 0){		
			m_actualV = m_throwVelocity[m_vIndex];
			m_vIndex = (m_vIndex + 1 % ThrowV);
			m_throw_at_once = 3;
		}

		ostringstream oss;
		oss << (int) (Nbr_launch_game - m_left_to_launch);
		string s = "# Throw number "+ oss.str() ;
		oss.str("");
		oss << Nbr_launch_game;
		s += " of " + oss.str() + "\n";

		m_log.startLogWrite();
		m_log.writeToLog(s.c_str());

		m_log.writeToLog("# Type of Balls \n");
		for(unsigned int i=0; i< m_throwed_object_list.size(); i++)
		{
			oss.str("");
			oss << i;
			s = " Ball " + oss.str() + " ->" + ballValue(m_throwed_object_list[i]) + "\n";
			m_log.writeToLog(s.c_str());
		}

		m_log.endLogWrite();

		return;
	}	

	
	m_passe++;
	if(m_passe >= 2)
		m_Fin_jeu = true;
	shuffleV();
	reportScoreInfo();
	m_vIndex = 0;
	m_catchs = 0;
	m_good_catchs = 0;
	m_bad_catchs = 0;
	m_ok_catchs = 0;
	m_score = 0;
	ostringstream oss;
	oss << (unsigned int)m_throwed_rigid_list.size();
	string s = "# Number of balls par throw "+ oss.str() + "\n" ;
	m_log.saveToLogFile(s.c_str());
	m_left_to_launch = Nbr_launch_game;
	m_throw_at_once = 3;
	changeParam();	
	m_log.saveEndInfo();
	m_log.saveBeginInfo();
	m_startLog = false;	
	m_adaptationTime = time(NULL);
	
	  
		if(m_Fin_jeu == true){			
			m_renderer.setFin();			
		}
		else{
			struct log_dyn* p = new struct log_dyn();
			p->phase = true;
			p->phaseNbr = m_passe;
			p->currentTime = m_log.getCurrent();
			m_logsVec.push_back(p);
		}
	

	m_waitTime = time(NULL);
	m_wait = true;
}

const char* DualTouch::ballValue(Object* ball){

	int score = ball->getScore();

	switch(score){
		case 20 : return " Good ball ";
		case 10 : return " Ok ball ";
		case -10 : return " Bad ball ";
		default : return " Never expected ";
	}

}

void DualTouch::reportScoreInfo(){

	ostringstream oss;
	string s = " ";
	
	m_log.startLogWrite();

		m_log.writeToLog("# Catch status \n");

		oss << m_good_catchs;
		s= " Number of Good caught balls " +  oss.str() + "\n";
		oss.str("");
		m_log.writeToLog(s.c_str());

		oss << m_bad_catchs;
		s= " Number of Bad caught balls " +  oss.str() + "\n";
		oss.str("");
		m_log.writeToLog(s.c_str());

		oss << m_ok_catchs;
		s= " Number of  Ok caught balls " +  oss.str() + "\n";
		oss.str("");
		m_log.writeToLog(s.c_str());

		oss << m_catchs;
		s= " Number of caught balls " +  oss.str() + "\n";
		oss.str("");
		m_log.writeToLog(s.c_str());

	m_log.endLogWrite();

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
	m_effObj = new Object(shape,&(m_hds.m_effectors[deviceId]),m_cursorColors[deviceId]);
	m_renderer.addObject(m_effObj);

}

void DualTouch::throwMultiObject(btScalar Onumber, float canonPos, int index){
	   	float dec = 0.0f;		

		int r = Onumber;

		//int z = (rand() % 10) + 1 ;
		//int y = (rand() % 3) ;
		//m_goodToCatch[index] = y;
		m_velocityZ = Gut_vz + 5;
		m_velocityY = Gut_vy + m_actualV; 
		

		btVector3 targetpos1 = btVector3(canonPos+dec,m_lunch_y,m_lunch_z);
		btVector3 targetpos2 = btVector3(canonPos-dec,m_lunch_y,m_lunch_z);

		btTransform* t ; 		
		btCollisionShape * shape ;	

		btRigidBody* body;
		Object* obj;

		m_impactY = m_hds.getEffectorPosition().y() ;		
		
		for(unsigned int i = 0;i<Onumber;i++)
		{			  
		  int f = (rand() % 35) + 0.1 ;
		  int initial_x;
		  int var = (index)%4;
		  if(index % 2 == 0)
			  m_velocityZ = Gut_vz + ( var + 0.5) ;
		  else 
			  m_velocityZ = Gut_vz - ( var - 0.2) ;

		  if (f % 2 == 0){
			initial_x = (-f/10) -0.5;			
			t = new btTransform(btQuaternion(),targetpos1); 
			m_throwed_x.push_back(canonPos+dec);		
			
		  }else{
			initial_x= (f/10) +0.5;
			
			t = new btTransform(btQuaternion(),targetpos2);
			m_throwed_x.push_back(canonPos-dec);		
			
		  }

		  shape = new btSphereShape (Ball_Size);		  
		  body = m_physic.addRigidBody(BALL_MASS,t,shape);		
		  switch(index){

		  case 0 :   obj = new Object(shape,t,light_red);
			         obj->setScore(20);
					 break;
		  case 2 :  
					 obj = new Object(shape,t,blue);
					 obj->setScore(10);
					 break;		  
		  default:  
			         obj = new Object(shape,t,yellow);
			         obj->setScore(-10);
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
void DualTouch::getFinalPos(btTransform* target,unsigned int targetIndex, btScalar vx,  btScalar x_dec){
	float time = 0;	
	int i = 0;
	unsigned int j = 0;
	bool s = true;
	unsigned int stop = 0;
	btVector3 gravity = m_physic.m_dynamicsWorld->getGravity();
	btScalar y = m_lunch_y;
	btScalar z = 0;
	btScalar x = 0;
	btVector3* p;
	while(y>m_impactY - m_lunch_y && j< max_calculated_points){
		++j;
		++i;
		y = m_velocityY * cos(m_theta) * time;
		y = m_lunch_y - y;
		z = (gravity.z()/2  * pow(time,2) ) +( m_velocityZ * sin(m_theta)*time ) +  m_lunch_z ;
		x =vx*(time-m_timeSpeed)+x_dec;
		time += m_timeSpeed; 

		p = new btVector3(x,y,z);
		m_trajectory[targetIndex].push_back(p);
		
		if(y <= m_impactY  && s)
		{
			m_impactPossible[targetIndex] = btVector3(x,y+0.1,z);
			stop = j;			
			s = false;
		}
	}
	
	if(m_withTraj)
		m_renderer.setPoints(&(m_trajectory[targetIndex]), targetIndex, &m_throwed_object_list[targetIndex]->m_color);
	m_hds.setTrajectory(&(m_trajectory[targetIndex]), targetIndex, i);	
}

void DualTouch::setVelocityTarget(btScalar time,btRigidBody* target,btScalar x){
	
	btVector3 gravity = m_physic.m_dynamicsWorld->getGravity();
	
	btScalar z = (gravity.z()/2 * pow(time,2)) +( m_velocityZ * sin(m_theta)*time) +  m_lunch_z;
		
	btScalar vy = -m_velocityY * cos(m_theta);
	btScalar vz = gravity.z()*time  +  m_velocityZ * sin(m_theta);
	target->setLinearVelocity(btVector3(x,vy,vz));
	
}

void DualTouch::setAfterColideCoord(){
	
	return;
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
			
		//m_renderer.setPoints(&(m_trajectory[index]), index);
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
	switch(score){
		case 20: m_good_catchs++;
				 break;
		case 10: m_ok_catchs++;
				 break;
		case -10: m_bad_catchs++;
				  break;
	}
	m_catchs++;
		//cout << " index " << index  << " val " << m_goodToCatch[index] << " catch " << m_catchs << " lanced " << m_lancerNbr << endl;
    m_note = "";
	if( m_score >= 200)
		m_note = " Genial !!!! ";
	//cout << " index " << index  << " val " << m_goodToCatch[index] << endl;	
}

string DualTouch::stringFromBtvector(btVector3* vec){
	
	ostringstream oss;
	double x,y,z;
	x = (btScalar) vec->getX();
	y = (btScalar) vec->getY();
	z = (btScalar) vec->getZ();

	oss << x ;
	
	string s =  " " + oss.str();
	oss.str("");

	oss << y ;
	s +=  " " + oss.str();
	oss.str("");

	oss << z ;
	s +=  " " + oss.str();

	return s;
}

string DualTouch::colorFromScore(int score){
	switch(score){
		case 20: return string(" Red");
		case 10: return string(" Blue");
		case -10: return string(" Yellow");
		default :
				return string(" Unexpected ");
	}
}

void DualTouch::logDynamic(bool lancer){
	// 
	struct log_dyn* myLog = new struct log_dyn();
	myLog->effcPosition = m_hds.getEffectorPosition();
	myLog->vitesseLancer = m_velocityY;
	myLog->currentTime = m_log.getCurrent();
	myLog->lancer = lancer;
	for(unsigned int i =0; i < m_throwed_rigid_list.size(); i++){
		myLog->throwed[i].pos = m_throwed_object_list[i]->getTransform()->getOrigin();
		myLog->throwed[i].score = m_throwed_object_list[i]->getScore();
		myLog->throwed[i].isBall = true;			
	}

	myLog->caught = m_hds.isCaught();

	if(myLog->caught){
		myLog->index = m_hds.getCaughtIndex();
		myLog->caughtValue = m_hds.getCaughtScore();
		myLog->score = m_score;
		myLog->catchNbr = m_catchs;
		myLog->goodCatchNbr = m_good_catchs;
		myLog->okCatchNbr = m_ok_catchs;
		myLog->badCatchNbr = m_bad_catchs;		
	}

	m_logsVec.push_back(myLog);
}

void DualTouch::saveLogDynamic(){
	if(m_logsVec.size() <= 0)
		return;
	m_log.dynamicBegin();
	for(unsigned int st = 0; st < m_logsVec.size() ; st ++){

		string s = "***** status at : ";
		ostringstream oss;
		oss << m_logsVec[st]->currentTime.hh;
		s += oss.str() + ":";
		oss.str("");
		oss << m_logsVec[st]->currentTime.mm;
		s += oss.str() + ":";
		oss.str("");
		oss << m_logsVec[st]->currentTime.ss;
		s += oss.str() + ":";
		oss.str("");
		oss << m_logsVec[st]->currentTime.mil;
		s += oss.str() + " **** \n";
		oss.str("");
		m_log.dynamicWrite(s.c_str());

		if( m_logsVec[st]->lancer){
			s = "# lancer \n";		
			m_log.dynamicWrite(s.c_str());
		}

		btVector3 temp = m_logsVec[st]->effcPosition;
		s = "# Effector position ";
		s += stringFromBtvector(&temp);	
		s += "\n";	

		m_log.dynamicWrite(s.c_str());
	    
		
		if(m_logsVec[st]->phase){
			oss << m_logsVec[st]->phaseNbr;
			s = "# Nouvelle phase numero " + oss.str() + "\n";
			m_log.dynamicWrite(s.c_str());
			oss.str("");
		}
		else{
				
			int j =0;
			for(unsigned int i =0; i < ThronNumber; i++)
			{
				if(m_logsVec[st]->throwed[i].isBall){
					oss << (unsigned int) i;
					btVector3 temp = m_logsVec[st]->throwed[i].pos;
					s = "# Ball " + oss.str() +"\n# Position ";
					s += stringFromBtvector(&temp);
					s+= "\n";
					m_log.dynamicWrite(s.c_str());

					int score = m_logsVec[st]->throwed[i].score;

					s = "# Color ";
					s += colorFromScore(score);
					s += "\n";
					m_log.dynamicWrite(s.c_str());

		
					oss.str("");
					oss << score;
					s = "# Value " + oss.str() + "\n";
					m_log.dynamicWrite(s.c_str());
					oss.str("");
					j++;
				}
			}

			oss << j;
			s = "# NBR balls " + oss.str() +"\n ";				
			m_log.dynamicWrite(s.c_str());

			oss.str("");
			oss << m_logsVec[st]->vitesseLancer;
			s = "# Vitesse de depart " + oss.str() + "\n";
			m_log.dynamicWrite(s.c_str());
			oss.str("");
		
			if(m_logsVec[st]->caught){
				m_log.dynamicWrite("## A ball was caught \n");
				oss.str("");

				int score = m_logsVec[st]->caughtValue;		
				oss << m_logsVec[st]->index;

				s = "# Ball " + oss.str() +"\n# Color ";
				s += colorFromScore(score);
				s += "\n";
				m_log.dynamicWrite(s.c_str());						

				oss.str("");
				oss << score;
				s = "# Value " + oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");

				oss << m_logsVec[st]->score;
				string s = "# Actuel score " + oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");

	
				oss << m_logsVec[st]->catchNbr;
				s = "# Actuel caught number " + oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");

	
				oss << m_logsVec[st]->goodCatchNbr;
				s= "# Actuel Good caught balls " +  oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");

				oss << m_logsVec[st]->badCatchNbr;
				s= "# Actuel of Bad caught balls " +  oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");

				oss << m_logsVec[st]->okCatchNbr ;
				s= "# Actuel of  Ok caught balls " +  oss.str() + "\n";
				m_log.dynamicWrite(s.c_str());
				oss.str("");
			}
		}
		delete m_logsVec[st];
	}
	m_logsVec.clear();
	string s= "###### Fin experiance ######## \n" ;
	m_log.dynamicWrite(s.c_str());
	m_log.dynamicEnd();
}

void DualTouch::waitFeadBack(){	
	if(!m_Fin_jeu){
		if(m_hds.isReadyLaunch()){
		
			m_time = time(NULL);
			deleteThrowedObjects();	
			btScalar canons[canonNbr];

			int b = (rand() % (canonNbr-4))+2;
			for (int i=0; i<canonNbr; i++) { // shuffle
				int r = (b+i) % canonNbr; 		    
				canons[i] = m_CanonPos[r];			
			}	
		

			for(int i = 0; i< m_throw_at_once; i++){
				throwMultiObject(1, canons[i],i);
			}
		
			if(m_startLog)
				gameStatus();
			logDynamic(true);
			setHapticParam();
			m_hds.waitTargetChoice();
			m_hds.setWaitLunch();	
			//m_log.saveTrajectory(m_trajectory,max_calculated_points);
			m_eval = true;
			if(m_throw_at_once < ThronNumber)
				m_throw_at_once++;
			m_timerBegan = false;
		}
	
		// current time	
		SYSTEMTIME stime;
		GetSystemTime(&stime);
		WORD millis = (stime.wSecond * 1000) + stime.wMilliseconds;

	
		WORD m = millis - m_log_milis;

			if(  m > 0.0001  &&  m_startLog && m_throwed_object_list.size() > 0)
			{
				logDynamic(false);
				m_log_milis = m;
			}
   
	
		// case physics collision
		if(m_physic.collHapend()){
			m_log.dynamicBegin();		
			string	s= "### Collision !!! ### \n " ;	
			m_log.dynamicWrite(s.c_str());
			m_log.dynamicEnd();
			//m_time = now;					
			//m_hds.Lunch();
			//m_hds.deactivateMove();
			//setAfterColideCoord();
		}
	   
		string* s = new string("");
		ostringstream oss;

		// timed launch
		time_t now = time(NULL);
		time_t time = now - m_time;
		if(testIfCanThrow()){
			
			if(time > Time){
				m_time = now;					
				m_hds.Lunch();
				m_hds.deactivateMove();		
				m_renderer.stopShowTime();
			}else{
				oss << (int) (Time - time);	
				*s = " Prochain lancer dans  ... " + oss.str() + " s ! " ;		
				m_renderer.setLeftToThrow(s->c_str());
			}

		}else{
			m_time = now ;	
			if(!m_feed)
				m_renderer.setDir(&m_hds.getEffectorPosition(),&m_hds.getImpactDir());
		}

		// eval score
		if(m_hds.isCaught() && m_eval){
			m_eval = false;
			if( m_startLog){
				evaluateScore();
				logDynamic(false);
			}
			m_hds.freeIf();
		}
		
		oss.str("");
		*s = "";
		if(m_score > 0){
			oss << m_score;
			*s += " Votre score : " + oss.str() + ". " + m_note;
		}

		m_renderer.setText(s);

	}
				
	
}

void DualTouch::idle()
{	
	
	m_hds.run();	
	m_physic.run();
	m_physic.tick();
	m_hds.feedback(*m_physic.m_dynamicsWorld);	
	inPauseMode();
	
}

void DualTouch::inPauseMode(){
	if(!m_wait){
		start();
		randomMoveCanons();
		waitFeadBack();
	}else{
		time_t now = time(NULL);
		time_t time = now - m_waitTime;
		m_renderer.setWait(PauseLenght - time);
		if(time > PauseLenght){			
			m_wait = false;
			m_renderer.stopWait();
		}
	}		
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

bool DualTouch::testIfCanThrow(){
	if(!m_timerBegan && m_throwed_object_list.size() > 0 && !m_hds.isCaught()){
		btVector3 eff = m_hds.getEffectorPosition();
		btVector3 ball0 = m_throwed_object_list[0]->getTransform()->getOrigin();
		if(eff.y() > ball0.y()){
			m_timerBegan = true;
			return true;
		}
		return false;
	}
	m_timerBegan = true;
	return true;
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

void DualTouch::start(){

	if(!m_startLog){
		time_t now = time(NULL);	
		time_t time = now - m_adaptationTime;
		if(time > LearnTime -10){

			m_renderer.showStart(true,(int) (LearnTime - time));
		}else
			m_renderer.showStart(true,-1);

		if(time > LearnTime){
			m_time = now;
			m_startLog = true; 
			m_renderer.showStart(false,0);
		}	
	}
		
}

void DualTouch::keyboard1(unsigned char key, int x, int y)
{
	//m_camera1.m_key = key;
	switch(key){
		case('t'):  m_withTraj = !m_withTraj;					
					m_log.setOption('t',m_withTraj);
			        if(m_withTraj)
						cout << " Activate Trajectory " << endl;
					else
						cout << " Deactivate Trajectory " << endl;
					break;
		case('h'):  m_feed = !m_feed;
			        m_log.setOption('f',m_feed);
					if(m_feed)
						cout << " Activate Feedback " << endl;
					else
						cout << " Deactivate  Feedback " << endl;
					break;
		case('s'):  m_startLog = !m_startLog;			       
					if(m_feed)
						cout << " Activate log " << endl;
					else
						cout << " Deactivate  log " << endl;
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

