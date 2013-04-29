#include "Renderer.h"
#include <math.h>
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

// cube ///////////////////////////////////////////////////////////////////////
//    v6----- v5
//   /|      /|
//  v1------v0|
//  | |     | |
//  | |v7---|-|v4
//  |/      |/
//  v2------v3

// vertex coords array
GLfloat boxVertices[] = {0.5,0.5,0.5,  -0.5,0.5,0.5,  -0.5,-0.5,0.5,  0.5,-0.5,0.5,      // v0-v1-v2-v3
                      0.5,0.5,0.5,  0.5,-0.5,0.5,  0.5,-0.5,-0.5,  0.5,0.5,-0.5,        // v0-v3-v4-v5
                      0.5,0.5,0.5,  0.5,0.5,-0.5,  -0.5,0.5,-0.5,  -0.5,0.5,0.5,        // v0-v5-v6-v1
                      -0.5,0.5,0.5,  -0.5,0.5,-0.5,  -0.5,-0.5,-0.5,  -0.5,-0.5,0.5,    // v1-v6-v7-v2
                      -0.5,-0.5,-0.5,  0.5,-0.5,-0.5,  0.5,-0.5,0.5,  -0.5,-0.5,0.5,    // v7-v4-v3-v2
                      0.5,-0.5,-0.5,  -0.5,-0.5,-0.5,  -0.5,0.5,-0.5,  0.5,0.5,-0.5};   // v4-v7-v6-v5

GLfloat boxTextures[] = {1,0,0,  0,0,0,  0,1,0, 1,1,0,      // v0-v1-v2-v3
						0,0,0,  0,1,0,  1,1,0,  1,0,0,        // v0-v3-v4-v5
						0,0,0,  0,1,0,  1,1,0,  1,0,0,        // v0-v5-v6-v1
						0,0,0,  0,1,0,  1,1,0,  1,0,0,    // v1-v6-v7-v2
						0,0,0,  0,1,0,  1,1,0,  1,0,0,     // v7-v4-v3-v2
						1,1,0,  0,1,0,  0,0,0, 1,0,0};   // v4-v7-v6-v5


// normal array
GLfloat boxNormals[] = {0,0,1,  0,0,1,  0,0,1,  0,0,1,             // v0-v1-v2-v3
                     1,0,0,  1,0,0,  1,0,0, 1,0,0,              // v0-v3-v4-v5
                     0,1,0,  0,1,0,  0,1,0, 0,1,0,              // v0-v5-v6-v1
                     -1,0,0,  -1,0,0, -1,0,0,  -1,0,0,          // v1-v6-v7-v2
                     0,-1,0,  0,-1,0,  0,-1,0,  0,-1,0,         // v7-v4-v3-v2
                     0,0,-1,  0,0,-1,  0,0,-1,  0,0,-1};        // v4-v7-v6-v5



Renderer::Renderer(void)
{
	m_wireColor[0]=0.2f;
	m_wireColor[1]=0.2f;
	m_wireColor[2]=0.2f;

	for(int i = 0; i < ThronNumber;i++)
		m_points[i].clear();
	m_text = new std::string(" Hello ");
	m_oultines =true;
	m_shapecaches.clear();
	m_repaire = 0;
}

Renderer::~Renderer(void)
{
	for (int i=0;i<m_shapecaches.size();i++)
	{
		m_shapecaches[i]->~ShapeCache();
		btAlignedFree(m_shapecaches[i]);
	}
	m_shapecaches.clear();

	for(unsigned int i=0;i<m_objects.size();i++)
		delete m_objects[i];

	m_objects.clear();	

	glDeleteTextures(1,(const GLuint*) &m_texturehandle);

	printf(" Renderer Done . \n");
}

Object* Renderer::addObject(Object * object)
{
	m_objects.push_back(object);
	return object;
}

void Renderer::delObject(Object* o)
{
	int j = 0;
	for (std::vector<Object*>::iterator i =m_objects.begin(); i != m_objects.end(); i++)
    {		
		if(o == m_objects[j])
		{
			delete m_objects[j];
			m_objects.erase(i);
			return;
		}

		j++;
    }
	
}

void Renderer::replaceObject(Object* oldObject, Object* newObject)
{
	for(unsigned int i=0;i<m_objects.size();i++)
		if(oldObject == m_objects[i])
		{
		m_objects[i] = NULL;
		m_objects[i] = newObject;
		//delete m_objects[i];
		//delete o;
		}
	
}

void Renderer::init()
{
	m_matDiffuse[0] = 0.6f;m_matDiffuse[1] = 0.6f;m_matDiffuse[2] = 0.6f;m_matDiffuse[3] = 1.0f;
	m_matAmbient[0] = 0.5f;m_matAmbient[1] = 0.5f;m_matAmbient[2] = 0.5f;m_matAmbient[3] = 1.0f;

	GLfloat m_lightDiffuse[] = {0.6f,0.6f,0.6f};
	GLfloat m_lightAmbient[] = {0.5f,0.5f,0.5f,1.0f};
	GLfloat m_lightSpecular[] = {0.0f,0.0f,0.0f,1.0f};

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_MULTISAMPLE);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);
	glLineWidth(2);

	glEnable(GL_LIGHT0);
	m_lightPos[0]=0;
	m_lightPos[1]=-100;
	m_lightPos[2]=100;

	glLightfv(GL_LIGHT0, GL_POSITION, m_lightPos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, m_lightDiffuse);
	glLightfv(GL_LIGHT0, GL_AMBIENT, m_lightAmbient);
	glLightfv(GL_LIGHT0, GL_SPECULAR, m_lightSpecular);
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT,m_lightAmbient);

	glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,0);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,m_matDiffuse);

	/*short size = 64;
	short hsize = size/2;
	GLubyte * image=new GLubyte[size*size*3];
	for(int y=0;y<size;y++)
	{
		GLubyte*	pi=image+y*size*3;
		for(int x=0;x<size;x++)
		{	
			GLubyte r=0,g=0,b=0;

			if(y<hsize ==0)
			{
				if(x<hsize ==0)
				{
					r=255;
					g=0;
					b=0;
				}
				else
				{
					r=0;
					g=84;
					b=168;
				}	
			}
			else
			{
				if(x<hsize ==0)
				{
					r=0;
					g=84;
					b=168;
				}
				else
				{
					r=255;
					g=0;
					b=0;

				}	
			}
			pi[0]=r;
			pi[1]=g;
			pi[2]=b;
			pi+=3;
		}
	}*/

	 /*
	glGenTextures(1,(GLuint*)&m_texturehandle);
	glBindTexture(GL_TEXTURE_2D,m_texturehandle);
	//glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
	gluBuild2DMipmaps(GL_TEXTURE_2D,3,size,size,GL_RGB,GL_UNSIGNED_BYTE,image);*/
						
	//delete[] image;
	//glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,m_matDiffuse);
	//glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,m_matAmbient);
}

void Renderer::display()
{
	glClear(GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);  
	glClearColor(m_clearColor[0],m_clearColor[1],m_clearColor[2],1.0f);
	
	drawSky();

	for(int k = -10; k <= 10; k+=5)  
		distanceLine(&btVector3(k,m_repaire,0.01),(int)(m_repaire *2), 0.5);	

	// show trajectory if object thrown
	renderTrajectory();

	glLightfv(GL_LIGHT0, GL_POSITION, m_lightPos);
	//glEnable(GL_LIGHTING);

	glClear(GL_STENCIL_BUFFER_BIT);
	glEnable(GL_CULL_FACE);
	renderScene();

	//computing shadows mask
	/*glDisable(GL_LIGHTING);
	glDepthMask(GL_FALSE);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_STENCIL_TEST);
	glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
	glStencilFunc(GL_ALWAYS,1,0xFFFFFFFFL);
	glFrontFace(GL_CCW);
	glStencilOp(GL_KEEP,GL_KEEP,GL_INCR);
	renderShadows();
	glFrontFace(GL_CW);
	glStencilOp(GL_KEEP,GL_KEEP,GL_DECR);
	renderShadows();
	glFrontFace(GL_CCW);

	glPolygonMode(GL_FRONT,GL_FILL);
	glPolygonMode(GL_BACK,GL_FILL);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_LIGHTING);
	glDepthMask(GL_TRUE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

	glDepthFunc(GL_LEQUAL);
	glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
	glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
	glDisable(GL_LIGHTING);

	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, 1, 1, 0, 0, 1);
	glDisable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glColor4f(0,0,0,0.5f);
	glBegin(GL_QUADS);
		glVertex2i(0, 0);
		glVertex2i(0, 1);
		glVertex2i(1, 1);
		glVertex2i(1, 0);
	glEnd();
	glDisable(GL_BLEND);

	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	
	glEnable(GL_LIGHTING);
	glDepthFunc(GL_LESS);
	glDisable(GL_STENCIL_TEST);
	glDisable(GL_CULL_FACE);	*/
	
	infoGame();	
	
}


void Renderer::renderScene()
{ 
	
	
	//glEnable(GL_COLOR_MATERIAL);

	
	for(unsigned int i=0;i<m_objects.size();i++)
	{if(m_objects[i] != NULL){
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,m_objects[i]->m_color.m_floats);
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,m_objects[i]->m_color.m_floats);

			if(m_objects[i]->m_texture) 
			{
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D,m_texturehandle);
			} else
			{
				glDisable(GL_TEXTURE_2D);
			}

			switch(m_objects[i]->m_shape->getShapeType())
			{
				case(BOX_SHAPE_PROXYTYPE):
				{
					const btBoxShape* boxShape = static_cast<const btBoxShape*>(m_objects[i]->m_shape);
					btVector3 extent = boxShape->getHalfExtentsWithMargin();
					extent*=2;

					btScalar m[16];
					m_objects[i]->m_transform->getOpenGLMatrix(m);

					glPushMatrix();
					glMultMatrixf(m);

					drawBox(extent);

					glPopMatrix();
					break;
				}
				case(CONE_SHAPE_PROXYTYPE):
				{
					const btConeShape* coneShape = static_cast<const btConeShape*>(m_objects[i]->m_shape);
					int upIndex = coneShape->getConeUpIndex();
					float radius = coneShape->getRadius();//+coneShape->getMargin();
					float height = coneShape->getHeight();//+coneShape->getMargin();
					switch (upIndex)
					{
					case 0:
						//glRotatef(90.0, 0.0, 1.0, 0.0);
						break;
					case 1:
						//glRotatef(-90.0, 1.0, 0.0, 0.0);
						break;
					case 2:
						break;
					default:
						{
						}
					};

					btScalar m[16];

					btScalar mat[16];

					btTransform myTrans = btTransform(*(m_objects[i]->m_transform));

					btVector3 trans = myTrans.getOrigin();					

					btQuaternion rot = myTrans.getRotation();						

					myTrans.setOrigin(btVector3(0,0,0));

					btTransform myNewTrans = btTransform(myTrans);	

					myNewTrans.setRotation(rot);

					myNewTrans.getOpenGLMatrix(mat);

					
					//rot.normalize();

					myTrans.getOpenGLMatrix(m);

					glPushMatrix();

						glTranslatef(trans.x(),trans.y(),0);

						//glPushMatrix();    
						    
							//glTranslatef(0.0, -0.5f*height , 0.0);

							//glMultMatrixf(mat);		

							//coneShadow(radius,height);
	
						//glPopMatrix();

						glTranslatef(0,0,trans.z());						

						glMultMatrixf(m);

						glTranslatef(0.0, 0.0, -0.5f*height);

						drawCone(radius,height);
				
					glPopMatrix();


					break;
				}
				case(CYLINDER_SHAPE_PROXYTYPE):
				{
					const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(m_objects[i]->m_shape);

					int upAxis = cylinder->getUpAxis();
					float radius = cylinder->getRadius();
					float halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];

					btScalar m[16];
					m_objects[i]->m_transform->getOpenGLMatrix(m);

					glPushMatrix();
					glMultMatrixf(m);

					drawCylinder(radius,halfHeight,upAxis);

					glPopMatrix();
					break;
				}
				case(SPHERE_SHAPE_PROXYTYPE):
				{
					const btSphereShape* sphereShape = static_cast<const btSphereShape*>(m_objects[i]->m_shape);
					btScalar radius = sphereShape->getRadius();

					btScalar m[16];
					//m_objects[i]->m_transform->getOpenGLMatrix(m);

					btTransform myTrans = btTransform(*(m_objects[i]->m_transform));

					btVector3 trans = myTrans.getOrigin();

					myTrans.setOrigin(btVector3(0,0,0));

					myTrans.getOpenGLMatrix(m);

					glPushMatrix();

					glTranslatef(trans.x(),trans.y(),0);

					sphereShadow(radius);

					glTranslatef(0,0,trans.z());

					selectedSphere(radius,m_objects[i]->m_show);

					glMultMatrixf(m);				

					drawSphere(radius);	

					glPopMatrix();

					break;
				}			
			}
		}
	}	
}

void Renderer::renderShadows()
{
	btVector3 extrusion(-m_lightPos[0],-m_lightPos[1],-m_lightPos[2]);
	glDisable(GL_LIGHTING);
	for(unsigned int i=0;i<m_objects.size();i++)
	{
		if(m_objects[i] != NULL)
		{
			btScalar m[16];
			m_objects[i]->m_transform->getOpenGLMatrix(m);

			glPushMatrix();
			glMultMatrixf(m);

			glColor3f(0.8f,0.2f,0.2f);
			drawShadow(m_objects[i]->m_shape,extrusion*m_objects[i]->m_transform->getBasis());			
			glPopMatrix();
		}
	}
	glEnable(GL_LIGHTING);
}

void Renderer::drawBox(const btVector3 &halfSize)
{
	if(m_oultines)
	{
		glPushMatrix();
		glDisable(GL_LIGHTING);
		glColor3fv(m_wireColor);
		glScalef(halfSize.x(),halfSize.y(),halfSize.z());
		glutWireCube(1);
		glPopMatrix();
	}

	glEnable(GL_LIGHTING);
	
	glBegin(GL_QUADS);
	for(int i=0;i<72;i+=3)
	{
		if( i< 12)
			glTexCoord2f(boxTextures[i]*halfSize.x()*2,boxTextures[i+1]*halfSize.y()*2);
		else if(i<24)
			glTexCoord2f(boxTextures[i]*halfSize.z()*2,boxTextures[i+1]*halfSize.y()*2);
		else if(i<36)
			glTexCoord2f(boxTextures[i]*halfSize.x()*2,boxTextures[i+1]*halfSize.z()*2);
		else if(i<48)
			glTexCoord2f(boxTextures[i]*halfSize.y()*2,boxTextures[i+1]*halfSize.z()*2);
		else if(i<60)
			glTexCoord2f(boxTextures[i]*halfSize.z()*2,boxTextures[i+1]*halfSize.x()*2);
		else if(i>=60)
			glTexCoord2f(boxTextures[i]*halfSize.x()*2,boxTextures[i+1]*halfSize.y()*2);
		else
			glTexCoord2f(boxTextures[i],boxTextures[i+1]);

		glNormal3f(boxNormals[i],boxNormals[i+1],boxNormals[i+2]);
		glVertex3f(boxVertices[i]*halfSize.x(),boxVertices[i+1]*halfSize.y(),boxVertices[i+2]*halfSize.z());
	}
	glEnd();
}

void Renderer::drawCone(const btScalar & radius , const btScalar & height)
{
	int slices = 26;
	if(m_oultines)
	{
		// Outlines
		glDisable(GL_LIGHTING);
		glCullFace (GL_FRONT);				
		glPolygonMode (GL_BACK, GL_LINE);	
		glColor3fv(m_wireColor);
		glutSolidCone(radius,height,slices,2);

		glBegin(GL_LINE_LOOP);
		for(int i=0;i<slices;i++)
		{
			float rad = i/(float)(slices) * FLT_2_PI;
			glVertex3f(cos(rad)*radius,sin(rad)*radius,0);
		}
		glEnd();
	}

	glEnable(GL_LIGHTING);
	glCullFace (GL_BACK);				// Reset The Face To Be Culled
	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);		// Reset Back-Facing Polygon Drawing Mode
	glutSolidCone(radius,height,slices,2);
	GLUquadricObj *quadObj = gluNewQuadric();
    glPushMatrix();
	glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(quadObj,0,radius,slices, 2);
	glPopMatrix();
	gluDeleteQuadric(quadObj);
}

void Renderer::coneShadow(const btScalar & radius , const btScalar & height){
	int slices = 26;
	glDisable(GL_LIGHTING);
	//circle(radius,false,true);
	//glutSolidCone(radius,height,slices,2);	

	glBegin(GL_LINE_LOOP);
			for(int k = 0; k < 90; k+=1){
				GLfloat angle = 2 * PI * k / 180;
				GLfloat x = radius*cos((double)-angle);
				GLfloat y = radius*sin((double)-angle);
				glVertex3f(0,0,height);
				glVertex2f(x,y);
				glVertex3f(0,0,0);
			}
	 glEnd();

	glEnable(GL_LIGHTING);
}

void Renderer::drawCylinder(const btScalar & radius,const btScalar & halfHeight,int upAxis)
{

	glPushMatrix();

	switch (upAxis)
	{
	case 0:
		glRotatef(-90.0, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	case 1:
		glRotatef(-90.0, 1.0, 0.0, 0.0);
		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	case 2:

		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	default:
		{
			btAssert(0);
		}

	}

	int slices = 26;
	GLUquadricObj *quadObj = gluNewQuadric();

	if(m_oultines)
	{
		// Outlines
		glDisable(GL_LIGHTING);
		glCullFace (GL_FRONT);				
		glPolygonMode (GL_BACK, GL_LINE);	


		glColor3fv(m_wireColor);
		gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, 2);


		glBegin(GL_LINE_LOOP);
		for(int i=0;i<slices;i++)
		{
			float rad = i/(float)(slices) * FLT_2_PI;
			glVertex3f(cos(rad)*radius,sin(rad)*radius,0);
		}
		glEnd();

		glBegin(GL_LINE_LOOP);
		for(int i=0;i<slices;i++)
		{
			float rad = i/(float)(slices) * FLT_2_PI;
			glVertex3f(cos(rad)*radius,sin(rad)*radius,2.f*halfHeight);
		}
		glEnd();
	}

	glEnable(GL_LIGHTING);
	glCullFace (GL_BACK);							// Reset The Face To Be Culled
	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);		// Reset Back-Facing Polygon Drawing Mode

	glPushMatrix();
	glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(quadObj,0,radius,slices, 2);
	glPopMatrix();

	gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, 2);
	glTranslatef(0.0f, 0.0f, 2.f*halfHeight);
	gluDisk(quadObj,0.f,radius,slices, 2);

	glPopMatrix();
	gluDeleteQuadric(quadObj);
}

void Renderer::drawSphere(const btScalar & radius)
{
	
	if(m_oultines)
	{
		// Outlines
		glDisable(GL_LIGHTING);
		glCullFace (GL_FRONT);				
		glPolygonMode (GL_BACK, GL_LINE);	
		glColor3fv(m_wireColor);
		
		glutSolidSphere(radius,20,13);		
	}

	glEnable(GL_LIGHTING);
	glCullFace (GL_BACK);				// Reset The Face To Be Culled
	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);		// Reset Back-Facing Polygon Drawing Mode
	glutSolidSphere(radius,20,10);
	
}

void Renderer::selectedSphere(const btScalar & radius, bool show){
	if(show)
	{		
		glDisable(GL_LIGHTING);
		//glCullFace (GL_FRONT);				
		//glPolygonMode (GL_BACK, GL_LINE);	
		glColor3fv(green);
		//x = r*cos(theta)
		//y = r*sin(theta)
		//glutWireSphere(radius+0.1,20,13);
		GLfloat r = radius + 0.1;
		glLineWidth(0.2);
		circle(r,true,false);
	    glEnable(GL_LIGHTING);
	}
}

void Renderer::sphereShadow(const btScalar & radius){
	glDisable(GL_LIGHTING);
	glLineWidth(0.1);
	glColor3fv(black);
	GLfloat r = radius - 0.1;
	circle(r,false,true);
	glEnable(GL_LIGHTING);
}

void Renderer::circle(GLfloat radius,bool axeZ,bool Fill){
	if(axeZ){
	    glBegin(GL_LINE_LOOP);
			for(int k = 0; k < 360; k+=1){
				GLfloat x = radius*cos((double)k);
				GLfloat z = radius*sin((double)k);
				glVertex3f(x,0,z);
				if(Fill)
					glVertex3f(0,0,0);
			}
		 glEnd();
	}else{
		 glBegin(GL_LINE_LOOP);
			for(int k = 0; k < 360; k+=1){
				GLfloat x = radius*cos((double)k);
				GLfloat y = radius*sin((double)k);
				glVertex2f(x,y);
				if(Fill)
					glVertex3f(0,0,0);
			}
		glEnd();
	}
}

void Renderer::distanceLine(btVector3* start, int stop,GLfloat size){

	if(stop < 1)
		return;
	GLfloat x = start->x();	
	GLfloat y = start->y();	
	GLfloat z = start->z();	

	glDisable(GL_LIGHTING);

	glColor3fv(light_Grey);

	for(int i = 0; i<stop; i++){

		if(i % 2 == 0){
			glBegin(GL_QUADS);			    
				glVertex3f(x,y,z);
				glVertex3f(x+size,y,z);
				glVertex3f(x+size,y+size,z);
				glVertex3f(x,y+size,z);
			glEnd();
		}

		y--;
	}

	glEnable(GL_LIGHTING);
}

void Renderer::setRepaire(int r){
	m_repaire = r;
}

void Renderer::drawShadow(const btCollisionShape* shape,const btVector3 &extrusion)
{
	if (shape->isConvex())
	{
		ShapeCache*	sc=cache((btConvexShape*)shape);
		btShapeHull* hull =&sc->m_shapehull;
		glBegin(GL_QUADS);
		for(int i=0;i<sc->m_edges.size();++i)
		{			
			const btScalar		d=btDot(sc->m_edges[i].n[0],extrusion);
			if((d*btDot(sc->m_edges[i].n[1],extrusion))<0)
			{
				const int			q=	d<0?1:0;
				const btVector3&	a=	hull->getVertexPointer()[sc->m_edges[i].v[q]];
				const btVector3&	b=	hull->getVertexPointer()[sc->m_edges[i].v[1-q]];
				glVertex3f(a[0],a[1],a[2]);
				glVertex3f(b[0],b[1],b[2]);
				glVertex3f(b[0]+extrusion[0],b[1]+extrusion[1],b[2]+extrusion[2]);
				glVertex3f(a[0]+extrusion[0],a[1]+extrusion[1],a[2]+extrusion[2]);
			}
		}
		glEnd();
	}
}

Renderer::ShapeCache*	Renderer::cache(btConvexShape* shape)
{
	ShapeCache*		sc=(ShapeCache*)shape->getUserPointer();
	if(!sc)
	{
		sc=new(btAlignedAlloc(sizeof(ShapeCache),16)) ShapeCache(shape);
		sc->m_shapehull.buildHull(shape->getMargin());
		m_shapecaches.push_back(sc);
		shape->setUserPointer(sc);
		/* Build edges	*/ 
		const int			ni=sc->m_shapehull.numIndices();
		const int			nv=sc->m_shapehull.numVertices();
		const unsigned int*	pi=sc->m_shapehull.getIndexPointer();
		const btVector3*	pv=sc->m_shapehull.getVertexPointer();
		btAlignedObjectArray<ShapeCache::Edge*>	edges;
		sc->m_edges.reserve(ni);
		edges.resize(nv*nv,0);
		for(int i=0;i<ni;i+=3)
		{
			const unsigned int* ti=pi+i;
			const btVector3		nrm=btCross(pv[ti[1]]-pv[ti[0]],pv[ti[2]]-pv[ti[0]]).normalized();
			for(int j=2,k=0;k<3;j=k++)
			{
				const unsigned int	a=ti[j];
				const unsigned int	b=ti[k];
				ShapeCache::Edge*&	e=edges[btMin(a,b)*nv+btMax(a,b)];
				if(!e)
				{
					sc->m_edges.push_back(ShapeCache::Edge());
					e=&sc->m_edges[sc->m_edges.size()-1];
					e->n[0]=nrm;e->n[1]=-nrm;
					e->v[0]=a;e->v[1]=b;
				}
				else
				{
					e->n[1]=nrm;
				}
			}
		}
	}
	return(sc);
}

Object * Renderer::getObject(btCollisionShape * shape)
{
	for(unsigned int i=0;i<m_objects.size();i++)
	{
		if(m_objects[i]->m_shape == shape)
			return m_objects[i];
	}
	return NULL;
}

void Renderer::drawSky()
{
	glColor3f(m_clearColor[0],m_clearColor[1],m_clearColor[2]);
	glDisable(GL_LIGHTING);
	glutSolidCube(1000);
	glEnable(GL_LIGHTING);
}

void Renderer::renderTrajectory(){
	glDisable(GL_LIGHTING);
	glColor3f(0.2,0.2,0.2);

	glLineWidth(0.8);
	for (int j = 0; j < ThronNumber;j++)			
		if(m_points[j].size()>0){			
			for(unsigned int i =0 ;i< (m_points[j].size());i++){				
				drawPoint((m_points[j])[i]->getX(), 
						  (m_points[j])[i]->getY(),
						  (m_points[j])[i]->getZ());					
			}			
	}
	
}

void Renderer::drawPoint(GLfloat x, GLfloat y, GLfloat z){	
	GLfloat c = 0.04;
	//glPointSize(0.001);

	glBegin(GL_LINES);	
	glVertex3f(x-c,y,z);
	glVertex3f(x+c,y,z);
	glEnd();

	glBegin(GL_LINES);	
	glVertex3f(x,y,z-c);
	glVertex3f(x,y,z+c);
	glEnd();

	/*glBegin(GL_LINES);	
	glVertex3f(x,y-c,z);
	glVertex3f(x,y+c,z);
	glEnd();*/
}

void Renderer::setPoints(std::vector<btVector3*>* points, int index){
	for(unsigned int i = 0 ; i < points->size(); i++)
		m_points[index].push_back((*points)[i]);
}

void Renderer::clearPoints(){
	for(int i =0 ; i < ThronNumber;i++)
		m_points[i].clear();
}

void Renderer::WriteStatus(std::string* text){	
	glEnter2D();
	glWrite(10, 20, GLUT_BITMAP_HELVETICA_12, text );
	glLeave2D();
}

void Renderer::glEnter2D(){
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   gluOrtho2D(0, glGetViewportWidth(), 0, glGetViewportHeight());
 
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
 
   glDisable(GL_DEPTH_TEST);
}
 
void Renderer::glLeave2D(){
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   glMatrixMode(GL_MODELVIEW);
   glPopMatrix();
 
   glEnable(GL_DEPTH_TEST);
}

GLint Renderer::glGetViewportWidth()
{
   GLint w[4];
   glGetIntegerv(GL_VIEWPORT, w);
   return(w[2] - w[0]);
}
 
GLint Renderer::glGetViewportHeight()
 {
   GLint h[4];
   glGetIntegerv(GL_VIEWPORT, h);
   return(h[3] - h[1]);
 }

void Renderer::glWrite(GLfloat x,GLfloat y, void * font,std::string* text){ 
		
	glRasterPos2f(x, y);
	for (int i= 0; i< text->size(); i ++)
		glutBitmapCharacter(font, int((*text)[i]));
}

void Renderer::printText(){	
	if(m_text != NULL)
		WriteStatus(m_text);
}

void Renderer::setText(std::string* text){
	delete m_text;
	 m_text = text;
}

void Renderer::infoGame(){

	glDisable(GL_LIGHTING);	

	glColor3fv(black);

	printText(); 

	glEnter2D();	

	glWrite(10, glGetViewportHeight() - 20, GLUT_BITMAP_HELVETICA_12, &(std::string(" Points par couleur : ")));	

	glColor3fv(red);

	glWrite(15, glGetViewportHeight() - 45, GLUT_BITMAP_HELVETICA_12, &(std::string(" Rouge    ==>  20  ")));

	glColor3fv(light_blue);

	glWrite(15, glGetViewportHeight() - 60, GLUT_BITMAP_HELVETICA_12, &(std::string(" Bleu      ==>  10  ")));

	glColor3fv(yellow);
	 
	glWrite(15, glGetViewportHeight() - 75, GLUT_BITMAP_HELVETICA_12, &(std::string(" Jaune    ==> -10  ")));

	glLeave2D();

	glEnable(GL_LIGHTING);				
}