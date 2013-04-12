#pragma once

//#include <GL/freeglut.h>
#include <GL/glut.h>
#include <vector>

#include "Object.h"

#include "BulletCollision/CollisionShapes/btShapeHull.h"

#define FLT_2_PI 6.283185307f
const GLfloat black[3] = {0.1,0.1,0.1};

class Renderer
{
	struct ShapeCache
	{
		struct Edge { btVector3 n[2];int v[2]; };
		ShapeCache(btConvexShape* s) : m_shapehull(s) {}
		btShapeHull					m_shapehull;
		btAlignedObjectArray<Edge>	m_edges;
	};

public:
	Renderer(void);
	~Renderer(void);
	void renderScene();
	void renderShadows();
	void display();
	void init();

	void drawSky();
	void renderTrajectory();
	void drawBox(const btVector3 &halfSize);
	void drawCone(const btScalar & radius , const btScalar & height);
	void drawSphere(const btScalar & radius);
	void drawCylinder(const btScalar & radius , const btScalar & height,int upAxis);
	void drawShadow(const btCollisionShape* shape,const btVector3 & extrusion);
	
	void setClearColor(float r,float g,float b){m_clearColor[0]=r;m_clearColor[1]=g;m_clearColor[2]=b;}
	Object* addObject(Object * object);
	void delObject(Object* o);
	void replaceObject(Object* oldObject, Object* newObject);
	Object * getObject(btCollisionShape * shape);

	void setText(std::string* text);
	void WriteStatus(std::string* text);
	void setPoints(std::vector<btVector3*>* points, int index);
	void clearPoints();
private:

	//GL_ShapeDrawer m_shapedrawer; 
	Renderer::ShapeCache* cache(btConvexShape* shape);
	btAlignedObjectArray<ShapeCache*>	m_shapecaches;

	void glWrite(GLfloat x,GLfloat y, void * font,std::string* text);
	void drawPoint(GLfloat x, GLfloat y, GLfloat z);
	void glEnter2D();
	void glLeave2D();
	void printText();
	GLint glGetViewportWidth();
	GLint glGetViewportHeight();
	unsigned int m_texturehandle;
	std::vector<Object *> m_objects;
	std::vector <btVector3*> m_points[ThronNumber];
	GLfloat m_lightPos[3];
	GLfloat m_wireColor[3];
	GLfloat m_clearColor[3];
	GLfloat m_matDiffuse[4];
	GLfloat m_matAmbient[4];
	bool m_oultines;
	std::string* m_text;

};
