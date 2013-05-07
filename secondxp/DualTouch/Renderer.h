#pragma once

//#include <GL/freeglut.h>
#include <GL/glut.h>
#include <vector>

#include "Object.h"

#include "BulletCollision/CollisionShapes/btShapeHull.h"

#define FLT_2_PI 6.283185307f
#define PI 3.14
const GLfloat black[3] = {0.1,0.1,0.1};
const int waitShow = 5;

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
	void setPoints(std::vector<btVector3*>* points, int index, btVector3* color);
	void infoGame();
	void clearPoints();
	void selectedSphere(const btScalar & radius,bool show);

	void circle(GLfloat radius,bool axeZ,bool Fill);
	void sphereShadow(const btScalar & radius);
	void coneShadow(const btScalar & radius , const btScalar & height);

	void distanceLine(btVector3* start, int stop, GLfloat);
	void setRepaire(int r);
	void StartMessage();
	void showStart(bool info,int time);
	void showFeedBack(bool with);
	void printFeed();
	void setDir(btVector3* depart, btVector3* arrivee);
	void setLeftToThrow(const char * time);
	void showLeftToThrow();
	void stopShowTime();
	void renderDir();
	void printFin();
	void setFin();
	void setWait(int time);
	void showWait();
	void stopWait();

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
	btVector3 m_traj_color[ThronNumber];
	GLfloat m_lightPos[3];
	GLfloat m_wireColor[3];
	GLfloat m_clearColor[3];
	GLfloat m_matDiffuse[4];
	GLfloat m_matAmbient[4];
	bool m_oultines;
	bool m_InformeStart;
	int m_leftToStart;
	bool m_showFeed;
	bool m_feedback;
	time_t m_timer;
	std::string* m_text;
	bool m_dirset;
	bool m_beforeThrow;
	bool m_finJeu;
	bool m_waitMode;
	int m_waitTime;
	std::string m_throwText;
	btVector3 m_cursor;
	btVector3 m_fleche;
	GLfloat m_repaire;
};
