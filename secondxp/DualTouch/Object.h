#pragma once

#include "BulletCollision\CollisionShapes\btCollisionShape.h"
#include <iostream>

const int ThronNumber = 4;
const btVector3 green(0.0f,1.0f,0.0f);
const btVector3 dark_red(0.4f,0.15f,0.15f);
const btVector3 orange(0.8f,0.6f,0.2f);
const btVector3 yellow(0.9f,0.89f,0.0f);
const btVector3 blue(0.0f,0.33f,0.66f);
const btVector3 light_blue(0.0f,0.0f,0.8f);
const btVector3 light_Grey(0.60f,0.60f,0.60f);
const btVector3 dark_Grey(0.2f,0.2f,0.2f);
const btVector3 gray(0.6f,0.6f,0.6f);
const btVector3 neutral(0.9f,0.9f,0.9f);
const btVector3 red(1.0f,0.0f,0.0f);
const btVector3 light_red(0.8f,0.0f,0.0f);

class Object
{
public:
	Object(btCollisionShape * shape,btTransform * transform,const btVector3 &color);
	~Object(void);
	btCollisionShape * m_shape;
	btTransform  * m_transform;
	btVector3 m_color;
	bool m_texture;
	void print();
	void setShow(bool show);
	btVector3 getpos();
	btCollisionShape * getshape();
	void setScore(int score);
	int getScore();
	void setColor(const btVector3 &color);	
	void setTransform(const btTransform &newtrans);
	btTransform* getTransform();
	bool m_show;
	int m_score;
};
