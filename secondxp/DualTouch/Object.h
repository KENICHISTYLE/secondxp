#pragma once

#include "BulletCollision\CollisionShapes\btCollisionShape.h"
#include <iostream>
#include "Consts.h"


class Object
{
public:
	Object(btCollisionShape * shape,btTransform * transform,const btVector3 &color);
	Object();
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
