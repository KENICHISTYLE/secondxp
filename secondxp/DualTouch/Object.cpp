#include "Object.h"

Object::Object(btCollisionShape * shape,btTransform * transform,const btVector3 &color)
{
	m_shape = shape;
	m_transform = transform;
	m_color = color;
	m_texture = false;
	m_score = 0;
	m_show = false;
}

Object::~Object(void)
{
	//delete m_shape; //physic class made it
	if(m_transform != NULL)
		delete m_transform;
}

void Object::setColor(const btVector3 &color){
	m_color = color;
}

void Object::setShow(bool show){
	m_show = show;
}

void Object::setScore(int score){
	m_score = score;
}

int Object::getScore(){
	return m_score;
}

void Object::print(){
	std::cout<<" x " << m_transform->getOrigin().getX()
		<<" y " << m_transform->getOrigin().getY()
		<<" z " << m_transform->getOrigin().getZ()
		<<std::endl;
}

btVector3 Object::getpos(){
	return m_transform->getOrigin();
}

btCollisionShape* Object::getshape(){
	return m_shape;
}

void Object::setTransform(const btTransform &newtrans){
	*m_transform = newtrans;
}
btTransform* Object::getTransform(){
	return m_transform;
}