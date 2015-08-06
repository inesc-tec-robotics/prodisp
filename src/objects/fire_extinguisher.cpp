#include "objects/fire_extinguisher.h"

#include "LoggerSys.hpp"

using namespace std;

FireExtinguisher::FireExtinguisher(const string& name, XmlRpc::XmlRpcValue data)
	: Plane(0.5/(581.0/218.0), 0.5, 2)
{
	FDEBUG("FireExtinguisher::FireExtinguisher: " << name);

	assert(data.size() == 3 && "ParamServer error: Each fire_extinguisher must contain 3 elements");

	XmlRpc::XmlRpcValue::iterator next_it = data.begin();
	double time	= (next_it++)->second;
	Vector3 pos;
	pos.c[0]		= (double)(next_it++)->second;
	pos.c[1]		= (double)(next_it)->second;
	pos.c[2]		= 0;

	FDEBUG("time=" << time);
	init(pos);
}

FireExtinguisher::FireExtinguisher(Vector3 pos)
	: Plane(0.5/(581.0/218.0), 0.5, 2)
{
	FDEBUG("FireExtinguisher::FireExtinguisher (new)");
	init(pos);
}

void FireExtinguisher::init(Vector3 pos)
{
	FDEBUG("FireExtinguisher::init at " << pos.c[0] << ", " << pos.c[1] << ", " << pos.c[2] << "]");

	transform_.setTranslation(pos);
	transform_.setRotation(180,0,0);

	// Graphics:
	setTextureAutoAspect("fire_extinguisher.bmp");
	createPrimitive();

	makeSelectable();
}















//void Stud::setTf(sg::Tf pose)
//{
//    FINFO("Stud::setTf");
//    pose_ = pose;
////    stud_data_[2] = XmlRpc::XmlRpcValue((float)pose.translation_.x());
////    stud_data_[3] = XmlRpc::XmlRpcValue((float)pose.translation_.y());
//}
