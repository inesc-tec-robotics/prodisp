#include "objects/stud.h"

#include "LoggerSys.hpp"

using namespace std;

Stud::Stud(const string& name, XmlRpc::XmlRpcValue stud_data)
{
	FDEBUG("Stud::Stud: " << name);

	assert(stud_data.size() == 4 && "Stud_data error: Each stud must contain 4 elements");

	XmlRpc::XmlRpcValue::iterator next_it = stud_data.begin();
	state_			= (stud::states)(int)( (next_it++)->second );
	double time	= (next_it++)->second;
	Vector3 pos;
	pos.c[0]		= (double)(next_it++)->second;
	pos.c[1]		= (double)(next_it)->second;
	pos.c[2]		= 0;

	FDEBUG("state=" << state_ << ", time=" << time);
	init(pos);
}

Stud::Stud(Vector3 pos)
{
	FDEBUG("Stud::Stud: (new)");
	init(pos);
}

void Stud::init(Vector3 pos)
{
	FDEBUG("Stud::init at " << pos.c[0] << ", " << pos.c[1] << ", " << pos.c[2] << "]");

	gl_id_ = 0;
	transform_.setTranslation(pos);

	// Graphics:
	setColour(Colour::RED);
	createPrimitive();
	scaleVertices(0.015);

	makeSelectable();
}















//void Stud::setTf(sg::Tf pose)
//{
//    FINFO("Stud::setTf");
//    pose_ = pose;
////    stud_data_[2] = XmlRpc::XmlRpcValue((float)pose.translation_.x());
////    stud_data_[3] = XmlRpc::XmlRpcValue((float)pose.translation_.y());
//}
