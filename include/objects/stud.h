#ifndef STUD_H__
#define STUD_H__

#include "objects/line_cross.h"

#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include <ros/xmlrpc_manager.h>

class Stud : public LineCross
{
public:
	 Stud(const std::string &name, XmlRpc::XmlRpcValue stud_data);
	 Stud(Vector3 pos);

	 stud::states	getState(void) const	{ return state_; }

private:
	 void init(Vector3 pos);

	 stud::states	state_;
	 GLint			gl_id_;
};


#endif // STUD_H__



