#ifndef FIRE_EXTINGUISHER_H__
#define FIRE_EXTINGUISHER_H__

#include "objects/plane.h"

//#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include <ros/xmlrpc_manager.h>

class FireExtinguisher : public Plane
{
public:
	 FireExtinguisher(const std::string &name, XmlRpc::XmlRpcValue data);
	 FireExtinguisher(Vector3 pos);

private:
	 void init(Vector3 pos);

	 GLint			gl_id_;
};


#endif // FIRE_EXTINGUISHER_H__



