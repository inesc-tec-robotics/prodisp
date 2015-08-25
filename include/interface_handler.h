#ifndef InterfaceHandler_H__
#define InterfaceHandler_H__

#include "prerequisites.h"
#include "objects/stud.h"
#include "objects/tf.h"

#include "MathOperations.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
//#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "mission_ctrl_msgs/performTeachingAction.h"
#include "mission_ctrl_msgs/moveArmAction.h"

#include <opencv2/highgui/highgui.hpp>
#include <boost/scoped_ptr.hpp>

// Forward declarations:
namespace sg { class Tf; }
class PointCloud;
class WiiHandler;
class Cursor;


class InterfaceHandler
{
public:
	InterfaceHandler();
	~InterfaceHandler();

//	void bindAlvar(sg::Tf* transform)	{ tf_alvar_ = transform; }
	void bindWii(boost::weak_ptr<Cursor> cursor);

	void addRosTf(std::string src_frame, std::string trg_frame, sg::Tf* tf);
	void refreshRosTf();

	void moveProjectorIncr(float x, float y);
	void wiiTeachingComplete(const std::vector<Stud *> &studs);

/****** Structs ******/
private:
	struct RosTfSub
	{
		RosTfSub(std::string src_frame, std::string trg_frame, sg::Tf* tf)
			: connected_(false)
			, src_frame_(src_frame)
			, trg_frame_(trg_frame)
			, tf_(tf)
		{}

		bool connected_;
		std::string src_frame_;
		std::string trg_frame_;
		sg::Tf* tf_;
	};


/****** Interfaces ******/
private:
//	void cbAlvar(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg);

	ros::NodeHandle					nh_;

	ros::Subscriber					sub_alvar_;
	sg::Tf*								tf_alvar_;

	bool									lookupTf(std::string trg_frame, std::string src_frame,
														MathOp::Transform& tf,
														ros::Duration = ros::Duration(0.0));
	bool									lookupTf(RosTfSub& tf_sub,
														ros::Duration wait_duration = ros::Duration(0.0));
	tf2_ros::Buffer					ros_tf_buffer;
	tf2_ros::TransformListener		ros_tf_listener_;
	std::vector<RosTfSub>			ros_tf_subs_;

	actionlib::SimpleActionServer<mission_ctrl_msgs::performTeachingAction> as_wii_teaching_;
	void									cbWiiTeachingGoal(void);
	std::string							wii_teaching_task_;
	std::vector<Object*>				loadTask(std::string task, int& direction);

	MathOp::Transform					projection_pose_start_;
	cv::Vec2d							projection_pose_relative_confirmed_;
	cv::Vec2d							projection_pose_relative_requested_;
	actionlib::SimpleActionClient<mission_ctrl_msgs::moveArmAction> as_move_arm_;
	void									cbMoveArmDone(const actionlib::SimpleClientGoalState &state,
															  const mission_ctrl_msgs::moveArmResultConstPtr &result);
	bool									move_arm_active_;


//	ros::Publisher						pub_move_robot;
//	ros::Subscriber					sub_robot_pos;

	boost::scoped_ptr<WiiHandler> wii_handler_;
};

#endif // InterfaceHandler_H__



