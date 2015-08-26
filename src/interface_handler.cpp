#include "objects/tf.h"
#include "objects/point_cloud.h"
#include "objects/fire_extinguisher.h"
#include "interface_handler.h"
#include "configuration.h"
#include "wii_handler.h"
#include "renderer.h"

#include "LoggerSys.hpp"
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_ctrl_msgs/projectionPoseAction.h"
#include "mission_ctrl_msgs/moveArmAction.h"

using namespace std;

// Singleton instance
template<> InterfaceHandler* Singleton<InterfaceHandler>::msInstance = 0;

InterfaceHandler::InterfaceHandler()
	 : as_wii_teaching_(nh_, CARLOS_TEACHING_ACTION, false)
	 , as_move_arm_(CARLOS_MOVE_ARM_ACTION, true)
	 , move_arm_active_(false)
	 , ros_tf_listener_(ros_tf_buffer)
	, tf_alvar_(NULL)
{
	FDEBUG("InterfaceHandler::InterfaceHandler");

	// Register the goal callback and start:
	as_wii_teaching_.registerGoalCallback(boost::bind(&InterfaceHandler::cbWiiTeachingGoal, this));
	as_wii_teaching_.start();

	// Read and save pose (refreshed for each teaching goal):
	projection_pose_relative_confirmed_ = cv::Vec2d(0,0);
	lookupTf("base_footprint", "projector_link", projection_pose_start_);

	// Start publisher to robot arm:
//	pub_move_robot = nh_.advertise<geometry_msgs::PoseArray>(CARLOS_JOINT_MOVE_MSG, 10);

	// Interface to Wii remote:
	wii_handler_.reset( new WiiHandler );
}

InterfaceHandler::~InterfaceHandler()
{
	FDEBUG("InterfaceHandler::~InterfaceHandler");

	if (as_wii_teaching_.isActive())
	{
		cerr << "Cancelling active action goal due to shutdown" << endl;
		// Reply action server:
		mission_ctrl_msgs::performTeachingResult result;
		result.description = "Teaching failed due to shutdown";
		result.succeeded = false;
		as_wii_teaching_.setAborted(result);
	}

	// Shut down ROS stuff:
	ros::spinOnce();
	as_wii_teaching_.shutdown();
//	sub_alvar_.shutdown();
}

void InterfaceHandler::bindWii(boost::weak_ptr<Cursor> cursor)
{
	FDEBUG("InterfaceHandler::bindWii");
	wii_handler_->bindCursor(cursor);
}

void InterfaceHandler::addRosTf(std::string src_frame, std::string trg_frame, sg::Tf* tf)
{
	FINFO("InterfaceHandler::addRosTf from '" << src_frame << "' to '" << trg_frame << "'");
	RosTfSub tf_sub(src_frame, trg_frame, tf);

	// Call with timeout first time:
	MathOp::Transform mtf;
	tf_sub.connected_ = lookupTf(tf_sub.src_frame_, tf_sub.trg_frame_, mtf, ros::Duration(2.0));

	if (tf_sub.connected_)
	{
		tf_sub.tf_->set(mtf);
	}
	else
	{
		FERROR("TF from '" << src_frame << "' to '" << trg_frame << "' not connected on startup.");
	}

	ros_tf_subs_.push_back(tf_sub);
}

void InterfaceHandler::refreshRosTf()
{
	for (vector<RosTfSub>::iterator tf_sub = ros_tf_subs_.begin(); tf_sub != ros_tf_subs_.end(); ++tf_sub)
	{
		lookupTf(*tf_sub);
	}
}

//void InterfaceHandler::cbAlvar(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
//{
//	FDEBUG("InterfaceHandler::cbAlvar");
//	for (int i = 0; i < (int)msg->markers.size(); i++)
//	{
//		ar_track_alvar_msgs::AlvarMarker m = msg->markers[i];
//		int id = m.id;
//		geometry_msgs::Pose pos = m.pose.pose;

//		if (id == 0 && tf_alvar_ != NULL)
//		{
//			MathOp::Transform transform(pos);
//			tf_alvar_->set(transform);
//		}
//	}
//}

void InterfaceHandler::cbWiiTeachingGoal(void)
{
	FDEBUG("InterfaceHandler::cbWiiTeachingGoal");

	// Store task goal:
	wii_teaching_task_ = "/mission/tasks/" + as_wii_teaching_.acceptNewGoal()->task_name;

	// Load studs from param server:
	int direction;
	vector<Object*> studs = loadTask(wii_teaching_task_, direction);

	// Move to projection pose:
	actionlib::SimpleActionClient<mission_ctrl_msgs::projectionPoseAction>
			ac(CARLOS_PROJECTION_ACTION, true);
	if ( !ac.waitForServer(ros::Duration(3.0)) )
	{
		FERROR("Failed to call action on '" << CARLOS_PROJECTION_ACTION
				 << "'. Cannot move to projection pose.");
	}
	else
	{
		FDEBUG("Action server on '" << CARLOS_PROJECTION_ACTION << "' started, sending goal.");
		mission_ctrl_msgs::projectionPoseGoal goal;
		goal.side = direction;
		ac.sendGoal(goal);
		if ( !ac.waitForResult(ros::Duration(20.0)) )
		{
			FERROR("Action goal on '" << CARLOS_PROJECTION_ACTION
						<< "' did not finish before the time out.");
			ac.cancelAllGoals();
		}
		else
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			FDEBUG("Action goal on  '" << CARLOS_PROJECTION_ACTION
						<< "' finished with state: " << state.toString());
		}
	}

	// Read and save pose:
	projection_pose_relative_confirmed_ = cv::Vec2d(0,0);
	lookupTf("base_footprint", "projector_link", projection_pose_start_);
	cout << "New teaching goal, projection_pose_start_ t = " << projection_pose_start_.getTranslation()
		  << ", r = " << projection_pose_start_.getQuaternions() << endl;

	// Start projection:
	Renderer::getInstance().start(studs);
}

void InterfaceHandler::moveProjectorIncr(float x, float y)
{
	FDEBUG("InterfaceHandler::moveProjectorIncr");

	// Ignore if not active
	if (!Renderer::getInstance().isActive())
		return;

	// Chack if arm is already moving:
	if (move_arm_active_)
		return;

	// Create projector pose:
	projection_pose_relative_requested_[0] = projection_pose_relative_confirmed_[0] + x;
	projection_pose_relative_requested_[1] = projection_pose_relative_confirmed_[1] + y;
	MathOp::Transform pose_relative(
				0, -projection_pose_relative_requested_[0], -projection_pose_relative_requested_[1], 0, 0, 0, MathOp::ZYX_EULER);
	MathOp::Transform pose_new = projection_pose_start_ * pose_relative;

	// Check that action server is connected:
	if (!as_move_arm_.isServerConnected())
	{
		FWARN("Server on " << CARLOS_MOVE_ARM_ACTION << " not connected.");
		return;
	}

	// Create message:
	mission_ctrl_msgs::moveArmGoal goal;
	goal.pose.header.frame_id = "base_footprint";
	goal.pose.pose = pose_new.getGeometryMsgsPose();

	// Send goal and bind callback:
	move_arm_active_ = true;
	as_move_arm_.sendGoal(goal, boost::bind(&InterfaceHandler::cbMoveArmDone, this, _1, _2));
}

void InterfaceHandler::cbMoveArmDone(const actionlib::SimpleClientGoalState &state,
												 const mission_ctrl_msgs::moveArmResultConstPtr &result)
{
	FDEBUG("InterfaceHandler::cbMoveArmDone");
	if (state.state_ == actionlib::SimpleClientGoalState::ABORTED ||
		 state.state_ == actionlib::SimpleClientGoalState::LOST)
	{
		FERROR("MoveArm error: " << state.state_);
	}
	else
	{
		projection_pose_relative_confirmed_ = projection_pose_relative_requested_;
	}
	move_arm_active_ = false;
}

void InterfaceHandler::wiiTeachingComplete(const vector<Stud*>& studs)
{
	FDEBUG("InterfaceHandler::wiiTeachingComplete");

	// Make sure that the action is active:
	if (!as_wii_teaching_.isActive())
	{
		FERROR("Attempting to complete wii teaching action, "
				 << "but action sever is not running! Should not happen.");
		return;
	}

	// Delete studs from param server:
	ros::param::del(wii_teaching_task_ + "/studs/");

	// Create studs on param server:
	double time_stamp = ros::Time::now().toSec();
	for (int stud_num = 0; stud_num < (int)studs.size(); stud_num++)
	{
		// Create stud name on param server:
		stringstream stud_name_ss;
		stud_name_ss << wii_teaching_task_ << "/studs/stud"
						  << setfill('0') << setw(3) << stud_num+1
						  << "/";
		string stud_name = stud_name_ss.str();

		// Get stud:
		Stud* stud = studs[stud_num];

		Vector3 t = stud->getTf().translation_;
		ros::param::set(stud_name + "x", t.c[0]);
		ros::param::set(stud_name + "y", t.c[1]);
		ros::param::set(stud_name + "state", (int)stud->getState());
		ros::param::set(stud_name + "time_stamp", time_stamp);
	}

	// Move robot arm to "home" position:
	std_srvs::Empty home_srv;
	ros::ServiceClient home_srv_client = nh_.serviceClient<std_srvs::Empty>(CARLOS_ARM_HOME_SRV);
	if (!home_srv_client.waitForExistence(ros::Duration(1.0)))
	{
		FERROR("Failed to move arm to home position; service '" << CARLOS_ARM_HOME_SRV << "' does not exist.");
	}
	else
	{
		if (!home_srv_client.call(home_srv))
		{
			FERROR("Failed to move arm to home position; service '" << CARLOS_ARM_HOME_SRV << "' failed.");
		}
	}

	// Reply action server:
	mission_ctrl_msgs::performTeachingResult result;
	result.description = "Teaching completed";
	result.succeeded = true;
	as_wii_teaching_.setSucceeded(result);
	FINFO("Teaching succeded; " << studs.size() << " stud positions returned.");
}

vector<Object*> InterfaceHandler::loadTask(string task, int &direction)
{
	FDEBUG("InterfaceHandler::loadStuds");
	FINFO("Loading task: " << task);

	// Studs:
	vector<Object*> objects;
	string key_studs = task + "/studs/";
	XmlRpc::XmlRpcValue xml_studs;
	ros::param::get(key_studs, xml_studs);

	for (XmlRpc::XmlRpcValue::iterator stud_it = xml_studs.begin(); stud_it != xml_studs.end(); ++stud_it)
	{
		string name = stud_it->first;
		objects.push_back(new Stud(name, stud_it->second));
	}
	FDEBUG("Loaded " << objects.size() << " studs.");

	// Obstacles:
	string key_obstacles = task + "/obstacles/";
	XmlRpc::XmlRpcValue xml_obstacles;
	ros::param::get(key_obstacles, xml_obstacles);

	for (XmlRpc::XmlRpcValue::iterator obs_it = xml_obstacles.begin();
		  obs_it != xml_obstacles.end(); ++obs_it)
	{
		string name = obs_it->first;
		if (name.find("fire_extinguisher") != string::npos)
			objects.push_back(new FireExtinguisher(name, obs_it->second));
	}

	// Load direction:
	string key_direction = task + "/direction";
	ros::param::get(key_direction, direction);

	FINFO("Loaded " << objects.size() << " objects at direction " << direction);

	return objects;
}

bool InterfaceHandler::lookupTf(std::string trg_frame, std::string src_frame,
										  MathOp::Transform& tf, ros::Duration wait_duration)
{
	try
	{
		geometry_msgs::TransformStamped tf_stamped =
				ros_tf_buffer.lookupTransform(trg_frame, src_frame,
														ros::Time(0), wait_duration);
		cout << " QUAT: " << tf_stamped.transform.rotation << endl;
		tf = MathOp::Transform(tf_stamped.transform);
	}
	catch (tf2::TransformException &ex)
	{
		FERROR("TF error from '" << trg_frame << "'' to '" << src_frame << "': " << ex.what());
		return false;
	}
	return true;
}

bool InterfaceHandler::lookupTf(RosTfSub& tf_sub, ros::Duration wait_duration)
{
	try
	{
		geometry_msgs::TransformStamped tf_stamped =
				ros_tf_buffer.lookupTransform(tf_sub.src_frame_, tf_sub.trg_frame_,
														ros::Time(0), wait_duration);
		tf_sub.tf_->set(MathOp::Transform(tf_stamped.transform));
		if (tf_sub.connected_ == false)
		{
			tf_sub.connected_ = true;
			FINFO("TF now connected from '" << tf_sub.src_frame_ << "'' to '" << tf_sub.trg_frame_ << "'");
		}
	}
	catch (tf2::TransformException &ex)
	{
		if (tf_sub.connected_ == true)
		{
			FERROR("TF error (disconnected) from '" << tf_sub.src_frame_ << "'' to '" << tf_sub.trg_frame_ << "': " << ex.what());
			tf_sub.connected_ = false;
		}
		return false;
	}
	return true;
}






