#ifndef WII_HANDLER_H
#define WII_HANDLER_H

#include "common.h"
#include "cursor.h"

#include "ros/node_handle.h"
#include <sensor_msgs/Joy.h>



class WiiHandler
{
public:
	WiiHandler();
	~WiiHandler();

	void		bindCursor(boost::weak_ptr<Cursor> cursor);

private:
	void		joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

private:
	enum WiiButtons
	{
		Wii_1		= 0,
		Wii_2		= 1,
		Wii_A		= 2,
		Wii_BACK = 3,
		Wii_PLUS	= 4,
		Wii_MINUS= 5,
		Wii_LEFT	= 6,
		Wii_RIGHT= 7,
		Wii_UP	= 8,
		Wii_DOWN	= 9,
		Wii_HOME	= 10,
	};

	// Velocity offset for cursor:
	double					offset_x_;
	double					offset_y_;

	// Wii state:
	boost::weak_ptr<Cursor>	cursor_;
	std::vector<int>		buttons_down_last_;
//	bool						calibrating_;
	int						calibration_samples_;
	double					calib_x_sum_;
	double					calib_y_sum_;

	// ROS interface:
	ros::NodeHandle		nh_;
	ros::Subscriber		joy_sub_;
};


#endif // WII_HANDLER_H
