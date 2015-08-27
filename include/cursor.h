#ifndef CURSOR_H
#define CURSOR_H

#include "common.h"

#include <boost/function.hpp>
#include <boost/weak_ptr.hpp>
#include <ros/timer.h>
#include <ros/node_handle.h>

// Forward declarations:
class Renderer;

namespace CursorStates
{
	enum State
	{
		Idle,
		Holding,
		Marking,
		Calibrating
	};
}

namespace CursorEvents
{
	enum Event
	{
		Select,
		Add,
		Delete,
		Calibrate,
		LockX,
		LockY,
		EnableRobot,
		Stop,

		NoEvent	// Dummy
	};
}


struct CursorST
{
public:
	CursorST(
			CursorStates::State start_state,
			CursorEvents::Event event)
		: start_state_(start_state),
		  event_(event)
	{}

public:
	CursorStates::State				start_state_;
	CursorEvents::Event				event_;
};


class Cursor
{
public:
	Cursor(Renderer* renderer);

	// Actions:
	CursorStates::State	newEvent(CursorEvents::Event event, bool start = true);
	void						incrPos(float ix, float iy);

	Colour					getColour(void) const;
	CursorStates::State	getState(void) const			{ return state_; }
	cv::Point2f				getPos(void) const { return pos_; }
	cv::Point2f				getPosNDC(void) const;

	bool						lockedX(void)					{ return (x_locked_ > 0); }
	bool						lockedY(void)					{ return (y_locked_ > 0); }

private:
	void						cbCursorBorder(const ros::TimerEvent&);

	void						handleSelect		(bool start);
	void						handleAdd			(void);
	void						handleDelete		(void);
	void						handleCalibrate	(bool start);
	void						handleLockX			(bool start);
	void						handleLockY			(bool start);
	void						handleEnableRobot	(bool start);
	void						handleStop			(void);

private:
	ros::NodeHandle nh_;

	Renderer* renderer_;
	std::vector<CursorST> transition_mat_;
//	std::vector<CursorEvents::Event> pending_actions_;
	CursorStates::State	state_;

	ros::Timer				cursor_border_timer_;
	cv::Point2f				pos_;

	int						x_locked_;
	int						y_locked_;
	bool						robot_movement_enabled_;
};



#endif // CURSOR_H
