#include "cursor.h"
#include "configuration.h"
#include "renderer.h"

#include "LoggerSys.hpp"

using namespace std;

namespace CE = CursorEvents;
namespace CS = CursorStates;

Cursor::Cursor(Renderer* renderer)
	: renderer_(renderer)
	, state_(CursorStates::Idle)
	, pos_(0, 0)
	, x_locked_(0)
	, y_locked_(0)
{
	FDEBUG("Cursor::Cursor");

	// Start timer:
	cursor_border_timer_ = nh_.createTimer(ros::Duration(0.04), &Cursor::cbCursorBorder, this);
}

Colour Cursor::getColour(void) const
{
	Colour c;

	switch (state_)
	{
		case CursorStates::Marking:
		case CursorStates::Holding:
			c = Colour::GREEN;
		break;

		case CursorStates::Idle:
			c = Colour::WHITE;
		break;

		case CursorStates::Calibrating:
			c = Colour::MAGENTA;
		break;

		default:
			FERROR("Unknown cursor state: " << state_);
	}

	return c;
}

CursorStates::State Cursor::newEvent(CursorEvents::Event event, bool start)
{
	FDEBUG("Cursor::newEvent: " << event);

	switch (event)
	{
	case CursorEvents::Select:		handleSelect(start);		break;
	case CursorEvents::Add:			handleAdd();				break;
	case CursorEvents::Delete:		handleDelete();			break;
	case CursorEvents::Calibrate:	handleCalibrate(start);	break;
	case CursorEvents::LockX:		handleLockX(start);		break;
	case CursorEvents::LockY:		handleLockY(start);		break;
	case CursorEvents::Stop:		handleStop();				break;
	default: FERROR("Unknown event:" << event);				break;
	}

//	for (int i = 0; i < (int)transition_matrix_.size(); ++i)
//	{
//		if (transition_matrix_[i].start_state_ == state_ &&
//			 transition_matrix_[i].event_ == event)
//		{
//			// Change state if possible:
//			setState(transition_matrix_[i].end_state_);
//			break;
//		}
//	}

	return state_;
}

void Cursor::incrPos(float ix, float iy)
{
	static const Rectf limits = Configuration::getInstance().getProj2dEyeArea();

	if (x_locked_ == 0)
	{
		pos_.x += ix;
		if(pos_.x < limits.left)
			pos_.x = limits.left;
		if(pos_.x > limits.right)
			pos_.x = limits.right;
	}

	if (y_locked_ == 0)
	{
		pos_.y += iy;
		if(pos_.y < limits.bottom)
			pos_.y = limits.bottom;
		if(pos_.y > limits.top)
			pos_.y = limits.top;
	}
}

cv::Point2f Cursor::getPosNDC(void) const
{
	static const Rectf limits = Configuration::getInstance().getProj2dEyeArea();
	return cv::Point2f(pos_.x/limits.right, pos_.y/limits.top);
}

//cv::Point2f Cursor::getPosViewport(void) const
//{
//	static const double resx_half = Configuration::getInstance().getProjResX() / 2.0;
//	static const double resy_half	= Configuration::getInstance().getProjResY() / 2.0;

//	cout << "pos_.x = " << pos_.x << ", resy_half = " << resy_half << ", resx_half = " << resx_half << endl;

//	return cv::Point2f(pos_.x*resy_half+resx_half, pos_.y*resy_half+resy_half);
//}

void Cursor::cbCursorBorder(const ros::TimerEvent&)
{
	static const Rectf limits = Configuration::getInstance().getProj2dEyeArea();
	const static float b_curs = 0.07;
	const static float b_proj = 0.06;
	const static float vel_curs = 0.002;
	const static float rot_proj = 0.1;	// [rad]

	float curs_x = 0.0;
	float curs_y = 0.0;
	float proj_x = 0.0;
	float proj_y = 0.0;

	// Cursor movement x:
	float dist_left  = pos_.x-limits.left;
	float dist_right = limits.right-pos_.x;
	float dist_abs_x = min(dist_left, dist_right);
	int sign_x = dist_left < dist_right ? 1 : -1;
	if (dist_abs_x < b_curs)
		curs_x = sign_x * vel_curs;

	// Projector movement x:
	if (dist_abs_x < b_proj)
		proj_x = sign_x * rot_proj;

	// Cursor movement y:
	float dist_bot = pos_.y-limits.bottom;
	float dist_top = limits.top-pos_.y;
	float dist_abs_y = min(dist_bot, dist_top);
	int sign_y = dist_bot < dist_top ? 1 : -1;
	if (dist_abs_y < b_curs)
		curs_y = sign_y * vel_curs;

	// Projector movement y:
	if (dist_abs_y < b_proj)
		proj_y = sign_y * rot_proj;

	// Move cursor:
	if (curs_x != 0.0 || curs_y != 0.0)
		incrPos(curs_x, curs_y);

	// Move robot arm:
	if (state_ != CursorStates::Calibrating && (proj_x != 0.0 || proj_y != 0.0))
	{
		// RSA_TODO comment in
//		Renderer::getInstance().getInterfaceHandler().moveProjectorIncr(proj_x, proj_y);
	}

}

void Cursor::handleSelect(bool start)
{
	if (start)
	{
		if (state_ == CS::Idle)
		{
			if (renderer_->select())
			{
				state_ = CS::Holding;
			}
			else if (renderer_->markStart())
			{
				state_ = CS::Marking;
			}
		}
	}

	if (!start)
	{
		if (state_ == CS::Holding)
		{
			if (renderer_->release())
				state_ = CS::Idle;
		}
		else if (state_ == CS::Marking)
		{
			if (renderer_->markStop())
				state_ = CS::Idle;
		}
	}
}

void Cursor::handleAdd(void)
{
	renderer_->add();
}

void Cursor::handleDelete(void)
{
	renderer_->remove();
}

void Cursor::handleCalibrate(bool start)
{
	if (start)
	{
		x_locked_ = 0;
		y_locked_ = 0;
		state_ = CursorStates::Calibrating;
	}
	else
	{
		state_ = CursorStates::Idle;
	}
}

void Cursor::handleLockX(bool start)
{
	if (state_ != CursorStates::Calibrating)
	{
		if (start)
			x_locked_++;
		else
			x_locked_ = max(0, x_locked_-1);
	}
}

void Cursor::handleLockY(bool start)
{
	if (state_ != CursorStates::Calibrating)
	{
		if (start)
			y_locked_++;
		else
			y_locked_ = max(0, y_locked_-1);
	}
}

void Cursor::handleStop(void)
{
	if (state_ != CursorStates::Calibrating)
	{
		renderer_->stop();
	}
}






//CursorEvents::Event Cursor::pendingActionGetNext(void)
//{
//	if (pending_actions_.size() > 0)
//		return pending_actions_.back();
//	else
//		return CursorEvents::NoEvent;
//}

//void Cursor::pendingActionAccept(void)
//{
//	if (pending_actions_.size() == 0)
//		return;

//	// Do transition:


//	pending_actions_.pop_back();
//}

//void Cursor::pendingActionReject(void)
//{
//	if (pending_actions_.size() > 0)
//		pending_actions_.pop_back();
//}














