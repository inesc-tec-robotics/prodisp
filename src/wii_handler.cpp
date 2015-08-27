#include "wii_handler.h"

#include "LoggerSys.hpp"

using namespace std;

WiiHandler::WiiHandler()
	: nh_("~")
{
	FDEBUG("WiiHandler::WiiHandler");

	nh_.getParam("wii_offset_x", offset_x_);
	nh_.getParam("wii_offset_y", offset_y_);

	// Start curser:
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/wiimote/joy", 10, &WiiHandler::joyCallback, this);
}

WiiHandler::~WiiHandler()
{
	FDEBUG("WiiHandler::~WiiHandler");
}

void WiiHandler::bindCursor(boost::weak_ptr<Cursor> cursor)
{
	cursor_ = cursor;
}

void WiiHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// Acquire lock on cursor:
	boost::shared_ptr<Cursor> cursor = cursor_.lock();

	// Ignore callback if no cursor is bound:
	if (!cursor)
		return;

	vector<float> t = joy->axes;
	vector<int> buttons_down = joy->buttons;

	if (buttons_down_last_.size() > 0)
	{
		for (int i = 0; i < (int)buttons_down.size(); i++)
		{
			// Check if button state has changed:
			if (buttons_down[i] != buttons_down_last_[i])
			{
				WiiButtons button = (WiiButtons)i;
				bool down = buttons_down[i];

				switch (button)
				{
				case Wii_1:
					if (down)
					{
						if (cursor->newEvent(CursorEvents::Calibrate, true) == CursorStates::Calibrating)
						{
							FINFO("Calibrating...");
							calibration_samples_ = -100;
							calib_x_sum_ = 0;
							calib_y_sum_ = 0;
						}
					}
					break;

				case Wii_2:																				break;
				case Wii_A:			cursor->newEvent(CursorEvents::Select, down);		break;
				case Wii_BACK:		cursor->newEvent(CursorEvents::EnableRobot, down);	break;
				case Wii_PLUS:		if(down) cursor->newEvent(CursorEvents::Add);		break;
				case Wii_MINUS:	if(down) cursor->newEvent(CursorEvents::Delete);	break;
				case Wii_LEFT:
				case Wii_RIGHT:	cursor->newEvent(CursorEvents::LockY, down);			break;
				case Wii_UP:
				case Wii_DOWN:		cursor->newEvent(CursorEvents::LockX, down);			break;
				case Wii_HOME:		if(down) cursor->newEvent(CursorEvents::Stop);	break;
				default:				FERROR("Unknown Wii button detected: " << button)	break;
				}
			}
		}
	}


	// Perform calibration:
	if (cursor->getState() == CursorStates::Calibrating)
	{
		calibration_samples_++;
		if (calibration_samples_ > 0)
		{
			calib_x_sum_ += joy->axes[3];
			calib_y_sum_ += joy->axes[5];
		}

		// Stop calibration:
		if (calibration_samples_ == 200)
		{
			offset_x_ = calib_x_sum_/200.0;
			offset_y_ = calib_y_sum_/200.0;
			calibration_samples_ = 0;
			cursor->newEvent(CursorEvents::Calibrate, false);

			nh_.setParam("wii_offset_x", offset_x_);
			nh_.setParam("wii_offset_y", offset_y_);

			FINFO("Calibration finished");
		}
	}

	// Move cursor:
	float x_incr = -0.03*(joy->axes[3] - offset_x_);
	float y_incr = 0.03*(joy->axes[5] - offset_y_);
	cursor->incrPos(x_incr, y_incr);

	// Update buttons for next iteration:
	buttons_down_last_ = buttons_down;
}
