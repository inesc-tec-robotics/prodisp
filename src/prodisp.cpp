#include "application.h"

#include "LoggerSys.hpp"
#include "ros/init.h"
#include <iostream>
#include <boost/scoped_ptr.hpp>

// Global due to GLUT's flawed design (calling exit() directly on termination)
//static boost::scoped_ptr<Application> app;

int main(int argc, char* argv[])
{
	// Start ROS:
	ros::init(argc, argv, "prodisp");
	InitLogger("prodisp", true, true, true);

	FINFO("Starting ProDisp...");

	int ret_val = -1;
	try
	{
		Application app(&argc, argv);
		ret_val = app.run();
	}
	catch (std::exception& e)
	{
		std::cerr << "Uncaught exception: " << e.what() << std::endl;
	}
	catch (...)
	{
		std::cerr << "Uncaught exception: Unknown exception\n";
	}

	// Close ROS:
	ros::shutdown();

	return ret_val;
}
