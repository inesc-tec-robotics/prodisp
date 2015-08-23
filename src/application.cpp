#include "application.h"
#include "objects/tf.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <GL/freeglut.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <signal.h>
#include "LoggerSys.hpp"

using namespace std;

// Singleton instance
template<> Application* Singleton<Application>::msInstance = 0;

void sigintHandler(int sig)
{
	FWARN("Ctrl+C detected");

	glutLeaveMainLoop();

	// scoped_ptr<Application> goes out of scope on exit(), thus calling destructor.
	// This is what GLUT already does.
//	exit(0);
}

Application::Application(int* argcp, char* argv[])
{
	FDEBUG("Application::Application");

	// Let GLUT parse command line parameters
	glutInit(argcp, argv);

	// Read configuration
	config_.reset(new Configuration());
	config_->initialize(*argcp, argv);

	// Create subsystems
	render_system_.reset(new RenderSystem());
	renderer_.reset(new Renderer());

//	signal(SIGINT, boost::bind(&Application::sigintHandler, this));
	signal(SIGINT, sigintHandler);

	FINFO("ProDisp is running...");
}

Application::~Application()
{
	cout << "Shutting down ProDisp..." << endl;
	// NOTE: The member objects are automatically destroyed in opposite order.
}

int Application::run()
{
	FDEBUG("Application::run");
	render_system_->run();
	return 0;
}

void Application::onKey(unsigned char key, int x, int y)
{
	FDEBUG("Application::onKey, key=" << key << ", x=" << x << ", y=" << y);

	Configuration& config = Configuration::getInstance();
	Renderer& renderer = Renderer::getInstance();

	key = tolower(key);
	switch (key)
	{
	case 27: // ESCAPE
		// Terminate the application. (This will still perform cleanup, due to the static creation
		// of the Application object.)

		// Cleanup subsystems before ROS closes:
//		delete renderer_;

//		exit(0);		// ROS closes here
		glutLeaveMainLoop();
		break;

	case 13: // ENTER
		renderer.stop();
		break;

	case 'h': // Switch Help display
		config.toggleGuiHelp();
		FINFO("HELP has been switched "
				<< (config.getGuiHelp() ? "ON" : "OFF")
				<< ".");
		break;

	case 'f': // Switch full screen
		config.toggleFullScreen();
		render_system_->updateWindowMode();
		FINFO("FULL SCREEN has been switched "
				<< (config.getFullScreen() ? "ON" : "OFF")
				<< ".");
		break;

	case 'p': // Goto projector view
		renderer.getCamera().reset();
		FINFO("Camera has been moved to the PROJECTOR VIEW.");
		break;

		// Camera movement:
	case 'q':
		renderer.getCamera().incRotation( 0.0, 1.0, 0.0);
		break;
	case 'e':
		renderer.getCamera().incRotation( 0.0,-1.0, 0.0);
		break;
	case 'w':
		renderer.getCamera().incTranslation( 0.0,-0.2, 0.0);
		break;
	case 's':
		renderer.getCamera().incTranslation( 0.0, 0.2, 0.0);
		break;
	case 'd':
		renderer.getCamera().incTranslation(-0.2, 0.0, 0.0);
		break;
	case 'a':
		renderer.getCamera().incTranslation( 0.2, 0.0, 0.0);
		break;
	}
}

void Application::onArrow(unsigned char key, int x, int y)
{
	FDEBUG("Application::onArrow, key=" << key << ", x=" << x << ", y=" << y);
	Configuration& config = Configuration::getInstance();
	Renderer& renderer = Renderer::getInstance();

	switch (key)
	{
	case GLUT_KEY_PAGE_UP:
		renderer.getCamera().incTranslation(0,0,0.2);
		break;

	case GLUT_KEY_PAGE_DOWN:
		renderer.getCamera().incTranslation(0,0,-0.2);
		break;

	case GLUT_KEY_RIGHT:
		renderer.setSceneNum( renderer.getSceneNum()+1 );
		break;

	case GLUT_KEY_LEFT:
		renderer.setSceneNum( renderer.getSceneNum()-1 );
		break;

	case GLUT_KEY_F1: // Switch Help display
		config.toggleGuiHelp();
		cout	<< "HELP has been switched "
				<< (config.getGuiHelp() ? "ON" : "OFF")
				<< "." << endl;
		break;
	}
	//cout << "Moving to state " << stateManager.getStateNo() << "." << endl;
}

void Application::onMouse(int button, int state, int x, int y)
{
	FDEBUG("Application::onMouse, button=" << button << ", state=" << state << ", x=" << x << ", y=" << y);
#if 0
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN
			&& mCalibrationState != -1)
	{
		Vector2 point(
			static_cast<float>(x - mRenderSystem->getStartX()) / mRenderSystem->getWidth(),
			1.0f - static_cast<float>(y - mRenderSystem->getStartY()) / mRenderSystem->getHeight());

		point *= 2.0f;
		point -= Vector2(1.0f, 1.0f);

		// Correct for aspect ratio
		point.x *= Capturer::getInstance().getAspectRatio();

		mCalibrationPoints[mCalibrationState] = point;

		cout << "Calibration point " << (mCalibrationState+1) << ": ("
			<< mCalibrationPoints[mCalibrationState].x << ", "
			<< mCalibrationPoints[mCalibrationState].y << ")\n";

		if (++mCalibrationState == NUM_CALIBRATION_POINTS)
		{
			// Perform calibration
			mEstimator->calibrate(/*mCalibrationPoints*/);

			mCalibrationState = -1;
		}
	}
#endif
}

