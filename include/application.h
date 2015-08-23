#ifndef Application_H__
#define Application_H__

#include "prerequisites.h"
#include "singleton.h"
#include "configuration.h"
#include "interface_handler.h"
#include "render_system.h"
#include "renderer.h"

#include <boost/scoped_ptr.hpp>

class Application : public Singleton<Application>
{
public:
	Application(int* argcp, char* argv[]);
	~Application();

	int run();

	void onKey(unsigned char key, int x, int y);
	void onArrow(unsigned char key, int x, int y);
	void onMouse(int button, int state, int x, int y);

private:
//	void sigintHandler(int sig);

	boost::scoped_ptr<Configuration> config_;
	boost::scoped_ptr<RenderSystem> render_system_;
	boost::scoped_ptr<Renderer> renderer_;
};

#endif // Application_H__
