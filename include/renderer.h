#ifndef Renderer_H__
#define Renderer_H__

#include "prerequisites.h"
#include "objects/tf.h"
#include "singleton.h"
#include "scene.h"
#include "interface_handler.h"

#include <boost/scoped_ptr.hpp>

// Forward declarations:
class LineCoordsys;
class Object;
class Plane;
class PointCloud;
class Scene;
class WiiHandler;
class Cursor;

class Renderer : public Singleton<Renderer>
{
public:
	Renderer();
	~Renderer();

	void					start(const std::vector<Object*> &studs);
	void					stop(void);

	void					onDisplay();
	sg::Tf&				getCamera();

	// Interact:
	bool					select();
	bool					release();
	bool					markStart();
	bool					markStop();
	bool					add(void);
	bool					remove(void);

	InterfaceHandler&	getInterfaceHandler(void) { return interface_handler_; }

	int					getSceneNum() { return current_scene_num_; }
	void					setSceneNum(int scene_num);

private:
	// World handler stuff:
	void						setupBaseScene();
	void						constructScenes();
	void						setupGraphics2D();
	void						addStud(Stud *stud);
	Object*					chooseWallObj(cv::Point3d cursor);
	cv::Point3d				getCursorOnWall(void);
	std::vector<Object*>	selectInside(cv::Point2f p1, cv::Point2f p2);

	// Render graphics stuff:
	void					renderGraphics3D();
	void					renderGraphics2D();

	// GUI rendering
	void					renderHelp();

	// Test:
	void					renderTest3d();

private:
	bool                active_;
	InterfaceHandler    interface_handler_;

	// Scene graph objects:
	Object*						graph_origin_;		// Draws everything recursively
	Object*						scene_origin_;				// Normally on top of projector (except when debugging)
	Object*						wall_;
//	std::vector<Stud*>		studs_;
	std::vector<ScenePtr>	scenes_;
	int							current_scene_num_;
	Position*					cursor_hit_;

//	boost::shared_ptr< std::map<GLuint, Object*> > selectable_objects_;
	Object*						held_object_;
	std::vector<Object*>		selected_objects_;

	// Interaction stuff
	cv::Point2f					mark_start_ndc_;
	cv::Point2f					mark_stop_ndc_;

	// Scene graph objects (tests):
	Object*						rotator_;

	// 2D objects:
	boost::shared_ptr<Cursor> cursor_;
};

#endif // Renderer_H__
