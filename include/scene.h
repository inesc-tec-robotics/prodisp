#ifndef SCENE_H
#define SCENE_H

#include <boost/shared_ptr.hpp>

class Object;
class Position;
class PointCloud;

class Scene
{
public:
	Scene(Object* scene_hook);
	~Scene();

	// Note: addObject should be changed to only setting parent on enable to avoid dublicate models.
	// This requires a new way of deleting all object, eg. a static vector<Object*> in the Object class.
	void			addObject(Object* object);
	void			enable();
	void			disable();

	Position*	getOrigin() { return scene_origin_; }

private:
	Object*		scene_hook_;
	Position*	scene_origin_;
};

typedef boost::shared_ptr<Scene> ScenePtr;


#endif // SCENE_H
