#include "scene.h"
#include "interface_handler.h"
#include "objects/object.h"
#include "objects/position.h"
#include "objects/point_cloud.h"

Scene::Scene(Object* scene_hook)
{
	scene_hook_ = scene_hook;
	scene_origin_ = new Position;
}

Scene::~Scene()
{
	delete scene_origin_;
}

void Scene::addObject(Object* object)
{
	object->setParent(scene_origin_);
}

void Scene::enable()
{
	scene_origin_->setParent(scene_hook_);
}

void Scene::disable()
{
	scene_origin_->setParent(NULL);
}
