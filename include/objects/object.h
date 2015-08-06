#ifndef OBJECT_H
#define OBJECT_H

#include "singleton.h"
#include "objects/object.h"
#include "objects/tf.h"

#include <boost/weak_ptr.hpp>

class Object
{
public:
	Object();
	Object(const Object& src);	// Copy constructor
	virtual ~Object();

	void setParent(Object* parent);
	Object* getParent(void);
	std::list<Object*>& getChildren(void);

	bool drawTest(void);
	bool draw(void);
	sg::Tf& getTf(void)		{ return transform_; }
	MathOp::Transform getTfFromOrigin(void) const;

	void scaleVertices(float scale);
	void scaleVerticesAlong(float x, float y, float z);
	virtual void setColour(const Colour& colour) = 0;

	void makeSelectable(void) { selectable_objects_.insert(this); }
	static std::set<Object*>& getSelectable() { return selectable_objects_; }

protected:
	int addVertex(float x, float y, float z);
	int addVertex(const float v[3]);
	int addVertex(const Vector3& v);

	virtual bool drawVertices() = 0;	// Called by draw()

protected:
	std::vector<Vector3*> vertices_;

	Object* parent_;
	std::list<Object*> children_;
	sg::Tf transform_;

private:
	// Selection:
//	boost::weak_ptr< std::map<GLuint, Object*> > selectable_objects_;
	static std::set<Object*> selectable_objects_;
//	GLuint name_;		// Selectable: name_ > 0
};



#endif // OBJECT_H
