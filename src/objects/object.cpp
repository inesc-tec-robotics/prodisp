#include "objects/object.h"

using std::list;

std::set<Object*> Object::selectable_objects_;

Object::Object()
	: parent_(NULL)
//	, name_(0)
{
}

Object::Object(const Object& src)
	: parent_(NULL)
//	, name_(0)
{
	setParent(src.parent_);
	children_.clear();
	transform_ = src.transform_;

//	// If selectable, choose a unique name:
//	if (src.name_ != 0)
//		makeSelectable(selectable_objects_);

	// Copy vertices:
	for (int i = 0; i < (int)src.vertices_.size(); ++i)
		addVertex(src.vertices_[i]->c);
}

Object::~Object()
{
	// Remove from selectable:
	if (selectable_objects_.find(this) != selectable_objects_.end())
	{
		selectable_objects_.erase(this);
	}

	// Delete vertices:
	for (int i = 0; i < (int)vertices_.size(); ++i)
		delete vertices_[i];

	// Delete from parent's children list:
	if (parent_ != NULL)
	{
		parent_->children_.remove(this);
		parent_ = NULL;
	}

	// Delete this' children:
	while(!children_.empty())
	{
		delete children_.front();	// Will also remove from list<Object*> children_
	}
}

void Object::setParent(Object* parent)
{
	if (parent == NULL && parent_ == NULL)
	{
		// Do nothing
	}
	else if (parent == NULL && parent_ != NULL)
	{
		// Remove parent:
		parent_->children_.remove(this);
		parent_ = NULL;
	}
	else
	{
		// Make sure *this is not in the parent chain of the new parent:
		bool parent_paradox = false;
		for (Object* p = this; p->getParent() != NULL; p = p->getParent())
		{
			if (p->getParent() == this)
			{
				parent_paradox = true;
				std::cerr << "Attempting to create parent chain!" << std::endl;
				break;
			}
		}

		// Set new parent:
		if (!parent_paradox)
		{
			if (parent_ != NULL)
			{
				parent_->children_.remove(this);
			}
			parent_ = parent;
			parent_->children_.push_back(this);
		}
	}
}

Object* Object::getParent(void)
{
	return parent_;
}

std::list<Object*>& Object::getChildren(void)
{
	return children_;
}

bool Object::drawTest()
{
	bool drawn = false;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef(transform_.translation_.x(),
					 transform_.translation_.y(),
					 transform_.translation_.z());
	glRotatef(transform_.rotation_deg_.x()*0.1, 1, 0, 0);
	glRotatef(transform_.rotation_deg_.y(), 0, 1, 0);
	glRotatef(transform_.rotation_deg_.z(), 0, 0, 1);

	// Draw this:
	drawn = drawVertices();

	// Draw children:
	for (list<Object*>::iterator it = children_.begin(); it != children_.end(); ++it)
	{
		Object* obj = dynamic_cast<Object*>(*it);

		if (obj != NULL)
		{
			drawn = obj->draw() | drawn;
		}
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	return drawn;
}

bool Object::draw()
{
	bool drawn = false;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef(transform_.translation_.x(),
					 transform_.translation_.y(),
					 transform_.translation_.z());
	glRotatef(transform_.rotation_deg_.z(), 0, 0, 1);	// ZYX Euler angles
	glRotatef(transform_.rotation_deg_.y(), 0, 1, 0);
	glRotatef(transform_.rotation_deg_.x(), 1, 0, 0);

	// Draw this:
	drawn = drawVertices();

	// Draw children:
	for (list<Object*>::iterator it = children_.begin(); it != children_.end(); ++it)
	{
		Object* obj = dynamic_cast<Object*>(*it);

		if (obj != NULL)
		{
			drawn = obj->draw() | drawn;
		}
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	return drawn;
}

MathOp::Transform Object::getTfFromOrigin(void) const
{
	if (parent_ == NULL)
	{
		return transform_.getMathOp();
	}
	else
	{
		return parent_->getTfFromOrigin() * transform_.getMathOp();
	}
}

void Object::scaleVerticesAlong(float x, float y, float z)
{
	for (int i = 0; i < (int)vertices_.size(); ++i)
	{
		vertices_[i]->c[0] *= x;
		vertices_[i]->c[1] *= y;
		vertices_[i]->c[2] *= z;
	}
}

void Object::scaleVertices(float scale)
{
	scaleVerticesAlong(scale, scale, scale);
}

//GLuint Object::makeSelectable(boost::weak_ptr< std::map<GLuint, Object*> > selectable_objects)
//{
//	// Store reference to map:
//	selectable_objects_ = selectable_objects;
//	boost::shared_ptr< std::map<GLuint, Object*> > map_ptr = selectable_objects_.lock();

//	// Find name:
//	name_ = 1;
//	while (map_ptr->find(name_) != map_ptr->end())
//	{
//		name_++;
//	}

//	// Insert:
//	(*map_ptr)[name_] = this;

//	return name_;
//}

//void Object::makeSelectable(void)
//{
//	selectable_objects_.insert(this);
//}

int Object::addVertex(float x, float y, float z)
{
	vertices_.push_back( new Vector3(x, y, z) );

	return ((int)vertices_.size()-1);
}

int Object::addVertex(const float v[3])
{
	vertices_.push_back( new Vector3(v) );

	return ((int)vertices_.size()-1);
}

int Object::addVertex(const Vector3& v)
{
	vertices_.push_back(new Vector3(v));

	return ((int)vertices_.size()-1);
}

