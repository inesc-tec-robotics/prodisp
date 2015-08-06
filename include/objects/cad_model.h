#ifndef CAD_MODEL_H
#define CAD_MODEL_H

#include "objects/object.h"

// Forward declarations:
struct _GLMmodel;


class CadModel : public Object
{
public:
	CadModel(std::string obj_name);
//	CadModel(const PointCloud& src);	// Copy constructor
	virtual ~CadModel();

	virtual void	setColour(const Colour& colour)	{}
	void				setWireframe(bool mode)				{ wireframe_mode_ = mode; }

private:
	bool				readModel(std::string obj_path);
	bool				drawVertices();

private:
	bool wireframe_mode_;
	_GLMmodel*	meshmodel_;
	GLuint		dl_;
};

#endif // CAD_MODEL_H
