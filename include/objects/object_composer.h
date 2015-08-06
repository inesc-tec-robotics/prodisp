#ifndef OBJECT_COMPOSER_H
#define OBJECT_COMPOSER_H

#include "singleton.h"
#include "objects/object.h"
#include "objects/tf.h"

class ObjectComposer : public Singleton<ObjectComposer>
{
public:
	ObjectComposer();
	~ObjectComposer();

//	Object* getStud(void);
	Object* getRing(void);
	Object* getColourCube(float x, float y, float z);
	Object* getCoordBlocks(float lengths);
};


#endif // OBJECT_COMPOSER_H
