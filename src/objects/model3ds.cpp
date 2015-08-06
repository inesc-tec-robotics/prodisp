//// inspireed by http://donkerdump.nl/node/207
//#include "objects/model3ds.h"

//#include <lib3ds/mesh.h>
//#include <cstring>
//#include <GL/gl.h>
//#include <iostream>

//using namespace std;

//// Load 3DS model
//Model3ds::Model3ds(std::string filename, Colour ambient, Colour diffuse,
//						  Colour specular, GLfloat shineness)
//	: Renderable(GL_TRIANGLES), mAmbient(ambient), mDiffuse(diffuse),
//	mSpecular(specular), mShineness(shineness)
//{
//	Lib3dsFile * model = lib3ds_file_load(filename.c_str());
//	// If loading the model failed, we throw an exception
//	if(!model)
//	{
//		cerr << "Exception: Unable to load modelfile: "
//				  << filename.c_str() << endl;
//		throw runtime_error("Exception in Model3d.cpp.");
//	}

//	InitialiseVBO(model);
	
//	if (mNumVertices == 0)
//	{
//		cerr << "Exception: Modelfile does not contain any vertices: "
//				  << filename.c_str() << endl;
//		throw runtime_error("Exception in Model3d.cpp.");
//	}
	

//	// We no longer need lib3ds
//	lib3ds_file_free(model);
//	model = NULL;
//}

//Model3ds::~Model3ds()
//{
//}

//// Copy vertices and normals to the memory of the GPU
//void Model3ds::InitialiseVBO(Lib3dsFile * model)
//{
//	assert(model != NULL);
	
//	// Calculate the number of vertices/faces we have in total
//	CountVertices(model);
	
//	// Allocate memory for our vertices and normals
//	Lib3dsVector * vertices = new Lib3dsVector[mNumVertices];
//	Lib3dsVector * normals = new Lib3dsVector[mNumVertices];
	
//	Lib3dsMesh * mesh;
//	unsigned int FinishedFaces = 0;
//	// Loop through all the meshes
//	for(mesh = model->meshes;mesh != NULL;mesh = mesh->next)
//	{
//		lib3ds_mesh_calculate_normals(mesh, &normals[FinishedFaces*3]);
//		// Loop through every face
//		for(unsigned int cur_face = 0; cur_face < mesh->faces;cur_face++)
//		{
//			Lib3dsFace * face = &mesh->faceL[cur_face];
//			for(unsigned int i = 0;i < 3;i++)
//			{
//				memcpy(&vertices[FinishedFaces*3 + i], mesh->pointL[face->points[i]].pos, sizeof(Lib3dsVector));
//			}
//			FinishedFaces++;
//		}
//	}
	
//	// Initialise a VBO for the vertices and a VBO for the normals
//	mVertexVBO.reset(new VertexBufferObject(3, GL_FLOAT, mNumVertices,GL_STATIC_DRAW, vertices));
//	mNormalVBO.reset(new VertexBufferObject(3, GL_FLOAT, mNumVertices,GL_STATIC_DRAW, normals,NORMAL));
	
//	// Clean up our allocated memory
//	delete vertices;
//	delete normals;
//}

//// Count the total number of Vertices this model has
//void Model3ds::CountVertices(Lib3dsFile * model)
//{
//	assert(model != NULL);
	
//	unsigned int numFaces = 0;
//	Lib3dsMesh * mesh;
//	// Loop through every mesh
//	for(mesh = model->meshes;mesh != NULL;mesh = mesh->next)
//	{
//		// Add the number of faces this mesh has to the total faces
//		numFaces += mesh->faces;
//	}
//	mNumVertices = 3*numFaces;// 3 vertices per triangle
//}
