#include "objects/mesh_generic.h"

#include <cstring>
#include <iostream>
#include <algorithm>
//#include <lib3ds/file.h>
//#include <lib3ds/mesh.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

bool FaceGeneric::adjacentTo(const FaceGeneric& f) const
{
	bool commonVertex = false;
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
			if (vertices[i] == f.vertices[j])
			{
				if (commonVertex == true)
					return true;		// At least two common vertices
				commonVertex = true;
			}
			
	return false;
}

void FaceGeneric::addNeighbourFace(const FaceGeneric& f, const int& faceNo)
{
	int firstCommonVertex = -1;
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
			if (vertices[i] == f.vertices[j])
			{
				if (firstCommonVertex == -1)
				{
					firstCommonVertex = i;
				} else
				{
					if ((firstCommonVertex == 0) && (i == 1))
						neighbour_faces[0] = faceNo;
					else if ((firstCommonVertex == 0) && (i == 2))
						neighbour_faces[2] = faceNo;
					else //((firstCommonVertex == 1) && (i == 2))
						neighbour_faces[1] = faceNo;
					return;
				}
			}
	throw runtime_error("Attempted to add neighbourface with no common edge (Mesh.cpp).");
}

bool FaceGeneric::hasEdge(const Edge& edge) const
{
	bool hasFirst = false;

	// Check whether the face has the first edge vertex
	for (unsigned i = 0; i < 3; ++i)
		if (vertices[i] == edge.first)
		{
			hasFirst = true;
			break;
		}
	if (!hasFirst)
		return false;

	// Check whether the face also has the second edge vertex
	for (unsigned i = 0; i < 3; ++i)
		if (vertices[i] == edge.second)
			return true;

	return false;
}

MeshGeneric::MeshGeneric(std::string filename)
{
	loadModel(filename.c_str());

	// Perform mesh precalculations
	testForDoubleVertices(filename.c_str());
	linkAdjacentFaces();
	markSharpEdges();

	calcDimensions();
}

MeshGeneric::~MeshGeneric()
{
}

void MeshGeneric::loadModel(const char* filename)
{
	cerr << "MeshGeneric::loadModel error: lib3ds models no longer supported" << endl;
//	Lib3dsFile* model = lib3ds_file_load(filename);
//	if(!model)
//	{
//		cerr << "Mesh: Error loading model " << filename << endl;
//		throw runtime_error("could not load model");
//	}

//	unsigned numVertices = 0;
//	unsigned numFaces = 0;

//	// Count total number of vertices and faces
//	for (Lib3dsMesh* mesh = model->meshes; mesh != NULL; mesh = mesh->next)
//	{
//		numVertices += mesh->points;
//		numFaces    += mesh->faces;
//	}

//	// Allocate space for vertices and faces
//	mVertices.resize(numVertices);
//	mFaces.resize(numFaces);
//	mTransformedVertices.resize(numVertices);
//	mTransformedNormals.resize(numFaces);

//	unsigned vertexOffset = 0;
//	unsigned faceOffset = 0;

//	// Copy vertices and faces from model
//	for (Lib3dsMesh* mesh = model->meshes; mesh != NULL; mesh = mesh->next)
//	{
//		// Copy vertices from this mesh
//		memcpy(&mVertices[vertexOffset], mesh->pointL, mesh->points*sizeof(Vector3));
		
//		// Copy faces from this mesh
//		for (unsigned faceIndex = 0; faceIndex < mesh->faces; ++faceIndex)
//		{
//			FaceGeneric* dest = &mFaces[faceOffset + faceIndex];
			
//			for (unsigned i = 0; i < 3; ++i)
//				dest->vertices[i] = vertexOffset + mesh->faceL[faceIndex].points[i];

//			dest->normal = Vector3(mesh->faceL[faceIndex].normal);
//		}

//		vertexOffset += mesh->points;
//		faceOffset   += mesh->faces;
//	}

//	lib3ds_file_free(model);
}

void MeshGeneric::testForDoubleVertices(const char* filename)
{
	// Outputs a warning if model contains double vertices
	vector<Vector3>::iterator it;
	vector<Vector3>::iterator itPrevious;
	unsigned int noOfRepetetiveVertices = 0;
	for (it =  mVertices.begin(); it != mVertices.end(); it++)
	{
		for(itPrevious = mVertices.begin(); itPrevious < it; itPrevious++)
		{
			if (*itPrevious == *it)
			{
				noOfRepetetiveVertices++;
				break;
			}
		}
	}
	
	if (noOfRepetetiveVertices != 0)
	{
		cout << "WARNING: Model:\n "
			  << filename
			  << "\ncontains "
			  << noOfRepetetiveVertices
			  << " double vertices!\n\n";
	}
}

void MeshGeneric::linkAdjacentFaces()
{
	int noNeighbours;
	
	// Links adjacent faces
	for (unsigned int i = 0; i < mFaces.size()-1; i++)
	{
		noNeighbours = 0;
		
		// Cycle through potential neighbours:
		for (unsigned int j = 0; j < mFaces.size(); j++)
		{
			if (mFaces.at(i).adjacentTo(mFaces.at(j)))
			{
				noNeighbours++;
				if (j > i)	// Only add new neighbourpairs:
				{
					mFaces.at(i).addNeighbourFace(mFaces.at(j),j);
					mFaces.at(j).addNeighbourFace(mFaces.at(i),i);
				}
			}
		}
		
		// Test if all faces has exactly three neigbour faces:
		if (noNeighbours != 4)	// Three neighbours plus the face itself
		{
			cout << "   noNeighbours(" << i << ") = " << noNeighbours << endl;
			throw runtime_error(
				"A loaded model contains faces with wrong number of neighbour faces! (Mesh.cpp).");
		}
	}
	
	// Test if the three neighbour faces each face are all placed on defferent sides:
	for (unsigned int i = 0; i < mFaces.size(); i++)
		for (unsigned int j = 0; j < 3; j++)
		{
			if (mFaces.at(i).neighbour_faces[j] == ~0u)
				throw runtime_error(
				"A loaded model contains faces with the 3 neighbour faces placed incorrectly! (Mesh.cpp).");
		}
}

// Mark sharp edges (and increase mNumSharpEdges)
void MeshGeneric::markSharpEdges()
{
	Configuration& config = Configuration::getInstance();
	mNumSharpEdges = 0;
// 	const float sharpAngleMin = 45.0;		// Degrees
// 	const float planeAngleMax = 0.01;		// Degrees
	const float crossMinForSharp = sin(config.getSharpAngleMin()*CV_PI/180.0);
	const float crossMaxForPlane = sin(config.getPlaneAngleMax()*CV_PI/180.0);
	
#if 0
	//TEST print vertices:
	for (unsigned int i = 0; i < mVertices.size(); i++)
			cout << "Vertex " << i << ": "
			  << "(" << mVertices.at(i).x << ", "
			  << mVertices.at(i).y << ", "
			  << mVertices.at(i).z << ")"
			  << endl;
	
	//TEST print faces:
	for (unsigned int i = 0; i < mFaces.size(); i++)
			cout << "Face " << i << ": "
			  << "(" << mVertices.at(mFaces.at(i).vertices[0]).x << ", "
			  << mVertices.at(mFaces.at(i).vertices[0]).y << ", "
			  << mVertices.at(mFaces.at(i).vertices[0]).z << "), "
			  << "(" << mVertices.at(mFaces.at(i).vertices[1]).x << ", "
			  << mVertices.at(mFaces.at(i).vertices[1]).y << ", "
			  << mVertices.at(mFaces.at(i).vertices[1]).z << "), "
			  << "(" << mVertices.at(mFaces.at(i).vertices[2]).x << ", "
			  << mVertices.at(mFaces.at(i).vertices[2]).y << ", "
			  << mVertices.at(mFaces.at(i).vertices[2]).z << "), "
			  << endl;
#endif
	
	for (unsigned int face = 0; face < mFaces.size(); face++)
	{
		for (unsigned int neighbour = 0; neighbour < 3; neighbour++)
		{
			unsigned int neighbourFace = mFaces.at(face).neighbour_faces[neighbour];
			if (neighbourFace > face)	// Avoids counting sharp edges twice
			{
				float cross = mFaces.at(face).normal.cross(
								  mFaces.at(neighbourFace).normal).length();
				EdgeType edgeType;
				if (cross > crossMinForSharp)		// TRUE if sharp edge
					edgeType = SHARP;
				else if (cross < crossMaxForPlane)
					edgeType = NOTPOSIABLESILHOUETTE;
				else
				{
					//This is a soft edge, but is it an outer edge???
					
					//The vertex not touching the neighbour and one vertex from the neighbour:
					Vector3 vertA = mVertices[mFaces.at(face).vertices[(neighbour+2)%3]];
					Vector3 vertB = mVertices[mFaces.at(face).vertices[neighbour]];
					//The two normals:
					Vector3 norA = mFaces.at(face).normal;
					Vector3 norB = mFaces.at(neighbourFace).normal;
					if( ((vertB+norB)-(vertA+norA)).length() > ((vertB+norA)-(vertA+norB)).length() )
						edgeType = SOFTOUTER;
					else
						edgeType = NOTPOSIABLESILHOUETTE;
				}
				
				// Update edge for current face:
				mFaces.at(face).edges[neighbour] = edgeType;
				
				// Update corresponding edge for neighbour edge:
				for (unsigned int i = 0; i < 3; i++)
					if (mFaces.at(neighbourFace).neighbour_faces[i] == face)
						mFaces.at(neighbourFace).edges[i] = edgeType;
			}
		}
	}
}

// Calculate x-, y-, and z-dimensions (for synthesizing from correct viewpoint)
void MeshGeneric::calcDimensions()
{
	float xmax = std::numeric_limits<float>::min();
	float ymax = std::numeric_limits<float>::min();
	float zmax = std::numeric_limits<float>::min();
	float xmin = std::numeric_limits<float>::max();
	float ymin = std::numeric_limits<float>::max();
	float zmin = std::numeric_limits<float>::max();

	vector<Vector3>::iterator it;
	for (it =  mVertices.begin(); it != mVertices.end(); it++)
	{
		xmax = max(xmax, it->c[0]);
		xmin = min(xmin, it->c[0]);
		ymax = max(ymax, it->c[1]);
		ymin = min(ymin, it->c[1]);
		zmax = max(zmax, it->c[2]);
		zmin = min(zmin, it->c[2]);
	}

	mCenter.c[0] = (xmax+xmin)/2;
	mCenter.c[1] = (ymax+ymin)/2;
	mCenter.c[2] = (zmax+zmin)/2;

	mDimX = xmax - xmin;
	mDimY = ymax - ymin;
	mDimZ = zmax - zmin;
}

void MeshGeneric::findVisibleEdges(const Matrix4& modelView, const Matrix4& projection)
{
	mVisibleEdges.clear();

	// Transform and project vertices and normals
	applyTransformation(modelView, projection);

	// Reset candidate edges and add candidate edges
	mCandidateEdges.resize(0);
	addCandidateEdges();

	// Add only visible candidate edges to the final set of visible edges
	addVisibleEdges();
}

void MeshGeneric::applyTransformation(const Matrix4& modelView, const Matrix4& projection)
{
	// Transform vertices into view space
	for (unsigned i = 0; i < mVertices.size(); ++i)
		mTransformedVertices[i] = modelView.transformAffine(mVertices[i]);

	// Transform normals into view space
	for (unsigned i = 0; i < mFaces.size(); ++i)
		mTransformedNormals[i] = modelView.transform(mFaces[i].normal);

	// Determine whether faces are facing towards the camera (front-faces)
	for (unsigned i = 0; i < mFaces.size(); ++i)
	{
		mFaces[i].front_face = dot(mTransformedNormals[i],
				-mTransformedVertices[mFaces[i].vertices[0]]) > 0.0f;
	}

	// Transform the transformed vertices into normalized device coordinates; [-1;1] in x, y, and z:
	for (unsigned i = 0; i < mVertices.size(); ++i)
	{
		// Apply perspective projection:
		mTransformedVertices[i] = projection.transform(mTransformedVertices[i]);

		if (mTransformedVertices[i].z() == 0.0f)
			continue; // avoid division by zero

		// Divide by depth:
		mTransformedVertices[i].c[0] /= -mTransformedVertices[i].c[2];
		mTransformedVertices[i].c[1] /= -mTransformedVertices[i].c[2];
	}
}

void MeshGeneric::addCandidateEdges()
{
	// Add silhouette edges and visible sharp edges to candidate edges
	for (unsigned faceIndex = 0; faceIndex < mFaces.size(); ++faceIndex)
	{
		const FaceGeneric& face = mFaces[faceIndex];

		for (unsigned edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
		{
			unsigned neighbourIndex = face.neighbour_faces[edgeIndex];

			if (neighbourIndex > faceIndex)
			{
				if (face.edges[edgeIndex] == SOFTOUTER)
				{
					// Edges between front- and back-faces are silhouette edges
					if (face.front_face != mFaces[neighbourIndex].front_face)
					{
						mCandidateEdges.push_back(std::make_pair(
									face.vertices[edgeIndex],
									face.vertices[(edgeIndex + 1) % 3]));
					}
				}else if (face.edges[edgeIndex] == SHARP)
				{
					// Sparp edges touching a front-face are candidate edges
					if ( face.front_face || mFaces[neighbourIndex].front_face)
					{
						mCandidateEdges.push_back(std::make_pair(
									face.vertices[edgeIndex],
									face.vertices[(edgeIndex + 1) % 3]));
					}
				}
			}
		}
	}
}

struct IsEdgeHidden
{
	const MeshGeneric& mesh;

	IsEdgeHidden(const MeshGeneric& mesh) : mesh(mesh) {}

	bool operator()(const Edge& edge)
	{
		const MeshGeneric::FaceVector&   faces    = mesh.getFaces();
		const MeshGeneric::VertexVector& vertices = mesh.getTransformedVertices();

		// Check whether the edge is hidden behind any face
		for (unsigned faceIndex = 0; faceIndex < mesh.getFaces().size(); ++faceIndex)
		{
			// Only front-faces are relevant
			if (!faces[faceIndex].front_face)
				continue;

			// A face which has this edge can not hide it
			if (faces[faceIndex].hasEdge(edge))
				continue;

			float s, t;
			Vector3 segments[4];
			segments[0] = vertices[edge.first];
			segments[1] = vertices[edge.second];

			unsigned numIntersections = 0;
			unsigned numIntersectionsOfInfinitEdge = 0;
			bool definitelyOutside = true;

			// Check for intersection between the edge and each edge of the face
			for (unsigned i = 0; i < 3 && numIntersectionsOfInfinitEdge < 2; ++i)
			{
				if(i==2 && definitelyOutside)
					break;//if the edge does not point in the direction of one of the first 2 fase-edges it does not point at the last one.

				segments[2] = vertices[faces[faceIndex].vertices[i]];
				segments[3] = vertices[faces[faceIndex].vertices[(i+1)%3]];

#define TEST_IF_INSIDE_IN_FIRST_TEST
#ifdef TEST_IF_INSIDE_IN_FIRST_TEST
				float lastS;
#endif

				if (lineIntersection(segments, s, t))
				{
					if (t < EPSILON || t > 1.0f-EPSILON)
						continue; // segments do not intersect
					else
						definitelyOutside = false;

					numIntersectionsOfInfinitEdge++;

#ifdef TEST_IF_INSIDE_IN_FIRST_TEST
					// If InfinitLineIntersections==2 (one previous intersection)
					// and the previous intersection was not inside a line segment
					// test if the edge is outside the face
					if ( numIntersections==0 && numIntersectionsOfInfinitEdge==2 )
						if ( (s < EPSILON && lastS < EPSILON) 
								|| (s > 1.0f-EPSILON && lastS > 1.0f-EPSILON ) )
						{
							definitelyOutside = true;
							break;
						}//else: at least one edge-endpoint is inside the face
					lastS = s;
#endif

					if (s < EPSILON || s > 1.0f-EPSILON )
						continue; // segments do not intersect

					numIntersections++;

					// Calculate z-coordinates at the intersection
					float zEdge = interpolatePerspective(s, segments[0].z(), segments[1].z());
					float zFace = interpolatePerspective(t, segments[2].z(), segments[3].z());

					// Check whether the edge is behind the face at this intersection
					if (zEdge < zFace - EPSILON)
						return true; // the edge is behind the face at the intersection
				}
			}

			if (numIntersections == 2 || definitelyOutside)
				continue; // there are two intersections in front, or the edge is definitely outside

			// NOTE: Here there is either no intersection, or there is a single intersection
			//       which is in front of the face.

			segments[0] = vertices[faces[faceIndex].vertices[0]];
			segments[1] = vertices[faces[faceIndex].vertices[1]];
			segments[2] = vertices[faces[faceIndex].vertices[2]];

			// Check for each end-point whether it is inside the face, and then if it is behind
			// the face.
			for (unsigned i = 0; i < 2; ++i)
			{
				segments[3] = vertices[(i == 0) ? edge.first : edge.second];

				if (lineIntersection(segments, s, t))
				{
					t = 1.0f/t;

					// Check whether the end-point is inside the face
					if (s < EPSILON || s > 1.0f-EPSILON || t < EPSILON || t > 1.0f-EPSILON)
						continue; // end-point is outside the face

					// Calculate the interpolated depth of the triangle at the intersection
					float z     = interpolatePerspective(s, segments[0].z(), segments[1].z());
					float zFace = interpolatePerspective(t, segments[2].z(), z);
					float zEdge = segments[3].z();

					if (zEdge < zFace - EPSILON)
						return true; // the edge is behind the face at the intersection
					
					if (numIntersections == 1)
						break;//If there are only 1 intersection and this end point is inside the face, the other end-point is outside.
				}
			}
		}

		return false;
	}
};

void MeshGeneric::addVisibleEdges()
{
	// Copy all non-hidden edges from candidate edges to visible edges
	remove_copy_if(mCandidateEdges.begin(), mCandidateEdges.end(), 
			back_inserter(mVisibleEdges), IsEdgeHidden(*this));
}
