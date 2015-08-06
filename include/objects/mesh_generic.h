#ifndef Mesh_H__
#define Mesh_H__

#include "prerequisites.h"
#include "configuration.h"

#include <vector>
#include <boost/shared_ptr.hpp>

typedef std::pair<unsigned, unsigned> Edge;

enum EdgeType
{
	SHARP,
	SOFTOUTER,
	NOTPOSIABLESILHOUETTE
};

struct FaceGeneric
{
	unsigned vertices[3];
	Vector3 normal;
	EdgeType edges[3];
	unsigned int neighbour_faces[3];
	bool front_face; // updated in MeshGeneric::applyTransformation

	FaceGeneric()
	{
		neighbour_faces[0] = ~0;
		neighbour_faces[1] = ~0;
		neighbour_faces[2] = ~0;
	}

	bool adjacentTo(const FaceGeneric& f) const;
	void addNeighbourFace(const FaceGeneric& f, const int& faceNo);
	bool hasEdge(const Edge& edge) const;
};

class MeshGeneric
{
public:
	typedef std::vector<Vector3> VertexVector;
	typedef std::vector<Vector3> NormalVector;
	typedef std::vector<FaceGeneric> FaceVector;
	typedef std::vector<Edge> EdgeVector;

	MeshGeneric(std::string filename);
	~MeshGeneric();

	const VertexVector& getVertices() const { return mVertices; }
	const FaceVector& getFaces() const { return mFaces; }

	// Hidden Line Removal
	void findVisibleEdges(const Matrix4& modelView, const Matrix4& projection);
	const EdgeVector& getVisibleEdges() const { return mVisibleEdges; }
	const VertexVector& getTransformedVertices() const { return mTransformedVertices; }
	const NormalVector& getTransformedNormals() const { return mTransformedNormals; }

	Vector3 getCenter() const { return mCenter; }
	float getDimX() const { return mDimX; }
	float getDimY() const { return mDimY; }
	float getDimZ() const { return mDimZ; }
	float getDimMax() const { return std::max(std::max(mDimX,mDimX),mDimZ); }

private:
	VertexVector mVertices;
	FaceVector mFaces;

	// Hidden Line Removal
	unsigned mNumSharpEdges;
	EdgeVector mCandidateEdges;
	EdgeVector mVisibleEdges;
	VertexVector mTransformedVertices;
	NormalVector mTransformedNormals;

	void loadModel(const char* filename);
	void testForDoubleVertices(const char* filename);
	void linkAdjacentFaces();
	void markSharpEdges();
	void calcDimensions();

	// Hidden Line Removal
	void applyTransformation(const Matrix4& modelView, const Matrix4& projection);
	void addCandidateEdges();
	void addVisibleEdges();

	Vector3 mCenter;
	float mDimX;	// width
	float mDimY;	// depth
	float mDimZ;	// height
};

typedef boost::shared_ptr<MeshGeneric> Mesh_ptr;

#endif // Mesh_H__
