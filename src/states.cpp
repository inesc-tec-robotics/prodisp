#include "Configuration.h"

#include <string>
#include <iostream>

using namespace std;

// Singleton instance
template<> StateManager* Singleton<StateManager>::msInstance = 0;


#if 0  //USE GRUNDFOS HIFI-MODELS?
	const std::string HIFIDIR = "../../secret_grundfos/cad/";
	const GLfloat BASEPLATE_OFFSET_Z = 12.0f;
#else
	const std::string HIFIDIR = "cad_models/";
	const GLfloat BASEPLATE_OFFSET_Z = 0.0f;
#endif //USE GRUNDFOS HIFI-MODELS?
const std::string LOFIDIR = "cad_models/";


State::State(std::string partName, unsigned int noParts, int visibleForXStates)
	: mName(partName), mWarning(""), mUseForce(false), mHasWarning(false),
	  mNoParts(noParts), mVisibleForXStates(visibleForXStates)
{
}

StateManager::StateManager()
	: mStateNo(0), 
	  mNoStates(0)//mNoStates is reset last in this constructor
{
	vector<string> model_files = Configuration::getInstance().getModels();

	for (unsigned int i = 0 ; i < model_files.size(); i++)
	{
		string m = model_files[i];

		// Object name:
		string name = m;
		size_t pos = name.find_last_of('/');
		if (pos != string::npos)
			name = name.substr(pos+1);
		pos = name.find(".3ds");
		if (pos != string::npos)
			name = name.substr(0, pos);
		std::replace( name.begin(), name.end(), '_', ' ');
		name[0] = toupper(name[0]);
		for (pos = name.find(' '); pos != string::npos && pos < name.length()-1; pos = name.find(' ', pos+1))
		{
			name[pos+1] = toupper(name[pos+1]);
		}


		mStateVec.push_back(State(name));
		mStateVec.back().mModel.push_back(Model3ds_ptr(
			new Model3ds(HIFIDIR+m,Colour::WHITE*0.10,Colour::WHITE*0.4,Colour::WHITE*0.4, 110)));
		mStateVec.back().mModelLowfi.push_back(Mesh_ptr(new Mesh(LOFIDIR+m)));
		mStateVec.back().mPose.push_back(
			Matrix4::translation(Vector3(0,0,0))
				*Matrix4::rotation(Vector3(0,0,0).normalized(), 0));
		mStateVec.back().mDirection.push_back(Vector3(0,0,0));
	}

//	mStateVec.push_back(State("GMcardoor"));
//	//mStateVec.back().mWarning = "";
//	mStateVec.back().mModel.push_back(Model3ds_ptr(
//		new Model3ds(HIFIDIR+"cardoors/GMcardoor_manifold_noholes_rotated_reduced.3ds",Colour::WHITE*0.1,Colour::WHITE*0.2)));
//	mStateVec.back().mModelLowfi.push_back(Mesh_ptr(new Mesh(LOFIDIR+"cardoors/GMcardoor_manifold_noholes_rotated_reduced.3ds")));
//	mStateVec.back().mPose.push_back(
//		Matrix4::translation(Vector3(0.0f, 0.0f, BASEPLATE_OFFSET_Z))
//			*Matrix4::rotation(Vector3(0.0f, 0.0f, 1.0f).normalized(), 0.0f));
//	mStateVec.back().mDirection.push_back(Vector3(0.0,0.0,100.0));


//	mStateVec.push_back(State("Ronzoni"));
//	//mStateVec.back().mWarning = "";
//	mStateVec.back().mModel.push_back(Model3ds_ptr(
//		new Model3ds(HIFIDIR+"ronzoni.3ds",Colour::WHITE*0.1,Colour::WHITE*0.2)));
//	mStateVec.back().mModelLowfi.push_back(Mesh_ptr(new Mesh(LOFIDIR+"ronzoni.3ds")));
//	mStateVec.back().mPose.push_back(
//		Matrix4::translation(Vector3(0.0f, 0.0f, BASEPLATE_OFFSET_Z))
//			*Matrix4::rotation(Vector3(0.0f, 0.0f, 1.0f).normalized(), 0.0f));
//	mStateVec.back().mDirection.push_back(Vector3(0.0,0.0,100.0));

//	mStateVec.push_back(State("Main chamber"));
//	//mStateVec.back().mWarning = "Be careful not to turn it 90 degress wrong.";
//	mStateVec.back().mModel.push_back(Model3ds_ptr(
//		new Model3ds(HIFIDIR+"main_chamber.3ds",
//			Colour::WHITE*0.3,Colour::WHITE*0.6,Colour::WHITE*0.6,40.0)));
//	mStateVec.back().mModelLowfi.push_back(Mesh_ptr(new Mesh(LOFIDIR+"ronzoni.3ds")));
//	mStateVec.back().mPose.push_back(
//		Matrix4::translation(Vector3(0.0f, 0.0f, 50.0f))//før: 35
//			*Matrix4::rotation(Vector3(0.0f, 0.0f, 1.0f).normalized(), 90.0f));
//	mStateVec.back().mDirection.push_back(Vector3(0.0,0.0,100.0));


	// Set number of states equal to number of loaded states:
	mNoStates = mStateVec.size();

	//Error control
	#ifndef NDEBUG
		for(unsigned int i=0; i<mNoStates;++i)
		{
			if(mStateVec[i].mPose.size()!=mStateVec[i].mNoParts)
			{
				cerr << "ERROR1 in State.cpp: mStateVec["<<i
					  <<"].mPose.size!=mStateVec[i].mNoParts"<<endl;
				exit(1);
			}
			if(mStateVec[i].mModel.size()!=mStateVec[i].mNoParts)
			{
				cerr << "ERROR2 in State.cpp: mStateVec["<<i
					  <<"].mModel.size!=mStateVec[i].mNoParts"<<endl;
				exit(1);
			}
			if(mStateVec[i].mDirection.size()!=mStateVec[i].mNoParts)
			{
				cerr << "ERROR4 in State.cpp: mStateVec["<<i
					  <<"].mDirection.size!=mStateVec[i].mNoParts"<<endl;
				exit(1);
			}
		}
	#endif //NDEBUG
}

StateManager::~StateManager() {}

void StateManager::setStateNo(int value)
{
	if (value < 0)
	{
		mStateNo = 0;
	} else if (value >= int(mNoStates) )
	{
		mStateNo = mNoStates-1;
	} else
	{
		mStateNo = value;
	}
}
