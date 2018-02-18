
#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "Simulation_utility.hpp"
#include "Pendulum.hpp"
#include "Parameters.hpp"
#include <iostream>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces/CommonParameterInterface.h"

class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);



class Simulation : public CommonRigidBodyBase
{

private: 
	float m_time;
	btVector3 gravity = btVector3(0,0,-9.8);

	btAlignedObjectArray<btVector3> m_objcenter; // store center position calculated from bounding box for all objs, before start trans
    btAlignedObjectArray<btVector3> m_objboundingbox; // store bounding box for all objs, before start trans

	// Pendulum* pendulum1;
	btRigidBody* box1;
	btRigidBody* box2;
	btGeneric6DofSpringConstraint* spring;
	btJointFeedback springFeedback;

public:

	Simulation(struct GUIHelperInterface* helper):CommonRigidBodyBase(helper){}
	virtual ~Simulation(){}
	virtual void initPhysics();
	virtual void exitPhysics();
	virtual void stepSimulation();
	virtual void renderScene();
	

	void resetCamera()
	{
		// xz plane
		// float dist = 0.06*SCALE;
		// float pitch = 0;
		// float yaw = 0;

		// yz plane
		// float dist = 0.06*SCALE;
		// float pitch = 0;
		// float yaw = 90;

		// xy plane
		// float dist = 0.05*SCALE;
		// float pitch = -89;
		// float yaw = 180;

		// other
		float dist = 1.;
		float pitch = -30;
		float yaw = 45;

		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}

	bool exitFlag;
	bool applyTestForce = false;
	bool releaseTestForce = false;
	Parameters* parameters =  new Parameters();

};

#endif //BASIC_DEMO_PHYSICS_SETUP_H
