

#include "Simulation_utility.hpp"

// Helper Functions for simulation
// ================================


// function to create dynamic body
btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape,GUIHelperInterface* m_guiHelper)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic){
		shape->calculateLocalInertia(mass, localInertia);
	}
	else{
		std::cout << "Warning: body mass is zero!" << std::endl;
	}

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	// body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	body->setUserIndex(-1);
	
	
	return body;
}

// function to create frame
btTransform createFrame(btVector3 origin, btVector3 rotation){
	btTransform frame;
	frame = btTransform::getIdentity();
	frame.setOrigin(origin);
	frame.getBasis().setEulerZYX(rotation[0],rotation[1],rotation[2]);
	return frame;
}

// function to translate frame
void translateFrame(btTransform& transform, btVector3 origin){

	transform.setOrigin(origin);
}

// function to rotate frame with eular angles
void rotateFrame(btTransform& transform, btVector3 rotation){

	btScalar rx = rotation[0];	// roll
	btScalar ry = rotation[1];	// pitch
	btScalar rz = rotation[2];	// yaw

	transform.getBasis().setEulerZYX(rx,ry,rz);
}


