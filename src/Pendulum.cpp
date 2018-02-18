

#include "Pendulum.hpp"



Pendulum::Pendulum(float l, float s, float stiff, float damp, btVector3 pos){
	length = l;
    size = s;
    position = pos;
    stiffness = stiff;
    damping = damp;
	
		
}


void Pendulum::createPendulumSpring(GUIHelperInterface* helper,btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameters* parameters){
	
    std::cout << "Creating pendulum..." << std::endl;
	std::cout << "-length: " << length << std::endl;
	std::cout << "-mass: " << mass << std::endl;
	std::cout << "-stiffness: " << stiffness << std::endl;
	std::cout << "-damping: " << damping << std::endl;

	// create object to collide with
	btCollisionShape* no_shape = new btEmptyShape();
	shapes->push_back(no_shape);

	btTransform fixpointTransform = createFrame(position);
	
	btScalar mass_fixpoint(0);
	btRigidBody* fixpoint = createDynamicBody(mass_fixpoint,fixpointTransform,no_shape,helper);
	
	//create object to collide with

	float s = 0.05; 

	btCollisionShape* pendulum_shape = new btSphereShape(s);
	shapes->push_back(pendulum_shape);

	btTransform pendulumTransform = createFrame(btVector3(0,-length/2,0));
	btScalar pendulum_mass(mass);
	btRigidBody* pendulum = createDynamicBody(pendulum_mass,fixpointTransform*pendulumTransform,pendulum_shape,helper);
    world->addRigidBody(pendulum);
	pendulum->setActivationState(DISABLE_DEACTIVATION);

	// create frames for constraint
	btTransform frameFixpoint;				
	btTransform framePendulum;

	frameFixpoint = createFrame(btVector3(0,0,0));				
	framePendulum = createFrame(btVector3(0,length,0));
			
	// create constraint
	btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*fixpoint, *pendulum, frameFixpoint,framePendulum,true);

	spring->setLinearLowerLimit(btVector3(0., 0, 0.0)); 
	spring->setLinearUpperLimit(btVector3(0., 0, 0.0));

	spring->setAngularLowerLimit(btVector3(0,0,1.)); // allow rotation about z axis
	spring->setAngularUpperLimit(btVector3(0,0.,0.));

	// add constraint to world
	world->addConstraint(spring, true); // true -> collision between linked bodies disabled
	spring->setDbgDrawSize(btScalar(5.f));

	//enable springs at constraints
	spring->enableSpring(5,true);
	spring->setStiffness(5,stiffness);
	spring->setDamping(5,damping);
	spring->setEquilibriumPoint(5,0.);
    
    framePendulum = pendulum->getCenterOfMassTransform();
	btTransform rotation = createFrame(btVector3(0,0,0),btVector3(0,0,PI/4));
	pendulum->setCenterOfMassTransform(rotation*framePendulum);
	
	// generate graphics
	helper->autogenerateGraphicsObjects(world);
	
		
}

void Pendulum::createPendulumSpring2(GUIHelperInterface* helper,btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameters* parameters){
	
    std::cout << "Creating pendulum..." << std::endl;
	std::cout << "-length: " << length << std::endl;
	std::cout << "-mass: " << mass << std::endl;
	std::cout << "-stiffness: " << stiffness << std::endl;
	std::cout << "-damping: " << damping << std::endl;

	// create object to collide with
	btCollisionShape* no_shape = new btEmptyShape();
	shapes->push_back(no_shape);

	btTransform fixpointTransform = createFrame(position);
	
	btScalar mass_fixpoint(0);
	btRigidBody* fixpoint = createDynamicBody(mass_fixpoint,fixpointTransform,no_shape,helper);
	
	//create object to collide with

	float s = 0.05; 

	btCollisionShape* pendulum_shape = new btSphereShape(s);
	shapes->push_back(pendulum_shape);

	btTransform pendulumTransform = createFrame(btVector3(0,-length/2,0));
	btScalar pendulum_mass(mass);
	btRigidBody* pendulum = createDynamicBody(pendulum_mass,fixpointTransform*pendulumTransform,pendulum_shape,helper);
    world->addRigidBody(pendulum);
	pendulum->setActivationState(DISABLE_DEACTIVATION);

	// create frames for constraint
	btTransform frameFixpoint;				
	btTransform framePendulum;

	frameFixpoint = createFrame(btVector3(0,0,0));				
	framePendulum = createFrame(btVector3(0,length,0));
			
	// create constraint
	btGeneric6DofSpring2Constraint* spring = new btGeneric6DofSpring2Constraint(*fixpoint, *pendulum, frameFixpoint,framePendulum);

	spring->setLinearLowerLimit(btVector3(0., 0, 0.0)); 
	spring->setLinearUpperLimit(btVector3(0., 0, 0.0));

	spring->setAngularLowerLimit(btVector3(0,0,1.)); // allow rotation about z axis
	spring->setAngularUpperLimit(btVector3(0,0.,0.));

	// add constraint to world
	world->addConstraint(spring, true); // true -> collision between linked bodies disabled
	spring->setDbgDrawSize(btScalar(5.f));

	//enable springs at constraints
	spring->enableSpring(5,true);
	spring->setStiffness(5,stiffness);
	spring->setDamping(5,damping);
	spring->setEquilibriumPoint(5,0.);
    
    framePendulum = pendulum->getCenterOfMassTransform();
	btTransform rotation = createFrame(btVector3(0,0,0),btVector3(0,0,PI/4));
	pendulum->setCenterOfMassTransform(rotation*framePendulum);
	
	// generate graphics
	helper->autogenerateGraphicsObjects(world);
	
		
}