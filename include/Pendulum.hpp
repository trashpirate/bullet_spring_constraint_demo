
#ifndef PENDULUM_HPP
#define PENDULUM_HPP

#include <vector>
#include <string>
#include "Parameters.hpp"
#include "Simulation_utility.hpp"

class Pendulum
{
private:

    float length;
    float size;
    float stiffness;
    float damping;
    float mass = 1;

    btVector3 position;

	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*>* m_collisionShapes;
	GUIHelperInterface* m_guiHelper;
	
public:

	Pendulum(float l, float s, float stiff, float damp, btVector3 pos);
	~Pendulum(){}

    void createPendulumSpring(GUIHelperInterface* helper, btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameters* parameters);
    void createPendulumSpring2(GUIHelperInterface* helper, btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameters* parameters);

};




#endif //BASIC_DEMO_PHYSICS_SETUP_H