#ifndef SIMULATION_UTILITY_HPP
#define SIMULATION_UTILITY_HPP


#define PI 3.1415927

#include <iostream>
#include <vector>

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces/CommonParameterInterface.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
// #include "ImportObjDemo/LoadMeshFromObj.h"
// #include "ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h"
// #include "ImportURDFDemo/BulletUrdfImporter.h"
// #include "RigidBodyFromObj.h"
#include "GLInstanceGraphicsShape.h"

#include "Pendulum.hpp"


btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape, GUIHelperInterface* m_guiHelper);

void translateFrame(btTransform& transform, btVector3 origin=btVector3(0.,0.,0.));
void rotateFrame(btTransform& transform, btVector3 rotation=btVector3(0.,0.,0.));
btTransform createFrame(btVector3 origin=btVector3(0.,0.,0.), btVector3 rotation=btVector3(0.,0.,0.));


#endif