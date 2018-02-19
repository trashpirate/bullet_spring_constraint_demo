# App_Simulation is a minimal sample creating, stepping and deleting a Bullet dynamics world
cmake_minimum_required(VERSION 3.7)

#SET(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)

IF(APPLE)
    SET(DYLIB_SUFFIX dylib)
ELSEIF(UNIX)
    SET(DYLIB_SUFFIX so)
ENDIF()

SET(BULLET_SOURCE_DIR ~/bullet3 CACHE STRING "Some other user-specified option")
SET(BULLET_BUILD_DIR ~/bullet3/bullet_build CACHE STRING "Some other user-specified option")
SET(BULLET_ROOT_DIR /usr/local/ CACHE STRING "Bullet install position")


FIND_PACKAGE(OpenGL)


INCLUDE_DIRECTORIES(
        include
        ${BULLET_SOURCE_DIR}/src
        ${BULLET_SOURCE_DIR}/btgui
        ${BULLET_SOURCE_DIR}/examples
        ${BULLET_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
        ${BULLET_SOURCE_DIR}/examples/ThirdPartyLibs
        ${BULLET_SOURCE_DIR}/examples/OpenGLWindow
)


SET(SOURCES
    src/Parameters.cpp
    src/Simulation_utility.cpp
    src/Simulation.cpp
    src/Pendulum.cpp

)

SET(EXT_SOURCES
    ${BULLET_SOURCE_DIR}/build3/bullet.rc
    ${BULLET_SOURCE_DIR}/examples/ExampleBrowser/OpenGLGuiHelper.cpp
    ${BULLET_SOURCE_DIR}/examples/ExampleBrowser/GL_ShapeDrawer.cpp
    ${BULLET_SOURCE_DIR}/examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp
    ${BULLET_SOURCE_DIR}/examples/Utils/b3Clock.cpp
    ${BULLET_SOURCE_DIR}/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
    ${BULLET_SOURCE_DIR}/examples/OpenGLWindow/SimpleOpenGL3App.cpp
    ${BULLET_SOURCE_DIR}/examples/ThirdPartyLibs/stb_image/stb_image.cpp

)

LINK_LIBRARIES(
    ${BULLET_ROOT_DIR}/lib/libBullet3Common.${DYLIB_SUFFIX} 
    ${BULLET_ROOT_DIR}/lib/libBulletDynamics.${DYLIB_SUFFIX} 
    ${BULLET_ROOT_DIR}/lib/libBulletCollision.${DYLIB_SUFFIX}  
    ${BULLET_ROOT_DIR}/lib/libLinearMath.${DYLIB_SUFFIX}  
    ${BULLET_BUILD_DIR}/examples/OpenGLWindow/libOpenGLWindow.${DYLIB_SUFFIX}
)


#################
# Standalone SimulationGui using OpenGL (but not the example browser)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)


#some code to support OpenGL and Glew cross platform
IF (WIN32)
        INCLUDE_DIRECTORIES(
             ${BULLET_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
             #${BULLET_SOURCE_DIR}/examples/OpenGLWindow/SimpleOpenGL3App.cpp
        )
        ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

        ELSE(APPLE)
                INCLUDE_DIRECTORIES(
                        ${BULLET_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
                        ${BULLET_SOURCE_DIR}/examples/OpenGLWindow
                )
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

                LINK_LIBRARIES(  pthread ${DL} )
        ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(SpringConstraintDemo
    ${EXT_SOURCES}
    ${SOURCES}
    ${CMAKE_SOURCE_DIR}/src/main_opengl.cpp
)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(SpringConstraintDemo PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(SpringConstraintDemo PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(SpringConstraintDemo PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


