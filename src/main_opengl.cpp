/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "Simulation.hpp"

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "Utils/b3Clock.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Quaternion.h"
#include <stdio.h>
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include <iostream>
#include <vector>
#include <boost/program_options.hpp>

char* gVideoFileName = 0;
char* gPngFileName = 0;

static b3WheelCallback sOldWheelCB = 0;
static b3ResizeCallback sOldResizeCB = 0;
static b3MouseMoveCallback sOldMouseMoveCB = 0;
static b3MouseButtonCallback sOldMouseButtonCB = 0;
static b3KeyboardCallback sOldKeyboardCB = 0;
//static b3RenderCallback sOldRenderCB = 0;

float gWidth = 1024;
float gHeight = 768;

Simulation*    simulation;
int gSharedMemoryKey=-1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove( float x, float y)
{
	bool handled = false; 
	handled = simulation->mouseMoveCallback(x,y); 	 
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback (x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = simulation->mouseButtonCallback(button, state, x,y); 
	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};

int main(int argc, char** argv) 
{ 
	Parameters* param = new Parameters();
	set_default(param);

  	try 
  	{ 
    /** Define and parse the program options 
     */ 
	    namespace po = boost::program_options; 
	    po::options_description desc("Options");
	    desc.add_options() 
	      ("help,h", "Help screen")
	      ("SOLVER", po::value<int>(&param->SOLVER), "NNCG")
	      ("DEBUG", po::value<int>(&param->DEBUG), "debug on/off")
	      
	      ("TIME_STEP", po::value<float>(&param->TIME_STEP), "time step")
	      ("NUM_STEP_INT", po::value<int>(&param->NUM_STEP_INT), "number of internal steps")
	      ("TIME_STOP", po::value<float>(&param->TIME_STOP), "time stop")
		  
		  ("SAVE_VIDEO", po::value<int>(&param->SAVE_VIDEO), "video on/off")
	    
		  
		  ("file_video", po::value<std::string>(&param->file_video), "output file for simulation video");
	 	

	 	
	    po::variables_map vm; 

	    // for(int i=0;i<param->stiffness_x.size();i++){
    	// 	std::cout << param->stiffness_x[i] << std::endl;
    	// }

	    try { 
		    po::store(po::parse_command_line(argc, argv, desc), vm); // can throw 
		 	po::notify(vm);

		 	if ( vm.count("help")  ) { 
		        std::cout << "Bullet Whisker Simulation" << std::endl 
		                  << desc << std::endl; 
		        return 0; 
		    } 
		   

	    	

		    // update_parameters(param);

			SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Whisker Simulation",1024,768,true);
			
			prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
			prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

			app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
			app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
			
			OpenGLGuiHelper gui(app,false);
			CommonExampleOptions options(&gui);
			

			simulation = SimulationCreateFunc(options);
			simulation->processCommandLineArgs(argc, argv);

			simulation->parameters = param; // save parameters in simulation object

			simulation->initPhysics();
			simulation->resetCamera();

			char fileName[1024];
			int textureWidth = 128;
			int textureHeight = 128;
	
			unsigned char*	image = new unsigned char[textureWidth*textureHeight * 4];
			int textureHandle = app->m_renderer->registerTexture(image, textureWidth, textureHeight);
			
			// int cubeIndex = app->registerCubeShape(1, 1, 1);
	
			// b3Vector3 pos = b3MakeVector3(0, 0, 0);
			// b3Quaternion orn(0, 0, 0, 1);
			// b3Vector3 color = b3MakeVector3(1, 0, 0);
			// b3Vector3 scaling = b3MakeVector3 (1, 1, 1);

			if(param->SAVE_VIDEO){
				std::string videoname = param->file_video;
				gVideoFileName = &videoname[0];
				
				if (gVideoFileName){
					std::cout << "Rendering video..." << std::endl;
					app->dumpFramesToVideo(gVideoFileName);
	
				}

				std::string pngname = "png_test";
				gPngFileName = &pngname[0];
				
				// app->m_renderer->registerGraphicsInstance(cubeIndex, pos, orn, color, scaling);
				app->m_renderer->writeTransforms();


			}
			
			

			do
			{
				if(param->SAVE_VIDEO){
					static int frameCount = 0;
					frameCount++;

					if (gPngFileName)
					{
						// printf("gPngFileName=%s\n", gPngFileName);
		
						sprintf(fileName, "%s%d.png", gPngFileName, frameCount++);
						app->dumpNextFrameToPng(fileName);
					}
		
		
		
					//update the texels of the texture using a simple pattern, animated using frame index
					for (int y = 0; y < textureHeight; ++y)
					{
						const int	t = (y + frameCount) >> 4;
						unsigned char*	pi = image + y*textureWidth * 3;
						for (int x = 0; x < textureWidth; ++x)
						{
							const int		s = x >> 4;
							const unsigned char	b = 180;
							unsigned char			c = b + ((s + (t & 1)) & 1)*(255 - b);
							pi[0] = pi[1] = pi[2] = pi[3] = c; pi += 3;
						}
					}
		
					app->m_renderer->activateTexture(textureHandle);
					app->m_renderer->updateTexture(textureHandle, image);
		
					float color[4] = { 255, 1, 1, 1 };
					app->m_primRenderer->drawTexturedRect(100, 200, gWidth / 2 - 50, gHeight / 2 - 50, color, 0, 0, 1, 1, true);
				}

				app->m_instancingRenderer->init();
		    	app->m_instancingRenderer->updateCamera(app->getUpAxis());
			
				simulation->stepSimulation();

				simulation->renderScene();
				
				app->m_renderer->renderScene();
				DrawGridData dg;
		        dg.upAxis = app->getUpAxis();
				app->drawGrid(dg);
				// char bla[1024];
				// std::sprintf(bla, "Simple test frame %d", frameCount);
				// app->drawText(bla, 10, 10);
				app->swapBuffer();


			} while (!app->m_window->requestedExit() && !(simulation->exitFlag));
			
			std::cout << "Saving data..." << std::endl;
			

			std::cout << "Exit simulation..." << std::endl;
			simulation->exitPhysics();
			delete simulation;
			delete app;
			delete[] image;
			std::cout << "Done." << std::endl;


	    } 
	    catch(po::error& e) 
	    { 
	      std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
	      std::cerr << desc << std::endl; 
	      return 1; 
	    } 
 
    // application code here // 
 
  	} 
  	catch(std::exception& e) 
  	{ 
    	std::cerr << "Unhandled Exception reached the top of main: " 
              << e.what() << ", application will now exit" << std::endl; 
    	return 2; 
 
  	} 
 
  	return 0; 
 
} // main 



// int main(int argc, char* argv[])
// {









	


// 	try
// 	  {
// 	    boost::program_options::options_description desc{"Options"};
// 	    desc.add_options()
// 	      ("help,h", "Help screen")
// 	      ("SOLVER", boost::program_options::value<int>()->default_value(1), "NNCG")
// 	      ("DEBUG", boost::program_options::value<int>()->default_value(0), "debug on/off")
// 	      ("ACTIVE", boost::program_options::value<int>()->default_value(0), "active on/off")
// 	      // ("STIFFNESS", boost::program_options::value<float>()->default_value(20000.), "stiffness value")
// 	      // ("DAMPING", boost::program_options::value<float>()->default_value(250.), "damping value")
// 	      ("TIME_STEP", boost::program_options::value<float>()->default_value(1./250.), "time step")
// 	      ("TIME_STOP", boost::program_options::value<float>()->default_value(1.), "time stop")
// 	      ("NO_CURVATURE", boost::program_options::value<int>()->default_value(0), "whisker curvature on/off")
// 	      ("NUM_UNITS", boost::program_options::value<int>()->default_value(8), "number of units");

// 	    boost::program_options::variables_map vm;
// 	    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
// 	    boost::program_options::notify(vm);

// 	    if (vm.count("help")){
// 	      std::cout << desc << '\n';
// 	    }
// 	    else if (vm.count("SOLVER")){
// 	    	simulation->parameters->SOLVER = vm["SOLVER"].as<int>();
// 	      	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    }
// 	    else if (vm.count("DEBUG")){
// 	    	simulation->parameters->DEBUG = vm["DEBUG"].as<int>();
// 	      	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    }
// 	    else if (vm.count("ACTIVE")){
// 	    	simulation->parameters->ACTIVE = vm["ACTIVE"].as<int>();
// 	      	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    }
	    
// 	    else if (vm.count("TIME_STEP")){
// 	    	simulation->parameters->TIME_STEP = vm["TIME_STEP"].as<float>();
// 	      	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    }
// 	    else if (vm.count("TIME_STOP")){
// 	    	simulation->parameters->TIME_STOP = vm["TIME_STOP"].as<float>();
// 	      	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    }
// 	    // else if (vm.count("NO_CURVATURE")){
// 	    // 	NO_CURVATURE = vm["NO_CURVATURE"].as<int>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("NUM_UNITS")){
// 	    // 	NUM_UNITS = vm["NUM_UNITS"].as<int>();
// 	    // 	NUM_LINKS = NUM_UNITS - 1;
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("DENSITY")){
// 	    // 	DENSITY = vm["DENSITY"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("E_BASE")){
// 	    // 	E_BASE = vm["E_BASE"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("E_TIP")){
// 	    // 	E_TIP = vm["E_TIP"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("ZETA_BASE")){
// 	    // 	ZETA_BASE = vm["ZETA_BASE"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("ZETA_TIP")){
// 	    // 	ZETA_TIP = vm["ZETA_TIP"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("BT_RATIO")){
// 	    // 	BT_RATIO = vm["BT_RATIO"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("STIFFNESS")){
// 	    // 	STIFFNESS = vm["STIFFNESS"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("DAMPING")){
// 	    // 	DAMPING = vm["DAMPING"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("WHISK_AMP")){
// 	    // 	WHISK_AMP = vm["WHISK_AMP"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }
// 	    // else if (vm.count("WHISK_FREQ")){
// 	    // 	WHISK_FREQ = vm["WHISK_FREQ"].as<float>();
// 	    //   	// std::cout << "Solver: " << vm["solver"].as<int>() << '\n';
// 	    // }

	    
// 		  }
// 		catch (const boost::program_options::error &ex){
// 		    std::cerr << ex.what() << '\n';
// 		  }


	
// 	return 0;
// }

