
#include "Parameters.hpp"

// set default parameter values
void set_default(Parameters* param){

 	param->SOLVER = 1;		// select solver
	param->DEBUG = 0;		// enable debug mode

	param->TIME_STEP = 1./1000.;	// set time step
	param->NUM_STEP_INT = 10;	// set internal time step
	param->TIME_STOP = 10.;		// set simulation time
   
	param->SAVE_VIDEO = 0;

	param->file_video = "../output/video_test.mp4";

}

