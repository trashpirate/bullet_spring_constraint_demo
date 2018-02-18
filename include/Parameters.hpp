
#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#define PI 3.1415927

#define LINK_THETA_MAX PI/4
#define LINK_PHI_MAX PI/4
#define LINK_ZETA_MAX PI/360

#include <string>
#include <vector>

struct Parameters{
    
    int SOLVER;
	int DEBUG;
   
	float TIME_STEP;
	int NUM_STEP_INT;
	float TIME_STOP;
	int SAVE_VIDEO;
    
	std::string file_video;

};

void set_default(Parameters* param);

#endif