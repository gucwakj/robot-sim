#ifndef INPUT_H_
#define INPUT_H_

// physical constants
#define NUM_ROBOTS	1	// number of robots to simulate
#define NUM_STEPS	4	// number of steps to simulate
#define MU_GROUND	0.3	// coefficient of friction (0-1) between robot and ground
#define COR_GROUND	0.45	// coefficient of restitution (0-1) between robot and ground
#define TIME_TOTAL	10.0	// total simulation time before exiting if nothing is completed

// camera stand
/*
dReal ang[NUM_STEPS][4*NUM_ROBOTS] = {0,	0,	-95.5293,	0,
					0,	0,	-95.5293,	45,
					0,	-85,	123456789,	45,
					0,	-90,	10,		45,
					0,	-90,	10,		90};
dReal vel[NUM_STEPS][4*NUM_ROBOTS] = {	1,	1,	1,	1,
					1,	1,	1,	1,
					1,	1,	1,	1,
					1,	1,	1,	1,
					1,	1,	1,	1};
*/

// disabled motor test
/*
dReal ang[NUM_STEPS][4*NUM_ROBOTS] = {	0,	0,	123456789,	45};
dReal vel[NUM_STEPS][4*NUM_ROBOTS] = {	1,	1,	1,	1};
*/

// friction test
dReal ang[NUM_STEPS][4*NUM_ROBOTS] = {	0,	0,	25,	0,
					0,	0,	45,	0,
					0,	0,	55,	0,
					0,	0,	85,	0};
dReal vel[NUM_STEPS][4*NUM_ROBOTS] = {	1,	1,	1,	1,
					1,	1,	1,	1,
					1,	1,	1,	1,
					1,	1,	1,	1};

// dog rolling
/*
dReal ang[NUM_STEPS][4*NUM_ROBOTS] = {	
	0,	0,	0,	0,	0,	0,	-30,	0,	0,	0,	-30,	0,	0,	0,	-30,	0,	0,	0,	-30,	0,
	0,	0,	0,	0,	0,	30,	-30,	0,	0,	30,	-30,	0,	0,	30,	-30,	0,	0,	30,	-30,	0,
	0,	0,	0,	0,	-360,	30,	-30,	0,	360,	30,	-30,	0,	-360,	30,	-30,	0,	360,	30,	-30,	0
};
dReal vel[NUM_STEPS][4*NUM_ROBOTS] = {
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1
};
*/

//#include "world_geom3.h"
// arrays of world_mesh data
//static float world_vertices[];
//static float world_normals[];
//static dTriIndex world_indices[];

#endif

