#ifndef ROBOSIM_H_
#define ROBOSIM_H_

#include "base.h"
#include "config.h"
#include <iostream>
#include <tinyxml2.h>
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

// ground struct
typedef struct ground_s {
	dBodyID body;
	dGeomID geom;
	double r, g, b, alpha;
	struct ground_s *next;
} *ground_t;

class DLLIMPORT RoboSim {
	public:
		RoboSim(char *name, int pause);
		virtual ~RoboSim();

		int addRobot(CRobot *robot);
		int deleteRobot(CRobot *robot);
		double getClock(void);
		int getPause(void);
		double getStep(void);
		int getUnits(void);
		int runSimulation(void);
		int setCollisions(int mode);
		int setPause(int mode);
	private:
		// robots struct
		typedef struct robots_s {
			CRobot *robot;
			int node;
			THREAD_T thread;
			struct robots_s *next;
		} *robots_t;

		// private variables to store general information about simulation
		dWorldID _world;			// world in which simulation occurs
		dSpaceID _space;			// space for robots in which to live
		dJointGroupID _group;		// group to store joints
		ground_t _ground;			// ground (static) objects
		robots_t _robots;			// robot data within simulation
		xml_robot_t _bot;			// robots read from config file
		bool _collision;			// check to perform collisions
		double _clock;				// clock time of simulation
		double _cor[2];				// coefficient of restitution [body/ground, body/body]
		double _mu[2];				// coefficient of friction [body/ground, body/body]
		double _step;				// time of each step of simulation
		int _pause;					// is the simulation paused
		int _preconfig;				// preconfigured robot shape or not
		int _rt;					// whether to run at real time speeds
		int _running;				// is the program running
		int _us;					// us customary units
		COND_T _running_cond;		// condition for actively running program
		MUTEX_T _clock_mutex;		// mutex for getting the clock
		MUTEX_T _pause_mutex;		// mutex for paused simulation
		MUTEX_T _robot_mutex;		// mutex for ground collisions
		MUTEX_T _running_mutex;		// mutex for actively running program
		MUTEX_T _step_mutex;		// mutex for getting the step value
		THREAD_T _simulation;		// simulation thread

		// private functions
		int init_ode(void);											// init function for ode variables
		int init_sim(int pause);									// init function for simulation variables
		int init_xml(char *name);									// init function to read xml config file
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		static void* simulation_thread(void *arg);					// simulation thread function

#ifdef ENABLE_GRAPHICS
		// enumeration of drawing objects
		typedef enum drawing_objects_e {
			LINE,
			POINT,
			TEXT,
			NUM_TYPES
		} drawingObjects_t;

		// graphics objects struct
		typedef struct drawing_s {
			double p1[3], p2[3], c[4];
			int i, type;
			std::string str;
			struct drawing_s *next;
		} *drawing_t;

		// variables
		double _grid[6];							// grid spacing (tics, major, total)
		int _ending;								// temp variable for deleting robots
		int _graphics;								// flag for graphics
		int _viewer;								// flag for viewer
		osgShadow::ShadowedScene *_shadowed;		// root node to hold graphics
		osg::Group *_staging;						// temp variable for adding robots
		drawing_t _drawings;						// all graphics objects
		COND_T _graphics_cond;						// condition for graphics
		MUTEX_T _graphics_mutex;					// mutex for graphics existence
		MUTEX_T _viewer_mutex;						// mutex for viewer running state
		THREAD_T _osgThread;						// thread to hold graphics

		// functions
		int init_viz(void);							// visualization initialization function
		static void* graphics_thread(void *arg);	// thread for graphics objects
#endif // ENABLE_GRAPHICS
};

#endif	// ROBOSIM_H_
