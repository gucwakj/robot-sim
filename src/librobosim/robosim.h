#ifndef ROBOSIM_H_
#define ROBOSIM_H_

#include "base.h"

#ifndef _CH_
#include "config.h"
#include <iostream>
#include <tinyxml2.h>
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
#endif // not _CH_

class DLLIMPORT RoboSim {
	public:
		RoboSim(int pause);
		virtual ~RoboSim();

#ifndef _CH_
		int addRobot(CRobot *robot);
		int deleteRobot(CRobot *robot);
		int getUnits(void);
		int setExitState(void);
	private:
		// ground struct
		typedef struct ground_s {
			dGeomID object;
			struct ground_s *next;
		} *ground_t;
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
		double _clock;				// clock time of simulation
		double _cor[2];				// coefficient of restitution [body/ground, body/body]
		double _mu[2];				// coefficient of friction [body/ground, body/body]
		double _step;				// time of each step of simulation
		int _pause;					// is the simulation paused
		int _running;				// is the program running
		int _preconfig;				// preconfigured robot shape or not
		COND_T _running_cond;		// condition for actively running program
		MUTEX_T _pause_mutex;		// mutex for paused simulation
		MUTEX_T _robot_mutex;		// mutex for ground collisions
		MUTEX_T _running_mutex;		// mutex for actively running program
		THREAD_T _simulation;		// simulation thread

		// private functions
		int init_ode(void);			// init function for ode variables
		int init_sim(int pause);	// init function for simulation variables
		int init_xml(void);			// init function to read xml config file
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		static void* simulation_thread(void *arg);					// simulation thread function
		void print_intermediate_data(void);							// print data out at each time step for analysis

#ifdef ENABLE_GRAPHICS
		// variables
		double _grid[3];							// grid spacing (tics, major, total)
		int _us;									// us customary units
		int _graphics;								// flag for graphics
		int _viewer;								// flag for viewer
		osgShadow::ShadowedScene *_shadowed;		// root node to hold graphics
		COND_T _graphics_cond;						// condition for graphics
		MUTEX_T _graphics_mutex;					// mutex for graphics existence
		MUTEX_T _viewer_mutex;						// mutex for viewer running state
		THREAD_T _osgThread;						// thread to hold graphics
		// functions
		int init_viz(void);							// visualization initialization function
		static void* graphics_thread(void *arg);	// thread for graphics objects
#endif // ENABLE_GRAPHICS
#else
	public:
		static void *_dlhandle;
		static int _dlcount;
#endif // not _CH_
};

int isEmbeddedCh(void);

#endif	// ROBOSIM_H_
