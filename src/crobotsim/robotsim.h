#ifndef CROBOTSIM_H_
#define CROBOTSIM_H_

#include "base.h"

#ifndef _CH_
#include "config.h"
#include <iostream>
#include <tinyxml2.h>
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
#endif // not _CH_

class DLLIMPORT CRobotSim {
	public:
		CRobotSim();
		virtual ~CRobotSim();

#ifndef _CH_
		int addRobot(CRobot *robot);
		int setExitState(void);
	private:
		// ground struct
		typedef struct ground_s {
			dGeomID object;
			struct ground_s *next;
		} *ground_t;

		// private variables to store general information about simulation
		dWorldID _world;					// world in which simulation occurs
		dSpaceID _space;					// space for robots in which to live
		dJointGroupID _group;				// group to store joints
		ground_t _ground;					// ground (static) objects
		CRobot** _robot[NUM_TYPES];			// array of all robots of every type
		bot_t bot;
		dReal _step;						// time of each step of simulation
		dReal _clock;						// clock time of simulation
		double _mu[2];						// coefficient of friction [body/ground, body/body]
		double _cor[2];						// coefficient of restitution [body/ground, body/body]
		int _robotNumber[NUM_TYPES];		// number of each robot type
		int _robotConnected[NUM_TYPES];		// number of each robot type
		int _connected[NUM_TYPES];			// number connected of each robot type
		int _running;						// is the program running
		int _pause;							// is the simulation paused
		MUTEX_T _robot_mutex;				// mutex for ground collisions
		MUTEX_T _running_mutex;				// mutex for actively running program
		MUTEX_T _pause_mutex;				// mutex for paused simulation
		COND_T _running_cond;				// condition for actively running program
		THREAD_T _simulation;				// simulation thread
		THREAD_T* _robotThread[NUM_TYPES];	// thread for each robot

		// private functions
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		int init_ode(void);
		int init_sim(void);
		int init_xml(void);

#ifdef ENABLE_GRAPHICS
		// variables
		osgViewer::Viewer *_viewer;	// viewer class holds all objects
		THREAD_T _osgThread;		// thread to hold graphics
		osg::Group *_osgRoot;		// osg root node
		int _graphics;
		MUTEX_T _graphics_mutex;
		COND_T _graphics_cond;
		// functions
		int init_viz(void);
		static void* graphicsThread(void *arg);
#endif // ENABLE_GRAPHICS
#else
	public:
		static void *g_chrobotsim_dlhandle;
		static int g_chrobotsim_dlcount;
#endif // not _CH_
};

#ifndef _CH_
DLLIMPORT void delay(double seconds);
#endif

#endif	// CROBOTSIM_H_
