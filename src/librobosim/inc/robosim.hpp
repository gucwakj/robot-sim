#ifndef ROBOSIM_HPP_
#define ROBOSIM_HPP_

#include <iostream>
#include <tinyxml2.h>

#include "config.h"
#include "robot.hpp"
#include "modularrobot.hpp"
#ifdef ENABLE_GRAPHICS
#include "graphics.hpp"
#endif // ENABLE_GRAPHICS

// ground objects
struct Ground {
	dBodyID body;
	dGeomID geom;
	double r, g, b, alpha;
};

class DLLIMPORT RoboSim {
	// public api
	public:
		RoboSim(char*, int);
		virtual ~RoboSim(void);

		int addRobot(Robot*);
		int addRobot(ModularRobot*);
		int deleteRobot(int);
		double getClock(void);
		int getCOR(double&, double&);
		int getMu(double&, double&);
		int getPause(void);
		double getStep(void);
		int getUnits(void);
		int runSimulation(void);
		int setCollisions(int);
		int setCOR(double, double);
		int setMu(double, double);
		int setPause(int);

	// public data
	public:
#ifdef ENABLE_GRAPHICS
		std::string _tex_path;
#endif // ENABLE_GRAPHICS

	// private functions
	private:
		int init_ode(void);								// init function for ode variables
		int init_sim(int);								// init function for simulation variables
		int init_xml(char*);							// init function to read xml config file
		static void collision(void*, dGeomID, dGeomID);	// wrapper function for nearCallback to work in class
		static void* simulation_thread(void*);			// simulation thread function
#ifdef ENABLE_GRAPHICS
		int init_viz(void);								// visualization initialization function
		static void* graphics_thread(void*);			// thread for graphics objects
#endif // ENABLE_GRAPHICS

	// private data
	private:
		// robots
		struct Robots {
			Robot *robot;
			int node;
			THREAD_T thread;
		};
#ifdef ENABLE_GRAPHICS
		// enumeration of drawing objects
		typedef enum drawing_objects_e {
			DOT,
			LINE,
			TEXT,
			NUM_TYPES
		} drawingObjects_t;
		// graphics objects
		struct Drawing {
			double p1[3], p2[3], c[4];
			int i, type;
			std::string str;
		};
#endif // ENABLE_GRAPHICS

		dWorldID _world;				// world in which simulation occurs
		dSpaceID _space;				// space for robots in which to live
		dJointGroupID _group;			// group to store joints
		std::vector<Ground*> _ground;	// ground (static) objects
		std::vector<Robots*> _robots;	// all robots in simulation
		xml_robot_t _bot;				// robots read from config file
		bool _collision;				// check to perform collisions
		double _clock;					// clock time of simulation
		double _cor[2];					// coefficient of restitution [body/ground, body/body]
		double _mu[2];					// coefficient of friction [body/ground, body/body]
		double _step;					// time of each step of simulation
		int _pause;						// is the simulation paused
		int _preconfig;					// preconfigured robot shape or not
		int _rt;						// whether to run at real time speeds
		int _running;					// is the program running
		int _us;						// us customary units
		COND_T _running_cond;			// condition for actively running program
		MUTEX_T _clock_mutex;			// mutex for getting the clock
		MUTEX_T _pause_mutex;			// mutex for paused simulation
		MUTEX_T _robot_mutex;			// mutex for ground collisions
		MUTEX_T _running_mutex;			// mutex for actively running program
		MUTEX_T _step_mutex;			// mutex for getting the step value
		THREAD_T _simulation;			// simulation thread
#ifdef ENABLE_GRAPHICS
		std::vector<Drawing*> _drawings;	// all graphics objects
		double _grid[7];					// grid spacing (tics, major, total)
		int _ending;						// temp variable for deleting robots
		int _graphics;						// flag for graphics
		int _viewer;						// flag for viewer
		osgShadow::ShadowedScene *_shadowed;// root node to hold graphics
		osg::Group *_staging;				// temp variable for adding robots
		COND_T _graphics_cond;				// condition for graphics
		MUTEX_T _graphics_mutex;			// mutex for graphics existence
		MUTEX_T _viewer_mutex;				// mutex for viewer running state
		THREAD_T _osgThread;				// thread to hold graphics
#endif // ENABLE_GRAPHICS
};

#endif	// ROBOSIM_HPP_

