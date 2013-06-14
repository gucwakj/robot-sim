#ifndef CRobotSim_H_
#define CRobotSim_H_

#include <iostream>
#include "base.h"
#include "mobotsim.h"
#include "linkbotsim.h"

#ifdef _CH_
//#pragma package <chrobotsim>
//#define dDOUBLE
//#define dReal double
//#define ENABLE_GRAPHICS
#else
#include "config.h"
#include <tinyxml2.h>
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
#endif // _CH_

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

#ifndef _CH_
class CRobot;
#endif // not _CH_

class DLLIMPORT CRobotSim {
	public:
		CRobotSim();
		~CRobotSim();

		int getNumberOfRobots(int type);

		// set simulation variables
		void setCOR(dReal cor_g, dReal cor_b);
		int setExitState(int state);
		void setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundSphere(dReal r, dReal px, dReal py, dReal pz);
		void setMu(dReal mu_g, dReal mu_b);

#ifndef _CH_
		int addRobot(CRobot &robot);
#else
		//int addRobot(...);
		int addMobot(CMobot &robot);
		int addLinkbotI(CLinkbotI &robot);
		int addLinkbotL(CLinkbotL &robot);
#endif // not _CH_
#ifndef _CH_
	private:
		// private variables to store general information about simulation
		dWorldID _world;					// world in which simulation occurs
		dSpaceID _space;					// space for robots in which to live
		dJointGroupID _group;				// group to store joints
		dGeomID* _ground;					// ground (static) objects
		CRobot** _robot[NUM_TYPES];			// array of all robots of every type
		bot_t bot;
		dReal _step;						// time of each step of simulation
		dReal _clock;						// clock time of simulation
		dReal _mu[2];						// coefficient of friction [body/ground, body/body]
		dReal _cor[2];						// coefficient of restitution [body/ground, body/body]
		int _groundNumber;					// number of ground objects
		int _robotNumber[NUM_TYPES];		// number of each robot type
		int _robotConnected[NUM_TYPES];		// number of each robot type
		int _connected[NUM_TYPES];			// number connected of each robot type
		int _running;						// is the program running
		MUTEX_T _ground_mutex;				// mutex for ground collisions
		MUTEX_T _robot_mutex;				// mutex for ground collisions
		MUTEX_T _running_mutex;				// mutex for actively running program
		COND_T _running_cond;				// condition for actively running program
		THREAD_T _simulation;				// simulation thread
		THREAD_T* _robotThread[NUM_TYPES];	// thread for each robot

		// private functions
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
#ifndef _WIN32
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
#endif
		int init_ode(void);
		int init_sim(void);
		int init_xml(void);

#ifdef ENABLE_GRAPHICS
		// variables
		//ViewerFrameThread *_osgThread;		// osg thread
		osgViewer::Viewer *viewer;
		THREAD_T _osgThread;
		osg::Group *_osgRoot;				// osg root node
		// functions
		int init_viz(void);
		static void* graphicsThread(void *arg);
		//osg::TextureCubeMap* readCubeMap(void);
		//osg::Geometry* createWall(const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3,osg::StateSet* stateset);
		//osg::Node* createRoom(void);
#endif // ENABLE_GRAPHICS
#else
	public:
		static void *g_chrobotsim_dlhandle;
		static int g_chrobotsim_dlcount;
#endif // not _CH_
};

#ifdef _CH_
void* CRobotSim::g_chrobotsim_dlhandle = NULL;
int CRobotSim::g_chrobotsim_dlcount = 0;
#pragma importf "chrobotsim.chf"
#endif

#endif	/* CROBOTSIM_H_ */