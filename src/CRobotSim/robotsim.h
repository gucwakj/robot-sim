#ifndef CRobotSim_H_
#define CRobotSim_H_

#include <iostream>
#include <stdbool.h>
#include "config.h"
#include "robot.h"
#include "mobotsim.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif /* ENABLE_GRAPHICS */

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class CRobot;
class CRobot4;

class CRobotSim {
	public:
		CRobotSim(void);
		~CRobotSim(void);

		int getNumberOfRobots(int type);

		// set simulation variables
		void setCOR(dReal cor_g, dReal cor_b);
		int setExitState(int state);
		void setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundSphere(dReal r, dReal px, dReal py, dReal pz);
		void setMu(dReal mu_g, dReal mu_b);

		int addRobot(CRobot &robot);

		// build models of the iMobot
		//void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2);
		//void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		//void addMobotConnected(CRobot4 *robot, CRobot4 *base, int face1, int face2);
		//void addMobotConnected(CRobot4 *robot, CRobot4 *base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
	private:
		// connector
		typedef struct Conn_s {
			int robot, face1, face2, type;
			struct Conn_s *next;
		} Conn_t;
		// robot
		typedef struct Bot_s {
			int type;
			int id;
			double x, y, z;
			double psi, theta, phi;
			double angle1, angle2, angle3, angle4;
			struct Conn_s *conn;
			struct Bot_s *next;
		} Bot_t;
		// private variables to store general information about simulation
		dWorldID _world;					// world in which simulation occurs
		dSpaceID _space;					// space for robots in which to live
		dJointGroupID _group;				// group to store joints
		dGeomID* _ground;					// ground (static) objects
		CRobot** _robot[NUM_TYPES];			// array of all robots of every type
		Bot_t *bot;
		dReal _step;						// time of each step of simulation
		dReal _clock;						// clock time of simulation
		dReal _mu[2];						// coefficient of friction [body/ground, body/body]
		dReal _cor[2];						// coefficient of restitution [body/ground, body/body]
		int _groundNumber;					// number of ground objects
		int _robotNumber[NUM_TYPES];		// number of each robot type
		int _robotConnected[NUM_TYPES];		// number of each robot type
		int _connected[NUM_TYPES];			// number connected of each robot type
		pthread_mutex_t _ground_mutex;		// mutex for ground collisions
		pthread_mutex_t _robot_mutex;		// mutex to lock robot data
		pthread_t _simulation;				// simulation thread
		pthread_t* _robotThread[NUM_TYPES];	// thread for each robot
#ifdef ENABLE_GRAPHICS
		ViewerFrameThread *_osgThread;		// osg thread
		osg::Group *_osgRoot;				// osg root node
#endif /* ENABLE_GRAPHICS */

#ifdef ENABLE_GRAPHICS
		int graphics_init(void);
		osg::TextureCubeMap* readCubeMap(void);
		osg::Geometry* createWall(const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3,osg::StateSet* stateset);
		osg::Node* createRoom(void);
#endif /* ENABLE_GRAPHICS */
		void robot_init(void);
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
};

#endif	/* CROBOTSIM_H_ */
