#ifndef IRSE_H_
#define IRSE_H_

#include <iostream>
#include <stdbool.h>
#include "robot.h"
#include "mobot.h"
#include "graphics.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class robotSim;
class robot4Sim;
class mobotSim;
class iMobotSim;

class IRSE {
	public:
		IRSE(void);
		~IRSE(void);

		int getNumberOfRobots(int type);

		// set simulation variables
		void setCOR(dReal cor_g, dReal cor_b);
		void setMu(dReal mu_g, dReal mu_b);
		void setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundSphere(dReal r, dReal px, dReal py, dReal pz);

		void simAddRobot(dWorldID &world, dSpaceID &space, dReal **clock);

		// build models of the iMobot
		void addiMobot(robot4Sim *robot);
		void addiMobot(robot4Sim *robot, dReal x, dReal y, dReal z);
		void addiMobot(robot4Sim *robot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addiMobot(robot4Sim *robot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2);
		//void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addiMobotAnchored(iMobotSim &imobot, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		void addMobot(robot4Sim *robot);
		void addMobot(robot4Sim *robot, dReal x, dReal y, dReal z);
		void addMobot(robot4Sim *robot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addMobot(robot4Sim *robot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2);
		//void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addMobotAnchored(mobotSim &mobot, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
	private:
		// private variables to store general information about simulation
		dWorldID _world;					// world in which simulation occurs
		dSpaceID _space;					// space for robots in which to live
		dJointGroupID _group;				// group to store joints
		dGeomID* _ground;					// ground (static) objects
		robotSim** _robot[NUM_TYPES];		// array of all robots of every type
		dReal _step;						// time of each step of simulation
		dReal _clock;						// clock time of simulation
		dReal _mu[2];						// coefficient of friction [body/ground, body/body]
		dReal _cor[2];						// coefficient of restitution [body/ground, body/body]
		int _groundNumber;					// number of ground objects
		int _robotNumber[NUM_TYPES];		// number of each robot type
		pthread_mutex_t _ground_mutex;		// mutex for ground collisions
		pthread_mutex_t _robot_mutex;		// mutex to lock robot data
		pthread_t _simulation;				// simulation thread
		pthread_t* _robotThread[NUM_TYPES];	// thread for each robot
		ViewerFrameThread *_osgThread;		// osg thread
		osg::Group *_osgRoot;				// osg root node

		int graphics_init(void);
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
};

#endif	/* IRSE_H_ */
