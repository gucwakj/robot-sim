#ifndef CMOBOTFD_H_
#define CMOBOTFD_H_

#include <iostream>
#include "mobot.h"
#include "graphics.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class CMobotFD {
	public:
        CMobotFD(void);
		~CMobotFD(void);

		// set simulation variables
		void setCOR(dReal cor_g, dReal cor_b);
		void setMu(dReal mu_g, dReal mu_b);
		void setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void setGroundSphere(dReal r, dReal px, dReal py, dReal pz);

		// build models of the iMobot
		void addiMobot(iMobotSim &imobot);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		void addMobot(mobotSim &mobot);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
	private:
		// private variables to store general information about simulation
		dWorldID world;						// world in which simulation occurs
		dSpaceID space;						// space for robots in which to live
		dJointGroupID group;				// group to store joints
		dGeomID *ground;					// ground (static) objects
		pthread_mutex_t ground_mutex;		// mutex for ground collisions
		int groundNumber;					// number of ground objects
		robotSim **robot[NUM_TYPES];		// array of robots
		pthread_mutex_t robot_mutex;		// mutex for robots
		int robotNumber[NUM_TYPES];			// number of robots
		pthread_t *robotThread[NUM_TYPES];	// thread for each robot

		dReal   m_t_step,			// time of each step of simulation
				m_mu_g,				//coefficient of friction of body_ground
				m_mu_b,				// coefficient of friction of body_body
				m_cor_g,			// coefficient of restitution of body_ground
				m_cor_b;			// coefficient of restitution of body_body
		pthread_t simulation;		// simulation thread
		osg::ref_ptr<osgViewer::Viewer> viewer;

		// simulation functions
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

        // utility functions
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
};

#endif	/* CMOBOTFD_H_ */
