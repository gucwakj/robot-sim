#ifndef CMOBOTFD_H_
#define CMOBOTFD_H_

#include <iostream>
#include "mobot.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class IRSE {
	public:
		IRSE(void);
		~IRSE(void);

		// set simulation variables
        void setCOR(dReal cor_g, dReal cor_b);
        void setMu(dReal mu_g, dReal mu_b);
        void setNumStatics(int num_statics);
        void setStaticBox(int num, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCapsule(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCylinder(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz);

		// build models of the iMobot
		void addiMobot(iMobotSim &imobot);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2);
		void addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addiMobotAnchored(iMobotSim &imobot, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		void addMobot(mobotSim &mobot);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2);
		void addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void addMobotAnchored(mobotSim &mobot, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
	private:
		// private variables to store general information about simulation
		dWorldID _world;			// world in which simulation occurs
		dSpaceID _space;			// space for robots in which to live
		dJointGroupID _group;		// group to store joints
		dGeomID _ground;			// ground plane
		dGeomID *m_statics;			// array of ground objects
		robotSim** _robot[NUM_TYPES];	// array of all robots of every type
		pthread_mutex_t _robot_mutex;	// mutex to lock robot data
		int _robotNumber[NUM_TYPES];	// number of each robot type
		pthread_t* _robotThread[NUM_TYPES];	// thread for each robot
		dReal _step;				// time of each step of simulation
		dReal _mu_g;				// coefficient of friction of body_ground
		dReal _mu_b;				// coefficient of friction of body_body
		dReal _cor_g;				// coefficient of restitution of body_ground
		dReal _cor_b;				// coefficient of restitution of body_body
		int	m_num_statics;			// number of pieces of ground
		pthread_t _simulation;		// simulation thread

		// simulation functions
		void print_intermediate_data(void);			// print data out at each time step for analysis
		static void* simulationThread(void *arg);
		static void collision(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

        // utility functions
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
};

#endif	/* CMOBOTFD_H_ */
