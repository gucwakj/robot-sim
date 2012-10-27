#ifndef CMOBOTFD_H_
#define CMOBOTFD_H_

#include <iostream>
#include "mobot.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

enum robot_types_e {
	IMOBOT,
	MOBOT,
	KIDBOT,
	NXT,
	NUM_TYPES
};

class CMobotFD {
	public:
        CMobotFD(void);
		~CMobotFD(void);

		// set simulation variables
        void setCOR(dReal cor_g, dReal cor_b);
        void setMu(dReal mu_g, dReal mu_b);
        void setNumStatics(int num_statics);
        void setNumTargets(int num_targets);
        void setStaticBox(int num, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCapsule(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCylinder(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz);
        void setTarget(int num, dReal x, dReal y, dReal z);

		// build models of the iMobot
		void addiMobot(CiMobotSim &mobot);
		void addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z);
		void addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addiMobotConnected(CiMobotSim &mobot, CiMobotSim &base, int face1, int face2);
		void addiMobotConnected(CiMobotSim &mobot, CiMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// build models of the Mobot
		void addMobot(CMobotSim &mobot);
		void addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z);
		void addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2);
		void addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// return message on completion
		int getReplyMessage(void);
		double getReplyTime(void);
	private:
        typedef struct cimobotfdtarget_s {
            dReal x, y, z;
            dGeomID geomID;
        } CMobotFDTarget;

		// private variables to store general information about simulation
		dWorldID world;				// world in which simulation occurs
		dSpaceID space;				// space for robots in which to live
		dJointGroupID group;		// group to store joints
		dGeomID ground;				// ground plane
		dGeomID *m_statics;			// array of ground objects
		CiMobotSim **bot;			// array of robots in simulation
		CMobotFDTarget *m_targets;	// array of targets
		dReal   m_t_step,			// time of each step of simulation
				m_mu_g,				//coefficient of friction of body_ground
				m_mu_b,				// coefficient of friction of body_body
				m_cor_g,			// coefficient of restitution of body_ground
				m_cor_b;			// coefficient of restitution of body_body
		int	m_num_statics,			// number of pieces of ground
			m_num_targets;			// total number of targets
		int m_number[NUM_TYPES];	// number of each type of robot in simulation
		pthread_t simulation;		// simulation thread
		pthread_t visualization;	// visualization thread

		// simulation functions
		void simulation_loop(void);		// loop to complete simulation
		void update_angles(void);					// update struct with modified angles
		void print_intermediate_data(void);			// print data out at each time step for analysis
		void check_success(void);					// check if motion is complete
		void collision(dGeomID o1, dGeomID o2);		// callback function for contact of bodies
		static void collision_wrapper(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class
		static void* simulation_wrapper(void *arg);

        // utility functions
		unsigned int diff_nsecs(struct timespec t1, struct timespec t2);
};

#endif	/* CMOBOTFD_H_ */
