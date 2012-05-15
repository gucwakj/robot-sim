#ifndef CMOBOTFD_H_
#define CMOBOTFD_H_

#include <iostream>
#include "robot.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

class CMobotFD {
	public:
        CMobotFD(int bot_type, int num_bot, int num_stp);
		~CMobotFD(void);

		// set simulation variables
        void setAngles(dReal *ang);
        void setAngularVelocity(dReal *vel);
        void setCOR(dReal cor_g, dReal cor_b);
        void setMu(dReal mu_g, dReal mu_b);
        void setNumStatics(int num_statics);
        void setNumTargets(int num_targets);
        void setStaticBox(int num, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCapsule(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticCylinder(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
        void setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz);
        void setTime(dReal time_total);
        void setTarget(int num, dReal x, dReal y, dReal z);

		// build models of iMobot
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z);
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void iMobotBuildAttached(int botNum, int attNum, int face1, int face2);
		void iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		// run the simulation
		void runSimulation(int argc, char **argv);

		// return message on completion
		int getReplyMessage(void);
		double getReplyTime(void);
	private:
		typedef struct cimobotfdreply_s {			// information on success to reply
			dReal time;								// time to successful completion
			int message;							// reply message code
		} CMobotFDReply;
        typedef struct cimobotfdtarget_s {
            dReal x, y, z;
            dGeomID geomID;
        } CMobotFDTarget;

		// private variables to store general information about simulation
		dWorldID world;								// world in which simulation occurs
		dSpaceID space;								// space for robots in which to live
		dJointGroupID group;                        // group to store joints
		dGeomID ground;                             // ground plane
		dGeomID *m_statics;                         // array of ground objects
		Robot **bot;                                // array of robots in simulation
		CMobotFDReply *m_reply;					// struct of data to return after finishing simulation
		CMobotFDTarget *m_targets;                 // array of targets
		dReal   m_t_step,							// time of each step of simulation
				m_mu_g,								// coefficient of friction of body_ground
				m_mu_b,								// coefficient of friction of body_body
				m_cor_g,							// coefficient of restitution of body_ground
				m_cor_b;							// coefficient of restitution of body_body
		int		m_num_bot,							// number of modules in simulation
                m_num_stp,							// total number of steps
                m_num_statics,                      // number of pieces of ground
                m_num_targets,                      // total number of targets
				m_cur_stp,							// current step number
				m_t_tot_step,						// total number of time steps
				m_t_cur_step;						// current time step
		bool	*m_flag_comp,						// flag for each bot - completed step
				*m_flag_disable;					// flag for each bot - disabled/enabled
		#ifdef ENABLE_DRAWSTUFF
		dsFunctions m_fn;							// struct to store drawstuff functions
		#endif

		// simulation functions
		#ifdef ENABLE_DRAWSTUFF
		void ds_start(void);						// initialization of drawstuff scene
        void ds_drawBodies(void);                   // draw all of the bodies
        void ds_drawStatics(void);                  // draw ground objects
        void ds_drawTargets(void);                  // draw sphere at each target
		void ds_command(int cmd);					// keyboard commands for ds
		void ds_simulationLoop(int pause);			// callback function for simulation
		#else
		void simulation_loop(void);					// loop to complete simulation
		#endif
		void update_angles(void);					// update struct with modified angles
		void print_intermediate_data(void);			// print data out at each time step for analysis
		void set_flags(void);						// set flags for complete/not-complete
		void increment_step(void);					// increment step to next set of angles
		void set_angles(void);						// set new angles for step of sim
		void end_simulation(bool &loop);			// check if simulation is complete and exit
		void collision(dGeomID o1, dGeomID o2);		// callback function for contact of bodies
		static void collision_wrapper(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

        // utility functions
		bool is_true(int length, bool *a);	// check if all values in array are true
};

#endif	/* CMOBOTFD_H_ */
