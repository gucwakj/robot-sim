#ifndef CIMOBOTSIM_H_
#define CIMOBOTSIM_H_

#include <iostream>
#include "robot.h"

enum simulation_reply_message_e {
	FD_SUCCESS,
	FD_ERROR_TIME,
	FD_ERROR_STALL
};

/*
 *	simulation class
 */
class CiMobotFD {
	public:
		/*
		 *	constructor and destructor
		 */
		CiMobotFD(int num_bot, int num_stp, int num_gnd, int num_targets);
		~CiMobotFD(void);

		/*
		 *	functions to input simulation variables
		 */
        void setAngles(dReal *ang);
		void setAngularVelocity(dReal *vel);
		void setMu(dReal mu_g, dReal mu_b);
		void setCOR(dReal cor_g, dReal cor_b);
        void setTime(dReal time_total);
        void setTarget(int num, double x, double y, double z);

		/*
		 *	functions to build models of iMobot
		 */
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z);
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void iMobotBuildAttached(int botNum, int attNum, int face1, int face2);
		//void iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        void iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		/*
		 *	build ground out of simple objects
		 */
		void groundBox(int gndNum, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void groundCapsule(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void groundCylinder(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z);
		void groundPlane(int gndNum, dReal a, dReal b, dReal c, dReal d);
		void groundSphere(int gndNum, dReal r, dReal px, dReal py, dReal pz);

		/*
		 * 	run the simulation
		 */
		void runSimulation(int argc, char **argv);

		/*
		 *	return a message on the success of the simulation
		 */
		int getReplyMessage();
		dReal getReplyTime();

	private:
		typedef struct cimobotfdreply_s {			// information on success to reply
			dReal time;								// time to successful completion
			int message;							// reply message code
		} CiMobotFDReply;
        typedef struct cimobotfdtarget_s {
            dReal x, y, z;
            dGeomID geomID;
        } CiMobotFDTarget;

		/*
		 *	private variables to store general information about simulation
		 */
		dWorldID world;								// world in which simulation occurs
		dSpaceID space;								// space for robots in which to live
		dJointGroupID group;						// group to store joints
		Robot **bot;
		CiMobotFDReply *m_reply;					// struct of data to return after finishing simulation
		CiMobotFDTarget *target;                    // array of targets
		dGeomID *m_ground;							// array of ground objects
		dReal	m_t,								// current time of simulation
				m_t_total,							// total time for simulation to run
				m_t_step,							// time of each step of simulation
				m_mu_g,								// coefficient of friction of body_ground
				m_mu_b,								// coefficient of friction of body_body
				m_cor_g,							// coefficient of restitution of body_ground
				m_cor_b;							// coefficient of restitution of body_body
		int		m_num_bot,							// number of modules in simulation
				m_num_gnd,							// number of pieces of ground
				m_num_stp,							// total number of steps
                m_num_targets,                      // total number of targets
				m_cur_stp,							// current step number
				m_t_tot_step,						// total number of time steps
				m_t_cur_step;						// current time step
		bool	*m_flag_comp,						// flag for each bot - completed step
				*m_flag_disable;					// flag for each bot - disabled/enabled
		#ifdef ENABLE_DRAWSTUFF
		dsFunctions m_fn;							// struct to store drawstuff functions
		#endif

		/*
		 *	simulation functions
		 */
		#ifdef ENABLE_DRAWSTUFF
		void ds_drawBodies(void);					// draw all of the bodies
		void ds_start(void);						// initialization of drawstuff scene
        void ds_drawTargets(void);                  // draw sphere at each target
		void ds_command(int cmd);					// keyboard commands for ds
		void ds_simulationLoop(int pause);			// callback function for simulation
		#else
		void simulation_loop();						// loop to complete simulation
		#endif
		void update_angles();						// update struct with modified angles
		void print_intermediate_data();				// print data out at each time step for analysis
		void set_flags();							// set flags for complete/not-complete
		void increment_step();						// increment step to next set of angles
		void set_angles();							// set new angles for step of sim
		void end_simulation(bool &loop);			// check if simulation is complete and exit
		void increment_time();						// update simulation time
		void collision(dGeomID o1, dGeomID o2);		// callback function for contact of bodies
		static void collision_wrapper(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

		/*
		 *	functions to build attached imobots
		 */
		//void imobot_build_attached_00(int botNum, int attNum, int face1, int face2);
		//void imobot_build_attached_10(int botNum, int attNum, int face1, int face2);
		//void imobot_build_attached_01(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		//void imobot_build_attached_11(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		/*
		 *	utility functions
		 */
		inline dReal D2R(dReal x);			// convert degrees to radians
		inline dReal R2D(dReal x);			// convert radians to degrees
		bool is_true(int length, bool *a);	// check if all values in array are true
        void rotation_matrix_from_euler_angles(dMatrix3 R, dReal psi, dReal theta, dReal phi);  // create rotation matrix from euler angles
};

#endif	/* CIMOBOTSIM_H_ */
