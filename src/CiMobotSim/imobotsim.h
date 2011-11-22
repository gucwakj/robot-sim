#ifndef CIMOBOTSIM_H_
#define CIMOBOTSIM_H_

#include <iostream>
#include "config.h"
#include "pid.h"
#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
#include <drawstuff/drawstuff.h>
#define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
#endif

/*
 *	iMobot dimension macros
 */
#ifndef CIMOBOTIK_H_
#define CENTER_LENGTH		0.07303
#define CENTER_WIDTH		0.02540
#define CENTER_HEIGHT		0.06909
#define CENTER_RADIUS		0.03554
#define BODY_LENGTH			0.03785
#define BODY_WIDTH			0.07239
#define BODY_HEIGHT			0.07239
#define BODY_RADIUS			0.03620
#define BODY_INNER_WIDTH	0.02287
#define BODY_END_DEPTH		0.01994
#define BODY_MOUNT_CENTER	0.03792
#define END_WIDTH			0.07239
#define END_HEIGHT			0.07239
#define END_DEPTH			0.00476
#define END_RADIUS			0.01778
#endif

/*
 *	enumerations for parts of iMobot
 */
enum robot_bodies_e {		// each body which has a degree of freedom
	LE,
	LB,
	RB,
	RE,
	NUM_DOF
};
enum robot_pieces_e {		// each body part which is built
	BODY_L,
	BODY_R,
	CENTER,
	ENDCAP_L,
	ENDCAP_R,
	NUM_PARTS
};
enum robot_build_e {		// build or rebuild a part
	BUILD,
	REBUILD
};
enum simulation_reply_message_e {
	SIM_SUCCESS,
	SIM_ERROR_TIME,
	SIM_ERROR_STALL
};

/*
 *	simulation class
 */
class CiMobotSim {
	public:
		/*
		 *	constructor and destructor
		 */
		CiMobotSim(int num_bot, int num_stp, int num_gnd, int num_targets, dReal *ang);
		~CiMobotSim(void);

		/*
		 *	functions to input simulation variables
		 */
		void setAngVel(dReal *vel);
		void setMu(dReal mu_g, dReal mu_b);
		void setCOR(dReal cor_g, dReal cor_b);
        void setTime(dReal time_total);
        void setTarget(int num, double x, double y, double z);

		/*
		 *	functions to build models of iMobot
		 */
		/* default build function without rotation support
			botNum:				number of module to build
			x, y, z:			global position of center of module (in meters)	*/
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z);
		/* rotate module about z-axis
			botNum:				number of module to build
			x, y, z:			global position of center of module (in meters)
			theta, psi, phi:	Euler Angles in aircraft notation (in degrees)	*/
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		/* build a new module with predefined angles for body parts
			botNum:				number of module to build
			x, y, z:			global position of center of module (in meters)
			psi, theta, phi:	Euler Angles in aircraft notation (in degrees)
			le, re:				rotation of endcaps about respective bodies (in degrees)
			lb, rb:				rotation of bodies about center (in degrees)	*/
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		/* build a new module attached to a preexisting module
			botNum:				number of new module to build
			attNum:				number of module which to attach
			face1:				face of attNum module
			face2:				face of new module to attach	*/
		void iMobotBuildAttached(int botNum, int attNum, int face1, int face2);
		/* build a new module attached to a preexisting module
			botNum:				number of new module to build
			attNum:				number of module which to attach
			face1:				face of attNum module
			face2:				face of new module to attach
			le, re:				rotation of endcaps about respective bodies (in degrees)
			lb, rb:				rotation of bodies about center (in degrees)	*/
		void iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
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
		/*
		 *	structs to store simulation data
		 */
		typedef struct cimobotsimpart_s {			// information about each body part
			dBodyID bodyID;							// id of body part
			dGeomID *geomID;						// ids of geoms which make up each body part
			#ifdef ENABLE_DRAWSTUFF
			float color[3];							// rgb color for each body part
			int num_geomID;							// total number of geomID for part
			#endif
		} CiMobotSimPart;
		typedef struct cimobotsimbot_s {			// information about each iMobot module
			CiMobotSimPart *bodyPart;				// body parts
			PID *pid;								// PID control for each joint
			dJointID	*joints,					// joints between body parts
						*motors;					// motors to drive body parts
			dReal	*cur_ang,						// current angle of each body part
					*fut_ang,						// future angle being driven toward
					*jnt_vel,						// desired joint velocity
					*ang,							// array of angles
					*vel,							// array of velocities
					*pos,							// initial position of robot
					*rot,							// initial rotation of robot by three Euler angles
					*ori;							// initial orientation of body parts
		} CiMobotSimBot;
		typedef struct cimobotsimreply_s {			// information on success to reply
			dReal time;								// time to successful completion
			int message;							// reply message code
		} CiMobotSimReply;
        typedef struct cimobotsimtarget_s {
            dReal x, y, z;
            dGeomID geomID;
        } CiMobotSimTarget;

		/*
		 *	private variables to store general information about simulation
		 */
		dWorldID world;								// world in which simulation occurs
		dSpaceID space;								// space for robots in which to live
		dSpaceID *space_bot;						// space for each robot
		dJointGroupID group;						// group to store joints
		CiMobotSimBot **bot;						// array of structs for each bot
		CiMobotSimReply *m_reply;					// struct of data to return after finishing simulation
		CiMobotSimTarget *target;                   // array of targets
		dGeomID *m_ground;							// array of ground objects
		dReal	m_t,								// current time of simulation
				m_t_total,							// total time for simulation to run
				m_t_step,							// time of each step of simulation
				m_mu_g,								// coefficient of friction of body_ground
				m_mu_b,								// coefficient of friction of body_body
				m_cor_g,							// coefficient of restitution of body_ground
				m_cor_b,							// coefficient of restitution of body_body
				m_motor_res,						// motor angle resolution
				*m_joint_vel_max,					// maximum joint velocity possible
				*m_joint_vel_min,					// minimum joint velocity possible
				*m_joint_frc_max;					// maximum force that can be applied to each body part
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
		void ds_drawPart(dGeomID part);				// draw each geometry of each body
		void ds_command(int cmd);					// keyboard commands for ds
		void ds_simulationLoop(int pause);			// callback function for simulation
		#else
		void simulation_loop();						// loop to complete simulation
		void init_angles();							// initialize angles for simulation
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
		 *	functions to build body parts of iMobot
		 */
		void imobot_build_lb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb, int rebuild);
		void imobot_build_rb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb, int rebuild);
		void imobot_build_ce(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild);
		void imobot_build_en(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild);

		/*
		 *	functions to build attached imobots
		 */
		void imobot_build_attached_00(int botNum, int attNum, int face1, int face2);
		void imobot_build_attached_10(int botNum, int attNum, int face1, int face2);
		void imobot_build_attached_01(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void imobot_build_attached_11(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		/*
		 *	utility functions
		 */
		inline dReal D2R(dReal x);			// convert degrees to radians
		inline dReal R2D(dReal x);			// convert radians to degrees
		bool is_true(int length, bool *a);	// check if all values in array are true
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate); // modify angle from ODE for endcaps to count continuously
		void rotation_matrix_from_euler_angles(dMatrix3 R, dReal psi, dReal theta, dReal phi);	// create rotation matrix from euler angles
};

#endif	/* CIMOBOTSIM_H_ */
