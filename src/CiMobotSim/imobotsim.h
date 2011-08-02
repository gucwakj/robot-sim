#ifndef CIMOBOTSIM_H_
#define CIMOBOTSIM_H_

#include "config.h"
#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
#include <drawstuff/drawstuff.h>
#define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
#endif

/*
 *	iMobot dimension macros
 */
#define CENTER_LENGTH		2.865
#define CENTER_WIDTH		1.05
#define CENTER_HEIGHT		2.60
#define CENTER_RADIUS		1.30
#define BODY_LENGTH			1.55
#define BODY_WIDTH			2.85
#define BODY_HEIGHT			2.85
#define BODY_RADIUS			1.425
#define BODY_INNER_WIDTH	0.875
#define BODY_END_DEPTH		0.2
#define BODY_MOUNT_CENTER	1.28
#define END_WIDTH			2.85
#define END_HEIGHT			2.85
#define END_DEPTH			0.125
#define END_RADIUS			0.85

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

/*
 *	simulation class
 */
class CiMobotSim {
	public:
		/*
		 *	constructor and destructor
		 */
		CiMobotSim(int numBot, int numStp, int numGnd, dReal *ang);
		~CiMobotSim();

		/*
		 *	functions to input simulation variables
		 */
		void setAngVel(dReal *vel);
		void setMu(dReal mu_g, dReal mu_b);
		void setCOR(dReal cor_g, dReal cor_b);
		void setTime(dReal time_total);

		/*
		 *	functions to build models of iMobot
		 */
		/* default build function without rotation support
			botNum:				number of module to build
			x, y, z:			global position of center of module	*/
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z);
		/* rotate module about z-axis
			botNum:				number of module to build
			x, y, z:			global position of center of module
			theta, psi, phi:	Euler Angles in aircraft notation (in degrees)	*/
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		/* build a new module with predefined angles for body parts
			botNum:				number of module to build
			x, y, z:			global position of center of module
			theta, psi, phi:	Euler Angles in aircraft notation (in degrees)
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
		void runSimulation(int argc, char **argv);			// run simulation

		/*
		 *	return a message on the success of the simulation
		 */
		void replyMessage();						// reply message to send back to main program

	private:
		/*
		 *	variables to store simulation properties
		 */
		dWorldID world;								// world in which simulation occurs
		dSpaceID space;								// space for robots in which to live
		dSpaceID *space_bot;						// space for each robot
		dJointGroupID group;						// group to store joints

		/*
		 *	structs to store simulation data
		 */
		typedef struct cimobotsimpart_s {			// information about each body part
			dBodyID bodyID;							// id of body part
			dGeomID *geomID;						// ids of geoms which make up each body part
			dReal ang;								// initial angle of part about rotational axis
			#ifdef ENABLE_DRAWSTUFF
			float color[3];							// rgb color for each body part
			int num_geomID;							// total number of geomID for part
			#endif
		} CiMobotSimPart;
		typedef struct cimobotsimbot_s {			// information about each iMobot module
			CiMobotSimPart *bodyPart;				// body parts
			dJointID *joints;						// joints between body parts
			dJointID *motors;						// motors to drive body parts
			dReal *curAng;							// current angle of each body part
			dReal *futAng;							// future angle being driven toward
			dReal *jntVel;							// desired joint velocity
			dReal *ang;								// array of angles
			dReal *vel;								// array of velocities
			dReal *pos;								// 3D position of center in world
			dReal *rot;								// 3D rotation matrix of robot about center
			bool *cmpStp;							// flag to check if step is complete
		} CiMobotSimBot;
		typedef struct cimobotsimreply_s {			// information on success to reply
			bool success;							// flag if simulation is successful
			double time;							// time to successful completion
			int message;							// reply message code
		} CiMobotSimReply;

		/*
		 *	private variables to store general information about simulation
		 */
		CiMobotSimBot **bot;						// array of structs for each bot
		CiMobotSimReply *m_reply;					// struct of data to return after finishing simulation
		dGeomID *m_ground;							// array of ground objects
		dReal m_t;									// current time of simulation
		dReal m_t_total;							// total time for simulation to run
		dReal m_t_step;								// time of each step of simulation
		dReal m_mu_g;								// coefficient of friction of body_ground
		dReal m_mu_b;								// coefficient of friction of body_body
		dReal m_cor_g;								// coefficient of restitution of body_ground
		dReal m_cor_b;								// coefficient of restitution of body_body
		dReal m_motor_res;							// motor angle resolution
		dReal *m_joint_vel_max;						// maximum joint velocity possible
		dReal *m_joint_vel_min;						// minimum joint velocity possible
		dReal *m_joint_vel_del;						// range of joint velocities
		dReal *m_joint_frc_max;						// maximum force that can be applied to each body part
		int m_num_bot;								// number of modules in simulation
		int m_num_gnd;								// number of pieces of ground
		int m_num_stp;								// total number of steps
		int m_cur_stp;								// current step number
		bool *m_flags;								// flag for each bot  - completed step
		bool *m_disable;							// flag for each bot - disabled/enabled
		#ifdef ENABLE_DRAWSTUFF
		dsFunctions m_fn;							// struct to store drawstuff functions
		#endif

		/*
		 *	simulation functions
		 */
		#ifdef ENABLE_DRAWSTUFF
		void ds_drawBodies();						// draw all of the bodies
		void ds_start();							// initialization of drawstuff scene
		void ds_drawPart(dGeomID part);				// draw each geometry of each body
		void ds_command(int cmd);					// keyboard commands for ds
		void ds_simulation(int pause);				// callback function for simulation
		#else
		void simulationLoop();						// loop to complete simulation
		#endif
		void updateAngles();						// update struct with modified angles
		void setFlags();							// set flags for complete/not-complete
		void incrementStep();						// increment step to next set of angles
		void setAngles();							// set new angles for step of sim
		void printIntermediateData();				// print data out at each time step for analysis
		bool endSimulation(double totalTime);		// check if simulation is complete and exit
		void incrementTime(double tStep);			// update simulation time
		void collision(dGeomID o1, dGeomID o2);		// callback function for contact of bodies
		static void collisionWrapper(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

		/*
		 *	functions to build body parts of iMobot
		 */
		void imobot_build_lb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb, int rebuild);
		void imobot_build_ce(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild);
		void imobot_build_rb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb, int rebuild);
		void imobot_build_en(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_e, int rebuild);

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
		inline dReal I2M(dReal x);			// convert inches to meters
		inline dReal M2I(dReal x);			// convert meters to inches
		inline dReal D2R(dReal x);			// convert degrees to radians
		inline dReal R2D(dReal x);			// convert radians to degrees
		bool isTrue(bool *a, int length);	// check if all values in array are true
		dReal angMod(dReal pasAng, dReal curAng, dReal angRat); // modify angle from ODE for endcaps to count continuously
		void rotMatrixFromEulerAngles(dMatrix3 R, dReal psi, dReal theta, dReal phi);	// create rotation matrix from euler angles
};

#endif	/* CIMOBOTSIM_H_ */
