#ifndef CIMOBOTSIM_H_
#define CIMOBOTSIM_H_
#include "config.h"

#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
#include <drawstuff/drawstuff.h>
#define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
#endif

class CiMobotSim {
	public:
		/*
		 *	Constructor and Destructor
		 */
		CiMobotSim(int numBot, int numStp, int numGnd, dReal tmeTot, dReal *ang, dReal *vel);
		~CiMobotSim();

		/*
		 *	Functions to input simulation variables
		 */
		void setMu(dReal mu_g, dReal mu_b);
		void setCOR(dReal cor_g, dReal cor_b);

		/*
		 *	Variables to store simulation properties
		 */
		dWorldID world;
		dSpaceID space;
		dSpaceID *space_robots;
		dJointGroupID group;

		/*
		 *	Functions to build models of iMobot
		 */
		// default build function without rotation support
			// botNum:	number of module to build
			// x,y,z:	global position of center of module
		void iMobotBuild(int botNum, dReal x, dReal y, dReal z);
		// rotate module about z-axis
			// botNum:	number of module to build
			// x,y,z:	global position of center of module
			// R:		rotation matrix of module about center point
		void iMobotBuildRotated(int botNum, dReal x, dReal y, dReal z, dMatrix3 R);
		// build a new module attached to a preexisting module
			// botNum:	number of new module to build
			// attNum:	number of module which to attach
			// face1:	face of attNum module
			// face2:	face of new module to attach
		void iMobotBuildAttached(int botNum, int attNum, int face1, int face2);
		// build a new module with predefined angles for body parts
			// botNum:	number of module to build
			// x,y,z:	global position of center of module
			// R:		rotation matrix of module about center point
			// le,re:	rotation of endcaps about respective bodies (in degrees)
			// lb,rb:	rotation of bodies about center (in degrees)
		//void iMobotBuildPositioned(int botNum, dReal x, dReal y, dReal z, dMatrix3 R, dReal le, dReal lb, dReal rb, dReal re);

		/*	
		 *	Build ground out of simple objects
		 */
		void groundPlane(int gndNum, dReal a, dReal b, dReal c, dReal d);
		void groundBox(int gndNum, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dMatrix3 R);
		void groundCapsule(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dMatrix3 R);
		void groundCylinder(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dMatrix3 R);
		void groundSphere(int gndNum, dReal r, dReal px, dReal py, dReal pz);

		/* Run the simulation */
		void run(int argc, char** argv);

		/*
		 *	Utility functions
		 */
		void printData();			// print data to the screen for testing purposes
		void replyMessage();		// reply message to send back to main program
	private:
		/*
		 *	Simulation functions
		 */
		#ifdef ENABLE_DRAWSTUFF
		void ds_start();
		void ds_drawPart(dGeomID part);
		void ds_command(int cmd);
		void simulationLoop(int pause);		// loop to complete simulation
		#else
		void simulationLoop();				// loop to complete simulation
		#endif

		/*
		 *	Structs to store simulation data
		 */
		// information about each body part
		typedef struct cimobotsimpart_s {
			dBodyID bodyID;					// id of body part
			dGeomID *geomID;				// ids of geoms which make up each body part
			#ifdef ENABLE_DRAWSTUFF
			float color[3];
			int num_geomID;
			#endif
		} CiMobotSimPart;
		// information about each iMobot module
		typedef struct cimobotsimbot_s {
			CiMobotSimPart *bdyPts;			// body parts
			dJointID *joints;				// joints between body parts
			dJointID *motors;				// motors to drive body parts
			dReal *futAng;					// future angle being driven toward
			dReal *curAng;					// current angle of each body part
			dReal *jntVel;					// desired joint velocity
			dReal *ang;						// array of angles
			dReal *vel;						// array of velocities
			dReal *pos;						// 3D position of center in world
			dReal *rot;						// 3D rotation matrix of robot about center
			bool *cmpStp;					// flag to check if step is complete
		} CiMobotSimBot;
		// information on success to reply
		typedef struct cimobotsimreply_s {
			bool success;					// flag if simulation is successful
			double time;					// time to successful completion
			int message;					// reply message code
		} CiMobotSimReply;

		/*
		 *	Private variables to store general information about simulation
		 */
		CiMobotSimBot **bot;				// array of structs for each bot
		CiMobotSimReply *reply;				// struct of data to return after finishing simulation
		dGeomID *ground;					// array of ground objects
		dReal t;							// current time of simulation
		dReal tmeTot;						// total time for simulation to run
		dReal tmeStp;						// time of each step of simulation
		dReal mu_g, mu_b;					// coefficient of friction of body_ground and body_body
		dReal cor_g, cor_b;					// coefficient of restitution of body_ground and body_body
		dReal mtrRes;						// motor angle resolution
		dReal *jntVelMax;					// maximum joint velocity possible
		dReal *jntVelMin;					// minimum joint velocity possible
		dReal *jntVelDel;					// range of joint velocities
		dReal *frcMax;						// maximum force that can be applied to each body part
		int numBot;							// number of modules in simulation
		int numGnd;							// number of pieces of ground
		int numStp;							// total number of steps
		int curStp;							// current step number
		bool *flags;						// flag for each bot  - completed step
		bool *disable;						// flag for each bot - disabled/enabled
		bool newStp;						// flag for new step
		#ifdef ENABLE_DRAWSTUFF
		dsFunctions m_fn;
		#endif

		/*
		 *	Enumerations for parts of iMobot
		 */
		// each body which has a degree of freedom
		enum robot_bodies_e {
			LE,
			LB,
			RB,
			RE,
			NUM_DOF
		};
		// each body part which is built
		enum robot_pieces_e {
			BODY_L,
			BODY_R,
			CENTER,
			ENDCAP_L,
			ENDCAP_R,
			NUM_PARTS
		};

		/*
		 *	Utility functions
		 */
		// convert units
		inline dReal I2M(dReal x);
		inline dReal M2I(dReal x);
		inline dReal D2R(dReal x);
		inline dReal R2D(dReal x);
		// check if all values in array are true
		bool isTrue(bool *a, int length);
		// modify angle from ODE for endcaps to count continuously
		dReal angMod(dReal pasAng, dReal curAng, dReal angRat);

		/*
		 *	Simulation functions
		 */
		void setAngles();								// set new angles for step of sim
		void updateAngles();							// update struct with modified angles
		void setFlags();								// set flags for complete/not-complete
		void incrementStep();							// increment step to next set of angles
		bool endSimulation(double totalTime);			// check if simulation is complete and exit
		void incrementTime(double tStep);				// update simulation time
		void collision(dGeomID o1, dGeomID o2);			// callback function for contact of bodies
		static void collisionWrapper(void *data, dGeomID o1, dGeomID o2);	// wrapper function for nearCallback to work in class

		/*
		 *	Functions to build body parts of iMobot
		 */
		void buildLeftBody(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R);
		void buildRightBody(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R);
		void buildCenter(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R);
		void buildEndcap(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R);
};

#endif	/* CIMOBOTSIM_H_ */
