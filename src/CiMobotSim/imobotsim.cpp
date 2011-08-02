#include <cmath>
#include <iostream>
#include "config.h"
#include "imobotsim.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

using namespace std;

#ifdef ENABLE_DRAWSTUFF
/* If drawstuff is enabled, we need a global pointer to an instantiated
 * CiMobotSim class so that the callback functions can access the necessary
 * data. */
CiMobotSim *g_imobotsim;
#endif

CiMobotSim::CiMobotSim(int numBot, int numStp, int numGnd, dReal tmeTot, dReal *ang) {
	// initialize parameters for simulation
	int i, j;
	
	// values for simulation passed into constructor
	this->m_num_bot = numBot;
	this->m_num_stp = numStp;
	this->m_num_gnd = numGnd;
	this->m_t_tot = tmeTot;
	
	// default simulation parameters
	this->m_mu_g = 0.4;
	this->m_mu_b = 0.4;
	this->m_cor_g = 0.3;
	this->m_cor_b = 0.3;
	
	// iMobot physical parameters
	this->m_motor_res = D2R(0.5);
	this->m_joint_vel_max = new dReal[NUM_DOF];
	this->m_joint_vel_max[LE] = 6.70;
	this->m_joint_vel_max[LB] = 2.61;
	this->m_joint_vel_max[RB] = 2.61;
	this->m_joint_vel_max[RE] = 6.70;
	this->m_joint_vel_min = new dReal[NUM_DOF];
	this->m_joint_vel_min[LE] = 3.22;
	this->m_joint_vel_min[LB] = 1.25;
	this->m_joint_vel_min[RB] = 1.25;
	this->m_joint_vel_min[RE] = 3.22;
	this->m_joint_vel_del = new dReal[NUM_DOF];
	for ( j = 0; j < NUM_DOF; j++ ) this->m_joint_vel_del[j] = this->m_joint_vel_max[j] - this->m_joint_vel_min[j];
	this->m_joint_frc_max = new dReal[NUM_DOF];
	this->m_joint_frc_max[LE] = 0.260;
	this->m_joint_frc_max[LB] = 1.059;
	this->m_joint_frc_max[RB] = 1.059;
	this->m_joint_frc_max[RE] = 0.260;
	
	// variables to keep track of progress of simulation
	this->m_cur_stp = 0;
	this->m_t = 0.0;
	this->m_t_step = 0.004;
	this->m_flags = new bool[numBot];
	this->m_disable = new bool[numBot];
	this->m_ground = new dGeomID[numGnd];
	this->m_reply = new CiMobotSimReply;
	this->m_reply->success = false;
	this->m_reply->time = 0.0;
	this->m_reply->message = 0;
	
	// create and populate struct for each module in simulation
	this->bot = new CiMobotSimBot * [numBot];
	for ( i = 0; i < numBot; i++ ) {
		this->bot[i] = new CiMobotSimBot;
		this->bot[i]->bodyPart = new CiMobotSimPart[NUM_PARTS];
		this->bot[i]->bodyPart[ENDCAP_L].geomID = new dGeomID[7];
		this->bot[i]->bodyPart[BODY_L].geomID = new dGeomID[5];
		this->bot[i]->bodyPart[CENTER].geomID = new dGeomID[3];
		this->bot[i]->bodyPart[BODY_R].geomID = new dGeomID[5];
		this->bot[i]->bodyPart[ENDCAP_R].geomID = new dGeomID[7];
		#ifdef ENABLE_DRAWSTUFF
		this->bot[i]->bodyPart[ENDCAP_L].num_geomID = 7;
		this->bot[i]->bodyPart[BODY_L].num_geomID = 5;
		this->bot[i]->bodyPart[CENTER].num_geomID = 3;
		this->bot[i]->bodyPart[BODY_R].num_geomID = 5;
		this->bot[i]->bodyPart[ENDCAP_R].num_geomID = 7;
		#endif
		this->bot[i]->joints = new dJointID[6];
		this->bot[i]->motors = new dJointID[4];
		this->bot[i]->ang = new dReal[NUM_DOF*numStp];
		this->bot[i]->vel = new dReal[NUM_DOF*numStp];
		for ( j = 0; j < NUM_DOF*numStp; j++ ) {
			this->bot[i]->ang[j] = D2R(ang[4*i + NUM_DOF*numBot*(j/NUM_DOF) + j%NUM_DOF]);
			this->bot[i]->vel[j] = 1;
		}
		this->bot[i]->curAng = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->curAng[j] = 0;
		this->bot[i]->futAng = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->futAng[j] = this->bot[i]->ang[j];
		this->bot[i]->jntVel = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVel[j] = this->m_joint_vel_max[j];
		this->bot[i]->pos = new dReal[3];
		for ( j = 0; j < 3; j++ ) this->bot[i]->pos[j] = 0;
		this->bot[i]->rot = new dReal[3];
		for ( j = 0; j < 3; j++ ) this->bot[i]->rot[j] = 0;
		this->bot[i]->cmpStp = new bool[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->cmpStp[j] = false;
		this->m_flags[i] = false;
		this->m_disable[i] = false;
	}

	// create ODE simulation space
	dInitODE2(0);												// initialized ode library
	this->world  = dWorldCreate();								// create world for simulation
	this->space  = dHashSpaceCreate(0);							// create space for robots
	this->space_bot = new dSpaceID[numBot];						// create array of spaces for each robot
	for ( i = 0; i < numBot; i++ ) this->space_bot[i] = dHashSpaceCreate(this->space);		// create each robot's space
	this->group  = dJointGroupCreate(0);

	// simulation parameters
	dWorldSetAutoDisableFlag(this->world, 1);					// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(this->world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(this->world, 0.01);		// linear velocity threshold
	dWorldSetAutoDisableSteps(this->world, 10);					// number of steps below thresholds before stationary
	dWorldSetCFM(this->world, 0.0000000001);					// constraint force mixing - how much a joint can be violated by excess force
	dWorldSetContactSurfaceLayer(this->world, 0.001);			// depth each body can sink into another body before resting
	dWorldSetERP(this->world, 0.95);							// error reduction parameter (0-1) - how much error is corrected on each step
	dWorldSetGravity(this->world, 0, 0, -9.81);					// gravity

	#ifdef ENABLE_DRAWSTUFF
	m_fn.version = DS_VERSION;									// version of drawstuff
	m_fn.start = (void (*)(void))&CiMobotSim::ds_start;			// initialization function
	m_fn.step = (void (*)(int))&CiMobotSim::ds_simulation;		// function for each step of simulation
	m_fn.command = (void (*)(int))&CiMobotSim::ds_command;		// keyboard commands to control simulation
	m_fn.stop = 0;												// stopping parameter
	m_fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;				// path to texture files
	g_imobotsim = this;											// pointer to class
	#endif
}

CiMobotSim::~CiMobotSim() {
	// free all arrays created dynamically in constructor
	for (int i = this->m_num_bot - 1; i >= 0; i--) {
		for (int j = 0; j < NUM_PARTS; j++) delete [] this->bot[i]->bodyPart[j].geomID;
		delete [] this->bot[i]->bodyPart;
		delete [] this->bot[i]->joints;
		delete [] this->bot[i]->motors;
		delete [] this->bot[i]->futAng;
		delete [] this->bot[i]->curAng;
		delete [] this->bot[i]->jntVel;
		delete [] this->bot[i]->ang;
		delete [] this->bot[i]->vel;
		delete [] this->bot[i]->pos;
		delete [] this->bot[i]->rot;
		delete [] this->bot[i]->cmpStp;
		delete this->bot[i];
	}
	delete [] this->bot;
	delete [] this->m_flags;
	delete [] this->m_disable;
	delete [] this->m_ground;
	delete [] this->m_joint_vel_max;
	delete [] this->m_joint_vel_min;
	delete [] this->m_joint_vel_del;
	delete [] this->m_joint_frc_max;
	delete this->m_reply;

	// destroy all ODE objects
	dJointGroupDestroy(group);
	for ( int i = 0; i < this->m_num_bot; i++ ) dSpaceDestroy(this->space_bot[i]);
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
}

void CiMobotSim::setMu(dReal mu_g, dReal mu_b) {
	this->m_mu_g = mu_g;
	this->m_mu_b = mu_b;
}

void CiMobotSim::setCOR(dReal cor_g, dReal cor_b) {
	this->m_cor_g = cor_g;
	this->m_cor_b = cor_b;
}

void CiMobotSim::setAngVel(dReal *vel) {
	// initialize loop variables
	int i, j;
	
	// loop through each robot and assign new array of vel values
	for ( i = 0; i < this->m_num_bot; i++ ) {
		this->bot[i]->vel = new dReal[NUM_DOF*m_num_stp];
		for ( j = 0; j < NUM_DOF*this->m_num_stp; j++ ) this->bot[i]->vel[j] = vel[4*i + NUM_DOF*this->m_num_bot*(j/NUM_DOF) + j%NUM_DOF];
		this->bot[i]->jntVel = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVel[j] = this->m_joint_vel_min[j] + this->bot[i]->vel[j]*this->m_joint_vel_del[j];
	}
}

void CiMobotSim::run(int argc, char **argv) {
	#ifdef ENABLE_DRAWSTUFF
		dsSimulationLoop(argc, argv, 352, 288, &m_fn);
	#else
		simulationLoop();
	#endif
}

#ifdef ENABLE_DRAWSTUFF
/* If drawstuff is enabled, we cannot use any reference to 'this', because
 * simulationLoop is called as a callback function and any reference to 'this'
 * will be NULL. Instead, if we are using Drawstuff, we sholud reference the
 * global variable, g_imobotsim */
#define this g_imobotsim
void CiMobotSim::ds_simulation(int pause) {
	bool loop = true;													// initialize

	this->updateAngles();												// update angles for current step

	dSpaceCollide(this->space, this, &this->collisionWrapper);			// collide all geometries together
	dWorldStep(this->world, this->m_t_step);							// step world time by one
	dJointGroupEmpty(this->group);										// clear out all contact joints

	this->printIntermediateData();										// print out incremental data

	this->setFlags();													// set flags for completion of steps
	this->incrementStep();												// check whether to increment to next step

	loop = this->endSimulation(this->m_t_tot);							// check whether to end simulation
	this->incrementTime(this->m_t_step);								// increment time

	this->ds_drawBodies();
	if (!loop) dsStop();

	#undef this
}
#else
void CiMobotSim::simulationLoop(void) {
	bool loop = true;													// initialize

	while (loop) {														// loop continuously until simulation is stopped
		this->updateAngles();											// update angles for current step

		dSpaceCollide(this->space, this, &this->collisionWrapper);		// collide all geometries together
		dWorldStep(this->world, this->m_t_step);							// step world time by one
		dJointGroupEmpty(this->group);									// clear out all contact joints

		this->printIntermediateData();									// print out incremental data

		this->setFlags();												// set flags for completion of steps
		this->incrementStep();											// check whether to increment to next step

		loop = this->endSimulation(this->m_t_tot);						// check whether to end simulation
		this->incrementTime(this->m_t_step);							// increment time
	}
}
#endif

void CiMobotSim::updateAngles() {
	// update stored data in struct with data from ODE
	for (int i = 0; i < this->m_num_bot; i++) {
		// must be done for each degree of freedom
		for (int j = 0; j < NUM_DOF; j++) {
			// update current angle
			if ( j == LE || j == RE )
				this->bot[i]->curAng[j] = angMod(this->bot[i]->curAng[j],
												 dJointGetHingeAngle(this->bot[i]->joints[j]),
												 dJointGetHingeAngleRate(this->bot[i]->joints[j]));
			else
				this->bot[i]->curAng[j] = dJointGetHingeAngle(this->bot[i]->joints[j]);

			// set motor angle to current angle
			dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);

			// drive motor to get current angle to match future angle
			if ( !dJointIsEnabled(this->bot[i]->motors[j]) ) {
				this->bot[i]->futAng[j] = this->bot[i]->curAng[j];
			}
			else {
				if (this->bot[i]->curAng[j] < this->bot[i]->futAng[j] - this->m_motor_res) {
					dJointSetAMotorParam(this->bot[i]->motors[j], dParamVel, this->bot[i]->jntVel[j]);
					this->bot[i]->cmpStp[j] = false;
				}
				else if (this->bot[i]->curAng[j] > this->bot[i]->futAng[j] + this->m_motor_res) {
					dJointSetAMotorParam(this->bot[i]->motors[j], dParamVel, -this->bot[i]->jntVel[j]);
					this->bot[i]->cmpStp[j] = false;
				}
				else {
					dJointSetAMotorParam(this->bot[i]->motors[j], dParamVel, 0);
					this->bot[i]->cmpStp[j] = true;
				}
			}
		}
	}
}

void CiMobotSim::collisionWrapper(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CiMobotSim *ptr;
	ptr = (CiMobotSim *) data;
	if (ptr)
	    ptr->collision(o1, o2);

}

void CiMobotSim::collision(dGeomID o1, dGeomID o2) {
	// initialize values
	dBodyID b1, b2;
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);

	// if geoms are in same body do not intersect
	if (b1 && b2 && dAreConnected(b1, b2)) return;

	// special case for collision of spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, this, &this->collisionWrapper);
		if ( dGeomIsSpace(o1) )
			dSpaceCollide( (dSpaceID)o1, this, &this->collisionWrapper);
		if ( dGeomIsSpace(o2) )
			dSpaceCollide( (dSpaceID)o2, this, &this->collisionWrapper);
	}
	// create contact point for two geoms
	else {
		const int N = 8;
		dContact contact[N];
		int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
		if (n > 0) {
			for (int i = 0; i < n; i++) {
				// different properties for ground contact vs body contact
				if ( dGeomGetSpace(o1) == this->space || dGeomGetSpace(o2) == this->space ) {
					contact[i].surface.mode = dContactBounce | dContactApprox1;
					contact[i].surface.mu = this->m_mu_g;
					contact[i].surface.bounce = this->m_cor_g;
				}
				else {
					contact[i].surface.mode = dContactBounce | dContactApprox1;
					contact[i].surface.mu = this->m_mu_b;
					contact[i].surface.bounce = this->m_cor_b;
				}
				dJointID c = dJointCreateContact(this->world, this->group, contact+i);
				dJointAttach(c, b1, b2);
			}
		}
	}
}

void CiMobotSim::setFlags() {
	// set flags for each module in simulation
	for (int i = 0; i < this->m_num_bot; i++) {
		// all body parts of module finished
		if ( this->isTrue(this->bot[i]->cmpStp, NUM_DOF) )
			this->m_flags[i] = true;
		// module is disabled
		if ( !dBodyIsEnabled(this->bot[i]->bodyPart[CENTER].bodyID) )
			this->m_disable[i] = true;
		else
			this->m_disable[i] = false;
	}
}

void CiMobotSim::incrementStep() {
	// initialize
	static int k = 1;

	// all robots have completed motion
	if ( this->isTrue(this->m_flags, this->m_num_bot) ) {
		// increment to next step
		this->m_cur_stp++;
		// reached last step
		if ( (this->m_cur_stp == this->m_num_stp) ) {
			// finished all steps
			this->m_reply->success = true;
			// record time to completion
			if ( k == 1)
				this->m_reply->time = this->m_t;
				// step back current step number
				this->m_cur_stp = m_num_stp-1;
				k++;
		}
		this->setAngles();
		// reset all flags to zero
		for (int i = 0; i < this->m_num_bot; i++) {
			this->m_flags[i] = false;
			for (int j = 0; j < NUM_DOF; j++) {
				this->bot[i]->cmpStp[j] = false;
			}
		}
	}
}

void CiMobotSim::setAngles() {
	for (int i = 0; i < this->m_num_bot; i++) {
		// for two end cap joints
		for (int j = 0; j < NUM_DOF; j+=3) {
			if ( (int)(this->bot[i]->ang[NUM_DOF*this->m_cur_stp + j]) == (int)(D2R(123456789)) ) {
				dJointDisable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = angMod(this->bot[i]->curAng[j], 
												 dJointGetHingeAngle(this->bot[i]->joints[j]),
												 dJointGetHingeAngleRate(this->bot[i]->joints[j]));
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = true;
			}
			else {
				dJointEnable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = this->bot[i]->bodyPart[j<1?ENDCAP_L:ENDCAP_R].ang;
				for ( int k = 0; k <= this->m_cur_stp; k++) { this->bot[i]->futAng[j] += this->bot[i]->ang[NUM_DOF*k + j]; }
				this->bot[i]->jntVel[j] = this->m_joint_vel_min[j] + this->bot[i]->vel[NUM_DOF*this->m_cur_stp + j]*this->m_joint_vel_del[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = false;
			}
		}
		// for two body joints
		for (int j = 1; j < NUM_DOF-1; j++) {
			if ( (int)(this->bot[i]->ang[NUM_DOF*this->m_cur_stp + j]) == (int)(D2R(123456789)) ) {
				dJointDisable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = dJointGetHingeAngle(this->bot[i]->joints[j]);
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = true;
			}
			else {
				dJointEnable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = this->bot[i]->ang[NUM_DOF*this->m_cur_stp + j];
				this->bot[i]->jntVel[j] = this->m_joint_vel_min[j] + this->bot[i]->vel[NUM_DOF*this->m_cur_stp + j]*this->m_joint_vel_del[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = false;
			}
		}
	}
}

void CiMobotSim::printIntermediateData() {
	cout.width(3); cout << this->m_t / this->m_t_step;
	cout.width(6); cout << this->m_t;
	cout.width(3); cout << this->m_cur_stp;
	cout << "\t\t";
	//const dReal *pos;
	cout.precision(4);
	for (int i = 0; i < this->m_num_bot; i++) {
		cout << this->bot[i]->futAng[LE] << " ";
		cout << this->bot[i]->curAng[LE] << " ";
		//cout << this->bot[i]->jntVel[LE] << " ";
		//cout << this->bot[i]->cmpStp[LE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LE]) << " ";
		//
		//cout << this->bot[i]->futAng[LB] << " ";
		//cout << this->bot[i]->curAng[LB] << " ";
		//cout << this->bot[i]->jntVel[LB] << " ";
		//cout << this->bot[i]->cmpStp[LB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LB]) << " ";
		//			
		//pos = dBodyGetPosition(this->bot[i]->bodyPart[CENTER].bodyID);
		//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
		//
		//cout << this->bot[i]->futAng[RB] << " ";
		//cout << this->bot[i]->curAng[RB] << " ";
		//cout << this->bot[i]->jntVel[RB] << " ";
		//cout << this->bot[i]->cmpStp[RB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RB]) << " ";
		//
		//cout << this->bot[i]->futAng[RE] << " ";
		//cout << this->bot[i]->curAng[RE] << " ";
		//cout << this->bot[i]->jntVel[RE] << " ";
		//cout << this->bot[i]->cmpStp[RE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RE]) << " ";
	}
	//for (int i = 0; i < this->m_num_bot; i++) {
	//	cout.width(2); cout << this->m_flags[i];
	//}
	//cout << " ";
	//for (int i = 0; i < this->m_num_bot; i++) {
	//	cout << this->m_disable[i];
	//}
	cout << endl;
}

bool CiMobotSim::endSimulation(double totalTime) {
	// initialize
	bool loop = true;

	// reached end of simulation time == FAIL
	if ( (totalTime - this->m_t) < 0.0000000596047 ) {
		this->m_reply->message = 1;
		loop = false;
	}
	// all modules are disabled
	else if ( this->isTrue(this->m_disable, this->m_num_bot) ) {
		// if all steps are completed == SUCCESS
		if ( this->m_reply->success )
			this->m_reply->message = 2;
		// everything is stalled == FAIL
		else
			this->m_reply->message = 3;
		loop = false;
	}
	
	return loop;
}

void CiMobotSim::incrementTime(double tStep) {
		this->m_t += tStep;
}

void CiMobotSim::replyMessage() {
	if ( this->m_reply->message == 1 )
		cout << "Failure: reached end of simulation time" << endl;
	else if ( this->m_reply->message == 2 )
		cout << "Success: all robots have completed all of their steps in " << this->m_reply->time << " seconds"<< endl;
	else if ( this->m_reply->message == 3 )
		cout << "Failure: all robots are stationary\n" << endl;
}

/**********************************************************
	Ground Functions
 **********************************************************/
void CiMobotSim::groundBox(int gndNum, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// create rotation matrix
	dMatrix3 R, R_x, R_y, R_z, R_xy;
	dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
	dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
	dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
	dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
	dMultiply0(R, R_xy, R_z, 3, 3, 3);

	// position box
	this->m_ground[gndNum] = dCreateBox(this->space, lx, ly, lz);
	dGeomSetPosition(this->m_ground[gndNum], px, py, pz);
	dGeomSetRotation(this->m_ground[gndNum], R);
}

void CiMobotSim::groundCapsule(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// create rotation matrix
	dMatrix3 R, R_x, R_y, R_z, R_xy;
	dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
	dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
	dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
	dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
	dMultiply0(R, R_xy, R_z, 3, 3, 3);

	// position capsule
	this->m_ground[gndNum] = dCreateCapsule(this->space, r, l);
	dGeomSetPosition(this->m_ground[gndNum], px, py, pz);
	dGeomSetRotation(this->m_ground[gndNum], R);
}

void CiMobotSim::groundCylinder(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// create rotation matrix
	dMatrix3 R, R_x, R_y, R_z, R_xy;
	dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
	dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
	dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
	dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
	dMultiply0(R, R_xy, R_z, 3, 3, 3);
	
	// position cylinder
	this->m_ground[gndNum] = dCreateCylinder(this->space, r, l);
	dGeomSetPosition(this->m_ground[gndNum], px, py, pz);
	dGeomSetRotation(this->m_ground[gndNum], R);
}

void CiMobotSim::groundPlane(int gndNum, dReal a, dReal b, dReal c, dReal d) {
	this->m_ground[gndNum] = dCreatePlane(this->space, a, b, c, d);
}

void CiMobotSim::groundSphere(int gndNum, dReal r, dReal px, dReal py, dReal pz) {
	this->m_ground[gndNum] = dCreateSphere(this->space, r);
	dGeomSetPosition(this->m_ground[gndNum], px, py, pz);
}

/**********************************************************
	Build Body Parts Functions
 **********************************************************/
void CiMobotSim::imobot_build_lb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb, int rebuild) {
	// define parameters
	dBodyID body;
	dGeomID geom;
	dMass m, m1, m2, m3;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetZero(&m);
	// create mass 1
	dMassSetBox(&m1, 2700, I2M(0.2), I2M(2.60), I2M(2.85) );
	dMassAdd(&m, &m1);
	// create mass 2
	dMassSetBox(&m2, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m2, -I2M(0.5 + 0.1), -I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m2);
	// create mass 3
	dMassSetBox(&m3, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m3, -I2M(0.5 + 0.1), I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m3);
	//dMassSetParameters( &m, 500, I2M(1), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// create body
	if ( !rebuild )
		body = dBodyCreate(this->world);
	else
		body = part.bodyID;

	// set body parameters
	dBodySetPosition(body, x, y, z);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
	dRFromAxisAndAngle(R3, 0, 0, 1, -r_lb);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry 1 - face
	geom = dCreateBox( space, I2M(BODY_END_DEPTH), I2M(BODY_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part.geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( space, I2M(BODY_LENGTH), I2M(BODY_INNER_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(BODY_LENGTH / 2 + BODY_END_DEPTH / 2) - m.c[0], -I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	part.geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( space, I2M(BODY_LENGTH), I2M(BODY_INNER_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(BODY_LENGTH / 2 + BODY_END_DEPTH / 2) - m.c[0], I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	part.geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( space, I2M(BODY_RADIUS), I2M(BODY_INNER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(BODY_LENGTH + BODY_END_DEPTH / 2) - m.c[0], -I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R2);
	part.geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( space, I2M(BODY_RADIUS), I2M(BODY_INNER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(BODY_LENGTH + BODY_END_DEPTH / 2) - m.c[0], I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R2);
	part.geomID[4] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part.bodyID = body;
	part.ang = r_lb;
	#ifdef ENABLE_DRAWSTUFF
	part.color[0] = 1;
	part.color[1] = 0;
	part.color[2] = 0;
	#endif
}

void CiMobotSim::imobot_build_rb(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb, int rebuild) {
	// define parameters
	dBodyID body;
	dGeomID geom;
	dMass m, m1, m2, m3;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetZero(&m);
	// create mass 1
	dMassSetBox(&m1, 2700, I2M(0.2), I2M(2.60), I2M(2.85) );
	dMassAdd(&m, &m1);
	// create mass 2
	dMassSetBox(&m2, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m2, -I2M(0.5 + 0.1), -I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m2);
	// create mass 3
	dMassSetBox(&m3, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m3, -I2M(0.5 + 0.1), I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m3);
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// create body
	if ( !rebuild )
		body = dBodyCreate(this->world);
	else
		body = part.bodyID;

	// set body parameters
	dBodySetPosition(body, x, y, z);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
	dRFromAxisAndAngle(R3, 0, 0, 1, -r_rb);
	dMultiply0(R2, R1, R3, 3, 3, 3);
	
	// set geometry 1 - face
	geom = dCreateBox( space, I2M(BODY_END_DEPTH), I2M(BODY_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part.geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( space, I2M(BODY_LENGTH), I2M(BODY_INNER_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(BODY_LENGTH / 2 + BODY_END_DEPTH / 2) - m.c[0], -I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	part.geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( space, I2M(BODY_LENGTH), I2M(BODY_INNER_WIDTH), I2M(BODY_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(BODY_LENGTH / 2 + BODY_END_DEPTH / 2) - m.c[0], I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	part.geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( space, I2M(BODY_RADIUS), I2M(BODY_INNER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(BODY_LENGTH + BODY_END_DEPTH / 2) - m.c[0], -I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R2);
	part.geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( space, I2M(BODY_RADIUS), I2M(BODY_INNER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(BODY_LENGTH + BODY_END_DEPTH / 2) - m.c[0], I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R2);
	part.geomID[4] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part.bodyID = body;
	part.ang = r_rb;
	#ifdef ENABLE_DRAWSTUFF
	part.color[0] = 1;
	part.color[1] = 1;
	part.color[2] = 1;
	#endif
}

void CiMobotSim::imobot_build_ce(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild) {
	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// set mass of body
	dMassSetZero(&m);
	dMassSetCapsule(&m, 2700, 1, I2M(CENTER_RADIUS), I2M(CENTER_LENGTH) );
	dMassAdjust(&m, 0.24);
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// create body
	if ( !rebuild )
		body = dBodyCreate(this->world);
	else
		body = part.bodyID;

	// set body parameters
	dBodySetPosition(body, x, y, z);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);

	// set geometry 1 - center rectangle
	geom = dCreateBox( space, I2M(CENTER_LENGTH), I2M(CENTER_WIDTH), I2M(CENTER_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part.geomID[0] = geom;

	// set geometry 2 - side curve
	geom = dCreateCylinder( space, I2M(CENTER_RADIUS), I2M(CENTER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(CENTER_LENGTH / 2) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[1] = geom;

	// set geometry 3 - side curve
	geom = dCreateCylinder( space, I2M(CENTER_RADIUS), I2M(CENTER_WIDTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(CENTER_LENGTH / 2) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[2] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part.bodyID = body;
	part.ang = 0;
	#ifdef ENABLE_DRAWSTUFF
	part.color[0] = 0;
	part.color[1] = 1;
	part.color[2] = 0;
	#endif
}

void CiMobotSim::imobot_build_en(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_e, int rebuild) {
	// define parameters
	dBodyID body;
	dGeomID geom;
	dMass m;
	dMatrix3 R1;

	// set mass of body
	dMassSetBox(&m, 2700, I2M(END_DEPTH), I2M(END_WIDTH), I2M(END_HEIGHT) );
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// create body
	if ( !rebuild )
		body = dBodyCreate(this->world);
	else
		body = part.bodyID;
	
	// set body parameters
	dBodySetPosition(body, x, y, z);
	dBodySetRotation(body, R);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry 1 - center box
	geom = dCreateBox( space, I2M(END_DEPTH), I2M(END_WIDTH - 2*END_RADIUS), I2M(END_HEIGHT) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part.geomID[0] = geom;

	// set geometry 2 - left box
	geom = dCreateBox( space, I2M(END_DEPTH), I2M(END_RADIUS), I2M(END_HEIGHT - 2*END_RADIUS) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(END_WIDTH / 2 - END_RADIUS / 2) - m.c[1], -m.c[2] );
	part.geomID[1] = geom;

	// set geometry 3 - right box
	geom = dCreateBox( space, I2M(END_DEPTH), I2M(END_RADIUS), I2M(END_HEIGHT - 2*END_RADIUS) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(END_WIDTH / 2 - END_RADIUS / 2) - m.c[1], -m.c[2] );
	part.geomID[2] = geom;

	// set geometry 4 - fillet upper left
	geom = dCreateCylinder( space, I2M(END_RADIUS), I2M(END_DEPTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(END_WIDTH / 2 - END_RADIUS) - m.c[1], I2M(END_WIDTH / 2 - END_RADIUS) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[3] = geom;

	// set geometry 5 - fillet upper right
	geom = dCreateCylinder( space, I2M(END_RADIUS), I2M(END_DEPTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(END_WIDTH / 2 - END_RADIUS) - m.c[1], I2M(END_WIDTH / 2 - END_RADIUS) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[4] = geom;

	// set geometry 6 - fillet lower right
	geom = dCreateCylinder( space, I2M(END_RADIUS), I2M(END_DEPTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(END_WIDTH / 2 - END_RADIUS) - m.c[1], -I2M(END_WIDTH / 2 - END_RADIUS) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[5] = geom;

	// set geometry 7 - fillet lower left
	geom = dCreateCylinder( space, I2M(END_RADIUS), I2M(END_DEPTH) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(END_WIDTH / 2 - END_RADIUS) - m.c[1], -I2M(END_WIDTH / 2 - END_RADIUS) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part.geomID[6] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part.bodyID = body;
	part.ang = r_e;
	#ifdef ENABLE_DRAWSTUFF
	part.color[0] = 0;
	part.color[1] = 0;
	part.color[2] = 1;
	#endif
}

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CiMobotSim::iMobotBuild(int botNum, dReal x, dReal y, dReal z) {
	this->iMobotBuild(botNum, x, y, z, 0, 0, 0);
}

void CiMobotSim::iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// convert input positions to meters
	x = I2M(x);
	y = I2M(y);
	z = I2M(z + BODY_HEIGHT/2);
	// convert input angles to radians
	psi = D2R(psi);			// roll: x
	theta = D2R(theta);		// pitch: -y
	phi = D2R(phi);			// yaw: z

	// create rotation matrix for robot
	dMatrix3 R;
	rotMatrixFromEulerAngles(R, psi, theta, phi);

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH/2), 0, 0, I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH), 0, 0};
	dReal lb[6] = {I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH/2), 0, 0, I2M(-CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal rb[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal re[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};

	// build pieces of module
	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R, 0, BUILD);
	imobot_build_lb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0, BUILD);
	imobot_build_ce(this->space_bot[botNum], this->bot[botNum]->bodyPart[CENTER], x, y, z, R, BUILD);
	imobot_build_rb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0, BUILD);
	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R, 0, BUILD);

	// store position and rotation of center of module
	this->bot[botNum]->pos[0] = x;
	this->bot[botNum]->pos[1] = y;
	this->bot[botNum]->pos[2] = z - I2M(BODY_HEIGHT/2);
	this->bot[botNum]->rot[0] = psi;
	this->bot[botNum]->rot[1] = theta;
	this->bot[botNum]->rot[2] = phi;

	// create joint
	dJointID joint;

	// joint for left endcap to body
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[BODY_L].bodyID, this->bot[botNum]->bodyPart[ENDCAP_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
	dJointSetHingeAxis(joint, R[0], R[4], R[8]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[0] = joint;

	// joint for center to left body 1
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z);
	dJointSetHingeAxis(joint, -R[1], -R[5], -R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[1] = joint;

	// joint for center to left body 2
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z);
	dJointSetHingeAxis(joint, R[1], R[5], R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[4] = joint;

	// joint for center to right body 1
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z);
	dJointSetHingeAxis(joint, R[1], R[5], R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[2] = joint;

	// joint for center to right body 2
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z);
	dJointSetHingeAxis(joint, -R[1], -R[5], -R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[5] = joint;

	// joint for right body to endcap
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
	dJointSetHingeAxis(joint, -R[0], -R[4], -R[8]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[3] = joint;

	// create motor
	dJointID motor;

	// motor for left endcap to body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_L].bodyID, this->bot[botNum]->bodyPart[ENDCAP_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[0], R[4], R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[LE]);
	this->bot[botNum]->motors[0] = motor;

	// motor for center to left body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[LB]);
	this->bot[botNum]->motors[1] = motor;

	// motor for center to right body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[RB]);
	this->bot[botNum]->motors[2] = motor;

	// motor for right body to endcap
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[RE]);
	this->bot[botNum]->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->bot[botNum]->bodyPart[i].bodyID, 0.1, 0.1);
}

void CiMobotSim::iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// convert input positions to meters
	x = I2M(x);
	y = I2M(y);
	z = I2M(z + BODY_HEIGHT/2);
	// convert input angles to radians
	psi = D2R(psi);				// roll: x
	theta = D2R(theta);			// pitch: -y
	phi = D2R(phi);				// yaw: z
	r_le = D2R(r_le);			// left end
	r_lb = D2R(r_lb);			// left body
	r_rb = D2R(r_rb);			// right body
	r_re = D2R(r_re);			// right end

	// create rotation matrix for robot
	dMatrix3 R;
	rotMatrixFromEulerAngles(R, psi, theta, phi);

	// store initial body angles into array
	this->bot[botNum]->curAng[LE] = r_le;
	this->bot[botNum]->curAng[LB] = r_lb;
	this->bot[botNum]->curAng[RB] = r_rb;
	this->bot[botNum]->curAng[RE] = r_re;
	this->bot[botNum]->futAng[LE] += r_le;
	this->bot[botNum]->futAng[RE] += r_re;

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {-I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, -I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};
	dReal lb[6] = {-I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, -I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal rb[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal re[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};

	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R, 0, BUILD);
	imobot_build_lb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0, BUILD);
	imobot_build_ce(this->space_bot[botNum], this->bot[botNum]->bodyPart[CENTER], x, y, z, R, BUILD);
	imobot_build_rb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0, BUILD);
	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R, 0, BUILD);

	// store position and rotation of center of module
	this->bot[botNum]->pos[0] = x;
	this->bot[botNum]->pos[1] = y;
	this->bot[botNum]->pos[2] = z - I2M(BODY_HEIGHT/2);
	this->bot[botNum]->rot[0] = psi;
	this->bot[botNum]->rot[1] = theta;
	this->bot[botNum]->rot[2] = phi;

	// create joint
	dJointID joint;

	// joint for left endcap to body
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[BODY_L].bodyID, this->bot[botNum]->bodyPart[ENDCAP_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
	dJointSetHingeAxis(joint, R[0], R[4], R[8]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[0] = joint;

	// joint for center to left body 1
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z);
	dJointSetHingeAxis(joint, -R[1], -R[5], -R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[1] = joint;

	// joint for center to left body 2
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z);
	dJointSetHingeAxis(joint, R[1], R[5], R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[4] = joint;

	// joint for center to right body 1
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z);
	dJointSetHingeAxis(joint, R[1], R[5], R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[2] = joint;

	// joint for center to right body 2
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z);
	dJointSetHingeAxis(joint, -R[1], -R[5], -R[9]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[5] = joint;

	// joint for right body to endcap
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetHingeAnchor(joint, R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
	dJointSetHingeAxis(joint, -R[0], -R[4], -R[8]);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[3] = joint;

	// create rotation matrices for each body part
	dMatrix3 R_e, R_b, R_le, R_lb, R_rb, R_re;
	dRFromAxisAndAngle(R_b, 0, 1, 0, r_lb);
	dMultiply0(R_lb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, -1, 0, 0, r_le);
	dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
	dRFromAxisAndAngle(R_b, 0, -1, 0, r_rb);
	dMultiply0(R_rb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, 1, 0, 0, r_re);
	dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

	// offset values from center of robot
	dReal le_r[3] = {-I2M(CENTER_LENGTH/2) - I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_lb), 0, I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_lb)};
	dReal lb_r[3] = {-I2M(CENTER_LENGTH/2) - I2M(BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_lb), 0, I2M(BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_lb)};
	dReal rb_r[3] = {I2M(CENTER_LENGTH/2) + I2M(BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_rb), 0, I2M(BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_rb)};
	dReal re_r[3] = {I2M(CENTER_LENGTH/2) + I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_rb), 0, I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_rb)};

	// re-build pieces of module
	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le, r_le, REBUILD);
	imobot_build_lb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, r_lb, REBUILD);
	imobot_build_rb(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, r_rb, REBUILD);
	imobot_build_en(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re, r_re, REBUILD);

	// create motor
	dJointID motor;

	// motor for left endcap to body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_L].bodyID, this->bot[botNum]->bodyPart[ENDCAP_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R_lb[0], R_lb[4], R_lb[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[LE]);
	this->bot[botNum]->motors[0] = motor;
	
	// motor for center to left body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[LB]);
	this->bot[botNum]->motors[1] = motor;
	
	// motor for center to right body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[RB]);
	this->bot[botNum]->motors[2] = motor;

	// motor for right body to endcap
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->m_joint_frc_max[RE]);
	this->bot[botNum]->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->bot[botNum]->bodyPart[i].bodyID, 0.1, 0.1);
}

void CiMobotSim::iMobotBuildAttached(int botNum, int attNum, int face1, int face2) {
	if ( fabs(this->bot[attNum]->curAng[LE]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[LB]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[RB]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[RE]) < DBL_EPSILON )
		imobot_build_attached_00(botNum, attNum, face1, face2);
	else
		imobot_build_attached_10(botNum, attNum, face1, face2);
}

void CiMobotSim::iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	if ( fabs(this->bot[attNum]->curAng[LE]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[LB]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[RB]) < DBL_EPSILON && fabs(this->bot[attNum]->curAng[RE]) < DBL_EPSILON	)
		imobot_build_attached_01(botNum, attNum, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		imobot_build_attached_11(botNum, attNum, face1, face2, r_le, r_lb, r_rb, r_re);
}

void CiMobotSim::imobot_build_attached_00(int botNum, int attNum, int face1, int face2) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, m[3] = {0};
	dMatrix3 R, R1, R_att;

	// generate rotation matrix for base robot
	rotMatrixFromEulerAngles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

	if ( face1 == 1 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = 0;
	}
	else if ( face1 == 1 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 1 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 1 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 1 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 1 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = 0;
	}
	else if ( face1 == 2 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 2 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 2 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 2 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 2 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 2 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 3 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 3 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 3 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 3 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 3 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 3 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 4 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 4 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 4 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 4 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH +	2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 4 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = -BODY_WIDTH;
	}
	else if ( face1 == 4 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 5 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 5 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 5 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 5 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 5 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH	+	2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
		m[1] = BODY_WIDTH;
	}
	else if ( face1 == 5 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 6 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRSetIdentity(R1);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = 0;
	}
	else if ( face1 == 6 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 6 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH +	BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 6 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 6 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
	}
	else if ( face1 == 6 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
		m[1] = 0;
	}

	// extract Euler Angles from rotation matrix
	if ( fabs(R[8]-1) < DBL_EPSILON ) {			// R_31 == 1; theta = M_PI/2
		psi = atan2(-R[1], -R[2]);
		theta = M_PI/2;
		phi = 0;
	}
	else if ( fabs(R[8]+1) < DBL_EPSILON ) {	// R_31 == -1; theta = -M_PI/2
		psi = atan2(R[1], R[2]);
		theta = -M_PI/2;
		phi = 0;
	}
	else {
		theta = asin(R[8]);
		psi = atan2(R[9]/cos(theta), R[10]/cos(theta));
		phi = atan2(R[4]/cos(theta), R[0]/cos(theta));
	}

	// set x,y,z position for new module
	x = M2I(this->bot[attNum]->pos[0]) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = M2I(this->bot[attNum]->pos[1]) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = M2I(this->bot[attNum]->pos[2]) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi));

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->bodyPart[face1_part].bodyID, this->bot[botNum]->bodyPart[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotSim::imobot_build_attached_10(int botNum, int attNum, int face1, int face2) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R_att;

	// generate rotation matrix for base robot
	rotMatrixFromEulerAngles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

	if ( face1 == 1 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH 	+ R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH				+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH				+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	 							 R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						-BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	 							 R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}

	// extract Euler Angles from rotation matrix
	if ( fabs(R[8]-1) < DBL_EPSILON ) {			// R_31 == 1; theta = M_PI/2
		psi = atan2(-R[1], -R[2]);
		theta = M_PI/2;
		phi = 0;
	}
	else if ( fabs(R[8]+1) < DBL_EPSILON ) {	// R_31 == -1; theta = -M_PI/2
		psi = atan2(R[1], R[2]);
		theta = -M_PI/2;
		phi = 0;
	}
	else {
		theta = asin(R[8]);
		psi = atan2(R[9]/cos(theta), R[10]/cos(theta));
		phi = atan2(R[4]/cos(theta), R[0]/cos(theta));
	}

	// set x,y,z position for new module
	x = M2I(this->bot[attNum]->pos[0]) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = M2I(this->bot[attNum]->pos[1]) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = M2I(this->bot[attNum]->pos[2]) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi));

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->bodyPart[face1_part].bodyID, this->bot[botNum]->bodyPart[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotSim::imobot_build_attached_01(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, r_e, r_b, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R_att;

	// generate rotation matrix for base robot
	rotMatrixFromEulerAngles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

	// rotation of body about fixed point
	if ( face2 == 1 ) {
		r_e = D2R(r_le);
		r_b = D2R(r_lb);
	}
	else if ( face2 == 2 ) {
		r_e = 0;
		r_b = D2R(r_lb);
	}
	else if ( face2 == 3 ) {
		r_e = 0;
		r_b = D2R(r_lb);
	}
	else if ( face2 == 4 ) {
		r_e = 0;
		r_b = D2R(r_rb);
	}
	else if ( face2 == 5 ) {
		r_e = 0;
		r_b = D2R(r_rb);
	}
	else if ( face2 == 6 ) {
		r_e = D2R(r_re);
		r_b = D2R(r_rb);
	}

	if ( face1 == 1 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH + R1[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[0]*(-0.5*CENTER_LENGTH);
		m[1] =																	 R1[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =																	 R1[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH + R1[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = R1[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[4]*(-0.5*CENTER_LENGTH);
		m[2] = R1[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH 	+ R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
		m[2] = R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
		m[2] = R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R2[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH				+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
		m[2] = R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
		m[2] = R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = -0.5*CENTER_LENGTH				+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						-BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
		m[1] = -BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
		m[1] = -BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
		m[1] = BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
		m[1] = BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
		m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_e);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + R1[0]*(BODY_END_DEPTH + BODY_LENGTH) + R3[0]*(0.5*CENTER_LENGTH);
		m[1] = R1[4]*(BODY_END_DEPTH + BODY_LENGTH) + R3[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(BODY_END_DEPTH + BODY_LENGTH) + R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
		m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
		m[2] = R1[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
		m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 1, 0, 0, r_e);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + R1[0]*(BODY_END_DEPTH + BODY_LENGTH) + R3[0]*(0.5*CENTER_LENGTH);
		m[1] = R1[4]*(BODY_END_DEPTH + BODY_LENGTH) + R3[4]*(0.5*CENTER_LENGTH);
		m[2] = R1[8]*(BODY_END_DEPTH + BODY_LENGTH) + R3[8]*(0.5*CENTER_LENGTH);
	}

	// extract Euler Angles from rotation matrix
	if ( fabs(R[8]-1) < DBL_EPSILON ) {			// R_31 == 1; theta = M_PI/2
		psi = atan2(-R[1], -R[2]);
		theta = M_PI/2;
		phi = 0;
	}
	else if ( fabs(R[8]+1) < DBL_EPSILON ) {	// R_31 == -1; theta = -M_PI/2
		psi = atan2(R[1], R[2]);
		theta = -M_PI/2;
		phi = 0;
	}
	else {
		theta = asin(R[8]);
		psi = atan2(R[9]/cos(theta), R[10]/cos(theta));
		phi = atan2(R[4]/cos(theta), R[0]/cos(theta));
	}

	// set x,y,z position for new module
	x = M2I(this->bot[attNum]->pos[0]) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = M2I(this->bot[attNum]->pos[1]) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = M2I(this->bot[attNum]->pos[2]) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->bodyPart[face1_part].bodyID, this->bot[botNum]->bodyPart[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotSim::imobot_build_attached_11(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, r_e, r_b, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R6, R7, R8, R9, R_att;

	// generate rotation matrix for base robot
	rotMatrixFromEulerAngles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

	// rotation of body about fixed point
	if ( face2 == 1 ) {
		r_e = D2R(r_le);
		r_b = D2R(r_lb);
	}
	else if ( face2 == 2 ) {
		r_e = 0;
		r_b = D2R(r_lb);
	}
	else if ( face2 == 3 ) {
		r_e = 0;
		r_b = D2R(r_lb);
	}
	else if ( face2 == 4 ) {
		r_e = 0;
		r_b = D2R(r_rb);
	}
	else if ( face2 == 5 ) {
		r_e = 0;
		r_b = D2R(r_rb);
	}
	else if ( face2 == 6 ) {
		r_e = D2R(r_re);
		r_b = D2R(r_rb);
	}

	if ( face1 == 1 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
		dMultiply0(R7, R6, R5, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH) + R5[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH) + R5[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[4]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH) + R5[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 1 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[BODY_L].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[BODY_L].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->bodyPart[ENDCAP_L].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
		dMultiply0(R7, R6, R5, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH) + R5[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH) + R5[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[4]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH) + R5[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH 	+ R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) 								+ R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH				+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH				+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =									+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	 							 R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH				+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						-BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH					+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =										+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	 							 R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -	END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;
		
		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);
		
		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH				+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH				+ R3[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R3[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R3[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R3[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R3[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[face1_part].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[face1_part].ang);
		dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
		dMultiply0(R7, R6, R5, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH) + R5[0]*(BODY_END_DEPTH + BODY_LENGTH) + R7[0]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH) + R5[4]*(BODY_END_DEPTH + BODY_LENGTH) + R7[4]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH) + R5[8]*(BODY_END_DEPTH + BODY_LENGTH) + R7[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 6 && face2 == 6 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->bodyPart[BODY_R].ang);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->bodyPart[BODY_R].ang);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->bodyPart[ENDCAP_R].ang);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
		dMultiply0(R5, R4, R3, 3, 3, 3);
		dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
		dMultiply0(R7, R6, R5, 3, 3, 3);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH) + R5[0]*(BODY_END_DEPTH + BODY_LENGTH) + R7[0]*(0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH) + R5[4]*(BODY_END_DEPTH + BODY_LENGTH) + R7[4]*(0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH) + R5[8]*(BODY_END_DEPTH + BODY_LENGTH) + R7[8]*(0.5*CENTER_LENGTH);
	}

	// extract Euler Angles from rotation matrix
	if ( fabs(R[8]-1) < DBL_EPSILON ) {			// R_31 == 1; theta = M_PI/2
		psi = atan2(-R[1], -R[2]);
		theta = M_PI/2;
		phi = 0;
	}
	else if ( fabs(R[8]+1) < DBL_EPSILON ) {	// R_31 == -1; theta = -M_PI/2
		psi = atan2(R[1], R[2]);
		theta = -M_PI/2;
		phi = 0;
	}
	else {
		theta = asin(R[8]);
		psi = atan2(R[9]/cos(theta), R[10]/cos(theta));
		phi = atan2(R[4]/cos(theta), R[0]/cos(theta));
	}

	// set x,y,z position for new module
	x = M2I(this->bot[attNum]->pos[0]) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = M2I(this->bot[attNum]->pos[1]) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = M2I(this->bot[attNum]->pos[2]) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->bodyPart[face1_part].bodyID, this->bot[botNum]->bodyPart[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

/**********************************************************
	Utility Functions
 **********************************************************/
inline dReal CiMobotSim::I2M( dReal x ) {
	return x*0.0254;
}
inline dReal CiMobotSim::M2I( dReal x ) {
	return x/0.0254;
}
inline dReal CiMobotSim::D2R( dReal x ) {
	return x*M_PI/180;
}
inline dReal CiMobotSim::R2D( dReal x ) {
	return x/M_PI*180;
}

bool CiMobotSim::isTrue(bool *a, int length) {
	for (int i = 0; i < length; i++) {
		if ( a[i] == false )
			return false;
	}
	return true;
}

dReal CiMobotSim::angMod(dReal pasAng, dReal curAng, dReal angRat) {
	dReal newAng = 0;
	int stp = (int)( fabs(pasAng) / M_PI );
	dReal pasAngMod = fabs(pasAng) - stp*M_PI;

	if ( (int)angRat == 0 ) {
		newAng = pasAng;
	}
	// positive angular velocity, positive angle
	else if ( angRat > 0 && pasAng >= 0 ) {
		// cross 180
		if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + 2*M_PI);	}
		// negative
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// cross 0
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// positive
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
	}
	// positive angular velocity, negative angle
	else if ( angRat > 0 && pasAng < 0 ) {
		// cross 180
		if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod + M_PI);	}
		// negative
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// cross 0
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// positive
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
	}
	// negative angular velocity, positive angle
	else if ( angRat < 0 && pasAng >= 0 ) {
		// cross 180
		if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod - M_PI);	}
		// negative
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod + M_PI);	}
		// cross 0
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
		// positive
		else if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng - pasAngMod);	}
	}
	// negative angular velocity, negative angle
	else if ( angRat < 0 && pasAng < 0 ) {
		// cross 180
		if ( curAng > 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - 2*M_PI);	}
		// negative
		else if ( curAng < 0 && !(stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod);	}
		// cross 0
		else if ( curAng < 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
		// positive
		else if ( curAng > 0 && (stp % 2) ) {	newAng = pasAng + (curAng + pasAngMod - M_PI);	}
	}

	return newAng;
}

void CiMobotSim::rotMatrixFromEulerAngles(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
	dReal	sphi = sin(phi), cphi = cos(phi),
			stheta = sin(theta), ctheta = cos(theta),
			spsi = sin(psi), cpsi = cos(psi);

	R[0] =  cphi*ctheta;
	R[1] = -cphi*stheta*spsi - sphi*cpsi;
	R[2] = -cphi*stheta*cpsi + sphi*spsi;
	R[3] = 0;
	R[4] =  sphi*ctheta;
	R[5] = -sphi*stheta*spsi + cphi*cpsi;
	R[6] = -sphi*stheta*cpsi - cphi*spsi;
	R[7] = 0;
	R[8] =  stheta;
	R[9] =  ctheta*spsi;
	R[10] = ctheta*cpsi;
	R[11] = 0;
}

#ifdef ENABLE_DRAWSTUFF
/**********************************************************
	Drawstuff Functions
 **********************************************************/
void CiMobotSim::ds_drawBodies() {
	// draw the bodies
	for (int i = 0; i < this->m_num_bot; i++) {
		for (int j = 0; j < NUM_PARTS; j++) {
			if (dBodyIsEnabled(this->bot[i]->bodyPart[j].bodyID))
				dsSetColor(	this->bot[i]->bodyPart[j].color[0], this->bot[i]->bodyPart[j].color[1], this->bot[i]->bodyPart[j].color[2]);
			else
				dsSetColor(0.5,0.5,0.5);

			for (int k = 0; k < this->bot[i]->bodyPart[j].num_geomID; k++) { ds_drawPart(this->bot[i]->bodyPart[j].geomID[k]); }
		}
	}
}

void CiMobotSim::ds_start() {
	dAllocateODEDataForThread(dAllocateMaskAll);
	/*static float xyz[3] = {I2M(10), I2M(10), I2M(5)};		// right side
	static float hpr[3] = {225.0, -20.0, 0.0};				// defined in degrees*/
	static float xyz[3] = {I2M(10), I2M(-10), I2M(5)};		// left side
	static float hpr[3] = {135.0, -20.0, 0.0};				// defined in degrees
	dsSetViewpoint(xyz, hpr);
}

void CiMobotSim::ds_command(int cmd) {
	switch (cmd) {
		case 'q': case 'Q':
			dsStop();
	}
}

void CiMobotSim::ds_drawPart(dGeomID geom) {
	// make sure correct geom is passed to function
	if (!geom) return;

	// get pos, rot of body
	const dReal *position = dGeomGetPosition(geom);
	const dReal *rotation = dGeomGetRotation(geom);

	switch (dGeomGetClass(geom)) {
		case dSphereClass:
			{
				dReal r = dGeomSphereGetRadius(geom);
				dsDrawSphere(position, rotation, r);
				break;
			}
		case dBoxClass:
			{
				dVector3 sides;
				dGeomBoxGetLengths(geom, sides);
				dsDrawBox(position, rotation, sides);
				break;
			}
		case dCylinderClass:
			{
				dReal r, l;
				dGeomCylinderGetParams(geom, &r, &l);
				dsDrawCylinder(position, rotation, l, r);
				break;
			}
		case dCapsuleClass:
			{
				dReal r, l;
				dGeomCapsuleGetParams(geom, &r, &l);
				dsDrawCapsule(position, rotation, l, r);
				break;
			}
	}
}
#endif
