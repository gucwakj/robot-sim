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
	this->numBot = numBot;
	this->numStp = numStp;
	this->numGnd = numGnd;
	this->tmeTot = tmeTot;
	
	// default simulation parameters
	this->mu_g = 0.4;
	this->mu_b = 0.4;
	this->cor_g = 0.3;
	this->cor_b = 0.3;
	
	// iMobot physical parameters
	this->mtrRes = D2R(0.5);
	this->jntVelMax = new dReal[NUM_DOF];
	this->jntVelMax[LE] = 6.70;
	this->jntVelMax[LB] = 2.61;
	this->jntVelMax[RB] = 2.61;
	this->jntVelMax[RE] = 6.70;
	this->jntVelMin = new dReal[NUM_DOF];
	this->jntVelMin[LE] = 3.22;
	this->jntVelMin[LB] = 1.25;
	this->jntVelMin[RB] = 1.25;
	this->jntVelMin[RE] = 3.22;
	this->jntVelDel = new dReal[NUM_DOF];
	for ( j = 0; j < NUM_DOF; j++ ) this->jntVelDel[j] = this->jntVelMax[j] - this->jntVelMin[j];
	this->frcMax = new dReal[NUM_DOF];
	this->frcMax[LE] = 0.260;
	this->frcMax[LB] = 1.059;
	this->frcMax[RB] = 1.059;
	this->frcMax[RE] = 0.260;
	
	// variables to keep track of progress of simulation
	this->curStp = 0;
	this->t = 0.0;
	this->tmeStp = 0.004;
	this->flags = new bool[numBot];
	this->disable = new bool[numBot];
	this->ground = new dGeomID[numGnd];
	this->reply = new CiMobotSimReply;
	this->reply->success = false;
	this->reply->time = 0.0;
	this->reply->message = 0;
	
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
			this->bot[i]->ang[j] = D2R(ang[4*i + NUM_DOF*this->numBot*(j/NUM_DOF) + j%NUM_DOF]);
			this->bot[i]->vel[j] = 1;
		}
		this->bot[i]->curAng = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->curAng[j] = 0;
		this->bot[i]->futAng = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->futAng[j] = this->bot[i]->ang[j];
		this->bot[i]->jntVel = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVel[j] = this->jntVelMax[j];
		this->bot[i]->pos = new dReal[3];
		for ( j = 0; j < 3; j++ ) this->bot[i]->pos[j] = 0;
		this->bot[i]->rot = new dReal[12];
		for ( j = 0; j < 12; j++ ) this->bot[i]->rot[j] = 0;
		this->bot[i]->cmpStp = new bool[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->cmpStp[j] = false;
		this->flags[i] = false;
		this->disable[i] = false;
	}

	#ifdef ENABLE_DRAWSTUFF
	/* Initialize drawstuff */
	m_fn.version = DS_VERSION;
	m_fn.start = (void (*)(void))&CiMobotSim::ds_start;
	m_fn.step = (void (*)(int))&CiMobotSim::simulationLoop;
	m_fn.command = (void (*)(int))&CiMobotSim::ds_command;
	m_fn.stop = 0;
	m_fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
	#endif

	// create ODE simulation space
	dInitODE2(0);
	this->world  = dWorldCreate();
	this->space  = dHashSpaceCreate(0);
	this->space_bot = new dSpaceID[numBot];
	for ( int i = 0; i < numBot; i++ ) {
		this->space_bot[i] = dHashSpaceCreate(this->space);
		dSpaceSetCleanup(this->space_bot[i],1);
	}
	this->group  = dJointGroupCreate(1000000);
	
	// simulation parameters
	dWorldSetAutoDisableFlag(this->world, 1);					// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(this->world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(this->world, 0.01);		// linear velocity threshold
	dWorldSetAutoDisableSteps(this->world, 10);					// number of steps below thresholds before stationary
	dWorldSetCFM(this->world, 0.0000000001);					// constraint force mixing - how much a joint can be violated by excess force
	dWorldSetContactSurfaceLayer(this->world, 0.001);			// depth each body can sink into another body before resting
	dWorldSetDamping(this->world, 0.1, 0.1);					// damping of all bodies in world
	dWorldSetERP(this->world, 0.95);							// error reduction parameter (0-1) - how much error is corrected on each step
	dWorldSetGravity(this->world, 0, 0, -9.81);					// gravity

	#ifdef ENABLE_DRAWSTUFF
	g_imobotsim = this;
	#endif
}

CiMobotSim::~CiMobotSim() {
	// free all arrays created dynamically in constructor
	for (int i = this->numBot - 1; i >= 0; i--) {
		for (int j = 0; j < NUM_PARTS; j++) {
			delete [] this->bot[i]->bodyPart[j].geomID;
		}
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
	delete [] this->flags;
	delete [] this->disable;
	delete [] this->ground;
	delete [] this->jntVelMax;
	delete [] this->jntVelMin;
	delete [] this->jntVelDel;
	delete [] this->frcMax;
	delete this->reply;

	// destroy all ODE objects
	dJointGroupDestroy(group);
	for ( int i = 0; i < this->numBot; i++ ) {
		dSpaceDestroy(this->space_bot[i]);
	}
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
}

void CiMobotSim::setMu(dReal mu_g, dReal mu_b) {
	this->mu_g = mu_g;
	this->mu_b = mu_b;
}

void CiMobotSim::setCOR(dReal cor_g, dReal cor_b) {
	this->cor_g = cor_g;
	this->cor_b = cor_b;
}

void CiMobotSim::setAngVel(dReal *vel) {
	int i, j;
	for ( i = 0; i < this->numBot; i++ ) {
		this->bot[i]->vel = new dReal[NUM_DOF*numStp];
		for ( j = 0; j < NUM_DOF*numStp; j++ ) this->bot[i]->vel[j] = vel[4*i + NUM_DOF*this->numBot*(j/NUM_DOF) + j%NUM_DOF];
		this->bot[i]->jntVel = new dReal[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVel[j] = this->jntVelMin[j] + this->bot[i]->vel[j]*this->jntVelDel[j];
	}
}

void CiMobotSim::printData() {
	for (int i = 0; i < this->numBot; i++) {
		cout << this->bot[i]->ang[0] << "\t" << this->bot[i]->ang[1] << "\t" << this->bot[i]->ang[2] << "\t" << this->bot[i]->ang[3] << endl;
		cout << this->bot[i]->ang[4] << "\t" << this->bot[i]->ang[5] << "\t" << this->bot[i]->ang[6] << "\t" << this->bot[i]->ang[7] << endl;
		cout << this->bot[i]->ang[8] << "\t" << this->bot[i]->ang[9] << "\t" << this->bot[i]->ang[10] << "\t" << this->bot[i]->ang[11] << endl;
		cout << this->bot[i]->ang[12] << "\t" << this->bot[i]->ang[13] << "\t" << this->bot[i]->ang[14] << "\t" << this->bot[i]->ang[15] << endl;
		cout << this->bot[i]->vel[0] << "\t" << this->bot[i]->vel[1] << "\t" << this->bot[i]->vel[2] << "\t" << this->bot[i]->vel[3] << endl;
		cout << this->bot[i]->vel[4] << "\t" << this->bot[i]->vel[5] << "\t" << this->bot[i]->vel[6] << "\t" << this->bot[i]->vel[7] << endl;
		cout << this->bot[i]->vel[8] << "\t" << this->bot[i]->vel[9] << "\t" << this->bot[i]->vel[10] << "\t" << this->bot[i]->vel[11] << endl;
		cout << this->bot[i]->vel[12] << "\t" << this->bot[i]->vel[13] << "\t" << this->bot[i]->vel[14] << "\t" << this->bot[i]->vel[15] << endl;
	}
}

void CiMobotSim::run(int argc, char** argv) {
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
void CiMobotSim::simulationLoop(int pause) {
#else
void CiMobotSim::simulationLoop(void) {
#endif
	// initialize
	bool loop = true;

	// loop continuously until simulation is stopped
	#ifndef ENABLE_DRAWSTUFF
	while (loop) {
	#endif
		this->updateAngles();											// update angles for current step

		dSpaceCollide(this->space, this, &this->collisionWrapper);		// collide all geometries together
		dWorldStep(this->world, this->tmeStp);							// step world time by one
		dJointGroupEmpty(this->group);									// clear out all contact joints
		
		this->printIntermediateData();									// print out incremental data

		this->setFlags();												// set flags for completion of steps
		this->incrementStep();											// check whether to increment to next step

		loop = this->endSimulation(this->tmeTot);						// check whether to end simulation
		this->incrementTime(this->tmeStp);								// increment time

		#ifdef ENABLE_DRAWSTUFF
		this->ds_drawBodies();
		if (!loop) dsStop();
		#endif

	#ifndef ENABLE_DRAWSTUFF
	}
	#else
	#undef this
	#endif
}

void CiMobotSim::updateAngles() {
	// update stored data in struct with data from ODE
	for (int i = 0; i < this->numBot; i++) {
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
				if (this->bot[i]->curAng[j] < this->bot[i]->futAng[j] - this->mtrRes) {
					dJointSetAMotorParam(this->bot[i]->motors[j], dParamVel, this->bot[i]->jntVel[j]);
					this->bot[i]->cmpStp[j] = false;
				}
				else if (this->bot[i]->curAng[j] > this->bot[i]->futAng[j] + this->mtrRes) {
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
					contact[i].surface.mu = this->mu_g;
					contact[i].surface.bounce = this->cor_g;
				}
				else {
					contact[i].surface.mode = dContactBounce | dContactApprox1;
					contact[i].surface.mu = this->mu_b;
					contact[i].surface.bounce = this->cor_b;
				}
				dJointID c = dJointCreateContact(this->world, this->group, contact+i);
				dJointAttach(c, b1, b2);
			}
		}
	}
}

void CiMobotSim::setFlags() {
	// set flags for each module in simulation
	for (int i = 0; i < this->numBot; i++) {
		// all body parts of module finished
		if ( this->isTrue(this->bot[i]->cmpStp, NUM_DOF) )
			this->flags[i] = true;
		// module is disabled
		if ( !dBodyIsEnabled(this->bot[i]->bodyPart[0].bodyID) )
			this->disable[i] = true;
		else
			this->disable[i] = false;
	}
}

void CiMobotSim::incrementStep() {
	// initialize
	static int k = 1;

	// all robots have completed motion
	if ( this->isTrue(this->flags, this->numBot) ) {
		// increment to next step
		this->curStp++;
		// reached last step
		if ( (this->curStp == this->numStp) ) {
			// finished all steps
			this->reply->success = true;
			// record time to completion
			if ( k == 1)
				this->reply->time = this->t;
				// step back current step number
				this->curStp = numStp-1;
				k++;
		}
		this->setAngles();
		// reset all flags to zero
		for (int i = 0; i < this->numBot; i++) {
			this->flags[i] = false;
			for (int j = 0; j < NUM_DOF; j++) {
				this->bot[i]->cmpStp[j] = false;
			}
		}
	}
}

void CiMobotSim::setAngles() {
	for (int i = 0; i < this->numBot; i++) {
		// for two end cap joints
		for (int j = 0; j < NUM_DOF; j+=3) {
			if ( (int)(this->bot[i]->ang[NUM_DOF*this->curStp + j]) == (int)(D2R(123456789)) ) {
				dJointDisable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = angMod(this->bot[i]->curAng[j], 
												 dJointGetHingeAngle(this->bot[i]->joints[j]),
												 dJointGetHingeAngleRate(this->bot[i]->joints[j]));
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = true;
			}
			else {
				dJointEnable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = this->bot[i]->bodyPart[j].ang;
				for ( int k = 0; k <= this->curStp; k++)
					this->bot[i]->futAng[j] += this->bot[i]->ang[NUM_DOF*k + j];
				this->bot[i]->jntVel[j] = this->jntVelMin[j] + this->bot[i]->vel[NUM_DOF*this->curStp + j]*this->jntVelDel[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = false;
			}
		}
		// for two body joints
		for (int j = 1; j < NUM_DOF-1; j++) {
			if ( (int)(this->bot[i]->ang[NUM_DOF*this->curStp + j]) == (int)(D2R(123456789)) ) {
				dJointDisable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = dJointGetHingeAngle(this->bot[i]->joints[j]);
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = true;
			}
			else {
				dJointEnable(this->bot[i]->motors[j]);
				this->bot[i]->futAng[j] = this->bot[i]->ang[NUM_DOF*this->curStp + j];
				this->bot[i]->jntVel[j] = this->jntVelMin[j] + this->bot[i]->vel[NUM_DOF*this->curStp + j]*this->jntVelDel[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = false;
			}
		}
	}
}

void CiMobotSim::printIntermediateData() {
	cout.width(3); cout << this->t / this->tmeStp;
	cout.width(6); cout << this->t;
	cout.width(3); cout << this->curStp;
	cout << "\t\t";
	//const dReal *pos;
	cout.precision(4);
	for (int i = 0; i < this->numBot; i++) {
		cout << this->bot[i]->futAng[LE] << " ";
		cout << this->bot[i]->curAng[LE] << " ";
		//cout << this->bot[i]->jntVel[LE] << " ";
		//cout << this->bot[i]->cmpStp[LE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LE]) << " ";
		//
		cout << this->bot[i]->futAng[LB] << " ";
		cout << this->bot[i]->curAng[LB] << " ";
		//cout << this->bot[i]->jntVel[LB] << " ";
		//cout << this->bot[i]->cmpStp[LB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LB]) << " ";
		//			
		//pos = dBodyGetPosition(this->bot[i]->bodyPart[CENTER].bodyID);
		//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
		//
		cout << this->bot[i]->futAng[RB] << " ";
		cout << this->bot[i]->curAng[RB] << " ";
		//cout << this->bot[i]->jntVel[RB] << " ";
		//cout << this->bot[i]->cmpStp[RB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RB]) << " ";
		//
		cout << this->bot[i]->futAng[RE] << " ";
		cout << this->bot[i]->curAng[RE] << " ";
		//cout << this->bot[i]->jntVel[RE] << " ";
		//cout << this->bot[i]->cmpStp[RE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RE]) << " ";
	}
	//for (int i = 0; i < this->numBot; i++) {
	//	cout.width(2); cout << this->flags[i];
	//}
	//cout << " ";
	//for (int i = 0; i < this->numBot; i++) {
	//	cout << this->disable[i];
	//}
	cout << endl;
}

bool CiMobotSim::endSimulation(double totalTime) {
	// initialize
	bool loop = true;

	// reached end of simulation time == FAIL
	if ( (totalTime - this->t) < 0.0000000596047 ) {
		this->reply->message = 1;
		loop = false;
	}
	// all modules are disabled
	else if ( this->isTrue(this->disable, this->numBot) ) {
		// if all steps are completed == SUCCESS
		if ( this->reply->success )
			this->reply->message = 2;
		// everything is stalled == FAIL
		else
			this->reply->message = 3;
		loop = false;
	}
	
	return loop;
}

void CiMobotSim::incrementTime(double tStep) {
		this->t += tStep;
}

void CiMobotSim::replyMessage() {
	if ( this->reply->message == 1 )
		cout << "Failure: reached end of simulation time" << endl;
	else if ( this->reply->message == 2 )
		cout << "Success: all robots have completed all of their steps in " << this->reply->time << " seconds"<< endl;
	else if ( this->reply->message == 3 )
		cout << "Failure: all robots are stationary\n" << endl;
}

/**********************************************************
	Ground Functions
 **********************************************************/
void CiMobotSim::groundBox(int gndNum, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dMatrix3 R) {
	this->ground[gndNum] = dCreateBox(this->space, lx, ly, lz);
	dGeomSetPosition(this->ground[gndNum], px, py, pz);
	dGeomSetRotation(this->ground[gndNum], R);
}

void CiMobotSim::groundCapsule(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dMatrix3 R) {
	this->ground[gndNum] = dCreateCapsule(this->space, r, l);
	dGeomSetPosition(this->ground[gndNum], px, py, pz);
	dGeomSetRotation(this->ground[gndNum], R);
}

void CiMobotSim::groundCylinder(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dMatrix3 R) {
	this->ground[gndNum] = dCreateCapsule(this->space, r, l);
	dGeomSetPosition(this->ground[gndNum], px, py, pz);
	dGeomSetRotation(this->ground[gndNum], R);
}

void CiMobotSim::groundPlane(int gndNum, dReal a, dReal b, dReal c, dReal d) {
	this->ground[gndNum] = dCreatePlane(this->space, a, b, c, d);
}

void CiMobotSim::groundSphere(int gndNum, dReal r, dReal px, dReal py, dReal pz) {
	this->ground[gndNum] = dCreateSphere(this->space, r);
	dGeomSetPosition(this->ground[gndNum], px, py, pz);
}

/**********************************************************
	Build Body Parts Functions
 **********************************************************/
void CiMobotSim::buildLeftBody(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_lb, int rebuild) {
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
	dGeomSetOffsetPosition( geom, I2M(BODY_INNER_WIDTH / 2 + BODY_END_DEPTH / 2) - m.c[0], I2M(BODY_RADIUS - BODY_INNER_WIDTH / 2) - m.c[1], -m.c[2] );
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

void CiMobotSim::buildRightBody(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_rb, int rebuild) {
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

void CiMobotSim::buildCenter(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild) {
	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R_x, R_y, R_z, R1, R2;
	
	/*// create rotation matrix from supplied angles
	dRFromAxisAndAngle(R_x, 1, 0, 0, r_x);
	dRFromAxisAndAngle(R_y, R_x[1], R_x[5], R_x[9], r_y);
	dRFromAxisAndAngle(R_z, R_y[2], R_y[6], R_y[10], r_z);
	dRFromAxisAndAngle(R, R_z[1], R_z[5], R_z[9], 0);*/

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
	dBodySetPosition(body, x, y, z);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

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

void CiMobotSim::buildEndcap(dSpaceID &space, CiMobotSimPart &part, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_e, int rebuild) {
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
	dMatrix3 R;
	dRSetIdentity(R);
	this->iMobotBuildRotated(botNum, x, y, z, R);
}

void CiMobotSim::iMobotBuildRotated(int botNum, dReal x, dReal y, dReal z, dMatrix3 R) {
	// convert input positions to meters
	x = I2M(x);
	y = I2M(y);
	z = I2M(z + BODY_HEIGHT/2);

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH/2), 0, 0, I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH), 0, 0};
	dReal lb[6] = {I2M(-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH/2), 0, 0, I2M(-CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal ce[3] = {0, 0, 0};
	dReal rb[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal re[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};

	// build pieces of module
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R, 0, BUILD);
	buildLeftBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0, BUILD);
	buildCenter(this->space_bot[botNum], this->bot[botNum]->bodyPart[CENTER], x, y, z, R, BUILD);
	buildRightBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0, BUILD);
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R, 0, BUILD);
	
	// store position and rotation of center of module
	this->bot[botNum]->pos[0] = x;
	this->bot[botNum]->pos[1] = y;
	this->bot[botNum]->pos[2] = z - I2M(BODY_HEIGHT/2);
	for (int i = 0; i < 12; i++) this->bot[botNum]->rot[i] = R[i];

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
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[LE]);
	this->bot[botNum]->motors[0] = motor;

	// motor for center to left body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[LB]);
	this->bot[botNum]->motors[1] = motor;

	// motor for center to right body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[RB]);
	this->bot[botNum]->motors[2] = motor;

	// motor for right body to endcap
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[RE]);
	this->bot[botNum]->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->bot[botNum]->bodyPart[i].bodyID, 0.1, 0.1);
}

void CiMobotSim::iMobotBuildPositioned(int botNum, dReal x, dReal y, dReal z, dMatrix3 R, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// convert input positions to meters
	x = I2M(x);
	y = I2M(y);
	z = I2M(z + BODY_HEIGHT/2);
	r_le = D2R(r_le);
	r_lb = D2R(r_lb);
	r_rb = D2R(r_rb);
	r_re = D2R(r_re);
	
	// store initial body angles into array
	this->bot[botNum]->curAng[LE] = r_le;
	this->bot[botNum]->curAng[RE] = r_re;
	this->bot[botNum]->futAng[LE] += r_le;
	this->bot[botNum]->futAng[RE] += r_re;

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {-I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, -I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};
	dReal lb[6] = {-I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, -I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal ce[3] = {0, 0, 0};
	dReal rb[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2), I2M(CENTER_WIDTH/2), 0};
	dReal re[6] = {I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2), 0, 0, I2M(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH), 0, 0};
	
	// build pieces of module
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R, 0, BUILD);
	buildLeftBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0, BUILD);
	buildCenter(this->space_bot[botNum], this->bot[botNum]->bodyPart[CENTER], x, y, z, R, BUILD);
	buildRightBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0, BUILD);
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R, 0, BUILD);
	
	// store position and rotation of center of module
	this->bot[botNum]->pos[0] = x;
	this->bot[botNum]->pos[1] = y;
	this->bot[botNum]->pos[2] = z - I2M(BODY_HEIGHT/2);
	for (int i = 0; i < 12; i++) this->bot[botNum]->rot[i] = R[i];
	
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
	dMatrix3 R_e, R_le, R_lb, R_rb, R_re;
	dRFromAxisAndAngle(R_lb, R[1], R[5], R[9], r_lb);
	dRFromAxisAndAngle(R_e, -R[0], -R[4], -R[8], r_le);
	dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
	dRFromAxisAndAngle(R_rb, -R[1], -R[5], -R[9], r_rb);
	dRFromAxisAndAngle(R_e, R[0], R[4], R[8], r_re);
	dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

	// re-build pieces of module
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_L], x - I2M(CENTER_LENGTH/2) - I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_lb), y, I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_lb) + z, R_le, r_le, REBUILD);
	buildLeftBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_L], x - I2M(CENTER_LENGTH/2) - I2M(BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_lb), y, I2M(BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_lb) + z, R_lb, r_lb, REBUILD);
	buildRightBody(this->space_bot[botNum], this->bot[botNum]->bodyPart[BODY_R], I2M(CENTER_LENGTH/2) + I2M(BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_rb) + x, y, I2M(BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_rb) + z, R_rb, r_rb, REBUILD);
	buildEndcap(this->space_bot[botNum], this->bot[botNum]->bodyPart[ENDCAP_R], I2M(CENTER_LENGTH/2) + I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_rb) + x, y, I2M(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_rb) + z, R_re, r_re, REBUILD);

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
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[LE]);
	this->bot[botNum]->motors[0] = motor;
	
	// motor for center to left body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[LB]);
	this->bot[botNum]->motors[1] = motor;
	
	// motor for center to right body
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[CENTER].bodyID, this->bot[botNum]->bodyPart[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[RB]);
	this->bot[botNum]->motors[2] = motor;
	
	// motor for right body to endcap
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bodyPart[BODY_R].bodyID, this->bot[botNum]->bodyPart[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->frcMax[RE]);
	this->bot[botNum]->motors[3] = motor;
	
	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->bot[botNum]->bodyPart[i].bodyID, 0.1, 0.1);
}

void CiMobotSim::iMobotBuildAttached(int botNum, int attNum, int face1, int face2) {
	dMatrix3 R, R1;
	dVector3 m;
	dReal x, y, z;
	int face1_part, face2_part;

	if ( face1 == 1 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28;
		m[1] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28;
		m[1] = -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[1] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[1] = -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 2 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[1] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 2 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 2 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = (CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 3 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = (CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 3 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 3 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 4 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[1] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 4 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 4 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 5 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 5 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = BODY_WIDTH;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 5 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2);
		m[1] = (CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 6 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 6 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28;
		m[1] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28;
		m[1] = -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[1] = CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 6 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH - 1.28);
		m[1] = -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - BODY_WIDTH/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_R;
	}

	// build rotation matrix for new module
	dMultiply0(R1, this->bot[attNum]->rot, R, 3, 3, 3);

	// set x,y,z position for new module
	x = M2I(this->bot[attNum]->pos[0]) + R1[0]*m[0] + R1[1]*m[1] + R1[2]*m[2];
	y = M2I(this->bot[attNum]->pos[1]) + R1[4]*m[0] + R1[5]*m[1] + R1[6]*m[2];
	z = R1[8]*m[0] + R1[9]*m[1] + R1[10]*m[2];

	// build new module
	this->iMobotBuildRotated(botNum, x, y, z, R1);

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->bodyPart[face1_part].bodyID, this->bot[botNum]->bodyPart[face2_part].bodyID);
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.85);
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

#ifdef ENABLE_DRAWSTUFF
/**********************************************************
	Drawstuff Functions
 **********************************************************/
void CiMobotSim::ds_drawBodies() {
	// draw the bodies
	for (int i = 0; i < this->numBot; i++) {
		for (int j = 0; j < NUM_PARTS; j++) {
			if (dBodyIsEnabled(this->bot[i]->bodyPart[j].bodyID)) {
				dsSetColor(	this->bot[i]->bodyPart[j].color[0],
							this->bot[i]->bodyPart[j].color[1],
				this->bot[i]->bodyPart[j].color[2]);
			}
			else { 
				dsSetColor(0.5,0.5,0.5);
			}
			for (int k = 0; k < this->bot[i]->bodyPart[j].num_geomID; k++) {
				ds_drawPart(this->bot[i]->bodyPart[j].geomID[k]);
			}
		}
	}
}

void CiMobotSim::ds_start() {
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {I2M(5), I2M(-10), I2M(5)};
	static float hpr[3] = {110.0, -10.0, 0.0};	// defined in degrees
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
			}
	}
}
#endif
