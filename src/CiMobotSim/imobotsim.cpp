#include <cmath>
#include <iostream>
#include "imobotsim.h"

using namespace std;

extern "C" {
	//void *__dso_handle = NULL;
}

CiMobotSim::CiMobotSim(int numBot, int numStp, int numGnd, dReal tmeTot, dReal mu_g, dReal mu_b, dReal cor_g, dReal cor_b, dReal *ang, dReal *vel) {
	// initialize parameters for simulation
	int i, j;
	this->numBot = numBot;
	this->numStp = numStp;
	this->numGnd = numGnd;
	this->tmeTot = tmeTot;
	this->mu_g = mu_g;
	this->mu_b = mu_b;
	this->cor_g = cor_g;
	this->cor_b = cor_b;
	this->curStp = 1;
	this->newStp = true;
	this->tmeStp = 0.004;
	this->flags = new bool[numBot];
	this->disable = new bool[numBot];
	this->ground = new dGeomID[numGnd];
	this->space_robots = new dSpaceID[numBot];
	this->reply = new CiMobotSimReply;
	this->reply->success = false;
	this->reply->time = 0.0;
	this->reply->message = 0;
	this->bot = new CiMobotSimBot * [numBot];
	// create and populate struct for each module in simulation
	for ( i = 0; i < numBot; i++ ) {
		this->bot[i] = new CiMobotSimBot;
		this->bot[i]->bdyPts = new CiMobotSimPart[NUM_PARTS];
		this->bot[i]->bdyPts[BODY_L].geomID = new dGeomID[5];
		this->bot[i]->bdyPts[BODY_R].geomID = new dGeomID[5];
		this->bot[i]->bdyPts[CENTER].geomID = new dGeomID[3];
		this->bot[i]->bdyPts[ENDCAP_L].geomID = new dGeomID[7];
		this->bot[i]->bdyPts[ENDCAP_R].geomID = new dGeomID[7];
		this->bot[i]->joints = new dJointID[6];
		this->bot[i]->motors = new dJointID[4];
		this->bot[i]->futAng = new dReal[NUM_DOF];
		this->bot[i]->curAng = new dReal[NUM_DOF];
		this->bot[i]->pasAng = new dReal[NUM_DOF];
		this->bot[i]->delAng = new dReal[NUM_DOF];
		this->bot[i]->jntVel = new dReal[NUM_DOF];
		this->bot[i]->jntVelMax = new dReal[NUM_DOF];
		this->bot[i]->jntVelMin = new dReal[NUM_DOF];
		this->bot[i]->jntVelDel = new dReal[NUM_DOF];
		this->bot[i]->radPerEnc = new dReal[NUM_DOF];
		this->bot[i]->frcMax = new dReal[NUM_DOF];
		this->bot[i]->ang = new dReal[NUM_DOF*numStp*numBot];
		this->bot[i]->vel = new dReal[NUM_DOF*numStp*numBot];
		this->bot[i]->pos = new dReal[3];
		this->bot[i]->rot = new dReal[12];
		this->bot[i]->futEncCnt = new int[NUM_DOF];
		this->bot[i]->curEncCnt = new int[NUM_DOF];
		this->bot[i]->delEncCnt = new int[NUM_DOF];
		this->bot[i]->cmpStp = new bool[NUM_DOF];
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->futAng[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->curAng[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->pasAng[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->delAng[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVel[j] = 0;
		this->bot[i]->jntVelMax[LE] = 6.70;
		this->bot[i]->jntVelMax[LB] = 2.61;
		this->bot[i]->jntVelMax[RB] = 2.61;
		this->bot[i]->jntVelMax[RE] = 6.70;
		this->bot[i]->jntVelMin[LE] = 3.22;
		this->bot[i]->jntVelMin[LB] = 1.25;
		this->bot[i]->jntVelMin[RB] = 1.25;
		this->bot[i]->jntVelMin[RE] = 3.22;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->jntVelDel[j] = 0;
		this->bot[i]->radPerEnc[LE] = 0.02094;
		this->bot[i]->radPerEnc[LB] = 0.00873;
		this->bot[i]->radPerEnc[RB] = 0.00873;
		this->bot[i]->radPerEnc[RE] = 0.02094;
		this->bot[i]->frcMax[LE] = 0.260;
		this->bot[i]->frcMax[LB] = 1.059;
		this->bot[i]->frcMax[RB] = 1.059;
		this->bot[i]->frcMax[RE] = 0.260;
		for ( j = 0; j < NUM_DOF*numStp; j++ ) {
			this->bot[i]->ang[j] = D2R(ang[4*i + NUM_DOF*this->numBot*(j/NUM_DOF) + j%NUM_DOF]);
			this->bot[i]->vel[j] = vel[4*i + NUM_DOF*this->numBot*(j/NUM_DOF) + j%NUM_DOF];
		}
		for ( j = 0; j < 3; j++ ) this->bot[i]->pos[j] = 0;
		for ( j = 0; j < 12; j++ ) this->bot[i]->rot[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->futEncCnt[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->curEncCnt[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->delEncCnt[j] = 0;
		for ( j = 0; j < NUM_DOF; j++ ) this->bot[i]->cmpStp[j] = false;
		this->flags[i] = false;
		this->disable[i] = false;
	}

	// create ODE simulation space
	dInitODE2(0);
	this->world  = dWorldCreate();
	this->space  = dHashSpaceCreate(0);
	this->group  = dJointGroupCreate(1000000);
	for ( int i = 0; i < numBot; i++ ) {
		this->space_robots[i] = dHashSpaceCreate(this->space);
		dSpaceSetCleanup(this->space_robots[i],1);
	}
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
}

CiMobotSim::~CiMobotSim() {
	// free all arrays created dynamically in constructor
	for (int i = this->numBot - 1; i >= 0; i--) {
		for (int j = 0; j < NUM_PARTS; j++) {
			delete [] this->bot[i]->bdyPts[j].geomID;
		}
		delete [] this->bot[i]->bdyPts;
		delete [] this->bot[i]->joints;
		delete [] this->bot[i]->motors;
		delete [] this->bot[i]->futAng;
		delete [] this->bot[i]->curAng;
		delete [] this->bot[i]->pasAng;
		delete [] this->bot[i]->delAng;
		delete [] this->bot[i]->jntVel;
		delete [] this->bot[i]->jntVelMax;
		delete [] this->bot[i]->jntVelMin;
		delete [] this->bot[i]->jntVelDel;
		delete [] this->bot[i]->radPerEnc;
		delete [] this->bot[i]->frcMax;
		delete [] this->bot[i]->ang;
		delete [] this->bot[i]->vel;
		delete [] this->bot[i]->pos;
		delete [] this->bot[i]->rot;
		delete [] this->bot[i]->futEncCnt;
		delete [] this->bot[i]->curEncCnt;
		delete [] this->bot[i]->delEncCnt;
		delete [] this->bot[i]->cmpStp;
		delete [] this->bot[i];
	}
	delete [] this->bot;
	delete [] this->flags;
	delete [] this->disable;
	delete [] this->ground;

	// destroy all ODE objects
	dJointGroupDestroy(group);
	for ( int i = 0; i < this->numBot; i++ ) {
		dSpaceDestroy(this->space_robots[i]);
	}
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
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

void CiMobotSim::simulationLoop(void) {
	// initialize
	bool loop = true;

	// loop continuously until simulation is stopped
	while (loop) {
		if ( this->newStp )
			this->setAngles();		// set angles for new step
		else
			this->updateAngles();	// update angles for current step

		// collide all geoms together
		dSpaceCollide(this->space, this, &this->collisionWrapper);
		dWorldStep(this->world, this->tmeStp);
		dJointGroupEmpty(this->group);

		this->setFlags();			// set flags for completion of steps
		this->incrementStep();		// check whether to increment to next step

		// print out intermediate data for analysis
		//cout.width(3); cout << this->t / this->tmeStp;
		//cout.width(6); cout << this->t;
		//cout.width(3); cout << this->newStp;
		//cout.width(3); cout << this->curStp;
		//cout.width(3); cout << this->reply->success;
		//cout.width(6); cout << this->reply->time;
		//cout << "\t\t";
		//const dReal *pos;
		//for (int i = 0; i < this->numBot; i++) {
			//cout << this->bot[i]->futAng[LE] << " ";
			//cout << this->bot[i]->curAng[LE] << " ";
			//cout << this->bot[i]->pasAng[LE] << " ";
			//cout << this->bot[i]->futEncCnt[LE] << " ";
			//cout << this->bot[i]->curEncCnt[LE] << " ";
			//cout << this->bot[i]->radPerEnc[LE] << " ";
			//cout << this->bot[i]->jntVel[LE] << " ";
			//cout << this->bot[i]->cmpStp[LE] << " ";
			//cout << dJointGetAMotorParam(this->bot[i]->motors[LE], dParamVel) << " ";
			//cout << dJointGetHingeAngle(this->bot[i]->joints[LE]) << " ";
			//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LE]) << " ";
			//
			//cout << this->bot[i]->futAng[LB] << " ";
			//cout << this->bot[i]->curAng[LB] << " ";
			//cout << this->bot[i]->pasAng[LB] << " ";
			//cout << this->bot[i]->futEncCnt[LB] << " ";
			//cout << this->bot[i]->curEncCnt[LB] << " ";
			//cout << this->bot[i]->radPerEnc[LB] << " ";
			//cout << this->bot[i]->jntVel[LB] << " ";
			//cout << this->bot[i]->cmpStp[LB] << " ";
			//cout << dJointGetAMotorParam(this->bot[i]->motors[LB], dParamVel) << " ";
			//cout << dJointGetHingeAngle(this->bot[i]->joints[LB]) << " ";
			//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LB]) << " ";
			//			
			//pos = dBodyGetPosition(this->bot[i]->bdyPts[CENTER].bodyID);
			//printf("[%f %f %f]\t", M2I(pos[0]), M2I(pos[1]), M2I(pos[2]));
			//
			//cout << this->bot[i]->futAng[RB] << " ";
			//cout << this->bot[i]->curAng[RB] << " ";
			//cout << this->bot[i]->pasAng[RB] << " ";
			//cout << this->bot[i]->futEncCnt[RB] << " ";
			//cout << this->bot[i]->curEncCnt[RB] << " ";
			//cout << this->bot[i]->radPerEnc[RB] << " ";
			//cout << this->bot[i]->jntVel[RB] << " ";
			//cout << this->bot[i]->cmpStp[RB] << " ";
			//cout << dJointGetAMotorParam(this->bot[i]->motors[RB], dParamVel) << " ";
			//cout << dJointGetHingeAngle(this->bot[i]->joints[RB]) << " ";
			//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RB]) << " ";
			//
			//cout << this->bot[i]->futAng[RE] << " ";
			//cout << this->bot[i]->curAng[RE] << " ";
			//cout << this->bot[i]->pasAng[RE] << " ";
			//cout << this->bot[i]->futEncCnt[RE] << " ";
			//cout << this->bot[i]->curEncCnt[RE] << " ";
			//cout << this->bot[i]->radPerEnc[RE] << " ";
			//cout << this->bot[i]->jntVel[RE] << " ";
			//cout << this->bot[i]->cmpStp[RE] << " ";
			//cout << dJointGetAMotorParam(this->bot[i]->motors[RE], dParamVel) << " ";
			//cout << dJointGetHingeAngle(this->bot[i]->joints[RE]) << " ";
			//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RE]) << " ";
		//}
		/*for (int i = 0; i < this->numBot; i++) {
			cout.width(2); cout << this->flags[i];
		}*/
		//cout << " ";
		//for (int i = 0; i < this->numBot; i++) {
		//	cout << this->disable[i];
		//}
		//cout << endl;

		loop = this->endSimulation(this->tmeTot);		// check whether to end simulation
		this->incrementTime(this->tmeStp);				// increment time
	}
}

void CiMobotSim::setAngles() {
	// for each module set new angles for next step
	for (int i = 0; i < this->numBot; i++) {
		// disable new step flag to update next iteration
		this->newStp = false;
		// initialize step variable
		int step = this->curStp - 1;

		// each degree of freedom is checked individually
		for (int j = 0; j < NUM_DOF; j++) {
			// check if motor should be disabled for current step
			if ( this->bot[i]->ang[NUM_DOF*step + j] == D2R(123456789) ) {
				// disable joint
				dJointDisable(this->bot[i]->motors[j]);
				// set future angle equal to current angle
				if ( j == LE || j == RE )
					this->bot[i]->futAng[j] = angMod(this->bot[i]->curAng[j], dJointGetHingeAngle(this->bot[i]->joints[j]),	dJointGetHingeAngleRate(this->bot[i]->joints[j]));
				else
					this->bot[i]->futAng[j] = dJointGetHingeAngle(this->bot[i]->joints[j]);
				this->bot[i]->curAng[j] = this->bot[i]->futAng[j];
				this->bot[i]->pasAng[j] = this->bot[i]->curEncCnt[j] * this->bot[i]->radPerEnc[j];
				this->bot[i]->futEncCnt[j] = this->bot[i]->curEncCnt[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
				this->bot[i]->cmpStp[j] = true;
			}
			// set future values for motor to reach
			else {
				dJointEnable(this->bot[i]->motors[j]);
				// endcaps get modified angle to count continuously
				if ( j == LE || j == RE ) {
					this->bot[i]->futAng[j] += this->bot[i]->ang[NUM_DOF*step + j];
					this->bot[i]->curAng[j] = angMod(this->bot[i]->curAng[j], dJointGetHingeAngle(this->bot[i]->joints[j]), dJointGetHingeAngleRate(this->bot[i]->joints[j]));
				}
				else {
					this->bot[i]->futAng[j] = this->bot[i]->ang[NUM_DOF*step + j];
				}
				// store values for new step into struct
				this->bot[i]->pasAng[j] = this->bot[i]->curEncCnt[j] * this->bot[i]->radPerEnc[j];
				this->bot[i]->delAng[j] = this->bot[i]->futAng[j] - this->bot[i]->pasAng[j];
				this->bot[i]->delEncCnt[j] = (int)(this->bot[i]->delAng[j]/this->bot[i]->radPerEnc[j] + 0.5);
				this->bot[i]->futEncCnt[j] += this->bot[i]->delEncCnt[j];
				this->bot[i]->jntVel[j] = this->bot[i]->jntVelMin[j] + this->bot[i]->vel[NUM_DOF*step + j]*this->bot[i]->jntVelDel[j];
				dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
			}
		}
	}
}

void CiMobotSim::updateAngles() {
	// update stored data in struct with data from ODE
	for (int i = 0; i < this->numBot; i++) {
		// must be done for each degree of freedom
		for (int j = 0; j < NUM_DOF; j++) {
			// update current angle
			this->bot[i]->curAng[j] = angMod(this->bot[i]->curAng[j], dJointGetHingeAngle(this->bot[i]->joints[j]),	dJointGetHingeAngleRate(this->bot[i]->joints[j]));
			dJointSetAMotorAngle(this->bot[i]->motors[j], 0, this->bot[i]->curAng[j]);
			this->bot[i]->delAng[j] = this->bot[i]->curAng[j] - this->bot[i]->pasAng[j];
			// when delta angle is greater than one encoder step, increment encoder count
			if ( fabs(this->bot[i]->delAng[j]) >= this->bot[i]->radPerEnc[j] ) {
				this->bot[i]->delEncCnt[j] = (int)(this->bot[i]->delAng[j] / this->bot[i]->radPerEnc[j]);
				this->bot[i]->curEncCnt[j] += this->bot[i]->delEncCnt[j];
				this->bot[i]->pasAng[j] += this->bot[i]->delEncCnt[j] * this->bot[i]->radPerEnc[j];
			}
			// if joint is being driven for current step, set angles to current values
			if ( !dJointIsEnabled(this->bot[i]->motors[j]) ) {
				this->bot[i]->futAng[j] = this->bot[i]->curAng[j];
				this->bot[i]->pasAng[j] = this->bot[i]->curEncCnt[j] * this->bot[i]->radPerEnc[j];
				this->bot[i]->futEncCnt[j] = this->bot[i]->curEncCnt[j];
			}
			// otherwise drive with motor
			else {
				if (this->bot[i]->curEncCnt[j] < this->bot[i]->futEncCnt[j]) {
					dJointSetAMotorParam(this->bot[i]->motors[j], dParamVel, this->bot[i]->jntVel[j]);
					this->bot[i]->cmpStp[j] = false;
				}
				else if (this->bot[i]->curEncCnt[j] > this->bot[i]->futEncCnt[j]) {
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
		if ( !dBodyIsEnabled(this->bot[i]->bdyPts[0].bodyID) )
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
		this->newStp = true;
		// reset all flags to zero
		for (int i = 0; i < this->numBot; i++) {
			this->flags[i] = false;
			for (int j = 0; j < NUM_DOF; j++) {
				this->bot[i]->cmpStp[j] = false;
			}
		}
		// reached last step
		if ( (this->curStp - 1 == this->numStp) && (this->newStp) ) {
			// finished all steps
			this->reply->success = true;
			// record time to completion
			if ( k == 1)
				this->reply->time = this->t;
			// step back current step number
			this->curStp = numStp;
			// dont move to new step
			this->newStp = false;
			// increment counter
			k++;
		}
	}
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
		cout << "Success: all robots have completed all of their steps" << endl;
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
void CiMobotSim::buildLeftBody(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	// define parameters
	dMass m, m1, m2, m3;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// set mass of body
	dMassSetZero(&m);
	// create mass 1
	dMassSetBox(&m1, 2700, I2M(0.2), I2M(2.60), I2M(2.85) );
	dMassAdd(&m, &m1);
	// create mass 2
	dMassSetBox(&m2, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m2, I2M(0.5 + 0.1), -I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m2);
	// create mass 3
	dMassSetBox(&m3, 2700, I2M(1.0), I2M(0.125), I2M(2.85) );
	dMassTranslate(&m3, I2M(0.5 + 0.1), I2M(2.6/2 + 0.125/2), 0 );
	dMassAdd(&m, &m3);
	//dMassSetParameters( &m, 500, I2M(1), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(this->world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - face
	geom = dCreateBox( *space, I2M(0.2), I2M(2.85), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.55/2 + 0.1) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.55/2 + 0.1) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.65) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.65) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part->bodyID = body;
}

void CiMobotSim::buildRightBody(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	// define parameters
	dMass m, m1, m2, m3;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

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
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(this->world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - face
	geom = dCreateBox( *space, I2M(0.2), I2M(2.85), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.55/2 + 0.1) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - side square
	geom = dCreateBox( *space, I2M(1.55), I2M(0.875), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.55/2 + 0.1) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.65) - m.c[0], -I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - side curve
	geom = dCreateCylinder( *space, I2M(2.85/2), I2M(0.875) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.65) - m.c[0], I2M(2.85/2 - 0.875/2) - m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part->bodyID = body;
}

/*
 *	build copy of center
 */
void CiMobotSim::buildCenter(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// set mass of body
	dMassSetZero(&m);
	dMassSetCapsule(&m, 2700, 1, I2M(1.3), I2M(5.465 - 1.3 - 1.3) );
	dMassAdjust(&m, 0.24);
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(this->world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves of d-shapes
	dRFromAxisAndAngle(R1,1,0,0,M_PI/2);

	// set geometry 1 - center rectangle
	geom = dCreateBox( *space, I2M(2.865), I2M(0.125*2 + 0.8), I2M(2.6) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - side curve
	geom = dCreateCylinder( *space, I2M(1.3), I2M(0.125*2 + 0.8) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -I2M(1.4325) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[1] = geom;

	// set geometry 3 - side curve
	geom = dCreateCylinder( *space, I2M(1.3), I2M(0.125*2 + 0.8) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, I2M(1.4325) - m.c[0], -m.c[1], -m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[2] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part->bodyID = body;
}

/*
 *	build copy of endcap
 */
void CiMobotSim::buildEndcap(dSpaceID *space, CiMobotSimPart *part, dReal x, dReal y, dReal z, dMatrix3 R) {
	// define parameters
	dMass m;
	dBodyID body;
	dGeomID geom;
	dMatrix3 R1;

	// set mass of body
	dMassSetBox(&m, 2700, I2M(0.125), I2M(2.85), I2M(2.85) );
	//dMassSetParameters( &m, 500, I2M(0.45), I2M(0), I2M(0), 0.5, 0.5, 0.5, 0, 0, 0);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2] - m.c[0];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2] - m.c[1];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2] - m.c[2];

	// create body
	body = dBodyCreate(this->world);
	dBodySetPosition(body, x + m.c[0], y + m.c[1], z + m.c[2]);
	dBodySetRotation(body, R);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1,0,1,0,M_PI/2);

	// set geometry 1 - center box
	geom = dCreateBox( *space, I2M(0.125), I2M(1.15), I2M(2.85) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
	part->geomID[0] = geom;

	// set geometry 2 - left box
	geom = dCreateBox( *space, I2M(0.125), I2M(0.85), I2M(1.15) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575 + 0.85/2) - m.c[1], -m.c[2] );
	part->geomID[1] = geom;

	// set geometry 3 - right box
	geom = dCreateBox( *space, I2M(0.125), I2M(0.85), I2M(1.15) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575 + 0.85/2) - m.c[1], -m.c[2] );
	part->geomID[2] = geom;

	// set geometry 4 - fillet upper left
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575) - m.c[1], I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[3] = geom;

	// set geometry 5 - fillet upper right
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575) - m.c[1], I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[4] = geom;

	// set geometry 6 - fillet lower right
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], I2M(0.575) - m.c[1], -I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[5] = geom;

	// set geometry 7 - fillet lower left
	geom = dCreateCylinder( *space, I2M(0.85), I2M(0.125) );
	dGeomSetBody( geom, body);
	dGeomSetOffsetPosition( geom, -m.c[0], -I2M(0.575) - m.c[1], -I2M(0.575) - m.c[2] );
	dGeomSetOffsetRotation( geom, R1);
	part->geomID[6] = geom;

	// set mass center to (0,0,0) of body
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(body, &m);

	// put into robot struct
	part->bodyID = body;
}

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CiMobotSim::iMobotBuild(int botNum, dReal x, dReal y, dReal z) {
	dMatrix3 R;
	dRFromAxisAndAngle(R,0,0,1,0);
	this->iMobotBuildRotated(botNum, x, y, z, R);
}

void CiMobotSim::iMobotBuildRotated(int botNum, dReal x, dReal y, dReal z, dMatrix3 R) {
	// assign rotation matrix to variable
	for (int i = 0; i < 12; i++) this->bot[botNum]->rot[i] = R[i];

	// desired velocity of each body
	this->bot[botNum]->jntVel[LE] = this->bot[botNum]->jntVelMax[LE];
	this->bot[botNum]->jntVel[LB] = this->bot[botNum]->jntVelMax[LB];
	this->bot[botNum]->jntVel[RB] = this->bot[botNum]->jntVelMax[RB];
	this->bot[botNum]->jntVel[RE] = this->bot[botNum]->jntVelMax[RE];

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal le[6] = {-2.865/2-1.55-0.2-0.125/2, 0, 0, -2.865/2 - 1.55 - 0.2, 0, 0};
	dReal lb[6] = {-2.865/2-1.55-0.2/2, 0, 0, -2.865/2, 1.3, 0};
	dReal ce[3] = {0, 0, 0};
	dReal rb[6] = {2.865/2+1.55+0.2/2, 0, 0, 2.865/2, 1.3, 0};
	dReal re[6] = {2.865/2+1.55+0.2+0.125/2, 0, 0, 2.865/2 + 1.55 + 0.2, 0, 0};

	// build pieces of robot
	buildLeftBody(&(this->space_robots[botNum]), &(this->bot[botNum]->bdyPts[BODY_L]),	I2M(R[0]*lb[0] + R[1]*lb[1] + R[2]*lb[2] + x),
																						I2M(R[4]*lb[0] + R[5]*lb[1] + R[6]*lb[2] + y),
																						I2M(R[8]*lb[0] + R[9]*lb[1] + R[10]*lb[2] + z + 1.425), R);
	buildCenter(&(this->space_robots[botNum]), &(this->bot[botNum]->bdyPts[CENTER]),	I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x),
																						I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y),
																						I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z + 1.425), R);
	buildRightBody(&(this->space_robots[botNum]), &(this->bot[botNum]->bdyPts[BODY_R]),	I2M(R[0]*rb[0] + R[1]*rb[1] + R[2]*rb[2] + x),
																						I2M(R[4]*rb[0] + R[5]*rb[1] + R[6]*rb[2] + y),
																						I2M(R[8]*rb[0] + R[9]*rb[1] + R[10]*rb[2] + z + 1.425), R);
	buildEndcap(&(this->space_robots[botNum]), &(this->bot[botNum]->bdyPts[ENDCAP_L]),	I2M(R[0]*le[0] + R[1]*le[1] + R[2]*le[2] + x),
																						I2M(R[4]*le[0] + R[5]*le[1] + R[6]*le[2] + y),
																						I2M(R[8]*le[0] + R[9]*le[1] + R[10]*le[2] + z + 1.425), R);
	buildEndcap(&(this->space_robots[botNum]), &(this->bot[botNum]->bdyPts[ENDCAP_R]),	I2M(R[0]*re[0] + R[1]*re[1] + R[2]*re[2] + x),
																						I2M(R[4]*re[0] + R[5]*re[1] + R[6]*re[2] + y),
																						I2M(R[8]*re[0] + R[9]*re[1] + R[10]*re[2] + z + 1.425), R);
	this->bot[botNum]->pos[0] = I2M(R[0]*ce[0] + R[1]*ce[1] + R[2]*ce[2] + x);
	this->bot[botNum]->pos[1] = I2M(R[4]*ce[0] + R[5]*ce[1] + R[6]*ce[2] + y);
	this->bot[botNum]->pos[2] = I2M(R[8]*ce[0] + R[9]*ce[1] + R[10]*ce[2] + z);

	// create joint
	dJointID joint;

	// joint for left endcap to body
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[BODY_L].bodyID, this->bot[botNum]->bdyPts[ENDCAP_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x),
								I2M(R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y),
								I2M(R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*1 + R[1]*0 + R[2]*0,
								R[4]*1 + R[5]*0 + R[6]*0,
								R[8]*1 + R[9]*0 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[0] = joint;

	// joint for left body 1 to center
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[1] = joint;

	// joint for left body 2 to center
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_L].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x),
								I2M(R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y),
								I2M(R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[4] = joint;

	// joint for center to right body 1
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*1 + R[2]*0,
								R[4]*0 + R[5]*1 + R[6]*0,
								R[8]*0 + R[9]*1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[2] = joint;

	// joint for center to right body 2
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x),
								I2M(R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y),
								I2M(R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*0 + R[1]*-1 + R[2]*0,
								R[4]*0 + R[5]*-1 + R[6]*0,
								R[8]*0 + R[9]*-1 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[5] = joint;

	// joint for right body to endcap
	joint = dJointCreateHinge(this->world, 0);
	dJointAttach(joint, this->bot[botNum]->bdyPts[BODY_R].bodyID, this->bot[botNum]->bdyPts[ENDCAP_R].bodyID);
	dJointSetHingeAnchor(joint, I2M(R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x),
								I2M(R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y),
								I2M(R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z + 1.425) );
	dJointSetHingeAxis(joint,	R[0]*-1 + R[1]*0 + R[2]*0,
								R[4]*-1 + R[5]*0 + R[6]*0,
								R[8]*-1 + R[9]*0 + R[10]*0);
	dJointSetHingeParam(joint, dParamCFM, 0);
	this->bot[botNum]->joints[3] = joint;

	// create motor
	dJointID motor;

	// motor for left endcap to body joint
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bdyPts[BODY_L].bodyID, this->bot[botNum]->bdyPts[ENDCAP_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[0], R[4], R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->bot[botNum]->frcMax[LE]);
	this->bot[botNum]->motors[0] = motor;

	// motor for left body to center joint
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_L].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->bot[botNum]->frcMax[LB]);
	this->bot[botNum]->motors[1] = motor;

	// motor for center to right body 1 joint
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bdyPts[CENTER].bodyID, this->bot[botNum]->bdyPts[BODY_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->bot[botNum]->frcMax[RB]);
	this->bot[botNum]->motors[2] = motor;

	// motor for right body to endcap joint
	motor = dJointCreateAMotor(this->world, 0);
	dJointAttach(motor, this->bot[botNum]->bdyPts[BODY_R].bodyID, this->bot[botNum]->bdyPts[ENDCAP_R].bodyID);
	dJointSetAMotorMode(motor, dAMotorUser);
	dJointSetAMotorNumAxes(motor, 1);
	dJointSetAMotorAxis(motor, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(motor, 0, 0);
	dJointSetAMotorParam(motor, dParamCFM, 0);
	dJointSetAMotorParam(motor, dParamFMax, this->bot[botNum]->frcMax[RE]);
	this->bot[botNum]->motors[3] = motor;

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) {
		dBodySetDamping(this->bot[botNum]->bdyPts[i].bodyID, 0.1, 0.1);
	}
}

void CiMobotSim::iMobotBuildAttached(int botNum, int attNum, int face1, int face2) {
	dMatrix3 R, R1;
	dVector3 m;
	dReal x, y, z;
	int face1_part, face2_part;

	if ( face1 == 1 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 1 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 1 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*(2.865/2+1.55+0.2+0.125);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 2 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 2 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 2 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 2 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 3 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 3 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_L;
	}
	else if ( face1 == 3 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = -2*1.9025;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = BODY_R;
	}
	else if ( face1 == 3 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_L;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 4 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[1] = 2.865/2+1.55+0.2-1.28;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 4 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 4 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 4 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 5 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = (2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = -1*(2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 5 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*1.9025;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI);
		m[0] = 0;
		m[1] = -2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 5 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 0;
		m[1] = 2.85;
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 5 && face2 == 6 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2+0.125+2.85/2);
		m[1] = (2.865/2+1.55+0.2-1.28);
		m[2] = 0;
		face1_part = BODY_R;
		face2_part = ENDCAP_R;
	}
	else if ( face1 == 6 && face2 == 1 ) {
		dRFromAxisAndAngle(R,0,0,1,0);
		m[0] = 2*(2.865/2+1.55+0.2+0.125);
		m[1] = 0;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = ENDCAP_L;
	}
	else if ( face1 == 6 && face2 == 2 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 3 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = 2.865/2+1.55+0.2-1.28;
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_L;
	}
	else if ( face1 == 6 && face2 == 4 ) {
		dRFromAxisAndAngle(R,0,0,1,-M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = 2.865/2+1.55+0.2+0.125+2.85/2;
		m[2] = 0;
		face1_part = ENDCAP_R;
		face2_part = BODY_R;
	}
	else if ( face1 == 6 && face2 == 5 ) {
		dRFromAxisAndAngle(R,0,0,1,M_PI/2);
		m[0] = -1*(2.865/2+1.55+0.2-1.28);
		m[1] = -2.865/2-1.55-0.2-0.125-2.85/2;
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
	dJointAttach(joint, this->bot[attNum]->bdyPts[face1_part].bodyID, this->bot[botNum]->bdyPts[face2_part].bodyID);
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
