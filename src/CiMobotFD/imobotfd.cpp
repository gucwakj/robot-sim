#include <cmath>
#include "imobotfd.h"

using namespace std;

#ifdef ENABLE_DRAWSTUFF
/* If drawstuff is enabled, we need a global pointer to an instantiated
 * CiMobotFD class so that the callback functions can access the necessary
 * data. */
CiMobotFD *g_imobotfd;
#endif

CiMobotFD::CiMobotFD(int num_bot, int num_stp, int num_gnd, int num_targets) {
	// values for simulation passed into constructor
	this->m_num_bot = num_bot;
	this->m_num_stp = num_stp;
	this->m_num_gnd = num_gnd;
    this->m_num_targets = num_targets;

	// default simulation parameters
	this->m_mu_g = 0.4;
	this->m_mu_b = 0.3;
	this->m_cor_g = 0.3;
	this->m_cor_b = 0.3;
	this->m_t_total = 1.0;

	// variables to keep track of progress of simulation
	this->m_cur_stp = 0;
	this->m_t = 0.0;
	this->m_t_step = 0.004;
	this->m_t_tot_step = (int)((this->m_t_total + this->m_t_step) / this->m_t_step);
	this->m_t_cur_step = 0;
	this->m_flag_comp = new bool[num_bot];
	this->m_flag_disable = new bool[num_bot];
	this->m_ground = new dGeomID[num_gnd];
    this->target = new CiMobotFDTarget[num_targets];

	// initialze reply struct
	this->m_reply = new CiMobotFDReply;
	this->m_reply->time = 0.0;
	this->m_reply->message = FD_ERROR_TIME;

    // create ODE simulation space
    dInitODE2(0);                                               // initialized ode library
    this->world = dWorldCreate();                               // create world for simulation
    this->space = dHashSpaceCreate(0);                          // create space for robots
    this->group = dJointGroupCreate(0);

    // simulation parameters
    dWorldSetAutoDisableFlag(this->world, 1);                   // auto-disable bodies that are not moving
    dWorldSetAutoDisableAngularThreshold(this->world, 0.01);    // threshold velocity for defining movement
    dWorldSetAutoDisableLinearThreshold(this->world, 0.01);     // linear velocity threshold
    dWorldSetAutoDisableSteps(this->world, 4);                  // number of steps below thresholds before stationary
    dWorldSetCFM(this->world, 0.0000000001);                    // constraint force mixing - how much a joint can be violated by excess force
    dWorldSetContactSurfaceLayer(this->world, 0.001);           // depth each body can sink into another body before resting
    dWorldSetERP(this->world, 0.95);                            // error reduction parameter (0-1) - how much error is corrected on each step
    dWorldSetGravity(this->world, 0, 0, -9.81);                 // gravity

	// create instance for each module in simulation
    this->bot = new Robot * [num_bot];
    for ( int i = 0; i < num_bot; i++ ) {
        this->bot[i] = new Robot(this->world, this->space, num_stp);
    }

	#ifdef ENABLE_DRAWSTUFF
	// drawstuff simulation parameters
	m_fn.version = DS_VERSION;									// version of drawstuff
	m_fn.start = (void (*)(void))&CiMobotFD::ds_start;			// initialization function
	m_fn.step = (void (*)(int))&CiMobotFD::ds_simulationLoop;	// function for each step of simulation
	m_fn.command = (void (*)(int))&CiMobotFD::ds_command;		// keyboard commands to control simulation
	m_fn.stop = 0;												// stopping parameter
	m_fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;				// path to texture files
	g_imobotfd = this;											// pointer to class
	#endif
}

CiMobotFD::~CiMobotFD(void) {
	// free all arrays created dynamically in constructor
	for ( int i = this->m_num_bot - 1; i >= 0; i-- ) { delete this->bot[i]; }
	delete [] this->bot;
	delete [] this->m_flag_comp;
	delete [] this->m_flag_disable;
	delete [] this->m_ground;
    delete [] this->target;
	delete this->m_reply;

	// destroy all ODE objects
	dJointGroupDestroy(group);
	//for ( int i = 0; i < this->m_num_bot; i++ ) { dSpaceDestroy(this->space_bot[i]); }
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
}

/**********************************************************
	Public Member Functions
 **********************************************************/
void CiMobotFD::setAngles(dReal *ang) {
    dReal ang2[NUM_DOF*this->m_num_stp];
    for ( int i = 0; i < this->m_num_bot; i++ ) {
        for ( int j = 0; j < NUM_DOF*this->m_num_stp; j++ ) {
            ang2[j] = ang[NUM_DOF*i + NUM_DOF*this->m_num_bot*(j/NUM_DOF) + j%NUM_DOF];
        }
        this->bot[i]->setAngles(ang2);
    }
}

void CiMobotFD::setAngularVelocity(dReal *vel) {
	dReal vel2[NUM_DOF*this->m_num_stp];
	for ( int i = 0; i < this->m_num_bot; i++ ) {
        for ( int j = 0; j < NUM_DOF*this->m_num_stp; j++ ) {
            vel2[j] = vel[NUM_DOF*i + NUM_DOF*this->m_num_bot*(j/NUM_DOF) + j%NUM_DOF];
        }
        this->bot[i]->setAngularVelocity(vel2);
    }
}

void CiMobotFD::setCOR(dReal cor_g, dReal cor_b) {
	this->m_cor_g = cor_g;
	this->m_cor_b = cor_b;
}

void CiMobotFD::setMu(dReal mu_g, dReal mu_b) {
	this->m_mu_g = mu_g;
	this->m_mu_b = mu_b;
}

void CiMobotFD::setTime(dReal time_total) {
	this->m_t_total = time_total;
	this->m_t_tot_step = (int)((this->m_t_total + this->m_t_step) / this->m_t_step);
}

void CiMobotFD::setTarget(int num, double x, double y, double z) {
    this->target[num].x = x;
    this->target[num].y = y;
    this->target[num].z = z;
    this->target[num].geomID = dCreateSphere(this->space, 0.01);
    dGeomSetPosition(this->target[num].geomID, x, y, z);
    dGeomDisable(this->target[num].geomID);
}

void CiMobotFD::runSimulation(int argc, char **argv) {
	#ifdef ENABLE_DRAWSTUFF
	dsSimulationLoop(argc, argv, 352, 288, &m_fn);
	#else
	simulation_loop();
	#endif
}

int CiMobotFD::getReplyMessage(void) {
	return this->m_reply->message;
}

double CiMobotFD::getReplyTime(void) {
	return (double)(this->m_reply->time);
}

/**********************************************************
	Private Simulation Functions
 **********************************************************/
#ifdef ENABLE_DRAWSTUFF
/* If drawstuff is enabled, we cannot use any reference to 'this', because
 * simulationLoop is called as a callback function and any reference to 'this'
 * will be NULL. Instead, if we are using Drawstuff, we sholud reference the
 * global variable, g_imobotfd */
#define this g_imobotfd
void CiMobotFD::ds_simulationLoop(int pause) {
	bool loop = true;												// initialize
	this->update_angles();											// update angles for current step
	dSpaceCollide(this->space, this, &this->collision_wrapper);		// collide all geometries together
	dWorldStep(this->world, this->m_t_step);						// step world time by one
	dJointGroupEmpty(this->group);									// clear out all contact joints
	this->print_intermediate_data();								// print out incremental data
	this->set_flags();												// set flags for completion of steps
	this->increment_step();											// check whether to increment to next step
	this->end_simulation(loop);										// check whether to end simulation
	this->increment_time();											// increment time

	this->ds_drawBodies();											// draw bodies onto screen
    this->ds_drawTargets();                                         // draw targets onto screen
	if ((this->m_t_cur_step == this->m_t_tot_step+1) || !loop) dsStop();// stop simulation
}
#undef this
#else
void CiMobotFD::simulation_loop(void) {
	bool loop = true;												// initialize loop tracker
	this->set_angles();                                             // initialize angles for simulation
	while (this->m_t_cur_step <= this->m_t_tot_step && loop) {		// loop continuously until simulation is stopped
		this->update_angles();										// update angles for current step
		dSpaceCollide(this->space, this, &this->collision_wrapper);	// collide all geometries together
		dWorldStep(this->world, this->m_t_step);					// step world time by one
		dJointGroupEmpty(this->group);								// clear out all contact joints
		this->print_intermediate_data();							// print out incremental data
		this->set_flags();											// set flags for completion of steps
		this->increment_step();										// check whether to increment to next step
		this->end_simulation(loop);									// check whether to end simulation
		this->increment_time();										// increment time
	}
}
#endif

void CiMobotFD::update_angles(void) {
	// initialze loop counters
	int i, j;

	// update stored data in struct with data from ODE
	for ( i = 0; i < this->m_num_bot; i++ ) {
		// must be done for each degree of freedom
		for ( j = 0; j < NUM_DOF; j++ ) {
			// update current angle
            this->bot[i]->updateCurrentAngle(j);

			// set motor angle to current angle
			dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getCurrentAngle(j));

			// drive motor to get current angle to match future angle
			if ( dJointIsEnabled(this->bot[i]->getMotorID(j)) ) {
                this->bot[i]->updateMotorSpeed(j);
			}
		}
	}
}

void CiMobotFD::collision_wrapper(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CiMobotFD *ptr;
	ptr = (CiMobotFD *) data;
	if (ptr)
	    ptr->collision(o1, o2);

}

void CiMobotFD::collision(dGeomID o1, dGeomID o2) {
	// get bodies of geoms
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// if geom bodies are connected, do not intersect
	if ( b1 && b2 && dAreConnected(b1, b2) ) return;

	// special case for collision of spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, this, &this->collision_wrapper);
		if ( dGeomIsSpace(o1) )	dSpaceCollide((dSpaceID)o1, this, &this->collision_wrapper);
		if ( dGeomIsSpace(o2) ) dSpaceCollide((dSpaceID)o2, this, &this->collision_wrapper);
	}
	else {
		dContact contact[8];
		for ( int i = 0; i < dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact)); i++ ) {
			// different properties for ground contact vs body contact
			if ( dGeomGetSpace(o1) == this->space || dGeomGetSpace(o2) == this->space ) {
				contact[i].surface.mu = this->m_mu_g;
				contact[i].surface.bounce = this->m_cor_g;
			}
			else {
				contact[i].surface.mu = this->m_mu_b;
				contact[i].surface.bounce = this->m_cor_b;
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach( dJointCreateContact(this->world, this->group, contact + i), b1, b2);
		}
	}
}

void CiMobotFD::set_flags(void) {
	// initialze loop counters
	int c, i, j;

	// set flags for each module in simulation
	for ( i = 0; i < this->m_num_bot; i++ ) {
		// restart counter for each robot
		c = 0;

		// check if joint speed is zero -> joint has completed step
		for ( j = 0; j < NUM_DOF; j++ ) { if ( !(int)dJointGetAMotorParam(this->bot[i]->getMotorID(j), dParamVel) ) c++; }

		// set flag for each robot that has completed all four joint motions
		if ( c == 4 ) this->m_flag_comp[i] = true;

		// module is disabled
        this->m_flag_disable[i] = this->bot[i]->isDisabled();
	}
}

void CiMobotFD::increment_step(void) {
	// initialze loop counters
	int i, j;

	// if completed step and bodies are at rest, then increment
	if ( this->is_true(this->m_num_bot, this->m_flag_comp) && this->is_true(this->m_num_bot, this->m_flag_disable) ) {
		// if haven't reached last step, increment
		if ( this->m_cur_stp != this->m_num_stp - 1 ) {
			this->m_cur_stp++;
			this->set_angles();
		}
		// otherwise successfully reached last step
		else {
			this->m_reply->message = FD_SUCCESS;
			this->m_reply->time = this->m_t;
		}

		// reset all flags to zero
		for ( i = 0; i < this->m_num_bot; i++ ) {
			this->m_flag_comp[i] = false;
			this->m_flag_disable[i] = false;
            this->bot[i]->resetPID();
		}
	}
}

void CiMobotFD::set_angles(void) {
	// set arrays of angles for new step
	for ( int i = 0; i < this->m_num_bot; i++ ) {
		for ( int j = 0; j < NUM_DOF; j++ ) {
            if ( this->bot[i]->isJointDisabled(j, this->m_cur_stp) ) {
				dJointDisable(this->bot[i]->getMotorID(j));
                this->bot[i]->updateFutureAngle(j, this->m_cur_stp, 0);
				dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getCurrentAngle(j));
			}
			else {
				dJointEnable(this->bot[i]->getMotorID(j));
				this->bot[i]->updateFutureAngle(j, this->m_cur_stp, 1);
                this->bot[i]->updateJointVelocity(j, this->m_cur_stp);
				dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getCurrentAngle(j));
			}
		}
		this->bot[i]->enable();     // re-enable robots for next step
	}
}

void CiMobotFD::print_intermediate_data(void) {
	// initialze loop counters
	int i;

	//cout.width(3); cout << this->m_t_cur_step;
	//cout.width(4); cout << this->m_t_tot_step;
	cout.width(6); cout << this->m_t;
	cout.width(3); cout << this->m_cur_stp;
	cout << "\t\t";
	const dReal *pos;
    //for ( i = 0; i < 1; i++ ) {
    cout.width(10);// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	for (i = 1; i < this->m_num_bot; i++) {
		//cout << this->bot[i]->fut_ang[LE] << " ";
		//cout << this->bot[i]->body[ENDCAP_L].ang << " ";
		//cout << this->bot[i]->cur_ang[LE] << ", ";
		//cout << this->bot[i]->jnt_vel[LE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LE]) << " ";
		//
		//cout << this->bot[i]->fut_ang[LB] << " ";
		//cout << this->bot[i]->body[BODY_L].ang << " ";
		//cout << this->bot[i]->cur_ang[LB] << ", ";
		//cout << this->bot[i]->jnt_vel[LB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[LB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[LB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[LB]) << " ";
		//
		//pos = dBodyGetPosition(this->bot[i]->body[CENTER].bodyID);
		//printf("[%f %f %f]\t", pos[0], pos[1], pos[2]);
		//cout << dBodyIsEnabled(this->bot[i]->body[CENTER].bodyID) << " ";
		//
		//cout << this->bot[i]->fut_ang[RB] << " ";
		//cout << this->bot[i]->body[BODY_R].ang << " ";
		//cout << this->bot[i]->cur_ang[RB] << ", ";
		//cout << this->bot[i]->jnt_vel[RB] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RB], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RB]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RB]) << " ";
		//
		//cout << this->bot[i]->fut_ang[RE] << " ";
		//cout << this->bot[i]->body[ENDCAP_R].ang << " ";
		//cout << this->bot[i]->cur_ang[RE] << " ";
		//cout << this->bot[i]->jnt_vel[RE] << " ";
		//cout << dJointGetAMotorParam(this->bot[i]->motors[RE], dParamVel) << " ";
		//cout << dJointGetHingeAngle(this->bot[i]->joints[RE]) << " ";
		//cout << dJointGetHingeAngleRate(this->bot[i]->joints[RE]) << " ";
        //cout << "\t\t";
	}
	//cout << ">" << endl;
	//for (i = 0; i < this->m_num_bot; i++) { cout << this->m_flag_comp[i] << " "; }
	//cout << " ";
	for (i = 0; i < this->m_num_bot; i++) { cout << this->m_flag_disable[i] << " "; }
	cout << endl;
    //pos = dBodyGetPosition(this->bot[1]->body[ENDCAP_R].bodyID);
    //cout << "<" << this->target[0].x << "\t" << this->target[0].y << "\t" << this->target[0].z << ">";
    //cout << "<" << pos[0] << "\t" << pos[1] << "\t" << pos[2] << ">" << endl;
}

void CiMobotFD::end_simulation(bool &loop) {
	// increment time step
	this->m_t_cur_step++;

	// have not completed steps && stalled
	if ( !this->is_true(this->m_num_bot, this->m_flag_comp) && this->is_true(this->m_num_bot, this->m_flag_disable) ) {
		this->m_reply->message = FD_ERROR_STALL;
		loop = false;
	}
	// success on all steps
	else if ( !this->m_reply->message ) { loop = false; }
}

void CiMobotFD::increment_time(void) {
		this->m_t += this->m_t_step;
}

/**********************************************************
	Ground Functions
 **********************************************************/
void CiMobotFD::groundBox(int gndNum, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CiMobotFD::groundCapsule(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CiMobotFD::groundCylinder(int gndNum, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CiMobotFD::groundPlane(int gndNum, dReal a, dReal b, dReal c, dReal d) {
	this->m_ground[gndNum] = dCreatePlane(this->space, a, b, c, d);
}

void CiMobotFD::groundSphere(int gndNum, dReal r, dReal px, dReal py, dReal pz) {
	this->m_ground[gndNum] = dCreateSphere(this->space, r);
	dGeomSetPosition(this->m_ground[gndNum], px, py, pz);
}

/*void CiMobotFD::imobot_build_effector(dSpaceID &space, CiMobotFDPart &part, dReal x, dReal y, dReal z, dMatrix3 R, int rebuild) {
    // define parameters
    dBodyID body;
    dGeomID geom;
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 2700, END_DEPTH, END_WIDTH, END_HEIGHT );
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // create body
    //if ( !rebuild )
        //body = dBodyCreate(this->world);
    //else
        body = part.bodyID;

    // set body parameters
    //dBodySetPosition(body, x, y, z);
    //dBodySetRotation(body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    geom = dCreateBox( space, END_DEPTH, END_WIDTH - 2*END_RADIUS, END_HEIGHT );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], -m.c[1], -m.c[2] );
    part.geomID[0] = geom;

    // set geometry 2 - left box
    geom = dCreateBox( space, END_DEPTH, END_RADIUS, END_HEIGHT - 2*END_RADIUS );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], -END_WIDTH/2 + END_RADIUS/2 - m.c[1], -m.c[2] );
    part.geomID[1] = geom;

    // set geometry 3 - right box
    geom = dCreateBox( space, END_DEPTH, END_RADIUS, END_HEIGHT - 2*END_RADIUS );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], END_WIDTH/2 - END_RADIUS/2 - m.c[1], -m.c[2] );
    part.geomID[2] = geom;

    // set geometry 4 - fillet upper left
    geom = dCreateCylinder( space, END_RADIUS, END_DEPTH );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], -END_WIDTH/2 + END_RADIUS - m.c[1], END_WIDTH/2 - END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( geom, R1);
    part.geomID[3] = geom;

    // set geometry 5 - fillet upper right
    geom = dCreateCylinder( space, END_RADIUS, END_DEPTH );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], END_WIDTH/2 - END_RADIUS - m.c[1], END_WIDTH/2 - END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( geom, R1);
    part.geomID[4] = geom;

    // set geometry 6 - fillet lower right
    geom = dCreateCylinder( space, END_RADIUS, END_DEPTH );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], END_WIDTH/2 - END_RADIUS - m.c[1], -END_WIDTH/2 + END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( geom, R1);
    part.geomID[5] = geom;

    // set geometry 7 - fillet lower left
    geom = dCreateCylinder( space, END_RADIUS, END_DEPTH );
    dGeomSetBody( geom, body);
    dGeomSetOffsetPosition( geom, -m.c[0], -END_WIDTH/2 + END_RADIUS - m.c[1], -END_WIDTH/2 + END_RADIUS - m.c[2] );
    dGeomSetOffsetRotation( geom, R1);
    part.geomID[6] = geom;

    // set geometry 8 - upper effector
    geom = dCreateBox(space, 4*END_DEPTH, END_DEPTH, END_HEIGHT/3 );
    dGeomSetBody(geom, body);
    dGeomSetOffsetPosition(geom, 2*END_DEPTH - m.c[0], -m.c[1], END_HEIGHT/4 - m.c[2] );
    part.geomID[7] = geom;

    // set geometry 9 - lower effector
    geom = dCreateBox(space, 4*END_DEPTH, END_DEPTH, END_HEIGHT/3 );
    dGeomSetBody(geom, body);
    dGeomSetOffsetPosition(geom, 2*END_DEPTH - m.c[0], -m.c[1], -END_HEIGHT/4 - m.c[2] );
    part.geomID[8] = geom;

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(body, &m);

    // put into robot struct
    part.bodyID = body;
    #ifdef ENABLE_DRAWSTUFF
    part.color[0] = 0;
    part.color[1] = 0;
    part.color[2] = 1;
    #endif
}*/

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CiMobotFD::iMobotBuild(int botNum, dReal x, dReal y, dReal z) {
	this->bot[botNum]->build(x, y, z, 0, 0, 0);
}

void CiMobotFD::iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
    this->bot[botNum]->build(x, y, z, psi, theta, phi);
}

void CiMobotFD::iMobotBuild(int botNum, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    this->bot[botNum]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
}

/*void CiMobotFD::iMobotBuildAttached(int botNum, int attNum, int face1, int face2) {
	if ( fabs(this->bot[attNum]->cur_ang[LE]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[LB]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[RB]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[RE]) < DBL_EPSILON )
		//imobot_build_attached_00(botNum, attNum, face1, face2);
        this->bot[botNum]->buildAttached(attNum, face1, face2, 00);
	else
		//imobot_build_attached_10(botNum, attNum, face1, face2);
        this->bot[botNum]->buildAttached(attNum, face1, face2, 10);
}

void CiMobotFD::iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	if ( fabs(this->bot[attNum]->cur_ang[LE]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[LB]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[RB]) < DBL_EPSILON && fabs(this->bot[attNum]->cur_ang[RE]) < DBL_EPSILON	)
		imobot_build_attached_01(botNum, attNum, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		imobot_build_attached_11(botNum, attNum, face1, face2, r_le, r_lb, r_rb, r_re);
}*/

void CiMobotFD::iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {

    if ( end == ENDCAP_L )
        this->iMobotBuild(botNum, x + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);
    else
        this->iMobotBuild(botNum, x - END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach 'END' to static environment
    dJointID joint = dJointCreateFixed(this->world, 0);
    dJointAttach(joint, 0, this->bot[botNum]->getBodyID(end));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}

/*void CiMobotFD::imobot_build_attached_00(int botNum, int attNum, int face1, int face2) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, m[3] = {0};
	dMatrix3 R, R1, R_att;

	// generate rotation matrix for base robot
	rotation_matrix_from_euler_angles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

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
	x = this->bot[attNum]->pos[0] + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = this->bot[attNum]->pos[1] + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = this->bot[attNum]->pos[2] + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi));

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->body[face1_part]->getBodyID(), this->bot[botNum]->body[face2_part]->getBodyID());
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotFD::imobot_build_attached_10(int botNum, int attNum, int face1, int face2) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R_att;

	// generate rotation matrix for base robot
	rotation_matrix_from_euler_angles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

	if ( face1 == 1 && face2 == 1 ) {
		// set which body parts are to be connected
		face1_part = ENDCAP_L;
		face2_part = ENDCAP_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		m[0] = -0.5*CENTER_LENGTH					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 						- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =										+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 2 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		m[0] = -0.5*CENTER_LENGTH +	R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + 							 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		m[0] = -0.5*CENTER_LENGTH	+	2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)	+ BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)					+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 3 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_L;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						-BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 3 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		m[0] = 0.5*CENTER_LENGTH	+	2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[0]*(0.5*CENTER_LENGTH);
		m[1] = 							2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)	- BODY_WIDTH	+ R1[4]*(0.5*CENTER_LENGTH);
		m[2] =							2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)					+ R1[8]*(0.5*CENTER_LENGTH);
	}
	else if ( face1 == 4 && face2 == 5 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		m[0] = 0.5*CENTER_LENGTH +	R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[1] = 						R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +	END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
		m[2] =						R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 								 R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 2 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_L;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		m[0] = 0.5*CENTER_LENGTH				+ R1[0]*(-0.5*CENTER_LENGTH);
		m[1] = 						BODY_WIDTH	+ R1[4]*(-0.5*CENTER_LENGTH);
		m[2] =									+ R1[8]*(-0.5*CENTER_LENGTH);
	}
	else if ( face1 == 5 && face2 == 4 ) {
		// set which body parts are to be connected
		face1_part = BODY_R;
		face2_part = BODY_R;

		// generate rotation matrix
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R1, R_att, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
	x = this->bot[attNum]->pos[0] + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = this->bot[attNum]->pos[1] + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = this->bot[attNum]->pos[2] + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi));

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->body[face1_part]->getBodyID(), this->bot[botNum]->body[face2_part]->getBodyID());
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotFD::imobot_build_attached_01(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, r_e, r_b, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R_att;

	// generate rotation matrix for base robot
	rotation_matrix_from_euler_angles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

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
		dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
		dMultiply0(R3, R2, R1, 3, 3, 3);
		m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
		m[1] = -END_DEPTH - 0.5*BODY_WIDTH 	+ R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
		m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
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
	x = this->bot[attNum]->pos[0] + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = this->bot[attNum]->pos[1] + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = this->bot[attNum]->pos[2] + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->body[face1_part]->getBodyID(), this->bot[botNum]->body[face2_part]->getBodyID());
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CiMobotFD::imobot_build_attached_11(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// initialize variables
	int face1_part, face2_part;
	dReal x, y, z, psi, theta, phi, r_e, r_b, m[3];
	dMatrix3 R, R1, R2, R3, R4, R5, R6, R7, R8, R9, R_att;

	// generate rotation matrix for base robot
	rotation_matrix_from_euler_angles(R_att, this->bot[attNum]->rot[0], this->bot[attNum]->rot[1], this->bot[attNum]->rot[2]);

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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->cur_ang[LE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -this->bot[attNum]->cur_ang[LE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[LB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, this->bot[attNum]->cur_ang[LB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R2, R1, R_att, 3, 3, 3);
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
		dMultiply0(R, R3, R2, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
		dMultiply0(R, R5, R4, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
		dMultiply0(R, R7, R6, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
		dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], this->bot[attNum]->cur_ang[RB]);
		dMultiply0(R4, R3, R2, 3, 3, 3);
		dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -this->bot[attNum]->cur_ang[RE]);
		dMultiply0(R6, R5, R4, 3, 3, 3);
		dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
		dMultiply0(R8, R7, R6, 3, 3, 3);
		dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
		dMultiply0(R, R9, R8, 3, 3, 3);

		// generate offset for mass center of new module
		dRFromAxisAndAngle(R1, 0, 1, 0, -this->bot[attNum]->cur_ang[RB]);
		dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], this->bot[attNum]->cur_ang[RE]);
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
	x = this->bot[attNum]->pos[0] + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2];
	y = this->bot[attNum]->pos[1] + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2];
	z = this->bot[attNum]->pos[2] + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2];

	// build new module
	this->iMobotBuild(botNum, x, y, z, R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

	// add fixed joint to attach two modules
	dJointID joint = dJointCreateFixed(this->world, 0);
	dJointAttach(joint, this->bot[attNum]->body[face1_part]->getBodyID(), this->bot[botNum]->body[face2_part]->getBodyID());
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);
}*/

/**********************************************************
	Utility Functions
 **********************************************************/
inline dReal CiMobotFD::D2R( dReal x ) {
	return x*M_PI/180;
}

inline dReal CiMobotFD::R2D( dReal x ) {
	return x/M_PI*180;
}

bool CiMobotFD::is_true(int length, bool *a) {
	for (int i = 0; i < length; i++) {
		if ( a[i] == false )
			return false;
	}
	return true;
}

void CiMobotFD::rotation_matrix_from_euler_angles(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
	dReal	sphi = sin(phi), 		cphi = cos(phi),
			stheta = sin(theta),	ctheta = cos(theta),
			spsi = sin(psi),		cpsi = cos(psi);

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
void CiMobotFD::ds_drawBodies(void) {
	for (int i = 0; i < this->m_num_bot; i++) {
        this->bot[i]->drawRobot();
    }
}

void CiMobotFD::ds_drawTargets(void) {
    for (int i = 0; i < this->m_num_targets; i++) {
        dsSetColor(1, 0, 0);
        const dReal *position = dGeomGetPosition(this->target[i].geomID);     // get position
        const dReal *rotation = dGeomGetRotation(this->target[i].geomID);     // get rotation
        dReal r = dGeomSphereGetRadius(this->target[i].geomID);
        dsDrawSphere(position, rotation, r);
    }
}

void CiMobotFD::ds_start(void) {
	dAllocateODEDataForThread(dAllocateMaskAll);
	static float xyz[3] = {-0.35, -0.254, 0.127};		// left side
	static float hpr[3] = {45.0, -15.0, 0.0};			// defined in degrees
	dsSetViewpoint(xyz, hpr);
}

void CiMobotFD::ds_command(int cmd) {
	switch (cmd) {
		case 'q': case 'Q':
			dsStop();
	}
}
#endif