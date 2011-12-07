#include "imobotfd.h"
using namespace std;

#ifdef ENABLE_DRAWSTUFF
/* If drawstuff is enabled, we need a global pointer to an instantiated
 * CiMobotFD class so that the callback functions can access the necessary
 * data. */
CiMobotFD *g_imobotfd;
#endif

CiMobotFD::CiMobotFD(int num_bot, int num_stp) {
    // create ODE simulation space
    dInitODE2(0);                                               // initialized ode library
    this->world = dWorldCreate();                               // create world for simulation
    this->space = dHashSpaceCreate(0);                          // create space for robots
    this->group = dJointGroupCreate(0);                         // create group for joints
    this->ground = dCreatePlane(this->space, 0, 0, 1, 0);       // create ground plane

    // simulation parameters
    dWorldSetAutoDisableFlag(this->world, 1);                   // auto-disable bodies that are not moving
    dWorldSetAutoDisableAngularThreshold(this->world, 0.01);    // threshold velocity for defining movement
    dWorldSetAutoDisableLinearThreshold(this->world, 0.01);     // linear velocity threshold
    dWorldSetAutoDisableSteps(this->world, 4);                  // number of steps below thresholds before stationary
    dWorldSetCFM(this->world, 0.0000000001);                    // constraint force mixing - how much a joint can be violated by excess force
    dWorldSetContactSurfaceLayer(this->world, 0.001);           // depth each body can sink into another body before resting
    dWorldSetERP(this->world, 0.95);                            // error reduction parameter (0-1) - how much error is corrected on each step
    dWorldSetGravity(this->world, 0, 0, -9.81);                 // gravity

	// default collision parameters
	this->m_mu_g = 0.4;
	this->m_mu_b = 0.3;
	this->m_cor_g = 0.3;
	this->m_cor_b = 0.3;

	// variables to keep track of progress of simulation
	this->m_cur_stp = 0;
	this->m_t_step = 0.004;
	this->m_t_tot_step = (int)((1.0 + this->m_t_step) / this->m_t_step);
	this->m_t_cur_step = 0;
	this->m_flag_comp = new bool[num_bot];
    this->m_flag_disable = new bool[num_bot];
    this->m_num_bot = num_bot;
    this->m_num_stp = num_stp;
    this->m_num_statics = 0;
    this->m_num_targets = 0;

	// create instance for each module in simulation
    this->bot = new Robot * [num_bot];
    for ( int i = 0; i < num_bot; i++ ) {
        this->bot[i] = new Robot(this->world, this->space, num_stp);
    }

    // initialze reply struct
    this->m_reply = new CiMobotFDReply;
    this->m_reply->time = 0.0;
    this->m_reply->message = FD_ERROR_TIME;

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
	if ( this->m_num_statics ) delete [] this->m_statics;
    if ( this->m_num_targets ) delete [] this->m_targets;
	delete this->m_reply;

	// destroy all ODE objects
	dJointGroupDestroy(this->group);
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

void CiMobotFD::setNumStatics(int num_statics) {
    this->m_num_statics = num_statics;
    this->m_statics = new dGeomID[num_statics];
}

void CiMobotFD::setNumTargets(int num_targets) {
    this->m_num_targets = num_targets;
    this->m_targets = new CiMobotFDTarget[num_targets];
}

void CiMobotFD::setStaticBox(int num, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position box
    this->m_statics[num] = dCreateBox(this->space, lx, ly, lz);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
    dGeomSetRotation(this->m_statics[num], R);
}

void CiMobotFD::setStaticCapsule(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position capsule
    this->m_statics[num] = dCreateCapsule(this->space, r, l);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
    dGeomSetRotation(this->m_statics[num], R);
}

void CiMobotFD::setStaticCylinder(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position cylinder
    this->m_statics[num] = dCreateCylinder(this->space, r, l);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
    dGeomSetRotation(this->m_statics[num], R);
}

void CiMobotFD::setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz) {
    this->m_statics[num] = dCreateSphere(this->space, r);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
}

void CiMobotFD::setTarget(int num, dReal x, dReal y, dReal z) {
    this->m_targets[num].x = x;
    this->m_targets[num].y = y;
    this->m_targets[num].z = z;
    this->m_targets[num].geomID = dCreateSphere(this->space, 0.01);
    dGeomSetPosition(this->m_targets[num].geomID, x, y, z);
    dGeomDisable(this->m_targets[num].geomID);
}

void CiMobotFD::setTime(dReal time_total) {
	this->m_t_tot_step = (int)((time_total + this->m_t_step) / this->m_t_step);
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

	this->ds_drawBodies();											// draw bodies onto screen
    this->ds_drawStatics();                                         // draw ground onto screen
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
			//this->m_reply->time = this->m_t;
            this->m_reply->time = this->m_t_cur_step*this->m_t_step;
		}

		// reset all flags to zero
		for ( int i = 0; i < this->m_num_bot; i++ ) {
			this->m_flag_comp[i] = false;
			this->m_flag_disable[i] = false;
            this->bot[i]->resetPID();
		}
	}
	// increment time step
	this->m_t_cur_step++;
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
	// have not completed steps && stalled
	if ( !this->is_true(this->m_num_bot, this->m_flag_comp) && this->is_true(this->m_num_bot, this->m_flag_disable) ) {
		this->m_reply->message = FD_ERROR_STALL;
		loop = false;
	}
	// success on all steps
	else if ( !this->m_reply->message ) { loop = false; }
}

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

void CiMobotFD::iMobotBuildAttached(int botNum, int attNum, int face1, int face2) {
	if ( this->bot[attNum]->isHome() )
        this->bot[botNum]->buildAttached00(this->bot[attNum], face1, face2);
    else
        this->bot[botNum]->buildAttached10(this->bot[attNum], face1, face2);
}

void CiMobotFD::iMobotBuildAttached(int botNum, int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    if ( this->bot[attNum]->isHome() )
        this->bot[botNum]->buildAttached01(this->bot[attNum], face1, face2, r_le, r_lb, r_rb, r_re);
	else
        this->bot[botNum]->buildAttached11(this->bot[attNum], face1, face2, r_le, r_lb, r_rb, r_re);
}

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

/**********************************************************
	Utility Functions
 **********************************************************/
bool CiMobotFD::is_true(int length, bool *a) {
	for (int i = 0; i < length; i++) {
		if ( a[i] == false )
			return false;
	}
	return true;
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

void CiMobotFD::ds_drawStatics(void) {
    for (int i = 0; i < this->m_num_statics; i++) {
        const dReal *position = dGeomGetPosition(this->m_statics[i]);     // get position
        const dReal *rotation = dGeomGetRotation(this->m_statics[i]);     // get rotation
        dReal r, l;
        dVector3 sides;
        dsSetColor(0.5, 0.5, 0.5);
        switch (dGeomGetClass(this->m_statics[i])) {
            case dSphereClass:
                r = dGeomSphereGetRadius(this->m_statics[i]);
                dsDrawSphere(position, rotation, r);
                break;
            case dBoxClass:
                dGeomBoxGetLengths(this->m_statics[i], sides);
                dsDrawBox(position, rotation, sides);
                break;
            case dCylinderClass:
                dGeomCylinderGetParams(this->m_statics[i], &r, &l);
                dsDrawCylinder(position, rotation, l, r);
                break;
            case dCapsuleClass:
                dGeomCapsuleGetParams(this->m_statics[i], &r, &l);
                dsDrawCapsule(position, rotation, l, r);
                break;
        }
    }
}

void CiMobotFD::ds_drawTargets(void) {
    for (int i = 0; i < this->m_num_targets; i++) {
        const dReal *position = dGeomGetPosition(this->m_targets[i].geomID);     // get position
        const dReal *rotation = dGeomGetRotation(this->m_targets[i].geomID);     // get rotation
        dReal r = dGeomSphereGetRadius(this->m_targets[i].geomID);
        dsSetColor(1, 0, 0);
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
