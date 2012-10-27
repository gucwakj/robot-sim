#include <stdbool.h>
#include "mobotfd.h"
using namespace std;

CMobotFD::CMobotFD(void) {
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

	pthread_create(&(this->simulation), NULL, (void* (*)(void *))&CMobotFD::simulation_wrapper, (void *)this);
	//pthread_create(&visualization, NULL, (void *)vizLoop, NULL);

	// variables to keep track of progress of simulation
	//this->m_cur_stp = 0;
	//this->m_flag_comp = new bool[num_bot];
    //this->m_flag_disable = new bool[num_bot];
	//this->m_flag_comp = NULL;
	//this->m_flag_disable = NULL;
    //this->m_num_bot = num_bot;
    //this->m_num_stp = 1;
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		this->m_number[i] = 0;
	}
    this->m_num_statics = 0;
    this->m_num_targets = 0;
    this->m_t_step = 0.004;
    //this->m_t_tot_step = (int)((1.0 + this->m_t_step) / this->m_t_step);
    //this->m_t_cur_step = 0;

	this->bot = NULL;
	//this->pose = NULL;
	//this->bots[IMOBOT] = new CiMobotSim;
	//this->bots[MOBOT] = new CMobotSim;
	//this->bots[KIDBOT] = new CKidbotSim;
	//this->bots[NXT] = new CNXTSim;

    // initialze reply struct
    //this->m_reply = new CMobotFDReply;
    //this->m_reply->time = 0.0;
    //this->m_reply->message = FD_ERROR_TIME;
}

CMobotFD::~CMobotFD(void) {
	// free all arrays created dynamically in constructor
	//for ( int i = this->m_number - 1; i >= 0; i-- ) { delete this->bot[i]; }
	delete [] this->bot;
	//delete [] this->m_flag_comp;
	//delete [] this->m_flag_disable;
	if ( this->m_num_statics ) delete [] this->m_statics;
    if ( this->m_num_targets ) delete [] this->m_targets;
	//delete this->m_reply;

	// destroy all ODE objects
	dJointGroupDestroy(this->group);
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
}

/**********************************************************
	Public Member Functions
 **********************************************************/
/*void CMobotFD::setAngularVelocity(dReal *vel) {
	dReal vel2[NUM_DOF*this->m_num_stp];
	for ( int i = 0; i < this->m_num_bot; i++ ) {
        for ( int j = 0; j < NUM_DOF*this->m_num_stp; j++ ) {
            vel2[j] = vel[NUM_DOF*i + NUM_DOF*this->m_num_bot*(j/NUM_DOF) + j%NUM_DOF];
        }
        this->bot[i]->setAngularVelocity(vel2);
    }
}*/

void CMobotFD::setCOR(dReal cor_g, dReal cor_b) {
	this->m_cor_g = cor_g;
	this->m_cor_b = cor_b;
}

void CMobotFD::setMu(dReal mu_g, dReal mu_b) {
	this->m_mu_g = mu_g;
	this->m_mu_b = mu_b;
}

void CMobotFD::setNumStatics(int num_statics) {
    this->m_num_statics = num_statics;
    this->m_statics = new dGeomID[num_statics];
}

void CMobotFD::setNumTargets(int num_targets) {
    this->m_num_targets = num_targets;
    this->m_targets = new CMobotFDTarget[num_targets];
}

void CMobotFD::setStaticBox(int num, dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CMobotFD::setStaticCapsule(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CMobotFD::setStaticCylinder(int num, dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CMobotFD::setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz) {
    this->m_statics[num] = dCreateSphere(this->space, r);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
}

void CMobotFD::setTarget(int num, dReal x, dReal y, dReal z) {
    this->m_targets[num].x = x;
    this->m_targets[num].y = y;
    this->m_targets[num].z = z;
    this->m_targets[num].geomID = dCreateSphere(this->space, 0.01);
    dGeomSetPosition(this->m_targets[num].geomID, x, y, z);
    dGeomDisable(this->m_targets[num].geomID);
}

/*void CMobotFD::setTime(dReal time_total) {
	this->m_t_tot_step = (int)((time_total + this->m_t_step) / this->m_t_step);
}*/

void* CMobotFD::simulation_wrapper(void *arg) {
	CMobotFD *sim = (CMobotFD *)arg;
	sim->simulation_loop();
}

/*int CMobotFD::getReplyMessage(void) {
	return this->m_reply->message;
}

double CMobotFD::getReplyTime(void) {
	return (double)(this->m_reply->time);
}*/

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void CMobotFD::simulation_loop(void) {
	//struct timespec cur_time, itime;
	//unsigned int dt;
	//bool loop = true;												// initialize loop tracker
	int i;
	while (1) {													// loop continuously until simulation is stopped
		// prevent viz from running until next step if over
		//pthread_rw_rlock(&viz_rwlock);

		// get start time of execution
		//clock_gettime(CLOCK_REALTIME, &cur_time);

		// lock goal and angle
		for (i = 0; i < this->m_number[0]; i++) {
			this->bot[i]->simThreadsGoalRLock();
			this->bot[i]->simThreadsAngleLock();
		}
		//printf("locking done\t");

		this->update_angles();										// update angles for current step

		// step world
		dSpaceCollide(this->space, this, &this->collision_wrapper);	// collide all geometries together
		dWorldStep(this->world, this->m_t_step);					// step world time by one
		dJointGroupEmpty(this->group);								// clear out all contact joints

		this->print_intermediate_data();							// print out incremental data
		this->check_success();										// check success of current motion

		/*this->set_flags();											// set flags for completion of steps
		this->increment_step();										// check whether to increment to next step
		this->end_simulation(loop);									// check whether to end simulation*/

		// unlock goal and angle
		for (i = 0; i < this->m_number[0]; i++) {
			this->bot[i]->simThreadsAngleUnlock();
			this->bot[i]->simThreadsGoalRUnlock();
		}
		//printf("unlocking done\n");

		// check end time of execution
		//clock_gettime(CLOCK_REALTIME, &itime);
		// sleep until next step
		//dt = diff_nsecs(cur_time, itime);
		//if ( dt < 500000 ) { usleep(500 - dt/1000); }

		// allow viz to run if requesting access
		//pthread_rw_runlock(&viz_rwlock);
	}
}

void CMobotFD::check_success(void) {
	int i;
	for ( i = 0; i < this->m_number[0]; i++ ) {
		this->bot[i]->isComplete();
	}
}

void CMobotFD::update_angles(void) {
	// initialze loop counters
	int i, j;

	// update stored data in struct with data from ODE
	for ( i = 0; i < this->m_number[0]; i++ ) {

		// must be done for each degree of freedom
		for ( j = 0; j < NUM_DOF; j++ ) {
			// update current angle
			//this->bot[i]->updateAngle(j);

			// set motor angle to current angle
			//dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getAngle(j));

			// drive motor to get current angle to match future angle
			this->bot[i]->updateMotorSpeed(j);
		}
	}
}

void CMobotFD::collision_wrapper(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CMobotFD *ptr;
	ptr = (CMobotFD *) data;
	if (ptr)
	    ptr->collision(o1, o2);

}

void CMobotFD::collision(dGeomID o1, dGeomID o2) {
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

/*void CMobotFD::set_flags(void) {
	// initialze loop counters
	int c, i, j;

	// set flags for each module in simulation
	for ( i = 0; i < this->m_number[0]; i++ ) {
		// restart counter for each robot
		c = 0;

		// check if joint speed is zero -> joint has completed step
		for ( j = 0; j < NUM_DOF; j++ ) { if ( !(int)dJointGetAMotorParam(this->bot[i]->getMotorID(j), dParamVel) ) c++; }

		// set flag for each robot that has completed all four joint motions
		if ( c == 4 ) this->m_flag_comp[i] = true;

		// module is disabled
        this->m_flag_disable[i] = this->bot[i]->isDisabled();
	}
}*/

/*void CMobotFD::increment_step(void) {
	// if completed step and bodies are at rest, then increment
	if ( this->is_true(this->m_number[0], this->m_flag_comp) && this->is_true(this->m_number[0], this->m_flag_disable) ) {
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
		for ( int i = 0; i < this->m_number[0]; i++ ) {
			this->m_flag_comp[i] = false;
			this->m_flag_disable[i] = false;
            this->bot[i]->resetPID();
		}
	}
	// increment time step
	this->m_t_cur_step++;
}*/

/*void CMobotFD::set_angles(void) {
	// set arrays of angles for new step
	for ( int i = 0; i < this->m_number[0]; i++ ) {
		for ( int j = 0; j < NUM_DOF; j++ ) {
            //if ( this->bot[i]->isJointDisabled(j, this->m_cur_stp) ) {
            if ( this->bot[i]->isJointDisabled(j, 0) ) {
				dJointDisable(this->bot[i]->getMotorID(j));
                //this->bot[i]->updateFutureAngle(j, this->m_cur_stp, 0);
                this->bot[i]->updateFutureAngle(j, 0, 0);
				dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getCurrentAngle(j));
			}
			else {
				dJointEnable(this->bot[i]->getMotorID(j));
				//this->bot[i]->updateFutureAngle(j, this->m_cur_stp, 1);
				this->bot[i]->updateFutureAngle(j, 0, 1);
                //this->bot[i]->updateJointVelocity(j, 0);
				dJointSetAMotorAngle(this->bot[i]->getMotorID(j), 0, this->bot[i]->getCurrentAngle(j));
			}
		}
		this->bot[i]->enable();     // re-enable robots for next step
	}
}*/

void CMobotFD::print_intermediate_data(void) {
	// initialze loop counters
	int i;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	for (i = 0; i < this->m_number[0]; i++) {
		//cout << "bot: " << i << " success: " << this->bot[i]->getSuccess() << " enabled: " << dJointIsEnabled(this->bot[i]->getMotorID(0)) << " ";
		//cout << "bot: " << i << " vel: " << dJointGetAMotorParam(this->bot[i]->getMotorID(0), dParamVel) << " ";
		cout << this->bot[i]->getAngle(LE) << " ";
		cout << this->bot[i]->getAngle(LB) << " ";
		cout << this->bot[i]->getAngle(RB) << " ";
		cout << this->bot[i]->getAngle(RE) << "\t";
	}
	cout << endl;
}

/*void CMobotFD::end_simulation(bool &loop) {
	// have not completed steps && stalled
	if ( !this->is_true(this->m_number[0], this->m_flag_comp) && this->is_true(this->m_number[0], this->m_flag_disable) ) {
		this->m_reply->message = FD_ERROR_STALL;
		loop = false;
	}
	// success on all steps
	else if ( !this->m_reply->message ) { loop = false; }
}*/

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CMobotFD::addiMobot(CiMobotSim &mobot) {
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[IMOBOT] + 1)*sizeof(bool));
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[IMOBOT] + 1)*sizeof(bool));
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
	this->bot[this->m_number[IMOBOT]++]->build(0, 0, 0, 0, 0, 0);
}

void CMobotFD::addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[IMOBOT] + 1)*sizeof(bool));
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[IMOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
	this->bot[this->m_number[IMOBOT]++]->build(x, y, z, 0, 0, 0);
}

void CMobotFD::addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[IMOBOT] + 1)*sizeof(bool));
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[IMOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
    this->bot[this->m_number[IMOBOT]++]->build(x, y, z, psi, theta, phi);
}

void CMobotFD::addiMobot(CiMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[IMOBOT] + 1)*sizeof(bool));
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[IMOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
    this->bot[this->m_number[IMOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
}

void CMobotFD::addiMobotConnected(CiMobotSim &mobot, CiMobotSim &base, int face1, int face2) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[IMOBOT] + 1)*sizeof(bool));
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[IMOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
	if ( base.isHome() )
        this->bot[this->m_number[IMOBOT]]->buildAttached00(&base, face1, face2);
    else
        this->bot[this->m_number[IMOBOT]]->buildAttached10(&base, face1, face2);
	this->m_number[IMOBOT]++;
}

void CMobotFD::addiMobotConnected(CiMobotSim &mobot, CiMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	//this->m_flag_comp = (bool *)realloc(this->m_flag_comp, this->m_number[IMOBOT] + 1);
    //this->m_flag_disable = (bool *)realloc(this->m_flag_disable, this->m_number[IMOBOT] + 1);
	this->bot[this->m_number[IMOBOT]] = &mobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space, this, IMOBOT, this->m_number[IMOBOT]);
	if ( base.isHome() )
        this->bot[this->m_number[IMOBOT]]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
        this->bot[this->m_number[IMOBOT]]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	this->m_number[IMOBOT]++;
}

/*void CMobotFD::iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    if ( end == ENDCAP_L )
        this->addiMobot(botNum, x + IMOBOT_END_DEPTH + IMOBOT_BODY_END_DEPTH + IMOBOT_BODY_LENGTH + 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);
    else
        this->addiMobot(botNum, x - IMOBOT_END_DEPTH - IMOBOT_BODY_END_DEPTH - IMOBOT_BODY_LENGTH - 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach 'END' to static environment
    dJointID joint = dJointCreateFixed(this->world, 0);
    dJointAttach(joint, 0, this->bot[botNum]->getBodyID(end));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}*/

/**********************************************************
	Build Mobot Functions
 **********************************************************/
/*void CMobotFD::addMobot(CMobotSim &mobot) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[MOBOT] + 1)*sizeof(bool));
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[MOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
	this->bot[this->m_number[MOBOT]++]->build(0, 0, 0, 0, 0, 0);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[MOBOT] + 1)*sizeof(bool));
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[MOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
	this->bot[this->m_number[MOBOT]++]->build(x, y, z, 0, 0, 0);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[MOBOT] + 1)*sizeof(bool));
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[MOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
    this->bot[this->m_number[MOBOT]++]->build(x, y, z, psi, theta, phi);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[MOBOT] + 1)*sizeof(bool));
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[MOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
    this->bot[this->m_number[MOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
}

void CMobotFD::addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, (this->m_number[MOBOT] + 1)*sizeof(bool));
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, (this->m_number[MOBOT] + 1)*sizeof(bool));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
	if ( base.isHome() )
        this->bot[this->m_number[MOBOT]]->buildAttached00(&base, face1, face2);
    else
        this->bot[this->m_number[MOBOT]]->buildAttached10(&base, face1, face2);
	this->m_number[MOBOT]++;
}

void CMobotFD::addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->m_flag_comp = (bool *)realloc(this->m_flag_comp, this->m_number[MOBOT] + 1);
    this->m_flag_disable = (bool *)realloc(this->m_flag_disable, this->m_number[MOBOT] + 1);
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space, this->pose);
	if ( base.isHome() )
        this->bot[this->m_number[MOBOT]]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
        this->bot[this->m_number[MOBOT]]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	this->m_number[MOBOT]++;
}*/

/*void CMobotFD::MobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    if ( end == ENDCAP_L )
        this->addMobot(botNum, x + IMOBOT_END_DEPTH + IMOBOT_BODY_END_DEPTH + IMOBOT_BODY_LENGTH + 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);
    else
        this->addMobot(botNum, x - IMOBOT_END_DEPTH - IMOBOT_BODY_END_DEPTH - IMOBOT_BODY_LENGTH - 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach 'END' to static environment
    dJointID joint = dJointCreateFixed(this->world, 0);
    dJointAttach(joint, 0, this->bot[botNum]->getBodyID(end));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}*/

/**********************************************************
	Utility Functions
 **********************************************************/
// get difference in two time stamps in nanoseconds
unsigned int CMobotFD::diff_nsecs(struct timespec t1, struct timespec t2) {
	return (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
}
