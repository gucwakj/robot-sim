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

	pthread_create(&(this->simulation), NULL, (void* (*)(void *))&CMobotFD::simulationThread, (void *)this);

	// variables to keep track of progress of simulation
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		this->m_number[i] = 0;
	}
    this->m_num_statics = 0;
    this->m_num_targets = 0;
    this->m_t_step = 0.004;

	this->bot = NULL;
	//this->bot[IMOBOT] = dynamic_cast<CiMobotSim *>(this->bot[IMOBOT]);;
	//this->bots[MOBOT] = new CMobotSim;
	//this->bots[KIDBOT] = new CKidbotSim;
	//this->bots[NXT] = new CNXTSim;
}

CMobotFD::~CMobotFD(void) {
	// free all arrays created dynamically in constructor
	delete [] this->bot;
	if ( this->m_num_statics ) delete [] this->m_statics;
    if ( this->m_num_targets ) delete [] this->m_targets;

	// destroy all ODE objects
	dJointGroupDestroy(this->group);
	dSpaceDestroy(this->space);
	dWorldDestroy(this->world);
	dCloseODE();
}

/**********************************************************
	Public Member Functions
 **********************************************************/
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

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void* CMobotFD::simulationThread(void *arg) {
	// cast to type sim 
	CMobotFD *sim = (CMobotFD *)arg;

	// initialize local variables
	//struct timespec cur_time, itime;
	//unsigned int dt;
	int i;
	pthread_t imobot[sim->m_number[IMOBOT]];

	while (1) {
		// get start time of execution
		//clock_gettime(CLOCK_REALTIME, &cur_time);

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles 
		for (i = 0; i < sim->m_number[IMOBOT]; i++) {
			pthread_create(&imobot[i], NULL, (void* (*)(void *))&CMobotFD::preCollisionThread<CiMobotSim>, (void *)(sim->bot[i]));
			pthread_join(imobot[i], NULL);
		}

		// step world
		dSpaceCollide(sim->space, sim, &sim->collision_wrapper);	// collide all geometries together
		dWorldStep(sim->world, sim->m_t_step);					// step world time by one
		dJointGroupEmpty(sim->group);								// clear out all contact joints

		//sim->print_intermediate_data();							// print out incremental data

		// perform post-collision updates
		//  - unlock angle and goal
		//  - check if success 
		for (i = 0; i < sim->m_number[IMOBOT]; i++) {
			pthread_create(&imobot[i], NULL, (void* (*)(void *))&CMobotFD::postCollisionThread<CiMobotSim>, (void *)(sim->bot[i]));
			pthread_join(imobot[i], NULL);
		}

		// check end time of execution
		//clock_gettime(CLOCK_REALTIME, &itime);
		// sleep until next step
		//dt = diff_nsecs(cur_time, itime);
		//if ( dt < 500000 ) { usleep(500 - dt/1000); }
	}
	free(sim);
}

template <class T> 
void* CMobotFD::preCollisionThread(void *arg) {
	// cast to type T
	T *bot = (T *)arg;

	// lock angle and goal
	bot->simThreadsGoalRLock();
	bot->simThreadsAngleLock();

	// update angle values
	bot->updateAngles();

	// unlock angle and goal
	bot->simThreadsAngleUnlock();
	bot->simThreadsGoalRUnlock();

	//delete bot;
	return 0;
}

template <class T> 
void* CMobotFD::postCollisionThread(void *arg) {
	// cast to type T
	T *bot = (T *)arg;

	// lock angle and goal
	bot->simThreadsGoalRLock();
	bot->simThreadsAngleLock();

	// check if complete
	bot->isComplete();

	// unlock angle and goal
	bot->simThreadsAngleUnlock();
	bot->simThreadsGoalRUnlock();

	//delete bot;
	return 0;
}

void CMobotFD::collision_wrapper(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CMobotFD *ptr = (CMobotFD *)data;
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

void CMobotFD::print_intermediate_data(void) {
	// initialze loop counters
	int i;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	for (i = 0; i < this->m_number[0]; i++) {
		cout << this->bot[i]->getAngle(LE) << " ";
		cout << this->bot[i]->getAngle(LB) << " ";
		cout << this->bot[i]->getAngle(RB) << " ";
		cout << this->bot[i]->getAngle(RE) << "\t";
	}
	cout << endl;
}

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CMobotFD::addiMobot(CiMobotSim &imobot) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
	this->bot[this->m_number[IMOBOT]++]->build(0, 0, 0, 0, 0, 0);
}

void CMobotFD::addiMobot(CiMobotSim &imobot, dReal x, dReal y, dReal z) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
	this->bot[this->m_number[IMOBOT]++]->build(x, y, z, 0, 0, 0);
}

void CMobotFD::addiMobot(CiMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
    this->bot[this->m_number[IMOBOT]++]->build(x, y, z, psi, theta, phi);
}

void CMobotFD::addiMobot(CiMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
    this->bot[this->m_number[IMOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
}

void CMobotFD::addiMobotConnected(CiMobotSim &imobot, CiMobotSim &base, int face1, int face2) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
	if ( base.isHome() )
        this->bot[this->m_number[IMOBOT]]->buildAttached00(&base, face1, face2);
    else
        this->bot[this->m_number[IMOBOT]]->buildAttached10(&base, face1, face2);
	this->m_number[IMOBOT]++;
}

void CMobotFD::addiMobotConnected(CiMobotSim &imobot, CiMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CiMobotSim **)realloc(this->bot, (this->m_number[IMOBOT] + 1)*sizeof(CiMobotSim *));
	this->bot[this->m_number[IMOBOT]] = &imobot;
	this->bot[this->m_number[IMOBOT]]->addToSim(this->world, this->space);
	if ( base.isHome() )
        this->bot[this->m_number[IMOBOT]++]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
        this->bot[this->m_number[IMOBOT]++]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
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
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
	this->bot[this->m_number[MOBOT]++]->build(0, 0, 0, 0, 0, 0);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
	this->bot[this->m_number[MOBOT]++]->build(x, y, z, 0, 0, 0);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
    this->bot[this->m_number[MOBOT]++]->build(x, y, z, psi, theta, phi);
}

void CMobotFD::addMobot(CMobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
    this->bot[this->m_number[MOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
}

void CMobotFD::addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
	if ( base.isHome() )
        this->bot[this->m_number[MOBOT]]->buildAttached00(&base, face1, face2);
    else
        this->bot[this->m_number[MOBOT]]->buildAttached10(&base, face1, face2);
	this->m_number[MOBOT]++;
}

void CMobotFD::addMobotConnected(CMobotSim &mobot, CMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	this->bot = (CMobotSim **)realloc(this->bot, (this->m_number[MOBOT] + 1)*sizeof(CMobotSim *));
	this->bot[this->m_number[MOBOT]] = &mobot;
	this->bot[this->m_number[MOBOT]]->addToSim(this->world, this->space);
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
