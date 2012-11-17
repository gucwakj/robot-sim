#include <stdbool.h>
#include "mobotfd.h"
using namespace std;

CMobotFD::CMobotFD(void) {
    // create ODE simulation space
    dInitODE2(0);                                               // initialized ode library
    _world = dWorldCreate();                               // create world for simulation
    _space = dHashSpaceCreate(0);                          // create space for robots
    _group = dJointGroupCreate(0);                         // create group for joints
    _ground = dCreatePlane(_space, 0, 0, 1, 0);       // create ground plane

    // simulation parameters
    dWorldSetAutoDisableFlag(_world, 1);                   // auto-disable bodies that are not moving
    dWorldSetAutoDisableAngularThreshold(_world, 0.01);    // threshold velocity for defining movement
    dWorldSetAutoDisableLinearThreshold(_world, 0.01);     // linear velocity threshold
    dWorldSetAutoDisableSteps(_world, 4);                  // number of steps below thresholds before stationary
    dWorldSetCFM(_world, 0.0000000001);                    // constraint force mixing - how much a joint can be violated by excess force
    dWorldSetContactSurfaceLayer(_world, 0.001);           // depth each body can sink into another body before resting
    dWorldSetERP(_world, 0.95);                            // error reduction parameter (0-1) - how much error is corrected on each step
    dWorldSetGravity(_world, 0, 0, -9.81);                 // gravity

	// default collision parameters
	_mu_g = 0.4;
	_mu_b = 0.3;
	_cor_g = 0.3;
	_cor_b = 0.3;

	// thread variables
	pthread_create(&_simulation, NULL, (void* (*)(void *))&CMobotFD::simulationThread, (void *)this);
	pthread_mutex_init(&_robot_mutex, NULL);

	// variables to keep track of progress of simulation
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		_robot[i] = NULL;
		_robotNumber[i] = 0;
		_robotThread[i] = NULL;
	}
    this->m_num_statics = 0;
    _step = 0.004;
}

CMobotFD::~CMobotFD(void) {
	// free all arrays created dynamically in constructor
	if ( this->m_num_statics ) delete [] this->m_statics;

	// destroy all ODE objects
	dJointGroupDestroy(_group);
	dSpaceDestroy(_space);
	dWorldDestroy(_world);
	dCloseODE();
}

/**********************************************************
	Public Member Functions
 **********************************************************/
void CMobotFD::setCOR(dReal cor_g, dReal cor_b) {
	_cor_g = cor_g;
	_cor_b = cor_b;
}

void CMobotFD::setMu(dReal mu_g, dReal mu_b) {
	_mu_g = mu_g;
	_mu_b = mu_b;
}

void CMobotFD::setNumStatics(int num_statics) {
    this->m_num_statics = num_statics;
    this->m_statics = new dGeomID[num_statics];
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
    this->m_statics[num] = dCreateBox(_space, lx, ly, lz);
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
    this->m_statics[num] = dCreateCapsule(_space, r, l);
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
    this->m_statics[num] = dCreateCylinder(_space, r, l);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
    dGeomSetRotation(this->m_statics[num], R);
}

void CMobotFD::setStaticSphere(int num, dReal r, dReal px, dReal py, dReal pz) {
    this->m_statics[num] = dCreateSphere(_space, r);
    dGeomSetPosition(this->m_statics[num], px, py, pz);
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
	int i, j;

	while (1) {
		// lock array of robots for sim step
		pthread_mutex_lock(&(sim->_robot_mutex));

		// get start time of execution
		//clock_gettime(CLOCK_REALTIME, &cur_time);

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotNumber[i]; j++) {
				pthread_create(&(sim->_robotThread[i][j]), NULL, (void* (*)(void *))&robotSim::simPreCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				pthread_join(sim->_robotThread[i][j], NULL);
			}
		}

		// step world
		dSpaceCollide(sim->_space, sim, &sim->collision);// collide all geometries together
		dWorldStep(sim->_world, sim->_step);			// step world time by one
		dJointGroupEmpty(sim->_group);					// clear out all contact joints

		sim->print_intermediate_data();

		// perform post-collision updates
		//  - unlock angle and goal
		//  - check if success 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotNumber[i]; j++) {
				pthread_create(&(sim->_robotThread[i][j]), NULL, (void* (*)(void *))&robotSim::simPostCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				pthread_join(sim->_robotThread[i][j], NULL);
			}
		}

		// check end time of execution
		//clock_gettime(CLOCK_REALTIME, &itime);
		// sleep until next step
		//dt = diff_nsecs(cur_time, itime);
		//if ( dt < 500000 ) { usleep(500 - dt/1000); }

		// unlock array of robots to allow another to be 
		pthread_mutex_unlock(&(sim->_robot_mutex));
	}
	free(sim);
}

void CMobotFD::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CMobotFD *ptr = (CMobotFD *)data;

	// get bodies of geoms
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// if geom bodies are connected, do not intersect
	if ( b1 && b2 && dAreConnected(b1, b2) ) return;

	// special case for collision of spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, ptr, &ptr->collision);
		if ( dGeomIsSpace(o1) )	dSpaceCollide((dSpaceID)o1, ptr, &ptr->collision);
		if ( dGeomIsSpace(o2) ) dSpaceCollide((dSpaceID)o2, ptr, &ptr->collision);
	}
	else {
		dContact contact[8];
		for ( int i = 0; i < dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact)); i++ ) {
			if ( dGeomGetSpace(o1) == ptr->_space || dGeomGetSpace(o2) == ptr->_space ) {
				contact[i].surface.mu = ptr->_mu_g;
				contact[i].surface.bounce = ptr->_cor_g;
			}
			else {
				contact[i].surface.mu = ptr->_mu_b;
				contact[i].surface.bounce = ptr->_cor_b;
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach( dJointCreateContact(ptr->_world, ptr->_group, contact + i), b1, b2);
		}
	}
}

void CMobotFD::print_intermediate_data(void) {
	// initialze loop counters
	int i;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	for (i = 0; i < _robotNumber[IMOBOT]; i++) {
		//cout << _robot[IMOBOT][i]->getAngle(IMOBOT_JOINT1) << " ";
		//cout << _robot[IMOBOT][i]->getAngle(IMOBOT_JOINT2) << " ";
		//cout << _robot[IMOBOT][i]->getAngle(IMOBOT_JOINT3) << " ";
		//cout << _robot[IMOBOT][i]->getAngle(IMOBOT_JOINT4) << "\t";
		cout << _robot[IMOBOT][i]->getPosition(2, 0) << " ";
		cout << _robot[IMOBOT][i]->getPosition(2, 1) << " ";
		cout << _robot[IMOBOT][i]->getPosition(2, 2) << "\t";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT1) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT2) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT3) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT4) << "\t";
	}
	cout << endl;
}

/**********************************************************
	Build iMobot Functions
 **********************************************************/
void CMobotFD::addiMobot(iMobotSim &imobot) {
	this->addiMobot(imobot, 0, 0, 0);
}

void CMobotFD::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z) {
	this->addiMobot(imobot, x, y, z, 0, 0, 0);
}

void CMobotFD::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	_robot[IMOBOT][_robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	_robot[IMOBOT][_robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	if ( base.isHome() )
		_robot[IMOBOT][_robotNumber[IMOBOT]++]->buildAttached00(&base, face1, face2);
	else
		_robot[IMOBOT][_robotNumber[IMOBOT]++]->buildAttached10(&base, face1, face2);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	if ( base.isHome() )
		_robot[IMOBOT][_robotNumber[IMOBOT]++]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		_robot[IMOBOT][_robotNumber[IMOBOT]++]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

/*void CMobotFD::iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    if ( end == ENDCAP_L )
        this->addiMobot(botNum, x + IMOBOT_END_DEPTH + IMOBOT_BODY_END_DEPTH + IMOBOT_BODY_LENGTH + 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);
    else
        this->addiMobot(botNum, x - IMOBOT_END_DEPTH - IMOBOT_BODY_END_DEPTH - IMOBOT_BODY_LENGTH - 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach 'END' to static environment
    dJointID joint = dJointCreateFixed(_world, 0);
    dJointAttach(joint, 0, this->bot[botNum]->getBodyID(end));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}*/

/**********************************************************
	Build Mobot Functions
 **********************************************************/
void CMobotFD::addMobot(mobotSim &mobot) {
	this->addMobot(mobot, 0, 0, 0);
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z) {
	this->addMobot(mobot, x, y, z, 0, 0, 0);
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	_robot[MOBOT][_robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	_robot[MOBOT][_robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	if ( base.isHome() )
		_robot[MOBOT][_robotNumber[MOBOT]++]->buildAttached00(&base, face1, face2);
	else
		_robot[MOBOT][_robotNumber[MOBOT]++]->buildAttached10(&base, face1, face2);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void CMobotFD::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	if ( base.isHome() )
		_robot[MOBOT][_robotNumber[MOBOT]++]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		_robot[MOBOT][_robotNumber[MOBOT]++]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

/*void CMobotFD::MobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    if ( end == ENDCAP_L )
        this->addMobot(botNum, x + IMOBOT_END_DEPTH + IMOBOT_BODY_END_DEPTH + IMOBOT_BODY_LENGTH + 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);
    else
        this->addMobot(botNum, x - IMOBOT_END_DEPTH - IMOBOT_BODY_END_DEPTH - IMOBOT_BODY_LENGTH - 0.5*IMOBOT_CENTER_LENGTH, y, z, psi, theta, psi, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach 'END' to static environment
    dJointID joint = dJointCreateFixed(_world, 0);
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
