#include <stdbool.h>
#include "irse.h"
using namespace std;

IRSE::IRSE(void) {
    // create ODE simulation space
    dInitODE2(0);										// initialized ode library
    _world = dWorldCreate();							// create world for simulation
    _space = dHashSpaceCreate(0);						// create space for robots
    _group = dJointGroupCreate(0);						// create group for joints
	_ground = new dGeomID[1];							// create array for ground objects
	_ground[0] = dCreatePlane(_space, 0, 0, 1, 0);		// create ground plane

    // simulation parameters
    dWorldSetAutoDisableFlag(_world, 1);				// auto-disable bodies that are not moving
    dWorldSetAutoDisableAngularThreshold(_world, 0.01);	// threshold velocity for defining movement
    dWorldSetAutoDisableLinearThreshold(_world, 0.01);	// linear velocity threshold
    dWorldSetAutoDisableSteps(_world, 4);				// number of steps below thresholds before stationary
    dWorldSetCFM(_world, 0.0000000001);					// constraint force mixing - how much a joint can be violated by excess force
    dWorldSetContactSurfaceLayer(_world, 0.001);		// depth each body can sink into another body before resting
    dWorldSetERP(_world, 0.95);							// error reduction parameter (0-1) - how much error is corrected on each step
    dWorldSetGravity(_world, 0, 0, -9.81);				// gravity

	// default collision parameters
	_mu[0] = 0.4;	_mu[1] = 0.3;
	_cor[0] = 0.3;	_cor[1] = 0.3;

	// thread variables
	pthread_create(&_simulation, NULL, (void* (*)(void *))&IRSE::simulationThread, (void *)this);
	pthread_mutex_init(&_robot_mutex, NULL);
	pthread_mutex_init(&_ground_mutex, NULL);

	// variables to keep track of progress of simulation
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		_robot[i] = NULL;
		_robotNumber[i] = 0;
		_robotThread[i] = NULL;
	}
	_groundNumber = 1;
    _step = 0.004;
	_clock = 0;

	graphics_init();
}

IRSE::~IRSE(void) {
	//delete [] _ground;
	for ( int i = 0; i < NUM_TYPES; i++) {
		//delete [] _robot[i];
		//delete [] _robotThread[i];
	}
	//delete [] _robot;
	//delete [] _robotThread;

	// destroy all ODE objects
	dJointGroupDestroy(_group);
	dSpaceDestroy(_space);
	dWorldDestroy(_world);
	dCloseODE();
}

int IRSE::graphics_init(void) {
    // Creating the viewer  
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();

	// window traits
	osg::GraphicsContext::WindowingSystemInterface *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi) {
		osg::notify(osg::NOTICE)<<"View::setUpViewAcrossAllScreens() : Error, no WindowSystemInterface available, cannot create windows."<<endl;
		return 1;
	}
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 250;
	traits->y = 200;
	traits->width = 800;
	traits->height = 600;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid()) {
		gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else {
		osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<endl;
		return 1;
	}
    viewer->getCamera()->setGraphicsContext(gc.get());
	viewer->getCamera()->setClearColor(osg::Vec4(0.2, 0.2, 0.4, 0.0));
    viewer->getCamera()->setViewport(0, 0, traits->width, traits->height);
	viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(1, -1, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	// set up the camera manipulators
	viewer->setCameraManipulator(new osgGA::TerrainManipulator);
	viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(1, -1, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

    // Creating the root node
	_osgRoot = new osg::Group();
	_osgRoot->setUpdateCallback(new rootNodeCallback(this, _robot, _osgRoot));


	// viewer event handlers
	viewer->addEventHandler(new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()));
    // add the thread model handler
    viewer->addEventHandler(new osgViewer::ThreadingHandler);
    // add the window size toggle handler
    viewer->addEventHandler(new osgViewer::WindowSizeHandler);
    // add the stats handler
    viewer->addEventHandler(new osgViewer::StatsHandler);

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_osgRoot);

	// Set viewable
	viewer->setSceneData(_osgRoot);
	_osgThread = new ViewerFrameThread(viewer.get(), true);
	_osgThread->startThread();

	return 0;
}

/**********************************************************
	Public Member Functions
 **********************************************************/
int IRSE::getNumberOfRobots(int type) {
	return _robotNumber[type];
}

void IRSE::setCOR(dReal cor_g, dReal cor_b) {
	_cor[0] = cor_g;
	_cor[1] = cor_b;
}

void IRSE::setMu(dReal mu_g, dReal mu_b) {
	_mu[0] = mu_g;
	_mu[1] = mu_b;
}

void IRSE::setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	pthread_mutex_lock(&_ground_mutex);

	// resize ground array
	_ground = (dGeomID *)realloc(_ground, (_groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position box
	_ground[_groundNumber] = dCreateBox(_space, lx, ly, lz);
	dGeomSetPosition(_ground[_groundNumber], px, py, pz);
	dGeomSetRotation(_ground[_groundNumber++], R);

	// unlock ground objects
	pthread_mutex_unlock(&_ground_mutex);
}

void IRSE::setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	pthread_mutex_lock(&_ground_mutex);

	// resize ground array
	_ground = (dGeomID *)realloc(_ground, (_groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position capsule
    _ground[_groundNumber] = dCreateCapsule(_space, r, l);
    dGeomSetPosition(_ground[_groundNumber], px, py, pz);
    dGeomSetRotation(_ground[_groundNumber++], R);

	// unlock ground objects
	pthread_mutex_unlock(&_ground_mutex);
}

void IRSE::setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	pthread_mutex_lock(&_ground_mutex);

	// resize ground array
	_ground = (dGeomID *)realloc(_ground, (_groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position cylinder
    _ground[_groundNumber] = dCreateCylinder(_space, r, l);
    dGeomSetPosition(_ground[_groundNumber], px, py, pz);
    dGeomSetRotation(_ground[_groundNumber++], R);

	// unlock ground objects
	pthread_mutex_unlock(&_ground_mutex);
}

void IRSE::setGroundSphere(dReal r, dReal px, dReal py, dReal pz) {
	// lock ground objects
	pthread_mutex_lock(&_ground_mutex);

	// resize ground array
	_ground = (dGeomID *)realloc(_ground, (_groundNumber + 1)*sizeof(dGeomID));

	// position sphere
    _ground[_groundNumber] = dCreateSphere(_space, r);
    dGeomSetPosition(_ground[_groundNumber++], px, py, pz);

	// unlock ground objects
	pthread_mutex_unlock(&_ground_mutex);
}

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void* IRSE::simulationThread(void *arg) {
	// cast to type sim 
	IRSE *sim = (IRSE *)arg;

	// initialize local variables
	struct timespec start_time, end_time;
	unsigned int dt;
	int i, j;

	while (1) {
		// lock array of robots for sim step
		pthread_mutex_lock(&(sim->_robot_mutex));

		// get start time of execution
		clock_gettime(CLOCK_REALTIME, &start_time);

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
		pthread_mutex_lock(&(sim->_ground_mutex));			// lock ground objects
		dSpaceCollide(sim->_space, sim, &sim->collision);	// collide all geometries together
		dWorldStep(sim->_world, sim->_step);				// step world time by one
		sim->_clock += sim->_step;							// increment world clock
		dJointGroupEmpty(sim->_group);						// clear out all contact joints
		pthread_mutex_unlock(&(sim->_ground_mutex));		// unlock ground objects

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
		clock_gettime(CLOCK_REALTIME, &end_time);
		// sleep until next step
		dt = sim->diff_nsecs(start_time, end_time);
		if ( dt < sim->_step*1000000000 ) { usleep(sim->_step*1000000 - dt/1000); }

		// unlock array of robots to allow another to be 
		pthread_mutex_unlock(&(sim->_robot_mutex));
	}
}

void IRSE::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	IRSE *ptr = (IRSE *)data;

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
				contact[i].surface.mu = ptr->_mu[0];
				contact[i].surface.bounce = ptr->_cor[0];
			}
			else {
				contact[i].surface.mu = ptr->_mu[1];
				contact[i].surface.bounce = ptr->_cor[1];
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach( dJointCreateContact(ptr->_world, ptr->_group, contact + i), b1, b2);
		}
	}
}

void IRSE::print_intermediate_data(void) {
	// initialze loop counters
	int i;
	static int j = 0;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	cout << j++*_step << " ";
	for (i = 0; i < _robotNumber[MOBOT]; i++) {
		cout << RAD2DEG(_robot[MOBOT][i]->getAngle(MOBOT_JOINT1)) << " ";
		//cout << _robot[MOBOT][i]->getAngle(MOBOT_JOINT2) << " ";
		//cout << _robot[MOBOT][i]->getAngle(MOBOT_JOINT3) << " ";
		//cout << _robot[MOBOT][i]->getAngle(MOBOT_JOINT4) << "\t";
		//cout << _robot[MOBOT][i]->getPosition(2, 0) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 1) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 2) << "\t";
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
void IRSE::addiMobot(iMobotSim &imobot) {
	this->addiMobot(imobot, 0, 0, 0);
}

void IRSE::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z) {
	this->addiMobot(imobot, x, y, z, 0, 0, 0);
}

void IRSE::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space, _clock);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	_robot[IMOBOT][_robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void IRSE::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space, _clock);
	// create new thread array for imobots
	delete _robotThread[IMOBOT];
	_robotThread[IMOBOT] = new pthread_t[_robotNumber[IMOBOT]];
	// build new imobot geometry
	_robot[IMOBOT][_robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void IRSE::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space, _clock);
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

void IRSE::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[IMOBOT] =  (robotSim **)realloc(_robot[IMOBOT], (_robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	_robot[IMOBOT][_robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	_robot[IMOBOT][_robotNumber[IMOBOT]]->simAddRobot(_world, _space, _clock);
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

/*void IRSE::iMobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
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
void IRSE::addMobot(mobotSim &mobot) {
	this->addMobot(mobot, 0, 0, 0);
}

void IRSE::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z) {
	this->addMobot(mobot, x, y, z, 0, 0, 0);
}

void IRSE::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space, _clock);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	_robot[MOBOT][_robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void IRSE::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space, _clock);
	// create new thread array for imobots
	delete _robotThread[MOBOT];
	_robotThread[MOBOT] = new pthread_t[_robotNumber[MOBOT]];
	// build new mobot geometry
	_robot[MOBOT][_robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&_robot_mutex);
}

void IRSE::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space, _clock);
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

void IRSE::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);
	// add new imobot
	_robot[MOBOT] =  (robotSim **)realloc(_robot[MOBOT], (_robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	_robot[MOBOT][_robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	_robot[MOBOT][_robotNumber[MOBOT]]->simAddRobot(_world, _space, _clock);
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

/*void IRSE::MobotAnchor(int botNum, int end, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
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
unsigned int IRSE::diff_nsecs(struct timespec t1, struct timespec t2) {
	return (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
}
