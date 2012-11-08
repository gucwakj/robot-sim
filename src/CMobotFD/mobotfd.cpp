#include <stdbool.h>
#include "mobotfd.h"
using namespace std;

CMobotFD::CMobotFD(void) {
    // create ODE simulation space
    dInitODE2(0);												// initialized ode library
    this->world = dWorldCreate();								// create world for simulation
    this->space = dHashSpaceCreate(0);							// create space for robots
    this->group = dJointGroupCreate(0);							// create group for joints
	this->ground = new dGeomID[1];								// create array for ground objects
    this->ground[0] = dCreatePlane(this->space, 0, 0, 1, 0);	// create ground plane

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
	_mu[0] = 0.4; _mu[1] = 0.3;
	_cor[0] = 0.3; _cor[1] = 0.3;

	// create simulation thread variables
	pthread_create(&(this->simulation), NULL, (void* (*)(void *))&CMobotFD::simulationThread, (void *)this);
	pthread_mutex_init(&robot_mutex, NULL);
	pthread_mutex_init(&ground_mutex, NULL);

	// variables to keep track of progress of simulation
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		this->robot[i] = NULL;
		this->robotNumber[i] = 0;
		this->robotThread[i] = NULL;
	}
	this->groundNumber = 1;
    _time_step = 0.004;



	// Graphics init
	viewer = new osgViewer::Viewer();
    /*osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 200;
    traits->y = 200;
    traits->width = 800;
    traits->height = 600;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
    if (!gw) {
        osg::notify(osg::NOTICE)<<"Error: unable to create graphics window."<<std::endl;
        //return 1;
    }

    // Creating the viewer  
    viewer->getCamera()->setGraphicsContext(gc.get());
    viewer->getCamera()->setViewport(0,0,800,600);*/

    // Creating the root node
    osg::ref_ptr<osg::Group> root (new osg::Group);

    // Add Capsule to scene
    /*osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
    myshapegeode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
    osg::ref_ptr<osg::PositionAttitudeTransform> objectPat (new osg::PositionAttitudeTransform);
    osg::Vec3f objectPosTrans = osg::Vec3f(-1,3,5);
    objectPat->addChild(myshapegeode.get());
    objectPat->setPosition(objectPosTrans);
    root->addChild(objectPat.get());*/
    // StateSet for capsule
    //osg::ref_ptr<osg::StateSet> nodess (myshapegeode->getOrCreateStateSet());
    //osg::ref_ptr<osg::Image> image (osgDB::readImageFile("wood.png"));
    //osg::ref_ptr<osg::Texture2D> tex (new osg::Texture2D);
    //tex->setImage(image.get());
    //nodess->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    //nodess->setTextureAttributeAndModes(0,tex.get(),osg::StateAttribute::ON);
    // Moving Body 2
    /*osg::ref_ptr<osg::Geode> mobotBody1 (new osg::Geode);
    osg::ref_ptr<osg::Geode> mobotBody2 (new osg::Geode);
    mobotBody1->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
    osg::ref_ptr<osg::PositionAttitudeTransform> mobotBody1PAT (new osg::PositionAttitudeTransform);
    mobotBody1PAT->addChild(mobotBody1.get());
    mobotBody1PAT->setPosition(osg::Vec3f(1,3,5));
    mobotBody1PAT->setUpdateCallback(new mobotNodeCallback(0));
    root->addChild(mobotBody1PAT.get());
    mobotBody2->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
    osg::ref_ptr<osg::PositionAttitudeTransform> mobotBody2PAT (new osg::PositionAttitudeTransform);
    mobotBody2PAT->addChild(mobotBody2.get());
    mobotBody2PAT->setPosition(osg::Vec3f(1,6,10));
    mobotBody2PAT->setUpdateCallback(new mobotNodeCallback(1));
    root->addChild(mobotBody2PAT.get());*/

    // array
    /*osg::ref_ptr<osg::Geode> mobotBody[5];
    osg::ref_ptr<osg::PositionAttitudeTransform> mobotBodyPAT[5];
    for ( int i = 0; i < 5; i++ ) {
        mobotBody[i] = new osg::Geode;
        mobotBodyPAT[i] = new osg::PositionAttitudeTransform;
        mobotBody[i]->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
        mobotBodyPAT[i]->addChild(mobotBody[i].get());
        mobotBodyPAT[i]->setPosition(osg::Vec3f(0.1+i,0.5+i,0.3+i));
        mobotBodyPAT[i]->setUpdateCallback(new mobotNodeCallback(this, i));
        root->addChild(mobotBodyPAT[i].get());
    }*/

    //Loading the terrain node
    /*osg::ref_ptr<osg::MatrixTransform> terrainScaleMat (new osg::MatrixTransform);
    osg::Matrix terrainScaleMatrix;
    terrainScaleMatrix.makeScale(0.05f,0.05f,0.03f);
    osg::Vec3f terrainScale = osg::Vec3f(0.5f,0.5f,0.5f);
    osg::ref_ptr<osg::Node> terrainnode (osgDB::readNodeFile("Terrain2.3ds"));
    terrainScaleMat->addChild(terrainnode.get());
    terrainScaleMat->setMatrix(terrainScaleMatrix);
    root->addChild(terrainScaleMat.get());*/

    // Event Handlers
    //viewer->addEventHandler( new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()) );

    // Set viewable
    viewer->setSceneData( root.get() );
	//viewer.run();
	//ViewerFrameThread viewerThread(viewer.get(), true);
	//viewerThread.startThread();
}

CMobotFD::~CMobotFD(void) {
	// free all arrays created dynamically in constructor
	delete [] this->ground;
	for ( int i = 0; i < NUM_TYPES; i++) {
		delete [] this->robot[i];
		delete [] this->robotThread[i];
	}

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
	_cor[0] = cor_g;
	_cor[1] = cor_b;
}

void CMobotFD::setMu(dReal mu_g, dReal mu_b) {
	_mu[0] = mu_g;
	_mu[1] = mu_b;
}

void CMobotFD::setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// resize ground array
	this->ground = (dGeomID *)realloc(this->ground, (this->groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position box
    this->ground[this->groundNumber] = dCreateBox(this->space, lx, ly, lz);
    dGeomSetPosition(this->ground[this->groundNumber], px, py, pz);
    dGeomSetRotation(this->ground[this->groundNumber++], R);
}

void CMobotFD::setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// resize ground array
	this->ground = (dGeomID *)realloc(this->ground, (this->groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position capsule
    this->ground[this->groundNumber] = dCreateCapsule(this->space, r, l);
    dGeomSetPosition(this->ground[this->groundNumber], px, py, pz);
    dGeomSetRotation(this->ground[this->groundNumber++], R);
}

void CMobotFD::setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// resize ground array
	this->ground = (dGeomID *)realloc(this->ground, (this->groundNumber + 1)*sizeof(dGeomID));

    // create rotation matrix
    dMatrix3 R, R_x, R_y, R_z, R_xy;
    dRFromAxisAndAngle(R_x, 1, 0, 0, 0);
    dRFromAxisAndAngle(R_y, 0, 1, 0, 0);
    dRFromAxisAndAngle(R_z, 0, 0, 1, 0);
    dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
    dMultiply0(R, R_xy, R_z, 3, 3, 3);

    // position cylinder
    this->ground[this->groundNumber] = dCreateCylinder(this->space, r, l);
    dGeomSetPosition(this->ground[this->groundNumber], px, py, pz);
    dGeomSetRotation(this->ground[this->groundNumber++], R);
}

void CMobotFD::setGroundSphere(dReal r, dReal px, dReal py, dReal pz) {
	// resize ground array
	this->ground = (dGeomID *)realloc(this->ground, (this->groundNumber + 1)*sizeof(dGeomID));

	// add sphere
    ground[groundNumber] = dCreateSphere(this->space, r);
    dGeomSetPosition(this->ground[this->groundNumber++], px, py, pz);
}

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void* CMobotFD::simulationThread(void *arg) {
	// cast to type sim 
	CMobotFD *sim = (CMobotFD *)arg;

	// initialize counters
	int i, j;

	while (1) {
		// lock array of robots for sim step
		pthread_mutex_lock(&(sim->robot_mutex));

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->robotNumber[i]; j++) {
				pthread_create(&(sim->robotThread[i][j]), NULL, (void* (*)(void *))&robotSim::simPreCollisionThreadEntry, (void *)(sim->robot[i][j]));
				pthread_join(sim->robotThread[i][j], NULL);
			}
		}

		// step world
		//pthread_mutex_lock(&(sim->ground_mutex));
		dSpaceCollide(sim->space, sim, &sim->collision);// collide all geometries together
		dWorldStep(sim->world, sim->_time_step);			// step world time by one
		dJointGroupEmpty(sim->group);					// clear out all contact joints
		//pthread_mutex_unlock(&(sim->ground_mutex));

		sim->print_intermediate_data();

		// perform post-collision updates
		//  - unlock angle and goal
		//  - check if success 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->robotNumber[i]; j++) {
				pthread_create(&(sim->robotThread[i][j]), NULL, (void* (*)(void *))&robotSim::simPostCollisionThreadEntry, (void *)(sim->robot[i][j]));
				pthread_join(sim->robotThread[i][j], NULL);
			}
		}

		// unlock array of robots to allow another to be 
		pthread_mutex_unlock(&(sim->robot_mutex));
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
		dSpaceCollide2(o1, o2, (void *)ptr, &ptr->collision);
		if ( dGeomIsSpace(o1) )	dSpaceCollide((dSpaceID)o1, (void *)ptr, &ptr->collision);
		if ( dGeomIsSpace(o2) ) dSpaceCollide((dSpaceID)o2, (void *)ptr, &ptr->collision);
	}
	else {
		dContact contact[8];
		for ( int i = 0; i < dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact)); i++ ) {
			if ( dGeomGetSpace(o1) == ptr->space || dGeomGetSpace(o2) == ptr->space ) {
				contact[i].surface.mu = ptr->_mu[0];
				contact[i].surface.bounce = ptr->_cor[0];
			}
			else {
				contact[i].surface.mu = ptr->_mu[1];
				contact[i].surface.bounce = ptr->_cor[1];
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach( dJointCreateContact(ptr->world, ptr->group, contact + i), b1, b2);
		}
	}
	//free(ptr);
}

void CMobotFD::print_intermediate_data(void) {
	// initialze loop counters
	int i;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	for (i = 0; i < this->robotNumber[IMOBOT]; i++) {
		cout << this->robot[IMOBOT][i]->getAngle(LE) << " ";
		cout << this->robot[IMOBOT][i]->getAngle(LB) << " ";
		cout << this->robot[IMOBOT][i]->getAngle(RB) << " ";
		cout << this->robot[IMOBOT][i]->getAngle(RE) << "\t";
		cout << this->robot[IMOBOT][i]->getSuccess(LE) << " ";
		cout << this->robot[IMOBOT][i]->getSuccess(LB) << " ";
		cout << this->robot[IMOBOT][i]->getSuccess(RB) << " ";
		cout << this->robot[IMOBOT][i]->getSuccess(RE) << "\t";
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
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[IMOBOT] =  (robotSim **)realloc(this->robot[IMOBOT], (this->robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	this->robot[IMOBOT][this->robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	this->robot[IMOBOT][this->robotNumber[IMOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[IMOBOT];
	this->robotThread[IMOBOT] = new pthread_t[this->robotNumber[IMOBOT]];
	// build new imobot geometry
	this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addiMobot(iMobotSim &imobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[IMOBOT] =  (robotSim **)realloc(this->robot[IMOBOT], (this->robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	this->robot[IMOBOT][this->robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	this->robot[IMOBOT][this->robotNumber[IMOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[IMOBOT];
	this->robotThread[IMOBOT] = new pthread_t[this->robotNumber[IMOBOT]];
	// build new imobot geometry
	this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[IMOBOT] =  (robotSim **)realloc(this->robot[IMOBOT], (this->robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	this->robot[IMOBOT][this->robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	this->robot[IMOBOT][this->robotNumber[IMOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[IMOBOT];
	this->robotThread[IMOBOT] = new pthread_t[this->robotNumber[IMOBOT]];
	// build new imobot geometry
	if ( base.isHome() )
		this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->buildAttached00(&base, face1, face2);
	else
		this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->buildAttached10(&base, face1, face2);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addiMobotConnected(iMobotSim &imobot, iMobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[IMOBOT] =  (robotSim **)realloc(this->robot[IMOBOT], (this->robotNumber[IMOBOT] + 1)*sizeof(robotSim *));
	this->robot[IMOBOT][this->robotNumber[IMOBOT]] = &imobot;
	// add imobot to simulation
	this->robot[IMOBOT][this->robotNumber[IMOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[IMOBOT];
	this->robotThread[IMOBOT] = new pthread_t[this->robotNumber[IMOBOT]];
	// build new imobot geometry
	if ( base.isHome() )
		this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		this->robot[IMOBOT][this->robotNumber[IMOBOT]++]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
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
void CMobotFD::addMobot(mobotSim &mobot) {
	this->addMobot(mobot, 0, 0, 0);
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z) {
	this->addMobot(mobot, x, y, z, 0, 0, 0);
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[MOBOT] =  (robotSim **)realloc(this->robot[MOBOT], (this->robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	this->robot[MOBOT][this->robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	this->robot[MOBOT][this->robotNumber[MOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[MOBOT];
	this->robotThread[MOBOT] = new pthread_t[this->robotNumber[MOBOT]];
	// build new mobot geometry
	this->robot[MOBOT][this->robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addMobot(mobotSim &mobot, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[MOBOT] =  (robotSim **)realloc(this->robot[MOBOT], (this->robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	this->robot[MOBOT][this->robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	this->robot[MOBOT][this->robotNumber[MOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[MOBOT];
	this->robotThread[MOBOT] = new pthread_t[this->robotNumber[MOBOT]];
	// build new mobot geometry
	this->robot[MOBOT][this->robotNumber[MOBOT]++]->build(x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[MOBOT] =  (robotSim **)realloc(this->robot[MOBOT], (this->robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	this->robot[MOBOT][this->robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	this->robot[MOBOT][this->robotNumber[MOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[MOBOT];
	this->robotThread[MOBOT] = new pthread_t[this->robotNumber[MOBOT]];
	// build new mobot geometry
	if ( base.isHome() )
		this->robot[MOBOT][this->robotNumber[MOBOT]++]->buildAttached00(&base, face1, face2);
	else
		this->robot[MOBOT][this->robotNumber[MOBOT]++]->buildAttached10(&base, face1, face2);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

void CMobotFD::addMobotConnected(mobotSim &mobot, mobotSim &base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&(this->robot_mutex));
	// add new imobot
	this->robot[MOBOT] =  (robotSim **)realloc(this->robot[MOBOT], (this->robotNumber[MOBOT] + 1)*sizeof(robotSim *));
	this->robot[MOBOT][this->robotNumber[MOBOT]] = &mobot;
	// add mobot to simulation
	this->robot[MOBOT][this->robotNumber[MOBOT]]->simAddRobot(this->world, this->space);
	// create new thread array for imobots
	delete this->robotThread[MOBOT];
	this->robotThread[MOBOT] = new pthread_t[this->robotNumber[MOBOT]];
	// build new mobot geometry
	if ( base.isHome() )
		this->robot[MOBOT][this->robotNumber[MOBOT]++]->buildAttached01(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	else
		this->robot[MOBOT][this->robotNumber[MOBOT]++]->buildAttached11(&base, face1, face2, r_le, r_lb, r_rb, r_re);
	// unlock robot data
	pthread_mutex_unlock(&(this->robot_mutex));
}

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
