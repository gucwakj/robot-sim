#include "robotsim.h"

using namespace std;
using namespace tinyxml2;

CRobotSim::CRobotSim(void) {
	// initialize ode
	init_ode();
	// initialize simulation
	init_sim();
	// initialize xml config file
	init_xml();
#ifdef ENABLE_GRAPHICS
	// initialize graphics
	init_viz();
#endif
}

CRobotSim::~CRobotSim(void) {
	// remove simulation
	THREAD_CANCEL(_simulation);

	// remove graphics
	delete _osgThread;

	// remove ground
	delete [] _ground;

	// remove robots
	for ( int i = NUM_TYPES - 1; i >= 0; i--) {
		delete [] _robot[i];
		for (int j = _robotNumber[i] - 1; j >= 0; j--) {
			THREAD_CANCEL(_robotThread[i][j]);
		}
		delete [] _robotThread[i];
	}
	delete [] _robot;

	// remove ode
	dJointGroupDestroy(_group);
	dSpaceDestroy(_space);
	dWorldDestroy(_world);
	dCloseODE();
}

int CRobotSim::init_ode(void) {
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

	// success
	return 0;
}

int CRobotSim::init_sim(void) {
	// default collision parameters
	_mu[0] = 0.4;	_mu[1] = 0.3;
	_cor[0] = 0.3;	_cor[1] = 0.3;

	// thread variables
	THREAD_CREATE(&_simulation, (void* (*)(void *))&CRobotSim::simulationThread, (void *)this);
	MUTEX_INIT(&_robot_mutex);
	MUTEX_INIT(&_ground_mutex);

	// variables to keep track of progress of simulation
	_groundNumber = 1;
    _step = 0.004;
	_clock = 0;

	// success
	return 0;
}

int CRobotSim::init_xml(void) {
	// initialize variables
	int i, *rtmp, *ftmp, *ntmp, ctype, cnum;
	bot = NULL;
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		_robot[i] = NULL;
		_robotNumber[i] = 0;
		_robotConnected[i] = 0;
		_robotThread[i] = NULL;
	}
	XMLElement *ele = NULL;
	XMLElement *side = NULL;

	// load xml config file
	XMLDocument doc;
	doc.LoadFile("robotsim.xml");

	// get root node of xml file
	XMLElement *node = doc.FirstChildElement("sim")->FirstChildElement();

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "mobot") ) {
			bot_t nr = (bot_t)malloc(sizeof(struct bot_s));
			nr->type = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0; nr->angle4 = 0;
			_robotNumber[MOBOT]++;
			node->QueryIntAttribute("id", &(nr->id));
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if (ele = node->FirstChildElement("rotation")) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if (ele = node->FirstChildElement("joint")) {
				ele->QueryDoubleAttribute("a1", &(nr->angle1));
				ele->QueryDoubleAttribute("a2", &(nr->angle2));
				ele->QueryDoubleAttribute("a3", &(nr->angle3));
				ele->QueryDoubleAttribute("a4", &(nr->angle4));
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			bot_t rtmp = bot;
			if ( bot == NULL )
				bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "linkboti") ) {
			bot_t nr = (bot_t)malloc(sizeof(struct bot_s));
			nr->type = LINKBOTI;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			_robotNumber[nr->type]++;
			node->QueryIntAttribute("id", &(nr->id));
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if (ele = node->FirstChildElement("rotation")) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if (ele = node->FirstChildElement("joint")) {
				ele->QueryDoubleAttribute("f1", &(nr->angle1));
				ele->QueryDoubleAttribute("f2", &(nr->angle2));
				ele->QueryDoubleAttribute("f3", &(nr->angle3));
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			bot_t rtmp = bot;
			if ( bot == NULL )
				bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			bot_t nr = (bot_t)malloc(sizeof(struct bot_s));
			nr->type = LINKBOTL;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			_robotNumber[nr->type]++;
			node->QueryIntAttribute("id", &(nr->id));
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if (ele = node->FirstChildElement("rotation")) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if (ele = node->FirstChildElement("joint")) {
				ele->QueryDoubleAttribute("f1", &(nr->angle1));
				ele->QueryDoubleAttribute("f2", &(nr->angle2));
				ele->QueryDoubleAttribute("f3", &(nr->angle3));
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			bot_t rtmp = bot;
			if ( bot == NULL )
				bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else {
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = CASTER;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "l") ) {
				ctype = L;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				ctype = SIMPLE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "smallwheel") ) {
				ctype = SMALLWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "square") ) {
				ctype = SQUARE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "tank") ) {
				ctype = TANK;
				cnum = 3;
			}
			rtmp = new int[cnum];
			ftmp = new int[cnum];
			ntmp = new int[cnum];

			// store connector to temp variables
			side = node->FirstChildElement();
			int i = 0;
			while (side) {
				side->QueryIntAttribute("id", &ntmp[i]);
				side->QueryIntAttribute("robot", &rtmp[i]);
				side->QueryIntAttribute("face", &ftmp[i++]);
				side = side->NextSiblingElement();
			}

			// store connectors to each robot
			bot_t tmp;
			Conn_t *ctmp;
			for (int j = 0; j < i; j++) {
				Conn_t *nc = (Conn_t *)malloc(sizeof(struct Conn_s));
				nc->robot = rtmp[0];
				nc->face1 = ftmp[0];
				nc->face2 = ftmp[j];
				nc->side = ntmp[j];
				nc->type = ctype;
				nc->next = NULL;
				tmp = bot;
				while (tmp && tmp->id != rtmp[j])
					tmp = tmp->next;
				if (tmp == NULL) { printf("ERROR: robot %d could not be found.\n", rtmp[j]); exit(1); }
				ctmp = tmp->conn;
				if ( tmp->conn == NULL )
					tmp->conn = nc;
				else {
					while (ctmp->next)
						ctmp = ctmp->next;
					ctmp->next = nc;
				}
			}
			delete rtmp;
			delete ftmp;
			delete ntmp;
		}

		// debug printing
		/*bot_t rtmp = bot;
		while (rtmp) {
			printf("type = %d, id = %d\n", rtmp->type, rtmp->id);
			printf("x = %lf, y = %lf, z = %lf\n", rtmp->x, rtmp->y, rtmp->z);
			printf("psi = %lf, theta = %lf, phi = %lf\n", rtmp->psi, rtmp->theta, rtmp->phi);
			printf("angle1 = %lf, angle2 = %lf, angle3 = %lf, angle4 = %lf\n", \
				rtmp->angle1, rtmp->angle2, rtmp->angle3, rtmp->angle4);
			Conn_t *ctmp = rtmp->conn;
			while (ctmp) {
				printf("on face %d: connect with robot %d on his face %d with type: %d\n", \
					ctmp->face2, ctmp->robot, ctmp->face1, ctmp->type);
				ctmp = ctmp->next;
			}
			printf("next = %p\n", rtmp->next);
			printf("\n");
			rtmp = rtmp->next;
		}
		printf("\n\n\n");*/

		// go to next node
		node = node->NextSiblingElement();
	}

	// set up robot variables
	for (int i = 0; i < NUM_TYPES; i++) {
		_robotThread[i] = new THREAD_T[_robotNumber[i]];
		_robot[i] =  (CRobot **)realloc(_robot[i], (_robotNumber[i] + 1)*sizeof(CRobot *));
	}

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CRobotSim::init_viz(void) {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

    // creating the viewer
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
		gc->setClearColor(osg::Vec4f(0.f,0.f,0.f,0.f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else {
		osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<endl;
		return 1;
	}
    viewer->getCamera()->setGraphicsContext(gc.get());
	viewer->getCamera()->setClearColor(osg::Vec4(0.2, 0.2, 0.4, 0.0));
    viewer->getCamera()->setViewport(0, 0, traits->width, traits->height);
	viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(1, 0, 0.8), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	// set up the camera manipulators
	//viewer->setCameraManipulator(new osgGA::TerrainManipulator);
	//viewer->setCameraManipulator(new osgGA::SphericalManipulator);
	viewer->setCameraManipulator(new osgGA::FirstPersonManipulator);
	//viewer->setCameraManipulator(new osgGA::TrackballManipulator);
	viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.7, 0.7, 0.4), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

    // Creating the root node
	_osgRoot = new osg::Group();
	_osgRoot->setUpdateCallback(new rootNodeCallback(this, _robot, _osgRoot));

	// load terrain node
	osg::ref_ptr<osg::MatrixTransform> terrainScaleMAT (new osg::MatrixTransform);
	osg::Matrix terrainScaleMatrix;
	terrainScaleMatrix.makeScale(0.1f,0.1f,0.006f);
	osg::ref_ptr<osg::Node> terrainnode = osgDB::readNodeFile("/usr/local/ch/package/chrobotsim/data/ground/terrain.3ds");
	terrainScaleMAT->addChild(terrainnode.get());
	terrainScaleMAT->setMatrix(terrainScaleMatrix);
	_osgRoot->addChild(terrainScaleMAT.get());

	// skybox
	osg::StateSet* stateset = new osg::StateSet();
	osg::TexEnv* te = new osg::TexEnv;
	te->setMode(osg::TexEnv::REPLACE);
	stateset->setTextureAttributeAndModes(0, te, osg::StateAttribute::ON);
	osg::TexGen *tg = new osg::TexGen;
	tg->setMode(osg::TexGen::NORMAL_MAP);
	stateset->setTextureAttributeAndModes(0, tg, osg::StateAttribute::ON);
	osg::TexMat *tm = new osg::TexMat;
	stateset->setTextureAttribute(0, tm);
	//osg::TextureCubeMap* skymap = readCubeMap();
    osg::TextureCubeMap* skymap = new osg::TextureCubeMap;
    #define SKY_FILENAME(face) "/usr/local/ch/package/chrobotsim/data/ground/checkered/" #face ".jpg"
	osg::Image* imagePosX = osgDB::readImageFile(SKY_FILENAME(checkered_front));
	osg::Image* imageNegX = osgDB::readImageFile(SKY_FILENAME(checkered_back));
	osg::Image* imagePosY = osgDB::readImageFile(SKY_FILENAME(checkered_top));
	osg::Image* imageNegY = osgDB::readImageFile(SKY_FILENAME(checkered_top));
	osg::Image* imagePosZ = osgDB::readImageFile(SKY_FILENAME(checkered_left));
	osg::Image* imageNegZ = osgDB::readImageFile(SKY_FILENAME(checkered_right));

	if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ) {
		skymap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);
		skymap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
		skymap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		skymap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	}

	stateset->setTextureAttributeAndModes(0, skymap, osg::StateAttribute::ON);
	stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	stateset->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
	// clear the depth to the far plane.
	osg::Depth* depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setRange(1.0,1.0);   
	stateset->setAttributeAndModes(depth, osg::StateAttribute::ON );
	stateset->setRenderBinDetails(-1,"RenderBin");
	osg::Drawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),1));
	osg::Geode* geode = new osg::Geode;
	geode->setCullingActive(false);
	geode->setStateSet( stateset );
	geode->addDrawable(drawable);
	osg::Transform* transform = new MoveEarthySkyWithEyePointTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);
	osg::ClearNode* clearNode = new osg::ClearNode;
	//clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TexMatCallback(*tm));
	clearNode->addChild(transform);
	_osgRoot->addChild(clearNode);

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
#endif // ENABLE_GRAPHICS

/**********************************************************
	Public Member Functions
 **********************************************************/
int CRobotSim::getNumberOfRobots(int type) {
	return _robotConnected[type];
}

void CRobotSim::setCOR(dReal cor_g, dReal cor_b) {
	_cor[0] = cor_g;
	_cor[1] = cor_b;
}

void CRobotSim::setMu(dReal mu_g, dReal mu_b) {
	_mu[0] = mu_g;
	_mu[1] = mu_b;
}

int CRobotSim::setExitState(int state) {
	while (state == 1) {
		usleep(1000000);
	}

	return 0;
}

void CRobotSim::setGroundBox(dReal lx, dReal ly, dReal lz, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	MUTEX_LOCK(&_ground_mutex);

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
	MUTEX_UNLOCK(&_ground_mutex);
}

void CRobotSim::setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	MUTEX_LOCK(&_ground_mutex);

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
	MUTEX_UNLOCK(&_ground_mutex);
}

void CRobotSim::setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
	// lock ground objects
	MUTEX_LOCK(&_ground_mutex);

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
	MUTEX_UNLOCK(&_ground_mutex);
}

void CRobotSim::setGroundSphere(dReal r, dReal px, dReal py, dReal pz) {
	// lock ground objects
	MUTEX_LOCK(&_ground_mutex);

	// resize ground array
	_ground = (dGeomID *)realloc(_ground, (_groundNumber + 1)*sizeof(dGeomID));

	// position sphere
    _ground[_groundNumber] = dCreateSphere(_space, r);
    dGeomSetPosition(_ground[_groundNumber++], px, py, pz);

	// unlock ground objects
	MUTEX_UNLOCK(&_ground_mutex);
}

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void* CRobotSim::simulationThread(void *arg) {
	// cast to type sim 
	CRobotSim *sim = (CRobotSim *)arg;

	// initialize local variables
	struct timespec start_time, end_time;
	unsigned int dt;
	int i, j;

	while (1) {
		// lock array of robots for sim step
		MUTEX_LOCK(&(sim->_robot_mutex));

		// get start time of execution
		clock_gettime(CLOCK_REALTIME, &start_time);

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				THREAD_CREATE(&(sim->_robotThread[i][j]), (void* (*)(void *))&CRobot::simPreCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				THREAD_JOIN(sim->_robotThread[i][j]);
			}
		}

		// step world
		MUTEX_LOCK(&(sim->_ground_mutex));
		dSpaceCollide(sim->_space, sim, &sim->collision);	// collide all geometries together
		dWorldStep(sim->_world, sim->_step);				// step world time by one
		sim->_clock += sim->_step;							// increment world clock
		dJointGroupEmpty(sim->_group);						// clear out all contact joints
		MUTEX_UNLOCK(&(sim->_ground_mutex));

		sim->print_intermediate_data();

		// perform post-collision updates
		//  - unlock angle and goal
		//  - check if success 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				THREAD_CREATE(&(sim->_robotThread[i][j]), (void* (*)(void *))&CRobot::simPostCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				THREAD_JOIN(sim->_robotThread[i][j]);
			}
		}

		// unlock array of robots to allow another to be 
		MUTEX_UNLOCK(&(sim->_robot_mutex));

		// check end time of execution
		clock_gettime(CLOCK_REALTIME, &end_time);
		// sleep until next step
		dt = sim->diff_nsecs(start_time, end_time);
		if ( dt < sim->_step*1000000000 ) { usleep(sim->_step*1000000 - dt/1000); }
	}
}

void CRobotSim::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	CRobotSim *ptr = (CRobotSim *)data;

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

void CRobotSim::print_intermediate_data(void) {
	// initialze loop counters
	static int j = 0;

    cout.width(10);		// cout.precision(4);
    cout.setf(ios::fixed, ios::floatfield);
	cout << j++*_step << " ";
	cout << "LinkbotI:" << "\t";
	for (int i = 0; i < _robotConnected[LINKBOTI]; i++) {
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(MOBOT_JOINT4)) << " ";
		cout << _robot[LINKBOTI][i]->getAngle(LINKBOT_JOINT1) << " ";
		cout << _robot[LINKBOTI][i]->getAngle(LINKBOT_JOINT2) << " ";
		cout << _robot[LINKBOTI][i]->getAngle(LINKBOT_JOINT3) << "\t";
		//cout << _robot[MOBOT][i]->getPosition(2, 0) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 1) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 2) << "\t";
		cout << _robot[LINKBOTI][i]->getSuccess(LINKBOT_JOINT1) << " ";
		cout << _robot[LINKBOTI][i]->getSuccess(LINKBOT_JOINT2) << " ";
		cout << _robot[LINKBOTI][i]->getSuccess(LINKBOT_JOINT3) << "\t";
	}
	//cout << endl;
	cout << "LinkbotL:" << "\t";
	for (int i = 0; i < _robotConnected[LINKBOTL]; i++) {
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(MOBOT_JOINT4)) << " ";
		cout << _robot[LINKBOTL][i]->getAngle(LINKBOT_JOINT1) << " ";
		cout << _robot[LINKBOTL][i]->getAngle(LINKBOT_JOINT2) << " ";
		cout << _robot[LINKBOTL][i]->getAngle(LINKBOT_JOINT3) << "\t";
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
	Add Robot Functions
 **********************************************************/
#ifdef _CH_
int CRobotSim::addRobot(...) {
#else
int CRobotSim::addRobot(CRobot &robot) {
#endif
	// get type of robot being added
	int type = robot.getType();
	// find next robot in list
	bot_t btmp = bot;
	while (btmp && btmp->type != type && btmp->id != _robotConnected[type]) {
		btmp = btmp->next;
	}

	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);
	// connect to robot class
	_robot[type][_robotConnected[type]] = &robot;
	// add simulation variables to robot class
	_robot[type][_robotConnected[type]]->addToSim(_world, _space, &_clock);
	// set unique id of this robot
	_robot[type][_robotConnected[type]]->setID(btmp->id);

	// find if robot is connected to another one
	Conn_t *ctmp = btmp->conn;
	while (ctmp) {
		if ( ctmp->robot != robot.getID() ) {
			break;
		}
		ctmp = ctmp->next;
	}

	// if robot is connected to another one
	if (ctmp) {
		for (int i = 0; i < _robotConnected[type]; i++) {
			if (_robot[type][i]->getID() == ctmp->robot) {
				_robot[type][_robotConnected[type]]->build(btmp, _robot[type][i], ctmp);
				break;
			}
		}
	}
	else {
		_robot[type][_robotConnected[type]]->build(btmp);
	}

	// another robot has been 'connected' to simulation
	_robotConnected[type]++;

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

/**********************************************************
	Utility Functions
 **********************************************************/
// get difference in two time stamps in nanoseconds
unsigned int CRobotSim::diff_nsecs(struct timespec t1, struct timespec t2) {
	return (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
}
