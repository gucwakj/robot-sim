#include "robotsim.h"
using namespace std;

#ifndef ROBOSIM_OBJECT
#define ROBOSIM_OBJECT
CRobotSim _simObject;
#endif

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
	THREAD_CANCEL(_osgThread);

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
	ground_t *ng = (ground_t *)malloc(sizeof(struct ground_s));
	ng->object = dCreatePlane(_space, 0, 0, 1, 0);
	ng->next = NULL;
	_ground = ng;

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
	MUTEX_INIT(&_running_mutex);
	COND_INIT(&_running_cond);

	// variables to keep track of progress of simulation
	_running = 1;
    _step = 0.004;
	_clock = 0;

	// success
	return 0;
}

int CRobotSim::init_xml(void) {
	// initialize variables
	int *rtmp, *ftmp, *ntmp, ctype, cnum;
	bot = NULL;
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		_robot[i] = NULL;
		_robotNumber[i] = 0;
		_robotConnected[i] = 0;
		_robotThread[i] = NULL;
	}
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;

	// load xml config file
	tinyxml2::XMLDocument doc;
	int output = doc.LoadFile("robotsim.xml");
	if (output) {
		fprintf(stderr, "ERROR: could not find xml config file.\n");
		exit(1);
	}

	// get root node of xml file
	tinyxml2::XMLElement *node = doc.FirstChildElement("sim")->FirstChildElement();

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "params") ) {
			node->QueryDoubleAttribute("mu_g", &(_mu[0]));
			node->QueryDoubleAttribute("mu_b", &(_mu[1]));
			node->QueryDoubleAttribute("cor_g", &(_cor[0]));
			node->QueryDoubleAttribute("cor_b", &(_cor[1]));
		}
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
		else if ( !strcmp(node->Value(), "g_box") ) {
			ground_t *ng = (ground_t *)malloc(sizeof(struct ground_s));
			double lx, ly, lz, px, py, pz, psi, theta, phi;
			if (ele = node->FirstChildElement("size")) {
				ele->QueryDoubleAttribute("x", &lx);
				ele->QueryDoubleAttribute("y", &ly);
				ele->QueryDoubleAttribute("z", &lz);
			}
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if (ele = node->FirstChildElement("rotation")) {
				ele->QueryDoubleAttribute("psi", &psi);
				ele->QueryDoubleAttribute("theta", &theta);
				ele->QueryDoubleAttribute("phi", &phi);
			}
			ng->next = NULL;

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);

			// position object
			ng->object = dCreateBox(_space, lx, ly, lz);
			dGeomSetPosition(ng->object, px, py, pz);
			dGeomSetRotation(ng->object, R);

			// add object to linked list
			ground_t *gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "g_cylinder") ) {
			ground_t *ng = (ground_t *)malloc(sizeof(struct ground_s));
			double r, l, px, py, pz, psi, theta, phi;
			if (ele = node->FirstChildElement("size")) {
				ele->QueryDoubleAttribute("radius", &r);
				ele->QueryDoubleAttribute("length", &l);
			}
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if (ele = node->FirstChildElement("rotation")) {
				ele->QueryDoubleAttribute("psi", &psi);
				ele->QueryDoubleAttribute("theta", &theta);
				ele->QueryDoubleAttribute("phi", &phi);
			}
			ng->next = NULL;

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);

			// position object
			ng->object = dCreateCylinder(_space, r, l);
			dGeomSetPosition(ng->object, px, py, pz);
			dGeomSetRotation(ng->object, R);

			// add object to linked list
			ground_t *gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "g_sphere") ) {
			ground_t *ng = (ground_t *)malloc(sizeof(struct ground_s));
			double r, px, py, pz;
			if (ele = node->FirstChildElement("size")) {
				ele->QueryDoubleAttribute("radius", &r);
			}
			if (ele = node->FirstChildElement("position")) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			ng->next = NULL;

			// position object
			ng->object = dCreateSphere(_space, r);
			dGeomSetPosition(ng->object, px, py, pz);

			// add object to linked list
			ground_t *gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
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
void *CRobotSim::graphicsWait(void *arg) {
	CRobotSim *sim = (CRobotSim *)arg;
// wait for graphics to be set up
	printf("graphics wait thread\n");
	MUTEX_LOCK(&sim->_graphics_mutex);
	printf("graphics wait thread\n");
	while (!sim->_graphics) {
		COND_WAIT(&sim->_graphics_cond, &sim->_graphics_mutex);
	}
	printf("set up graphics\n");
	MUTEX_UNLOCK(&sim->_graphics_mutex);
	
	// return
	return arg;
}

int CRobotSim::init_viz(void) {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

    // construct the viewer
	viewer = new osgViewer::Viewer();

	// init graphics mutex
	//_graphics = false;
	//MUTEX_INIT(&_graphics_mutex);
	//COND_INIT(&_graphics_cond);
	//THREAD_T thread;

	// create graphics thread
//THREAD_CREATE(&thread, (void* (*)(void *))&CRobotSim::graphicsWait, (void *)this);
	THREAD_CREATE(&_osgThread, (void* (*)(void *))&CRobotSim::graphicsThread, (void *)this);
//THREAD_JOIN(thread);


	// success
	return 0;
}

void* CRobotSim::graphicsThread(void *arg) {
	// initialize variables
	//double fovy, ar, zNear, zFar, nar/*, dar*/;
	unsigned int width, height;

	// cast viewer
	CRobotSim *sim = (CRobotSim *)arg;

	// window interface
	osg::GraphicsContext::WindowingSystemInterface *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi) {
		osg::notify(osg::NOTICE) << "View::setUpViewAcrossAllScreens() : Error, no WindowSystemInterface available, cannot create windows." << endl;
		return NULL;
	}
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	// window traits
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = width/4;
	traits->y = height/4;
	traits->width = width/2;
	traits->height = 3*width/8;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->vsync = false;
	traits->sharedContext = 0;

	// graphics context
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid()) {
		gc->setClearColor(osg::Vec4f(0.f,0.f,0.f,0.f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else {
		osg::notify(osg::NOTICE) << "  GraphicsWindow has not been created successfully." << endl;
		return NULL;
	}

	// graphics window
	/*osgViewer::GraphicsWindow *gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
	if (gw)
		gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(traits->x, traits->y, traits->width, traits->height);
	else
		OSG_NOTICE<<"  GraphicsWindow has not been created successfully."<<std::endl;*/

	// camera properties
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	/*camera->getProjectionMatrixAsPerspective(fovy, ar, zNear, zFar);
	nar = double(traits->width) / double(traits->height);
	if (nar/ar != 1.0) camera->getProjectionMatrix() *= osg::Matrix::scale(ar/nar, 1.0, 1.0);*/
	camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);
	//viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(1, 0, 0.8), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

	// viewer camera properties
	sim->viewer->addSlave(camera.get());
	//sim->viewer->setCameraManipulator(new osgGA::TerrainManipulator);
	//sim->viewer->setCameraManipulator(new osgGA::SphericalManipulator);
	//sim->viewer->setCameraManipulator(new osgGA::FirstPersonManipulator);
	sim->viewer->setCameraManipulator(new osgGA::TrackballManipulator);
	sim->viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0, 2, 1), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

	// viewer event handlers
	sim->viewer->addEventHandler(new osgGA::StateSetManipulator(camera->getOrCreateStateSet()));
	//sim->viewer->addEventHandler(new osgViewer::ThreadingHandler);
	sim->viewer->addEventHandler(new osgViewer::WindowSizeHandler);
    //sim->viewer->addEventHandler(new osgViewer::StatsHandler);

    // Creating the root node
	sim->_osgRoot = new osg::Group();
	sim->_osgRoot->setUpdateCallback(new rootNodeCallback(sim, sim->_robot, sim->_osgRoot));

	// load terrain node
	osg::ref_ptr<osg::MatrixTransform> terrainScaleMAT (new osg::MatrixTransform);
	osg::Matrix terrainScaleMatrix;
	terrainScaleMatrix.makeScale(0.1f,0.1f,0.006f);
	osg::ref_ptr<osg::Node> terrainnode = osgDB::readNodeFile(TEXTURE_PATH(ground/terrain.3ds));
	terrainScaleMAT->addChild(terrainnode.get());
	terrainScaleMAT->setMatrix(terrainScaleMatrix);
	//sim->_osgRoot->addChild(terrainScaleMAT.get());

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
    osg::TextureCubeMap* skymap = new osg::TextureCubeMap;
	osg::Image* imagePosX = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_front.png));
	osg::Image* imageNegX = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_back.png));
	osg::Image* imagePosY = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_top.png));
	osg::Image* imageNegY = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_top.png));
	osg::Image* imagePosZ = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_left.png));
	osg::Image* imageNegZ = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_right.png));

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
	clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TexMatCallback(*tm));
	clearNode->addChild(transform);
	//sim->_osgRoot->addChild(clearNode);

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(sim->_osgRoot);

	// set threading model
	//sim->viewer->setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);
	sim->viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// set viewable
	sim->viewer->setSceneData(sim->_osgRoot);

	// trigger graphics is set up
	/*printf("thread greaphics1: %d\n", sim->_graphics);
	MUTEX_LOCK(&(sim->_graphics_mutex));
	sim->_graphics = true;
	printf("thread greaphics2: %d\n", sim->_graphics);
	COND_SIGNAL(&(sim->_graphics_cond));
	MUTEX_UNLOCK(&(sim->_graphics_mutex));
	printf("\tsignaling\n");*/
	//SIGNAL(&(sim->_graphics_cond), &(sim->_graphics_mutex), sim->_graphics = true);

	// run viewer
	sim->viewer->run();

	// trigger end of code when graphics window is closed
	SIGNAL(&(sim->_running_cond), &(sim->_running_mutex), sim->_running = 0);

	// return
	return arg;
}
#endif // ENABLE_GRAPHICS

/**********************************************************
	Public Member Functions
 **********************************************************/
int CRobotSim::getNumberOfRobots(int type) {
	return _robotConnected[type];
}

int CRobotSim::setExitState(void) {
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);

	// success
	return 0;
}

/**********************************************************
	Private Simulation Functions
 **********************************************************/
void* CRobotSim::simulationThread(void *arg) {
	// cast to type sim 
	CRobotSim *sim = (CRobotSim *)arg;

	// initialize local variables
	unsigned int sum = 0, dt[4] = {0};
	int i, j;
#ifdef _WIN32
		DWORD start = GetTickCount();
#endif

	while (1) {
		// get start time of execution
#ifdef _WIN32
		DWORD start_time = GetTickCount();
#else
		struct timespec start_time, end_time;
		clock_gettime(CLOCK_REALTIME, &start_time);
#endif

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles
		MUTEX_LOCK(&(sim->_robot_mutex));
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				THREAD_CREATE(&(sim->_robotThread[i][j]), (void* (*)(void *))&CRobot::simPreCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				THREAD_JOIN(sim->_robotThread[i][j]);
			}
		}

		dSpaceCollide(sim->_space, sim, &sim->collision);	// collide all geometries together
		dWorldStep(sim->_world, sim->_step);				// step world time by one
		sim->_clock += sim->_step;							// increment world clock
		dJointGroupEmpty(sim->_group);						// clear out all contact joints

		//sim->print_intermediate_data();

		// perform post-collision updates
		//  - unlock angle and goal
		//  - check if success
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				THREAD_CREATE(&(sim->_robotThread[i][j]), (void* (*)(void *))&CRobot::simPostCollisionThreadEntry, (void *)(sim->_robot[i][j]));
				THREAD_JOIN(sim->_robotThread[i][j]);
			}
		}
		MUTEX_UNLOCK(&(sim->_robot_mutex));

		// sleep until next step
#ifdef _WIN32
		dt[0] = GetTickCount() - start_time;
		for (i = 0; i < 4; i++) { sum += dt[i]; }
		for (i = 2; i >=0; i--) { dt[i+1] = dt[i]; }
		sum /= 4;
		if (GetTickCount() - start > (unsigned int)(sim->_clock*1000))
			sim->_step = (GetTickCount() - start - (unsigned int)(sim->_clock*1000) + sum)/1000.0;
		else {
			sim->_step = sum/1000.0;
			Sleep((unsigned int)(sim->_clock*1000) - (GetTickCount() - start));
		}
		sim->_step = (sim->_step*1000 < 4) ? 0.004 : sim->_step;
#else
		clock_gettime(CLOCK_REALTIME, &end_time);
		dt = sim->diff_nsecs(start_time, end_time);
		if ( dt < sim->_step*1000000000 ) { usleep(sim->_step*1000000 - dt/1000); }
#endif
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

    cout.width(10);
    cout.setf(ios::fixed, ios::floatfield);
	//if (!((int)(_clock*1000) % 100)) { cout << _clock << "\t\t"; }
	cout << _clock << "\n";
	/*for (int i = 0; i < _robotConnected[MOBOT]; i++) {
		cout << RAD2DEG(_robot[MOBOT][i]->getAngle(ROBOT_JOINT1)) << " ";
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(ROBOT_JOINT2)) << " ";
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(ROBOT_JOINT3)) << " ";
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(ROBOT_JOINT4)) << "\t\t";
		//cout << _robot[MOBOT][i]->getPosition(2,0) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2,1) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2,2) << "\t\t";
	}
	cout << endl;*/
	/*cout << "LinkbotL:" << "\t";
	for (int i = 0; i < _robotConnected[LINKBOTL]; i++) {
		//cout << RAD2DEG(_robot[MOBOT][i]->getAngle(MOBOT_JOINT4)) << " ";
		//cout << _robot[LINKBOTL][i]->getAngle(ROBOT_JOINT1) << " ";
		//cout << _robot[LINKBOTL][i]->getAngle(ROBOT_JOINT2) << " ";
		//cout << _robot[LINKBOTL][i]->getAngle(ROBOT_JOINT3) << "\t";
		//cout << _robot[MOBOT][i]->getPosition(2, 0) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 1) << " ";
		//cout << _robot[MOBOT][i]->getPosition(2, 2) << "\t";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT1) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT2) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT3) << " ";
		//cout << _robot[IMOBOT][i]->getSuccess(IMOBOT_JOINT4) << "\t";
	}
	cout << endl;*/
}


/**********************************************************
	Add Robot Functions
 **********************************************************/
int CRobotSim::addRobot(CRobot *robot) {
	// get type of robot being added
	int type = robot->getType();

	// find next robot in list
	bot_t btmp = bot;
	while (btmp) {
		if (btmp->id != _robotConnected[type]) { btmp = btmp->next; continue; }
		if (btmp->type != type) { btmp = btmp->next; continue; }
		break;
	}
	if (btmp == NULL) { fprintf(stderr, "could not find robot\n"); exit(1); }

	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);
	// connect to robot class
	_robot[type][_robotConnected[type]] = robot;
	// add simulation variables to robot class
	_robot[type][_robotConnected[type]]->addToSim(_world, _space, &_clock);
	// set unique id of this robot
	_robot[type][_robotConnected[type]]->setID(btmp->id);

	// find if robot is connected to another one
	Conn_t *ctmp = btmp->conn;
	while (ctmp) {
		if ( ctmp->robot != robot->getID() ) {
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

	// draw robot
	//_robot[type][_robotConnected[type]]->draw(_osgRoot);

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
#ifndef _WIN32
// get difference in two time stamps in nanoseconds
unsigned int CRobotSim::diff_nsecs(struct timespec t1, struct timespec t2) {
	return (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
}
#endif

#ifndef _CH_
void delay(double seconds) {
#ifdef _WIN32
	Sleep((int)(seconds*1000));
#else
	usleep((int)(seconds*1000000));
#endif
}
#endif