#include "robosim.h"
using namespace std;

// global robot simulation object
RoboSim *_simObject = NULL;

RoboSim::RoboSim(void) {
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

RoboSim::~RoboSim(void) {
#ifdef ENABLE_GRAPHICS
	// remove graphics
	if (_osgThread) {
		MUTEX_LOCK(&_viewer_mutex);
		_viewer->setDone(true);
		MUTEX_UNLOCK(&_viewer_mutex);
		THREAD_JOIN(_osgThread);
		delete _viewer;
		MUTEX_DESTROY(&_viewer_mutex);
	}
#endif

	// remove simulation
	_running = 0;
	THREAD_JOIN(_simulation);
	COND_DESTROY(&_running_cond);
	MUTEX_DESTROY(&_running_mutex);
	MUTEX_DESTROY(&_pause_mutex);
	MUTEX_DESTROY(&_robot_mutex);

	// remove ode
	dJointGroupDestroy(_group);
	dSpaceDestroy(_space);
	dWorldDestroy(_world);
	dCloseODE();

	// remove ground
	ground_t gtmp = _ground;
	while (gtmp) {
		ground_t tmp = gtmp->next;
		delete gtmp;
		gtmp = tmp;
	}

	// remove robots
	robots_t rtmp = _robots;
	while (rtmp) {
		robots_t tmp = rtmp->next;
		delete rtmp;
		rtmp = tmp;
	}

	// remove bot+connector list
	xml_robot_t btmp = _bot;
	while (btmp) {
		xml_robot_t tmp = btmp->next;
		xml_conn_t ctmp = btmp->conn;
		while (ctmp) {
			xml_conn_t tmp2 = ctmp->next;
			delete ctmp;
			ctmp = tmp2;
		}
		delete btmp;
		btmp = tmp;
	}
}

/**********************************************************
	Initialization functions
 **********************************************************/
int RoboSim::init_ode(void) {
	// create ODE simulation space
	dInitODE2(0);										// initialized ode library
	_world = dWorldCreate();							// create world for simulation
	_space = dHashSpaceCreate(0);						// create space for robots
	_group = dJointGroupCreate(0);						// create group for joints
	ground_t ng = new struct ground_s;
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

int RoboSim::init_sim(void) {
	// default collision parameters
	_mu[0] = 0.4;	_mu[1] = 0.3;
	_cor[0] = 0.3;	_cor[1] = 0.3;

	// thread variables
	MUTEX_INIT(&_pause_mutex);
	MUTEX_INIT(&_robot_mutex);
	MUTEX_INIT(&_running_mutex);
	COND_INIT(&_running_cond);
	THREAD_CREATE(&_simulation, (void* (*)(void *))&RoboSim::simulation_thread, this);

	// variables to keep track of progress of simulation
	_running = 1;			// is simulation running
	_pause = 0;				// do not start paused w/o graphics
#ifdef ENABLE_GRAPHICS
	_pause = 1;				// pause on graphics start
#endif
    _step = 0.004;			// initial time step
	_clock = 0;				// start clock

	// success
	return 0;
}

int RoboSim::init_xml(void) {
	// initialize variables
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	_bot = NULL;
	_robots = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;

	// load xml config file
	tinyxml2::XMLDocument doc;
	char path[512];
#ifdef _WIN32
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path))) {
		strcat(path, "\\robosimrc");
	}
#else
	strcpy(path, getenv("HOME"));
	strcat(path, "/.robosimrc");
#endif
	int output = doc.LoadFile(path);
	if (output) {
		printf("ERROR: could not find xml config file.\n");
		exit(-1);
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
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = MOBOT;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0; nr->angle4 = 0;
			node->QueryIntAttribute("id", &(nr->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("a1", &(nr->angle1));
				ele->QueryDoubleAttribute("a2", &(nr->angle2));
				ele->QueryDoubleAttribute("a3", &(nr->angle3));
				ele->QueryDoubleAttribute("a4", &(nr->angle4));
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			xml_robot_t rtmp = _bot;
			if ( _bot == NULL )
				_bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "linkboti") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = LINKBOTI;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			node->QueryIntAttribute("id", &(nr->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(nr->angle1));
				ele->QueryDoubleAttribute("f2", &(nr->angle2));
				ele->QueryDoubleAttribute("f3", &(nr->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					nr->psi = 0;
				else if (o == 2)
					nr->psi = M_PI/2;
				else if (o == 3)
					nr->psi = M_PI;
				else if (o == 4)
					nr->psi = 3*M_PI/2;
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			xml_robot_t rtmp = _bot;
			if ( _bot == NULL )
				_bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = LINKBOTL;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			node->QueryIntAttribute("id", &(nr->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(nr->angle1));
				ele->QueryDoubleAttribute("f2", &(nr->angle2));
				ele->QueryDoubleAttribute("f3", &(nr->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					nr->psi = 0;
				else if (o == 2)
					nr->psi = M_PI/2;
				else if (o == 3)
					nr->psi = M_PI;
				else if (o == 4)
					nr->psi = 3*M_PI/2;
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			xml_robot_t rtmp = _bot;
			if ( _bot == NULL )
				_bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "linkbott") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = LINKBOTT;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			node->QueryIntAttribute("id", &(nr->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(nr->x));
				ele->QueryDoubleAttribute("y", &(nr->y));
				ele->QueryDoubleAttribute("z", &(nr->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(nr->psi));
				ele->QueryDoubleAttribute("theta", &(nr->theta));
				ele->QueryDoubleAttribute("phi", &(nr->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(nr->angle1));
				ele->QueryDoubleAttribute("f2", &(nr->angle2));
				ele->QueryDoubleAttribute("f3", &(nr->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					nr->psi = 0;
				else if (o == 2)
					nr->psi = M_PI/2;
				else if (o == 3)
					nr->psi = M_PI;
				else if (o == 4)
					nr->psi = 3*M_PI/2;
			}
			nr->conn = NULL;
			nr->next = NULL;

			// put new bot at end of list
			xml_robot_t rtmp = _bot;
			if ( _bot == NULL )
				_bot = nr;
			else {
				while (rtmp->next)
					rtmp = rtmp->next;
				rtmp->next = nr;
			}
		}
		else if ( !strcmp(node->Value(), "g_box") ) {
			ground_t ng = new struct ground_s;
			double lx, ly, lz, px, py, pz, psi, theta, phi;
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("x", &lx);
				ele->QueryDoubleAttribute("y", &ly);
				ele->QueryDoubleAttribute("z", &lz);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
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
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "g_cylinder") ) {
			ground_t ng = new struct ground_s;
			double r, l, px, py, pz, psi, theta, phi;
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
				ele->QueryDoubleAttribute("length", &l);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
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
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "g_sphere") ) {
			ground_t ng = new struct ground_s;
			double r, px, py, pz;
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			ng->next = NULL;

			// position object
			ng->object = dCreateSphere(_space, r);
			dGeomSetPosition(ng->object, px, py, pz);

			// add object to linked list
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else {
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "bridge") ) {
				ctype = BRIDGE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = CASTER;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "cube") ) {
				ctype = CUBE;
				cnum = 5;
			}
			else if ( !strcmp(node->Value(), "faceplate") ) {
				ctype = FACEPLATE;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "gripper") ) {
				ctype = GRIPPER;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "l") ) {
				ctype = L;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "omnidrive") ) {
				ctype = OMNIDRIVE;
				cnum = 4;
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
			atmp = new int[cnum];

			// store connector to temp variables
			int i = 0;
			if (cnum == 1) {
				i = 1;
				ntmp[0] = -1;
				atmp[0] = -1;
				node->QueryIntAttribute("robot", &rtmp[0]);
				node->QueryIntAttribute("face", &ftmp[0]);

			}
			else {
				side = node->FirstChildElement();
				while (side) {
					side->QueryIntAttribute("id", &ntmp[i]);
					side->QueryIntAttribute("robot", &rtmp[i]);
					if (side->QueryIntAttribute("conn", &atmp[i]) == tinyxml2::XML_NO_ATTRIBUTE) {
						atmp[i] = -1;
						side->QueryIntAttribute("face", &ftmp[i]);
					}
					else {
						ftmp[i] = ntmp[i];
						side->QueryIntAttribute("conn", &atmp[i]);
					}
					i++;
					side = side->NextSiblingElement();
				}
			}

			// store connectors to each robot
			xml_robot_t tmp;
			xml_conn_t ctmp;
			for (int j = 0; j < i; j++) {
				xml_conn_t nc = new struct xml_conn_s;
				nc->robot = rtmp[0];
				nc->face1 = ftmp[0];
				nc->conn = atmp[j];
				nc->face2 = ftmp[j];
				nc->side = ntmp[j];
				nc->type = ctype;
				nc->next = NULL;
				tmp = _bot;
				while (tmp && tmp->id != rtmp[j])
					tmp = tmp->next;
				if (tmp == NULL) { printf("ERROR: robot %d could not be found.\n", rtmp[j]); exit(-1); }
				ctmp = tmp->conn;
				if ( tmp->conn == NULL )
					tmp->conn = nc;
				else {
					while (ctmp->next)
						ctmp = ctmp->next;
					ctmp->next = nc;
				}
			}

			// delete temporary arrays
			delete [] rtmp;
			delete [] ftmp;
			delete [] ntmp;
			delete [] atmp;
		}

		// debug printing
		/*xml_robot_t rtmp = bot;
		while (rtmp) {
			printf("type = %d, id = %d\n", rtmp->type, rtmp->id);
			printf("x = %lf, y = %lf, z = %lf\n", rtmp->x, rtmp->y, rtmp->z);
			printf("psi = %lf, theta = %lf, phi = %lf\n", rtmp->psi, rtmp->theta, rtmp->phi);
			printf("angle1 = %lf, angle2 = %lf, angle3 = %lf, angle4 = %lf\n", \
				rtmp->angle1, rtmp->angle2, rtmp->angle3, rtmp->angle4);
			xml_conn_t ctmp = rtmp->conn;
			while (ctmp) {
				printf("on face %d connect with robot %d on his face %d with type %d from side %d with conn %d\n", \
					ctmp->face2, ctmp->robot, ctmp->face1, ctmp->type, ctmp->side, ctmp->conn);
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

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int RoboSim::init_viz(void) {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);
	//osg::setNotifyLevel(osg::DEBUG_FP);

    // construct the viewer
	_viewer = new osgViewer::Viewer();
	MUTEX_INIT(&_viewer_mutex);

	// graphics hasn't started yet
	COND_INIT(&_graphics_cond);
	MUTEX_INIT(&_graphics_mutex);
	_graphics = 0;

	// create graphics thread
	THREAD_CREATE(&_osgThread, (void* (*)(void *))&RoboSim::graphics_thread, (void *)this);

	// wait for graphics to be ready
	MUTEX_LOCK(&_graphics_mutex);
	while (!_graphics) {
		COND_WAIT(&_graphics_cond, &_graphics_mutex);
	}
	MUTEX_UNLOCK(&_graphics_mutex);

	// success
	return 0;
}
#endif

/**********************************************************
	Public member functions
 **********************************************************/
int RoboSim::addRobot(CRobot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot struct
	robots_t nr = new struct robots_s;
	nr->robot = robot;
	nr->next = NULL;

	// add to ll
	robots_t rtmp = _robots;
	int connected = 0;
	if ( _robots == NULL )
		_robots = nr;
	else {
		while (rtmp->next) {
			if (rtmp->robot->getType() == robot->getType()) connected++;
			rtmp = rtmp->next;
		}
		connected++;
		rtmp->next = nr;
	}

	// find specs about new robot
	xml_robot_t btmp = _bot;
	int num = 0;
	while (btmp) {
		if (btmp->type != robot->getType()) { btmp = btmp->next; continue; }
		else { if (num++ != connected) {btmp = btmp->next; continue;}}
		break;
	}
	if (btmp == NULL) { printf("could not find robot\n"); exit(-1); }
	
	// give simulation data to robot
	robot->addToSim(_world, _space, &_clock);
	robot->setID(btmp->id);

	// find if robot is connected to another one
	xml_conn_t ctmp = btmp->conn;
	while (ctmp) {
		if ( ctmp->robot != btmp->id ) {
			break;
		}
		ctmp = ctmp->next;
	}

	// if robot is connected to another one
	if (ctmp) {
		rtmp = _robots;
		while (rtmp) {
			if (rtmp->robot->getRobotID() == ctmp->robot) {
				robot->build(btmp, rtmp->robot, ctmp);
				break;
			}
			rtmp = rtmp->next;
		}
	}
	else {
		robot->build(btmp);
	}

#ifdef ENABLE_GRAPHICS
	// draw robot
	nr->node = robot->draw(_osgRoot);
#endif // ENABLE_GRAPHICS
	
	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::deleteRobot(CRobot *robot) {
	// lock robot data to delete
	MUTEX_LOCK(&_robot_mutex);

	// remove robots
	robots_t rtmp = _robots, tmp = NULL;
	if (rtmp->robot == robot) {
		_robots = rtmp->next;
		tmp = rtmp;
	}
	else {
		while (rtmp->next) {
			if (rtmp->next->robot == robot) {
				//robots_t tmp = rtmp->next;
				tmp = rtmp->next;
				rtmp->next = rtmp->next->next;
				//delete tmp;
				break;
			}
			rtmp = rtmp->next;
		}
	}

	// remove node callback
#ifdef ENABLE_GRAPHICS
	_osgRoot->removeChild(_osgRoot->getChild(tmp->node));
#endif

	// delete robot
	delete tmp;

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::setExitState(void) {
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);

	// success
	return 0;
}

/**********************************************************
	Private functions
 **********************************************************/
void* RoboSim::simulation_thread(void *arg) {
	// cast to type sim 
	RoboSim *sim = (RoboSim *)arg;

	// initialize local variables
	unsigned int sum = 0, dt[4] = {0};
	int i;
#ifdef _WIN32
	DWORD start_time, start;
#else
	struct timespec start_time, end_time;
	unsigned int start, end;
#endif

	MUTEX_LOCK(&(sim->_running_mutex));
	while (sim->_running) {
		MUTEX_UNLOCK(&(sim->_running_mutex));

		// get starting times
#ifdef _WIN32
		start = GetTickCount();
#else
		clock_gettime(CLOCK_REALTIME, &start_time);
		start = start_time.tv_sec*1000 + start_time.tv_nsec/1000000;
#endif

		// lock pause variable
		MUTEX_LOCK(&(sim->_pause_mutex));

		while (!(sim->_pause) && sim->_running) {
			// unlock pause variable
			MUTEX_UNLOCK(&(sim->_pause_mutex));

			// get start time of execution
#ifdef _WIN32
			start_time = GetTickCount();
#else
			clock_gettime(CLOCK_REALTIME, &start_time);
#endif

			// perform pre-collision updates
			MUTEX_LOCK(&(sim->_robot_mutex));
			robots_t rtmp = sim->_robots;
			while (rtmp) {
				THREAD_CREATE(&(rtmp->thread), (void* (*)(void *))&CRobot::simPreCollisionThreadEntry, (void *)rtmp->robot);
				THREAD_JOIN(rtmp->thread);
				rtmp = rtmp->next;
			}

			// perform ode update
			dSpaceCollide(sim->_space, sim, &sim->collision);
			dWorldStep(sim->_world, sim->_step);
			sim->_clock += sim->_step;
			dJointGroupEmpty(sim->_group);

			//sim->print_intermediate_data();

			// perform post-collision updates
			rtmp = sim->_robots;
			while (rtmp) {
				THREAD_CREATE(&(rtmp->thread), (void* (*)(void *))&CRobot::simPostCollisionThreadEntry, (void *)rtmp->robot);
				THREAD_JOIN(rtmp->thread);
				rtmp = rtmp->next;
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
			dt[0] = (end_time.tv_sec - start_time.tv_sec)*1000 + (end_time.tv_nsec - start_time.tv_nsec)/1000000;
			end = end_time.tv_sec*1000 + end_time.tv_nsec/1000000;
			for (i = 0; i < 4; i++) { sum += dt[i]; }
			for (i = 2; i >=0; i--) { dt[i+1] = dt[i]; }
			sum /= 4;
			if (end - start > (unsigned int)(sim->_clock*1000))
				sim->_step = (end - start - (unsigned int)(sim->_clock*1000) + sum)/1000.0;
			else {
				sim->_step = sum/1000.0;
				usleep(sim->_clock*1000000 - ((end - start)*1000));
			}
			sim->_step = (sim->_step*1000 < 4) ? 0.004 : sim->_step;
#endif
			// lock pause variable
			MUTEX_LOCK(&(sim->_pause_mutex));
		}
		// unlock pause variable
		MUTEX_UNLOCK(&(sim->_pause_mutex));
		MUTEX_LOCK(&(sim->_running_mutex));
	}
	MUTEX_UNLOCK(&(sim->_running_mutex));

	return NULL;
}

void RoboSim::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	RoboSim *ptr = (RoboSim *)data;

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
		dContact contact[8] = {0};
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

void RoboSim::print_intermediate_data(void) {
    cout.width(10);
    cout.setf(ios::fixed, ios::floatfield);
	cout << _clock << "\t\t";
	robots_t rtmp = _robots;
	while (rtmp) {
		if (rtmp->robot->getType() == MOBOT) {
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT1)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT2)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT3)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT4)) << "\t\t";
		}
		else if (rtmp->robot->getType() == LINKBOTI) {
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT1)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT2)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT3)) << " ";
		}
		else if (rtmp->robot->getType() == LINKBOTL) {
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT1)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT2)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT3)) << " ";
		}
		else if (rtmp->robot->getType() == LINKBOTT) {
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT1)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT2)) << " ";
			//cout << RAD2DEG(rtmp->robot->getAngle(ROBOT_JOINT3)) << " ";
		}
		rtmp = rtmp->next;
	}
	cout << endl;
}

#ifdef ENABLE_GRAPHICS
void* RoboSim::graphics_thread(void *arg) {
	// cast viewer
	RoboSim *sim = (RoboSim *)arg;

	// initialize variables
	unsigned int width, height;

	// window interface
	osg::GraphicsContext::WindowingSystemInterface *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi) {
		osg::notify(osg::NOTICE) << "osg: cannot create windows." << endl;
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
		osg::notify(osg::NOTICE) << "osg: cannot create graphics." << endl;
		return NULL;
	}

	// camera properties
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);
	sim->_viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	sim->_viewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
	sim->_viewer->getCamera()->setNearFarRatio(0.00001);

	// viewer camera properties
	sim->_viewer->addSlave(camera.get());
	osgGA::OrbitManipulator *cameraManipulator = new osgGA::OrbitManipulator();
	cameraManipulator->setDistance(0.1);
	cameraManipulator->setAllowThrow(false);
	cameraManipulator->setWheelZoomFactor(0);
	cameraManipulator->setVerticalAxisFixed(true);
	cameraManipulator->setElevation(0.5);
	sim->_viewer->setCameraManipulator(cameraManipulator);
	sim->_viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(1, 1, 0.75), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

    // Creating the root node
	sim->_osgRoot = new osg::Group();

	// load terrain node
	osg::Depth *t_depth = new osg::Depth;
	t_depth->setFunction(osg::Depth::LEQUAL);
	t_depth->setRange(1.0, 1.0);

	osg::StateSet *t_stateset = new osg::StateSet();
	t_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	t_stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	t_stateset->setAttributeAndModes(t_depth, osg::StateAttribute::ON);
	t_stateset->setRenderBinDetails(-1, "RenderBin");

	osg::ref_ptr<osg::Node> t_geode = osgDB::readNodeFile(TEXTURE_PATH(ground/terrain.3ds));
	t_geode->setCullingActive(false);
	t_geode->setStateSet(t_stateset);
	osg::ref_ptr<osg::PositionAttitudeTransform> t_transform = new osg::PositionAttitudeTransform();
	t_transform->setScale(osg::Vec3d(2, 2, 0.001));
	t_transform->setCullingActive(false);
	t_transform->addChild(t_geode);
	sim->_osgRoot->addChild(t_transform);

	osgUtil::LineSegmentIntersector *r_segment = new osgUtil::LineSegmentIntersector(osg::Vec3d(0, 0, 999), osg::Vec3d(0, 0, -999));
	osgUtil::IntersectionVisitor r_visitor;
	r_visitor.setIntersector(r_segment);
	t_transform->accept(r_visitor);
	osgUtil::LineSegmentIntersector::Intersection r_hits = r_segment->getFirstIntersection();
	osg::Vec3d r_pos = r_hits.getWorldIntersectPoint();
	t_transform->setPosition(osg::Vec3d(r_pos[0], r_pos[1], -r_pos[2]));

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
	osg::Image* imagePosX = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_right.png));
	osg::Image* imageNegX = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_left.png));
	osg::Image* imagePosY = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_top.png));
	osg::Image* imageNegY = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_top.png));
	osg::Image* imagePosZ = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_front.png));
	osg::Image* imageNegZ = osgDB::readImageFile(TEXTURE_PATH(ground/checkered/checkered_back.png));

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
	sim->_osgRoot->addChild(clearNode);

	// set up HUD
	osg::Geode *HUDGeode = new osg::Geode();
	osgText::Text *textHUD = new osgText::Text();
	osg::Projection *HUDProjectionMatrix = new osg::Projection;
	osg::MatrixTransform *HUDModelViewMatrix = new osg::MatrixTransform;
	osg::StateSet *HUDStateSet = new osg::StateSet();
	HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0,traits->width,0,traits->height));
	HUDProjectionMatrix->addChild(HUDModelViewMatrix);
	HUDModelViewMatrix->setMatrix(osg::Matrix::identity());
	HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	HUDModelViewMatrix->addChild(HUDGeode);
	HUDGeode->setStateSet(HUDStateSet);
	HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
	HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	HUDStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
	HUDStateSet->setRenderBinDetails( 11, "RenderBin");
	HUDGeode->addDrawable( textHUD );
	textHUD->setCharacterSize(20);
	textHUD->setText("PAUSED\npress space to start");
	textHUD->setAxisAlignment(osgText::Text::SCREEN);
	textHUD->setAlignment(osgText::Text::CENTER_CENTER);
	textHUD->setPosition( osg::Vec3(traits->width/2, 50, -1.5) );
	textHUD->setColor( osg::Vec4(199, 77, 15, 1) );
	sim->_osgRoot->addChild(HUDProjectionMatrix);

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(sim->_osgRoot);

	// set threading model
	sim->_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// viewer event handlers
	sim->_viewer->addEventHandler(new keyboardEventHandler(&(sim->_pause), textHUD));
	sim->_viewer->addEventHandler(new osgGA::StateSetManipulator(camera->getOrCreateStateSet()));
	sim->_viewer->addEventHandler(new osgViewer::WindowSizeHandler);

	// set viewable
	sim->_viewer->setSceneData(sim->_osgRoot);

	// signal connection functions that graphics are set up
	SIGNAL(&(sim->_graphics_cond), &(sim->_graphics_mutex), sim->_graphics = 1);

	// run viewer
	MUTEX_LOCK(&(sim->_viewer_mutex));
	while (!sim->_viewer->done()) {
		MUTEX_UNLOCK(&(sim->_viewer_mutex));
		sim->_viewer->frame();
		MUTEX_LOCK(&(sim->_viewer_mutex));
	}

	// trigger end of code when graphics window is closed
	SIGNAL(&(sim->_running_cond), &(sim->_running_mutex), sim->_running = 0);

	// return
	return arg;
}
#endif // ENABLE_GRAPHICS

/**********************************************************
	Utility functions
 **********************************************************/
#ifndef _CH_
void delay(double seconds) {
#ifdef _WIN32
	Sleep((int)(seconds*1000));
#else
	usleep((int)(seconds*1000000));
#endif
}
#endif
