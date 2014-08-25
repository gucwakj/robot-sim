#include "robosim.hpp"
using namespace std;

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(char *name, int pause) {
	// initialize ode
	init_ode();

	// initialize xml config file
	init_xml(name);

	// initialize simulation
	init_sim(pause);

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
		_viewer = 0;
		MUTEX_UNLOCK(&_viewer_mutex);
		THREAD_JOIN(_osgThread);
		COND_DESTROY(&_graphics_cond);
		MUTEX_DESTROY(&_graphics_mutex);
		MUTEX_DESTROY(&_viewer_mutex);
	}
#endif

	// remove simulation
	MUTEX_LOCK(&_running_mutex);
	_running = 0;
	MUTEX_UNLOCK(&_running_mutex);
	THREAD_JOIN(_simulation);
	COND_DESTROY(&_running_cond);
	MUTEX_DESTROY(&_clock_mutex);
	MUTEX_DESTROY(&_pause_mutex);
	MUTEX_DESTROY(&_robot_mutex);
	MUTEX_DESTROY(&_running_mutex);
	MUTEX_DESTROY(&_step_mutex);

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
	for (int i = 0; i < _robots.size(); i++) {
		delete _robots[i];
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
	ng->body = NULL;								// immovable
	ng->geom = dCreatePlane(_space, 0, 0, 1, 0);	// ground plane
	ng->next = NULL;
	_ground = ng;

	// simulation parameters
	dWorldSetAutoDisableFlag(_world, 1);				// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(_world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(_world, 0.01);	// linear velocity threshold
	dWorldSetAutoDisableSteps(_world, 4);				// number of steps below thresholds before stationary
	dWorldSetContactSurfaceLayer(_world, 0.01);			// depth each body can sink into another body before resting
	dWorldSetGravity(_world, 0, 0, -9.81);				// gravity

	// success
	return 0;
}

int RoboSim::init_sim(int pause) {
	// default collision parameters
	_mu[0] = 0.9;	_mu[1] = 0.6;
	_cor[0] = 0.3;	_cor[1] = 0.3;

	// thread variables
	MUTEX_INIT(&_clock_mutex);
	MUTEX_INIT(&_pause_mutex);
	MUTEX_INIT(&_robot_mutex);
	MUTEX_INIT(&_running_mutex);
	MUTEX_INIT(&_step_mutex);
	COND_INIT(&_running_cond);
	THREAD_CREATE(&_simulation, (void* (*)(void *))&RoboSim::simulation_thread, this);

	// variables to keep track of progress of simulation
	_running = 1;			// is simulation running
#ifdef ENABLE_GRAPHICS
	if (pause == 0) { _pause = 0; }
	else if (pause == 1) { _pause = 1; }
	else if (_pause == 3 && pause == 2) { _pause = 0; }
#else
	_pause = 0;				// do not start paused w/o graphics
#endif
    _step = 0.004;			// initial time step
	_clock = 0;				// start clock
	_collision = true;		// perform inter-robot collisions

	// success
	return 0;
}

int RoboSim::init_xml(char *name) {
	// initialize variables
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	int tracking = 0;
	int custom = 0;
	double size = 0;
	_bot = NULL;
	_preconfig = 0;
	_pause = 3;
	_rt = 1;
	tinyxml2::XMLElement *node = NULL;
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
	if (name) {
		FILE *fp = fopen(name, "r");
		if (fp) {
			strcpy(path, name);
			fclose(fp);
		}
		else {
			strcat(path, "/.robosimrc");
		}
	}
	else {
		strcat(path, "/.robosimrc");
	}
#endif
	int output = doc.LoadFile(path);
	if (output) {
		fprintf(stderr, "Error: Could not find RoboSim config file.\nPlease run RoboSim GUI.\n");
		exit(-1);
	}

	// check for custom mu params
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("mu")) ) {
		node->QueryDoubleAttribute("ground", &(_mu[0]));
		node->QueryDoubleAttribute("body", &(_mu[1]));
	}

	// check for custom cor params
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("cor")) ) {
		node->QueryDoubleAttribute("ground", &(_cor[0]));
		node->QueryDoubleAttribute("body", &(_cor[1]));
	}

	// check if should start paused
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("pause")) ) {
		node->QueryIntAttribute("val", &_pause);
	}

	// check if should run in real time
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("realtime")) ) {
		node->QueryIntAttribute("val", &_rt);
	}

	// check for existence of ground node
	if ( (node = doc.FirstChildElement("ground")) ) {
		node = node->FirstChildElement();
	}

	// loop over all ground nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// store default variables
			ground_t ng = new struct ground_s;
			ng->r = 0;
			ng->g = 0;
			ng->b = 0;
			ng->alpha = 1;
			ng->next = NULL;

			// get user defined values from xml
			double lx, ly, lz, px, py, pz, psi, theta, phi, mass;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &ng->r);
				ele->QueryDoubleAttribute("g", &ng->g);
				ele->QueryDoubleAttribute("b", &ng->b);
				ele->QueryDoubleAttribute("alpha", &ng->alpha);
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
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("x", &lx);
				ele->QueryDoubleAttribute("y", &ly);
				ele->QueryDoubleAttribute("z", &lz);
			}

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);

			// create body
			dMass m;
			ng->body = dBodyCreate(_world);
			if ( mass == 0 ) {
				dBodyDisable(ng->body);
				mass = 1;
			}
			dMassSetBoxTotal(&m, mass, lx, ly, lz);
			dBodySetPosition(ng->body, px, py, pz);
			dBodySetRotation(ng->body, R);

			// position geom
			ng->geom = dCreateBox(_space, lx, ly, lz);
			dGeomSetBody(ng->geom, ng->body);
			dGeomSetOffsetPosition(ng->geom, -m.c[0], -m.c[1], -m.c[2]);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(ng->body, &m);

			// add object to linked list
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// store default variables
			ground_t ng = new struct ground_s;
			ng->r = 0;
			ng->g = 0;
			ng->b = 0;
			ng->alpha = 1;
			ng->next = NULL;

			// get user defined values from xml
			double r, l, px, py, pz, psi, theta, phi, mass;
			int axis;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if (node->QueryIntAttribute("axis", &axis)) {
				axis = 1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &ng->r);
				ele->QueryDoubleAttribute("g", &ng->g);
				ele->QueryDoubleAttribute("b", &ng->b);
				ele->QueryDoubleAttribute("alpha", &ng->alpha);
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
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
				ele->QueryDoubleAttribute("length", &l);
			}

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);

			// create body
			dMass m;
			ng->body = dBodyCreate(_world);
			if ( mass == 0 ) {
				dBodyDisable(ng->body);
				mass = 1;
			}
			dMassSetCylinderTotal(&m, mass, axis, r, l);
			dBodySetPosition(ng->body, px, py, pz);
			dBodySetRotation(ng->body, R);

			// position geom
			ng->geom = dCreateCylinder(_space, r, l);
			dGeomSetBody(ng->geom, ng->body);
			dGeomSetOffsetPosition(ng->geom, -m.c[0], -m.c[1], -m.c[2]);
			dMatrix3 R1;
			if (axis == 1) {		dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2); }
			else if (axis == 2) {	dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2); }
			else if (axis == 3) {	dRFromAxisAndAngle(R1, 0, 0, 1, 0); }
			dGeomSetOffsetRotation(ng->geom, R1);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(ng->body, &m);

			// add object to linked list
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// store default variables
			ground_t ng = new struct ground_s;
			ng->r = 0;
			ng->g = 0;
			ng->b = 0;
			ng->alpha = 1;
			ng->next = NULL;

			// get user defined values from xml
			double r, px, py, pz, mass;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &ng->r);
				ele->QueryDoubleAttribute("g", &ng->g);
				ele->QueryDoubleAttribute("b", &ng->b);
				ele->QueryDoubleAttribute("alpha", &ng->alpha);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
			}

			// create body
			dMass m;
			ng->body = dBodyCreate(_world);
			if ( mass == 0 ) {
				dBodyDisable(ng->body);
				mass = 1;
			}
			dMassSetSphereTotal(&m, mass, r);
			dBodySetPosition(ng->body, px, py, pz);

			// position geom
			ng->geom = dCreateSphere(_space, r);
			dGeomSetBody(ng->geom, ng->body);
			dGeomSetOffsetPosition(ng->geom, -m.c[0], -m.c[1], -m.c[2]);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(ng->body, &m);

			// add object to linked list
			ground_t gtmp = _ground;
			while (gtmp->next)
				gtmp = gtmp->next;
			gtmp->next = ng;
		}

		// go to next node
		node = node->NextSiblingElement();
	}

#ifdef ENABLE_GRAPHICS
	// read in grid line configuration
	if ( (node = doc.FirstChildElement("graphics")->FirstChildElement("grid")) ) {
		node->QueryIntAttribute("units", &_us);
		node->QueryDoubleAttribute("tics", &_grid[0]);
		node->QueryDoubleAttribute("major", &_grid[1]);
		node->QueryDoubleAttribute("minx", &_grid[2]);
		node->QueryDoubleAttribute("maxx", &_grid[3]);
		node->QueryDoubleAttribute("miny", &_grid[4]);
		node->QueryDoubleAttribute("maxy", &_grid[5]);
		node->QueryDoubleAttribute("enabled", &_grid[6]);
	}
	else {
		_us = 1;			// customary units
		_grid[0] = 1;		// 1 inch per tic
		_grid[1] = 12;		// 12 inches per hash
		_grid[2] = -24;		// min x
		_grid[3] = 24;		// max x
		_grid[4] = -24;		// min y
		_grid[5] = 24;		// max y
		_grid[6] = 1;		// enabled or not
	}
	for (int i = 0; i < 6; i++) {
		if (_us)
			_grid[i] /= 39.37;
		else
			_grid[i] /= 100;
	}

	// check for existence of ground node
	if ( (node = doc.FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all graphics nodes ignoring grid
	while (node) {
		if ( !strcmp(node->Value(), "line") ) {
			// store default variables
			drawing_t nd = new struct drawing_s;
			nd->type = LINE;
			nd->next = NULL;

			// get user defined values from xml
			if (node->QueryIntAttribute("width", &nd->i)) {
				nd->i = 1;
			}
			if ( (ele = node->FirstChildElement("start")) ) {
				ele->QueryDoubleAttribute("x", &nd->p1[0]);
				ele->QueryDoubleAttribute("y", &nd->p1[1]);
				ele->QueryDoubleAttribute("z", &nd->p1[2]);
			}
			if ( (ele = node->FirstChildElement("end")) ) {
				ele->QueryDoubleAttribute("x", &nd->p2[0]);
				ele->QueryDoubleAttribute("y", &nd->p2[1]);
				ele->QueryDoubleAttribute("z", &nd->p2[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &nd->c[0]);
				ele->QueryDoubleAttribute("g", &nd->c[1]);
				ele->QueryDoubleAttribute("b", &nd->c[2]);
				ele->QueryDoubleAttribute("alpha", &nd->c[3]);
			}

			// add object to linked list
			drawing_t dtmp = _drawings;
			if (dtmp == NULL)
				_drawings = nd;
			else {
				while (dtmp->next)
					dtmp = dtmp->next;
				dtmp->next = nd;
			}
		}
		else if ( !strcmp(node->Value(), "point") ) {
			// store default variables
			drawing_t nd = new struct drawing_s;
			nd->type = DOT;
			nd->next = NULL;

			// get user defined values from xml
			if (node->QueryIntAttribute("size", &nd->i)) {
				nd->i = 1;
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &nd->p1[0]);
				ele->QueryDoubleAttribute("y", &nd->p1[1]);
				ele->QueryDoubleAttribute("z", &nd->p1[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &nd->c[0]);
				ele->QueryDoubleAttribute("g", &nd->c[1]);
				ele->QueryDoubleAttribute("b", &nd->c[2]);
				ele->QueryDoubleAttribute("alpha", &nd->c[3]);
			}

			// add object to linked list
			drawing_t dtmp = _drawings;
			if (dtmp == NULL)
				_drawings = nd;
			else {
				while (dtmp->next)
					dtmp = dtmp->next;
				dtmp->next = nd;
			}
		}
		else if ( !strcmp(node->Value(), "grid") ) {}
		else if ( !strcmp(node->Value(), "tracking") ) {
			node->QueryIntAttribute("val", &tracking);
		}
		else {
			// store default variables
			drawing_t nd = new struct drawing_s;
			nd->type = TEXT;
			nd->next = NULL;

			// get user defined values from xml
			nd->str = node->Value();
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &nd->p1[0]);
				ele->QueryDoubleAttribute("y", &nd->p1[1]);
				ele->QueryDoubleAttribute("z", &nd->p1[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &nd->c[0]);
				ele->QueryDoubleAttribute("g", &nd->c[1]);
				ele->QueryDoubleAttribute("b", &nd->c[2]);
				ele->QueryDoubleAttribute("alpha", &nd->c[3]);
			}

			// add object to linked list
			drawing_t dtmp = _drawings;
			if (dtmp == NULL)
				_drawings = nd;
			else {
				while (dtmp->next)
					dtmp = dtmp->next;
				dtmp->next = nd;
			}
		}

		// go to next node
		node = node->NextSiblingElement();
	}
#endif

	// get root node of xml file
	node = doc.FirstChildElement("sim")->FirstChildElement();

	// check if individual vs preconfig
	node->QueryIntAttribute("type", &_preconfig);

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "mobot") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = MOBOT;
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0; nr->angle4 = 0;
			nr->tracking = tracking;
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
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			nr->tracking = tracking;
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
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			nr->tracking = tracking;
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
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0;
			nr->tracking = tracking;
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
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
		else if ( !strcmp(node->Value(), "nxt") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = NXT;
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0; nr->angle4 = 0;
			nr->tracking = tracking;
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
				ele->QueryDoubleAttribute("w1", &(nr->angle1));
				ele->QueryDoubleAttribute("w2", &(nr->angle2));
			}
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
		else if ( !strcmp(node->Value(), "cubus") ) {
			xml_robot_t nr = new struct xml_robot_s;
			nr->type = CUBUS;
			nr->connected = 0;
			nr->x = 0; nr->y = 0; nr->z = 0;
			nr->psi = 0; nr->theta = 0; nr->phi = 0;
			nr->angle1 = 0; nr->angle2 = 0; nr->angle3 = 0; nr->angle4 = 0;
			nr->tracking = tracking;
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
			if (node->QueryIntAttribute("ground", &(nr->ground))) {
				nr->ground = -1;
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
				node->QueryIntAttribute("custom", &custom);
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
			else if ( !strcmp(node->Value(), "tinywheel") ) {
				ctype = TINYWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "wheel") ) {
				ctype = WHEEL;
				cnum = 1;
				node->QueryDoubleAttribute("radius", &size);
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
						if (atmp[i] == CASTER)
							side->QueryDoubleAttribute("custom", &size);
						else if (atmp[i] == WHEEL)
							side->QueryDoubleAttribute("radius", &size);
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
				nc->size = size;
				nc->next = NULL;
				tmp = _bot;
				while (tmp && tmp->id != rtmp[j])
					tmp = tmp->next;
				if (tmp == NULL) {
					fprintf(stderr, "Error: Robot %d could not be found in RoboSim config file.\n", rtmp[j]);
					exit(-1);
				}
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
		/*xml_robot_t rtmp = _bot;
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

	// check if any robots are possibly colliding
	if (!_preconfig) {
		xml_robot_t btmp = _bot;
		while (btmp) {
			xml_robot_t tmp2 = btmp->next;
			while (tmp2) {
				if ( (fabs(btmp->x - tmp2->x) < 0.1) && (fabs(btmp->y - tmp2->y) < 0.1) ) {
					fprintf(stderr, "Warning: Robot %d and Robot %d are possibly colliding.\n", btmp->id + 1, tmp2->id + 1);
					fprintf(stderr, "         Please check RoboSim GUI for x and y positions that may be too close.\n");
				}
				tmp2 = tmp2->next;
			}
			btmp = btmp->next;
		}
	}

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int RoboSim::init_viz(void) {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

    // construct the viewer
	MUTEX_INIT(&_viewer_mutex);
	_viewer = 1;

	// separate viewer and inserting into scenegraph
	_staging = new osg::Group;
	_ending = 0;

	// graphics haven't started yet
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
int RoboSim::addRobot(Robot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot struct
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

	// find specs about new robot
	xml_robot_t btmp = _bot;
	int form = 0;
	robot->getFormFactor(form);
	while (btmp) {
		if (btmp->type != form)
			btmp = btmp->next;
		else if (btmp->connected)
			btmp = btmp->next;
		else
			break;
	}


	// no robot found
	if (btmp == NULL || btmp->type != form) {
		switch (form) {
			case LINKBOTI:
				fprintf(stderr, "Error: Could not find LinkbotI in RoboSim GUI.\n");
				break;
			case LINKBOTL:
				fprintf(stderr, "Error: Could not find LinkbotL in RoboSim GUI.\n");
				break;
			case LINKBOTT:
				fprintf(stderr, "Error: Could not find LinkbotT in RoboSim GUI.\n");
				break;
			case MOBOT:
				fprintf(stderr, "Error: Could not find Mobot in RoboSim GUI.\n");
				break;
			case NXT:
				fprintf(stderr, "Error: Could not find NXT in RoboSim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, btmp->id, _robots.size()-1);

	// build robot
	robot->build(btmp);

	// robot now connected
	btmp->connected = 1;

#ifdef ENABLE_GRAPHICS
	// draw robot
	_robots.back()->node = robot->draw(_staging, btmp->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::addRobot(ModularRobot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot struct
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

	// find specs about new robot
	xml_robot_t btmp = _bot;
	int form = 0;
	robot->getFormFactor(form);
	while (btmp) {
		if (btmp->type != form)
			btmp = btmp->next;
		else if (btmp->connected)
			btmp = btmp->next;
		else
			break;
	}

	// no robot found
	if (btmp == NULL || btmp->type != form) {
		switch (form) {
			case LINKBOTI:
				fprintf(stderr, "Error: Could not find LinkbotI in RoboSim GUI.\n");
				break;
			case LINKBOTL:
				fprintf(stderr, "Error: Could not find LinkbotL in RoboSim GUI.\n");
				break;
			case LINKBOTT:
				fprintf(stderr, "Error: Could not find LinkbotT in RoboSim GUI.\n");
				break;
			case MOBOT:
				fprintf(stderr, "Error: Could not find Mobot in RoboSim GUI.\n");
				break;
			case NXT:
				fprintf(stderr, "Error: Could not find NXT in RoboSim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, btmp->id, _robots.size()-1);

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
		for (int i = 0; i < _robots.size(); i++) {
			if (_robots[i]->robot->getID() == ctmp->robot) {
				ModularRobot *r = dynamic_cast<ModularRobot *>(_robots[i]->robot);
				dBodyID body = r->getConnectorBodyID(ctmp->face1);
				dMatrix3 R;
				double m[3] = {0};
				r->getFaceParams(ctmp->face1, R, m);
				robot->build(btmp, R, m, body, ctmp);
				break;
			}
		}
	}
	else {
		robot->build(btmp);
	}

	// robot now connected
	btmp->connected = 1;

#ifdef ENABLE_GRAPHICS
	// draw robot
	_robots.back()->node = robot->draw(_staging, btmp->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::deleteRobot(int loc) {
#ifdef ENABLE_GRAPHICS
	// pause simulation to view results only on first delete
	MUTEX_LOCK(&(_pause_mutex));
	static int paused = 0;
	if (!paused++ && _running) {
		_pause = 1;
		MUTEX_UNLOCK(&(_pause_mutex));

		// get HUD and set message
		osg::Geode *geode = _shadowed->getParent(0)->getChild(1)->asGroup()->getChild(0)->asTransform()->getChild(0)->asGeode();
		osgText::Text *txt = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		txt->setText("Paused: Press any key to end");

		// sleep until pausing halted by user
		MUTEX_LOCK(&(_pause_mutex));
		while (_pause) {
			MUTEX_UNLOCK(&(_pause_mutex));
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
			MUTEX_LOCK(&(_pause_mutex));
		}
		MUTEX_UNLOCK(&(_pause_mutex));
	}
	MUTEX_UNLOCK(&(_pause_mutex));
#endif // ENABLE_GRAPHICS

	// lock robot data to delete
	MUTEX_LOCK(&_robot_mutex);

#ifdef ENABLE_GRAPHICS
	// save node location to delete later
	_ending = _robots[loc]->node;
#endif

	// remove robot
	_robots.erase(_robots.begin()+loc);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	if (!_robots.size())
		return 0;
	else
		return 1;
}

double RoboSim::getClock(void) {
	MUTEX_LOCK(&_clock_mutex);
	double clock = _clock;
	MUTEX_UNLOCK(&_clock_mutex);
	return clock;
}

int RoboSim::getCOR(double &robot, double &ground) {
	robot = _cor[0];
	ground = _cor[1];

	// success
	return 0;
}

int RoboSim::getMu(double &robot, double &ground) {
	robot = _mu[0];
	ground = _mu[1];

	// success
	return 0;
}

int RoboSim::getPause(void) {
	MUTEX_LOCK(&_pause_mutex);
	int pause = _pause;
	MUTEX_UNLOCK(&_pause_mutex);
	return pause;
}

double RoboSim::getStep(void) {
	MUTEX_LOCK(&_step_mutex);
	double step = _step;
	MUTEX_UNLOCK(&_step_mutex);
	return step;
}

int RoboSim::getUnits(void) {
	return _us;
}

int RoboSim::runSimulation(void) {
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);

	// success
	return 0;
}

int RoboSim::setCollisions(int mode) {
	switch (mode) {
		case 0:
			_collision = false;
			break;
		case 1:
			_collision = true;
			break;
		case 2:
			_collision = _collision ? false : true;
			break;
	}

	// success
	return 0;
}

int RoboSim::setCOR(double robot, double ground) {
	_cor[0] = robot;
	_cor[1] = ground;

	// success
	return 0;
}

int RoboSim::setMu(double robot, double ground) {
	_mu[0] = robot;
	_mu[1] = ground;

	// success
	return 0;
}

int RoboSim::setPause(int mode) {
	// lock pause
	MUTEX_LOCK(&_pause_mutex);

	// switch pause variable
	switch (mode) {
		case 0:
			_pause = 0;
			break;
		case 1:
			_pause = 1;
			break;
		case 2:
			_pause = _pause ? 0: 1;
			break;
	}

	// unlock pause
	MUTEX_UNLOCK(&_pause_mutex);

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
	int i, num = 20;
	unsigned int sum = 0, clock = 0, restart = 0;
	unsigned int *dt = new unsigned int[num]();
#ifdef _WIN32
	DWORD start_time = 0, start = 0, end = 0;
#else
	struct timespec s_time;
	unsigned int start_time = 0, start = 0, end = 0;
#endif

	MUTEX_LOCK(&(sim->_running_mutex));
	while (sim->_running) {
		MUTEX_UNLOCK(&(sim->_running_mutex));

		// lock pause variable
		MUTEX_LOCK(&(sim->_pause_mutex));

		if (sim->_rt) {
			// get starting times
#ifdef _WIN32
			start = GetTickCount();
#else
			clock_gettime(CLOCK_REALTIME, &s_time);
			start = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif
		}

		while (!(sim->_pause) && sim->_running) {
			// unlock pause variable
			MUTEX_UNLOCK(&(sim->_pause_mutex));

			if (sim->_rt) {
				// get start time of execution in milliseconds
#ifdef _WIN32
				start_time = GetTickCount();
#else
				clock_gettime(CLOCK_REALTIME, &s_time);
				start_time = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif
			}

			// perform pre-collision updates
			MUTEX_LOCK(&(sim->_robot_mutex));
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_CREATE(&(sim->_robots[j]->thread), (void* (*)(void *))&Robot::simPreCollisionThreadEntry, (void *)sim->_robots[j]->robot);
			}
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_JOIN(sim->_robots[j]->thread);
			}

			// perform ode update
			dSpaceCollide(sim->_space, sim, &sim->collision);
			dWorldStep(sim->_world, sim->_step);
			MUTEX_LOCK(&(sim->_clock_mutex));
			sim->_clock += sim->_step;
			MUTEX_UNLOCK(&(sim->_clock_mutex));
			dJointGroupEmpty(sim->_group);

			// perform post-collision updates
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_CREATE(&(sim->_robots[j]->thread), (void* (*)(void *))&Robot::simPostCollisionThreadEntry, (void *)sim->_robots[j]->robot);
			}
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_JOIN(sim->_robots[j]->thread);
			}
			MUTEX_UNLOCK(&(sim->_robot_mutex));

			// get ending time
			if (sim->_rt) {
#ifdef _WIN32
				end = GetTickCount();
#else
				clock_gettime(CLOCK_REALTIME, &s_time);
				end = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif

				// running mean of last four time steps
				if (!restart) {
					for (i = num-2; i >= 0; i--) { dt[i+1] = dt[i]; }
					dt[0] = end - start_time;
					for (i = 0; i < num; i++) { sum += dt[i]; }
					sum /= num;
				}
				// on restart, reset all time steps
				else {
					restart = 0;
					sum = dt[0];
					dt[0] = num;
					for (i = 1; i < num; i++) { dt[i] = 0; }
				}

				// lock step variable
				MUTEX_LOCK(&(sim->_step_mutex));

				// set next time step if calculations took longer than step
				if ( (end - start) > ((unsigned int)(sim->_clock*1000) - clock/1000) ) {
					sim->_step = ((end - start - ((unsigned int)(sim->_clock*1000) - clock/1000))/num + sum)/1000.0;
				}
				// sleep until clock time equals step time
				else {
					sim->_step = sum/1000.0;
#ifdef _WIN32
					Sleep((unsigned int)(sim->_clock*1000) - (end - start) - clock/1000);
#else
					usleep(sim->_clock*1000000 - ((end - start)*1000) - clock);
#endif
				}

				// make sure time step is large enough
				sim->_step = (sim->_step*1000 < 4) ? 0.004 : sim->_step;

				// unlock step variable
				MUTEX_UNLOCK(&(sim->_step_mutex));
			}

			// lock pause variable
			MUTEX_LOCK(&(sim->_pause_mutex));
		}
		// unlock pause variable
		MUTEX_UNLOCK(&(sim->_pause_mutex));

		if (sim->_rt) {
			// reset clock counters on pausing
			restart = 1;
			end = start;
			clock = (unsigned int)(sim->_clock*1000000);
		}

		// lock running mutex
		MUTEX_LOCK(&(sim->_running_mutex));
	}
	// unlock running variable
	MUTEX_UNLOCK(&(sim->_running_mutex));

	// cleanup
	delete [] dt;

	// end
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

	// do not collide spaces (robots) together
	if (!ptr->_collision && dGeomIsSpace(o1) && dGeomIsSpace(o2)) {
		return;
	}

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
	traits->width = width/2;
	traits->height = 3*width/8;
	traits->x = width - traits->width - 10;
	traits->y = height - traits->height - 50;
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

    // creating the viewer
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	// set threading model
	viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// camera properties
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);
	viewer->addSlave(camera.get());
	viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	viewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
	viewer->getCamera()->setCullingMode(osgUtil::CullVisitor::NO_CULLING);
	viewer->getCamera()->setNearFarRatio(0.00001);

	// viewer camera properties
	osg::ref_ptr<osgGA::OrbitManipulator> cameraManipulator = new osgGA::OrbitManipulator();
	cameraManipulator->setDistance(0.1);
	cameraManipulator->setAllowThrow(false);
	cameraManipulator->setWheelZoomFactor(0);
	cameraManipulator->setVerticalAxisFixed(true);
	cameraManipulator->setElevation(0.5);
	viewer->setCameraManipulator(cameraManipulator);
	viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.6, -0.8, 0.5), osg::Vec3f(0.1, 0.3, 0), osg::Vec3f(0, 0, 1));

	// Creating the root node
	osg::ref_ptr<osg::Group> root = new osg::Group;

	// add shadows
	osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
	sim->_shadowed = shadowedScene;
	root->addChild(shadowedScene);
	shadowedScene->setReceivesShadowTraversalMask(RECEIVES_SHADOW_MASK);
	shadowedScene->setCastsShadowTraversalMask(CASTS_SHADOW_MASK);
	//osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
	//sm->setTextureSize(osg::Vec2s(1024, 1024));
	//shadowedScene->setShadowTechnique(sm.get());

	// add light source
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0.5, 1.0, 1.0, 0.0));
	ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1.0));
	ls->getLight()->setAmbient(osg::Vec4(1, 1, 1, 1.0));
	ls->getLight()->setConstantAttenuation(0.05);
	ls->getLight()->setQuadraticAttenuation(0.05);
	shadowedScene->addChild(ls.get());

	// load terrain node
	osg::ref_ptr<osg::Depth> t_depth = new osg::Depth;
	t_depth->setFunction(osg::Depth::LEQUAL);
	t_depth->setRange(1.0, 1.0);
	osg::ref_ptr<osg::StateSet> t_stateset = new osg::StateSet();
	t_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	t_stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	t_stateset->setAttributeAndModes(t_depth, osg::StateAttribute::ON);
	t_stateset->setRenderBinDetails(-1, "RenderBin");
	t_stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::ref_ptr<osg::Node> t_geode = osgDB::readNodeFile(TEXTURE_PATH(ground/terrain.3ds));
	t_geode->setCullingActive(false);
	t_geode->setStateSet(t_stateset);
	osg::ref_ptr<osg::PositionAttitudeTransform> t_transform = new osg::PositionAttitudeTransform();
	t_transform->setScale(osg::Vec3d(2, 2, 0.001));
	t_transform->setCullingActive(false);
	t_transform->addChild(t_geode);
	osg::ref_ptr<osgUtil::LineSegmentIntersector> r_segment = new osgUtil::LineSegmentIntersector(osg::Vec3d(0, 0, 999), osg::Vec3d(0, 0, -999));
	osgUtil::IntersectionVisitor r_visitor(r_segment);
	t_transform->accept(r_visitor);
	osgUtil::LineSegmentIntersector::Intersection r_hits = r_segment->getFirstIntersection();
	osg::Vec3d r_pos = r_hits.getWorldIntersectPoint();
	t_transform->setPosition(osg::Vec3d(r_pos[0], r_pos[1], -r_pos[2]));
	//t_transform->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
	t_transform->setNodeMask(~IS_PICKABLE_MASK);
	shadowedScene->addChild(t_transform);

	// set up HUD
	osg::ref_ptr<osg::Geode> HUDGeode = new osg::Geode();
	osg::ref_ptr<osgText::Text> textHUD = new osgText::Text();
	osg::ref_ptr<osg::Projection> HUDProjectionMatrix = new osg::Projection;
	osg::ref_ptr<osg::MatrixTransform> HUDModelViewMatrix = new osg::MatrixTransform;
	osg::ref_ptr<osg::StateSet> HUDStateSet = new osg::StateSet();
	HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0,traits->width,0,traits->height));
	HUDProjectionMatrix->addChild(HUDModelViewMatrix);
	HUDModelViewMatrix->setMatrix(osg::Matrix::identity());
	HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	HUDModelViewMatrix->addChild(HUDGeode);
	HUDGeode->setStateSet(HUDStateSet);
	HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
	HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	HUDStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	HUDStateSet->setRenderBinDetails(11, "RenderBin");
	HUDGeode->addDrawable(textHUD);
	textHUD->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	textHUD->setMaximumWidth(traits->width);
	textHUD->setCharacterSize(15);
	if (sim->_pause) textHUD->setText("Paused: Press any key to start");
	textHUD->setAxisAlignment(osgText::Text::SCREEN);
	textHUD->setAlignment(osgText::Text::CENTER_CENTER);
	textHUD->setPosition(osg::Vec3(traits->width/2, 50, -1.5));
	textHUD->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	textHUD->setBackdropColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	textHUD->setBackdropOffset(0.1);
	root->addChild(HUDProjectionMatrix);

	// grid
	if ( (int)(sim->_grid[6]) ) {
		// grid lines for each sub-foot
		double minx = (int)((sim->_grid[2]*1.01)/sim->_grid[0])*sim->_grid[0];
		double miny = (int)((sim->_grid[4]*1.01)/sim->_grid[0])*sim->_grid[0];
		int numx = (int)((sim->_grid[3] - minx)/sim->_grid[0]+1);
		int numy = (int)((sim->_grid[5] - miny)/sim->_grid[0]+1);
		int numVertices = 2*numx + 2*numy;
		osg::Geode *gridGeode = new osg::Geode();
		osg::Geometry *gridLines = new osg::Geometry();
		osg::Vec3 *myCoords = new osg::Vec3[numVertices]();
		// draw x lines
		for (int i = 0, j = 0; i < numx; i++) {
			myCoords[j++] = osg::Vec3(minx + i*sim->_grid[0], sim->_grid[4], 0.0);
			myCoords[j++] = osg::Vec3(minx + i*sim->_grid[0], sim->_grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx; i < numy; i++) {
			myCoords[j++] = osg::Vec3(sim->_grid[2], miny + i*sim->_grid[0], 0.0);
			myCoords[j++] = osg::Vec3(sim->_grid[3], miny + i*sim->_grid[0], 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices = new osg::Vec3Array(numVertices, myCoords);
		gridLines->setVertexArray(vertices);
		gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
		// set color
		osg::Vec4Array *colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );	// white
		gridLines->setColorArray(colors);
		gridLines->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth = new osg::LineWidth();
		linewidth->setWidth(1.0f);
		gridGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode->getOrCreateStateSet()->setRenderBinDetails(-1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode->addDrawable(gridLines);
		shadowedScene->addChild(gridGeode);

		// grid lines for each foot
		double minx2 = (int)((sim->_grid[2]*1.01)/sim->_grid[1])*sim->_grid[1];
		double miny2 = (int)((sim->_grid[4]*1.01)/sim->_grid[1])*sim->_grid[1];
		int numx2 = (int)((sim->_grid[3] - minx2)/sim->_grid[1]+1);
		int numy2 = (int)((sim->_grid[5] - miny2)/sim->_grid[1]+1);
		int numVertices2 = 2*numx2 + 2*numy2;
		osg::Geode *gridGeode2 = new osg::Geode();
		osg::Geometry *gridLines2 = new osg::Geometry();
		osg::Vec3 *myCoords2 = new osg::Vec3[numVertices2]();
		// draw x lines
		for (int i = 0, j = 0; i < numx2; i++) {
			myCoords2[j++] = osg::Vec3(minx2 + i*sim->_grid[1], sim->_grid[4], 0.0);
			myCoords2[j++] = osg::Vec3(minx2 + i*sim->_grid[1], sim->_grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx2; i < numy2; i++) {
			myCoords2[j++] = osg::Vec3(sim->_grid[2], miny2 + i*sim->_grid[1], 0.0);
			myCoords2[j++] = osg::Vec3(sim->_grid[3], miny2 + i*sim->_grid[1], 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices2 = new osg::Vec3Array(numVertices2, myCoords2);
		gridLines2->setVertexArray(vertices2);
		gridLines2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices2));
		// set color
		osg::Vec4Array *colors2 = new osg::Vec4Array;
		colors2->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );	// red
		gridLines2->setColorArray(colors2);
		gridLines2->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth2 = new osg::LineWidth();
		linewidth2->setWidth(2.0f);
		gridGeode2->getOrCreateStateSet()->setAttributeAndModes(linewidth2, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode2->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode2->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode2->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode2->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode2->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode2->addDrawable(gridLines2);
		shadowedScene->addChild(gridGeode2);

		// x- and y-axis lines
		osg::Geode *gridGeode3 = new osg::Geode();
		osg::Geometry *gridLines3 = new osg::Geometry();
		osg::Vec3 myCoords3[4];
		if ( fabs(sim->_grid[3]) > fabs(sim->_grid[2]) ) {
			if (sim->_grid[2] < -EPSILON)
				myCoords3[0] = osg::Vec3(-sim->_grid[3], 0, 0);
			else
				myCoords3[0] = osg::Vec3(0, 0, 0);
			myCoords3[1] = osg::Vec3(sim->_grid[3], 0, 0);
		}
		else {
			if (sim->_grid[3] < -EPSILON)
				myCoords3[1] = osg::Vec3(0, 0, 0);
			else
				myCoords3[1] = osg::Vec3(sim->_grid[3], 0, 0);
			myCoords3[0] = osg::Vec3(sim->_grid[2], 0, 0);
		}
		if ( fabs(sim->_grid[5]) > fabs(sim->_grid[4]) ) {
			if (sim->_grid[4] < -EPSILON)
				myCoords3[2] = osg::Vec3(-sim->_grid[5], 0, 0);
			else
				myCoords3[2] = osg::Vec3(0, 0, 0);
			myCoords3[3] = osg::Vec3(0, sim->_grid[5], 0);
		}
		else {
			if (sim->_grid[5] < -EPSILON)
				myCoords3[3] = osg::Vec3(0, 0, 0);
			else
				myCoords3[3] = osg::Vec3(0, sim->_grid[5], 0);
			myCoords3[2] = osg::Vec3(0, sim->_grid[4], 0);
		}
		// add vertices
		osg::Vec3Array *vertices3 = new osg::Vec3Array(4, myCoords3);
		gridLines3->setVertexArray(vertices3);
		gridLines3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4));
		// set color
		osg::Vec4Array *colors3 = new osg::Vec4Array;
		colors3->push_back(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
		gridLines3->setColorArray(colors3);
		gridLines3->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth3 = new osg::LineWidth();
		linewidth3->setWidth(3.0f);
		gridGeode3->getOrCreateStateSet()->setAttributeAndModes(linewidth3, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode3->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode3->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode3->getOrCreateStateSet()->setRenderBinDetails(-1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode3->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode3->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode3->addDrawable(gridLines3);
		shadowedScene->addChild(gridGeode3);

		// x-axis label
		osg::ref_ptr<osg::Billboard> xbillboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> xtext = new osgText::Text();
		xtext->setText("x");
		xtext->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		xtext->setAlignment(osgText::Text::CENTER_BASE_LINE);
		xtext->setRotation(osg::Quat(-1.57, osg::Vec3(0, 0, 1)));
		xtext->setCharacterSize(50);
		xtext->setColor(osg::Vec4(0, 0, 0, 1));
		xtext->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		if ( fabs(sim->_grid[3]) > fabs(sim->_grid[2]) ) {
			if (sim->_grid[3] < EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(0.05, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(sim->_grid[3] + 0.05, 0.0, 0.0));
		}
		else {
			if (sim->_grid[2] < -EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(sim->_grid[3] + 0.05, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(0.05, 0.0, 0.0));
		}
		xbillboard->setMode(osg::Billboard::AXIAL_ROT);
		xbillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNodeMask(~IS_PICKABLE_MASK);
		root->addChild(xbillboard);

		// y-axis label
		osg::ref_ptr<osg::Billboard> ybillboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> ytext = new osgText::Text();
		ytext->setText("y");
		ytext->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		ytext->setAlignment(osgText::Text::CENTER_BASE_LINE);
		ytext->setCharacterSize(50);
		ytext->setColor(osg::Vec4(0, 0, 0, 1));
		ytext->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		if ( fabs(sim->_grid[5]) > fabs(sim->_grid[4]) ) {
			if (sim->_grid[5] < EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.05, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, sim->_grid[5] + 0.05, 0.0));
		}
		else {
			if (sim->_grid[4] < -EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, sim->_grid[5] + 0.05, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.05, 0.0));
		}
		ybillboard->setMode(osg::Billboard::AXIAL_ROT);
		ybillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNodeMask(~IS_PICKABLE_MASK);
		root->addChild(ybillboard);

		// x grid numbering
		osg::ref_ptr<osg::Billboard> xnum_billboard = new osg::Billboard();
		char text[50];
		osg::ref_ptr<osgText::Text> xzero_text = new osgText::Text();
		xzero_text->setText("0");
		xzero_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		xzero_text->setAlignment(osgText::Text::CENTER_CENTER);
		xzero_text->setCharacterSize(30);
		xzero_text->setColor(osg::Vec4(0, 0, 0, 1));
		xzero_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		xnum_billboard->addDrawable(xzero_text, osg::Vec3d(-0.5*sim->_grid[0], -0.5*sim->_grid[0], 0.0));
		// positive
		for (int i = 1; i < (int)(sim->_grid[3]/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumpos_text = new osgText::Text();
			if (sim->_us) sprintf(text, "   %.0lf", 39.37*i*sim->_grid[1]);
			else sprintf(text, "   %.0lf", 100*i*sim->_grid[1]);
			xnumpos_text->setText(text);
			xnumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumpos_text->setCharacterSize(30);
			xnumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumpos_text, osg::Vec3d(i*sim->_grid[1], -0.5*sim->_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(sim->_grid[2])/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumneg_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", -39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", -100*i*sim->_grid[1]);
			xnumneg_text->setText(text);
			xnumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumneg_text->setCharacterSize(30);
			xnumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumneg_text, osg::Vec3d(-i*sim->_grid[1], -0.5*sim->_grid[0], 0.0));
		}
		xnum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		xnum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		xnum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		xnum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		root->addChild(xnum_billboard);

		// y grid numbering
		osg::ref_ptr<osg::Billboard> ynum_billboard = new osg::Billboard();
		// positive
		for (int i = 1; i < (int)(sim->_grid[5]/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumpos_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", 39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", 100*i*sim->_grid[1]);
			ynumpos_text->setText(text);
			ynumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumpos_text->setCharacterSize(30);
			ynumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumpos_text, osg::Vec3d(0, i*sim->_grid[1] + 0.5*sim->_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(sim->_grid[4])/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumneg_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", -39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", -100*i*sim->_grid[1]);
			ynumneg_text->setText(text);
			ynumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumneg_text->setCharacterSize(30);
			ynumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumneg_text, osg::Vec3d(0, -i*sim->_grid[1] - 0.5*sim->_grid[0], 0.0));
		}
		ynum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		ynum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		ynum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		ynum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		root->addChild(ynum_billboard);
	}

	// skybox
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	osg::ref_ptr<osg::TexEnv> te = new osg::TexEnv;
	te->setMode(osg::TexEnv::REPLACE);
	stateset->setTextureAttributeAndModes(0, te, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexGen> tg = new osg::TexGen;
	tg->setMode(osg::TexGen::NORMAL_MAP);
	stateset->setTextureAttributeAndModes(0, tg, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexMat> tm = new osg::TexMat;
	stateset->setTextureAttribute(0, tm);
	osg::ref_ptr<osg::TextureCubeMap> skymap = new osg::TextureCubeMap;
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
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setRange(1.0,1.0);
	stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);
	stateset->setRenderBinDetails(-1, "RenderBin");
	stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::ref_ptr<osg::Drawable> drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),1));
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setCullingActive(false);
	geode->setStateSet(stateset);
	geode->addDrawable(drawable);
	osg::ref_ptr<osg::Transform> transform = new MoveEarthySkyWithEyePointTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);
	osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
	clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TexMatCallback(*tm));
	clearNode->addChild(transform);
	clearNode->setNodeMask(~IS_PICKABLE_MASK);
	root->addChild(clearNode);

	// ground objects
	const double *pos;
	double radius, length;
	dQuaternion quat;
	dVector3 dims;
	osg::Box *box;
	osg::Cylinder *cyl;
	osg::Sphere *sph;
	ground_t gtmp = sim->_ground->next;	// skip default ground plane node
	while (gtmp) {
		osg::ref_ptr<osg::Group> ground = new osg::Group();
		osg::ref_ptr<osg::Geode> body = new osg::Geode;
		osg::ref_ptr<osg::ShapeDrawable> shape;
		pos = dGeomGetOffsetPosition(gtmp->geom);
		dGeomGetOffsetQuaternion(gtmp->geom, quat);
		switch (dGeomGetClass(gtmp->geom)) {
			case dBoxClass:
				dGeomBoxGetLengths(gtmp->geom, dims);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), dims[0], dims[1], dims[2]);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				shape = new osg::ShapeDrawable(box);
				shape->setColor(osg::Vec4(gtmp->r, gtmp->g, gtmp->b, gtmp->alpha));
				body->addDrawable(shape);
				break;
			case dCylinderClass:
				dGeomCylinderGetParams(gtmp->geom, &radius, &length);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, length);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				shape = new osg::ShapeDrawable(cyl);
				shape->setColor(osg::Vec4(gtmp->r, gtmp->g, gtmp->b, gtmp->alpha));
				body->addDrawable(shape);
				break;
			case dSphereClass:
				radius = dGeomSphereGetRadius(gtmp->geom);
				sph = new osg::Sphere(osg::Vec3d(pos[0], pos[1], pos[2]), radius);
				shape = new osg::ShapeDrawable(sph);
				shape->setColor(osg::Vec4(gtmp->r, gtmp->g, gtmp->b, gtmp->alpha));
				body->addDrawable(shape);
				break;
		}

		// set rendering properties
		body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		body->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		body->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		body->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
		body->setCullingActive(false);

		// add positioning capability
		osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
		pat->addChild(body.get());
		ground->addChild(pat.get());

		// add update callback
		ground->setUpdateCallback(new groundNodeCallback(gtmp));

		// set user properties of node
		ground->setName("ground");

		// optimize object
		osgUtil::Optimizer optimizer;
		optimizer.optimize(ground);

		// add to scenegraph
		shadowedScene->addChild(ground);

		// next object
		gtmp = gtmp->next;
	}

	// drawing objects
	drawing_t dtmp = sim->_drawings;
	while (dtmp) {
		if (dtmp->type == DOT) {
			// create sphere
			osg::Sphere *sphere = new osg::Sphere(osg::Vec3d(dtmp->p1[0], dtmp->p1[1], dtmp->p1[2]), dtmp->i/500.0);
			osg::Geode *point = new osg::Geode;
			osg::ShapeDrawable *pointDrawable = new osg::ShapeDrawable(sphere);
			point->addDrawable(pointDrawable);
			pointDrawable->setColor(osg::Vec4(dtmp->c[0], dtmp->c[1], dtmp->c[2], dtmp->c[3]));

			// set rendering properties
			point->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			point->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			// optimize object
			osgUtil::Optimizer optimizer;
			optimizer.optimize(point);

			// add to scenegraph
			shadowedScene->addChild(point);
		}
		else if (dtmp->type == LINE) {
			// draw line
			osg::Geode *line = new osg::Geode();
			osg::Geometry *geom = new osg::Geometry();
			osg::Vec3Array *vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(dtmp->p1[0], dtmp->p1[1], dtmp->p1[2]));
			vert->push_back(osg::Vec3(dtmp->p2[0], dtmp->p2[1], dtmp->p2[2]));
			geom->setVertexArray(vert);
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
			osg::Vec4Array *colors = new osg::Vec4Array;
			colors->push_back(osg::Vec4(dtmp->c[0], dtmp->c[1], dtmp->c[2], dtmp->c[3]));
			geom->setColorArray(colors);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			line->addDrawable(geom);

			// set line width
			osg::LineWidth *width = new osg::LineWidth();
			width->setWidth(dtmp->i*3.0f);
			line->getOrCreateStateSet()->setAttributeAndModes(width, osg::StateAttribute::ON);

			// set rendering properties
			line->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			line->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			// optimize object
			osgUtil::Optimizer optimizer;
			optimizer.optimize(line);

			// add to scenegraph
			shadowedScene->addChild(line);
		}
		else if (dtmp->type == TEXT) {
			// generate text node
			osgText::Text *label = new osgText::Text();
			osg::Geode *label_geode = new osg::Geode();
			label_geode->addDrawable(label);
			label->setAlignment(osgText::Text::CENTER_CENTER);
			label->setAxisAlignment(osgText::Text::SCREEN);
			label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			label->setCharacterSize(25);
			label->setColor(osg::Vec4(dtmp->c[0], dtmp->c[1], dtmp->c[2], dtmp->c[3]));
			label->setDrawMode(osgText::Text::TEXT);
			label->setPosition(osg::Vec3(dtmp->p1[0], dtmp->p1[1], dtmp->p1[2]));
			label->setText(dtmp->str);

			// set rendering properties
			label_geode->getOrCreateStateSet()->setRenderBinDetails(22, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			label_geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			// optimize object
			osgUtil::Optimizer optimizer;
			optimizer.optimize(label_geode);

			// add to scenegraph
			shadowedScene->addChild(label_geode);
		}
		// next object
		dtmp = dtmp->next;
	}

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(root);

	// viewer event handlers
	viewer->addEventHandler(new keyboardEventHandler(textHUD));
	viewer->addEventHandler(new osgGA::StateSetManipulator(camera->getOrCreateStateSet()));
	viewer->addEventHandler(new osgViewer::WindowSizeHandler);
	viewer->addEventHandler(new pickHandler());

	// set viewable
	viewer->setSceneData(root);

	// signal connection functions that graphics are set up
	SIGNAL(&(sim->_graphics_cond), &(sim->_graphics_mutex), sim->_graphics = 1);

	// run viewer
	MUTEX_LOCK(&(sim->_viewer_mutex));
	while (sim->_viewer && !viewer->done()) {
		MUTEX_UNLOCK(&(sim->_viewer_mutex));

		viewer->frame();
		if (sim->_staging->getNumChildren()) {
			sim->_shadowed->addChild(sim->_staging->getChild(0));
			sim->_staging->removeChild(0, 1);
		}
		if (sim->_ending) {
			sim->_shadowed->removeChild(sim->_shadowed->getChild(sim->_ending));
			sim->_ending = 0;
		}

		MUTEX_LOCK(&(sim->_viewer_mutex));
	}
	MUTEX_UNLOCK(&(sim->_viewer_mutex));

	// clean up viewer & root
	viewer->setSceneData(NULL);
#ifdef _WIN32_
	delete viewer;
#endif

	// trigger end of code when graphics window is closed
	SIGNAL(&(sim->_running_cond), &(sim->_running_mutex), sim->_running = 0);

	// return
	return arg;
}
#endif // ENABLE_GRAPHICS

