#include <stdbool.h>
#include "robotsim.h"
using namespace std;

CRobotSim::CRobotSim(void) {
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
	pthread_create(&_simulation, NULL, (void* (*)(void *))&CRobotSim::simulationThread, (void *)this);
	pthread_mutex_init(&_robot_mutex, NULL);
	pthread_mutex_init(&_ground_mutex, NULL);

	// variables to keep track of progress of simulation
	_groundNumber = 1;
    _step = 0.004;
	_clock = 0;

	robot_init();

#ifdef ENABLE_GRAPHICS
	graphics_init();
#endif /* ENABLE_GRAPHICS */
}

CRobotSim::~CRobotSim(void) {
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

void CRobotSim::robot_init(void) {
	FILE *fp;
	char type[16] = {'\0'}, line[1024];
	char *begptr, *endptr, string[32];
	int i, *rtmp, *ftmp, ctype, cnum;
	bot = NULL;
	for ( int i = 0; i < NUM_TYPES; i++ ) {
		_robot[i] = NULL;
		_robotNumber[i] = 0;
		_robotConnected[i] = 0;
		_robotThread[i] = NULL;
	}

	// open config file
    fp = fopen(".robotsimrc", "r");
    if (fp == NULL) {
        printf("Error: cannot read config file\n");
        exit(-1);
    }

	// scan config file
    fgets(line, 1024, fp);
	while ( !feof(fp) ) {
		// skip comments
		if (line[0] == '#') {
    	    fgets(line, 1024, fp);
			continue;
		}

		// get type of line data (robot, connector)
		strncpy(type, line, strstr(line, ":") - line);
		// if robot
		if ( !strcmp(type, "mobot") ) {
			bot_t nr = (bot_t)malloc(sizeof(struct bot_s));
			// get type
			if ( !strcmp(type, "mobot") ) {
				nr->type = 0;
				_robotNumber[MOBOT]++;
			}
			else if ( !strcmp(type, "imobot") ) {
				nr->type = 1;
				_robotNumber[IMOBOT]++;
			}

			// get id
			begptr = strstr(line, ":");
			if ( begptr != NULL ) {
				endptr = strstr(begptr, ";");
				strncpy(string, begptr, endptr - begptr);
				string[endptr - begptr] = '\0';
				sscanf(string, "%*s%d", &nr->id);
			}
			// get position
			begptr = strstr(line, "position");
			if ( begptr != NULL ) {
				endptr = strstr(begptr, ";");
				strncpy(string, begptr, endptr - begptr);
				string[endptr - begptr] = '\0';
				sscanf(string, "%*s%lf%*s%lf%*s%lf", &nr->x, &nr->y, &nr->z);
			}
			// get rotation
			begptr = strstr(line, "rotation");
			if ( begptr != NULL ) {
				endptr = strstr(begptr, ";");
				strncpy(string, begptr, endptr - begptr);
				string[endptr - begptr] = '\0';
				sscanf(string, "%*s%lf%*s%lf%*s%lf", &nr->psi, &nr->theta, &nr->phi);
			}
			// get joint angles
			begptr = strstr(line, "joint");
			if ( begptr != NULL ) {
				endptr = strstr(begptr, ";");
				strncpy(string, begptr, endptr - begptr);
				string[endptr - begptr] = '\0';
				sscanf(string, "%*s%lf%*s%lf%*s%lf%*s%lf", &nr->angle1, &nr->angle2, &nr->angle3, &nr->angle4);
			}
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
		// else its a connector
		else {
			// get type
			if ( !strcmp(type, "simple") ) {
				ctype = SIMPLE;
				cnum = 2;
			}
			else if ( !strcmp(type, "caster") ) {
				ctype = CASTER;
				cnum = 1;
			}
			else if ( !strcmp(type, "bigwheel") ) {
				ctype = BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(type, "smallwheel") ) {
				ctype = SMALLWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(type, "l") ) {
				ctype = L;
				cnum = 3;
			}
			else if ( !strcmp(type, "tank") ) {
				ctype = TANK;
				cnum = 2;
			}
			rtmp = (int *)malloc(sizeof(int)*cnum);
			ftmp = (int *)malloc(sizeof(int)*cnum);

			// store connected robots to tmp variables
			begptr = strstr(line, ":");
			for (i = 0; i < cnum; i++) {
				if ( begptr != NULL ) {
					endptr = strstr(begptr, ";");
					if (!endptr)
						break;
					strncpy(string, begptr, endptr - begptr);
					string[endptr - begptr] = '\0';
					sscanf(string, "%*c%d%*s%d", &rtmp[i], &ftmp[i]);
				}
				begptr = endptr+1;
			}

			// store connectors to each robot
			bot_t tmp;
			Conn_t *ctmp;
			for (int j = 0; j < i; j++) {
				Conn_t *nc = (Conn_t *)malloc(sizeof(struct Conn_s));
				nc->robot = rtmp[0]; 
				nc->face1 = ftmp[0]; 
				nc->face2 = ftmp[j]; 
				nc->type = ctype; 
				nc->next = NULL;
				tmp = bot;
				while (tmp->id != rtmp[j])
					tmp = tmp->next;
				ctmp = tmp->conn;
				if ( tmp->conn == NULL )
					tmp->conn = nc;
				else {
					while (ctmp->next)
						ctmp = ctmp->next;
					ctmp->next = nc;
				}
			}
		}

		// debug printing
		/*bot_t rtmp = bot;
		while (rtmp) {
			printf("type = %d, id = %d\n", rtmp->type, rtmp->id);
			printf("x = %lf, y = %lf, z = %lf\n", rtmp->x, rtmp->y, rtmp->z);
			printf("psi = %lf, theta = %lf, phi = %lf\n", rtmp->psi, rtmp->theta, rtmp->phi);
			printf("angle1 = %lf, angle2 = %lf, angle3 = %lf, angle4 = %lf\n", rtmp->angle1, rtmp->angle2, rtmp->angle3, rtmp->angle4);
			Conn_t *ctmp = rtmp->conn;
			while (ctmp) {
				printf("on face %d: connect with robot %d on his face %d with type: %d\n", ctmp->face2, ctmp->robot, ctmp->face1, ctmp->type);
				ctmp = ctmp->next;
			}
			printf("next = %p\n", rtmp->next);
			printf("\n");
			rtmp = rtmp->next;
		}
		printf("\n\n\n");*/

		// get new line
    	fgets(line, 1024, fp);
	}

	// close config file
    fclose(fp);

	// set up robot variables
	for (int i = 0; i < NUM_TYPES; i++) {
		_robotThread[i] = new pthread_t[_robotNumber[i]];
		_robot[i] =  (CRobot **)realloc(_robot[i], (_robotNumber[i] + 1)*sizeof(CRobot *));
	}
}

#ifdef ENABLE_GRAPHICS
osg::TextureCubeMap* CRobotSim::readCubeMap(void) {
    osg::TextureCubeMap* cubemap = new osg::TextureCubeMap;
    //#define CUBEMAP_FILENAME(face) "nvlobby_" #face ".png"
    //#define CUBEMAP_FILENAME(face) "data/Cubemap_axis/" #face ".png"
    //#define CUBEMAP_FILENAME(face) "data/Cubemap_snow/" #face ".jpg"
    //#define SKY_FILENAME(face) "data/ground/sky/" #face ".jpg"
    #define SKY_FILENAME(face) "data/ground/checkered/" #face ".jpg"

    /*osg::Image* imagePosX = osgDB::readImageFile(CUBEMAP_FILENAME(posx));
    osg::Image* imageNegX = osgDB::readImageFile(CUBEMAP_FILENAME(negx));
    osg::Image* imagePosY = osgDB::readImageFile(CUBEMAP_FILENAME(posy));
    osg::Image* imageNegY = osgDB::readImageFile(CUBEMAP_FILENAME(negy));
    osg::Image* imagePosZ = osgDB::readImageFile(CUBEMAP_FILENAME(posz));
    osg::Image* imageNegZ = osgDB::readImageFile(CUBEMAP_FILENAME(negz));*/
    /*osg::Image* imagePosX = osgDB::readImageFile(SKY_FILENAME(frontsh));
    osg::Image* imageNegX = osgDB::readImageFile(SKY_FILENAME(backsh));
    osg::Image* imagePosY = osgDB::readImageFile(SKY_FILENAME(botsh));
    osg::Image* imageNegY = osgDB::readImageFile(SKY_FILENAME(toptsh));
    osg::Image* imagePosZ = osgDB::readImageFile(SKY_FILENAME(leftsh));
    osg::Image* imageNegZ = osgDB::readImageFile(SKY_FILENAME(rightsh));*/
    osg::Image* imagePosX = osgDB::readImageFile(SKY_FILENAME(checkered_front));
    osg::Image* imageNegX = osgDB::readImageFile(SKY_FILENAME(checkered_back));
    osg::Image* imagePosY = osgDB::readImageFile(SKY_FILENAME(checkered_top));
    osg::Image* imageNegY = osgDB::readImageFile(SKY_FILENAME(checkered_top));
    osg::Image* imagePosZ = osgDB::readImageFile(SKY_FILENAME(checkered_left));
    osg::Image* imageNegZ = osgDB::readImageFile(SKY_FILENAME(checkered_right));

    if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ)
    {
        cubemap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
        cubemap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
        cubemap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
        cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
        cubemap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
        cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);

        cubemap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        cubemap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        cubemap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);

        cubemap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        cubemap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    }

    return cubemap;
}

osg::Geometry* CRobotSim::createWall(const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3,osg::StateSet* stateset)
{

   // create a drawable for occluder.
    osg::Geometry* geom = new osg::Geometry;
    
    geom->setStateSet(stateset);

    unsigned int noXSteps = 100;
    unsigned int noYSteps = 100;
    
    osg::Vec3Array* coords = new osg::Vec3Array;
    coords->reserve(noXSteps*noYSteps);
    
    
    osg::Vec3 dx = (v2-v1)/((float)noXSteps-1.0f);
    osg::Vec3 dy = (v3-v1)/((float)noYSteps-1.0f);
    
    unsigned int row;
    osg::Vec3 vRowStart = v1;
    for(row=0;row<noYSteps;++row)
    {
        osg::Vec3 v = vRowStart;
        for(unsigned int col=0;col<noXSteps;++col)        
        {
            coords->push_back(v);
            v += dx;
        }
        vRowStart+=dy;
    }
    
    geom->setVertexArray(coords);
    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    
    for(row=0;row<noYSteps-1;++row)
    {
        osg::DrawElementsUShort* quadstrip = new osg::DrawElementsUShort(osg::PrimitiveSet::QUAD_STRIP);
        quadstrip->reserve(noXSteps*2);
        for(unsigned int col=0;col<noXSteps;++col)        
        {
            quadstrip->push_back((row+1)*noXSteps+col);
            quadstrip->push_back(row*noXSteps+col);
        }   
        geom->addPrimitiveSet(quadstrip);
    }
    
    // create the normals.    
    osgUtil::SmoothingVisitor::smooth(*geom);
    
    return geom;
 
}


osg::Node* CRobotSim::createRoom(void)
{
    // default scale for this model.
    osg::BoundingSphere bs(osg::Vec3(0.0f,0.0f,0.0f),1.0f);

    osg::Group* root = new osg::Group;

    /*if (loadedModel)
    {
        const osg::BoundingSphere& loaded_bs = loadedModel->getBound();

        osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform();
        pat->setPivotPoint(loaded_bs.center());
        
        pat->setUpdateCallback(new ModelTransformCallback(loaded_bs));
        pat->addChild(loadedModel);
        
        bs = pat->getBound();
        
        root->addChild(pat);

    }*/

    bs.radius()*=1.5f;

    // create a bounding box, which we'll use to size the room.
    osg::BoundingBox bb;
    bb.expandBy(bs);


    // create statesets.
    osg::StateSet* rootStateSet = new osg::StateSet;
    root->setStateSet(rootStateSet);

    osg::StateSet* wall = new osg::StateSet;
    wall->setMode(GL_CULL_FACE,osg::StateAttribute::ON);
    
    osg::StateSet* floor = new osg::StateSet;
    floor->setMode(GL_CULL_FACE,osg::StateAttribute::ON);

    osg::StateSet* roof = new osg::StateSet;
    roof->setMode(GL_CULL_FACE,osg::StateAttribute::ON);

    osg::Geode* geode = new osg::Geode;
    
    // create front side.
    geode->addDrawable(createWall(bb.corner(0),
                                  bb.corner(4),
                                  bb.corner(1),
                                  wall));

    // right side
    geode->addDrawable(createWall(bb.corner(1),
                                  bb.corner(5),
                                  bb.corner(3),
                                  wall));

    // left side
    geode->addDrawable(createWall(bb.corner(2),
                                  bb.corner(6),
                                  bb.corner(0),
                                  wall));
    // back side
    geode->addDrawable(createWall(bb.corner(3),
                                  bb.corner(7),
                                  bb.corner(2),
                                  wall));

    // floor
    geode->addDrawable(createWall(bb.corner(0),
                                  bb.corner(1),
                                  bb.corner(2),
                                  floor));

    // roof
    geode->addDrawable(createWall(bb.corner(6),
                                  bb.corner(7),
                                  bb.corner(4),
                                  roof));

    root->addChild(geode);
    
    //root->addChild(createLights(bb,rootStateSet));

    return root;
    
}    

int CRobotSim::graphics_init(void) {
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
	viewer->setCameraManipulator(new osgGA::TerrainManipulator);
	//viewer->setCameraManipulator(new osgGA::SphericalManipulator);
	//viewer->setCameraManipulator(new osgGA::FirstPersonManipulator);
	//viewer->setCameraManipulator(new osgGA::TrackballManipulator);
	viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.7, 0, 0.2), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));

    // Creating the root node
	_osgRoot = new osg::Group();
	_osgRoot->setUpdateCallback(new rootNodeCallback(this, _robot, _osgRoot));

	// load terrain node
    //osg::StateSet* floor = new osg::StateSet;
    //floor->setMode(GL_CULL_FACE,osg::StateAttribute::ON);
	osg::ref_ptr<osg::MatrixTransform> terrainScaleMAT (new osg::MatrixTransform);
	osg::Matrix terrainScaleMatrix;
	terrainScaleMatrix.makeScale(0.1f,0.1f,0.006f);
	osg::ref_ptr<osg::Node> terrainnode = osgDB::readNodeFile("data/ground/terrain.3ds");
	//osg::ref_ptr<osg::Geode> terrainnode;
    //terrainnode->addDrawable(createWall(osg::Vec3d(-100, 100, 0), osg::Vec3d(100, 100, 0), osg::Vec3d(100, -100, 0), floor));
	terrainScaleMAT->addChild(terrainnode.get());
	terrainScaleMAT->setMatrix(terrainScaleMatrix);
	_osgRoot->addChild(terrainScaleMAT.get());

	// load heightfield
    /*osg::Image* heightMap = osgDB::readImageFile("data/ground/flat");
    osg::HeightField* heightField = new osg::HeightField();
    heightField->allocate(heightMap->s(), heightMap->t());
    heightField->setOrigin(osg::Vec3(-heightMap->s() / 2, -heightMap->t() / 2, 0));
    heightField->setXInterval(1.0f);
    heightField->setYInterval(1.0f);
    heightField->setSkirtHeight(1.0f);
    for (int r = 0; r < heightField->getNumRows(); r++) {
        for (int c = 0; c < heightField->getNumColumns(); c++) {
            heightField->setHeight(c, r, ((*heightMap->data(c, r)) / 255.0f) * 80.0f);
        }
    }
    osg::Geode* geode2 = new osg::Geode();
    geode2->addDrawable(new osg::ShapeDrawable(heightField));
	_osgRoot->addChild(geode2);*/
    //osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile(texFile));
    //tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    //tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    //tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    //tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    //geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);*/



    /*// Define the Ephemeris Model and its radius
    osg::ref_ptr<osgEphemeris::EphemerisModel> ephemerisModel = new osgEphemeris::EphemerisModel;
	osg::BoundingSphere bs = terrainScaleMAT->getBound();
    ephemerisModel->setSkyDomeRadius(bs.radius()*2);
    ephemerisModel->setSkyDomeCenter(osg::Vec3d(0, 0, 0));
    // Optionally, Set the AutoDate and Time to false so we can control it with the GUI
    ephemerisModel->setAutoDateTime(false);
    // Optionally, uncomment this if you want to move the Skydome, Moon, Planets and StarField with the mouse
    ephemerisModel->setMoveWithEyePoint(false);
	// add sky model to root
    _osgRoot->addChild(ephemerisModel.get());*/

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
osg::TextureCubeMap* skymap = readCubeMap();
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

    //osg::Node* roomNode = createRoom();
	//_osgRoot->addChild(roomNode);

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
#endif /* ENABLE_GRAPHICS */

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

void CRobotSim::setGroundCapsule(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CRobotSim::setGroundCylinder(dReal r, dReal l, dReal px, dReal py, dReal pz, dReal r_x, dReal r_y, dReal r_z) {
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

void CRobotSim::setGroundSphere(dReal r, dReal px, dReal py, dReal pz) {
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

/*void CRobotSim::simAddRobot(dWorldID &world, dSpaceID &space, dReal **clock) {
	world = _world;
    space = _space;
	*clock = &_clock;
}*/

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
		pthread_mutex_lock(&(sim->_robot_mutex));

		// get start time of execution
		clock_gettime(CLOCK_REALTIME, &start_time);

		// perform pre-collision updates
		//  - lock angle and goal
		//  - update angles 
		for (i = 0; i < NUM_TYPES; i++) {
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				pthread_create(&(sim->_robotThread[i][j]), NULL, (void* (*)(void *))&CRobot::simPreCollisionThreadEntry, (void *)(sim->_robot[i][j]));
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
			for (j = 0; j < sim->_robotConnected[i]; j++) {
				pthread_create(&(sim->_robotThread[i][j]), NULL, (void* (*)(void *))&CRobot::simPostCollisionThreadEntry, (void *)(sim->_robot[i][j]));
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
		for (int i = 0; i < _robotConnected[MOBOT]; i++) {
			cout << RAD2DEG(_robot[MOBOT][i]->getAngle(MOBOT_JOINT2)) << " ";
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
	Add Robot Functions
 **********************************************************/
int CRobotSim::addRobot(CRobot &robot) {
	// get type of robot being added
	int type = robot.getType();

	// find next robot in list
	bot_t btmp = bot;
	while (btmp && btmp->type != type && btmp->id != _robotConnected[type])
		btmp = btmp->next;

	// lock robot data to insert a new one into simulation
	pthread_mutex_lock(&_robot_mutex);

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
	pthread_mutex_unlock(&_robot_mutex);

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
