#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <OpenThreads/Thread>

#include <osg/ClearNode>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Notify>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/Transform>

#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include <osgEphemeris/EphemerisModel.h>

#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>

#include <osgUtil/Optimizer>
#include <osgUtil/CullVisitor>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <ode/ode.h>
#include "robot.h"

class robotSim;
class IRSE;

class ViewerFrameThread : public OpenThreads::Thread {
    public:
        ViewerFrameThread(osgViewer::ViewerBase *viewerBase, bool doApplicationExit):
            _viewerBase(viewerBase),
            _doApplicationExit(doApplicationExit) {}
        ~ViewerFrameThread() {
            cancel();
            while ( isRunning() ) {
                OpenThreads::Thread::YieldCurrentThread();
            }
        }
        int cancel() {
            _viewerBase->setDone(true);
            return 0;
        }
        void run() {
            int result = _viewerBase->run();
            if (_doApplicationExit) exit(result);
        }
        osg::ref_ptr<osgViewer::ViewerBase> _viewerBase;
        bool _doApplicationExit;
};

class rootNodeCallback : public osg::NodeCallback {
	public:
		rootNodeCallback(IRSE *sim, robotSim ***robot, osg::Group *root);
		virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);
	private:
		IRSE *_sim;
		robotSim ***_robot;
		osg::Group *_root;
		int _number[NUM_TYPES];
};

class robot4NodeCallback : public osg::NodeCallback {
	public:
		robot4NodeCallback(robotSim *robot);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		robotSim *_robot;
};

#endif  /* GRAPHICS_H_ */
