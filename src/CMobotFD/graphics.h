#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include <OpenThreads/Thread>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgUtil/Optimizer>
//#include "mobotfd.h"
//class CMobotFD;

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

/*class mobotNodeCallback : public osg::NodeCallback {
    public:
        mobotNodeCallback(CMobotFD *sim, int number) : _sim(sim), _number(number) {}
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
            osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *> (node);
            if (pat) {
                osg::Vec3f current = pat->getPosition();
                //pthread_mutex_lock(&(robot[_number].angle_mutex));
				printf("first\n");
				//_sim->robot[MOBOT][_number]->simThreadsAngleLock();
				_sim->robot[MOBOT][_number]->simThreadsAngleLock();
                //osg::Vec3f pos = osg::Vec3f(robot[_number].angle[0]/10.0, robot[_number].angle[1]/10.0, robot[_number].angle[2]/10.0);
                //osg::Quat quat = osg::Quat(robot[_number].angle[0]/10.0+0.1, osg::Vec3f(1,0,0));
                //pthread_mutex_unlock(&(robot[_number].angle_mutex));
				//_sim->robot[MOBOT][_number]->simThreadsAngleUnlock();
				robot[MOBOT][_number]->simThreadsAngleUnlock();
                pat->setPosition(current);
                //pat->setAttitude(quat);
				printf("last\n");
            }
            traverse(node, nv);
        }
    private:
        int _number;
		CMobotFD *_sim;
};*/

#endif  /* GRAPHICS_H_ */
