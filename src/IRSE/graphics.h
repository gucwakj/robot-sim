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

#endif  /* GRAPHICS_H_ */
