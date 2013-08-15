#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "config.h"
#include "base.h"
#include <OpenThreads/Thread>

#include <osg/ClearNode>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Notify>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/TexEnv>
#include <osg/TexEnvCombine>
#include <osg/TexGen>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osg/Transform>
#include <osg/TextureCubeMap>
#include <osg/VertexProgram>

#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include <osgGA/StateSetManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/FirstPersonManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/SphericalManipulator>

#include <osgUtil/Optimizer>
#include <osgUtil/CullVisitor>
#include <osgUtil/SmoothingVisitor>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <ode/ode.h>

class CRobot;
class CRobotSim;

class MoveEarthySkyWithEyePointTransform : public osg::Transform {
	public:
		/** Get the transformation matrix which moves from local coords to world coords.*/
		virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const {
			osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
			if (cv) {
				osg::Vec3 eyePointLocal = cv->getEyeLocal();
				matrix.preMultTranslate(eyePointLocal);
			}
			return true;
		}
		/** Get the transformation matrix which moves from world coords to local coords.*/
		virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const {
			osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
			if (cv) {
	    		osg::Vec3 eyePointLocal = cv->getEyeLocal();
				matrix.postMultTranslate(-eyePointLocal);
			}
			return true;
		}
};

// Update texture matrix for cubemaps
class TexMatCallback : public osg::NodeCallback {
	public:
		TexMatCallback(osg::TexMat& tm) : _texMat(tm) {}
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		osg::TexMat& _texMat;
};

/**********************************************************
	Keyboard Event Handler
 **********************************************************/
class keyboardEventHandler : public osgGA::GUIEventHandler {
	public:
		keyboardEventHandler(int *pause, osgText::Text *text);
		virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
		virtual void accept(osgGA::GUIEventHandlerVisitor &v);
	private:
		int *_pause;
		osgText::Text *_text;
};

/**********************************************************
	Mouse Event Handler
 **********************************************************/
class mouseEventHandler : public osgGA::GUIEventHandler {
	public:
		mouseEventHandler(osgGA::SphericalManipulator *camera);
		virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
		virtual void accept(osgGA::GUIEventHandlerVisitor &v);
	private:
		osgGA::SphericalManipulator *_camera;
};

/**********************************************************
	Root Node Callback
 **********************************************************/
/*class rootNodeCallback : public osg::NodeCallback {
	public:
		rootNodeCallback(CRobotSim *sim, CRobot ***robot, osg::Group *root);
		virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);
	private:
		CRobotSim *_sim;
		CRobot ***_robot;
		osg::Group *_root;
		int _number[NUM_TYPES];
};*/

/**********************************************************
	Mobot Node Callback
 **********************************************************/
class mobotNodeCallback : public osg::NodeCallback {
	public:
		mobotNodeCallback(CRobot *robot);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		CRobot *_robot;
};

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
class linkbotNodeCallback : public osg::NodeCallback {
	public:
		linkbotNodeCallback(CRobot *robot);
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	private:
		CRobot *_robot;
};

#endif /* GRAPHICS_H_ */
