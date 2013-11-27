#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "config.h"
#include "base.h"

#include <OpenThreads/Thread>

#include <osg/Billboard>
#include <osg/ClearNode>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Notify>
#include <osg/Object>
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

#include <osgGA/OrbitManipulator>
#include <osgGA/StateSetManipulator>

#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>

#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <ode/ode.h>

class CRobot;

/**********************************************************
	MoveEarthySkyWithEyePointTransform
 **********************************************************/
class MoveEarthySkyWithEyePointTransform : public osg::Transform {
	public:
		virtual bool computeLocalToWorldMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const;
		virtual bool computeWorldToLocalMatrix(osg::Matrix &matrix, osg::NodeVisitor *nv) const;
};

/**********************************************************
	TexMatCallback
 **********************************************************/
class TexMatCallback : public osg::NodeCallback {
	public:
		TexMatCallback(osg::TexMat &tm) : _texMat(tm) {}
		virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);
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
	Mouse Pick Event Handler
 **********************************************************/
class pickHandler : public osgGA::GUIEventHandler {
	public:
		pickHandler(void);
		bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
		void pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer);
	private:
		float _mx, _my;
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

#endif /* GRAPHICS_H_ */
