#include "graphics.h"
#include "robosim.h"

/**********************************************************
	MoveEarthySkyWithEyePointTransform
 **********************************************************/
bool MoveEarthySkyWithEyePointTransform::computeLocalToWorldMatrix(osg::Matrix &matrix,osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.preMultTranslate(eyePointLocal);
	}
	return true;
}

bool MoveEarthySkyWithEyePointTransform::computeWorldToLocalMatrix(osg::Matrix &matrix,osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
  		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.postMultTranslate(-eyePointLocal);
	}
	return true;
}

/**********************************************************
	TexMatCallback
 **********************************************************/
void TexMatCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
		const osg::Matrix& MV = *(cv->getModelViewMatrix());
		const osg::Matrix R = osg::Matrix::rotate( osg::DegreesToRadians(112.0f), 0.0f,0.0f,1.0f)*
								osg::Matrix::rotate( osg::DegreesToRadians(90.0f), 1.0f,0.0f,0.0f);

		osg::Quat q = MV.getRotate();
		const osg::Matrix C = osg::Matrix::rotate( q.inverse() );

		_texMat.setMatrix( C*R );
	}
	traverse(node, nv);
}

/**********************************************************
	Keyboard Event Handler
 **********************************************************/
keyboardEventHandler::keyboardEventHandler(int *pause, osgText::Text *text) {
	_pause = pause;
	_text = text;
}

void keyboardEventHandler::accept(osgGA::GUIEventHandlerVisitor &v) {
	v.visit(*this);
};

bool keyboardEventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	osg::Group *root = dynamic_cast<osg::Group *>(viewer->getSceneData());
	osgShadow::ShadowedScene *shadow = dynamic_cast<osgShadow::ShadowedScene*>(root->getChild(0));

	switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch (ea.getKey()) {
				case 't':
					for (int i = 0; i < shadow->getNumChildren(); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							geode->setNodeMask((geode->getNodeMask() ? 0x0 : 0xffffffff));
						}
					}
					return true;
				default:
					*_pause = (*_pause) ? 0 : 1;
					if (*_pause)
						_text->setText("Paused: Press any key to restart");
					else
						_text->setText("");
					return true;
			} 
		default:
			return false;
	}
}

/**********************************************************
	Picking Event Handler
 **********************************************************/
pickHandler::pickHandler(void) {
	_mx = 0.0;
	_my = 0.0;
}

bool pickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	if (!viewer) return false;

	switch (ea.getEventType()) {
		case(osgGA::GUIEventAdapter::PUSH):
		case(osgGA::GUIEventAdapter::MOVE):
			_mx = ea.getX();
			_my = ea.getY();
			return false;
		case(osgGA::GUIEventAdapter::RELEASE):
			if (_mx == ea.getX() && _my == ea.getY())
				pick(ea,viewer);
			return true;
		default:
			return false;
	}
}

void pickHandler::pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer) {
	osg::Node *scene = viewer->getSceneData();
	if (!scene) return;

	osg::Group *grandparent = 0;
	osgUtil::LineSegmentIntersector *picker;
	picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, ea.getXnormalized(), ea.getYnormalized());
	osgUtil::IntersectionVisitor iv(picker);
	viewer->getCamera()->accept(iv);

	if (picker->containsIntersections()) {
		// get node at intersection
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
		osg::NodePath &nodePath = intersection.nodePath;
		grandparent = (nodePath.size()>=3) ? dynamic_cast<osg::Group *>(nodePath[nodePath.size()-3]) : 0;

		// toggle HUD
		if (grandparent->getName() == "robot") {
			osg::Geode *geode = dynamic_cast<osg::Geode *>(grandparent->getChild(0));
			geode->setNodeMask((geode->getNodeMask() ? 0x0 : 0xffffffff));
		}
	}
}

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
linkbotNodeCallback::linkbotNodeCallback(CRobot *robot, int units) {
	_robot = robot;
	_units = units;
}

void linkbotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+4; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw 'led'
		pos = dBodyGetPosition(_robot->getBodyID(0));
		quat = dBodyGetQuaternion(_robot->getBodyID(0));
		pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i++));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]+0.00001));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		// draw connectors
		while (_robot->getConnectorBodyIDs(k)) {
			pos = dBodyGetPosition(_robot->getConnectorBodyIDs(k));
			quat = dBodyGetQuaternion(_robot->getConnectorBodyIDs(k));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		pos = dBodyGetPosition(_robot->getBodyID(0));
		if (_units)
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getRobotID()+1, pos[0]*39.37, pos[1]*39.37);
		else
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getRobotID()+1, pos[0]*10, pos[1]*10);
		label->setText(text);
		label->setPosition(osg::Vec3(pos[0], pos[1], pos[2] + (_robot->getRobotID()%2 ? 0.1 : 0) + 0.15));
		// draw tracking line
		static int count = 2;
		osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
		osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
		osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
		vertices->push_back(osg::Vec3(pos[0], pos[1], 0));
		vertices->push_back(osg::Vec3(pos[0], pos[1], 0));
		osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
		array->setCount(count+=2);
	}
	traverse(node, nv);
}

/**********************************************************
	Mobot Node Callback
 **********************************************************/
mobotNodeCallback::mobotNodeCallback(CRobot *robot, int units) {
	_robot = robot;
	_units = units;
}

void mobotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+5; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw connectors
		while (_robot->getConnectorBodyIDs(k)) {
			pos = dBodyGetPosition(_robot->getConnectorBodyIDs(k));
			quat = dBodyGetQuaternion(_robot->getConnectorBodyIDs(k));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		pos = dBodyGetPosition(_robot->getBodyID(0));
		if (_units)
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getRobotID()+1, pos[0]*39.37, pos[1]*39.37);
		else
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getRobotID()+1, pos[0]*10, pos[1]*10);
		label->setText(text);
		label->setPosition(osg::Vec3(pos[0], pos[1], pos[2] + (_robot->getRobotID()%2 ? 0.1 : 0) + 0.15));
		// draw tracking line
		static int count = 2;
		osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
		osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
		osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
		vertices->push_back(osg::Vec3(pos[0], pos[1], 0));
		vertices->push_back(osg::Vec3(pos[0], pos[1], 0));
		osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
		array->setCount(count+=2);
	}
	traverse(node, nv);
}

