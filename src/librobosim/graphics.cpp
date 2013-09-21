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
	switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch (ea.getKey()) {
				case osgGA::GUIEventAdapter::KEY_Space:
				case 'p':
					*_pause = (*_pause) ? 0 : 1;
					if (*_pause)
						_text->setText("PAUSED\npress space to restart");
					else
						_text->setText("");
					return false;
				case 'q':
					return false;
				default:
					return false;
			} 
		default:
			return false;
	}
}

/**********************************************************
	Mobot Node Callback
 **********************************************************/
mobotNodeCallback::mobotNodeCallback(CRobot *robot) {
	_robot = robot;
}

void mobotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int k = 0;
		osg::PositionAttitudeTransform *pat;
		for (int i = 0; i < 5; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i));
			quat = dBodyGetQuaternion(_robot->getBodyID(i));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		while (_robot->getConnectorBodyIDs(k)) {
			pos = dBodyGetPosition(_robot->getConnectorBodyIDs(k));
			quat = dBodyGetQuaternion(_robot->getConnectorBodyIDs(k));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(5 + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
	}
	traverse(node, nv);
}

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
linkbotNodeCallback::linkbotNodeCallback(CRobot *robot) {
	_robot = robot;
}

void linkbotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (int i = 0; i < 4; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i));
			quat = dBodyGetQuaternion(_robot->getBodyID(i));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw 'led'
		pos = dBodyGetPosition(_robot->getBodyID(0));
		quat = dBodyGetQuaternion(_robot->getBodyID(0));
		pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(4));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]+0.00001));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		// draw connectors
		while (_robot->getConnectorBodyIDs(k)) {
			pos = dBodyGetPosition(_robot->getConnectorBodyIDs(k));
			quat = dBodyGetQuaternion(_robot->getConnectorBodyIDs(k));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(5 + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
	}
	traverse(node, nv);
}
