#include "graphics.h"
#include "robosim.h"
#include "linkbot.h"
#include "mobot.h"
#include "nxt.h"

osg::Node::NodeMask NOT_VISIBLE_MASK = 0x0;
osg::Node::NodeMask RECEIVES_SHADOW_MASK = 0x1;
osg::Node::NodeMask CASTS_SHADOW_MASK = 0x2;
osg::Node::NodeMask IS_PICKABLE_MASK = 0x3;
osg::Node::NodeMask VISIBLE_MASK = 0xffffffff;

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
keyboardEventHandler::keyboardEventHandler(osgText::Text *text) {
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
				case '1': {
					osg::Vec3f eye = osg::Vec3f(0.7, -0.7, 0.55);
					osg::Vec3f center = osg::Vec3f(0.1, 0.3, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case '2': {
					osg::Vec3f eye = osg::Vec3f(0, 0, 5);
					osg::Vec3f center = osg::Vec3f(0, 0, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case 'n': {
					osg::Billboard *billboard = dynamic_cast<osg::Billboard *>(root->getChild(3));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					billboard = dynamic_cast<osg::Billboard *>(root->getChild(4));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					return true;
				}
				case 'r': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) { geode->setNodeMask(VISIBLE_MASK); }
							for (int j = 2; j < (int)(shadow->getChild(i)->asGroup()->getNumChildren()); j++) {
								osg::PositionAttitudeTransform *pat;
								pat = dynamic_cast<osg::PositionAttitudeTransform *>(shadow->getChild(i)->asGroup()->getChild(j));
								pat->setNodeMask((pat->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
								pat->getNodeMask() ? g_sim->setCollisions(1) : g_sim->setCollisions(0);
							}
						}
					}
					return true;
				}
				case 't': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) {
								geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
							}
						}
					}
					return true;
				}
				default:
					g_sim->setPause(2);
					if (g_sim->getPause())
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
				pick(ea, viewer);
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
	double x = ea.getXnormalized(), y = ea.getYnormalized();
	picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, x, y);
	picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE);
	osgUtil::IntersectionVisitor iv(picker);
	iv.setTraversalMask(IS_PICKABLE_MASK);
	viewer->getCamera()->accept(iv);
	//scene->accept(iv);

	if (picker->containsIntersections()) {
		/*std::multiset<osgUtil::LineSegmentIntersector::Intersection> ins = picker->getIntersections();
		std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator it = ins.begin();
		osg::NodePath nodePath;
		for(int j = 0; j < ins.size(); j++) {
			nodePath = (*it).nodePath;
			it++;
			printf("%d nodePath:\t", j);
			for(int i = 0; i < nodePath.size(); i++) {
				printf("%s ", nodePath[i]->className());
			}
			printf("\n");
		}*/

		// get node at first intersection
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
		osg::NodePath &nodePath = intersection.nodePath;

		// get robot node
		grandparent = (nodePath.size()>=3) ? dynamic_cast<osg::Group *>(nodePath[nodePath.size()-3]) : 0;

		// toggle HUD
		if (grandparent && (grandparent->getName() == "robot")) {
			osg::Geode *geode = dynamic_cast<osg::Geode *>(grandparent->getChild(0));
			geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
		}
	}
}

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
linkbotNodeCallback::linkbotNodeCallback(CLinkbotT *robot) {
	_robot = robot;
	_count = 1;
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
		conn_t ctmp = _robot->_conn;
		while(ctmp) {
			dMatrix3 R;
			dQuaternion Q;
			double p[3] = {0};
			_robot->getFaceParams(ctmp->face, R, p);
			if (ctmp->d_side != -1) _robot->getConnectorParams(ctmp->d_type, ctmp->d_side, R, p);
			dRtoQ(R, Q);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));
			ctmp = ctmp->next;
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			colors->push_back(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0f) );
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
	}
	traverse(node, nv);
}

/**********************************************************
	Mobot Node Callback
 **********************************************************/
mobotNodeCallback::mobotNodeCallback(CMobot *robot) {
	_robot = robot;
	_count = 1;
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
		conn_t ctmp = _robot->_conn;
		while (ctmp) {
			pos = dBodyGetPosition(ctmp->body);
			quat = dBodyGetQuaternion(ctmp->body);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
	}
	traverse(node, nv);
}

/**********************************************************
	NXT Node Callback
 **********************************************************/
nxtNodeCallback::nxtNodeCallback(CNXT *robot) {
	_robot = robot;
	_count = 1;
}

void nxtNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+3; i++) {
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
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
	}
	traverse(node, nv);
}

/**********************************************************
	Ground Node Callback
 **********************************************************/
groundNodeCallback::groundNodeCallback(struct ground_s *ground) {
	_ground = ground;
}

void groundNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos = dBodyGetPosition(_ground->body);
		const double *quat = dBodyGetQuaternion(_ground->body);
		osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	}
	traverse(node, nv);
}

