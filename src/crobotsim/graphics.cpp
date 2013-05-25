#include "graphics.h"
#include "robotsim.h"

rootNodeCallback::rootNodeCallback(CRobotSim *sim, CRobot ***robot, osg::Group *root) {
	_sim = sim;
	_robot = robot;
	_root = root;
	for ( int i = 0; i < NUM_TYPES; i++) { _number[i] = 0; }
}

void rootNodeCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		for ( int i = 0; i < NUM_TYPES; i++) {
			if (_number[i] != _sim->getNumberOfRobots(i)) {
				_robot[i][_number[i]++]->draw(_root);
			}
		}
	}
	traverse(node, nv);
}

robot4NodeCallback::robot4NodeCallback(CRobot *robot) {
	_robot = robot;
}

void robot4NodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const dReal *pos, *quat;
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

linkbotNodeCallback::linkbotNodeCallback(CRobot *robot) {
	_robot = robot;
}

void linkbotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const dReal *pos, *quat;
		int k = 0;
		osg::PositionAttitudeTransform *pat;
		for (int i = 0; i < 4; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i));
			quat = dBodyGetQuaternion(_robot->getBodyID(i));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		while (_robot->getConnectorBodyIDs(k)) {
			pos = dBodyGetPosition(_robot->getConnectorBodyIDs(k));
			quat = dBodyGetQuaternion(_robot->getConnectorBodyIDs(k));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(4 + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
	}
	traverse(node, nv);
}
