#include <stdlib.h>
#include "linkbot.h"

CLinkbotTGroup::CLinkbotTGroup(void) {
	_robots = NULL;
}

CLinkbotTGroup::~CLinkbotTGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}
}

int CLinkbotTGroup::addRobot(CLinkbotT &robot) {
	// create new robot
	robots_t nr = (robots_t)malloc(sizeof(struct robots_s));
	nr->robot = &robot;
	nr->next = NULL;

	// store new robot
	robots_t rtmp = _robots;
	if ( _robots == NULL )
		_robots = nr;
	else {
		while (rtmp->next)
			rtmp = rtmp->next;
		rtmp->next = nr;
	}

	// success
	return 0;
}

int CLinkbotTGroup::addRobots(CLinkbotT robots[], int num) {
	for (int i = 0; i < num; i++) {
		this->addRobot(robots[i]);
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointAngleNB(id, a, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointCycloidalNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointHarmonicNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointSmoothNB(id, a0, af, vmax, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointTimeNB(robotJointId_t id, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointTimeNB(id, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToMaxSpeedNB(id, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToVelocityNB(id, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::closeGripper(void) {
	robots_t rtmp = _robots;
	while (rtmp->next) {
		rtmp->robot->closeGripperNB();
		rtmp = rtmp->next;
	}
	rtmp->robot->closeGripper();

	// success
	return 0;
}

int CLinkbotTGroup::closeGripperNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->closeGripperNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelCycloidalNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelCycloidalNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelDistanceNB(double radius, double a, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelDistanceNB(radius, a, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelHarmonicNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelHarmonicNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelSmoothNB(radius, a0, af, vmax, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelTimeNB(double radius, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelTimeNB(radius, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelToMaxSpeedNB(double radius, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToMaxSpeedNB(radius, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveAccelToVelocityNB(double radius, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToVelocityNB(radius, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::jumpTo(double angle1, double angle2, double angle3) {
	moveToNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::jumpToNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->jumpToNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::move(double angle1, double angle2, double angle3) {
	moveNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveTo(double angle1, double angle2, double angle3) {
	moveToNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveToNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::openGripper(double angle) {
	openGripperNB(angle);
	return moveWait();
}

int CLinkbotTGroup::openGripperNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->openGripperNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointSpeeds(double speed1, double speed2, double speed3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeeds(speed1, speed2, speed3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

