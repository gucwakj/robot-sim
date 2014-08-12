#include <stdlib.h>
#include "cubus.h"

CubusGroup::CubusGroup(void) {
	_robots = NULL;
}

CubusGroup::~CubusGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}
}

int CubusGroup::addRobot(Cubus &robot) {
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

int CubusGroup::addRobots(Cubus robots[], int num) {
	for (int i = 0; i < num; i++) {
		this->addRobot(robots[i]);
	}

	// success
	return 0;
}

int CubusGroup::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointAngleNB(id, a, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointCycloidalNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointHarmonicNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointSmoothNB(id, a0, af, vmax, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointTimeNB(robotJointId_t id, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointTimeNB(id, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToMaxSpeedNB(id, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToVelocityNB(id, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelCycloidalNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelCycloidalNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelDistanceNB(double radius, double a, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelDistanceNB(radius, a, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelHarmonicNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelHarmonicNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelSmoothNB(radius, a0, af, vmax, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelTimeNB(double radius, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelTimeNB(radius, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelToMaxSpeedNB(double radius, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToMaxSpeedNB(radius, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::driveAccelToVelocityNB(double radius, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToVelocityNB(radius, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::jumpTo(double angle1, double angle2, double angle3) {
	moveToNB(angle1, angle2, angle3);
	return moveWait();
}

int CubusGroup::jumpToNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->jumpToNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::move(double angle1, double angle2, double angle3) {
	moveNB(angle1, angle2, angle3);
	return moveWait();
}

int CubusGroup::moveNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::moveTo(double angle1, double angle2, double angle3) {
	moveToNB(angle1, angle2, angle3);
	return moveWait();
}

int CubusGroup::moveToNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::setJointSpeeds(double speed1, double speed2, double speed3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeeds(speed1, speed2, speed3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CubusGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatios(ratio1, ratio2, ratio3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

