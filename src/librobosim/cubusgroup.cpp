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

