#include <stdlib.h>
#include "nxt.h"

CNXTGroup::CNXTGroup(void) {
	_robots = NULL;
}

CNXTGroup::~CNXTGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}

	// kill thread
	THREAD_CANCEL(*_thread);
}

int CNXTGroup::addRobot(CNXT &robot) {
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

int CNXTGroup::addRobots(CNXT robots[], int num) {
	for (int i = 0; i < num; i++) {
		this->addRobot(robots[i]);
	}

	// success
	return 0;
}

int CNXTGroup::jumpTo(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::jumpToNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->jumpToNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::move(double angle1, double angle2) {
	moveNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveTo(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveToNB(double angle1, double angle2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToNB(angle1, angle2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSpeeds(double speed1, double speed2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeeds(speed1, speed2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSpeedRatios(double ratio1, double ratio2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatios(ratio1, ratio2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

