#include <stdlib.h>
#include "nxt.h"

CNXTGroup::CNXTGroup(void) {
	_thread = (THREAD_T *)malloc(sizeof(THREAD_T));
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

int CNXTGroup::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointAngleNB(id, a, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointCycloidalNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointHarmonicNB(id, angle, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointSmoothNB(id, a0, af, vmax, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointTimeNB(robotJointId_t id, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointTimeNB(id, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToMaxSpeedNB(id, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->accelJointToVelocityNB(id, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
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

int CNXTGroup::blinkLED(double delay, int num) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->blinkLED(delay, num);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::connect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->connect();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelCycloidalNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelCycloidalNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelDistanceNB(double radius, double a, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelDistanceNB(radius, a, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelHarmonicNB(double radius, double d, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelHarmonicNB(radius, d, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelSmoothNB(radius, a0, af, vmax, d);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelTimeNB(double radius, double a, double t) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelTimeNB(radius, a, t);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelToMaxSpeedNB(double radius, double a) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToMaxSpeedNB(radius, a);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveAccelToVelocityNB(double radius, double a, double v) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveAccelToVelocityNB(radius, a, v);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveBackward(double angle) {
	driveBackwardNB(angle);
	return moveWait();
}

int CNXTGroup::driveBackwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveBackwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveDistance(double distance, double radius) {
	driveDistanceNB(distance, radius);
	return moveWait();
}

int CNXTGroup::driveDistanceNB(double distance, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveDistanceNB(distance, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveForward(double angle) {
	driveForwardNB(angle);
	return moveWait();
}

int CNXTGroup::driveForwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveTime(double seconds) {
	int msecs = seconds * 1000.0;

	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForeverNB();
		rtmp = rtmp->next;
	}

#ifdef _WIN32
	Sleep(msecs);
#else
	usleep(msecs*1000);
#endif

	rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::driveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveTimeNB(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::holdJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::holdJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::holdJointsAtExit(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJointsAtExit();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::isMoving(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		if (rtmp->robot->isMoving()) {
			return 1;
		}
		rtmp = rtmp->next;
	}

	// not moving
	return 0;
}

int CNXTGroup::isNotMoving(void) {
	return !(this->isMoving());
}

int CNXTGroup::jumpJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveJointWait(id);
}

int CNXTGroup::jumpJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->jumpJointToNB(id, angle);
		rtmp = rtmp->next;
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

int CNXTGroup::moveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveJoint(robotJointId_t id, double angle) {
	moveJointNB(id, angle);
	return moveWait();
}

int CNXTGroup::moveJointNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveJointForeverNB(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveJointTime(robotJointId_t id, double seconds) {
	this->moveJointTimeNB(id, seconds);

#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// success
	return 0;
}

int CNXTGroup::moveJointTimeNB(robotJointId_t id, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveWait();
}

int CNXTGroup::moveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveJointWait(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointWait(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveTime(double seconds) {
	int msecs = seconds * 1000.0;

	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForeverNB();
		rtmp = rtmp->next;
	}

#ifdef _WIN32
	Sleep(msecs);
#else
	usleep(msecs*1000);
#endif

	rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveTimeNB(seconds);
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

int CNXTGroup::moveToZero(void) {
	moveToZeroNB();
	return moveWait();
}

int CNXTGroup::moveToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::moveWait(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveWait();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::relaxJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::relaxJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::resetToZero(void) {
	resetToZeroNB();
	return moveWait();
}

int CNXTGroup::resetToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->resetToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setBuzzerFrequency(int frequency, double time) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequency(frequency, time);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setBuzzerFrequencyOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setBuzzerFrequencyOn(int frequency) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOn(frequency);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setLEDColor(char *color) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColor(color);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setLEDColorRGB(int r, int g, int b) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColorRGB(r, g, b);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointPower(robotJointId_t id, int power) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointPower(id, power);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSafetyAngle(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngle(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSafetyAngleTimeout(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngleTimeout(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::setJointSpeed(robotJointId_t id, double speed) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeed(id, speed);
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

int CNXTGroup::setJointSpeedRatio(robotJointId_t id, double ratio) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatio(id, ratio);
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

int CNXTGroup::setSpeed(double speed, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setSpeed(speed, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::traceOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::traceOn(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOn();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	return moveWait();
}

int CNXTGroup::turnLeftNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnLeftNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CNXTGroup::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	return moveWait();
}

int CNXTGroup::turnRightNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnRightNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

