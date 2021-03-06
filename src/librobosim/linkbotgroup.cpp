#include <stdlib.h>
#include "linkbot.h"

CLinkbotTGroup::CLinkbotTGroup(void) {
	_thread = (THREAD_T *)malloc(sizeof(THREAD_T));
	_robots = NULL;
}

CLinkbotTGroup::~CLinkbotTGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}

	// kill thread
	THREAD_CANCEL(*_thread);
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

int CLinkbotTGroup::blinkLED(double delay, int num) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->blinkLED(delay, num);
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

int CLinkbotTGroup::connect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->connect();
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

int CLinkbotTGroup::driveBackward(double angle) {
	driveBackwardNB(angle);
	return moveWait();
}

int CLinkbotTGroup::driveBackwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveBackwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveDistance(double distance, double radius) {
	driveDistanceNB(distance, radius);
	return moveWait();
}

int CLinkbotTGroup::driveDistanceNB(double distance, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveDistanceNB(distance, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveForward(double angle) {
	driveForwardNB(angle);
	return moveWait();
}

int CLinkbotTGroup::driveForwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveTime(double seconds) {
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

int CLinkbotTGroup::driveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveTimeNB(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::holdJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::holdJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::holdJointsAtExit(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJointsAtExit();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::isMoving(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		if(rtmp->robot->isMoving()) {
			// moving
			return 1;
		}
		rtmp = rtmp->next;
	}

	// not moving
	return 0;
}

int CLinkbotTGroup::isNotMoving(void) {
	return !(this->isMoving());
}

int CLinkbotTGroup::jumpJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveJointWait(id);
}

int CLinkbotTGroup::jumpJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->jumpJointToNB(id, angle);
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

int CLinkbotTGroup::moveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJoint(robotJointId_t id, double angle) {
	moveJointNB(id, angle);
	return moveWait();
}

int CLinkbotTGroup::moveJointNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJointForeverNB(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJointTime(robotJointId_t id, double seconds) {
	this->moveJointTimeNB(id, seconds);

#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// success
	return 0;
}

int CLinkbotTGroup::moveJointTimeNB(robotJointId_t id, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveWait();
}

int CLinkbotTGroup::moveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJointWait(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointWait(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveTime(double seconds) {
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

int CLinkbotTGroup::moveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveTimeNB(seconds);
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

int CLinkbotTGroup::moveToZero(void) {
	moveToZeroNB();
	return moveWait();
}

int CLinkbotTGroup::moveToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveWait(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveWait();
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

int CLinkbotTGroup::relaxJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::relaxJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::resetToZero(void) {
	resetToZeroNB();
	return moveWait();
}

int CLinkbotTGroup::resetToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->resetToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setBuzzerFrequency(int frequency, double time) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequency(frequency, time);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setBuzzerFrequencyOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setBuzzerFrequencyOn(int frequency) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOn(frequency);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setLEDColor(char *color) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColor(color);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setLEDColorRGB(int r, int g, int b) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColorRGB(r, g, b);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointPower(robotJointId_t id, int power) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointPower(id, power);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointSafetyAngle(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngle(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointSafetyAngleTimeout(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngleTimeout(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::setJointSpeed(robotJointId_t id, double speed) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeed(id, speed);
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

int CLinkbotTGroup::setJointSpeedRatio(robotJointId_t id, double ratio) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatio(id, ratio);
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

int CLinkbotTGroup::setSpeed(double speed, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setSpeed(speed, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::traceOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::traceOn(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOn();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	return moveWait();
}

int CLinkbotTGroup::turnLeftNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnLeftNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	return moveWait();
}

int CLinkbotTGroup::turnRightNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnRightNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}
