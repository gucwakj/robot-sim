#include <stdlib.h>
#include "linkbotsim.h"

CLinkbotTGroup::CLinkbotTGroup(void) {
	_motion = 0;
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

int CLinkbotTGroup::connect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->connect();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveJointTo(robotJointId_t id, double angle) {
	driveJointToDirectNB(id, angle);
	return moveWait();
}

int CLinkbotTGroup::driveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveJointToDirect(robotJointId_t id, double angle) {
	driveJointToDirectNB(id, angle);
	return moveWait();
}

int CLinkbotTGroup::driveJointToDirectNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveJointToDirectNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveTo(double angle1, double angle2, double angle3) {
	driveToDirectNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::driveToNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveToNB(angle1, angle2, angle3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::driveToDirect(double angle1, double angle2, double angle3) {
	driveToDirectNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::driveToDirectNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveToDirectNB(angle1, angle2, angle3);
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

int CLinkbotTGroup::motionDistance(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	this->motionDistanceThread(this);

	// success
	return 0;
}

int CLinkbotTGroup::motionDistanceNB(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	THREAD_CREATE(_thread, motionDistanceThread, (void *)this);

	// success
	return 0;
}

int CLinkbotTGroup::motionRollBackward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollBackwardThread(this);

	// success
	return 0;
}

int CLinkbotTGroup::motionRollBackwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollBackwardThread, (void *)this);

	// success
	return 0;
}

int CLinkbotTGroup::motionRollForward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollForwardThread(this);

	// success
	return 0;
}

int CLinkbotTGroup::motionRollForwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollForwardThread, (void *)this);

	// success
	return 0;
}

int CLinkbotTGroup::motionTurnLeft(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnLeftThread(this);

	// success
	return 0;
}

int CLinkbotTGroup::motionTurnLeftNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnLeftThread, (void *)this);

	// success
	return 0;
}

int CLinkbotTGroup::motionTurnRight(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnRightThread(this);

	// success
	return 0;
}

int CLinkbotTGroup::motionTurnRightNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnRightThread, (void *)this);

	// success
	return 0;
}

int CLinkbotTGroup::motionWait(void) {
	while (_motion > 0) {
#ifdef _WIN32
		Sleep(200);
#else
		usleep(200000);
#endif
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

int CLinkbotTGroup::moveBackward(double angle) {
	moveBackwardNB(angle);
	return moveWait();
}

int CLinkbotTGroup::moveBackwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveBackwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveContinuousNB(dir1, dir2, dir3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveContinuousTime(dir1, dir2, dir3, seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveDistance(double distance, double radius) {
	moveDistanceNB(distance, radius);
	return moveWait();
}

int CLinkbotTGroup::moveDistanceNB(double distance, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveDistanceNB(distance, radius);
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

int CLinkbotTGroup::moveForward(double angle) {
	moveForwardNB(angle);
	return moveWait();
}

int CLinkbotTGroup::moveForwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForwardNB(angle);
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

int CLinkbotTGroup::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointContinuousNB(id, dir);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointContinuousTime(id, dir, seconds);
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

int CLinkbotTGroup::moveJointToDirect(robotJointId_t id, double angle) {
	moveJointToDirectNB(id, angle);
	return moveWait();
}

int CLinkbotTGroup::moveJointToDirectNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToDirectNB(id, angle);
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

int CLinkbotTGroup::moveToDirect(double angle1, double angle2, double angle3) {
	moveToDirectNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveToDirectNB(double angle1, double angle2, double angle3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToDirectNB(angle1, angle2, angle3);
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

int CLinkbotTGroup::reset(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->reset();
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

int CLinkbotTGroup::setMotorPower(robotJointId_t id, int power) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setMotorPower(id, power);
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

int CLinkbotTGroup::stop(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stop();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::stopOneJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopOneJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::stopTwoJoints(robotJointId_t id1, robotJointId_t id2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopTwoJoints(id1, id2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CLinkbotTGroup::stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopThreeJoints(id1, id2, id3);
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

void* CLinkbotTGroup::motionDistanceThread(void* arg) {
	CLinkbotTGroup *clg = (CLinkbotTGroup *)arg;

	// move
	clg->move(RAD2DEG(clg->_d), 0, RAD2DEG(clg->_d));

	// motion is complete
	clg->_motion--;

	// success
	return NULL;
}

void* CLinkbotTGroup::motionRollBackwardThread(void* arg) {
	CLinkbotTGroup *clg = (CLinkbotTGroup *)arg;

	// move
	clg->move(-clg->_d, 0, -clg->_d);

	// motion is complete
	clg->_motion--;

	// success
	return NULL;
}

void* CLinkbotTGroup::motionRollForwardThread(void* arg) {
	CLinkbotTGroup *clg = (CLinkbotTGroup *)arg;

	// move
	clg->move(clg->_d, 0, clg->_d);

	// motion is complete
	clg->_motion--;

	// success
	return NULL;
}

void* CLinkbotTGroup::motionTurnLeftThread(void* arg) {
	CLinkbotTGroup *clg = (CLinkbotTGroup *)arg;

	// move
	clg->move(-clg->_d, 0, clg->_d);

	// motion is complete
	clg->_motion--;

	// success
	return NULL;
}

void* CLinkbotTGroup::motionTurnRightThread(void* arg) {
	CLinkbotTGroup *clg = (CLinkbotTGroup *)arg;

	// move
	clg->move(clg->_d, 0, -clg->_d);

	// motion is complete
	clg->_motion--;

	// success
	return NULL;
}

#ifdef _CH_
CLinkbotIGroup::~CLinkbotIGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}

	// kill thread
	THREAD_CANCEL(*_thread);

	// success
	return 0;
}

CLinkbotLGroup::~CLinkbotLGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}

	// kill thread
	THREAD_CANCEL(*_thread);

	// success
	return 0;
}
#endif
