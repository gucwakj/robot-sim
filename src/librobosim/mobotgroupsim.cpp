#include <stdlib.h>
#include "mobotsim.h"

CMobotGroup::CMobotGroup(void) {
	_motion = 0;
	_thread = (THREAD_T *)malloc(sizeof(THREAD_T));
	_robots = NULL;
}

CMobotGroup::~CMobotGroup(void) {
	// remove robots from group
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		free(_robots);
		_robots = tmp;
	}

	// kill thread
	THREAD_CANCEL(*_thread);
}

int CMobotGroup::addRobot(CMobot &robot) {
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

int CMobotGroup::addRobots(CMobot robots[], int num) {
	for (int i = 0; i < num; i++) {
		this->addRobot(robots[i]);
	}

	// success
	return 0;
}

int CMobotGroup::blinkLED(double delay, int num) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->blinkLED(delay, num);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::connect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->connect();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::driveJointTo(robotJointId_t id, double angle) {
	driveJointToDirectNB(id, angle);
	return moveWait();
}

int CMobotGroup::driveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::driveJointToDirect(robotJointId_t id, double angle) {
	driveJointToDirectNB(id, angle);
	return moveWait();
}

int CMobotGroup::driveJointToDirectNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveJointToDirectNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::driveTo(double angle1, double angle2, double angle3, double angle4) {
	driveToDirectNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::driveToNB(double angle1, double angle2, double angle3, double angle4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveToNB(angle1, angle2, angle3, angle4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::driveToDirect(double angle1, double angle2, double angle3, double angle4) {
	driveToDirectNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::driveToDirectNB(double angle1, double angle2, double angle3, double angle4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveToDirectNB(angle1, angle2, angle3, angle4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::holdJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::holdJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::holdJointsAtExit(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJointsAtExit();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::isMoving(void) {
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

int CMobotGroup::isNotMoving(void) {
	return !(this->isMoving());
}

int CMobotGroup::motionArch(double angle) {
	_d = angle;
	_motion++;
	this->motionArchThread(this);

	// success
	return 0;
}

int CMobotGroup::motionArchNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionArchThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionDistance(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	this->motionDistanceThread(this);

	// success
	return 0;
}

int CMobotGroup::motionDistanceNB(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	THREAD_CREATE(_thread, motionDistanceThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormLeft(int num) {
	_i = num;
	_motion++;
	this->motionInchwormLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormLeftNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionInchwormLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormRight(int num) {
	_i = num;
	_motion++;
	this->motionInchwormRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormRightNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionInchwormRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionRollBackward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollBackwardThread(this);

	// success
	return 0;
}

int CMobotGroup::motionRollBackwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollBackwardThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionRollForward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollForwardThread(this);

	// success
	return 0;
}

int CMobotGroup::motionRollForwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollForwardThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionSkinny(double angle) {
	_d = angle;
	_motion++;
	this->motionSkinnyThread(this);

	// success
	return 0;
}

int CMobotGroup::motionSkinnyNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionSkinnyThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionStand(void) {
	_motion++;
	this->motionStandThread(this);

	// success
	return 0;
}

int CMobotGroup::motionStandNB(void) {
	_motion++;
	THREAD_CREATE(_thread, motionStandThread, NULL);

	// success
	return 0;
}

int CMobotGroup::motionTurnLeft(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTurnLeftNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTurnRight(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTurnRightNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleRight(int num) {
	_i = num;
	_motion++;
	this->motionTumbleRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleRightNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionTumbleRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleLeft(int num) {
	_i = num;
	_motion++;
	this->motionTumbleLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleLeftNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionTumbleLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionUnstand(void) {
	_motion++;
	this->motionUnstandThread(this);

	// success
	return 0;
}

int CMobotGroup::motionUnstandNB(void) {
	_motion++;
	THREAD_CREATE(_thread, motionUnstandThread, NULL);

	// success
	return 0;
}

int CMobotGroup::motionWait(void) {
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

int CMobotGroup::move(double angle1, double angle2, double angle3, double angle4) {
	moveNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveNB(double angle1, double angle2, double angle3, double angle4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveNB(angle1, angle2, angle3, angle4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveBackward(double angle) {
	moveBackwardNB(angle);
	return moveWait();
}

int CMobotGroup::moveBackwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveBackwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveDistance(double distance, double radius) {
	moveDistanceNB(distance, radius);
	return moveWait();
}

int CMobotGroup::moveDistanceNB(double distance, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveDistanceNB(distance, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveForward(double angle) {
	moveForwardNB(angle);
	return moveWait();
}

int CMobotGroup::moveForwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveJoint(robotJointId_t id, double angle) {
	moveJointNB(id, angle);
	return moveWait();
}

int CMobotGroup::moveJointNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveJointForeverNB(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveWait();
}

int CMobotGroup::moveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveJointToDirect(robotJointId_t id, double angle) {
	moveJointToDirectNB(id, angle);
	return moveWait();
}

int CMobotGroup::moveJointToDirectNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToDirectNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveJointWait(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointWait(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveTime(double seconds) {
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

int CMobotGroup::moveTo(double angle1, double angle2, double angle3, double angle4) {
	moveToNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToNB(angle1, angle2, angle3, angle4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveToDirect(double angle1, double angle2, double angle3, double angle4) {
	moveToDirectNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveToDirectNB(double angle1, double angle2, double angle3, double angle4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToDirectNB(angle1, angle2, angle3, angle4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveToZero(void) {
	moveToZeroNB();
	return moveWait();
}

int CMobotGroup::moveToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::moveWait(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveWait();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::relaxJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::relaxJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::reset(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->reset();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::resetToZero(void) {
	resetToZeroNB();
	return moveWait();
}

int CMobotGroup::resetToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->resetToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	int msecs = seconds * 1000.0;

	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->resetToZeroNB();
		rtmp = rtmp->next;
	}

#ifdef _WIN32
	Sleep(msecs);
#else
	usleep(msecs * 1000);
#endif

	// success
	return 0;
}

int CMobotGroup::setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds) {
	int msecs = seconds * 1000.0;

	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointMovementStateTimeNB(id, dir, seconds);
		rtmp = rtmp->next;
	}

#ifdef _WIN32
	Sleep(msecs);
#else
	usleep(msecs * 1000);
#endif

	// success
	return 0;
}

int CMobotGroup::setJointSafetyAngle(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngle(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointSafetyAngleTimeout(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngleTimeout(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointSpeed(robotJointId_t id, double speed) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeed(id, speed);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeeds(speed1, speed2, speed3, speed4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointSpeedRatio(robotJointId_t id, double ratio) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatio(id, ratio);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setMovementStateTimeNB(dir1, dir2, dir3, dir4, seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::setSpeed(double speed, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setSpeed(speed, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::stopOneJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopOneJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::stopTwoJoints(robotJointId_t id1, robotJointId_t id2) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopTwoJoints(id1, id2);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->stopThreeJoints(id1, id2, id3);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	return moveWait();
}

int CMobotGroup::turnLeftNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnLeftNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

int CMobotGroup::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	return moveWait();
}

int CMobotGroup::turnRightNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnRightNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

void* CMobotGroup::motionArchThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, -cmg->_d/2);
	cmg->moveJointToNB(JOINT3, cmg->_d/2);
	cmg->moveJointWait(JOINT2);
	cmg->moveJointWait(JOINT3);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionDistanceThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(RAD2DEG(cmg->_d), 0, 0, RAD2DEG(cmg->_d));

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionInchwormLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();
	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT2, -50);
		cmg->moveJointTo(JOINT3, 50);
		cmg->moveJointTo(JOINT2, 0);
		cmg->moveJointTo(JOINT3, 0);
	}

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionInchwormRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();
	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT3, 50);
		cmg->moveJointTo(JOINT2, -50);
		cmg->moveJointTo(JOINT3, 0);
		cmg->moveJointTo(JOINT2, 0);
	}

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionRollBackwardThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(-cmg->_d, 0, 0, -cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionRollForwardThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(cmg->_d, 0, 0, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionSkinnyThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, cmg->_d);
	cmg->moveJointToNB(JOINT3, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionStandThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();
	cmg->moveJointTo(JOINT2, -85);
	cmg->moveJointTo(JOINT3, 70);
	cmg->moveWait();
	cmg->moveJointTo(JOINT1, 45);
	cmg->moveJointTo(JOINT2, 20);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTumbleLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();

#ifdef _WIN32
	Sleep(1000);
#else
	usleep(1000000);
#endif

	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT2, DEG2RAD(-85));
		cmg->moveJointTo(JOINT3, DEG2RAD(80));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(80));
		cmg->moveJointTo(JOINT2, DEG2RAD(45));
		cmg->moveJointTo(JOINT3, DEG2RAD(-85));
		cmg->moveJointTo(JOINT2, DEG2RAD(80));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(80));
		if (i != (cmg->_i - 1))
			cmg->moveJointTo(JOINT3, DEG2RAD(45));
	}
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTumbleRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();

#ifdef _WIN32
	Sleep(1000);
#else
	usleep(1000000);
#endif

	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT3, DEG2RAD(85));
		cmg->moveJointTo(JOINT2, DEG2RAD(-80));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(-80));
		cmg->moveJointTo(JOINT3, DEG2RAD(-45));
		cmg->moveJointTo(JOINT2, DEG2RAD(85));
		cmg->moveJointTo(JOINT3, DEG2RAD(-80));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(-80));
		if (i != (cmg->_i - 1))
			cmg->moveJointTo(JOINT2, DEG2RAD(-45));
	}
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveWait();

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTurnLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(-cmg->_d, 0, 0, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTurnRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(cmg->_d, 0, 0, -cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionUnstandThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveToDirect(0, 0, 0, 0);
	cmg->moveJointTo(JOINT3, 45);
	cmg->moveJointTo(JOINT2, -85);
	cmg->moveWait();
	cmg->moveToDirect(0, 0, 0, 0);
	cmg->moveJointTo(JOINT2, 20);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

