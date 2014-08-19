template<class T>
Group<T>::Group(void) {
	// create robots
	_robots = NULL;
}

template<class T>
Group<T>::~Group(void) {
	// delete robots
	while (_robots != NULL) {
		robots_t tmp = _robots->next;
		delete _robots;
		_robots = tmp;
	}
}

template<class T>
int Group<T>::addRobot(T &robot) {
	// create new robot
	robots_t nr = new struct robots_s;
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

template<class T>
int Group<T>::addRobots(T robots[], int num) {
	for (int i = 0; i < num; i++) {
		this->addRobot(robots[i]);
	}

	// success
	return 0;
}

template<class T>
int Group<T>::blinkLED(double delay, int num) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->blinkLED(delay, num);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::connect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->connect();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::disconnect(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->disconnect();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::driveBackward(double angle) {
	driveBackwardNB(angle);
	return moveWait();
}

template<class T>
int Group<T>::driveBackwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveBackwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::driveDistance(double distance, double radius) {
	driveDistanceNB(distance, radius);
	return moveWait();
}

template<class T>
int Group<T>::driveDistanceNB(double distance, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveDistanceNB(distance, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::driveForever(void) {
	driveForeverNB();
	return moveWait();
}

template<class T>
int Group<T>::driveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::driveForward(double angle) {
	driveForwardNB(angle);
	return moveWait();
}

template<class T>
int Group<T>::driveForwardNB(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveForwardNB(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::driveTime(double seconds) {
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

template<class T>
int Group<T>::driveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->driveTimeNB(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::holdJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::holdJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::holdJointsAtExit(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->holdJointsAtExit();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::isMoving(void) {
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

template<class T>
int Group<T>::isNotMoving(void) {
	return !(this->isMoving());
}

template<class T>
int Group<T>::moveForeverNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveForeverNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJoint(robotJointId_t id, double angle) {
	moveJointNB(id, angle);
	return moveWait();
}

template<class T>
int Group<T>::moveJointNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointByPowerNB(robotJointId_t id, int power) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointByPowerNB(id, power);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointForeverNB(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointTime(robotJointId_t id, double seconds) {
	this->moveJointTimeNB(id, seconds);

#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointTimeNB(robotJointId_t id, double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointForeverNB(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointTo(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveWait();
}

template<class T>
int Group<T>::moveJointToNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointToByTrackPos(robotJointId_t id, double angle) {
	moveJointToNB(id, angle);
	return moveJointWait(id);
}

template<class T>
int Group<T>::moveJointToByTrackPosNB(robotJointId_t id, double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointToByTrackPosNB(id, angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointWait(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveJointWait(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveTime(double seconds) {
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

template<class T>
int Group<T>::moveTimeNB(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveTimeNB(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveToZero(void) {
	moveToZeroNB();
	return moveWait();
}

template<class T>
int Group<T>::moveToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::moveWait(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->moveWait();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::relaxJoint(robotJointId_t id) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoint(id);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::relaxJoints(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->relaxJoints();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::resetToZero(void) {
	resetToZeroNB();
	return moveWait();
}

template<class T>
int Group<T>::resetToZeroNB(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->resetToZeroNB();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequency(int frequency, double time) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequency(frequency, time);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequencyOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequencyOn(int frequency) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setBuzzerFrequencyOn(frequency);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setLEDColor(char *color) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColor(color);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setLEDColorRGB(int r, int g, int b) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setLEDColorRGB(r, g, b);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setJointSafetyAngle(double angle) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngle(angle);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setJointSafetyAngleTimeout(double seconds) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSafetyAngleTimeout(seconds);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setJointSpeed(robotJointId_t id, double speed) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeed(id, speed);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setJointSpeedRatio(robotJointId_t id, double ratio) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setJointSpeedRatio(id, ratio);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::setSpeed(double speed, double radius) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->setSpeed(speed, radius);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::traceOff(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOff();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::traceOn(void) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->traceOn();
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	return moveWait();
}

template<class T>
int Group<T>::turnLeftNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnLeftNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

template<class T>
int Group<T>::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	return moveWait();
}

template<class T>
int Group<T>::turnRightNB(double angle, double radius, double trackwidth) {
	robots_t rtmp = _robots;
	while (rtmp) {
		rtmp->robot->turnRightNB(angle, radius, trackwidth);
		rtmp = rtmp->next;
	}

	// success
	return 0;
}

