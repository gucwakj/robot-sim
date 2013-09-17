#include "mobotsim.h"

CMobot::CMobot(void) {
	// initialize parameters
	init_params();
	// initialize dimensions
	init_dims();
}

CMobot::~CMobot(void) {
	// remove geoms
	if (_connected) {
		for (int i = NUM_PARTS - 1; i >= 0; i--) { delete [] _geom[i]; }
	}
}

int CMobot::blinkLED(double delay, int num) {
	printf("CMobot::blinkLED not implemented.\n");

	// success
	return 0;
}

int CMobot::connect(void) {
	_simObject.addRobot(this);
	_connected = 1;

	// success
	return 0;
}

int CMobot::disconnect(void) {
	_connected = 0;

	// success
	return 0;
}

int CMobot::driveJointTo(robotJointId_t id, double angle) {
	this->driveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::driveJointToDirect(robotJointId_t id, double angle) {
	this->driveJointToDirectNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::driveJointToDirectNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CMobot::driveJointToNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CMobot::driveTo(double angle1, double angle2, double angle3, double angle4) {
	this->driveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveToDirect(double angle1, double angle2, double angle3, double angle4) {
	this->driveToDirectNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveToDirectNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToDirectNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

int CMobot::driveToNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

int CMobot::getFormFactor(int &formFactor) {
	formFactor = _type;

	// success
	return 0;
}

int CMobot::getJointAngle(robotJointId_t id, double &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CMobot::getJointAngleAverage(robotJointId_t id, double &angle, int numReadings) {
	//initialize variables
	double d;
	angle = 0;
	
	// get joint angle numReadings times
	for (int i = 0; i < numReadings; i++) {
		if(this->getJointAngle(id, d)) {
			return -1;
		}
		angle += d;
	}

	// store average angle
	angle = angle/numReadings;

	// success
	return 0;
}

int CMobot::getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4) {
	this->getJointAngle(ROBOT_JOINT1, angle1);
	this->getJointAngle(ROBOT_JOINT2, angle2);
	this->getJointAngle(ROBOT_JOINT3, angle3);
	this->getJointAngle(ROBOT_JOINT4, angle4);

	// success
	return 0;
}

int CMobot::getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings) {
	this->getJointAngleAverage(ROBOT_JOINT1, angle1, numReadings);
	this->getJointAngleAverage(ROBOT_JOINT2, angle2, numReadings);
	this->getJointAngleAverage(ROBOT_JOINT3, angle3, numReadings);
	this->getJointAngleAverage(ROBOT_JOINT4, angle4, numReadings);

	// success
	return 0;
}

int CMobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed) {
	maxSpeed = _max_speed[id];

	// success
	return 0;
}

int CMobot::getJointSafetyAngle(double &angle) {
	angle = _safety_angle;

	// success
	return 0;
}

int CMobot::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _safety_timeout;

	// success
	return 0;
}

int CMobot::getJointSpeed(robotJointId_t id, double &speed) {
	speed = RAD2DEG(_speed[id]);

	// success
	return 0;
}

int CMobot::getJointSpeedRatio(robotJointId_t id, double &ratio) {
	ratio = _speed[id]/DEG2RAD(_max_speed[id]);
	// success
	return 0;
}

int CMobot::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = RAD2DEG(_speed[0]);
	speed2 = RAD2DEG(_speed[1]);
	speed3 = RAD2DEG(_speed[2]);
	speed4 = RAD2DEG(_speed[3]);

	// success
	return 0;
}

int CMobot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4) {
	ratio1 = _speed[0]/DEG2RAD(_max_speed[0]);
	ratio2 = _speed[1]/DEG2RAD(_max_speed[1]);
	ratio3 = _speed[2]/DEG2RAD(_max_speed[2]);
	ratio4 = _speed[3]/DEG2RAD(_max_speed[3]);

	// success
	return 0;
}

int CMobot::getJointState(robotJointId_t id, robotJointState_t &state) {
	state = (robotJointState_t)(_state[id]);

	// success
	return 0;
}

int CMobot::isConnected(void) {
	return _connected;
}

int CMobot::isMoving(void) {
	robotJointState_t state;

	for (int i = 1; i <= NUM_DOF; i++) {
		this->getJointState((robotJointId_t)i, state);
		if (state == ROBOT_FORWARD || state == ROBOT_BACKWARD) {
			return 1;
		}
	}

	return 0;
}

int CMobot::motionArch(double angle) {
	this->moveJointToNB(ROBOT_JOINT2, -angle/2.0);
	this->moveJointToNB(ROBOT_JOINT3, angle/2.0);
	this->moveJointWait(ROBOT_JOINT2);
	this->moveJointWait(ROBOT_JOINT3);

	// success
	return 0;
}

void* CMobot::motionArchThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionArch(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionArchNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionArchThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionDistance(double distance, double radius) {
	this->motionRollForward(distance/radius);

	// success
	return 0;
}

int CMobot::motionDistanceNB(double distance, double radius) {
	this->motionRollForwardNB(distance/radius);

	// success
	return 0;
}

int CMobot::motionInchwormLeft(int num) {
	this->moveJointToNB(ROBOT_JOINT2, 0);
	this->moveJointToNB(ROBOT_JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(ROBOT_JOINT2, -50);
		this->moveJointTo(ROBOT_JOINT3, 50);
		this->moveJointTo(ROBOT_JOINT2, 0);
		this->moveJointTo(ROBOT_JOINT3, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormLeftThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionInchwormLeft(mArg->i);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionInchwormLeftNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionInchwormLeftThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionInchwormRight(int num) {
	this->moveJointToNB(ROBOT_JOINT2, 0);
	this->moveJointToNB(ROBOT_JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(ROBOT_JOINT3, 50);
		this->moveJointTo(ROBOT_JOINT2, -50);
		this->moveJointTo(ROBOT_JOINT3, 0);
		this->moveJointTo(ROBOT_JOINT2, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormRightThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionInchwormRight(mArg->i);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionInchwormRightNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionInchwormRightThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionRollBackward(double angle) {
	this->move(-angle, 0, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionRollBackwardThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionRollBackward(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionRollBackwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollBackwardThread, (void *)mArg);

	// cleanup
	//delete mArg;

	// success
	return 0;
}

int CMobot::motionRollForward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(ROBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(ROBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(ROBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(ROBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionRollForwardThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionRollForward(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionRollForwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollForwardThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionSkinny(double angle) {
	this->moveJointToNB(ROBOT_JOINT2, angle);
	this->moveJointToNB(ROBOT_JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionSkinnyThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionSkinny(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionSkinnyNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionSkinnyThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionStand(void) {
	this->resetToZero();
	this->moveJointTo(ROBOT_JOINT2, -85);
	this->moveJointTo(ROBOT_JOINT3, 70);
	this->moveWait();
	this->moveJointTo(ROBOT_JOINT1, 45);
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif
	this->moveJointTo(ROBOT_JOINT2, 20);

	// success
	return 0;
}

void* CMobot::motionStandThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionStand();

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionStandNB(void) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionStandThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionTumbleLeft(int num) {
	this->resetToZero();
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif

	for (int i = 0; i < num; i++) {
		this->moveJointTo(ROBOT_JOINT2, -85);
		this->moveJointTo(ROBOT_JOINT3, 75);
		this->moveJointTo(ROBOT_JOINT2, 0);
		this->moveJointTo(ROBOT_JOINT3, 0);
		this->moveJointTo(ROBOT_JOINT2, 80);
		this->moveJointTo(ROBOT_JOINT2, 45);
		this->moveJointTo(ROBOT_JOINT3, -85);
		this->moveJointTo(ROBOT_JOINT2, 75);
		this->moveJointTo(ROBOT_JOINT3, 0);
		this->moveJointTo(ROBOT_JOINT2, 0);
		this->moveJointTo(ROBOT_JOINT3, 75);
		if (i != (num-1)) {
			this->moveJointTo(ROBOT_JOINT3, 45);
		}
	}
	this->moveJointToNB(ROBOT_JOINT2, 0);
	this->moveJointToNB(ROBOT_JOINT3, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleLeftThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionTumbleLeft(mArg->i);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTumbleLeftNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTumbleLeftThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionTumbleRight(int num) {
	this->resetToZero();
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif

	for (int i = 0; i < num; i++) {
		this->moveJointTo(ROBOT_JOINT3, 85);
		this->moveJointTo(ROBOT_JOINT2, -80);
		this->moveJointTo(ROBOT_JOINT3, 0);
		this->moveJointTo(ROBOT_JOINT2, 0);
		this->moveJointTo(ROBOT_JOINT3, -80);
		this->moveJointTo(ROBOT_JOINT3, -45);
		this->moveJointTo(ROBOT_JOINT2, 85);
		this->moveJointTo(ROBOT_JOINT3, -80);
		this->moveJointTo(ROBOT_JOINT2, 0);
		this->moveJointTo(ROBOT_JOINT3, 0);
		this->moveJointTo(ROBOT_JOINT2, -80);
		if (i != (num-1)) {
			this->moveJointTo(ROBOT_JOINT2, -45);
		}
	}
	this->moveJointToNB(ROBOT_JOINT3, 0);
	this->moveJointToNB(ROBOT_JOINT2, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleRightThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionTumbleRight(mArg->i);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTumbleRightNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTumbleRightThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionTurnLeft(double angle) {
	this->move(-angle, 0, 0, angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTurnLeftThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionTurnLeft(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTurnLeftNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnLeftThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionTurnRight(double angle) {
	this->move(angle, 0, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTurnRightThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionTurnRight(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTurnRightNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnRightThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionUnstand(void) {
	this->resetToZero();
	this->moveJointToNB(ROBOT_JOINT3, 45);
	this->moveJointToNB(ROBOT_JOINT2, -85);
	this->moveWait();
	this->resetToZero();

	// success
	return 0;
}

void* CMobot::motionUnstandThread(void *arg) {
	// cast arg
	motionArg_t *mArg = (motionArg_t *)arg;

	// perform motion
	mArg->robot->motionUnstand();

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionUnstandNB(void) {
	// create thread
	THREAD_T motion;

	// store args
	motionArg_t *mArg = new motionArg_t;
	mArg->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionUnstandThread, (void *)mArg);

	// cleanup
	delete mArg;

	// success
	return 0;
}

int CMobot::motionWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int CMobot::move(double angle1, double angle2, double angle3, double angle4) {
	this->moveNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveNB(double angle1, double angle2, double angle3, double angle4) {
	// store angles into array
	double delta[4] = {angle1, angle2, angle3, angle4};

	// lock mutexes
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// enable motor
	for ( int j = 0; j < NUM_DOF; j++ ) {
		dJointEnable(_motor[j]);
		_goal[j] += DEG2RAD(delta[j]);
		_seek[j] = true;
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = ROBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _speed[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = ROBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_speed[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = ROBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
		_success[j] = 0;
	}
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveBackward(double angle) {
	this->moveBackwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveBackwardNB(double angle) {
	return this->moveNB(-angle, 0, 0, -angle);
}

int CMobot::moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4) {
	return this->setMovementStateNB(dir1, dir2, dir3, dir4);
}

int CMobot::moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4, double seconds) {
	return this->setMovementStateTime(dir1, dir2, dir3, dir4, seconds);
}

int CMobot::moveDistance(double distance, double radius) {
	this->moveForwardNB(RAD2DEG(distance/radius));
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveDistanceNB(double distance, double radius) {
	return this->moveForwardNB(RAD2DEG(distance/radius));
}

int CMobot::moveForward(double angle) {
	this->moveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveForwardNB(double angle) {
	return this->moveNB(angle, 0, 0, angle);
}

int CMobot::moveJoint(robotJointId_t id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir) {
	return this->setJointMovementStateNB(id, dir);
}

int CMobot::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	return this->setJointMovementStateTime(id, dir, seconds);
}

int CMobot::moveJointNB(robotJointId_t id, double angle) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	_goal[id] += DEG2RAD(angle);

	// actively seeking an angle
	_seek[id] = true;

	// enable motor
	MUTEX_LOCK(&_angle_mutex);
	dJointEnable(_motor[id]);
	dJointSetAMotorAngle(_motor[id], 0, _angle[id]);

	// set motor state and velocity
	if ( angle > 0 ) {
		_state[id] = ROBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _speed[id]);
	}
	else if ( angle < 0 ) {
		_state[id] = ROBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_speed[id]);
	}
	else if ( fabs(angle-0) < EPSILON ) {
		_state[id] = ROBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[CENTER]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = 0;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveJointTo(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointToDirect(robotJointId_t id, double angle) {
	this->moveJointToDirectNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointToDirectNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CMobot::moveJointToNB(robotJointId_t id, double angle) {
	// store delta angle
	double delta = angle - _angle[id];

	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	_goal[id] = DEG2RAD(angle);

	// actively seeking an angle
	_seek[id] = true;

	// enable motor
	MUTEX_LOCK(&_angle_mutex);
	dJointEnable(_motor[id]);
	dJointSetAMotorAngle(_motor[id], 0, _angle[id]);

	// set motor state and velocity
	if ( delta > 0 ) {
		_state[id] = ROBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _speed[id]);
	}
	else if ( delta < 0 ) {
		_state[id] = ROBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_speed[id]);
	}
	else if ( fabs(delta-0) < EPSILON ) {
		_state[id] = ROBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[CENTER]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = 0;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveJointWait(robotJointId_t id) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ( !_success[id] ) { COND_WAIT(&_success_cond, &_success_mutex); }
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CMobot::moveTo(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveToDirect(double angle1, double angle2, double angle3, double angle4) {
	this->moveToDirectNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveToDirectNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

int CMobot::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	// store angles into array
	double delta[4] = {	DEG2RAD(angle1) - _angle[0], DEG2RAD(angle2) - _angle[1],
						DEG2RAD(angle3) - _angle[2], DEG2RAD(angle4) - _angle[3]};

	// lock mutexes
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// enable motor
	for (int j = 0; j < NUM_DOF; j++) {
		dJointEnable(_motor[j]);
		_goal[j] += delta[j];
		_seek[j] = true;
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = ROBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _speed[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = ROBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_speed[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = ROBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
		_success[j] = 0;
	}
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveToZero(void) {
	return this->moveTo(0, 0, 0, 0);
}

int CMobot::moveToZeroNB(void) {
	return this->moveToNB(0, 0, 0, 0);
}

int CMobot::moveWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ((_success[0] + _success[1] + _success[2] + _success[3]) != NUM_DOF) {
		COND_WAIT(&_success_cond, &_success_mutex);
	}
	for (int i = 0; i < NUM_DOF; i++) {
		_seek[i] = false;
	}
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

void* CMobot::recordAngleThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);

	// get 'num' data points
	for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = *(rArg->robot->_clock)*1000;
		if (i == 0) { start_time = rArg->time[i]; }
		rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angle
		rArg->angle1[i] = rArg->robot->_angle[rArg->id];

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(*(rArg->robot->_clock)*1000) < time ) {
#ifdef _WIN32
			Sleep(time - (int)(*(rArg->robot->_clock)*1000));
#else
			usleep((time - (int)(*(rArg->robot->_clock)*1000))*1000);
#endif
		}
	}

	// signal completion of recording
	SIGNAL(&rArg->robot->_recording_cond, &rArg->robot->_recording_mutex, rArg->robot->_recording[rArg->id] = false);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordAngle(robotJointId_t id, double *time, double *angle, int num, double seconds, int shiftData) {
	// check if recording already
	if (_recording[id]) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle;
	rArg->id = id;
	rArg->num = num;
	rArg->msecs = 1000*seconds;

	// lock recording for joint id
	_recording[id] = true;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CMobot::recordAngleThread, (void *)rArg);

	// success
	return 0;
}

void* CMobot::recordAngleBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time;
	int time = (int)((*(rArg->robot->_clock))*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[rArg->id] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// lock recording
	MUTEX_LOCK(&(rArg->robot->_recording_mutex));

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[rArg->id]; i++) {
		// unlock recording
		MUTEX_UNLOCK(&(rArg->robot->_recording_mutex));

		// store locally num of data points taken
		rArg->robot->_rec_num[rArg->id] = i;

		// resize array if filled current one
		if(i >= rArg->num) {
			rArg->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *rArg->ptime, sizeof(double)*i);
			free(*(rArg->ptime));
			*(rArg->ptime) = newBuf;
			// create larger array for angle
			newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *(rArg->pangle1), sizeof(double)*i);
			free(*(rArg->pangle1));
			*(rArg->pangle1) = newBuf;
		}

		// store joint angles
		(*(rArg->pangle1))[i] = rArg->robot->_angle[rArg->id];

		// store time of data point
		(*rArg->ptime)[i] = *(rArg->robot->_clock)*1000;
		if (i == 0) { start_time = (*rArg->ptime)[i]; }
		(*rArg->ptime)[i] = ((*rArg->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(*(rArg->robot->_clock)*1000) < time ) {
#ifdef _WIN32
			Sleep(time - (int)(*(rArg->robot->_clock)*1000));
#else
			usleep((time - (int)(*(rArg->robot->_clock)*1000))*1000);
#endif
		}

		// lock mutex to check on next loop
		MUTEX_LOCK(&(rArg->robot->_recording_mutex));
	}

	// done recording
	MUTEX_UNLOCK(&(rArg->robot->_recording_mutex));

	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[rArg->id] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData) {
	// check if recording already
	if (_recording[id]) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->id = id;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	time = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	rArg->ptime = &time;
	rArg->pangle1 = &angle;

	// store pointer to recorded angles locally
	_rec_angles[id] = &angle;

	// lock recording for joint id
	_recording[id] = true;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CMobot::recordAngleBeginThread, (void *)rArg);

	// success
	return 0;
}

int CMobot::recordAngleEnd(robotJointId_t id, int &num) {
	// sleep to capture last data point on ending time
#ifdef _WIN32
	Sleep(150);
#else
	usleep(150000);
#endif

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[id] = false;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[id]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[id];

	// success
	return 0;
}

void* CMobot::recordAnglesThread(void *arg) {
	// cast arg struct
    recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
    double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);

	// get 'num' data points
    for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = *(rArg->robot->_clock)*1000;
        if (i == 0) { start_time = rArg->time[i]; }
        rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angles
		rArg->angle1[i] = rArg->robot->_angle[ROBOT_JOINT1];
		rArg->angle2[i] = rArg->robot->_angle[ROBOT_JOINT2];
		rArg->angle3[i] = rArg->robot->_angle[ROBOT_JOINT3];
		rArg->angle4[i] = rArg->robot->_angle[ROBOT_JOINT4];

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(*(rArg->robot->_clock)*1000) < time ) {
#ifdef _WIN32
			Sleep(time - (int)(*(rArg->robot->_clock)*1000));
#else
			usleep((time - (int)(*(rArg->robot->_clock)*1000))*1000);
#endif
		}
    }

	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_recording_mutex);
    for (int i = 0; i < NUM_DOF; i++) {
        rArg->robot->_recording[i] = false;
    }
	COND_SIGNAL(&rArg->robot->_recording_cond);
	MUTEX_UNLOCK(&rArg->robot->_recording_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordAngles(double *time, double *angle1, double *angle2, double *angle3, double *angle4, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < NUM_DOF; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle1;
	rArg->angle2 = angle2;
	rArg->angle3 = angle3;
	rArg->angle4 = angle4;
	rArg->num = num;
	rArg->msecs = 1000*seconds;

	// lock recording for joints
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CMobot::recordAnglesThread, (void *)rArg);

	// success
	return 0;
}


void* CMobot::recordAnglesBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time;
	int time = (int)((*(rArg->robot->_clock))*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[ROBOT_JOINT1] = true;
	rArg->robot->_rec_active[ROBOT_JOINT2] = true;
	rArg->robot->_rec_active[ROBOT_JOINT3] = true;
	rArg->robot->_rec_active[ROBOT_JOINT4] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[rArg->id]; i++) {
		// store locally num of data points taken
		rArg->robot->_rec_num[ROBOT_JOINT1] = i;

		// resize array if filled current one
		if(i >= rArg->num) {
			rArg->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *rArg->ptime, sizeof(double)*i);
			//free(*(rArg->ptime));
			delete *(rArg->ptime);
			*(rArg->ptime) = newBuf;
			// create larger array for angle1
			newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *(rArg->pangle2), sizeof(double)*i);
			free(*(rArg->pangle2));
			*(rArg->pangle2) = newBuf;
			// create larger array for angle2
			newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *(rArg->pangle3), sizeof(double)*i);
			free(*(rArg->pangle3));
			*(rArg->pangle3) = newBuf;
			// create larger array for angle3
			newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *(rArg->pangle3), sizeof(double)*i);
			free(*(rArg->pangle3));
			*(rArg->pangle3) = newBuf;
			// create larger array for angle4
			newBuf = (double *)malloc(sizeof(double) * rArg->num);
			memcpy(newBuf, *(rArg->pangle4), sizeof(double)*i);
			free(*(rArg->pangle4));
			*(rArg->pangle4) = newBuf;
		}

		// store joint angles
		(*(rArg->pangle1))[i] = rArg->robot->_angle[ROBOT_JOINT1];
		(*(rArg->pangle2))[i] = rArg->robot->_angle[ROBOT_JOINT2];
		(*(rArg->pangle3))[i] = rArg->robot->_angle[ROBOT_JOINT3];
		(*(rArg->pangle4))[i] = rArg->robot->_angle[ROBOT_JOINT4];

		// store time of data point
		(*rArg->ptime)[i] = *(rArg->robot->_clock)*1000;
		if (i == 0) { start_time = (*rArg->ptime)[i]; }
		(*rArg->ptime)[i] = ((*rArg->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(*(rArg->robot->_clock)*1000) < time ) {
#ifdef _WIN32
			Sleep(time - (int)(*(rArg->robot->_clock)*1000));
#else
			usleep((time - (int)(*(rArg->robot->_clock)*1000))*1000);
#endif
		}
	}

	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[ROBOT_JOINT1] = false;
	rArg->robot->_rec_active[ROBOT_JOINT2] = false;
	rArg->robot->_rec_active[ROBOT_JOINT3] = false;
	rArg->robot->_rec_active[ROBOT_JOINT4] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, robotRecordData_t &angle4, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < NUM_DOF; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	time = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle1 = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle2 = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle3 = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle4 = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	rArg->ptime = &time;
	rArg->pangle1 = &angle1;
	rArg->pangle2 = &angle2;
	rArg->pangle3 = &angle3;
	rArg->pangle4 = &angle4;

	// store pointer to recorded angles locally
	_rec_angles[ROBOT_JOINT1] = &angle1;
	_rec_angles[ROBOT_JOINT2] = &angle2;
	_rec_angles[ROBOT_JOINT3] = &angle3;
	_rec_angles[ROBOT_JOINT4] = &angle4;

	// lock recording for joint id
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CMobot::recordAnglesBeginThread, (void *)rArg);

	// success
	return 0;
}

int CMobot::recordAnglesEnd(int &num) {
	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[ROBOT_JOINT1] = 0;
	_recording[ROBOT_JOINT2] = 0;
	_recording[ROBOT_JOINT3] = 0;
	_recording[ROBOT_JOINT4] = 0;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[ROBOT_JOINT1] && _rec_active[ROBOT_JOINT2] && _rec_active[ROBOT_JOINT3] && _rec_active[ROBOT_JOINT4]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[ROBOT_JOINT1];

	// success
	return 0;
}

int CMobot::recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData) {
	// set radius of robot
	_radius = radius;

	// record angle of desired joint
	this->recordAngleBegin(id, time, distance, seconds, shiftData);

	// success
	return 0;
}

int CMobot::recordDistanceEnd(robotJointId_t id, int &num) {
	// end recording of angles
	this->recordAngleEnd(id, num);

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		(*_rec_angles[id])[i] = DEG2RAD((*_rec_angles[id])[i]) * _radius;
	}

	// success
	return 0;
}

int CMobot::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, robotRecordData_t &distance4, double radius, double seconds, int shiftData) {
	// set radius of robot
	_radius = radius;

	// record angles
	this->recordAnglesBegin(time, distance1, distance2, distance3, distance4, seconds, shiftData);

	// success
	return 0;
}

int CMobot::recordDistancesEnd(int &num) {
	// end recording of angles
	this->recordAnglesEnd(num);

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < NUM_DOF; j++) {
			(*_rec_angles[j])[i] = DEG2RAD((*_rec_angles[j])[i]) * _radius;
		}
	}

	// success
	return 0;
}

int CMobot::recordWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_recording_mutex);
	while ( _recording[0] || _recording[1] || _recording[2] || _recording[3] ) {
		COND_WAIT(&_recording_cond, &_recording_mutex);
	}
	MUTEX_UNLOCK(&_recording_mutex);

	// success
	return 0;
}

int CMobot::reset(void) {
	MUTEX_LOCK(&_angle_mutex);
	for (int i = 0; i < NUM_DOF; i++) {
		_offset[i] = _angle[i];
		_angle[i] = 0;
		_goal[i] -= _offset[i];
		dJointSetAMotorAngle(_motor[i], 0, _angle[i]);
	}
	MUTEX_UNLOCK(&_angle_mutex);

	// success
	return 0;
}

int CMobot::resetToZero(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_angle_mutex);
	int rev = (int)(_angle[LE]/2/M_PI);
	if (rev) _angle[LE] -= 2*rev*M_PI;
	rev = (int)(_angle[RE]/2/M_PI);
	if (rev) _angle[RE] -= 2*rev*M_PI;
	MUTEX_UNLOCK(&_angle_mutex);

	// move to zero position
	this->moveToZero();

	// success
	return 0;
}

int CMobot::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_angle_mutex);
	int rev = (int)(_angle[LE]/2/M_PI);
	if (rev) _angle[LE] -= 2*rev*M_PI;
	rev = (int)(_angle[RE]/2/M_PI);
	if (rev) _angle[RE] -= 2*rev*M_PI;
	MUTEX_UNLOCK(&_angle_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int CMobot::setExitState(robotJointState_t exitState) {
	_simObject.setExitState();

	// success
	return 0;
}

int CMobot::setJointMovementStateNB(robotJointId_t id, robotJointState_t dir) {
	// lock mutexes
	MUTEX_LOCK(&_success_mutex);

	// enable motor
	dJointEnable(_motor[id]);
	dJointSetAMotorAngle(_motor[id], 0, _angle[id]);
	_seek[id] = false;
	switch (dir) {
		case ROBOT_FORWARD:
			_state[id] = ROBOT_FORWARD;
			dJointSetAMotorParam(_motor[id], dParamVel, _speed[id]);
			break;
		case ROBOT_BACKWARD:
			_state[id] = ROBOT_BACKWARD;
			dJointSetAMotorParam(_motor[id], dParamVel, -_speed[id]);
			break;
		case ROBOT_HOLD:
			_state[id] = ROBOT_HOLD;
			dJointSetAMotorParam(_motor[id], dParamVel, 0);
			break;
		case ROBOT_NEUTRAL:
			_state[id] = ROBOT_NEUTRAL;
			dJointDisable(_motor[id]);
			break;
	}
	_success[id] = 1;
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CMobot::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	// move joint
	this->setJointMovementStateNB(id, dir);

	// sleep
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// sleep
	this->setJointMovementStateNB(id, ROBOT_HOLD);

	// success
	return 0;
}

void* CMobot::setJointMovementStateTimeNBThread(void *arg) {
	// cast argument
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// sleep
#ifdef _WIN32
	Sleep(rArg->msecs);
#else
	usleep(rArg->msecs * 1000);
#endif

	// hold all robot motion
	CMobot *ptr = dynamic_cast<CMobot *>(rArg->robot);
	ptr->setJointMovementStateNB(rArg->id, ROBOT_HOLD);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds) {
	// set up threading
	THREAD_T moving;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->id = id;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->setJointMovementStateNB(id, dir);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&CMobot::setJointMovementStateTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int CMobot::setJointSafetyAngle(double angle) {
	_safety_angle = angle;

	// success
	return 0;
}

int CMobot::setJointSafetyAngleTimeout(double seconds) {
	_safety_timeout = seconds;

	// success
	return 0;
}

int CMobot::setJointSpeed(robotJointId_t id, double speed) {
	_speed[id] = DEG2RAD((speed > _max_speed[id]) ? _max_speed[id] : speed);

	// success
	return 0;
}

int CMobot::setJointSpeedRatio(robotJointId_t id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * _max_speed[(int)id]);
}

int CMobot::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	_speed[0] = DEG2RAD((speed1 > _max_speed[0]) ? _max_speed[0] : speed1);
	_speed[1] = DEG2RAD((speed2 > _max_speed[1]) ? _max_speed[1] : speed2);
	_speed[2] = DEG2RAD((speed3 > _max_speed[2]) ? _max_speed[2] : speed3);
	_speed[3] = DEG2RAD((speed4 > _max_speed[3]) ? _max_speed[3] : speed4);

	// success
	return 0;
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	this->setJointSpeedRatio(ROBOT_JOINT1, ratio1);
	this->setJointSpeedRatio(ROBOT_JOINT2, ratio2);
	this->setJointSpeedRatio(ROBOT_JOINT3, ratio3);
	this->setJointSpeedRatio(ROBOT_JOINT4, ratio4);

	// success
	return 0;
}

int CMobot::setMotorPower(robotJointId_t id, int power) {
	printf("not implemented yet\n");

	// success
	return 0;
}

int CMobot::setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, robotJointState_t dir4) {
	this->setJointMovementStateNB(ROBOT_JOINT1, dir1);
	this->setJointMovementStateNB(ROBOT_JOINT2, dir2);
	this->setJointMovementStateNB(ROBOT_JOINT3, dir3);
	this->setJointMovementStateNB(ROBOT_JOINT4, dir4);

	// success
	return 0;
}

int CMobot::setMovementStateTime(robotJointState_t dir1,
								 robotJointState_t dir2,
								 robotJointState_t dir3,
								 robotJointState_t dir4, double seconds) {
	// set joint movements
	this->setJointMovementStateNB(ROBOT_JOINT1, dir1);
	this->setJointMovementStateNB(ROBOT_JOINT2, dir2);
	this->setJointMovementStateNB(ROBOT_JOINT3, dir3);
	this->setJointMovementStateNB(ROBOT_JOINT4, dir4);

	// sleep
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// stop motion
	this->setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);

	// success
	return 0;
}

void* CMobot::setMovementStateTimeNBThread(void *arg) {
	// cast argument
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// sleep
#ifdef _WIN32
	Sleep(rArg->msecs);
#else
	usleep(rArg->msecs * 1000);
#endif

	// hold all robot motion
	CMobot *ptr = dynamic_cast<CMobot *>(rArg->robot);
	ptr->setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::setMovementStateTimeNB(robotJointState_t dir1,
								   robotJointState_t dir2,
								   robotJointState_t dir3,
								   robotJointState_t dir4, double seconds) {
	// set up threading
	THREAD_T moving;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->setJointMovementStateNB(ROBOT_JOINT1, dir1);
	this->setJointMovementStateNB(ROBOT_JOINT2, dir2);
	this->setJointMovementStateNB(ROBOT_JOINT3, dir3);
	this->setJointMovementStateNB(ROBOT_JOINT4, dir4);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&CMobot::setMovementStateTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int CMobot::setTwoWheelRobotSpeed(double speed, double radius) {
	this->setJointSpeed(ROBOT_JOINT1, RAD2DEG(speed/radius));
	this->setJointSpeed(ROBOT_JOINT4, RAD2DEG(speed/radius));

	// success
	return 0;
}

int CMobot::stop(void) {
	this->stopAllJoints();

	// success
	return 0;
}

int CMobot::stopOneJoint(robotJointId_t id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int CMobot::stopTwoJoints(robotJointId_t id1, robotJointId_t id2) {
	this->setJointSpeed(id1, 0);
	this->setJointSpeed(id2, 0);

	// success
	return 0;
}

int CMobot::stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3) {
	this->setJointSpeed(id1, 0);
	this->setJointSpeed(id2, 0);
	this->setJointSpeed(id3, 0);

	// success
	return 0;
}

int CMobot::stopAllJoints(void) {
	this->setJointSpeed(ROBOT_JOINT1, 0);
	this->setJointSpeed(ROBOT_JOINT2, 0);
	this->setJointSpeed(ROBOT_JOINT3, 0);
	this->setJointSpeed(ROBOT_JOINT4, 0);

	// success
	return 0;
}

int CMobot::turnLeft(double angle, double radius, double tracklength) {
	this->turnLeftNB(angle, radius, tracklength);
	this->moveWait();

	// success
	return 0;
}

int CMobot::turnLeftNB(double angle, double radius, double tracklength) {
	// calculate joint angle from global turn angle
	angle = (angle*tracklength)/(2*radius);

	// move
	this->moveNB(-angle, 0, 0, angle);

	// success
	return 0;
}

int CMobot::turnRight(double angle, double radius, double tracklength) {
	this->turnRightNB(angle, radius, tracklength);
	this->moveWait();

	// success
	return 0;
}

int CMobot::turnRightNB(double angle, double radius, double tracklength) {
	// calculate joint angle from global turn angle
	angle = (angle*tracklength)/(2*radius);

	// move
	this->moveNB(angle, 0, 0, -angle);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CMobot::addToSim(dWorldID &world, dSpaceID &space, dReal *clock) {
	_world = world;
    _space = dHashSpaceCreate(space);
	_clock = clock;

	// success
	return 0;
}

int CMobot::build(bot_t robot) {
	// create rotation matrix
	dReal   sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta, -cphi*stheta*spsi - sphi*cpsi, -cphi*stheta*cpsi + sphi*spsi, 0,
				  sphi*ctheta, -sphi*stheta*spsi + cphi*cpsi, -sphi*stheta*cpsi - cphi*spsi, 0,
				  stheta, ctheta*spsi, ctheta*cpsi, 0};

	// check for wheels
	Conn_t *ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->type == BIGWHEEL || ctmp->type == SMALLWHEEL) {
			robot->z += ((ctmp->type == SMALLWHEEL) ? _smallwheel_radius : _bigwheel_radius) - _end_height/2;
			break;
		}
		ctmp = ctmp->next;
	}

	// build robot
	this->build_individual(robot->x, robot->y, robot->z, R, robot->angle1, robot->angle2, robot->angle3, robot->angle4);

	// add connectors
	ctmp = robot->conn;
	while (ctmp) {
		if ( ctmp->robot == _id ) {
			this->add_connector(ctmp->type, ctmp->face1);
		}
		ctmp = ctmp->next;
	}

	// success
	return 0;
}

int CMobot::build(bot_t robot, CRobot *base, Conn_t *conn) {
	// build robot
	this->build_attached(robot, base, conn);

	// add connectors
	Conn_t *ctmp = robot->conn;
	while (ctmp) {
		if ( ctmp->robot == _id )
			this->add_connector(ctmp->type, ctmp->face1);
		ctmp = ctmp->next;
	}

	// success
	return 0;
}

dReal CMobot::getAngle(int i) {
	dReal angle = 0;
	if (i == LE || i == RE)
		angle = mod_angle(_angle[i], dJointGetHingeAngle(_joint[i]), dJointGetHingeAngleRate(_joint[i]));
	else
		angle = dJointGetHingeAngle(_joint[i]);
	return angle;
}

dBodyID CMobot::getBodyID(int id) {
    return _body[id];
}

int CMobot::getConnectionParams(int face, dMatrix3 R, dReal *p) {
	const dReal *pos, *R1;
	dMatrix3 R2;
	double offset[3] = {0};
	int i = 1;

	switch (face) {
		case 1:
			pos = dBodyGetPosition(_body[ENDCAP_L]);
			R1 = dBodyGetRotation(_body[ENDCAP_L]);
			offset[0] = -_end_depth/2;
			p[0] = pos[0] + R1[0]*offset[0];
			p[1] = pos[1] + R1[4]*offset[0];
			p[2] = pos[2] + R1[8]*offset[0];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 2: case 5:
			pos = dGeomGetPosition(_geom[BODY_L][0]);
			R1 = dBodyGetRotation(_body[BODY_L]);
			i = ((face == 5) ? 1 : -1);
			offset[0] = -_body_end_depth/2 + _body_mount_center;
			offset[1] = i*_body_width/2;
			p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
			p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 3: case 6:
			pos = dBodyGetPosition(_body[CENTER]);
			R1 = dBodyGetRotation(_body[CENTER]);
			i = (face == 6) ? 1 : -1;
			offset[1] = i*(_body_width/2) - _center_offset;
			p[0] = pos[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[5]*offset[1];
			p[2] = pos[2] + R1[9]*offset[1];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 4: case 7:
			pos = dGeomGetPosition(_geom[BODY_R][0]);
			R1 = dBodyGetRotation(_body[BODY_R]);
			i = (face == 7) ? 1 : -1;
			offset[0] = _body_end_depth/2 - _body_mount_center;
			offset[1] = i*_body_width/2;
			p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
			p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 8:
			pos = dBodyGetPosition(_body[ENDCAP_R]);
			R1 = dBodyGetRotation(_body[ENDCAP_R]);
			offset[0] = _end_depth/2;
			p[0] = pos[0] + R1[0]*offset[0];
			p[1] = pos[1] + R1[4]*offset[0];
			p[2] = pos[2] + R1[8]*offset[0];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], 0);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
	}

	// success
	return 0;
}

dBodyID CMobot::getConnectorBodyID(int face) {
	conn_t ctmp = _conn;
	while (ctmp) {
		if (ctmp->face == face) {
			return ctmp->body;
		}
		ctmp = ctmp->next;
	}
	return NULL;
}

dBodyID CMobot::getConnectorBodyIDs(int num) {
	conn_t ctmp = _conn;
	int i = 0;
	while (ctmp && i++ < num)
		ctmp = ctmp->next;
	if (ctmp) {
		return ctmp->body;
	}
	return NULL;
}

/*int CMobot::getEnc(int id) {
	
}*/

int CMobot::getRobotID(void) {
	return _id;
}

dJointID CMobot::getMotorID(int id) {
    return _motor[id];
}

dReal CMobot::getPosition(int body, int i) {
	const dReal *pos = dBodyGetPosition(_body[body]);
	return pos[i];
}

dReal CMobot::getRotation(int body, int i) {
	const dReal *R = dBodyGetRotation(_body[body]);
	dReal angles[3] = {0};
    if ( fabs(R[8]-1) < DBL_EPSILON ) {         // R_31 == 1; theta = M_PI/2
        angles[0] = atan2(-R[1], -R[2]);		// psi
        angles[1] = M_PI/2;						// theta
        angles[2] = 0;							// phi
    }
    else if ( fabs(R[8]+1) < DBL_EPSILON ) {    // R_31 == -1; theta = -M_PI/2
        angles[0] = atan2(R[1], R[2]);			// psi
        angles[1] = -M_PI/2;					// theta
        angles[2] = 0;							// phi
    }
    else {
        angles[1] = asin(R[8]);
        angles[0] = atan2(R[9]/cos(angles[0]), R[10]/cos(angles[0]));
        angles[2] = atan2(R[4]/cos(angles[0]), R[0]/cos(angles[0]));
    }
	return angles[i];
}

bool CMobot::getSuccess(int i) {
	return (bool)(_success[i]);
}

int CMobot::getType(void) {
	return _type;
}

bool CMobot::isHome(void) {
    return ( fabs(_angle[LE]) < EPSILON && fabs(_angle[LB]) < EPSILON && fabs(_angle[RB]) < EPSILON && fabs(_angle[RE]) < EPSILON );
}

int CMobot::setID(int id) {
	_id = id;
	return 0;
}

void CMobot::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);

	// update angle values for each degree of freedom
	for (int i = 0; i < NUM_DOF; i++) {
		// store current angle
		_angle[i] = getAngle(i);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i], 0, _angle[i]);
		// drive motor to get current angle to match future angle
		if (_seek[i]) {
			if (_angle[i] < _goal[i] - _encoder) {
				_state[i] = ROBOT_FORWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]);
			}
			else if (_angle[i] > _goal[i] + _encoder) {
				_state[i] = ROBOT_BACKWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]);
			}
			else {
				_state[i] = ROBOT_HOLD;
				dJointSetAMotorParam(_motor[i], dParamVel, 0);
			}
		}
		else {
			switch (_state[i]) {
				case ROBOT_FORWARD:
					dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]);
					break;
				case ROBOT_BACKWARD:
					dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]);
					break;
				case ROBOT_HOLD:
					dJointSetAMotorParam(_motor[i], dParamVel, 0);
					break;
				case ROBOT_NEUTRAL:
					dJointDisable(_motor[i]);
					break;
			}
		}
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void CMobot::simPostCollisionThread(void) {
	// lock
	MUTEX_LOCK(&_success_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < NUM_DOF; i++) {
		_success[i] = (!(int)(dJointGetAMotorParam(this->getMotorID(i), dParamVel)*1000) );
	}
	if ( _success[0] && _success[1] && _success[2] && _success[3] ) {
		COND_SIGNAL(&_success_cond);
	}

	// unlock
	MUTEX_UNLOCK(&_success_mutex);
}

#ifdef ENABLE_GRAPHICS
void CMobot::draw(osg::Group *root) {
	// initialize variables
	osg::ref_ptr<osg::Group> robot = new osg::Group();
	osg::ref_ptr<osg::Geode> body[5];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[5];
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for ( int i = 0; i < 5; i++) {
		body[i] = new osg::Geode;
	}

	// left endcap
	pos = dGeomGetOffsetPosition(_geom[0][0]);
	dGeomGetOffsetQuaternion(_geom[0][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_width - 2*_end_radius, _end_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[0][1]);
	dGeomGetOffsetQuaternion(_geom[0][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[0][2]);
	dGeomGetOffsetQuaternion(_geom[0][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[0][3]);
	dGeomGetOffsetQuaternion(_geom[0][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[0][4]);
	dGeomGetOffsetQuaternion(_geom[0][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[0][5]);
	dGeomGetOffsetQuaternion(_geom[0][5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[0][6]);
	dGeomGetOffsetQuaternion(_geom[0][6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));
    
    // left body 
	pos = dGeomGetOffsetPosition(_geom[BODY_L][0]);
	dGeomGetOffsetQuaternion(_geom[BODY_L][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_end_depth, _body_width, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_L][1]);
	dGeomGetOffsetQuaternion(_geom[BODY_L][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_length, _body_inner_width_left, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_L][2]);
	dGeomGetOffsetQuaternion(_geom[BODY_L][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_length, _body_inner_width_right, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_L][3]);
	dGeomGetOffsetQuaternion(_geom[BODY_L][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_inner_width_left);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[BODY_L][4]);
	dGeomGetOffsetQuaternion(_geom[BODY_L][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_inner_width_right);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_L]->addDrawable(new osg::ShapeDrawable(cyl));

	// center
	pos = dGeomGetOffsetPosition(_geom[CENTER][0]);
	dGeomGetOffsetQuaternion(_geom[CENTER][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _center_length, _center_width, _center_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[CENTER]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[CENTER][1]);
	dGeomGetOffsetQuaternion(_geom[CENTER][1], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _center_radius, _center_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[CENTER]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[CENTER][2]);
	dGeomGetOffsetQuaternion(_geom[CENTER][2], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _center_radius, _center_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[CENTER]->addDrawable(new osg::ShapeDrawable(cyl));

    // right body 
	pos = dGeomGetOffsetPosition(_geom[BODY_R][0]);
	dGeomGetOffsetQuaternion(_geom[BODY_R][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_end_depth, _body_width, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_R][1]);
	dGeomGetOffsetQuaternion(_geom[BODY_R][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_length, _body_inner_width_left, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_R][2]);
	dGeomGetOffsetQuaternion(_geom[BODY_R][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_length, _body_inner_width_right, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[BODY_R][3]);
	dGeomGetOffsetQuaternion(_geom[BODY_R][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_inner_width_left);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[BODY_R][4]);
	dGeomGetOffsetQuaternion(_geom[BODY_R][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_inner_width_right);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY_R]->addDrawable(new osg::ShapeDrawable(cyl));

	// right endcap
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][0]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_width - 2*_end_radius, _end_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][1]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][2]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][3]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][4]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][5]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_R][6]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_R][6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/body.png)));
    tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    robot->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// position each body within robot
	for (int i = 0; i < NUM_PARTS; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		robot->addChild(pat[i].get());
	}

	conn_t ctmp = _conn;
	while (ctmp) {
		switch (ctmp->type) {
			case BIGWHEEL:
				this->draw_bigwheel(ctmp, robot);
				break;
			case CASTER:
				this->draw_caster(ctmp, robot);
				break;
			case SIMPLE:
				this->draw_simple(ctmp, robot);
				break;
			case SMALLWHEEL:
				this->draw_smallwheel(ctmp, robot);
				break;
			case SQUARE:
				this->draw_square(ctmp, robot);
				break;
			case TANK:
				this->draw_tank(ctmp, robot);
				break;
		}
		ctmp = ctmp->next;
	}

	// set update callback for robot
	robot->setUpdateCallback(new mobotNodeCallback(this));

	// optimize robot
	osgUtil::Optimizer optimizer;
	optimizer.optimize(robot);

	// add to scenegraph
	root->addChild(robot);
}
#endif // ENABLE_GRAPHICS

/**********************************************************
	private functions
 **********************************************************/
int CMobot::add_connector(int type, int face) {
	// create new connector
	conn_t nc = (conn_t)malloc(sizeof(struct conn_s));
	nc->face = face; 
	nc->type = type; 
	nc->next = NULL;

	// add to list of connectors
	conn_t ctmp = _conn;
	if ( _conn == NULL )
		_conn = nc;
	else {
		while (ctmp->next)
			ctmp = ctmp->next;
		ctmp->next = nc;
	}

	// build connector
	switch (type) {
		case BIGWHEEL:
			this->build_bigwheel(nc, face);
			break;
		case CASTER:
			this->build_caster(nc, face);
			break;
		case SIMPLE:
			this->build_simple(nc, face);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(nc, face);
			break;
		case SQUARE:
			this->build_square(nc, face);
			break;
		case TANK:
			this->build_tank(nc, face);
			break;
	}

	// success
	return 0;
}

int CMobot::build_individual(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
    _geom[ENDCAP_L] = new dGeomID[7];
    _geom[BODY_L] = new dGeomID[5];
    _geom[CENTER] = new dGeomID[3];
    _geom[BODY_R] = new dGeomID[5];
    _geom[ENDCAP_R] = new dGeomID[7];

	// initialize PID class
	//for ( int i = 0; i < NUM_DOF; i++ ) { _pid[i].init(100, 1, 10, 0.1, 0.004); }

    // adjust input height by body height
	if (z < _end_height/2) {
    	x += R[2]*_end_height/2;
    	y += R[6]*_end_height/2;
    	z += R[10]*_end_height/2;
	}

    // convert input angles to radians
    _angle[LE] = DEG2RAD(r_le);       // left end
    _angle[LB] = DEG2RAD(r_lb);       // left body
    _angle[RB] = DEG2RAD(r_rb);       // right body
    _angle[RE] = DEG2RAD(r_re);       // right end

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-_body_radius - _body_length - _body_end_depth - _end_depth/2, 0, 0, -_body_radius/2 - _body_length - _body_end_depth, 0, 0};
    dReal lb[6] = {-_body_radius - _body_length - _body_end_depth/2, 0, 0, -_center_length/2, _center_width/2, 0};
	dReal ce[3] = {0, _center_offset, 0};
    dReal rb[6] = {_body_radius + _body_length + _body_end_depth/2, 0, 0, _center_length/2, _center_width/2, 0};
    dReal re[6] = {_body_radius + _body_length + _body_end_depth + _end_depth/2, 0, 0, _body_radius/2 + _body_length + _body_end_depth, 0, 0};

	// build robot bodies
    this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->build_center(R[1]*ce[1] + x, R[5]*ce[1] + y, R[9]*ce[1] + z, R);
    this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // joint for left endcap to body
    _joint[0] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetHingeAnchor(_joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x,
									R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y,
									R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[0], dParamCFM, 0);

    // joint for center to left body 1
    _joint[1] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[1], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[1], R[0]*lb[3] + R[1]*(_center_offset+lb[4]) + R[2]*lb[5] + x, 
									R[4]*lb[3] + R[5]*(_center_offset+lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset+lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[1], dParamCFM, 0);

    // joint for center to left body 2
    _joint[4] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[4], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[4], R[0]*lb[3] + R[1]*(_center_offset-lb[4]) + R[2]*lb[5] + x,
									R[4]*lb[3] + R[5]*(_center_offset-lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset-lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[4], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[4], dParamCFM, 0);

    // joint for center to right body 1
    _joint[2] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[2], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[2], R[0]*rb[3] + R[1]*(_center_offset+rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset+rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset+rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[2], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[2], dParamCFM, 0);

    // joint for center to right body 2
    _joint[5] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[5], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[5], R[0]*rb[3] + R[1]*(_center_offset-rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset-rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset-rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[5], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[5], dParamCFM, 0);

    // joint for right body to endcap
    _joint[3] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetHingeAnchor(_joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x,
									R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y,
									R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(_joint[3], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[3], dParamCFM, 0);

    // create rotation matrices for each body part
    dMatrix3 R_e, R_b, R_le, R_lb, R_rb, R_re;
    dRFromAxisAndAngle(R_b, 0, 1, 0, _angle[LB]);
    dMultiply0(R_lb, R, R_b, 3, 3, 3);
    dRFromAxisAndAngle(R_e, -1, 0, 0, _angle[LE]);
    dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
    dRFromAxisAndAngle(R_b, 0, 1, 0, _angle[RB]);
    dMultiply0(R_rb, R, R_b, 3, 3, 3);
    dRFromAxisAndAngle(R_e, -1, 0, 0, _angle[RE]);
    dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

	// if bodies are rotated, then redraw
	if (_angle[LE] != 0 || _angle[LB] != 0 ||_angle[RB] != 0 || _angle[RE] != 0 ) {
    	// offset values from center of robot
    	dReal le_r[3] = {-_body_radius - (_body_length + _body_end_depth + _end_depth/2)*cos(_angle[LB]), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_angle[LB])};
    	dReal lb_r[3] = {-_body_radius - (_body_length + _body_end_depth/2)*cos(_angle[LB]), 0, (_body_length + _body_end_depth/2)*sin(_angle[LB])};
    	dReal rb_r[3] = {_body_radius + (_body_length + _body_end_depth/2)*cos(_angle[RB]), 0, (_body_length + _body_end_depth/2)*sin(_angle[RB])};
    	dReal re_r[3] = {_body_radius + (_body_length + _body_end_depth + _end_depth/2)*cos(_angle[RB]), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_angle[RB])};

    	// re-build pieces of module
    	this->build_endcap(ENDCAP_L, R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
    	this->build_body(BODY_L, R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, r_lb);
    	this->build_body(BODY_R, R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, r_rb);
    	this->build_endcap(ENDCAP_R, R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);
	}

    // motor for left endcap to body
    _motor[0] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetAMotorMode(_motor[0], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[0], 1);
    dJointSetAMotorAxis(_motor[0], 0, 1, R_lb[0], R_lb[4], R_lb[8]);
    dJointSetAMotorAngle(_motor[0], 0, 0);
    dJointSetAMotorParam(_motor[0], dParamCFM, 0);
    dJointSetAMotorParam(_motor[0], dParamFMax, _max_force[LE]);
	dJointDisable(_motor[0]);

    // motor for center to left body
    _motor[1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[1], _body[CENTER], _body[BODY_L]);
    dJointSetAMotorMode(_motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[1], 1);
    dJointSetAMotorAxis(_motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[1], 0, 0);
    dJointSetAMotorParam(_motor[1], dParamCFM, 0);
    dJointSetAMotorParam(_motor[1], dParamFMax, _max_force[LB]);
	dJointDisable(_motor[1]);

    // motor for center to right body
    _motor[2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[2], _body[CENTER], _body[BODY_R]);
    dJointSetAMotorMode(_motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[2], 1);
    dJointSetAMotorAxis(_motor[2], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[2], 0, 0);
    dJointSetAMotorParam(_motor[2], dParamCFM, 0);
    dJointSetAMotorParam(_motor[2], dParamFMax, _max_force[RB]);
	dJointDisable(_motor[2]);

    // motor for right body to endcap
    _motor[3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetAMotorMode(_motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[3], 1);
    dJointSetAMotorAxis(_motor[3], 0, 1, R_rb[0], R_rb[4], R_rb[8]);
    dJointSetAMotorAngle(_motor[3], 0, 0);
    dJointSetAMotorParam(_motor[3], dParamCFM, 0);
    dJointSetAMotorParam(_motor[3], dParamFMax, _max_force[RE]);
	dJointDisable(_motor[3]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CMobot::build_attached(bot_t robot, CRobot *base, Conn_t *conn) {
	// initialize new variables
	int i = 1;
	dReal m[3] = {0}, offset[3] = {0};
	dMatrix3 R, R1, R2, R3, R4, R5, R6;

	// generate parameters for base robot
	base->getConnectionParams(conn->face1, R, m);

	// generate parameters for connector
	this->get_connector_params(conn, R, m);

	// collect data from struct
	dReal r_le = robot->angle1;
	dReal r_lb = robot->angle2;
	dReal r_rb = robot->angle3;
	dReal r_re = robot->angle4;

	switch (conn->face2) {
		case 1:
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], 0);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], DEG2RAD(r_le));
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -DEG2RAD(r_lb));
			dMultiply0(R6, R5, R4, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_lb));
			offset[0] = _end_depth + _body_end_depth + _body_length + R1[0]*_body_radius;
			offset[1] = R1[4]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 2: case 5:
			i = (conn->face2 == 2) ? -1 : 1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], i*M_PI/2);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -DEG2RAD(r_lb));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_lb));
			offset[0] = _body_width/2;
			offset[1] = i*_body_end_depth + i*_body_length - i*_body_mount_center + i*R1[0]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 3: case 6:
			i = (conn->face2 == 3) ? -1 : 1;
			// rotation matrix
    		dRFromAxisAndAngle(R1, R[2], R[6], R[10], i*M_PI/2);
        	dMultiply0(R6, R1, R, 3, 3, 3);
			// center offset
			offset[0] = _body_width/2;
			break;
		case 4: case 7:
			i = (conn->face2 == 4) ? 1 : -1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], -i*M_PI/2);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], DEG2RAD(r_rb));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_rb));
			offset[0] = _body_width/2;
			offset[1] = i*_body_end_depth + i*_body_length - i*_body_mount_center + i*R1[0]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 8:
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -DEG2RAD(r_re));
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], DEG2RAD(r_rb));
			dMultiply0(R6, R5, R4, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_rb));
			offset[0] = _end_depth + _body_end_depth + _body_length + R1[0]*_body_radius;
			offset[1] = R1[4]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
	}

	// adjust position by rotation matrix
	m[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	m[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	m[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];

    // build new module
	this->build_individual(m[0], m[1], m[2], R6, r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach two modules
	this->fix_body_to_connector(base->getConnectorBodyID(conn->face1), conn->face2);

	// success
	return 0;
}

int CMobot::build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
	int i = 1;
	if ( id == BODY_R )
		i = -1;

    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 2700, _body_end_depth, _center_height, _body_width );
    dMassAdd(&m, &m1);
    // create mass 2
    dMassSetBox(&m2, 2700, _body_inner_width_left, _end_depth, _body_width );
    dMassTranslate(&m2, 0.01524*i, -0.0346, 0 );
    dMassAdd(&m, &m2);
    // create mass 3
    dMassSetBox(&m3, 2700, _body_inner_width_right, _end_depth, _body_width );
    dMassTranslate(&m3, 0.01524*i, 0.0346, 0 );
    dMassAdd(&m, &m3);
    //dMassSetParameters( &m, 500, 1, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[id], x, y, z);
    dBodySetRotation(_body[id], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - face
    _geom[id][0] = dCreateBox(_space, _body_end_depth, _body_width, _body_height);
    dGeomSetBody(_geom[id][0], _body[id]);
    dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
    // set geometry 2 - side square
    _geom[id][1] = dCreateBox( _space, _body_length, _body_inner_width_left, _body_height);
    dGeomSetBody( _geom[id][1], _body[id]);
    dGeomSetOffsetPosition( _geom[id][1], i*_body_length/2 + i*_body_end_depth/2 - m.c[0], -_body_width/2 + _body_inner_width_left/2 - m.c[1], -m.c[2] );
    // set geometry 3 - side square
    _geom[id][2] = dCreateBox( _space, _body_length, _body_inner_width_right, _body_height);
    dGeomSetBody( _geom[id][2], _body[id]);
    dGeomSetOffsetPosition( _geom[id][2], i*_body_length/2 + i*_body_end_depth/2 - m.c[0], _body_width/2 - _body_inner_width_right/2 - m.c[1], -m.c[2] );
    // set geometry 4 - side curve
    _geom[id][3] = dCreateCylinder( _space, _body_radius, _body_inner_width_left);
    dGeomSetBody( _geom[id][3], _body[id]);
    dGeomSetOffsetPosition( _geom[id][3], i*_body_length + i*_body_end_depth/2 - m.c[0], -_body_width/2 + _body_inner_width_left/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[id][3], R2);
    // set geometry 5 - side curve
    _geom[id][4] = dCreateCylinder( _space, _body_radius, _body_inner_width_right);
    dGeomSetBody( _geom[id][4], _body[id]);
    dGeomSetOffsetPosition( _geom[id][4], i*_body_length + i*_body_end_depth/2 - m.c[0], _body_width/2 - _body_inner_width_right/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[id][4], R2);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CMobot::build_center(dReal x, dReal y, dReal z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetZero(&m);
    dMassSetCapsule(&m, 2700, 1, _center_radius, _center_length );
    dMassAdjust(&m, 0.24);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[CENTER], x, y, z);
    dBodySetRotation(_body[CENTER], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);

    // set geometry 1 - center rectangle
    _geom[CENTER][0] = dCreateBox(_space, _center_length, _center_width, _center_height );
    dGeomSetBody( _geom[CENTER][0], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - side curve
    _geom[CENTER][1] = dCreateCylinder(_space, _center_radius, _center_width );
    dGeomSetBody( _geom[CENTER][1], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][1], -_center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[CENTER][1], R1);

    // set geometry 3 - side curve
    _geom[CENTER][2] = dCreateCylinder(_space, _center_radius, _center_width );
    dGeomSetBody( _geom[CENTER][2], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][2], _center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[CENTER][2], R1);

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[CENTER], &m);

	// success
	return 0;
}

int CMobot::build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 2700, _end_depth, _end_width, _end_height );
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[id], x, y, z);
    dBodySetRotation(_body[id], R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    _geom[id][0] = dCreateBox(_space, _end_depth, _end_width - 2*_end_radius, _end_height );
    dGeomSetBody( _geom[id][0], _body[id]);
    dGeomSetOffsetPosition( _geom[id][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - left box
    _geom[id][1] = dCreateBox(_space, _end_depth, _end_radius, _end_height - 2*_end_radius );
    dGeomSetBody( _geom[id][1], _body[id]);
    dGeomSetOffsetPosition( _geom[id][1], -m.c[0], -_end_width/2 + _end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 3 - right box
    _geom[id][2] = dCreateBox(_space, _end_depth, _end_radius, _end_height - 2*_end_radius );
    dGeomSetBody( _geom[id][2], _body[id]);
    dGeomSetOffsetPosition( _geom[id][2], -m.c[0], _end_width/2 - _end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 4 - fillet upper left
    _geom[id][3] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][3], _body[id]);
    dGeomSetOffsetPosition( _geom[id][3], -m.c[0], -_end_width/2 + _end_radius - m.c[1], _end_width/2 - _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][3], R1);

    // set geometry 5 - fillet upper right
    _geom[id][4] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][4], _body[id]);
    dGeomSetOffsetPosition( _geom[id][4], -m.c[0], _end_width/2 - _end_radius - m.c[1], _end_width/2 - _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][4], R1);

    // set geometry 6 - fillet lower right
    _geom[id][5] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][5], _body[id]);
    dGeomSetOffsetPosition( _geom[id][5], -m.c[0], _end_width/2 - _end_radius - m.c[1], -_end_width/2 + _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][5], R1);

    // set geometry 7 - fillet lower left
    _geom[id][6] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][6], _body[id]);
    dGeomSetOffsetPosition( _geom[id][6], -m.c[0], -_end_width/2 + _end_radius - m.c[1], -_end_width/2 + _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][6], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CMobot::build_bigwheel(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_connector_depth/3, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _connector_depth/2, _end_width, _connector_height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry
    conn->geom[0] = dCreateCylinder(_space, _bigwheel_radius, 2*_connector_depth/3);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
    dGeomSetOffsetRotation(conn->geom[0], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::build_caster(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[10];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double	depth = _connector_depth,
			width = _end_width,
			height = _connector_height,
			radius = _connector_radius,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 2700, 0.0667, width, height/2);
	//dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width - 2*radius, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], -m.c[0], -width/2 + radius/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], -m.c[0], width/2 - radius/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], -m.c[0], -width/2 + radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[3], R1);

    // set geometry 5 - fillet upper right
    conn->geom[4] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[4], conn->body);
    dGeomSetOffsetPosition(conn->geom[4], -m.c[0], width/2 - radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[4], R1);

    // set geometry 6 - fillet lower right
    conn->geom[5] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[5], conn->body);
    dGeomSetOffsetPosition(conn->geom[5], -m.c[0], width/2 - radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[5], R1);

    // set geometry 7 - fillet lower left
    conn->geom[6] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[6], conn->body);
    dGeomSetOffsetPosition(conn->geom[6], -m.c[0], -width/2 + radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[6], R1);

    // set geometry 8 - horizontal support
	conn->geom[7] = dCreateBox(_space, 0.0667, 0.0222, 0.0032);
	dGeomSetBody(conn->geom[7], conn->body);
	dGeomSetOffsetPosition(conn->geom[7], depth/2 + 0.0667/2 - m.c[0], -m.c[1], -height/2 + 0.0016 - m.c[2]);

    // set geometry 9 - ball support
    conn->geom[8] = dCreateCylinder(_space, 0.0111, 0.0191);
    dGeomSetBody(conn->geom[8], conn->body);
    dGeomSetOffsetPosition(conn->geom[8], depth/2 + 0.0667 - m.c[0], -m.c[1], -height/2 - 0.0064 - m.c[2]);

    // set geometry 10 - sphere
    conn->geom[9] = dCreateSphere(_space, 0.0095);
    dGeomSetBody(conn->geom[9], conn->body);
    dGeomSetOffsetPosition(conn->geom[9], depth/2 + 0.0667 - m.c[0], -m.c[1], -height/2 - 0.0159 - m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::build_simple(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[7];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double	depth = _connector_depth,
			width = _end_width,
			height = _connector_height,
			radius = _connector_radius,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, depth, width, height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width - 2*radius, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], -m.c[0], -width/2 + radius/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], -m.c[0], width/2 - radius/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], -m.c[0], -width/2 + radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[3], R1);

    // set geometry 5 - fillet upper right
    conn->geom[4] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[4], conn->body);
    dGeomSetOffsetPosition(conn->geom[4], -m.c[0], width/2 - radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[4], R1);

    // set geometry 6 - fillet lower right
    conn->geom[5] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[5], conn->body);
    dGeomSetOffsetPosition(conn->geom[5], -m.c[0], width/2 - radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[5], R1);

    // set geometry 7 - fillet lower left
    conn->geom[6] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[6], conn->body);
    dGeomSetOffsetPosition(conn->geom[6], -m.c[0], -width/2 + radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[6], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::build_smallwheel(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_connector_depth/3, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _connector_depth/2, _end_width, _connector_height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry
    conn->geom[0] = dCreateCylinder(_space, _smallwheel_radius, 2*_connector_depth/3);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
    dGeomSetOffsetRotation(conn->geom[0], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::build_square(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[4];

    // define parameters
    dMass m;
    dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _end_width, _end_width, _connector_height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, _connector_depth, _end_width, _connector_height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, _end_width - 2*_connector_depth, _connector_depth, _connector_height);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], _end_width/2 - _connector_depth/2 - m.c[0], -_end_width/2 + _connector_depth/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, _end_width - 2*_connector_depth, _connector_depth, _connector_height);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], _end_width/2 - _connector_depth/2 - m.c[0], _end_width/2 - _connector_depth/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateBox(_space, _connector_depth, _end_width, _connector_height);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], _end_width - _connector_depth - m.c[0], -m.c[1], -m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::build_tank(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

    // define parameters
    dMass m;
    dMatrix3 R;
	double	depth = _tank_depth,
			width = _end_width,
			height = _tank_height,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, depth, width, height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], (_tank_height - _connector_height)/2 - m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CMobot::fix_body_to_connector(dBodyID cBody, int face) {
	if (!cBody) { fprintf(stderr,"connector body does not exist\n"); }

	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	switch (face) {
		case 1:
			dJointAttach(joint, cBody, this->getBodyID(ENDCAP_L));
			break;
		case 2: case 5:
			dJointAttach(joint, cBody, this->getBodyID(BODY_L));
			break;
		case 3: case 6:
			dJointAttach(joint, cBody, this->getBodyID(BODY_L));
			dJointAttach(joint, cBody, this->getBodyID(BODY_R));
			break;
		case 4: case 7:
			dJointAttach(joint, cBody, this->getBodyID(BODY_R));
			break;
		case 8:
			dJointAttach(joint, cBody, this->getBodyID(ENDCAP_R));
			break;
	}

	// set joint params
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);

	// success
	return 0;
}

int CMobot::fix_connector_to_body(int face, dBodyID cBody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);
	dJointID joint2 = dJointCreateFixed(_world, 0);

	// attach to correct body
	switch (face) {
		case 1:
			dJointAttach(joint, this->getBodyID(ENDCAP_L), cBody);
			break;
		case 2: case 5:
			dJointAttach(joint, this->getBodyID(BODY_L), cBody);
			break;
		case 3: case 6:
			dJointAttach(joint, this->getBodyID(BODY_L), cBody);
			dJointAttach(joint2, this->getBodyID(BODY_R), cBody);
			dJointSetFixed(joint2);
			dJointSetFixedParam(joint2, dParamCFM, 0);
			dJointSetFixedParam(joint2, dParamERP, 0.9);
			break;
		case 4: case 7:
			dJointAttach(joint, this->getBodyID(BODY_R), cBody);
			break;
		case 8:
			dJointAttach(joint, this->getBodyID(ENDCAP_R), cBody);
			break;
	}

	// set joint params
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);

	// success
	return 0;
}

int CMobot::get_connector_params(Conn_t *conn, dMatrix3 R, dReal *p) {
	double offset[3] = {0};
	dMatrix3 R1, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};

	switch (conn->type) {
		case SIMPLE:
			offset[0] = _connector_depth;
			dRSetIdentity(R1);
			break;
		/*case BIGWHEEL:
			offset[0] = 2*_connector_depth/3;
			dRSetIdentity(R1);
			break;
		case SMALLWHEEL:
			offset[0] = 2*_connector_depth/3;
			dRSetIdentity(R1);
			break;*/
		case SQUARE:
			if (conn->side == 2) {
				offset[0] = _end_width/2;
				offset[1] = _end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI/2);
			}
			else if (conn->side == 3) {
				offset[0] = _end_width;
				dRSetIdentity(R1);
			}
			else if (conn->side == 4) {
				offset[0] = _end_width/2;
				offset[1] = -_end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], -M_PI/2);
			}
			break;
		case TANK:
			if (conn->side == 2) {
				offset[0] = _tank_depth;
				dRSetIdentity(R1);
			}
			else if (conn->side == 3) {
				offset[0] = _tank_depth/2;
				offset[2] = _tank_height - _connector_height/2;
				dRFromAxisAndAngle(R1, R[1], R[5], R[9], -M_PI/2);
			}
			break;
	}
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int CMobot::init_params(void) {
	// create arrays for mobots
	_body = new dBodyID[NUM_PARTS];
	_joint = new dJointID[6];
	_motor = new dJointID[NUM_DOF];
	_angle = new dReal[NUM_DOF];
	_geom = new dGeomID * [NUM_PARTS];
	_goal = new dReal[NUM_DOF];
	_max_force = new dReal[NUM_DOF];
	_max_speed = new dReal[NUM_DOF];
	_offset = new double[NUM_DOF];
	_rec_active = new bool[NUM_DOF];
	_rec_angles = new double ** [NUM_DOF];
	_rec_num = new int[NUM_DOF];
	_recording = new bool[NUM_DOF];
	_seek = new bool[NUM_DOF];
	_speed = new dReal[NUM_DOF];
	_state = new int[NUM_DOF];
	_success = new int[NUM_DOF];

	// fill with default data
	for (int i = 0; i < NUM_DOF; i++) {
		_angle[i] = 0;
		_goal[i] = 0;
		_seek[i] = false;
		_recording[i] = false;
		_success[i] = 1;
		_state[i] = ROBOT_NEUTRAL;
		_speed[i] = 0.7854;		// 45 deg/sec
		_max_speed[i] = 120;	// deg/sec
	}
	_conn = NULL;
	_id = -1;
	_type = MOBOT;
	_encoder = DEG2RAD(0.5);
	_max_force[ROBOT_JOINT1] = 0.260;
	_max_force[ROBOT_JOINT2] = 1.059;
	_max_force[ROBOT_JOINT3] = 1.059;
	_max_force[ROBOT_JOINT4] = 0.260;
	_safety_angle = 10;
	_safety_timeout = 4;

	// success
	return 0;
}

int CMobot::init_dims(void) {
	_center_length = 0.0516;
	_center_width = 0.0327;
	_center_height = 0.0508;
	_center_radius = 0.0254;
	_center_offset = 0.0149;
	_body_length = 0.0258;
	_body_width = 0.0762;
	_body_height = 0.0508;
	_body_radius = 0.0254;
	_body_inner_width_left = 0.0366;
	_body_inner_width_right = 0.0069;
	_body_end_depth = 0.0352;
	_body_mount_center = 0.0374;
	_end_width = 0.0762;
	_end_height = 0.0762;
	_end_depth = 0.0080;
	_end_radius = 0.0254;
	_connector_depth = 0.0048;
	_connector_height = 0.0413;
	_connector_radius = 0.0064;
	_bigwheel_radius = 0.0571;
	_smallwheel_radius = 0.0445;
	_tank_depth = 0.0413;
	_tank_height = 0.0460;

	// success
	return 0;
}

dReal CMobot::mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate) {
    dReal new_ang = 0;
    int stp = (int)( fabs(past_ang) / M_PI );
    dReal past_ang_mod = fabs(past_ang) - stp*M_PI;

    if ( (int)(ang_rate*1000) == 0 ) {
        new_ang = past_ang;
    }
    // positive angular velocity, positive angle
    else if ( ang_rate > 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang < 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + 2*M_PI); }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // cross 0
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // positive angular velocity, negative angle
    else if ( ang_rate > 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang < 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang + past_ang_mod + M_PI);   }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
    }
    // negative angular velocity, positive angle
    else if ( ang_rate < 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang > 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang - past_ang_mod - M_PI);   }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // cross 0
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // negative angular velocity, negative angle
    else if ( ang_rate < 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang > 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - 2*M_PI); }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
    }

    return new_ang;
}

/*void CMobot::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}*/

#ifdef ENABLE_GRAPHICS
void CMobot::draw_bigwheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _bigwheel_radius, 2*_connector_depth/3);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_caster(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	osg::Sphere *sph;
	double	depth = _connector_depth,
			width = _end_width,
			height = _connector_height,
			radius = _connector_radius;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, width - 2*radius, height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, radius, height - 2*radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, radius, height - 2*radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[3]);
	dGeomGetOffsetQuaternion(conn->geom[3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[4]);
	dGeomGetOffsetQuaternion(conn->geom[4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[5]);
	dGeomGetOffsetQuaternion(conn->geom[5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[6]);
	dGeomGetOffsetQuaternion(conn->geom[6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[7]);
	dGeomGetOffsetQuaternion(conn->geom[7], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0667, 0.0222, 0.0032);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[8]);
	dGeomGetOffsetQuaternion(conn->geom[8], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0111, 0.0191);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[9]);
	dGeomGetOffsetQuaternion(conn->geom[9], quat);
	sph = new osg::Sphere(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0095);
	body->addDrawable(new osg::ShapeDrawable(sph));
    
	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_simple(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	double	depth = _connector_depth,
			width = _end_width,
			height = _connector_height,
			radius = _connector_radius;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, width - 2*radius, height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, radius, height - 2*radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, radius, height - 2*radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[3]);
	dGeomGetOffsetQuaternion(conn->geom[3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[4]);
	dGeomGetOffsetQuaternion(conn->geom[4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[5]);
	dGeomGetOffsetQuaternion(conn->geom[5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[6]);
	dGeomGetOffsetQuaternion(conn->geom[6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), radius, depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_smallwheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _smallwheel_radius, 2*_connector_depth/3);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_square(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;

	// draw geoms
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, _end_width, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_width - 2*_connector_depth, _connector_depth, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_width - 2*_connector_depth, _connector_depth, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[3]);
	dGeomGetOffsetQuaternion(conn->geom[3], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, _end_width, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_tank(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	double	depth = _tank_depth,
			width = _end_width,
			height = _tank_height;

	// draw geoms
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, width, height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}
#endif // ENABLE_GRAPHICS