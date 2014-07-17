#include "mobot.h"

CMobot::CMobot(void) {
	// initialize parameters
	this->initParams(0, MOBOT);

	// initialize dimensions
	this->initDims();
}

CMobot::~CMobot(void) {
	// remove robot from simulation
	if ( g_sim != NULL && !(g_sim->deleteRobot(this)) )
		delete g_sim;

	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}

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

int CMobot::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// add to simulation
	g_sim->addRobot(this);

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

int CMobot::delay(double milliseconds) {
	// set ending time
	double end = g_sim->getClock() + milliseconds/1000;

	// while clock hasn't reached ending time
	while ((end - g_sim->getClock()) >= EPSILON)
		this->doze(50);

	// success
	return 0;
}

int CMobot::delaySeconds(double seconds) {
	// delay milliseconds
	this->delay(1000 * seconds);

	// success
	return 0;
}

int CMobot::disableRecordDataShift(void) {
	_g_shift_data = 0;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CMobot::disconnect(void) {
	// and we are not connected
	_connected = 0;

	// success
	return 0;
}

int CMobot::driveBackward(double angle) {
	this->driveBackwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveBackwardNB(double angle) {
	this->moveNB(-angle, 0, 0, -angle);

	// success
	return 0;
}

int CMobot::driveDistance(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveDistanceNB(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));

	// success
	return 0;
}

int CMobot::driveForeverNB(void) {
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT4);

	// success
	return 0;
}

int CMobot::driveForward(double angle) {
	this->driveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveForwardNB(double angle) {
	this->moveNB(angle, 0, 0, angle);

	// success
	return 0;
}

int CMobot::driveTime(double seconds) {
	this->driveTimeNB(seconds);
	this->moveWait();

	// success
	return 0;
}

int CMobot::driveTimeNB(double seconds) {
	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT4);

	// sleep
	this->doze(seconds * 1000);

	// stop motion
	this->holdJoints();

	// success
	return 0;
}

int CMobot::drivexy(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// move to new global coordinates
	return this->drivexyTo(x + x0, y + y0, radius, trackwidth);
}

void* CMobot::drivexyThread(void *arg) {
	// cast arg
	mobotMoveArg_t *mArg = (mobotMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexy(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CMobot::drivexyNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	mobotMoveArg_t *mArg = new mobotMoveArg_t;
	mArg->robot = this;
	mArg->x = x;
	mArg->y = y;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, drivexyThread, (void *)mArg);

	// success
	return 0;
}

int CMobot::drivexyTo(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// get current rotation
	double r0 = this->getRotation(CENTER, 2);

	// compute rotation matrix for body frame
	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// get angle to turn in body coordinates (transform of R)
	double angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.01) {
		// turn in shortest path
		if (angle > EPSILON)
			this->turnRight(RAD2DEG(angle), radius, trackwidth);
		else if (angle < -EPSILON)
			this->turnLeft(RAD2DEG(-angle), radius, trackwidth);

		// calculate new rotation from error
		this->getxy(x0, y0);
		r0 = this->getRotation(CENTER, 2);
		dRFromAxisAndAngle(R, 0, 0, 1, r0);
		angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));
	}

	// move along length of line
	this->getxy(x0, y0);
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// success
	return 0;
}

void* CMobot::drivexyToThread(void *arg) {
	// cast arg
	mobotMoveArg_t *mArg = (mobotMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyTo(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CMobot::drivexyToNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	mobotMoveArg_t *mArg = new mobotMoveArg_t;
	mArg->robot = this;
	mArg->x = x;
	mArg->y = y;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, drivexyToThread, (void *)mArg);

	// success
	return 0;
}

int CMobot::drivexyWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int CMobot::enableRecordDataShift(void) {
	_g_shift_data = 1;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CMobot::getDistance(double &distance, double radius) {
	double angle;
	this->getJointAngle(JOINT1, angle, 2);
	distance = DEG2RAD(angle) * radius;

	// success
	return 0;
}

int CMobot::getFormFactor(int &formFactor) {
	formFactor = _type;

	// success
	return 0;
}

int CMobot::getJointAngle(robotJointId_t id, double &angle, int numReadings) {
	//initialize variables
	double d;
	angle = 0;

	// get joint angle numReadings times
	for (int i = 0; i < numReadings; i++) {
		if(this->getJointAngleInstant(id, d)) {
			return -1;
		}
		angle += d;
	}

	// store average angle
	angle = angle/numReadings;

	// success
	return 0;
}

int CMobot::getJointAngleInstant(robotJointId_t id, double &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CMobot::getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);
	this->getJointAngle(JOINT3, angle3, numReadings);
	this->getJointAngle(JOINT4, angle4, numReadings);

	// success
	return 0;
}

int CMobot::getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);
	this->getJointAngleInstant(JOINT3, angle3);
	this->getJointAngleInstant(JOINT4, angle4);

	// success
	return 0;
}

int CMobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed) {
	maxSpeed = RAD2DEG(_motor[id].omega_max);

	// success
	return 0;
}

int CMobot::getJointSafetyAngle(double &angle) {
	angle = _motor[JOINT1].safety_angle;

	// success
	return 0;
}

int CMobot::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _motor[JOINT1].safety_timeout;

	// success
	return 0;
}

int CMobot::getJointSpeed(robotJointId_t id, double &speed) {
	speed = RAD2DEG(_motor[id].omega);

	// success
	return 0;
}

int CMobot::getJointSpeedRatio(robotJointId_t id, double &ratio) {
	ratio = _motor[id].omega/_motor[id].omega_max;
	// success
	return 0;
}

int CMobot::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);
	speed3 = RAD2DEG(_motor[JOINT3].omega);
	speed4 = RAD2DEG(_motor[JOINT4].omega);

	// success
	return 0;
}

int CMobot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;
	ratio3 = _motor[JOINT3].omega/_motor[JOINT3].omega_max;
	ratio4 = _motor[JOINT4].omega/_motor[JOINT4].omega_max;

	// success
	return 0;
}

int CMobot::getxy(double &x, double &y) {
	// retrn x and y positions
	x = (g_sim->getUnits()) ? 39.37*this->getCenter(0) : 100*this->getCenter(0);
	y = (g_sim->getUnits()) ? 39.37*this->getCenter(1) : 100*this->getCenter(1);

	// success
	return 0;
}

int CMobot::holdJoint(robotJointId_t id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int CMobot::holdJoints(void) {
	this->setJointSpeed(JOINT1, 0);
	this->setJointSpeed(JOINT2, 0);
	this->setJointSpeed(JOINT3, 0);
	this->setJointSpeed(JOINT4, 0);

	// success
	return 0;
}

int CMobot::holdJointsAtExit(void) {
	// set joint speeds to zero
	this->holdJoints();

	// hold joints still
	this->moveForeverNB();

	// success
	return 0;
}

int CMobot::isConnected(void) {
	return _connected;
}

int CMobot::isMoving(void) {
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].state == POSITIVE || _motor[i].state == NEGATIVE) {
			return 1;
		}
	}

	// success
	return 0;
}

int CMobot::isNotMoving(void) {
	return !(this->isMoving());
}

int CMobot::jumpJointTo(robotJointId_t id, double angle) {
	this->jumpJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::jumpJointToNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CMobot::jumpTo(double angle1, double angle2, double angle3, double angle4) {
	this->jumpToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::jumpToNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CMobot::line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color) {
	return g_sim->line(x1, y1, z1, x2, y2, z2, linewidth, color);
}
#endif // ENABLE_GRAPHICS

int CMobot::motionArch(double angle) {
	this->moveJointToNB(JOINT2, -angle/2.0);
	this->moveJointToNB(JOINT3, angle/2.0);
	this->moveJointWait(JOINT2);
	this->moveJointWait(JOINT3);

	// success
	return 0;
}

void* CMobot::motionArchThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT2, -50);
		this->moveJointTo(JOINT3, 50);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormLeftThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT3, 50);
		this->moveJointTo(JOINT2, -50);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormRightThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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

int CMobot::motionRollForward(double angle) {
	double motorPosition[2];
	this->getJointAngleInstant(JOINT1, motorPosition[0]);
	this->getJointAngleInstant(JOINT4, motorPosition[1]);
	this->moveJointToNB(JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionRollForwardThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->moveJointToNB(JOINT2, angle);
	this->moveJointToNB(JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionSkinnyThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->moveJointTo(JOINT2, -85);
	this->moveJointTo(JOINT3, 70);
	this->moveWait();
	this->moveJointTo(JOINT1, 45);
	this->doze(1000);
	this->moveJointTo(JOINT2, 20);

	// success
	return 0;
}

void* CMobot::motionStandThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->doze(1000);

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT2, -85);
		this->moveJointTo(JOINT3, 75);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 80);
		this->moveJointTo(JOINT2, 45);
		this->moveJointTo(JOINT3, -85);
		this->moveJointTo(JOINT2, 75);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 75);
		if (i != (num-1)) {
			this->moveJointTo(JOINT3, 45);
		}
	}
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleLeftThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->doze(1000);

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT3, 85);
		this->moveJointTo(JOINT2, -80);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, -80);
		this->moveJointTo(JOINT3, -45);
		this->moveJointTo(JOINT2, 85);
		this->moveJointTo(JOINT3, -80);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, -80);
		if (i != (num-1)) {
			this->moveJointTo(JOINT2, -45);
		}
	}
	this->moveJointToNB(JOINT3, 0);
	this->moveJointToNB(JOINT2, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleRightThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	this->moveJointToNB(JOINT3, 45);
	this->moveJointToNB(JOINT2, -85);
	this->moveWait();
	this->resetToZero();

	// success
	return 0;
}

void* CMobot::motionUnstandThread(void *arg) {
	// cast arg
	mobotMotionArg_t *mArg = (mobotMotionArg_t *)arg;

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
	mobotMotionArg_t *mArg = new mobotMotionArg_t;
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
	MUTEX_LOCK(&_theta_mutex);

	// enable motor
	for ( int j = 0; j < _dof; j++ ) {
		MUTEX_LOCK(&_motor[j].success_mutex);
		dJointEnable(_motor[j].id);
		if (_motor[j].omega < -EPSILON) delta[j] = -delta[j];
		_motor[j].goal += DEG2RAD(delta[j]);
		_motor[j].mode = SEEK;
		dJointSetAMotorAngle(_motor[j].id, 0, _motor[j].theta);
		_motor[j].success = false;
		MUTEX_UNLOCK(&_motor[j].success_mutex);
	}
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveForeverNB(void) {
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT2);
	this->moveJointForeverNB(JOINT3);
	this->moveJointForeverNB(JOINT4);

	// success
	return 0;
}

int CMobot::moveJoint(robotJointId_t id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointNB(robotJointId_t id, double angle) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	if (_motor[id].omega < -EPSILON) angle = -angle;
	_motor[id].goal += DEG2RAD(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[CENTER]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveJointForeverNB(robotJointId_t id) {
	// lock mutexes
	MUTEX_LOCK(&_motor[id].success_mutex);

	// enable motor
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	_motor[id].mode = CONTINUOUS;
	if ( _motor[id].omega > EPSILON )
		_motor[id].state = POSITIVE;
	else if ( _motor[id].omega < EPSILON )
		_motor[id].state = NEGATIVE;
	else
		_motor[id].state = HOLD;
	_motor[id].success = true;
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int CMobot::moveJointTime(robotJointId_t id, double seconds) {
	this->moveJointTimeNB(id, seconds);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointTimeNB(robotJointId_t id, double seconds) {
	// move joint
	this->moveJointForeverNB(id);

	// sleep
	this->doze(seconds * 1000);

	// sleep
	this->holdJoint(id);

	// success
	return 0;
}

int CMobot::moveJointTo(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CMobot::moveJointToNB(robotJointId_t id, double angle) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	_motor[id].goal = DEG2RAD(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[CENTER]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CMobot::moveJointWait(robotJointId_t id) {
	// wait for motion to complete
	MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int CMobot::moveTime(double seconds) {
	this->moveTimeNB(seconds);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveTimeNB(double seconds) {
	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT2);
	this->moveJointForeverNB(JOINT3);
	this->moveJointForeverNB(JOINT4);

	// sleep
	this->doze(seconds * 1000);

	// stop motion
	this->holdJoints();

	// success
	return 0;
}

int CMobot::moveTo(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	// store angles into array
	double delta[4] = {	DEG2RAD(angle1) - _motor[JOINT1].theta, DEG2RAD(angle2) - _motor[JOINT2].theta,
						DEG2RAD(angle3) - _motor[JOINT3].theta, DEG2RAD(angle4) - _motor[JOINT4].theta};

	// lock mutexes
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// enable motor
	for (int j = 0; j < _dof; j++) {
		MUTEX_LOCK(&_motor[j].success_mutex);
		dJointEnable(_motor[j].id);
		_motor[j].goal += delta[j];
		_motor[j].mode = SEEK;
		dJointSetAMotorAngle(_motor[j].id, 0, _motor[j].theta);
		_motor[j].success = false;
		MUTEX_UNLOCK(&_motor[j].success_mutex);
	}
    dBodyEnable(_body[CENTER]);

	// unlock mutexes
	MUTEX_UNLOCK(&_theta_mutex);
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
	while ((_motor[JOINT1].success + _motor[JOINT2].success + _motor[JOINT3].success + _motor[JOINT4].success) != _dof) {
		COND_WAIT(&_success_cond, &_success_mutex);
	}
	for (int i = 0; i < _dof; i++) {
		_motor[i].mode = CONTINUOUS;
	}
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CMobot::point(double x, double y, double z, int pointsize, char *color) {
	return g_sim->point(x, y, z, pointsize, color);
}
#endif // ENABLE_GRAPHICS

void* CMobot::recordAngleThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time = 0;
	int time = (int)(g_sim->getClock()*1000);

	// is robot moving
	int *moving = new int[rArg->num];

	// get 'num' data points
	for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = g_sim->getClock()*1000;
		if (i == 0) { start_time = rArg->time[i]; }
		rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angle
		rArg->angle1[i] = RAD2DEG(rArg->robot->_motor[rArg->id].theta);

		// check if joint is moving
		moving[i] = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(rArg->id), dParamVel)*1000);

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rArg->robot->isShiftEnabled()) {
		for (int i = 0; i < rArg->num; i++) {
			if( moving[i] ) {
				shiftTime = rArg->time[i];
				shiftTimeIndex = i;
				break;
			}
		}
		for (int i = 0; i < rArg->num; i++) {
			if (i < shiftTimeIndex) {
				rArg->time[i] = 0;
				rArg->angle1[i] = rArg->angle1[shiftTimeIndex];
			}
			else {
				rArg->time[i] = rArg->time[i] - shiftTime;
			}
		}
	}

	// signal completion of recording
	SIGNAL(&rArg->robot->_recording_cond, &rArg->robot->_recording_mutex, rArg->robot->_recording[rArg->id] = false);

	// cleanup
	delete rArg;
	delete moving;

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
	double start_time = 0;
	int time = (int)(g_sim->getClock()*1000);

	// is robot moving
	int moving;

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
		(*(rArg->pangle1))[i] = RAD2DEG(rArg->robot->_motor[rArg->id].theta);

		// check if joint is moving
		moving = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(rArg->id), dParamVel)*1000);

		// store time of data point
		(*rArg->ptime)[i] = g_sim->getClock()*1000;
		if (i == 0) { start_time = (*rArg->ptime)[i]; }
		(*rArg->ptime)[i] = ((*rArg->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));

		// wait until movement to start recording
		if( !moving && rArg->robot->isShiftEnabled() ) {
			i--;
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
	this->doze(150);

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
    double start_time = 0;
	int time = (int)(g_sim->getClock()*1000);

	// is robot moving
	int *moving = new int[rArg->num];

	// get 'num' data points
    for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = g_sim->getClock()*1000;
        if (i == 0) { start_time = rArg->time[i]; }
        rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angles
		rArg->angle1[i] = RAD2DEG(rArg->robot->_motor[JOINT1].theta);
		rArg->angle2[i] = RAD2DEG(rArg->robot->_motor[JOINT2].theta);
		rArg->angle3[i] = RAD2DEG(rArg->robot->_motor[JOINT3].theta);
		rArg->angle4[i] = RAD2DEG(rArg->robot->_motor[JOINT4].theta);

		// check if joints are moving
		moving[i] = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT1), dParamVel)*1000);
		moving[i] += (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT2), dParamVel)*1000);
		moving[i] += (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT3), dParamVel)*1000);
		moving[i] += (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT4), dParamVel)*1000);

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
    }

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rArg->robot->isShiftEnabled()) {
		for (int i = 0; i < rArg->num; i++) {
			if( moving[i] ) {
				shiftTime = rArg->time[i];
				shiftTimeIndex = i;
				break;
			}
		}
		for (int i = 0; i < rArg->num; i++) {
			if (i < shiftTimeIndex) {
				rArg->time[i] = 0;
				rArg->angle1[i] = rArg->angle1[shiftTimeIndex];
				rArg->angle2[i] = rArg->angle2[shiftTimeIndex];
				rArg->angle3[i] = rArg->angle3[shiftTimeIndex];
			}
			else {
				rArg->time[i] = rArg->time[i] - shiftTime;
			}
		}
	}
	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_recording_mutex);
    for (int i = 0; i < rArg->robot->_dof; i++) {
        rArg->robot->_recording[i] = false;
    }
	COND_SIGNAL(&rArg->robot->_recording_cond);
	MUTEX_UNLOCK(&rArg->robot->_recording_mutex);

	// cleanup
	delete rArg;
	delete moving;

	// success
	return NULL;
}

int CMobot::recordAngles(double *time, double *angle1, double *angle2, double *angle3, double *angle4, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
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
	for (int i = 0; i < _dof; i++) {
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
	double start_time = 0;
	int time = (int)(g_sim->getClock()*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = true;
	rArg->robot->_rec_active[JOINT2] = true;
	rArg->robot->_rec_active[JOINT3] = true;
	rArg->robot->_rec_active[JOINT4] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[rArg->id]; i++) {
		// store locally num of data points taken
		rArg->robot->_rec_num[JOINT1] = i;

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
		(*(rArg->pangle1))[i] = RAD2DEG(rArg->robot->_motor[JOINT1].theta);
		(*(rArg->pangle2))[i] = RAD2DEG(rArg->robot->_motor[JOINT2].theta);
		(*(rArg->pangle3))[i] = RAD2DEG(rArg->robot->_motor[JOINT3].theta);
		(*(rArg->pangle4))[i] = RAD2DEG(rArg->robot->_motor[JOINT4].theta);

		// store time of data point
		(*rArg->ptime)[i] = g_sim->getClock()*1000;
		if (i == 0) { start_time = (*rArg->ptime)[i]; }
		(*rArg->ptime)[i] = ((*rArg->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = false;
	rArg->robot->_rec_active[JOINT2] = false;
	rArg->robot->_rec_active[JOINT3] = false;
	rArg->robot->_rec_active[JOINT4] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, robotRecordData_t &angle4, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
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
	_rec_angles[JOINT1] = &angle1;
	_rec_angles[JOINT2] = &angle2;
	_rec_angles[JOINT3] = &angle3;
	_rec_angles[JOINT4] = &angle4;

	// lock recording for joint id
	for (int i = 0; i < _dof; i++) {
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
	_recording[JOINT1] = 0;
	_recording[JOINT2] = 0;
	_recording[JOINT3] = 0;
	_recording[JOINT4] = 0;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[JOINT1] && _rec_active[JOINT2] && _rec_active[JOINT3] && _rec_active[JOINT4]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[JOINT1];

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
		(*_rec_angles[id])[i] += _distOffset;
	}

	// success
	return 0;
}

int CMobot::recordDistanceOffset(double distance) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// get current rotation
	dMatrix3 R;
	double r0 = this->getRotation(CENTER, 2);
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// calculate y offset from zero in body coordinates
	double y = R[1]*x0 + R[5]*y0;

	// print warning if different from given offset
	if (fabs(y-distance) > 0.01) {
		printf("Warning: Robot position different from the offset specified in recordDistanceOffset(%lf)\n", distance);
	}

	// set offset distance
	_distOffset = distance;

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
		for (int j = 0; j < _dof; j++) {
			(*_rec_angles[j])[i] = DEG2RAD((*_rec_angles[j])[i]) * _radius;
			(*_rec_angles[j])[i] += _distOffset;
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

void* CMobot::recordxyBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	int time = (int)(g_sim->getClock()*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = true;
	rArg->robot->_rec_active[JOINT2] = true;
	rArg->robot->_rec_active[JOINT3] = true;
	rArg->robot->_rec_active[JOINT4] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[JOINT1]; i++) {
		// store locally num of data points taken
		rArg->robot->_rec_num[JOINT1] = i;

		// resize array if filled current one
		if (i >= rArg->num) {
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

		// store positions
		if (rArg->robot->_trace) {
			(*(rArg->ptime))[i] = rArg->robot->getCenter(0);
			(*(rArg->pangle1))[i] = rArg->robot->getCenter(1);
		}
		else {
			i--;
		}

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// signal completion of recording
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = false;
	rArg->robot->_rec_active[JOINT2] = false;
	rArg->robot->_rec_active[JOINT3] = false;
	rArg->robot->_rec_active[JOINT4] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CMobot::recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int recordTrace, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	x = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	y = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	rArg->ptime = &x;
	rArg->pangle1 = &y;

	// store pointer to recorded angles locally
	_rec_angles[JOINT1] = &x;
	_rec_angles[JOINT2] = &y;

	// lock recording for joint id
	for (int i = 0; i < _dof; i++) {
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CMobot::recordxyBeginThread, (void *)rArg);

	// success
	return 0;
}

int CMobot::recordxyEnd(int &num) {
	// sleep to capture last data point on ending time
	this->doze(150);

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[JOINT1] = 0;
	_recording[JOINT2] = 0;
	_recording[JOINT3] = 0;
	_recording[JOINT4] = 0;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[JOINT1] && _rec_active[JOINT2] && _rec_active[JOINT3]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[JOINT1];

	// convert recorded values into in/cm
	double m2x = (g_sim->getUnits()) ? 39.37 : 100;
	for (int i = 0; i < num; i++) {
		(*_rec_angles[JOINT1])[i] = ((*_rec_angles[JOINT1])[i]) * m2x;
		(*_rec_angles[JOINT2])[i] = ((*_rec_angles[JOINT2])[i]) * m2x;
	}

	// success
	return 0;
}

int CMobot::relaxJoint(robotJointId_t id) {
	dJointDisable(_motor[id].id);

	// success
	return 0;
}

int CMobot::relaxJoints(void) {
	dJointDisable(_motor[JOINT1].id);
	dJointDisable(_motor[JOINT2].id);
	dJointDisable(_motor[JOINT3].id);
	dJointDisable(_motor[JOINT4].id);

	// success
	return 0;
}

int CMobot::reset(void) {
	MUTEX_LOCK(&_theta_mutex);
	for (int i = 0; i < _dof; i++) {
		_motor[i].offset = _motor[i].theta;
		_motor[i].theta = 0;
		_motor[i].goal -= _motor[i].offset;
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
	}
	MUTEX_UNLOCK(&_theta_mutex);

	// success
	return 0;
}

int CMobot::resetToZero(void) {
	this->resetToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int CMobot::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_theta_mutex);
	int rev = (int)(_motor[JOINT1].theta/2/M_PI);
	if (rev) _motor[JOINT1].theta -= 2*rev*M_PI;
	rev = (int)(_motor[JOINT4].theta/2/M_PI);
	if (rev) _motor[JOINT4].theta -= 2*rev*M_PI;
	MUTEX_UNLOCK(&_theta_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int CMobot::setJointPower(robotJointId_t id, int power) {
	_motor[id].omega = (power/100.0)*_motor[id].omega_max;

	// success
	return 0;
}

int CMobot::setJointSafetyAngle(double angle) {
	_motor[JOINT1].safety_angle = angle;
	_motor[JOINT2].safety_angle = angle;
	_motor[JOINT3].safety_angle = angle;
	_motor[JOINT4].safety_angle = angle;

	// success
	return 0;
}

int CMobot::setJointSafetyAngleTimeout(double seconds) {
	_motor[JOINT1].safety_timeout = seconds;
	_motor[JOINT2].safety_timeout = seconds;
	_motor[JOINT3].safety_timeout = seconds;
	_motor[JOINT4].safety_timeout = seconds;

	// success
	return 0;
}

int CMobot::setJointSpeed(robotJointId_t id, double speed) {
	if (speed > RAD2DEG(_motor[id].omega_max)) {
		fprintf(stderr, "Warning: Cannot set speed for joint %d to %.2lf degrees/second which is "
			"beyond the maximum limit of %.2lf degrees/second.\n",
			id, speed, RAD2DEG(_motor[id].omega_max));
		_motor[id].omega = _motor[id].omega_max;
	}
	else {
		_motor[id].omega = DEG2RAD(speed);
	}

	// success
	return 0;
}

int CMobot::setJointSpeedRatio(robotJointId_t id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * RAD2DEG(_motor[(int)id].omega_max));
}

int CMobot::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);
	this->setJointSpeed(JOINT4, speed4);

	// success
	return 0;
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);
	this->setJointSpeedRatio(JOINT4, ratio4);

	// success
	return 0;
}

int CMobot::setSpeed(double speed, double radius) {
	this->setJointSpeed(JOINT1, RAD2DEG(speed/radius));
	this->setJointSpeed(JOINT4, RAD2DEG(speed/radius));

	// success
	return 0;
}

int CMobot::stop(void) {
	this->setJointSpeed(JOINT1, 0);
	this->setJointSpeed(JOINT2, 0);
	this->setJointSpeed(JOINT3, 0);
	this->setJointSpeed(JOINT4, 0);

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

int CMobot::systemTime(double &time) {
	// get time
	time = g_sim->getClock();

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CMobot::text(double x, double y, double z, char *text) {
	return g_sim->text(x, y, z, text);
}
#endif // ENABLE_GRAPHICS

int CMobot::traceOff(void) {
	_trace = 0;

	// success
	return 0;
}

int CMobot::traceOn(void) {
#ifdef ENABLE_GRAPHICS
	// show trace
	osg::Geode *trace = dynamic_cast<osg::Geode *>(_robot->getChild(1));
	trace->setNodeMask(VISIBLE_MASK);
#endif // ENABLE_GRAPHICS

	// set trace on
	_trace = 1;

	// success
	return 0;
}

int CMobot::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CMobot::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(-angle, 0, 0, angle);

	// success
	return 0;
}

int CMobot::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CMobot::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(angle, 0, 0, -angle);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CMobot::build(xml_robot_t robot) {
	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta, -cphi*stheta*spsi - sphi*cpsi, -cphi*stheta*cpsi + sphi*spsi, 0,
				  sphi*ctheta, -sphi*stheta*spsi + cphi*cpsi, -sphi*stheta*cpsi - cphi*spsi, 0,
				  stheta, ctheta*spsi, ctheta*cpsi, 0};

	// check for wheels
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->type == BIGWHEEL) {
			robot->z += (_bigwheel_radius - _end_height/2);
			_radius = _bigwheel_radius;
			break;
		}
		else if (ctmp->type == SMALLWHEEL) {
			robot->z += (_smallwheel_radius - _end_height/2);
			_radius = _smallwheel_radius;
			break;
		}
		else if (ctmp->conn == WHEEL) {
			robot->z += (ctmp->size - _body_height/2);
			_radius = ctmp->size;
			if (fabs(robot->z) > (_body_radius-EPSILON)) {robot->z += _body_height/2; }
			break;
		}
		ctmp = ctmp->next;
	}

	// build robot
	double rot[4] = {robot->angle1, robot->angle2, robot->angle3, robot->angle4};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// add connectors
	ctmp = robot->conn;
	while (ctmp) {
		if ( ctmp->robot == _id )
			this->add_connector(ctmp->type, ctmp->face1, ctmp->size);
		ctmp = ctmp->next;
	}

	// set trackwidth
	double wheel[4] = {0};
	const double *pos;
	int i = 0;
	conn_t c2tmp = _conn;
	while (c2tmp) {
		switch (c2tmp->type) {
			case BIGWHEEL:
			case SMALLWHEEL:
			case TINYWHEEL:
			case WHEEL:
				pos = dBodyGetPosition(c2tmp->body);
				wheel[i++] = pos[0];
				wheel[i++] = pos[1];
				break;
			default:
				break;
		}
		c2tmp = c2tmp->next;
	}
	_trackwidth = sqrt(pow(wheel[0] - wheel[2], 2) + pow(wheel[1] - wheel[3], 2));

	// success
	return 0;
}

int CMobot::build(xml_robot_t robot, CRobot *base, xml_conn_t conn) {
	// build robot
	this->build_attached(robot, base, conn);

	// add connectors
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		if ( ctmp->robot == _id )
			this->add_connector(ctmp->type, ctmp->face1, ctmp->size);
		ctmp = ctmp->next;
	}

	// success
	return 0;
}

int CMobot::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
	_geom[ENDCAP_L] = new dGeomID[7];
	_geom[BODY_L] = new dGeomID[5];
	_geom[CENTER] = new dGeomID[3];
	_geom[BODY_R] = new dGeomID[5];
	_geom[ENDCAP_R] = new dGeomID[7];

	// adjust input height by body height
	if (z < _end_height/2) {
		x += R[2]*_end_height/2;
		y += R[6]*_end_height/2;
		z += R[10]*_end_height/2;
	}

	// convert input angles to radians
	_motor[JOINT1].theta = DEG2RAD(rot[JOINT1]);	// left end
	_motor[JOINT2].theta = DEG2RAD(rot[JOINT4]);	// left body
	_motor[JOINT3].theta = DEG2RAD(rot[JOINT4]);	// right body
	_motor[JOINT4].theta = DEG2RAD(rot[JOINT4]);	// right end

	// offset values for each body part[0-2] and joint[3-5] from center
	double le[6] = {-_body_radius - _body_length - _body_end_depth - _end_depth/2, 0, 0, -_body_radius/2 - _body_length - _body_end_depth, 0, 0};
	double lb[6] = {-_body_radius - _body_length - _body_end_depth/2, 0, 0, -_center_length/2, _center_width/2, 0};
	double ce[3] = {0, _center_offset, 0};
	double rb[6] = {_body_radius + _body_length + _body_end_depth/2, 0, 0, _center_length/2, _center_width/2, 0};
	double re[6] = {_body_radius + _body_length + _body_end_depth + _end_depth/2, 0, 0, _body_radius/2 + _body_length + _body_end_depth, 0, 0};

	// build robot bodies
	this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
	this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
	this->build_center(R[1]*ce[1] + x, R[5]*ce[1] + y, R[9]*ce[1] + z, R);
	this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
	this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = -0.0149;
	_center[2] = 0;

	// joint for left endcap to body
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY_L], _body[ENDCAP_L]);
	dJointSetHingeAnchor(_joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x,
									R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y,
									R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);

	// joint for center to left body 1
	_joint[1] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[1], _body[CENTER], _body[BODY_L]);
	dJointSetHingeAnchor(_joint[1], R[0]*lb[3] + R[1]*(_center_offset+lb[4]) + R[2]*lb[5] + x,
									R[4]*lb[3] + R[5]*(_center_offset+lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset+lb[4]) + R[10]*lb[5] + z);
	dJointSetHingeAxis(_joint[1], -R[1], -R[5], -R[9]);

	// joint for center to left body 2
	_joint[4] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[4], _body[CENTER], _body[BODY_L]);
	dJointSetHingeAnchor(_joint[4], R[0]*lb[3] + R[1]*(_center_offset-lb[4]) + R[2]*lb[5] + x,
									R[4]*lb[3] + R[5]*(_center_offset-lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset-lb[4]) + R[10]*lb[5] + z);
	dJointSetHingeAxis(_joint[4], R[1], R[5], R[9]);

	// joint for center to right body 1
	_joint[2] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[2], _body[CENTER], _body[BODY_R]);
	dJointSetHingeAnchor(_joint[2], R[0]*rb[3] + R[1]*(_center_offset+rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset+rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset+rb[4]) + R[10]*rb[5] + z);
	dJointSetHingeAxis(_joint[2], -R[1], -R[5], -R[9]);

	// joint for center to right body 2
	_joint[5] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[5], _body[CENTER], _body[BODY_R]);
	dJointSetHingeAnchor(_joint[5], R[0]*rb[3] + R[1]*(_center_offset-rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset-rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset-rb[4]) + R[10]*rb[5] + z);
	dJointSetHingeAxis(_joint[5], R[1], R[5], R[9]);

	// joint for right body to endcap
	_joint[3] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[3], _body[BODY_R], _body[ENDCAP_R]);
	dJointSetHingeAnchor(_joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x,
									R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y,
									R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
	dJointSetHingeAxis(_joint[3], R[0], R[4], R[8]);

	// create rotation matrices for each body part
	dMatrix3 R_e, R_b, R_le, R_lb, R_rb, R_re;
	dRFromAxisAndAngle(R_b, 0, 1, 0, _motor[JOINT1].theta);
	dMultiply0(R_lb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, -1, 0, 0, _motor[JOINT2].theta);
	dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
	dRFromAxisAndAngle(R_b, 0, 1, 0, _motor[JOINT3].theta);
	dMultiply0(R_rb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, -1, 0, 0, _motor[JOINT4].theta);
	dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

	// if bodies are rotated, then redraw
	if (_motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 || _motor[JOINT3].theta != 0 || _motor[JOINT4].theta != 0 ) {
		// offset values from center of robot
		double le_r[3] = {-_body_radius - (_body_length + _body_end_depth + _end_depth/2)*cos(_motor[JOINT2].theta), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_motor[JOINT2].theta)};
		double lb_r[3] = {-_body_radius - (_body_length + _body_end_depth/2)*cos(_motor[JOINT2].theta), 0, (_body_length + _body_end_depth/2)*sin(_motor[JOINT2].theta)};
		double rb_r[3] = {_body_radius + (_body_length + _body_end_depth/2)*cos(_motor[JOINT3].theta), 0, (_body_length + _body_end_depth/2)*sin(_motor[JOINT3].theta)};
		double re_r[3] = {_body_radius + (_body_length + _body_end_depth + _end_depth/2)*cos(_motor[JOINT3].theta), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_motor[JOINT3].theta)};
		// re-build pieces of module
		this->build_endcap(ENDCAP_L, R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
		this->build_body(BODY_L, R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, rot[JOINT2]);
		this->build_body(BODY_R, R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, rot[JOINT3]);
		this->build_endcap(ENDCAP_R, R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);
	}

	// motor for left endcap to body
	_motor[JOINT1].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT1].id, _body[BODY_L], _body[ENDCAP_L]);
	dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
	dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R_lb[0], R_lb[4], R_lb[8]);
	dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
	dJointDisable(_motor[JOINT1].id);

	// motor for center to left body
	_motor[JOINT2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT2].id, _body[CENTER], _body[BODY_L]);
	dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
	dJointDisable(_motor[JOINT2].id);

	// motor for center to right body
	_motor[JOINT3].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT3].id, _body[CENTER], _body[BODY_R]);
	dJointSetAMotorMode(_motor[JOINT3].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT3].id, 1);
	dJointSetAMotorAxis(_motor[JOINT3].id, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(_motor[JOINT3].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT3].id, dParamFMax, _motor[JOINT3].tau_max);
	dJointDisable(_motor[JOINT3].id);

	// motor for right body to endcap
	_motor[JOINT4].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT4].id, _body[BODY_R], _body[ENDCAP_R]);
	dJointSetAMotorMode(_motor[JOINT4].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT4].id, 1);
	dJointSetAMotorAxis(_motor[JOINT4].id, 0, 1, R_rb[0], R_rb[4], R_rb[8]);
	dJointSetAMotorAngle(_motor[JOINT4].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT4].id, dParamFMax, _motor[JOINT4].tau_max);
	dJointDisable(_motor[JOINT4].id);

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CMobot::draw(osg::Group *root, int tracking) {
	// initialize variables
	osg::Group *_robot = new osg::Group();
	osg::ref_ptr<osg::Geode> body[NUM_PARTS];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[NUM_PARTS];
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < NUM_PARTS; i++) {
		body[i] = new osg::Geode;
	}

	// left endcap
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][0]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_width - 2*_end_radius, _end_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][1]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][2]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _end_depth, _end_radius, _end_height - 2*_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][3]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][4]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][5]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(_geom[ENDCAP_L][6]);
	dGeomGetOffsetQuaternion(_geom[ENDCAP_L][6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _end_radius, _end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));

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
	{ // 'led'
		cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]+0.0001), 0.01, _body_height);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		_led = new osg::ShapeDrawable(cyl);
		body[BODY_R]->addDrawable(_led);
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1));
	}
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
    body[ENDCAP_L]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
    body[BODY_L]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
    body[CENTER]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
    body[BODY_R]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
    body[ENDCAP_R]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// set rendering properties
	for (int i = 0; i < NUM_PARTS; i++) {
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// position each body within robot
	for (int i = 0; i < NUM_PARTS; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robot->addChild(pat[i].get());
	}

	// add connectors
	conn_t ctmp = _conn;
	while (ctmp) {
		switch (ctmp->type) {
			case BIGWHEEL:
				this->draw_bigwheel(ctmp, _robot);
				break;
			case CASTER:
				this->draw_caster(ctmp, _robot);
				break;
			case SIMPLE:
				this->draw_simple(ctmp, _robot);
				break;
			case SMALLWHEEL:
				this->draw_smallwheel(ctmp, _robot);
				break;
			case SQUARE:
				this->draw_square(ctmp, _robot);
				break;
			case TANK:
				this->draw_tank(ctmp, _robot);
				break;
			case WHEEL:
				this->draw_wheel(ctmp, _robot);
				break;
		}
		ctmp = ctmp->next;
	}

	// set update callback for robot
	_robot->setUpdateCallback(new mobotNodeCallback(this));

	// set shadow mask
	//robot->setNodeMask(CASTS_SHADOW_MASK);
	//robot->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	osgText::Text *label = new osgText::Text();
	osg::Geode *label_geode = new osg::Geode();
	label_geode->addDrawable(label);
	label_geode->setNodeMask(NOT_VISIBLE_MASK);
	label_geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	label_geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	label_geode->getOrCreateStateSet()->setRenderBinDetails(22, "RenderBin");
	label_geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	label->setAlignment(osgText::Text::CENTER_CENTER);
	label->setAxisAlignment(osgText::Text::SCREEN);
	label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	label->setCharacterSize(30);
	label->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	label->setDrawMode(osgText::Text::TEXT | osgText::Text::ALIGNMENT | osgText::Text::BOUNDINGBOX);
	_robot->insertChild(0, label_geode);

	// draw tracking node
	_trace = tracking;
	osg::Geode *trackingGeode = new osg::Geode();
	osg::Geometry *trackingLine = new osg::Geometry();
	osg::Vec3Array *trackingVertices = new osg::Vec3Array();
	trackingGeode->setNodeMask((tracking) ? VISIBLE_MASK : NOT_VISIBLE_MASK);
	trackingLine->setVertexArray(trackingVertices);
	trackingLine->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1, 1));
	trackingLine->setDataVariance(osg::Object::DYNAMIC);
	trackingLine->setUseDisplayList(false);
	osg::Vec4Array *colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );	// green
	trackingLine->setColorArray(colors);
	trackingLine->setColorBinding(osg::Geometry::BIND_OVERALL);
	osg::Point *point = new osg::Point();
	point->setSize(4.0f);
	trackingGeode->getOrCreateStateSet()->setAttributeAndModes(point, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	trackingGeode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	trackingGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	trackingGeode->addDrawable(trackingLine);
	_robot->insertChild(1, trackingGeode);

	// set user properties of node
	_robot->setName("robot");

	// optimize robot
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_robot);

	// add to scenegraph
	root->addChild(_robot);

	// return position of robot in root node
	return (root->getChildIndex(_robot));
}
#endif // ENABLE_GRAPHICS

int CMobot::getConnectionParams(int face, dMatrix3 R, double *p) {
	const double *pos, *R1;
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

int CMobot::initParams(int disabled, int type) {
	_dof = 4;

	// create arrays for mobots
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[2];
	_geom = new dGeomID * [NUM_PARTS];
	_joint = new dJointID[6];
	_motor = new struct motor_s[_dof];
	_rec_active = new bool[_dof];
	_rec_angles = new double ** [_dof];
	_rec_num = new int[_dof];
	_recording = new bool[_dof];

	// fill with default data
	for (int i = 0; i < _dof; i++) {
		_motor[i].alpha = 0;
		_motor[i].encoder = DEG2RAD(0.1);
		_motor[i].goal = 0;
		_motor[i].mode = SEEK;
		_motor[i].offset = 0;
		_motor[i].omega = 0.7854;			//  45 deg/sec
		_motor[i].omega_max = 2.0943;		// 120 deg/sec
		_motor[i].safety_angle = 10;
		_motor[i].safety_timeout = 4;
		_motor[i].state = NEUTRAL;
		_motor[i].success = true;
		_motor[i].theta = 0;
		_motor[i].timeout = 0;
		MUTEX_INIT(&_motor[i].success_mutex);
		COND_INIT(&_motor[i].success_cond);
		_rec_active[i] = false;
		_rec_num[i] = 0;
		_recording[i] = false;
	}
	_conn = NULL;
	_disabled = disabled;
	_distOffset = 0;
	_id = -1;
	_motor[JOINT1].tau_max = 0.260;
	_motor[JOINT2].tau_max = 1.059;
	_motor[JOINT3].tau_max = 1.059;
	_motor[JOINT4].tau_max = 0.260;
	_motion = false;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 1;
	_shift_data = 0;
	_g_shift_data = 0;
	_g_shift_data_en = 0;
	_trace = 1;
	_type = type;

	// success
	return 0;
}

int CMobot::initDims(void) {
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
	_wheel_radius = 0.0445;

	// success
	return 0;
}

void CMobot::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[CENTER]);
	// put into accel array
	_accel[0] = R[8];
	_accel[1] = R[9];
	_accel[2] = R[10];
	// add gaussian noise to accel
	this->noisy(_accel, 3, 0.005);

	// update angle values for each degree of freedom
	for (int i = 0; i < _dof; i++) {
		// store current angle
		_motor[i].theta = getAngle(i);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
		// drive motor to get current angle to match future angle
		switch (_motor[i].mode) {
			case CONTINUOUS:
				switch (_motor[i].state) {
					case POSITIVE:
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
						break;
					case NEGATIVE:
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
						break;
					case HOLD:
						dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
						break;
					case NEUTRAL:
						dJointDisable(_motor[i].id);
						break;
				}
				break;
			case SEEK:
				if (_motor[i].theta < _motor[i].goal - _motor[i].encoder) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
				}
				else if (_motor[i].theta > _motor[i].goal + _motor[i].encoder) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
				}
				else {
					_motor[i].state = HOLD;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
				}
				break;
		}
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void CMobot::simPostCollisionThread(void) {
	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < _dof; i++) {
		// lock mutex
		MUTEX_LOCK(&_motor[i].success_mutex);
		// zero velocity == stopped
		_motor[i].success = (!(int)(dJointGetAMotorParam(this->getMotorID(i), dParamVel)*1000) );
		// signal success
		if (_motor[i].success)
			COND_SIGNAL(&_motor[i].success_cond);
		// unlock mutex
		MUTEX_UNLOCK(&_motor[i].success_mutex);
	}

	if (_motor[JOINT1].success && _motor[JOINT2].success && _motor[JOINT3].success && _motor[JOINT4].success)
		COND_SIGNAL(&_success_cond);
}

/**********************************************************
	private functions
 **********************************************************/
int CMobot::add_connector(int type, int face, double size) {
	// create new connector
	conn_t nc = new struct conn_s;
	nc->d_side = -1;
	nc->d_type = -1;
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
		case WHEEL:
			this->build_wheel(nc, face, size);
			break;
	}

	// success
	return 0;
}

int CMobot::build_attached(xml_robot_t robot, CRobot *base, xml_conn_t conn) {
	// initialize new variables
	int i = 1;
	double m[3] = {0}, offset[3] = {0};
	dMatrix3 R, R1, R2, R3, R4, R5, R6;

	// generate parameters for base robot
	base->getConnectionParams(conn->face1, R, m);

	// generate parameters for connector
	this->getConnectorParams(conn->type, conn->side, R, m);

	// collect data from struct
	double r_le = robot->angle1;
	double r_lb = robot->angle2;
	double r_rb = robot->angle3;
	double r_re = robot->angle4;

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
	double rot[4] = {r_le, r_lb, r_rb, r_re};
	this->buildIndividual(m[0], m[1], m[2], R6, rot);

    // add fixed joint to attach two modules
	this->fix_body_to_connector(base->getConnectorBodyID(conn->face1), conn->face2);

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

int CMobot::build_body(int id, double x, double y, double z, dMatrix3 R, double theta) {
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
	dMassSetBox(&m, 500, 0.0667, width, height/2);

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

int CMobot::build_center(double x, double y, double z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetZero(&m);
    dMassSetCapsule(&m, 2700, 1, _center_radius, _center_length );
    dMassAdjust(&m, 0.24);

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

int CMobot::build_endcap(int id, double x, double y, double z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 2700, _end_depth, _end_width, _end_height );

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

int CMobot::build_wheel(conn_t conn, int face, double size) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

	// store wheel radius
	_wheel_radius = size;

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
    conn->geom[0] = dCreateCylinder(_space, _wheel_radius, 2*_connector_depth/3);
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

#ifdef ENABLE_GRAPHICS
void CMobot::draw_bigwheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

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
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

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
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

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
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

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
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

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
	const double *pos;
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

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}

void CMobot::draw_wheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _wheel_radius, 2*_connector_depth/3);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(mobot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add body to pat
	pat->addChild(body.get());
	// optimize
	osgUtil::Optimizer optimizer;
	optimizer.optimize(pat);
	// add to scenegraph
	robot->addChild(pat);
}
#endif // ENABLE_GRAPHICS

int CMobot::fix_body_to_connector(dBodyID cBody, int face) {
	if (!cBody) { fprintf(stderr, "Error: connector body does not exist\n"); exit(-1); }

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

	// success
	return 0;
}

