#include "linkbotsim.h"

CLinkbotT::CLinkbotT(int disabled, int type) {
	// initialize parameters
	init_params(disabled, type);

	// initialize dimensions
	init_dims();
}

CLinkbotT::~CLinkbotT(void) {
	// remove robot from simulation
	if ( _simObject != NULL && !(_simObject->deleteRobot(this)) )
		delete _simObject;

	// destroy geoms
	if (_connected) {
		for (int i = NUM_PARTS - 1; i >= 0; i--) { delete [] _geom[i]; }
	}
}

int CLinkbotT::blinkLED(double delay, int num) {
#ifdef ENABLE_GRAPHICS
	// blink num-1 full times
	for (int i = 0; i < num-1; i++) {
		_led->setColor(osg::Vec4(1, 1, 1, 1));
#ifdef _WIN32
		Sleep(delay);
#else
		usleep(delay*1000);
#endif
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#ifdef _WIN32
		Sleep(delay);
#else
		usleep(delay*1000);
#endif
	}

	// one last off before resetting to original color
	_led->setColor(osg::Vec4(1, 1, 1, 1));
#ifdef _WIN32
	Sleep(delay);
#else
	usleep(delay*1000);
#endif
	_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#endif // ENABLE_GRAPHICS
	// success
	return 0;
}

int CLinkbotT::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!_simObject)
		_simObject = new RoboSim(name, pause);

	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// add to simulation
	_simObject->addRobot(this);

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

int CLinkbotT::delay(double milliseconds) {
	// set ending time
	double end = *_clock + milliseconds/1000;

	// while clock hasn't reached ending time
	while ((end - *_clock) > EPSILON) {
#ifdef _WIN32
		Sleep(5);
#else
		usleep(5000);
#endif
	}

	// success
	return 0;
}

int CLinkbotT::delaySeconds(double seconds) {
	// delay milliseconds
	this->delay(1000 * seconds);

	// success
	return 0;
}

int CLinkbotT::disableRecordDataShift() {
	_g_shift_data = 0;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CLinkbotT::disconnect(void) {
	// and we are not connected
	_connected = 0;

	// success
	return 0;
}

int CLinkbotT::driveJointTo(robotJointId_t id, double angle) {
	this->driveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbotT::driveJointToDirect(robotJointId_t id, double angle) {
	this->driveJointToDirectNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbotT::driveJointToDirectNB(robotJointId_t id, double angle) {
	this->moveJointToDirectNB(id, angle);

	// success
	return 0;
}

int CLinkbotT::driveJointToNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CLinkbotT::driveTo(double angle1, double angle2, double angle3) {
	this->driveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::driveToDirect(double angle1, double angle2, double angle3) {
	this->driveToDirectNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::driveToDirectNB(double angle1, double angle2, double angle3) {
	this->moveToDirectNB(angle1, angle2, angle3);

	// success
	return 0;
}

int CLinkbotT::driveToNB(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);

	// success
	return 0;
}

int CLinkbotT::enableRecordDataShift() {
	_g_shift_data = 1;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CLinkbotT::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z) {
	// output current accel data
	accel_x = _accel[0];
	accel_y = _accel[1];
	accel_z = _accel[2];

	// success
	return 0;
}

int CLinkbotT::getBatteryVoltage(double &voltage) {
	voltage = 100;

	// success
	return 0;
}

int CLinkbotT::getColorName(char color[]) {
	return this->getLEDColorName(color);
}

int CLinkbotT::getColorRGB(int &r, int &g, int &b) {
	return this->getLEDColorRGB(r, g, b);
}

int CLinkbotT::getLEDColorName(char color[]) {
	rgbHashTable *rgbTable = HT_Create();
	int getRGB[3] = {(int)(255*_rgb[0]), (int)(255*_rgb[1]), (int)(255*_rgb[2])};
	int retval = HT_GetKey(rgbTable, getRGB, color);
	HT_Destroy(rgbTable);

	// success
	return retval;
}

int CLinkbotT::getLEDColorRGB(int &r, int &g, int &b) {
	r = (int)(255*_rgb[0]);
	g = (int)(255*_rgb[1]);
	b = (int)(255*_rgb[2]);

	// success
	return 0;
}

int CLinkbotT::getDistance(double &distance, double radius) {
	double angle;
	this->getJointAngleAverage(JOINT1, angle, 2);
	distance = DEG2RAD(angle) * radius;

	// success
	return 0;
}

int CLinkbotT::getFormFactor(int &formFactor) {
	formFactor = _type;

	// success
	return 0;
}

int CLinkbotT::getID(void) {
	return _id;
}

int CLinkbotT::getJointAngle(robotJointId_t id, double &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CLinkbotT::getJointAngleAverage(robotJointId_t id, double &angle, int numReadings) {
	//initialize variables
	double d;
	angle = 0;

	// get joint angle numReadings times
	for (int i = 0; i < numReadings; i++) {
		if (this->getJointAngle(id, d)) {
			return -1;
		}
		angle += d;
	}

	// store average angle
	angle = angle/numReadings;

	// success
	return 0;
}

int CLinkbotT::getJointAngles(double &angle1, double &angle2, double &angle3) {
	this->getJointAngle(JOINT1, angle1);
	this->getJointAngle(JOINT2, angle2);
	this->getJointAngle(JOINT3, angle3);

	// success
	return 0;
}

int CLinkbotT::getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngleAverage(JOINT1, angle1, numReadings);
	this->getJointAngleAverage(JOINT2, angle2, numReadings);
	this->getJointAngleAverage(JOINT3, angle3, numReadings);

	// success
	return 0;
}

int CLinkbotT::getJointMaxSpeed(robotJointId_t id, double &maxSpeed) {
	maxSpeed = _max_speed[id];

	// success
	return 0;
}

int CLinkbotT::getJointSafetyAngle(double &angle) {
	angle = _safety_angle;

	// success
	return 0;
}

int CLinkbotT::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _safety_timeout;

	// success
	return 0;
}

int CLinkbotT::getJointSpeed(robotJointId_t id, double &speed) {
	speed = RAD2DEG(_speed[id]);

	// success
	return 0;
}

int CLinkbotT::getJointSpeedRatio(robotJointId_t id, double &ratio) {
	ratio = _speed[id]/DEG2RAD(_max_speed[id]);

	// success
	return 0;
}

int CLinkbotT::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	speed1 = RAD2DEG(_speed[0]);
	speed2 = RAD2DEG(_speed[1]);
	speed3 = RAD2DEG(_speed[2]);

	// success
	return 0;
}

int CLinkbotT::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	ratio1 = _speed[0]/DEG2RAD(_max_speed[0]);
	ratio2 = _speed[1]/DEG2RAD(_max_speed[1]);
	ratio3 = _speed[2]/DEG2RAD(_max_speed[2]);

	// success
	return 0;
}

int CLinkbotT::getJointState(robotJointId_t id, robotJointState_t &state) {
	state = (robotJointState_t)(_state[id]);

	// success
	return 0;
}

int CLinkbotT::getxy(double &x, double &y) {
	// retrn x and y positions
	x = (_simObject->getUnits()) ? 39.37*this->getCenter(0) : 100*this->getCenter(0);
	y = (_simObject->getUnits()) ? 39.37*this->getCenter(1) : 100*this->getCenter(1);

	// success
	return 0;
}

int CLinkbotT::isConnected(void) {
	return _connected;
}

int CLinkbotT::isMoving(void) {
	robotJointState_t state;

	for (int i = 0; i < NUM_DOF; i++) {
		this->getJointState((robotJointId_t)i, state);
		if (state == ROBOT_FORWARD || state == ROBOT_BACKWARD) {
			return 1;
		}
	}

	return 0;
}

int CLinkbotT::isNotMoving(void) {
	return !(this->isMoving());
}

#ifdef ENABLE_GRAPHICS
int CLinkbotT::line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color) {
	return _simObject->line(x1, y1, z1, x2, y2, z2, linewidth, color);
}
#endif // ENABLE_GRAPHICS

int CLinkbotT::motionDistance(double distance, double radius) {
	this->motionRollForward(distance/radius);

	// success
	return 0;
}

int CLinkbotT::motionDistanceNB(double distance, double radius) {
	this->motionRollForwardNB(distance/radius);

	// success
	return 0;
}

int CLinkbotT::motionRollBackward(double angle) {
	this->move(-angle, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CLinkbotT::motionRollBackwardThread(void *arg) {
	// cast arg
	linkbotMotionArg_t *mArg = (linkbotMotionArg_t *)arg;

	// perform motion
	mArg->robot->motionRollBackward(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::motionRollBackwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	linkbotMotionArg_t *mArg = new linkbotMotionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollBackwardThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::motionRollForward(double angle) {
	this->move(angle, 0, angle);
	this->moveWait();

	// success
	return 0;
}

void* CLinkbotT::motionRollForwardThread(void *arg) {
	// cast arg
	linkbotMotionArg_t *mArg = (linkbotMotionArg_t *)arg;

	// perform motion
	mArg->robot->motionRollForward(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::motionRollForwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	linkbotMotionArg_t *mArg = new linkbotMotionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollForwardThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::motionTurnLeft(double angle) {
	this->move(-angle, 0, angle);
	this->moveWait();

	// success
	return 0;
}

void* CLinkbotT::motionTurnLeftThread(void *arg) {
	// cast arg
	linkbotMotionArg_t *mArg = (linkbotMotionArg_t *)arg;

	// perform motion
	mArg->robot->motionTurnLeft(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::motionTurnLeftNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	linkbotMotionArg_t *mArg = new linkbotMotionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnLeftThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::motionTurnRight(double angle) {
	this->move(angle, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CLinkbotT::motionTurnRightThread(void *arg) {
	// cast arg
	linkbotMotionArg_t *mArg = (linkbotMotionArg_t *)arg;

	// perform motion
	mArg->robot->motionTurnRight(mArg->d);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::motionTurnRightNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	linkbotMotionArg_t *mArg = new linkbotMotionArg_t;
	mArg->robot = this;
	mArg->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnRightThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::motionWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int CLinkbotT::move(double angle1, double angle2, double angle3) {
	this->moveNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveNB(double angle1, double angle2, double angle3) {
	// store angles into array
	double delta[NUM_DOF] = {angle1, angle2, angle3};

	// lock mutexes
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// loop over joints
	for (int i = 0; i < ((_disabled == -1) ? 3 : 2); i++) {
		int j = _enabled[i];
		_goal[j] += DEG2RAD(delta[j]);
		_seek[j] = true;
		dJointEnable(_motor[j]);
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
		_success[j] = false;
	}

	// enable body
    dBodyEnable(_body[BODY]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::moveBackward(double angle) {
	this->moveBackwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveBackwardNB(double angle) {
	return this->moveNB(-angle, 0, angle);
}

int CLinkbotT::moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3) {
	return this->setMovementStateNB(dir1, dir2, dir3);
}

int CLinkbotT::moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds) {
	return this->setMovementStateTime(dir1, dir2, dir3, seconds);
}

int CLinkbotT::moveDistance(double distance, double radius) {
	return this->moveForward(RAD2DEG(distance/radius));
}

int CLinkbotT::moveDistanceNB(double distance, double radius) {
	return this->moveForwardNB(RAD2DEG(distance/radius));
}

int CLinkbotT::moveForward(double angle) {
	this->moveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveForwardNB(double angle) {
	return this->moveNB(angle, 0, -angle);
}

int CLinkbotT::moveJoint(robotJointId_t id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbotT::moveJointContinuousNB(robotJointId_t id, robotJointState_t dir) {
	return this->setJointMovementStateNB(id, dir);
}

int CLinkbotT::moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	return this->setJointMovementStateTime(id, dir, seconds);
}

int CLinkbotT::moveJointNB(robotJointId_t id, double angle) {
	// check if disabled joint
	if (_disabled == id) return 0;

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
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = false;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::moveJointTo(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbotT::moveJointToDirect(robotJointId_t id, double angle) {
	this->moveJointToDirectNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbotT::moveJointToDirectNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int CLinkbotT::moveJointToNB(robotJointId_t id, double angle) {
	// check if disabled joint
	if (_disabled == id) return 0;

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
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = false;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::moveJointWait(robotJointId_t id) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ( !_success[id] ) { COND_WAIT(&_success_cond, &_success_mutex); }
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CLinkbotT::moveTo(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveToDirect(double angle1, double angle2, double angle3) {
	this->moveToDirectNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveToDirectNB(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);

	// success
	return 0;
}

int CLinkbotT::moveToNB(double angle1, double angle2, double angle3) {
	// store angles into array
	double delta[3] = {DEG2RAD(angle1) - _angle[0], DEG2RAD(angle2) - _angle[1], DEG2RAD(angle3) - _angle[2]};

	// lock mutexes
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// loop over joints
	for (int i = 0; i < ((_disabled == -1) ? 3 : 2); i++) {
		int j = _enabled[i];
		_goal[j] += delta[j];
		_seek[j] = true;
		dJointEnable(_motor[j]);
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
		_success[j] = false;
	}

	// enable body
    dBodyEnable(_body[BODY]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::moveToZero(void) {
	this->moveTo(0, 0, 0);

	// success
	return 0;
}

int CLinkbotT::moveToZeroNB(void) {
	this->moveToNB(0, 0, 0);

	// success
	return 0;
}

int CLinkbotT::moveWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ((_success[0] + _success[1] + _success[2]) != NUM_DOF) {
		COND_WAIT(&_success_cond, &_success_mutex);
	}
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CLinkbotT::movexy(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// move to new global coordinates
	return this->movexyTo(x + x0, y + y0, radius, trackwidth);
}

void* CLinkbotT::movexyThread(void *arg) {
	// cast arg
	linkbotMoveArg_t *mArg = (linkbotMoveArg_t *)arg;

	// perform motion
	mArg->robot->movexy(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::movexyNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	linkbotMoveArg_t *mArg = new linkbotMoveArg_t;
	mArg->robot = this;
	mArg->x = x;
	mArg->y = y;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, movexyThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::movexyTo(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// if movement is too small, just call it good
	if (fabs(x-x0) < 0.1 && fabs(y-y0) < 0.1) {
		return 1;
	}

	// get current rotation
	double r0 = this->getRotation(BODY, 2);

	// compute rotation matrix for body frame
	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// get angle to turn in body coordinates (transform of R)
	double angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

	// get speed of robot
	double speed[NUM_DOF] = {0};
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	if (fabs(speed[0]) > 120) {
		this->setJointSpeeds(45, 45, 45);
	}

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// turn in shortest path
		if (angle > 0.005)
			this->turnRight(RAD2DEG(angle), radius, trackwidth);
		else if (angle < -0.005)
			this->turnLeft(RAD2DEG(-angle), radius, trackwidth);

		// calculate new rotation from error
		this->getxy(x0, y0);
		r0 = this->getRotation(BODY, 2);
		dRSetIdentity(R);
		dRFromAxisAndAngle(R, 0, 0, 1, r0);
		angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// move along length of line
	this->getxy(x0, y0);
	this->moveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// success
	return 0;
}

void* CLinkbotT::movexyToThread(void *arg) {
	// cast arg
	linkbotMoveArg_t *mArg = (linkbotMoveArg_t *)arg;

	// perform motion
	mArg->robot->movexyTo(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::movexyToNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	linkbotMoveArg_t *mArg = new linkbotMoveArg_t;
	mArg->robot = this;
	mArg->x = x;
	mArg->y = y;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, movexyToThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::movexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// movexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = func(x);
		this->movexyTo(x, y, radius, trackwidth);
	}

	// success
	return 0;
}

void* CLinkbotT::movexyToFuncThread(void *arg) {
	// cast arg
	linkbotMoveArg_t *mArg = (linkbotMoveArg_t *)arg;

	// perform motion
	mArg->robot->movexyToFunc(mArg->x, mArg->y, mArg->i, mArg->func, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::movexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	linkbotMoveArg_t *mArg = new linkbotMoveArg_t;
	mArg->robot = this;
	mArg->x = x0;
	mArg->y = xf;
	mArg->i = n;
	mArg->func = func;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, movexyToFuncThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::movexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// init variables
	double *coeff;
	int *power, order = 0;
	char str[5];
	char input[100];
	std::strcpy(input, poly);

	// parse 'fcn' into usable format
	char *var = std::strchr(input, '^');
	if (var != NULL) {
		order = atoi(++var);
		coeff = new double[order+1]();
		power = new int[order+1]();
		for (int i = 0; i < order+1; i++) {
			coeff[i] = 1;
			power[i] = order-i;
		}
		for (int i = 0; i < order; i++) {
			sprintf(str, "^%d", power[i]);
			var = std::strstr(input, str);
			if (var != NULL) {
				if (var[-2] == '*')
					coeff[i] = atof(var-=3);
				else if (var[-2] == ' ' || var[-2] == '=')
					coeff[i] = 1;
				else
					coeff[i] = atof(var-=2);
			}
			else {
				coeff[i] = 0;
			}
		}
		var = std::strrchr(input, 'x');
		var = std::strpbrk(input, "+-");
		if (var != NULL) {
			if (var[1] == ' ')
				var[1] = var[0];
			coeff[order] = atof(++var);
		}
		else {
			coeff[order] = 0;
		}
	}
	else {
		order = 1;
		coeff = new double[order+1];
		power = new int[order+1];
		power[0] = 1;
		var = std::strchr(input, 'x');
		if (var != NULL) {
			if (var[-1] == '*')
				coeff[0] = atof(var-=2);
			else
				coeff[0] = atof(--var);
		}
		var = std::strpbrk(input, "+-");
		if (var != NULL) {
			if (var[1] == ' ')
				var[1] = var[0];
			coeff[1] = atof(++var);
			power[1] = 0;
		}
	}

	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// movexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = 0;
		for (int j = 0; j <= order; j++) {
			y += coeff[j]*pow(x, power[j]);
		}
		this->movexyTo(x, y, radius, 0);
	}

	return 0;
}

void* CLinkbotT::movexyToPolyThread(void *arg) {
	// cast arg
	linkbotMoveArg_t *mArg = (linkbotMoveArg_t *)arg;

	// perform motion
	mArg->robot->movexyToPoly(mArg->x, mArg->y, mArg->i, mArg->expr, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CLinkbotT::movexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	linkbotMoveArg_t *mArg = new linkbotMoveArg_t;
	mArg->robot = this;
	mArg->x = x0;
	mArg->y = xf;
	mArg->i = n;
	mArg->expr = poly;
	mArg->radius = radius;
	mArg->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&move, movexyToPolyThread, (void *)mArg);

	// success
	return 0;
}

int CLinkbotT::movexyWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CLinkbotT::point(double x, double y, double z, int pointsize, char *color) {
	return _simObject->point(x, y, z, pointsize, color);
}
#endif // ENABLE_GRAPHICS

void* CLinkbotT::recordAngleThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);

	// is robot moving
	int *moving = new int[rArg->num];

	// get 'num' data points
	for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = *(rArg->robot->_clock)*1000;
		if (i == 0) { start_time = rArg->time[i]; }
		rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angle
		rArg->angle1[i] = RAD2DEG(rArg->robot->_angle[rArg->id]);

		// check if joint is moving
		moving[i] = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(rArg->id), dParamVel)*1000);

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

int CLinkbotT::recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData) {
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

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbotT::recordAngleThread, (void *)rArg);

	// success
	return 0;
}

void* CLinkbotT::recordAngleBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time = 0;
	int time = (int)((*(rArg->robot->_clock))*1000);

	// is robot moving
	int moving;

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[rArg->id] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[rArg->id]; i++) {
		// store locally num of data points taken
		rArg->robot->_rec_num[rArg->id] = i;

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

		// store joint angles
		(*(rArg->pangle1))[i] = RAD2DEG(rArg->robot->_angle[rArg->id]);
		moving = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(rArg->id), dParamVel)*1000);

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

		// wait until movement to start recording
		if( !moving && rArg->robot->isShiftEnabled() ) {
			i--;
		}
	}

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

int CLinkbotT::recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData) {
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

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbotT::recordAngleBeginThread, (void *)rArg);

	// success
	return 0;
}

int CLinkbotT::recordAngleEnd(robotJointId_t id, int &num) {
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

void* CLinkbotT::recordAnglesThread(void *arg) {
	// cast arg struct
    recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
    double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);

	// is robot moving
	int *moving = new int[rArg->num];

	// get 'num' data points
    for (int i = 0; i < rArg->num; i++) {
		// store time of data point
		rArg->time[i] = *(rArg->robot->_clock)*1000;
        if (i == 0) { start_time = rArg->time[i]; }
        rArg->time[i] = (rArg->time[i] - start_time) / 1000;

		// store joint angles
		rArg->angle1[i] = RAD2DEG(rArg->robot->_angle[JOINT1]);
		rArg->angle2[i] = RAD2DEG(rArg->robot->_angle[JOINT2]);
		rArg->angle3[i] = RAD2DEG(rArg->robot->_angle[JOINT3]);

		// check if joints are moving
		moving[i] = (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT1), dParamVel)*1000);
		moving[i] += (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT2), dParamVel)*1000);
		moving[i] += (int)(dJointGetAMotorParam(rArg->robot->getMotorID(JOINT3), dParamVel)*1000);

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
    for (int i = 0; i < NUM_DOF; i++) {
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

int CLinkbotT::recordAngles(double *time, double *angle1, double *angle2, double *angle3, int num, double seconds, int shiftData) {
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
	rArg->num = num;
	rArg->msecs = 1000*seconds;

	// lock recording for joints
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbotT::recordAnglesBeginThread, (void *)rArg);

	// success
	return 0;
}

void* CLinkbotT::recordAnglesBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	double start_time = 0;
	int time = (int)((*(rArg->robot->_clock))*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = true;
	rArg->robot->_rec_active[JOINT2] = true;
	rArg->robot->_rec_active[JOINT3] = true;
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
		}

		// store joint angles
		(*(rArg->pangle1))[i] = RAD2DEG(rArg->robot->_angle[JOINT1]);
		(*(rArg->pangle2))[i] = RAD2DEG(rArg->robot->_angle[JOINT2]);
		(*(rArg->pangle3))[i] = RAD2DEG(rArg->robot->_angle[JOINT3]);

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
	rArg->robot->_rec_active[JOINT1] = false;
	rArg->robot->_rec_active[JOINT2] = false;
	rArg->robot->_rec_active[JOINT3] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CLinkbotT::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, double seconds, int shiftData) {
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
	rArg->ptime = &time;
	rArg->pangle1 = &angle1;
	rArg->pangle2 = &angle2;
	rArg->pangle3 = &angle3;

	// store pointer to recorded angles locally
	_rec_angles[JOINT1] = &angle1;
	_rec_angles[JOINT2] = &angle2;
	_rec_angles[JOINT3] = &angle3;

	// lock recording for joint id
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbotT::recordAnglesBeginThread, (void *)rArg);

	// success
	return 0;
}

int CLinkbotT::recordAnglesEnd(int &num) {
	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[JOINT1] = 0;
	_recording[JOINT2] = 0;
	_recording[JOINT3] = 0;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[JOINT1] && _rec_active[JOINT2] && _rec_active[JOINT3]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[JOINT1];

	// success
	return 0;
}

int CLinkbotT::recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData) {
	// set radius of robot
	//_radius = radius;

	// record angle of desired joint
	this->recordAngleBegin(id, time, distance, seconds, shiftData);

	// success
	return 0;
}

int CLinkbotT::recordDistanceEnd(robotJointId_t id, int &num) {
	// end recording of angles
	this->recordAngleEnd(id, num);

	// convert radius to output units
	double radius = (_simObject->getUnits()) ? _radius*39.37 : _radius*100;

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		(*_rec_angles[id])[i] = DEG2RAD((*_rec_angles[id])[i]) * radius + _distOffset;
	}

	// success
	return 0;
}

int CLinkbotT::recordDistanceOffset(double distance) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// get current rotation
	dMatrix3 R;
	double r0 = this->getRotation(BODY, 2);
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

int CLinkbotT::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, double radius, double seconds, int shiftData) {
	// set radius of robot
	//_radius = radius;

	// record angles
	this->recordAnglesBegin(time, distance1, distance2, distance3, seconds, shiftData);

	// success
	return 0;
}

int CLinkbotT::recordDistancesEnd(int &num) {
	// end recording of angles
	this->recordAnglesEnd(num);

	// convert radius to output units
	double radius = (_simObject->getUnits()) ? _radius*39.37 : _radius*100;

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < NUM_DOF; j++) {
			(*_rec_angles[j])[i] = DEG2RAD((*_rec_angles[j])[i]) * radius + _distOffset;
		}
	}

	// success
	return 0;
}

int CLinkbotT::recordWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_recording_mutex);
	while ( _recording[0] || _recording[1] || _recording[2] ) {
		COND_WAIT(&_recording_cond, &_recording_mutex);
	}
	MUTEX_UNLOCK(&_recording_mutex);

	// success
	return 0;
}

void* CLinkbotT::recordxyBeginThread(void *arg) {
	// cast arg struct
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// create initial time points
	int time = (int)((*(rArg->robot->_clock))*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[JOINT1] = true;
	rArg->robot->_rec_active[JOINT2] = true;
	rArg->robot->_rec_active[JOINT3] = true;
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
	rArg->robot->_rec_active[JOINT1] = false;
	rArg->robot->_rec_active[JOINT2] = false;
	rArg->robot->_rec_active[JOINT3] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CLinkbotT::recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int shiftData) {
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
	x = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	y = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	rArg->ptime = &x;
	rArg->pangle1 = &y;

	// store pointer to recorded angles locally
	_rec_angles[JOINT1] = &x;
	_rec_angles[JOINT2] = &y;

	// lock recording for joint id
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbotT::recordxyBeginThread, (void *)rArg);

	// success
	return 0;
}

int CLinkbotT::recordxyEnd(int &num) {
	// sleep to capture last data point on ending time
#ifdef _WIN32
	Sleep(150);
#else
	usleep(150000);
#endif

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[JOINT1] = 0;
	_recording[JOINT2] = 0;
	_recording[JOINT3] = 0;
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
	double m2x = (_simObject->getUnits()) ? 39.37 : 100;
	for (int i = 0; i < num; i++) {
		(*_rec_angles[JOINT1])[i] = ((*_rec_angles[JOINT1])[i]) * m2x;
		(*_rec_angles[JOINT2])[i] = ((*_rec_angles[JOINT2])[i]) * m2x;
	}

	// success
	return 0;
}

int CLinkbotT::reset(void) {
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

int CLinkbotT::resetToZero(void) {
	this->resetToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_angle_mutex);
	for (int i = 0; i < 3; i++) {
		int rev = (int)(_angle[i]/2/M_PI);
		if (rev) {
			_angle[i] -= 2*rev*M_PI;
			_goal[i] -= 2*rev*M_PI;
		}
	}
	MUTEX_UNLOCK(&_angle_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int CLinkbotT::setBuzzerFrequency(int frequency, double time) {
	printf("CLinkbot::setBuzzerFrequency not implemented.\n");

	// success
	return 0;
}

int CLinkbotT::setBuzzerFrequencyOn(int frequency) {
	printf("CLinkbot::setBuzzerFrequencyOn not implemented.\n");

	// success
	return 0;
}

int CLinkbotT::setBuzzerFrequencyOff() {
	printf("CLinkbot::setBuzzerFrequencyOff not implemented.\n");

	// success
	return 0;
}

int CLinkbotT::setColor(char *color) {
	return this->setLEDColor(color);
}

int CLinkbotT::setColorRGB(int r, int g, int b) {
	return this->setLEDColorRGB(r, g, b);
}

int CLinkbotT::setLEDColor(char *color) {
	int getRGB[3] = {0};
	rgbHashTable *rgbTable = HT_Create();
	int htRetval = HT_Get(rgbTable, color, getRGB);
	HT_Destroy(rgbTable);

	if (htRetval) {
		_rgb[0] = getRGB[0]/255.0;
		_rgb[1] = getRGB[1]/255.0;
		_rgb[2] = getRGB[2]/255.0;

#ifdef ENABLE_GRAPHICS
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#endif // ENABLE_GRAPHICS

		// success
		return 0;
	}
	else {
		return htRetval;
	}
}

int CLinkbotT::setLEDColorRGB(int r, int g, int b) {
	_rgb[0] = r/255.0;
	_rgb[1] = g/255.0;
	_rgb[2] = b/255.0;
#ifdef ENABLE_GRAPHICS
	_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#endif // ENABLE_GRAPHICS

	// success
	return 0;
}

int CLinkbotT::setExitState(robotJointState_t exitState) {
	// set exit state of joints
	this->setMovementStateNB(exitState, exitState, exitState);

	// success
	return 0;
}

int CLinkbotT::setJointMovementStateNB(robotJointId_t id, robotJointState_t dir) {
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
		default:
			break;
	}
	_success[id] = true;
    dBodyEnable(_body[BODY]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CLinkbotT::setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds) {
	// switch direction for linkbot i to get forward movement
	if (_type == LINKBOTI && id == JOINT3) {
		switch (dir) {
			case ROBOT_FORWARD:
				dir = ROBOT_BACKWARD;
				break;
			case ROBOT_BACKWARD:
				dir = ROBOT_FORWARD;
				break;
			case ROBOT_POSITIVE:
				dir = ROBOT_FORWARD;
				break;
			case ROBOT_NEGATIVE:
				dir = ROBOT_BACKWARD;
				break;
			default:
				break;
		}
	}

	// move joint
	this->setJointMovementStateNB(id, dir);

	// sleep
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// stop joint
	this->setJointMovementStateNB(id, ROBOT_HOLD);

	// success
	return 0;
}

int CLinkbotT::setJointSafetyAngle(double angle) {
	_safety_angle = angle;

	// success
	return 0;
}

int CLinkbotT::setJointSafetyAngleTimeout(double seconds) {
	_safety_timeout = seconds;

	// success
	return 0;
}

int CLinkbotT::setJointSpeed(robotJointId_t id, double speed) {
	if (speed > _max_speed[id]) {
		fprintf(stderr, "Warning: Setting the speed for joint %d to %.2lf degrees per second is "
			"beyond the hardware limit of %.2lf degrees per second.\n",
			id+1, speed, _max_speed[id]);
	}
	_speed[id] = DEG2RAD(speed);

	// success
	return 0;
}

int CLinkbotT::setJointSpeedRatio(robotJointId_t id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * _max_speed[(int)id]);
}

int CLinkbotT::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);

	// success
	return 0;
}

int CLinkbotT::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);

	// success
	return 0;
}

int CLinkbotT::setMotorPower(robotJointId_t id, int power) {
	printf("CLinkbot::setMotorPower not implemented.\n");

	// success
	return 0;
}

int CLinkbotT::setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3) {
	// switch direction for linkbot i to get forward movement
	if (_type == LINKBOTI) {
		switch (dir3) {
			case ROBOT_FORWARD:
				dir3 = ROBOT_BACKWARD;
				break;
			case ROBOT_BACKWARD:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_POSITIVE:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_NEGATIVE:
				dir3 = ROBOT_BACKWARD;
				break;
			default:
				break;
		}
	}

	// set joint movements
	this->setJointMovementStateNB(JOINT1, dir1);
	this->setJointMovementStateNB(JOINT2, dir2);
	this->setJointMovementStateNB(JOINT3, dir3);

	// success
	return 0;
}

int CLinkbotT::setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds) {
	// switch direction for linkbot i to get forward movement
	if (_type == LINKBOTI) {
		switch (dir3) {
			case ROBOT_FORWARD:
				dir3 = ROBOT_BACKWARD;
				break;
			case ROBOT_BACKWARD:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_POSITIVE:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_NEGATIVE:
				dir3 = ROBOT_BACKWARD;
				break;
			default:
				break;
		}
	}

	// set joint movements
	this->setJointMovementStateNB(JOINT1, dir1);
	this->setJointMovementStateNB(JOINT2, dir2);
	this->setJointMovementStateNB(JOINT3, dir3);

	// sleep
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// stop motion
	this->setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);

	// success
	return 0;
}

void* CLinkbotT::setMovementStateTimeNBThread(void *arg) {
	// cast argument
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// sleep
#ifdef _WIN32
	Sleep(rArg->msecs);
#else
	usleep(rArg->msecs * 1000);
#endif

	// hold all robot motion
	CLinkbotT *ptr = dynamic_cast<CLinkbotT *>(rArg->robot);
	ptr->setMovementStateNB(ROBOT_HOLD, ROBOT_HOLD, ROBOT_HOLD);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CLinkbotT::setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds) {
	// switch direction for linkbot i to get forward movement
	if (_type == LINKBOTI) {
		switch (dir3) {
			case ROBOT_FORWARD:
				dir3 = ROBOT_BACKWARD;
				break;
			case ROBOT_BACKWARD:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_POSITIVE:
				dir3 = ROBOT_FORWARD;
				break;
			case ROBOT_NEGATIVE:
				dir3 = ROBOT_BACKWARD;
				break;
			default:
				break;
		}
	}

	// set up threading
	THREAD_T moving;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->setJointMovementStateNB(JOINT1, dir1);
	this->setJointMovementStateNB(JOINT2, dir2);
	this->setJointMovementStateNB(JOINT3, dir3);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&CLinkbotT::setMovementStateTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int CLinkbotT::setTwoWheelRobotSpeed(double speed, double radius) {
	if (RAD2DEG(speed/radius) > _max_speed[JOINT1]) {
		fprintf(stderr, "Warning: Speed %.2lf corresponds to joint speeds of %.2lf degrees/second.\n",
			speed, RAD2DEG(speed/radius));
	}
	this->setJointSpeed(JOINT1, RAD2DEG(speed/radius));
	this->setJointSpeed(JOINT3, RAD2DEG(speed/radius));

	// success
	return 0;
}

int CLinkbotT::stop(void) {
	this->stopAllJoints();

	// success
	return 0;
}

int CLinkbotT::stopOneJoint(robotJointId_t id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int CLinkbotT::stopTwoJoints(robotJointId_t id1, robotJointId_t id2) {
	this->setJointSpeed(id1, 0);
	this->setJointSpeed(id2, 0);

	// success
	return 0;
}

int CLinkbotT::stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3) {
	this->setJointSpeed(id1, 0);
	this->setJointSpeed(id2, 0);
	this->setJointSpeed(id3, 0);

	// success
	return 0;
}

int CLinkbotT::stopAllJoints(void) {
	this->setJointSpeed(JOINT1, 0);
	this->setJointSpeed(JOINT2, 0);
	this->setJointSpeed(JOINT3, 0);

	// success
	return 0;
}

int CLinkbotT::systemTime(double &time) {
	// get time
	time = *_clock;

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CLinkbotT::text(double x, double y, double z, char *text) {
	return _simObject->text(x, y, z, text);
}
#endif // ENABLE_GRAPHICS

int CLinkbotT::traceOff(void) {
	_trace = 0;

	// success
	return 0;
}

int CLinkbotT::traceOn(void) {
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

int CLinkbotT::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (_simObject->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(-angle, 0, -angle);

	// success
	return 0;
}

int CLinkbotT::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (_simObject->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(angle, 0, angle);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CLinkbotT::addToSim(dWorldID &world, dSpaceID &space, double *clock) {
	_world = world;
    _space = dHashSpaceCreate(space);
	_clock = clock;

	// success
	return 0;
}

int CLinkbotT::build(xml_robot_t robot) {
	// create rotation matrix
	double   sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta, -cphi*stheta*spsi - sphi*cpsi, -cphi*stheta*cpsi + sphi*spsi, 0,
				  sphi*ctheta, -sphi*stheta*spsi + cphi*cpsi, -sphi*stheta*cpsi - cphi*spsi, 0,
				  stheta, ctheta*spsi, ctheta*cpsi, 0};

	// check for wheels
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->conn == BIGWHEEL) {
			robot->z += (_bigwheel_radius - _body_height/2);
			_radius = _bigwheel_radius;
			break;
		}
		else if (ctmp->conn == SMALLWHEEL) {
			robot->z += (_smallwheel_radius - _body_height/2);
			_radius = _smallwheel_radius;
			break;
		}
		else if (ctmp->conn == TINYWHEEL) {
			robot->z += (_tinywheel_radius - _body_height/2);
			_radius = _tinywheel_radius;
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
	this->build_individual(robot->x, robot->y, robot->z, R, robot->angle1, robot->angle2, robot->angle3);

	// add connectors
	ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->robot == _id) {
			if (ctmp->conn == -1 )
				this->add_connector(ctmp->type, ctmp->face1, ctmp->size);
			else
				this->add_daisy_chain(ctmp->conn, ctmp->face1, ctmp->size, ctmp->side, ctmp->type);
		}
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

	// fix to ground
	if (robot->ground != -1) this->fix_body_to_ground(_body[robot->ground]);

	// success
	return 0;
}

int CLinkbotT::build(xml_robot_t robot, CRobot *base, xml_conn_t conn) {
	// build robot
	this->build_attached(robot, base, conn);

	// add connectors
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->robot == _id) {
			if (ctmp->conn == -1)
				this->add_connector(ctmp->type, ctmp->face1, ctmp->size);
			else
				this->add_daisy_chain(ctmp->conn, ctmp->face1, ctmp->size, ctmp->side, ctmp->type);
		}
		else if (ctmp->face2 != conn->face2) {
			this->fix_connector_to_body(this->getBodyID(ctmp->face2), base->getConnectorBodyID(ctmp->face1));
		}
		ctmp = ctmp->next;
	}

	// fix to ground
	if (robot->ground != -1) this->fix_body_to_ground(_body[robot->ground]);

	// success
	return 0;
}

double CLinkbotT::getAngle(int i) {
	if (i != _disabled)
		_angle[i] = mod_angle(_angle[i], dJointGetHingeAngle(_joint[i]), dJointGetHingeAngleRate(_joint[i])) - _offset[i];
	else
		_angle[i] = 0;

	// add noise to angle
	//this->noisy(&(_angle[i]), 1, 0.0005);

    return _angle[i];
}

double CLinkbotT::getAngularRate(int i) {
	return dJointGetAMotorParam(_motor[i], dParamVel);
}

dBodyID CLinkbotT::getBodyID(int id) {
    return _body[id];
}

double CLinkbotT::getCenter(int i) {
	const double *pos = dBodyGetPosition(_body[BODY]);
	const double *R = dBodyGetRotation(_body[BODY]);
	double p[3] = {	R[0]*_center[0] + R[1]*_center[1] + R[2]*_center[2],
					R[4]*_center[0] + R[5]*_center[1] + R[6]*_center[2],
					R[8]*_center[0] + R[9]*_center[1] + R[10]*_center[2]};
	return pos[i] + p[i];
}

int CLinkbotT::getConnectionParams(int face, dMatrix3 R, double *p) {
	double offset[3] = {0};
	const double *pos = dBodyGetPosition(_body[face]);
	const double *R1 = dBodyGetRotation(_body[face]);
	dMatrix3 R2;

	// get offset and rotation of face connection
	switch (face) {
		case 1:
			offset[0] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI);
			break;
		case 2:
			offset[1] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], -M_PI/2);
			break;
		case 3:
			offset[0] = _face_depth/2;
			dRSetIdentity(R2);
			break;
	}

	// generate new position
	p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
	p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
	p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
	// generate new rotation matrix
	dMultiply0(R, R2, R1, 3, 3, 3);

	// success
	return 0;
}

dBodyID CLinkbotT::getConnectorBodyID(int face) {
	conn_t ctmp = _conn;
	while (ctmp) {
		if (ctmp->face == face) {
			return ctmp->body;
		}
		ctmp = ctmp->next;
	}
	return NULL;
}

dBodyID CLinkbotT::getConnectorBodyIDs(int num) {
	conn_t ctmp = _conn;
	int i = 0;
	while (ctmp && i++ < num)
		ctmp = ctmp->next;
	if (ctmp) {
		return ctmp->body;
	}
	return NULL;
}

int CLinkbotT::getRobotID(void) {
	return _id;
}

dJointID CLinkbotT::getMotorID(int id) {
    return _motor[id];
}

double CLinkbotT::getPosition(int body, int i) {
	const double *pos = dBodyGetPosition(_body[body]);
	return pos[i];
}

double CLinkbotT::getRotation(int body, int i) {
	const double *R = dBodyGetRotation(_body[body]);
	double angles[3] = {0};
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

bool CLinkbotT::getSuccess(int i) {
	return _success[i];
}

int CLinkbotT::getType(void) {
	return _type;
}

bool CLinkbotT::isHome(void) {
    return ( fabs(_angle[FACE1]) < EPSILON && fabs(_angle[FACE2]) < EPSILON && fabs(_angle[FACE3]) < EPSILON );
}

int CLinkbotT::isShiftEnabled(void) {
	if(_shift_data && !_g_shift_data_en)
		return 1;
	else if (_g_shift_data_en && _g_shift_data)
		return 1;
	else
		return 0;
}

int CLinkbotT::setID(int id) {
	_id = id;
	return 0;
}

void CLinkbotT::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[BODY]);
	// put into accel array
	_accel[0] = R[8];
	_accel[1] = R[9];
	_accel[2] = R[10];
	// add gaussian noise to accel
	this->noisy(_accel, 3, 0.005);

	// update angle values for each degree of freedom
	for (int j = 0; j < ((_disabled == -1) ? 3 : 2); j++) {
		int i = _enabled[j];
		// store current angle
		_angle[i] = getAngle(i);
		// set rotation axis
		dVector3 axis;
		conn_t ctmp = _conn;
		dJointGetHingeAxis(_joint[i], axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
		while (ctmp) {
			if (ctmp->face == i+1)
				dBodySetFiniteRotationAxis(ctmp->body, axis[0], axis[1], axis[2]);
			ctmp = ctmp->next;
		}
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i], 0, _angle[i]);
		// drive motor to get current angle to match future angle
		if (_seek[i]) {
			if ((_goal[i] - 6*_encoder - _angle[i]) > EPSILON) {
				_state[i] = ROBOT_FORWARD;
				if (_starting[i]++ < 25)
					dJointSetAMotorParam(_motor[i], dParamVel, _starting[i]*_speed[i]/50);
				else if (_starting[i]++ < 50)
					dJointSetAMotorParam(_motor[i], dParamVel, _starting[i]*_speed[i]/150 + 0.3*_speed[i]);
				else if (_starting[i]++ < 100)
					dJointSetAMotorParam(_motor[i], dParamVel, _starting[i]*_speed[i]/150 + _speed[i]/3);
				else
					dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]);
			}
			else if ((_goal[i] - 3*_encoder - _angle[i]) > EPSILON) {
				_state[i] = ROBOT_FORWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]/2);
			}
			else if ((_goal[i] - _encoder - _angle[i]) > EPSILON) {
				_state[i] = ROBOT_FORWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]/4);
			}
			else if ((_angle[i] - _goal[i] - 6*_encoder) > EPSILON) {
				_state[i] = ROBOT_BACKWARD;
				if (_starting[i]++ < 25)
					dJointSetAMotorParam(_motor[i], dParamVel, -_starting[i]*_speed[i]/50);
				else if (_starting[i]++ < 50)
					dJointSetAMotorParam(_motor[i], dParamVel, -_starting[i]*_speed[i]/150 - 0.3*_speed[i]);
				else if (_starting[i]++ < 100)
					dJointSetAMotorParam(_motor[i], dParamVel, -_starting[i]*_speed[i]/150 - _speed[i]/3);
				else
					dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]);
			}
			else if ((_angle[i] - _goal[i] - 3*_encoder) > EPSILON) {
				_state[i] = ROBOT_BACKWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]/2);
			}
			else if ((_angle[i] - _goal[i] - _encoder) > EPSILON) {
				_state[i] = ROBOT_BACKWARD;
				dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]/4);
			}
			else {
				_state[i] = ROBOT_HOLD;
				_starting[i] = 0;
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

void CLinkbotT::simPostCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int j = 0; j < ((_disabled == -1) ? 3 : 2); j++) {
		int i = _enabled[j];
		_stopping[i] += (!(int)(dJointGetAMotorParam(this->getMotorID(i), dParamVel)*1000) );
		// once motor has been stopped for 10 steps
		if (_stopping[i] == 50) {
			_stopping[i] = 0;
			_success[i] = 1;
		}
	}

	// signal completion of step
	if (_success[0] && _success[1] && _success[2]) {
		COND_SIGNAL(&_success_cond);
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

#ifdef ENABLE_GRAPHICS
int CLinkbotT::draw(osg::Group *root, int tracking) {
	// initialize variables
	_robot = new osg::Group();
	osg::ref_ptr<osg::Geode> body[NUM_PARTS+1];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[NUM_PARTS+1];
	osg::ref_ptr<osg::Texture2D> tex[2];
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < NUM_PARTS+1; i++) {
		body[i] = new osg::Geode;
	}

	// body
	pos = dGeomGetOffsetPosition(_geom[BODY][0]);
	dGeomGetOffsetQuaternion(_geom[BODY][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_width, _body_length, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY]->addDrawable(new osg::ShapeDrawable(box));
	{ // 'led'
		cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]+0.0001), 0.01, _body_height);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		_led = new osg::ShapeDrawable(cyl);
		body[4]->addDrawable(_led);
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1));
	}
	pos = dGeomGetOffsetPosition(_geom[BODY][1]);
	dGeomGetOffsetQuaternion(_geom[BODY][1], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[BODY]->addDrawable(new osg::ShapeDrawable(cyl));

    // face1
	pos = dGeomGetOffsetPosition(_geom[FACE1][0]);
	dGeomGetOffsetQuaternion(_geom[FACE1][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE1]->addDrawable(new osg::ShapeDrawable(cyl));

    // face 2
	pos = dGeomGetOffsetPosition(_geom[FACE2][0]);
	dGeomGetOffsetQuaternion(_geom[FACE2][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE2]->addDrawable(new osg::ShapeDrawable(cyl));

    // face 3
	pos = dGeomGetOffsetPosition(_geom[FACE3][0]);
	dGeomGetOffsetQuaternion(_geom[FACE3][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE3]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex[0] = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/body.png)));
	tex[1] = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/face.png)));
	for (int i = 0; i < 2; i++) {
		tex[i]->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex[i]->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex[i]->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex[i]->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	}
    body[0]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
    body[1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
    body[2]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
    body[3]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
    body[4]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	if (_disabled > 0) {
		body[_disabled+1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	}

	// set rendering properties
	for (int i = 0; i < NUM_PARTS+1; i++) {
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// position each body within robot
	for (int i = 0; i < NUM_PARTS+1; i++) {
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
			case BRIDGE:
				this->draw_bridge(ctmp, _robot);
				break;
			case CASTER:
				this->draw_caster(ctmp, _robot);
				break;
			case CUBE:
				this->draw_cube(ctmp, _robot);
				break;
			case FACEPLATE:
				this->draw_faceplate(ctmp, _robot);
				break;
			case GRIPPER:
				this->draw_gripper(ctmp, _robot);
				break;
			case OMNIDRIVE:
				this->draw_omnidrive(ctmp, _robot);
				break;
			case SIMPLE:
				this->draw_simple(ctmp, _robot);
				break;
			case SMALLWHEEL:
				this->draw_smallwheel(ctmp, _robot);
				break;
			case TINYWHEEL:
				this->draw_tinywheel(ctmp, _robot);
				break;
			case WHEEL:
				this->draw_wheel(ctmp, _robot);
				break;
		}
		ctmp = ctmp->next;
	}

	// set update callback for robot
	_robot->setUpdateCallback(new linkbotNodeCallback(this, _simObject->getUnits()));

	// set masks
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
	colors->push_back(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0f) );
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

/**********************************************************
	private functions
 **********************************************************/
int CLinkbotT::add_connector(int type, int face, double size) {
	// create new connector
	conn_t nc = new struct conn_s;
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
		case BRIDGE:
			this->build_bridge(nc, face);
			break;
		case CASTER:
			this->build_caster(nc, face);
			break;
		case CUBE:
			this->build_cube(nc, face);
			break;
		case FACEPLATE:
			this->build_faceplate(nc, face);
			break;
		case GRIPPER:
			this->build_gripper(nc, 1);
			break;
		case OMNIDRIVE:
			this->build_omnidrive(nc, face);
			break;
		case SIMPLE:
			this->build_simple(nc, face);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(nc, face);
			break;
		case TINYWHEEL:
			this->build_tinywheel(nc, face);
			break;
		case WHEEL:
			this->build_wheel(nc, face, size);
			break;
	}

	if (type == GRIPPER) {
		conn_t nc2 = new struct conn_s;
		nc2->face = 3;
		nc2->type = GRIPPER;
		nc2->next = NULL;

		// add to list of connectors
		conn_t ctmp = _conn;
		while (ctmp->next)
			ctmp = ctmp->next;
		ctmp->next = nc2;

		// build
		this->build_gripper(nc2, 3);
	}

	// success
	return 0;
}

int CLinkbotT::add_daisy_chain(int conn, int face, double size, int side, int type) {
	// create new connector
	conn_t nc = new struct conn_s;
	nc->face = face;
	nc->type = conn;
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
	switch (conn) {
		case BIGWHEEL:
			this->build_bigwheel(nc, face, side, type);
			break;
		case BRIDGE:
			this->build_bridge(nc, face, side, type);
			break;
		case CASTER:
			this->build_caster(nc, face, side, type);
			break;
		case CUBE:
			this->build_cube(nc, face, side, type);
			break;
		case FACEPLATE:
			this->build_faceplate(nc, face, side, type);
			break;
		case OMNIDRIVE:
			this->build_omnidrive(nc, face, side, type);
			break;
		case SIMPLE:
			this->build_simple(nc, face, side, type);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(nc, face, side, type);
			break;
		case TINYWHEEL:
			this->build_tinywheel(nc, face, side, type);
			break;
		case WHEEL:
			this->build_wheel(nc, face, size, side, type);
			break;
	}

	// success
	return 0;
}


int CLinkbotT::build_individual(double x, double y, double z, dMatrix3 R, double r_f1, double r_f2, double r_f3) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
	_geom[BODY] = new dGeomID[2];
	_geom[FACE1] = new dGeomID[1];
	_geom[FACE2] = new dGeomID[1];
	_geom[FACE3] = new dGeomID[1];

	// adjust input height by body height
	if (fabs(z) < (_body_radius-EPSILON)) {z += _body_height/2; }

    // convert input angles to radians
    _angle[F1] = DEG2RAD(r_f1);	// face 1
    _angle[F2] = DEG2RAD(r_f2);	// face 2
    _angle[F3] = DEG2RAD(r_f3);	// face 3

	// set goal to current angle
	_goal[F1] = _angle[F1];
	_goal[F2] = _angle[F2];
	_goal[F3] = _angle[F3];

	// offset values for each body part[0-2] and joint[3-5] from center
	double f1[6] = {-_body_width/2 - _face_depth/2, 0, 0, -_body_width/2, 0, 0};
	double f2[6] = {0, -_body_length - _face_depth/2, 0, 0, -_body_length, 0};
	double f3[6] = {_body_width/2 + _face_depth/2, 0, 0, _body_width/2, 0, 0};

	// build robot bodies
	this->build_body(x, y, z, R, 0);
	this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R, 0);
	this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R, 0);
	this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R, 0);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = 0.012462;
	_center[2] = 0;

    // joint for body to face 1
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY], _body[FACE1]);
	dJointSetHingeAnchor(_joint[0], R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x,
									R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
									R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[FACE1], R[0], R[4], R[8]);

    // joint for body to face 2
	if (_disabled == 1) {
		dJointID joint = dJointCreateFixed(_world, 0);
		dJointAttach(joint, _body[BODY], _body[FACE2]);
		dJointSetFixed(joint);
	}
	else {
		_joint[1] = dJointCreateHinge(_world, 0);
		dJointAttach(_joint[1], _body[BODY], _body[FACE2]);
		dJointSetHingeAnchor(_joint[1], R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
										R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
										R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
		dJointSetHingeAxis(_joint[1], R[1], R[5], R[9]);
		dBodySetFiniteRotationAxis(_body[FACE2], R[1], R[5], R[9]);
	}

    // joint for body to face 3
	if (_disabled == 2) {
		dJointID joint = dJointCreateFixed(_world, 0);
		dJointAttach(joint, _body[BODY], _body[FACE3]);
		dJointSetFixed(joint);
	}
	else {
		_joint[2] = dJointCreateHinge(_world, 0);
		dJointAttach(_joint[2], _body[BODY], _body[FACE3]);
		dJointSetHingeAnchor(_joint[2], R[0]*f3[3] + R[1]*f3[4] + R[2]*f3[5] + x,
										R[4]*f3[3] + R[5]*f3[4] + R[6]*f3[5] + y,
										R[8]*f3[3] + R[9]*f3[4] + R[10]*f3[5] + z);
		dJointSetHingeAxis(_joint[2], -R[0], -R[4], -R[8]);
		dBodySetFiniteRotationAxis(_body[FACE3], -R[0], -R[4], -R[8]);
	}

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2, R_f3;
    dRFromAxisAndAngle(R_f, -1, 0, 0, _angle[F1]);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 0, -1, 0, _angle[F2]);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 1, 0, 0, _angle[F3]);
    dMultiply0(R_f3, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _angle[F1] != 0 || _angle[F2] != 0 ||_angle[F3] != 0 ) {
		this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R_f1, 0);
		this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R_f2, 0);
		this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R_f3, 0);
	}

    // motor for body to face 1
    _motor[F1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F1], _body[BODY], _body[FACE1]);
    dJointSetAMotorMode(_motor[F1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F1], 1);
    dJointSetAMotorAxis(_motor[F1], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[F1], 0, 0);
    dJointSetAMotorParam(_motor[F1], dParamFMax, _max_force[F1]);
    dJointSetAMotorParam(_motor[F1], dParamFudgeFactor, 0.3);
	dJointDisable(_motor[F1]);

    // motor for body to face 2
    _motor[F2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F2], _body[BODY], _body[FACE2]);
    dJointSetAMotorMode(_motor[F2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F2], 1);
    dJointSetAMotorAxis(_motor[F2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(_motor[F2], 0, 0);
    dJointSetAMotorParam(_motor[F2], dParamFMax, _max_force[F2]);
    dJointSetAMotorParam(_motor[F2], dParamFudgeFactor, 0.3);
	dJointDisable(_motor[F2]);

    // motor for body to face 3
    _motor[F3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F3], _body[BODY], _body[FACE3]);
    dJointSetAMotorMode(_motor[F3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F3], 1);
    dJointSetAMotorAxis(_motor[F3], 0, 1, -R[0], -R[4], -R[8]);
    dJointSetAMotorAngle(_motor[F3], 0, 0);
    dJointSetAMotorParam(_motor[F3], dParamFMax, _max_force[F3]);
    dJointSetAMotorParam(_motor[F3], dParamFudgeFactor, 0.3);
	dJointDisable(_motor[F3]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CLinkbotT::build_attached(xml_robot_t robot, CRobot *base, xml_conn_t conn) {
	// initialize new variables
	double m[3] = {0};
	dMatrix3 R;

	// generate parameters for base robot
	base->getConnectionParams(conn->face1, R, m);

	// generate parameters for connector
	this->get_connector_params(conn->type, conn->side, R, m);

	// find position of new module
	double angle = 0;
	switch (conn->face2) {
		case FACE1:
			angle = robot->angle1;
			break;
		case FACE2:
			angle = robot->angle2;
			break;
		case FACE3:
			angle = robot->angle3;
			break;
	}
	this->get_body_params(robot->psi, conn->face2, angle, R, m);

    // build new module
	this->build_individual(m[0], m[1], m[2], R, robot->angle1, robot->angle2, robot->angle3);

    // add fixed joint to attach two modules
	this->fix_body_to_connector(base->getConnectorBodyID(conn->face1), conn->face2);

	// success
	return 0;
}

int CLinkbotT::build_body(double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m, m1, m2;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, 0, -_body_length/2, -_body_height/2);
	dMassSetCylinder(&m2, 400, 1, _body_radius, _body_width);
	dMassAdd(&m, &m2);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[BODY], x, y, z);
	dBodySetRotation(_body[BODY], R);
	dBodySetFiniteRotationMode(_body[BODY], 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry 1 - box
	_geom[BODY][0] = dCreateBox(_space, _body_width, _body_length, _body_height);
	dGeomSetBody(_geom[BODY][0], _body[BODY]);
	dGeomSetOffsetPosition(_geom[BODY][0], -m.c[0], -_body_length/2 - m.c[1], -m.c[2]);

	// set geometry 2 - cylinder
	_geom[BODY][1] = dCreateCylinder(_space, _body_radius, _body_width);
	dGeomSetBody(_geom[BODY][1], _body[BODY]);
	dGeomSetOffsetPosition(_geom[BODY][1], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[BODY][1], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[BODY], &m);

	// success
	return 0;
}

int CLinkbotT::build_face(int id, double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of body
	if (id == 2)
		dMassSetCylinder(&m, 270, 2, 2*_face_radius, _face_depth);
	else
		dMassSetCylinder(&m, 270, 1, 2*_face_radius, _face_depth);

	// adjsut x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[id], x, y, z);
	dBodySetRotation(_body[id], R);
	dBodySetFiniteRotationMode(_body[id], 1);

	// rotation matrix
	if (id == 2)
	    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	else
	    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry
	_geom[id][0] = dCreateCylinder(_space, _face_radius, _face_depth);
	dGeomSetBody(_geom[id][0], _body[id]);
	dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[id][0], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CLinkbotT::build_bigwheel(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_wheel_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_bigwheel_radius, _wheel_depth);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _bigwheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_bridge(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, -_bridge_length/2 + _face_radius, 0};
	if (face == 3) offset[1] = _bridge_length/2 - _face_radius;

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0] + R[1]*offset[1];
	p[1] += R[4]*offset[0] + R[5]*offset[1];
	p[2] += R[8]*offset[0] + R[9]*offset[1];

	// set mass of body
	dMassSetBox(&m, 270, _connector_depth, _bridge_length, _connector_height);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _connector_depth, _bridge_length, _connector_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_caster(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[4];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_connector_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _connector_depth, 1.5*_face_radius, _body_height);
	dMassTranslate(&m, 2*_connector_depth, 0, -_body_height/2);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry 1 - box
	conn->geom[0] = dCreateBox(_space, _connector_depth, 1.5*_face_radius, _body_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set geometry 2 - horizontal support
	conn->geom[1] = dCreateBox(_space, 0.02, 0.022, 0.0032);
	dGeomSetBody(conn->geom[1], conn->body);
	dGeomSetOffsetPosition(conn->geom[1], _connector_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

	// set geometry 3 - ball support
	conn->geom[2] = dCreateCylinder(_space, 0.011, _radius -_face_radius - 0.006 + 0.0032);
	dGeomSetBody(conn->geom[2], conn->body);
	dGeomSetOffsetPosition(conn->geom[2], _connector_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 - (_radius -_face_radius - 0.006)/2 + 0.0016 - m.c[2]);

	// set geometry 4 - sphere
	conn->geom[3] = dCreateSphere(_space, 0.006);
	dGeomSetBody(conn->geom[3], conn->body);
	dGeomSetOffsetPosition(conn->geom[3], _connector_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 + _face_radius - _radius + 0.006 - m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_cube(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_cubic_length/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _cubic_length, _cubic_length, _cubic_length);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _cubic_length, _cubic_length, _cubic_length);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_faceplate(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _connector_depth, _body_height, _body_height);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _connector_depth, _body_height, _body_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_gripper(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[3];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, 0, 0};
	int i = (face == 1) ? 1 : -1;

	// position center of connector
	this->getConnectionParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _connector_depth, 2*_face_radius, _connector_height);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry 1
	conn->geom[0] = dCreateBox(_space, _connector_depth, 4*_face_radius, _connector_height/2);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetPosition(conn->geom[0], 0, -i*_face_radius, 0);

	// set geometry 2
	conn->geom[1] = dCreateBox(_space, 0.062, 0.04, _connector_depth);
	dGeomSetBody(conn->geom[1], conn->body);
	dGeomSetOffsetPosition(conn->geom[1], _connector_depth/2 - 0.062/2 - m.c[0],
							-i*3*_face_radius + i*0.02 - m.c[1],
							i*_connector_height/4 - i*_connector_depth/2 - m.c[2]);

	// set geometry 3
	conn->geom[2] = dCreateBox(_space, 0.0344, 0.04, 0.007);
	dGeomSetBody(conn->geom[2], conn->body);
	dGeomSetOffsetPosition(conn->geom[2], _connector_depth/2 - 0.062 + 0.0344/2 - m.c[0],
							-i*3*_face_radius + i*0.02 - m.c[1],
							i*_connector_height/4 - i*_connector_depth/2 - i*0.007/2 - m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_omnidrive(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, _omni_length/2 - _face_radius, -_omni_length/2 + _face_radius};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];

	// set mass of body
	dMassSetBox(&m, 270, _omni_length, _omni_length, _connector_depth);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _connector_depth, _omni_length, _omni_length);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_simple(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_connector_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _connector_depth, 2*_face_radius, _connector_height);

	// adjsut x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _connector_depth, 2*_face_radius, _connector_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_smallwheel(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_wheel_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_smallwheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _smallwheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_tinywheel(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_wheel_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_tinywheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _tinywheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::build_wheel(conn_t conn, int face, double size, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// store wheel radius
	_wheel_radius = size;

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_wheel_depth/2, 0, 0};

	// position center of connector
	this->getConnectionParams(face, R, p);
	if (side != -1) this->get_connector_params(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_wheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	if (side != -1)
		this->fix_connector_to_body(this->getConnectorBodyID(face), conn->body);
	else
		this->fix_connector_to_body(this->getBodyID(face), conn->body);

	// success
	return 0;
}

int CLinkbotT::fix_body_to_connector(dBodyID cBody, int face) {
	if (!cBody) { fprintf(stderr, "Error: connector body does not exist\n"); exit(-1); }

	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CLinkbotT::fix_body_to_ground(dBodyID cbody) {
	if (!cbody) { fprintf(stderr, "Error: connector body does not exist\n"); exit(-1); }

	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, 0, cbody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CLinkbotT::fix_connector_to_body(dBodyID rBody, dBodyID cBody) {
	if (!cBody) { fprintf(stderr, "Error: connector body does not exist\n"); exit(-1); }

	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, rBody, cBody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CLinkbotT::get_body_params(double angle, int face, double rotation, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, R5;

	// rotate about connection joint
	dRFromAxisAndAngle(R1, R[0], R[4], R[8], angle);
	dMultiply0(R2, R1, R, 3, 3, 3);

	// rotate body for connection face
	switch (face) {
		case 1:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], 0);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(rotation));
			break;
		case 2:
			offset[0] = _face_depth + _body_length;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], -M_PI/2);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -DEG2RAD(rotation));
			break;
		case 3:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], M_PI);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(rotation));
			break;
	}
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];
	dMultiply0(R, R5, R4, 3, 3, 3);

	// success
	return 0;
}

int CLinkbotT::get_connector_params(int type, int side, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};

	switch (type) {
		case BRIDGE:
			offset[1] = -_bridge_length + 2*_face_radius;
			dRFromAxisAndAngle(R1, R[1], R[5], R[9], M_PI);
			break;
		case CUBE:
			if (side == 2) {
				offset[0] = _cubic_length/2;
				offset[1] = _cubic_length/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI/2);
			}
			else if (side == 3) {
				offset[0] = _cubic_length;
				dRSetIdentity(R1);
			}
			else if (side == 4) {
				offset[0] = _cubic_length/2;
				offset[1] = -_cubic_length/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], -M_PI/2);
			}
			else if (side == 5) {
				offset[0] = _cubic_length/2;
				offset[2] = _cubic_length/2;
				dRFromAxisAndAngle(R2, R[1], R[5], R[9], -M_PI/2);
				dMultiply0(R3, R2, R, 3, 3, 3);
				dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -M_PI/2);
				dMultiply0(R1, R4, R2, 3, 3, 3);
			}
			break;
		case OMNIDRIVE:
			if (side == 2) {
				offset[2] = -_omni_length + 2*_face_radius;
			}
			else if (side == 3) {
				offset[1] = +_omni_length - 2*_face_radius;
			}
			else if (side == 4) {
				offset[1] = _omni_length - 2*_face_radius;
				offset[2] = -_omni_length + 2*_face_radius;
			}
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI);
			break;
		case SIMPLE:
			offset[0] = _connector_depth;
			dRSetIdentity(R1);
			break;
	}
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int CLinkbotT::init_params(int disabled, int type) {
	// create arrays for linkbots
	_angle = new double[NUM_DOF];
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[(disabled == -1) ? 3 : 2];
	_geom = new dGeomID * [NUM_PARTS];
	_goal = new double[NUM_DOF];
	_joint = new dJointID[NUM_DOF];
	_max_force = new double[NUM_DOF];
	_max_speed = new double[NUM_DOF];
	_motor = new dJointID[NUM_DOF];
	_offset = new double[NUM_DOF];
	_rec_active = new bool[NUM_DOF];
	_rec_angles = new double ** [NUM_DOF];
	_rec_num = new int[NUM_DOF];
	_recording = new bool[NUM_DOF];
	_seek = new bool[NUM_DOF];
	_speed = new double[NUM_DOF];
	_starting = new int[NUM_DOF];
	_stopping = new int[NUM_DOF];
	_state = new int[NUM_DOF];
	_success = new bool[NUM_DOF];

	// fill with default data
	for (int i = 0, j = 0; i < NUM_DOF; i++) {
		_angle[i] = 0;
		if (i != disabled) { _enabled[j++] = i; }
		_goal[i] = 0;
		_max_force[i] = 2;
		_max_speed[i] = 240;		// deg/sec
		_offset[i] = 0;
		_rec_active[i] = false;
		_rec_num[i] = 0;
		_recording[i] = false;
		_seek[i] = false;
		_speed[i] = 0.7854;			// 45 deg/sec
		_starting[i] = 0;
		_stopping[i] = 0;
		_state[i] = ROBOT_NEUTRAL;
		_success[i] = true;
	}
	_conn = NULL;
	_connected = 0;
	_disabled = disabled;
	_distOffset = 0;
	_encoder = DEG2RAD(0.25);
	_id = -1;
	_motion = false;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 1;
	_safety_angle = 10;
	_safety_timeout = 4;
	_shift_data = 0;
	_g_shift_data = 0;
	_g_shift_data_en = 0;
	_trace = 1;
	_type = type;

	// success
	return 0;
}

int CLinkbotT::init_dims(void) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_connector_depth = 0.00380;
	_connector_height = 0.03715;
	_bigwheel_radius = 0.05080;
	_bridge_length = 0.14025;
	_cubic_length = 0.07115;
	_omni_length = 0.17360;
	_radius = _body_height/2;
	_smallwheel_radius = 0.04445;
	_tinywheel_radius = 0.04128;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;

	// success
	return 0;
}

double CLinkbotT::mod_angle(double past_ang, double cur_ang, double ang_rate) {
    double new_ang = 0;
    int stp = (int)( fabs(past_ang) / M_PI );
    double past_ang_mod = fabs(past_ang) - stp*M_PI;

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

#ifdef ENABLE_GRAPHICS
void CLinkbotT::draw_bigwheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _bigwheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// add zero position dots
	int n = 5;
	double step = (_bigwheel_radius*5/6)/(n-1);
	for (int i = 0; i < n; i++) {
		cyl = new osg::Cylinder(osg::Vec3d(pos[0]+0.0001, pos[1], pos[2]+i*step), 0.0025, _wheel_depth);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		osg::ShapeDrawable *color = new osg::ShapeDrawable(cyl);
		color->setColor(osg::Vec4(0, 0, 0, 1));
		body->addDrawable(color);
	}

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_bridge(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	// draw geoms
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, _bridge_length, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_caster(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	osg::Sphere *sph;
	double	depth = _connector_depth,
			width = 1.5*_face_radius,
			height = _body_height;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, width, height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.02, 0.022, 0.0032);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), 0.011, _radius -_face_radius - 0.006 + 0.0032);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(conn->geom[3]);
	dGeomGetOffsetQuaternion(conn->geom[3], quat);
	sph = new osg::Sphere(osg::Vec3d(pos[0], pos[1], pos[2]), 0.006);
	body->addDrawable(new osg::ShapeDrawable(sph));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add body to pat
	pat->addChild(body.get());
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_cube(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	// draw geoms
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _cubic_length, _cubic_length, _cubic_length);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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

void CLinkbotT::draw_faceplate(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, _body_height, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_gripper(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, 4*_face_radius, _connector_height/2);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.062, 0.04, _connector_depth);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0344, 0.04, 0.007);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_omnidrive(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, _omni_length, _omni_length);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_simple(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, 2*_face_radius, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_smallwheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _smallwheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// add zero position dots
	int n = 4;
	double step = (_smallwheel_radius*5/6)/(n-1);
	for (int i = 0; i < n; i++) {
		cyl = new osg::Cylinder(osg::Vec3d(pos[0]+0.0001, pos[1], pos[2]+i*step), 0.0025, _wheel_depth);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		osg::ShapeDrawable *color = new osg::ShapeDrawable(cyl);
		color->setColor(osg::Vec4(0, 0, 0, 1));
		body->addDrawable(color);
	}

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
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
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_tinywheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _tinywheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// add zero position dots
	int n = 3;
	double step = (_tinywheel_radius*5/6)/(n-1);
	for (int i = 0; i < n; i++) {
		cyl = new osg::Cylinder(osg::Vec3d(pos[0]+0.0001, pos[1], pos[2]+i*step), 0.0025, _wheel_depth);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		osg::ShapeDrawable *color = new osg::ShapeDrawable(cyl);
		color->setColor(osg::Vec4(0, 0, 0, 1));
		body->addDrawable(color);
	}

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// add to scenegraph
	robot->addChild(pat);
}

void CLinkbotT::draw_wheel(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const double *pos;
	dQuaternion quat;
	osg::Cylinder *cyl;

    // set geometry
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _wheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(cyl));

	// add zero position dots
	int n = 2;
	double step = (_wheel_radius*5/6)/(n-1);
	for (int i = 0; i < n; i++) {
		cyl = new osg::Cylinder(osg::Vec3d(pos[0]+0.0001, pos[1], pos[2]+i*step), 0.0025, _wheel_depth);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		osg::ShapeDrawable *color = new osg::ShapeDrawable(cyl);
		color->setColor(osg::Vec4(0, 0, 0, 1));
		body->addDrawable(color);
	}

	// set rendering
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/conn.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// add body to pat
	pat->addChild(body.get());
	// add to scenegraph
	robot->addChild(pat);
}
#endif // ENABLE_GRAPHICS

#ifdef _CH_
CLinkbotI::CLinkbotI(void) {
	CLinkbotT(1, LINKBOTI);

	// success
	return 0;
}

CLinkbotI::~CLinkbotI(void) {
	// success
	return 0;
}

int CLinkbotI::connect(void) {
	_simObject->addRobot(this);

	// success
	return 0;
}

CLinkbotL::CLinkbotL(void) {
	CLinkbotT(2, LINKBOTL);

	// success
	return 0;
}

CLinkbotL::~CLinkbotL(void) {
	// success
	return 0;
}

int CLinkbotL::connect(void) {
	_simObject->addRobot(this);

	// success
	return 0;
}
#endif
