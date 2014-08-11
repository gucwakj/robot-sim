#include "robot.h"
#include "robosim.h"

Robot::Robot(void) {
	MUTEX_INIT(&_active_mutex);
	COND_INIT(&_active_cond);
	MUTEX_INIT(&_goal_mutex);
	MUTEX_INIT(&_motion_mutex);
	COND_INIT(&_motion_cond);
	MUTEX_INIT(&_recording_mutex);
	COND_INIT(&_recording_cond);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);
	MUTEX_INIT(&_theta_mutex);

	_seed = time(NULL);
}

Robot::~Robot(void) {
	// delete all arrays
	delete [] _body;
	delete [] _enabled;
	delete [] _geom;
	delete [] _joint;
	delete [] _motor;
	delete [] _rec_active;
	delete [] _rec_angles;
	delete [] _rec_num;
	delete [] _recording;

	// destroy mutexes
	MUTEX_DESTROY(&_active_mutex);
	COND_DESTROY(&_active_cond);
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_motion_mutex);
	COND_DESTROY(&_motion_cond);
	MUTEX_DESTROY(&_recording_mutex);
	COND_DESTROY(&_recording_cond);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
	MUTEX_DESTROY(&_theta_mutex);
}

int Robot::blinkLED(double delay, int num) {
#ifdef ENABLE_GRAPHICS
	// blink num-1 full times
	for (int i = 0; i < num-1; i++) {
		_led->setColor(osg::Vec4(1, 1, 1, 1));
		this->doze(delay);
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
		this->doze(delay);
	}

	// one last off before resetting to original color
	_led->setColor(osg::Vec4(1, 1, 1, 1));
	this->doze(delay);
	_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#endif // ENABLE_GRAPHICS

	// success
	return 0;
}

int Robot::connect(char *name, int pause) {
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

int Robot::delay(double milliseconds) {
	// set ending time
	double end = g_sim->getClock() + milliseconds/1000;

	// while clock hasn't reached ending time
	while ((end - g_sim->getClock()) >= EPSILON)
		this->doze(50);

	// success
	return 0;
}

int Robot::delaySeconds(double seconds) {
	// delay milliseconds
	this->delay(1000 * seconds);

	// success
	return 0;
}

int Robot::disableRecordDataShift(void) {
	_g_shift_data = 0;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int Robot::disconnect(void) {
	// and we are not connected
	_connected = 0;

	// success
	return 0;
}

int Robot::driveBackward(double angle) {
	this->driveForwardNB(-angle);
	this->moveWait();

	// success
	return 0;
}

int Robot::driveBackwardNB(double angle) {
	this->driveForwardNB(-angle);

	// success
	return 0;
}

int Robot::driveDistance(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));
	this->moveWait();

	// success
	return 0;
}

int Robot::driveDistanceNB(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));

	// success
	return 0;
}

int Robot::driveForever(void) {
	this->driveForeverNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::driveForeverNB(void) {
	this->moveJointForeverNB(static_cast<robotJointId_t>(0));
	this->moveJointForeverNB(static_cast<robotJointId_t>(_dof - 1));

	// success
	return 0;
}

int Robot::driveForward(double angle) {
	this->driveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int Robot::driveForwardNB(double angle) {
	this->moveJointNB(static_cast<robotJointId_t>(0), angle);
	this->moveJointNB(static_cast<robotJointId_t>(_dof - 1), angle);

	// success
	return 0;
}

int Robot::driveTime(double seconds) {
	// move joint
	this->driveForeverNB();

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoints();

	// success
	return 0;
}

int Robot::driveTimeNB(double seconds) {
	// set up threading
	THREAD_T moving;
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->driveForeverNB();

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::driveTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int Robot::drivexy(double x, double y, double radius, double trackwidth) {
	this->drivexyNB(x, y, radius, trackwidth);
	this->drivexyWait();

	// success
	return 0;
}

int Robot::drivexyNB(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// move to new global coordinates
	return this->drivexyToNB(x + x0, y + y0, radius, trackwidth);
}

int Robot::drivexyTo(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// if movement is too small, just call it good
	if (fabs(x-x0) < 0.1 && fabs(y-y0) < 0.1) {
		return 1;
	}

	// get current rotation
	double r0 = this->getRotation(0, 2);

	// compute rotation matrix for body frame
	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// get angle to turn in body coordinates (transform of R)
	double angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.01) {
		// turn in shortest path
		if (angle > 0.01)
			this->turnRight(RAD2DEG(angle), radius, trackwidth);
		else if (angle < -0.01)
			this->turnLeft(RAD2DEG(-angle), radius, trackwidth);

		// calculate new rotation from error
		this->getxy(x0, y0);
		r0 = this->getRotation(0, 2);
		dRSetIdentity(R);
		dRFromAxisAndAngle(R, 0, 0, 1, r0);
		angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));
	}

	// move along length of line
	this->getxy(x0, y0);
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// success
	return 0;
}

int Robot::drivexyToNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	moveArg_t *mArg = new moveArg_t;
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

int Robot::drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// drivexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = func(x);
		this->drivexyTo(x, y, radius, trackwidth);
	}

	// success
	return 0;
}

int Robot::drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	moveArg_t *mArg = new moveArg_t;
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
	THREAD_CREATE(&move, drivexyToFuncThread, (void *)mArg);

	// success
	return 0;
}

int Robot::drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
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

	// drivexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = 0;
		for (int j = 0; j <= order; j++) {
			y += coeff[j]*pow(x, power[j]);
		}
		this->drivexyTo(x, y, radius, 0);
	}

	return 0;
}

int Robot::drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	moveArg_t *mArg = new moveArg_t;
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
	THREAD_CREATE(&move, drivexyToPolyThread, (void *)mArg);

	// success
	return 0;
}

int Robot::drivexyWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int Robot::enableRecordDataShift(void) {
	_g_shift_data = 1;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int Robot::getBatteryVoltage(double &voltage) {
	voltage = 100;

	// success
	return 0;
}

int Robot::getDistance(double &distance, double radius) {
	double angle;
	this->getJointAngle(JOINT1, angle, 2);
	distance = DEG2RAD(angle) * radius;

	// success
	return 0;
}

int Robot::getFormFactor(int &formFactor) {
	formFactor = _type;

	// success
	return 0;
}

int Robot::getID(void) {
	// get id of robot
	return _id;
}

int Robot::getJointAngle(robotJointId_t id, double &angle, int numReadings) {
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

int Robot::getJointAngleInstant(robotJointId_t id, double &angle) {
	angle = RAD2DEG(_motor[id].theta);

	// success
	return 0;
}

int Robot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed) {
	maxSpeed = RAD2DEG(_motor[id].omega_max);

	// success
	return 0;
}

int Robot::getJointSafetyAngle(double &angle) {
	angle = _motor[JOINT1].safety_angle;

	// success
	return 0;
}

int Robot::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _motor[JOINT1].safety_timeout;

	// success
	return 0;
}

int Robot::getJointSpeed(robotJointId_t id, double &speed) {
	speed = RAD2DEG(_motor[id].omega);

	// success
	return 0;
}

int Robot::getJointSpeedRatio(robotJointId_t id, double &ratio) {
	ratio = _motor[id].omega/_motor[id].omega_max;

	// success
	return 0;
}

int Robot::getxy(double &x, double &y) {
	// return x and y positions
	x = (g_sim->getUnits()) ? 39.37*this->getCenter(0) : 100*this->getCenter(0);
	y = (g_sim->getUnits()) ? 39.37*this->getCenter(1) : 100*this->getCenter(1);

	// success
	return 0;
}

int Robot::holdJoint(robotJointId_t id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int Robot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		this->setJointSpeed(static_cast<robotJointId_t>(i), 0);
	}

	// success
	return 0;
}

int Robot::holdJointsAtExit(void) {
	// set joint speeds to zero
	this->holdJoints();

	// hold joints still
	this->moveForeverNB();

	// success
	return 0;
}

int Robot::isConnected(void) {
	// return connected status
	return _connected;
}

int Robot::isMoving(void) {
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].state == POSITIVE || _motor[i].state == NEGATIVE) {
			return 1;
		}
	}

	// success
	return 0;
}

int Robot::isNotMoving(void) {
	// oppositve of ismoving
	return !(this->isMoving());
}

int Robot::jumpJointTo(robotJointId_t id, double angle) {
	this->jumpJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::jumpJointToNB(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int Robot::moveForeverNB(void) {
	// set joint movements
	for (int i = 0; i < _dof; i++) {
		this->moveJointForeverNB(static_cast<robotJointId_t>(i));
	}

	// success
	return 0;
}

int Robot::moveJoint(robotJointId_t id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointNB(robotJointId_t id, double angle) {
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
	dBodyEnable(_body[0]);
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

int Robot::moveJointForeverNB(robotJointId_t id) {
	// lock mutexes
	MUTEX_LOCK(&_motor[id].success_mutex);
	// enable motor
	dJointEnable(_motor[id].id);
	// set motor angle to current angle
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	// set mode
	_motor[id].mode = CONTINUOUS;
	// drive in proper direction
	if ( _motor[id].omega > EPSILON )
		_motor[id].state = POSITIVE;
	else if ( _motor[id].omega < EPSILON )
		_motor[id].state = NEGATIVE;
	else
		_motor[id].state = HOLD;
	// successfully at 'goal'
	_motor[id].success = true;
	// enable bodies for collisions
    dBodyEnable(_body[0]);
	// unlock mutexes
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int Robot::moveJointTime(robotJointId_t id, double seconds) {
	// move joint
	this->moveJointForeverNB(id);

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoint(id);

	// success
	return 0;
}

int Robot::moveJointTimeNB(robotJointId_t id, double seconds) {
	// set up threading
	THREAD_T moving;
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;
	rArg->id = id;

	// set joint movements
	this->moveJointForeverNB(id);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::moveJointTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int Robot::moveJointTo(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointToNB(robotJointId_t id, double angle) {
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
	dBodyEnable(_body[0]);
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

int Robot::moveJointWait(robotJointId_t id) {
	// wait for motion to complete
	MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int Robot::moveTime(double seconds) {
	// move joint
	this->moveForeverNB();

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoints();

	// success
	return 0;
}

int Robot::moveTimeNB(double seconds) {
	// set up threading
	THREAD_T moving;
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->moveForeverNB();

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::moveTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int Robot::moveToZero(void) {
	this->moveToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::moveToZeroNB(void) {
	// move joints to zero
	for (int i = 0; i < _dof; i++) {
		this->moveJointToNB(static_cast<robotJointId_t>(i), 0);
	}

	// success
	return 0;
}

int Robot::moveWait(void) {
	// lock
	MUTEX_LOCK(&_success_mutex);
	// get number of successes
	int success = 0;
	for (int i = 0; i < _dof; i++) {
		success += _motor[static_cast<robotJointId_t>(i)].success;
	}
	// wait
	while (success != _dof) {
		COND_WAIT(&_success_cond, &_success_mutex);
		success = 0;
		for (int i = 0; i < _dof; i++) { success += _motor[static_cast<robotJointId_t>(i)].success; }
	}
	// reset motor states
	for (int i = 0; i < _dof; i++) {
		_motor[i].mode = CONTINUOUS;
	}
	// unlock
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int Robot::recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData) {
	// check if recording already
	if (_recording[id]) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->time = time;
	rArg->angle = new double * [1];
	rArg->angle[0] = angle;
	rArg->id = id;
	rArg->num = num;
	rArg->msecs = 1000*seconds;

	// lock recording for joint id
	_recording[id] = true;

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAngleThread, (void *)rArg);

	// success
	return 0;
}

int Robot::recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData) {
	// check if recording already
	if (_recording[id]) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->id = id;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	time = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	angle = (double *)malloc(sizeof(double) * RECORD_ANGLE_ALLOC_SIZE);
	rArg->ptime = &time;
	rArg->pangle = new double ** [_dof];
	rArg->pangle[0] = &angle;

	// store pointer to recorded angles locally
	_rec_angles[id] = &angle;

	// lock recording for joint id
	_recording[id] = true;

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAngleBeginThread, (void *)rArg);

	// success
	return 0;
}

int Robot::recordAngleEnd(robotJointId_t id, int &num) {
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

int Robot::recordAnglesEnd(int &num) {
	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	for (int i = 0; i < _dof; i++) {
		_recording[i] = 0;
	}
	MUTEX_UNLOCK(&_recording_mutex);

	// get number of joints recording
	int rec = 0;
	for (int i = 0; i < _dof; i++) {
		rec += _rec_active[i];
	}
	MUTEX_LOCK(&_active_mutex);
	while (rec) {
		COND_WAIT(&_active_cond, &_active_mutex);
		rec = 0;
		for (int i = 0; i < _dof; i++) { rec += _rec_active[i]; }
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[JOINT1];

	// success
	return 0;
}

int Robot::recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData) {
	// record angle of desired joint
	this->recordAngleBegin(id, time, distance, seconds, shiftData);

	// success
	return 0;
}

int Robot::recordDistanceEnd(robotJointId_t id, int &num) {
	// end recording of angles
	this->recordAngleEnd(id, num);

	// convert radius to output units
	double radius = (g_sim->getUnits()) ? _radius*39.37 : _radius*100;

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		(*_rec_angles[id])[i] = DEG2RAD((*_rec_angles[id])[i]) * radius + _distOffset;
	}

	// success
	return 0;
}

int Robot::recordDistanceOffset(double distance) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// get current rotation
	dMatrix3 R;
	double r0 = this->getRotation(0, 2);
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

int Robot::recordDistancesEnd(int &num) {
	// end recording of angles
	this->recordAnglesEnd(num);

	// convert radius to output units
	double radius = (g_sim->getUnits()) ? _radius*39.37 : _radius*100;

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < _dof; j++) {
			(*_rec_angles[j])[i] = DEG2RAD((*_rec_angles[j])[i]) * radius + _distOffset;
		}
	}

	// success
	return 0;
}

int Robot::recordWait(void) {
	// lock
	MUTEX_LOCK(&_recording_mutex);
	// get number of joints recording
	int recording = 0;
	for (int i = 0; i < _dof; i++) {
		recording += _recording[i];
	}
	// wait
	while (recording) {
		COND_WAIT(&_recording_cond, &_recording_mutex);
	}
	// unlock
	MUTEX_UNLOCK(&_recording_mutex);

	// success
	return 0;
}

int Robot::recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	x = new double[RECORD_ANGLE_ALLOC_SIZE];
	y = new double[RECORD_ANGLE_ALLOC_SIZE];
	rArg->ptime = &x;
	rArg->pangle = new double ** [1];
	rArg->pangle[0] = &y;

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
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordxyBeginThread, (void *)rArg);

	// success
	return 0;
}

int Robot::recordxyEnd(int &num) {
	// sleep to capture last data point on ending time
	this->doze(150);

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_recording[0] = 0;
	_recording[1] = 0;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_rec_active[0] && _rec_active[1]) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _rec_num[0];

	// convert recorded values into in/cm
	double m2x = (g_sim->getUnits()) ? 39.37 : 100;
	for (int i = 0; i < num; i++) {
		(*_rec_angles[0])[i] = ((*_rec_angles[0])[i]) * m2x;
		(*_rec_angles[1])[i] = ((*_rec_angles[1])[i]) * m2x;
	}

	// success
	return 0;
}

int Robot::relaxJoint(robotJointId_t id) {
	dJointDisable(_motor[id].id);

	// success
	return 0;
}

int Robot::relaxJoints(void) {
	for (int i = 0; i < _dof; i++) {
		this->relaxJoint(static_cast<robotJointId_t>(i));
	}

	// success
	return 0;
}

int Robot::resetToZero(void) {
	this->resetToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_theta_mutex);
	for (int i = 0; i < _dof; i++) {
		int rev = (int)(_motor[i].theta/2/M_PI);
		if (rev) {
			_motor[i].theta -= 2*rev*M_PI;
			_motor[i].goal -= 2*rev*M_PI;
		}
	}
	MUTEX_UNLOCK(&_theta_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int Robot::setBuzzerFrequency(int frequency, double time) {
	printf("::setBuzzerFrequency not implemented.\n");

	// success
	return 0;
}

int Robot::setBuzzerFrequencyOff(void) {
	printf("::setBuzzerFrequencyOff not implemented.\n");

	// success
	return 0;
}

int Robot::setBuzzerFrequencyOn(int frequency) {
	printf("::setBuzzerFrequencyOn not implemented.\n");

	// success
	return 0;
}

int Robot::setLEDColor(char *color) {
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

int Robot::setLEDColorRGB(int r, int g, int b) {
	_rgb[0] = r/255.0;
	_rgb[1] = g/255.0;
	_rgb[2] = b/255.0;
#ifdef ENABLE_GRAPHICS
	_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1.0));
#endif // ENABLE_GRAPHICS

	// success
	return 0;
}

int Robot::setJointPower(robotJointId_t id, int power) {
	_motor[id].omega = (power/100.0)*_motor[id].omega_max;

	// success
	return 0;
}

int Robot::setJointSafetyAngle(double angle) {
	for (int i = 0; i < _dof; i++) {
		_motor[i].safety_angle = angle;
	}

	// success
	return 0;
}

int Robot::setJointSafetyAngleTimeout(double seconds) {
	for (int i = 0; i < _dof; i++) {
		_motor[i].safety_timeout = seconds;
	}

	// success
	return 0;
}

int Robot::setJointSpeed(robotJointId_t id, double speed) {
	if (speed > RAD2DEG(_motor[id].omega_max)) {
		fprintf(stderr, "Warning: Setting the speed for joint %d to %.2lf degrees per second is "
			"beyond the hardware limit of %.2lf degrees per second.\n",
			id+1, speed, RAD2DEG(_motor[id].omega_max));
	}
	_motor[id].omega = DEG2RAD(speed);

	// success
	return 0;
}

int Robot::setJointSpeedRatio(robotJointId_t id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * RAD2DEG(_motor[(int)id].omega_max));
}

int Robot::systemTime(double &time) {
	// get time
	time = g_sim->getClock();

	// success
	return 0;
}

int Robot::traceOff(void) {
	_trace = 0;

	// success
	return 0;
}

int Robot::traceOn(void) {
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

int Robot::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int Robot::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move left joint backward
	this->moveJointNB(static_cast<robotJointId_t>(0), -angle);
	// move right joint forward
	this->moveJointNB(static_cast<robotJointId_t>(_dof - 1), angle);

	// success
	return 0;
}

int Robot::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int Robot::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move left joint forward
	this->moveJointNB(static_cast<robotJointId_t>(0), angle);
	// move right joint backward
	this->moveJointNB(static_cast<robotJointId_t>(_dof - 1), -angle);

	// success
	return 0;
}

/**********************************************************
	protected functions for variable DOF
 **********************************************************/
int Robot::moveNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointNB(static_cast<robotJointId_t>(i), angles[i]);
	}

	// success
	return 0;
}

int Robot::moveToNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointToNB(static_cast<robotJointId_t>(i), angles[i]);
	}

	// success
	return 0;
}

int Robot::recordAngles(double *time, double **angle, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->time = time;
	rArg->angle = new double * [_dof];
	rArg->angle = angle;
	rArg->num = num;
	rArg->msecs = 1000*seconds;

	// lock recording for joints
	for (int i = 0; i < _dof; i++) {
		rArg->angle[i] = angle[i];
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAnglesThread, (void *)rArg);

	// success
	return 0;
}

int Robot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t *&angle, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args struct
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->num = RECORD_ANGLE_ALLOC_SIZE;
	rArg->msecs = seconds * 1000;
	time = new double[RECORD_ANGLE_ALLOC_SIZE];
	for (int i = 0; i < _dof; i++) {
		angle[i] = new double[RECORD_ANGLE_ALLOC_SIZE];
	}
	rArg->ptime = &time;
	rArg->pangle = new double ** [_dof];
	for (int i = 0; i < _dof; i++) {
		rArg->pangle[i] = &angle[i];
	}

	// store pointer to recorded angles locally
	for (int i = 0; i < _dof; i++) {
		_rec_angles[i] = &angle[i];
	}

	// lock recording for joint id
	for (int i = 0; i < _dof; i++) {
		_recording[i] = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAnglesBeginThread, (void *)rArg);

	// success
	return 0;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int Robot::addToSim(dWorldID &world, dSpaceID &space, int id) {
	_world = world;
	_space = dHashSpaceCreate(space);
	_id = id;

	// success
	return 0;
}

int Robot::doze(double ms) {
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms*1000);
#endif
	// success
	return 0;
}

int Robot::fixBodyToGround(dBodyID cbody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, 0, cbody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

dBodyID Robot::getBodyID(int id) {
	return _body[id];
}

double Robot::getCenter(int i) {
	const double *pos = dBodyGetPosition(_body[0]);
	const double *R = dBodyGetRotation(_body[0]);
	double p[3] = {	R[0]*_center[0] + R[1]*_center[1] + R[2]*_center[2],
					R[4]*_center[0] + R[5]*_center[1] + R[6]*_center[2],
					R[8]*_center[0] + R[9]*_center[1] + R[10]*_center[2]};
	return pos[i] + p[i];
}

double Robot::getRotation(int body, int i) {
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

double Robot::mod_angle(double past_ang, double cur_ang, double ang_rate) {
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

int Robot::noisy(double *a, int length, double sigma) {
	// initialize variables
	double *rand = new double[length];
	double sum = 0;

	if (length == 1)
		a[0] += this->normal(sigma);
	else {
		// compute magnitude of randomized vector
		for (int i = 0; i < length; i++) {
			rand[i] = this->normal(sigma);
			sum += (a[i] + rand[i]) * (a[i] + rand[i]);
		}
		double mag = sqrt(sum);

		// normalize vector
		for (int i = 0; i < length; i++) {
			a[i] = (a[i] + rand[i])/mag;
		}
	}

	// clean up array
	delete [] rand;

	// success
	return 0;
}

void* Robot::simPreCollisionThreadEntry(void *arg) {
	Robot *p = (Robot *)arg;
	p->simPreCollisionThread();
	return arg;
}

void* Robot::simPostCollisionThreadEntry(void *arg) {
	Robot *p = (Robot *)arg;
	p->simPostCollisionThread();
	return arg;
}

/**********************************************************
	private functions
 **********************************************************/
bool Robot::is_shift_enabled(void) {
	if(_shift_data && !_g_shift_data_en)
		return 1;
	else if (_g_shift_data_en && _g_shift_data)
		return 1;
	else
		return 0;
}

double Robot::normal(double sigma) {
	// compute pair of random uniform data
	double u1 = this->uniform();
	double u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*M_PI*u2));
}

double Robot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
}

void* Robot::driveTimeNBThread(void *arg) {
	// cast argument
	recArg_t *rArg = (recArg_t *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rArg->robot);
	// sleep
	robot->doze(rArg->msecs);
	// hold all robot motion
	robot->holdJoints();

	// cleanup
	delete rArg;

	// success
	return NULL;
}

void* Robot::drivexyToThread(void *arg) {
	// cast arg
	moveArg_t *mArg = (moveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyTo(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

void* Robot::drivexyToFuncThread(void *arg) {
	// cast arg
	moveArg_t *mArg = (moveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyToFunc(mArg->x, mArg->y, mArg->i, mArg->func, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

void* Robot::drivexyToPolyThread(void *arg) {
	// cast arg
	moveArg_t *mArg = (moveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyToPoly(mArg->x, mArg->y, mArg->i, mArg->expr, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

void* Robot::moveJointTimeNBThread(void *arg) {
	// cast argument
	recArg_t *rArg = (recArg_t *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rArg->robot);
	// sleep
	robot->doze(rArg->msecs);
	// hold all robot motion
	robot->holdJoint(rArg->id);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

void* Robot::moveTimeNBThread(void *arg) {
	// cast argument
	recArg_t *rArg = (recArg_t *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rArg->robot);
	// sleep
	robot->doze(rArg->msecs);
	// hold all robot motion
	robot->holdJoints();

	// cleanup
	delete rArg;

	// success
	return NULL;
}

void* Robot::recordAngleThread(void *arg) {
	// cast arg struct
	recArg_t *rArg = (recArg_t *)arg;

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
		rArg->angle[0][i] = RAD2DEG(rArg->robot->_motor[rArg->id].theta);

		// check if joint is moving
		moving[i] = (int)(dJointGetAMotorParam(rArg->robot->_motor[rArg->id].id, dParamVel)*1000);

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rArg->robot->is_shift_enabled()) {
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
				rArg->angle[i] = rArg->angle[shiftTimeIndex];
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

void* Robot::recordAngleBeginThread(void *arg) {
	// cast arg struct
	recArg_t *rArg = (recArg_t *)arg;

	// create initial time points
	double start_time = 0;
	int time = (int)((g_sim->getClock())*1000);

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
			double *newbuf = new double[rArg->num];
			memcpy(newbuf, *rArg->ptime, sizeof(double)*i);
			delete *(rArg->ptime);
			*(rArg->ptime) = newbuf;
			// create larger array for angle
			newbuf = new double[rArg->num];
			memcpy(newbuf, *(rArg->pangle), sizeof(double)*i);
			delete (*(rArg->pangle));
			*(rArg->pangle[0]) = newbuf;
		}

		// store joint angles
		(*(rArg->pangle[0]))[i] = RAD2DEG(rArg->robot->_motor[rArg->id].theta);
		moving = (int)(dJointGetAMotorParam(rArg->robot->_motor[rArg->id].id, dParamVel)*1000);

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
		if( !moving && rArg->robot->is_shift_enabled() ) {
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

void* Robot::recordAnglesThread(void *arg) {
	// cast arg struct
    recArg_t *rArg = (recArg_t *)arg;

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
		for (int j = 0; j < rArg->robot->_dof; j++) {
			rArg->angle[j][i] = RAD2DEG(rArg->robot->_motor[j].theta);
		}

		// check if joints are moving
		moving[i] = 0;
		for (int j = 0; j < rArg->robot->_dof; j++) {
			moving[i] += (int)(dJointGetAMotorParam(rArg->robot->_motor[j].id, dParamVel)*1000);
		}

		// increment time step
		time += rArg->msecs;

		// pause until next step
		if ( (int)(g_sim->getClock()*1000) < time )
			rArg->robot->doze(time - (int)(g_sim->getClock()*1000));
    }

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rArg->robot->is_shift_enabled()) {
		for (int i = 0; i < rArg->num; i++) {
			if (moving[i]) {
				shiftTime = rArg->time[i];
				shiftTimeIndex = i;
				break;
			}
		}
		for (int i = 0; i < rArg->num; i++) {
			if (i < shiftTimeIndex) {
				rArg->time[i] = 0;
				for (int j = 0; j < rArg->robot->_dof; j++) {
					rArg->angle[j][i] = rArg->angle[j][shiftTimeIndex];
				}
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

void* Robot::recordAnglesBeginThread(void *arg) {
	// cast arg struct
	recArg_t *rArg = (recArg_t *)arg;

	// create initial time points
	double start_time = 0;
	int time = (int)((g_sim->getClock())*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	for (int i = 0; i < rArg->robot->_dof; i++) {
		rArg->robot->_rec_active[i] = true;
	}
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
			double *newbuf = new double[rArg->num];
			memcpy(newbuf, *rArg->ptime, sizeof(double)*i);
			delete *(rArg->ptime);
			*(rArg->ptime) = newbuf;
			for (int j = 0; j < rArg->robot->_dof; j++) {
				newbuf = new double[rArg->num];
				memcpy(newbuf, *(rArg->pangle[j]), sizeof(double)*i);
				delete (*(rArg->pangle[j]));
				*(rArg->pangle[j]) = newbuf;
			}
		}

		// store joint angles
		for (int j = 0; j < rArg->robot->_dof; j++) {
			(*(rArg->pangle[j]))[i] = RAD2DEG(rArg->robot->_motor[j].theta);
		}

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
	for (int i = 0; i < rArg->robot->_dof; i++) {
		rArg->robot->_rec_active[i] = false;
	}
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

void* Robot::recordxyBeginThread(void *arg) {
	// cast arg struct
	recArg_t *rArg = (recArg_t *)arg;

	// create initial time points
	int time = (int)((g_sim->getClock())*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rArg->robot->_active_mutex);
	rArg->robot->_rec_active[0] = true;
	rArg->robot->_rec_active[1] = true;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rArg->robot->_recording[JOINT1]; i++) {
		// store locally num of data points taken
		rArg->robot->_rec_num[0] = i;

		// resize array if filled current one
		if (i >= rArg->num) {
			rArg->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newbuf = new double[rArg->num];
			memcpy(newbuf, *rArg->ptime, sizeof(double)*i);
			delete *(rArg->ptime);
			*(rArg->ptime) = newbuf;
			// create larger array for angle
			newbuf = new double[rArg->num];
			memcpy(newbuf, *(rArg->pangle), sizeof(double)*i);
			delete (*(rArg->pangle));
			*(rArg->pangle[0]) = newbuf;
		}

		// store positions
		if (rArg->robot->_trace) {
			(*(rArg->ptime))[i] = rArg->robot->getCenter(0);
			(*(rArg->pangle[0]))[i] = rArg->robot->getCenter(1);
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
	rArg->robot->_rec_active[0] = false;
	rArg->robot->_rec_active[1] = false;
	COND_SIGNAL(&rArg->robot->_active_cond);
	MUTEX_UNLOCK(&rArg->robot->_active_mutex);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

