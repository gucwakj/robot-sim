#include "nxt.h"

CNXT::CNXT() {
	// initialize parameters
	this->initParams(0, NXT);

	// initialize dimensions
	this->initDims();
}

CNXT::~CNXT(void) {
	// remove robot from simulation
	if ( g_sim != NULL && !(g_sim->deleteRobot(this)) )
		delete g_sim;

	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}

	// destroy geoms
	if (_connected) {
		for (int i = NUM_PARTS - 1; i >= 0; i--) { delete [] _geom[i]; }
	}
}

int CNXT::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int CNXT::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_CYCLOIDAL;
	_motor[id].goal = DEG2RAD(angle);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CNXT::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_HARMONIC;
	_motor[id].goal = DEG2RAD(angle) - DEG2RAD(2);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CNXT::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = DEG2RAD(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CNXT::accelJointTimeNB(robotJointId_t id, double a, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( a > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	double step = g_sim->getStep();
	if (t == 0)
		_motor[id].timeout = fabs((_motor[id].omega_max-fabs(_motor[id].omega))/DEG2RAD(a)/step);
	else
		_motor[id].timeout = fabs(t/step);

	// set acceleration parameters
	_motor[id].alpha = DEG2RAD(a);
	_motor[id].mode = ACCEL_CONSTANT;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CNXT::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int CNXT::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int CNXT::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(JOINT1, RAD2DEG(d/radius), t);
	this->accelJointCycloidalNB(JOINT2, RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CNXT::driveAccelDistanceNB(double radius, double a, double d) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1, RAD2DEG(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(JOINT2, RAD2DEG(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CNXT::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(JOINT1, RAD2DEG(d/radius), t);
	this->accelJointHarmonicNB(JOINT2, RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CNXT::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(JOINT1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(JOINT2, a0, af, vmax, d/radius);

	// success
	return 0;
}

int CNXT::driveAccelTimeNB(double radius, double a, double t) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1, RAD2DEG(a/radius), t);
	this->accelJointTimeNB(JOINT2, RAD2DEG(a/radius), t);

	// success
	return 0;
}

int CNXT::driveAccelToMaxSpeedNB(double radius, double a) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1, RAD2DEG(a/radius), 0);
	this->accelJointTimeNB(JOINT2, RAD2DEG(a/radius), 0);

	// success
	return 0;
}

int CNXT::driveAccelToVelocityNB(double radius, double a, double v) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1, RAD2DEG(a/radius), v/a);
	this->accelJointTimeNB(JOINT2, RAD2DEG(a/radius), v/a);

	// success
	return 0;
}

int CNXT::driveBackward(double angle) {
	this->driveBackwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CNXT::driveBackwardNB(double angle) {
	this->moveNB(-angle, angle);

	// success
	return 0;
}

int CNXT::driveDistance(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));
	this->moveWait();

	// success
	return 0;
}

int CNXT::driveDistanceNB(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));

	// success
	return 0;
}

int CNXT::driveForeverNB(void) {
	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT2);

	// success
	return 0;
}

int CNXT::driveForward(double angle) {
	this->driveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CNXT::driveForwardNB(double angle) {
	this->moveNB(angle, -angle);

	// success
	return 0;
}

int CNXT::driveTime(double seconds) {
	// move joint
	this->driveForeverNB();

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoints();

	// success
	return 0;
}

void* CNXT::driveTimeNBThread(void *arg) {
	// cast argument
	recArg_t *rArg = (recArg_t *)arg;

	// get robot
	CNXT *robot = dynamic_cast<CNXT *>(rArg->robot);
	// sleep
	robot->doze(rArg->msecs);
	// hold all robot motion
	robot->holdJoints();

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CNXT::driveTimeNB(double seconds) {
	// set up threading
	THREAD_T moving;
	recArg_t *rArg = new recArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;

	// set joint movements
	this->driveForeverNB();

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&CNXT::driveTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int CNXT::drivexy(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// move to new global coordinates
	return this->drivexyTo(x + x0, y + y0, radius, trackwidth);
}

void* CNXT::drivexyThread(void *arg) {
	// cast arg
	nxtMoveArg_t *mArg = (nxtMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexy(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CNXT::drivexyNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	nxtMoveArg_t *mArg = new nxtMoveArg_t;
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

int CNXT::drivexyTo(double x, double y, double radius, double trackwidth) {
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
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1]);

	if (fabs(speed[0]) > 120) {
		this->setJointSpeeds(45, 45);
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
		this->setJointSpeeds(45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1]);

	// move along length of line
	this->getxy(x0, y0);
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// clean up
	delete speed;

	// success
	return 0;
}

void* CNXT::drivexyToThread(void *arg) {
	// cast arg
	nxtMoveArg_t *mArg = (nxtMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyTo(mArg->x, mArg->y, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CNXT::drivexyToNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	nxtMoveArg_t *mArg = new nxtMoveArg_t;
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

int CNXT::drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
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

void* CNXT::drivexyToFuncThread(void *arg) {
	// cast arg
	nxtMoveArg_t *mArg = (nxtMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyToFunc(mArg->x, mArg->y, mArg->i, mArg->func, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CNXT::drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	nxtMoveArg_t *mArg = new nxtMoveArg_t;
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

int CNXT::drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
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

void* CNXT::drivexyToPolyThread(void *arg) {
	// cast arg
	nxtMoveArg_t *mArg = (nxtMoveArg_t *)arg;

	// perform motion
	mArg->robot->drivexyToPoly(mArg->x, mArg->y, mArg->i, mArg->expr, mArg->radius, mArg->trackwidth);

	// signal successful completion
	SIGNAL(&mArg->robot->_motion_cond, &mArg->robot->_motion_mutex, mArg->robot->_motion = false);

	// cleanup
	delete mArg;

	// success
	return NULL;
}

int CNXT::drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// create thread
	THREAD_T move;

	// store args
	nxtMoveArg_t *mArg = new nxtMoveArg_t;
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

int CNXT::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z) {
	// output current accel data
	accel_x = _accel[0];
	accel_y = _accel[1];
	accel_z = _accel[2];

	// success
	return 0;
}

int CNXT::getLEDColorName(char color[]) {
	rgbHashTable *rgbTable = HT_Create();
	int getRGB[3] = {(int)(255*_rgb[0]), (int)(255*_rgb[1]), (int)(255*_rgb[2])};
	int retval = HT_GetKey(rgbTable, getRGB, color);
	HT_Destroy(rgbTable);

	// success
	return retval;
}

int CNXT::getLEDColorRGB(int &r, int &g, int &b) {
	r = (int)(255*_rgb[0]);
	g = (int)(255*_rgb[1]);
	b = (int)(255*_rgb[2]);

	// success
	return 0;
}

int CNXT::getJointAngles(double &angle1, double &angle2, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);

	// success
	return 0;
}

int CNXT::getJointAnglesInstant(double &angle1, double &angle2) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);

	// success
	return 0;
}

int CNXT::getJointSpeeds(double &speed1, double &speed2) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);

	// success
	return 0;
}

int CNXT::getJointSpeedRatios(double &ratio1, double &ratio2) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;

	// success
	return 0;
}

int CNXT::jumpTo(double angle1, double angle2) {
	this->jumpToNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::jumpToNB(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);

	// success
	return 0;
}

int CNXT::move(double angle1, double angle2) {
	this->moveNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::moveNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CNXT::moveTo(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::moveToNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CNXT::recordAngles(double *time, double *angle1, double *angle2, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CNXT::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CNXT::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CNXT::setJointSpeeds(double speed1, double speed2) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);

	// success
	return 0;
}

int CNXT::setJointSpeedRatios(double ratio1, double ratio2) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);

	// success
	return 0;
}

int CNXT::setSpeed(double speed, double radius) {
	if (RAD2DEG(speed/radius) > RAD2DEG(_motor[JOINT1].omega_max)) {
		fprintf(stderr, "Warning: Speed %.2lf corresponds to joint speeds of %.2lf degrees/second.\n",
			speed, RAD2DEG(speed/radius));
	}
	this->setJointSpeed(JOINT1, RAD2DEG(speed/radius));
	this->setJointSpeed(JOINT2, RAD2DEG(speed/radius));

	// success
	return 0;
}

int CNXT::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CNXT::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(-angle, -angle);

	// success
	return 0;
}

int CNXT::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int CNXT::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(angle, angle);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CNXT::build(xml_robot_t robot) {
	// adjust height for wheels
	robot->z += (_wheel_radius - _body_height/2);
	_radius = _wheel_radius;
	if (fabs(robot->z) > (_body_radius-EPSILON)) { robot->z += _body_height/2; }

	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta,	-cphi*stheta*spsi - sphi*cpsi,	-cphi*stheta*cpsi + sphi*spsi,	0,
				  sphi*ctheta,	-sphi*stheta*spsi + cphi*cpsi,	-sphi*stheta*cpsi - cphi*spsi,	0,
				  stheta,		ctheta*spsi,					ctheta*cpsi,					0};
	// build robot
	double rot[2] = {robot->angle1, robot->angle2};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// set trackwidth
	const double *pos1, *pos2;
	pos1 = dBodyGetPosition(_body[WHEEL1]);
	pos2 = dBodyGetPosition(_body[WHEEL2]);
	_trackwidth = sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2));

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int CNXT::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
	_geom[BODY] = new dGeomID[1];
	_geom[WHEEL1] = new dGeomID[1];
	_geom[WHEEL2] = new dGeomID[1];

	// adjust input height by body height
	if (fabs(z) < (_body_radius-EPSILON)) { z += _body_height/2; }

    // convert input angles to radians
    _motor[JOINT1].theta = DEG2RAD(rot[JOINT1]);
    _motor[JOINT2].theta = DEG2RAD(rot[JOINT2]);

	// set goal to current angle
	_motor[JOINT1].goal = _motor[JOINT1].theta;
	_motor[JOINT2].goal = _motor[JOINT2].theta;

	// offset values for each body part[0-2] and joint[3-5] from center
	double f1[6] = {-_body_width/2 - _wheel_depth/2, 0, 0, -_body_width/2, 0, 0};
	double f2[6] = {_body_width/2 + _wheel_depth/2, 0, 0, _body_width/2, 0, 0};

	// build robot bodies
	this->build_body(x, y, z, R, 0);
	this->build_wheel(WHEEL1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R, 0);
	this->build_wheel(WHEEL2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R, 0);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = 0;
	_center[2] = 0;

    // joint for body to wheel 1
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY], _body[WHEEL1]);
	dJointSetHingeAnchor(_joint[0], R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x,
									R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
									R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[WHEEL1], R[0], R[4], R[8]);

    // joint for body to wheel2
	_joint[1] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[1], _body[BODY], _body[WHEEL2]);
	dJointSetHingeAnchor(_joint[1], R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
									R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
									R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
	dJointSetHingeAxis(_joint[1], -R[0], -R[4], -R[8]);
	dBodySetFiniteRotationAxis(_body[WHEEL2], -R[0], -R[4], -R[8]);

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2;
    dRFromAxisAndAngle(R_f, -1, 0, 0, _motor[JOINT1].theta);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 1, 0, 0, _motor[JOINT2].theta);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 ) {
		this->build_wheel(WHEEL1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R_f1, 0);
		this->build_wheel(WHEEL2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R_f2, 0);
	}

    // motor for body to wheel 1
    _motor[JOINT1].id = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[JOINT1].id, _body[BODY], _body[WHEEL1]);
    dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
    dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
    dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT1].id);

	// motor for body to wheel 2
	_motor[JOINT2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT2].id, _body[BODY], _body[WHEEL2]);
	dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT2].id);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int CNXT::draw(osg::Group *root, int tracking) {
	// initialize variables
	_robot = new osg::Group();
	osg::ref_ptr<osg::Geode> body[NUM_PARTS+1];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[NUM_PARTS+1];
	osg::ref_ptr<osg::Texture2D> tex;
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
		body[NUM_PARTS]->addDrawable(_led);
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1));
	}

	// wheel1
	pos = dGeomGetOffsetPosition(_geom[WHEEL1][0]);
	dGeomGetOffsetQuaternion(_geom[WHEEL1][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _wheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[WHEEL1]->addDrawable(new osg::ShapeDrawable(cyl));

	// wheel2
	pos = dGeomGetOffsetPosition(_geom[WHEEL2][0]);
	dGeomGetOffsetQuaternion(_geom[WHEEL2][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _wheel_radius, _wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[WHEEL2]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/textures/body.png)));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

	for (int i = 0; i < NUM_PARTS+1; i++) {
		// set rendering properties
		body[i]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		// position in robot
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robot->addChild(pat[i].get());
	}

	// set update callback for robot
	_robot->setUpdateCallback(new nxtNodeCallback(this));

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
	label->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	label->setBoundingBoxColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.9f));
	label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	label->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX);
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

double CNXT::getAngle(int id) {
	_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;
    return _motor[id].theta;
}

int CNXT::initParams(int disabled, int type) {
	_dof = 2;

	// create arrays for linkbots
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[_dof];
	_geom = new dGeomID * [NUM_PARTS];
	_joint = new dJointID[_dof];
	_motor = new struct motor_s[_dof];
	_rec_active = new bool[_dof];
	_rec_angles = new double ** [_dof];
	_rec_num = new int[_dof];
	_recording = new bool[_dof];

	// fill with default data
	for (int i = 0, j = 0; i < _dof; i++) {
		_motor[i].accel.init = 0;
		_motor[i].accel.run = 0;
		_motor[i].accel.period = 0;
		_motor[i].accel.start = 0;
		_motor[i].alpha = 0;
		_motor[i].encoder = DEG2RAD(0.25);
		_motor[i].goal = 0;
		_motor[i].mode = SEEK;
		_motor[i].offset = 0;
		_motor[i].omega = 0.7854;			//  45 deg/sec
		_motor[i].omega_max = 4.1888;		// 240 deg/sec
		_motor[i].safety_angle = 10;
		_motor[i].safety_timeout = 4;
		_motor[i].starting = 0;
		_motor[i].state = NEUTRAL;
		_motor[i].stopping = 0;
		_motor[i].success = true;
		_motor[i].tau_max = 2;
		_motor[i].timeout = 0;
		_motor[i].theta = 0;
		MUTEX_INIT(&_motor[i].success_mutex);
		COND_INIT(&_motor[i].success_cond);
		_rec_active[i] = false;
		_rec_num[i] = 0;
		_recording[i] = false;
	}
	_connected = 0;
	_distOffset = 0;
	_id = -1;
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

int CNXT::initDims(void) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_radius = _body_height/2;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;

	// success
	return 0;
}

void CNXT::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[BODY]);
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
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_joint[i], axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
		// engage motor depending upon motor mode
		double t = 0, angle = 0, h = 0, dt = 0;
		double step = g_sim->getStep();
		switch (_motor[i].mode) {
			case ACCEL_CONSTANT:
				// check if done with acceleration
				if (_motor[i].timeout) {
					_motor[i].timeout--;
				}
				else {
					_motor[i].mode = CONTINUOUS;
					if (_motor[i].omega > 0) _motor[i].state = POSITIVE;
					else if (_motor[i].omega < 0) _motor[i].state = NEGATIVE;
					_motor[i].timeout = -1;
				}

				// set new theta
				_motor[i].goal += step*_motor[i].omega;
				if (_motor[i].omega <= _motor[i].omega_max) {
					_motor[i].goal += _motor[i].alpha*step*step/2;
				}

				// move to new theta
				dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].omega);

				// update omega
				_motor[i].omega += step * _motor[i].alpha;
				if (_motor[i].omega > _motor[i].omega_max)
					_motor[i].omega = _motor[i].omega_max;
				else if (_motor[i].omega < -_motor[i].omega_max)
					_motor[i].omega = -_motor[i].omega_max;
				break;
			case ACCEL_CYCLOIDAL:
			case ACCEL_HARMONIC:
				// init params on first run
				if (_motor[i].accel.run == 0) {
					_motor[i].accel.init = _motor[i].theta;
					_motor[i].accel.start = g_sim->getClock();
					_motor[i].accel.run = 1;
					break;
				}

				// calculate new angle
				h = _motor[i].goal - _motor[i].accel.init;
				t = g_sim->getClock();
				dt = (t - _motor[i].accel.start)/_motor[i].accel.period;
				if (_motor[i].mode == ACCEL_CYCLOIDAL)
					angle = h*(dt - sin(2*M_PI*dt)/2/M_PI) + _motor[i].accel.init;
				else if (_motor[i].mode == ACCEL_HARMONIC)
					angle = h*(1 - cos(M_PI*dt))/2 + _motor[i].accel.init;

				// set new omega
				_motor[i].omega = (angle - _motor[i].theta)/step;

				// give it an initial push
				if (0 < _motor[i].omega && _motor[i].omega < 0.01)
					_motor[i].omega = 0.01;
				else if (-0.01 < _motor[i].omega && _motor[i].omega < 0)
					_motor[i].omega = -0.01;

				// move until timeout is reached
				if (_motor[i].timeout) {
					dJointEnable(_motor[i].id);
					dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].omega);
					_motor[i].timeout--;
				}
				else {
					_motor[i].mode = CONTINUOUS;
					if (_motor[i].omega > 0) _motor[i].state = POSITIVE;
					else if (_motor[i].omega < 0) _motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
					_motor[i].accel.run = 0;
					_motor[i].timeout = -1;
				}
				break;
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
				if ((_motor[i].goal - 6*_motor[i].encoder - _motor[i].theta) > EPSILON) {
					_motor[i].state = POSITIVE;
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
				}
				else if ((_motor[i].goal - 3*_motor[i].encoder - _motor[i].theta) > EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].goal - _motor[i].encoder - _motor[i].theta) > EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/4);
				}
				else if ((_motor[i].theta - _motor[i].goal - 6*_motor[i].encoder) > EPSILON) {
					_motor[i].state = NEGATIVE;
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
				}
				else if ((_motor[i].theta - _motor[i].goal - 3*_motor[i].encoder) > EPSILON) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder) > EPSILON) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/4);
				}
				else {
					_motor[i].state = HOLD;
					_motor[i].starting = 0;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
				}
				break;
		}
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void CNXT::simPostCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);
	MUTEX_LOCK(&_success_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < _dof; i++) {
		// lock mutex
		MUTEX_LOCK(&_motor[i].success_mutex);
		// zero velocity == stopped
		_motor[i].stopping += (!(int)(dJointGetAMotorParam(this->_motor[i].id, dParamVel)*1000) );
		// once motor has been stopped for 10 steps
		if (_motor[i].stopping == 50) {
			_motor[i].stopping = 0;
			_motor[i].success = 1;
		}
		// signal success
		if (_motor[i].success)
			COND_SIGNAL(&_motor[i].success_cond);
		// unlock mutex
		MUTEX_UNLOCK(&_motor[i].success_mutex);
	}

	// all joints have completed their motions
	if (_motor[JOINT1].success && _motor[JOINT2].success)
		COND_SIGNAL(&_success_cond);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
int CNXT::build_body(double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m, m1, m2;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);

	// adjust x,y,z to position center of mass correctly
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
	dGeomSetOffsetPosition(_geom[BODY][0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[BODY], &m);

	// success
	return 0;
}

int CNXT::build_wheel(int id, double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of wheel
	dMassSetCylinder(&m, 270, 1, 2*_wheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[id], x, y, z);
	dBodySetRotation(_body[id], R);
	dBodySetFiniteRotationMode(_body[id], 1);

	// rotation matrix
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry
	_geom[id][0] = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(_geom[id][0], _body[id]);
	dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[id][0], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// success
	return 0;
}

