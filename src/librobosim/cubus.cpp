#include "cubus.h"

Cubus::Cubus(void) : Robot(JOINT1, JOINT3) {
	// initialize parameters
	this->initParams(0, CUBUS);

	// initialize dimensions
	this->initDims();
}

Cubus::~Cubus(void) {
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

int Cubus::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int Cubus::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
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

int Cubus::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
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

int Cubus::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = DEG2RAD(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int Cubus::accelJointTimeNB(robotJointId_t id, double a, double t) {
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

int Cubus::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int Cubus::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int Cubus::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointCycloidalNB(JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int Cubus::driveAccelDistanceNB(double radius, double a, double d) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int Cubus::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointHarmonicNB(JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int Cubus::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(JOINT1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(JOINT3, a0, af, vmax, d/radius);

	// success
	return 0;
}

int Cubus::driveAccelTimeNB(double radius, double a, double t) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), t);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), t);

	// success
	return 0;
}

int Cubus::driveAccelToMaxSpeedNB(double radius, double a) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), 0);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), 0);

	// success
	return 0;
}

int Cubus::driveAccelToVelocityNB(double radius, double a, double v) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), v/a);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), v/a);

	// success
	return 0;
}

int Cubus::driveForeverNB(void) {
	// negate speed to act as a car
	_motor[JOINT3].omega = -_motor[JOINT3].omega;

	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT3);

	// success
	return 0;
}

int Cubus::driveForwardNB(double angle) {
	this->moveJointNB(JOINT1, angle);
	this->moveJointNB(JOINT3, -angle);

	// success
	return 0;
}

int Cubus::drivexyTo(double x, double y, double radius, double trackwidth) {
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
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// clean up
	delete speed;

	// success
	return 0;
}

int Cubus::getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);
	this->getJointAngle(JOINT3, angle3, numReadings);

	// success
	return 0;
}

int Cubus::getJointAnglesInstant(double &angle1, double &angle2, double &angle3) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);
	this->getJointAngleInstant(JOINT3, angle3);

	// success
	return 0;
}

int Cubus::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);
	speed3 = RAD2DEG(_motor[JOINT3].omega);

	// success
	return 0;
}

int Cubus::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;
	ratio3 = _motor[JOINT3].omega/_motor[JOINT3].omega_max;

	// success
	return 0;
}

int Cubus::move(double angle1, double angle2, double angle3) {
	this->moveNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int Cubus::moveNB(double angle1, double angle2, double angle3) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int Cubus::moveTo(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int Cubus::moveToNB(double angle1, double angle2, double angle3) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int Cubus::moveToByTrackPos(double angle1, double angle2, double angle3) {
	this->moveToByTrackPosNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int Cubus::moveToByTrackPosNB(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);

	// success
	return 0;
}

int Cubus::recordAngles(double *time, double *angle1, double *angle2, double *angle3, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int Cubus::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int Cubus::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_recording[i]) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;
	angles[JOINT3] = distance3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int Cubus::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);

	// success
	return 0;
}

int Cubus::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);

	// success
	return 0;
}

int Cubus::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(-angle, 0, -angle);

	// success
	return 0;
}

int Cubus::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = (g_sim->getUnits()) ? _trackwidth*39.37 : _trackwidth*100;

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
int Cubus::addConnector(int type, int face, double size) {
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
	this->build_simple(nc, face);

	// success
	return 0;
}

int Cubus::build(xml_robot_t robot) {
	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta,	-cphi*stheta*spsi - sphi*cpsi,	-cphi*stheta*cpsi + sphi*spsi,	0,
				  sphi*ctheta,	-sphi*stheta*spsi + cphi*cpsi,	-sphi*stheta*cpsi - cphi*spsi,	0,
				  stheta,		ctheta*spsi,					ctheta*cpsi,					0};

	// build robot
	double rot[3] = {robot->angle1, robot->angle2, robot->angle3};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// add connectors
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		this->addConnector(ctmp->type, ctmp->face1, ctmp->size);
		ctmp = ctmp->next;
	}

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int Cubus::build(xml_robot_t robot, dMatrix3 R, double *m, dBodyID base, xml_conn_t conn) {
	// initialize new variables
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, R5, R6;

	// generate parameters for connector
	this->getConnectorParams(conn->type, conn->side, R, m);

	// rotate about connection joint
	dRFromAxisAndAngle(R1, R[0], R[4], R[8], robot->psi);
	dMultiply0(R2, R1, R, 3, 3, 3);

	// rotate body for connection face
	switch (conn->face2) {
		case 1:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], 0);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(robot->angle1));
			break;
		case 2:
			offset[0] = _face_depth + _body_length;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], -M_PI/2);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -DEG2RAD(robot->angle2));
			break;
		case 3:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], M_PI);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(robot->angle3));
			break;
	}
	m[0] += R[0]*offset[0];
	m[1] += R[4]*offset[0];
	m[2] += R[8]*offset[0];
	dMultiply0(R6, R5, R4, 3, 3, 3);

    // build new module
	double rot[3] = {robot->angle1, robot->angle2, robot->angle3};
	this->buildIndividual(m[0], m[1], m[2], R6, rot);

    // add fixed joint to attach two modules
	this->fixBodyToConnector(base, conn->face2);

	// add connectors
	xml_conn_t ctmp = robot->conn;
	while (ctmp) {
		if (ctmp->robot == _id)
			this->addConnector(ctmp->type, ctmp->face1, ctmp->size);
		else if (ctmp->face2 != conn->face2)
			this->fixConnectorToBody(ctmp->face2, base);
		ctmp = ctmp->next;
	}

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int Cubus::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
	_geom[BODY] = new dGeomID[1];
	_geom[FACE1] = new dGeomID[1];
	_geom[FACE2] = new dGeomID[1];
	_geom[FACE3] = new dGeomID[1];
	_geom[FACE4] = new dGeomID[1];
	_geom[FACE5] = new dGeomID[1];
	_geom[FACE6] = new dGeomID[1];

	// adjust input height by body height
	if (fabs(z) < (_body_height-EPSILON)) {z += _body_height/2; }

    // convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].theta = DEG2RAD(rot[i]);
		_motor[i].goal = _motor[i].theta;
	}

	// offset values for each body part[0-2] and joint[3-5] from center
	double f1[6] = {-_body_width/2 - _face_depth/2, 0, 0, -_body_width/2, 0, 0};
	double f2[6] = {0, -_body_length/2 - _face_depth/2, 0, 0, -_body_length/2, 0};
	double f3[6] = {_body_width/2 + _face_depth/2, 0, 0, _body_width/2, 0, 0};
	double f4[6] = {0, _body_length/2 + _face_depth/2, 0, 0, _body_length/2, 0};
	double f5[6] = {0, 0, -_body_height/2 - _face_depth/2, 0, 0, -_body_height/2};
	double f6[6] = {0, 0, _body_height/2 + _face_depth/2, 0, 0, _body_height/2};

	// build robot bodies
	this->build_body(x, y, z, R, 0);
	this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R, 0);
	this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R, 0);
	this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R, 0);
	this->build_face(FACE4, R[1]*f4[1] + x, R[5]*f4[1] + y, R[9]*f4[1] + z, R, 0);
	this->build_face(FACE5, R[2]*f5[2] + x, R[6]*f5[2] + y, R[10]*f5[2] + z, R, 0);
	this->build_face(FACE6, R[2]*f6[2] + x, R[6]*f6[2] + y, R[10]*f6[2] + z, R, 0);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = 0;
	_center[2] = 0;

	// joint for body to face 1
	_joint[JOINT1] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT1], _body[BODY], _body[FACE1]);
	dJointSetHingeAnchor(_joint[JOINT1],R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x,
										R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
										R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
	dJointSetHingeAxis(_joint[JOINT1], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[FACE1], R[0], R[4], R[8]);

	// joint for body to face 2
	_joint[JOINT2] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT2], _body[BODY], _body[FACE2]);
	dJointSetHingeAnchor(_joint[JOINT2],R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
										R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
										R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
	dJointSetHingeAxis(_joint[JOINT2], R[1], R[5], R[9]);
	dBodySetFiniteRotationAxis(_body[FACE2], R[1], R[5], R[9]);

	// joint for body to face 3
	_joint[JOINT3] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT3], _body[BODY], _body[FACE3]);
	dJointSetHingeAnchor(_joint[JOINT3],R[0]*f3[3] + R[1]*f3[4] + R[2]*f3[5] + x,
										R[4]*f3[3] + R[5]*f3[4] + R[6]*f3[5] + y,
										R[8]*f3[3] + R[9]*f3[4] + R[10]*f3[5] + z);
	dJointSetHingeAxis(_joint[JOINT3], -R[0], -R[4], -R[8]);
	dBodySetFiniteRotationAxis(_body[FACE3], -R[0], -R[4], -R[8]);

	// joint for body to face 4
	_joint[JOINT4] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT4], _body[BODY], _body[FACE4]);
	dJointSetHingeAnchor(_joint[JOINT4],R[0]*f4[3] + R[1]*f4[4] + R[2]*f4[5] + x,
										R[4]*f4[3] + R[5]*f4[4] + R[6]*f4[5] + y,
										R[8]*f4[3] + R[9]*f4[4] + R[10]*f4[5] + z);
	dJointSetHingeAxis(_joint[JOINT4], -R[1], -R[5], -R[9]);
	dBodySetFiniteRotationAxis(_body[FACE4], -R[1], -R[5], -R[9]);

	// joint for body to face 5
	_joint[JOINT5] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT5], _body[BODY], _body[FACE5]);
	dJointSetHingeAnchor(_joint[JOINT5],R[0]*f5[3] + R[1]*f5[4] + R[2]*f5[5] + x,
										R[4]*f5[3] + R[5]*f5[4] + R[6]*f5[5] + y,
										R[8]*f5[3] + R[9]*f5[4] + R[10]*f5[5] + z);
	dJointSetHingeAxis(_joint[JOINT5], R[2], R[6], R[10]);
	dBodySetFiniteRotationAxis(_body[FACE5], R[2], R[6], R[10]);
	// joint for body to face 6
	_joint[JOINT6] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[JOINT6], _body[BODY], _body[FACE6]);
	dJointSetHingeAnchor(_joint[JOINT6],R[0]*f6[3] + R[1]*f6[4] + R[2]*f6[5] + x,
										R[4]*f6[3] + R[5]*f6[4] + R[6]*f6[5] + y,
										R[8]*f6[3] + R[9]*f6[4] + R[10]*f6[5] + z);
	dJointSetHingeAxis(_joint[JOINT6], -R[2], -R[6], -R[10]);
	dBodySetFiniteRotationAxis(_body[FACE6], -R[2], -R[6], -R[10]);

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2, R_f3;
    dRFromAxisAndAngle(R_f, -1, 0, 0, _motor[JOINT1].theta);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 0, -1, 0, _motor[JOINT2].theta);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 1, 0, 0, _motor[JOINT3].theta);
    dMultiply0(R_f3, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 || _motor[JOINT3].theta != 0 ) {
		this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R_f1, 0);
		this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R_f2, 0);
		this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R_f3, 0);
	}

    // motor for body to face 1
	_motor[JOINT1].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT1].id, _body[BODY], _body[FACE1]);
	dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
	dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R[0], R[4], R[8]);
	dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
	dJointSetAMotorParam(_motor[JOINT1].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT1].id);

	// motor for body to face 2
	_motor[JOINT2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT2].id, _body[BODY], _body[FACE2]);
	dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, R[1], R[5], R[9]);
	dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT2].id);

	// motor for body to face 3
	_motor[JOINT3].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT3].id, _body[BODY], _body[FACE3]);
	dJointSetAMotorMode(_motor[JOINT3].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT3].id, 1);
	dJointSetAMotorAxis(_motor[JOINT3].id, 0, 1, -R[0], -R[4], -R[8]);
	dJointSetAMotorAngle(_motor[JOINT3].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT3].id, dParamFMax, _motor[JOINT3].tau_max);
	dJointSetAMotorParam(_motor[JOINT3].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT3].id);

	// motor for body to face 4
	_motor[JOINT4].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT4].id, _body[BODY], _body[FACE4]);
	dJointSetAMotorMode(_motor[JOINT4].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT4].id, 1);
	dJointSetAMotorAxis(_motor[JOINT4].id, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(_motor[JOINT4].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT4].id, dParamFMax, _motor[JOINT4].tau_max);
	dJointSetAMotorParam(_motor[JOINT4].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT4].id);

	// motor for body to face 5
	_motor[JOINT5].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT5].id, _body[BODY], _body[FACE5]);
	dJointSetAMotorMode(_motor[JOINT5].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT5].id, 1);
	dJointSetAMotorAxis(_motor[JOINT5].id, 0, 1, R[2], R[6], R[10]);
	dJointSetAMotorAngle(_motor[JOINT5].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT5].id, dParamFMax, _motor[JOINT5].tau_max);
	dJointSetAMotorParam(_motor[JOINT5].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT5].id);

	// motor for body to face 6
	_motor[JOINT6].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT6].id, _body[BODY], _body[FACE6]);
	dJointSetAMotorMode(_motor[JOINT6].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT6].id, 1);
	dJointSetAMotorAxis(_motor[JOINT6].id, 0, 1, -R[2], -R[6], -R[10]);
	dJointSetAMotorAngle(_motor[JOINT6].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT6].id, dParamFMax, _motor[JOINT6].tau_max);
	dJointSetAMotorParam(_motor[JOINT6].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT6].id);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

#ifdef ENABLE_GRAPHICS
int Cubus::draw(osg::Group *root, int tracking) {
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
		body[NUM_PARTS]->addDrawable(_led);
		_led->setColor(osg::Vec4(_rgb[0], _rgb[1], _rgb[2], 1));
	}

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

	// face 4
	pos = dGeomGetOffsetPosition(_geom[FACE4][0]);
	dGeomGetOffsetQuaternion(_geom[FACE4][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE4]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 5
	pos = dGeomGetOffsetPosition(_geom[FACE5][0]);
	dGeomGetOffsetQuaternion(_geom[FACE5][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE5]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 4
	pos = dGeomGetOffsetPosition(_geom[FACE6][0]);
	dGeomGetOffsetQuaternion(_geom[FACE6][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[FACE6]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex[0] = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/textures/body.png)));
	tex[1] = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/textures/face.png)));
	for (int i = 0; i < 2; i++) {
		tex[i]->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex[i]->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex[i]->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex[i]->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	}
	body[BODY]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	body[FACE1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[FACE2]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[FACE3]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[FACE4]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[FACE5]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[FACE6]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[NUM_PARTS]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);

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
		this->drawConnector(ctmp, _robot);
		ctmp = ctmp->next;
	}

	// set update callback for robot
	_robot->setUpdateCallback(new cubusNodeCallback(this));

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

int Cubus::drawConnector(conn_t conn, osg::Group *robot) {
	// initialize variables
	dMatrix3 R;
	dQuaternion Q;
	double p[3] = {0};

	// get connection parameters
	this->getFaceParams(conn->face, R, p);
	dRtoQ(R, Q);
	if (conn->d_side != -1) this->getConnectorParams(conn->d_type, conn->d_side, R, p);

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	transform->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));

	// create node to hold mesh
	osg::ref_ptr<osg::Node> geode = osgDB::readNodeFile(TEXTURE_PATH(linkbot/models/simple.stl));
	geode->setCullingActive(false);

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(TEXTURE_PATH(linkbot/textures/conn.png)));
	tex->setDataVariance(osg::Object::DYNAMIC);
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexEnv> texEnv = new osg::TexEnv(osg::TexEnv::DECAL);
	geode->getOrCreateStateSet()->setTextureAttribute(0, texEnv, osg::StateAttribute::ON);

	// set rendering
	geode->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add body to pat
	transform->addChild(geode);

	// set user properties of node
	geode->setName("connector");

	// add to scenegraph
	robot->addChild(transform);

	// success
	return 0;
}
#endif // ENABLE_GRAPHICS

int Cubus::fixBodyToConnector(dBodyID cBody, int face) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int Cubus::fixConnectorToBody(int face, dBodyID cBody, int conn) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// connector or body part
	dBodyID body;
	if (conn != -1)
		body = this->getConnectorBodyID(face);
	else
		body = this->getBodyID(face);

	// attach to correct body
	dJointAttach(joint, body, cBody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

double Cubus::getAngle(int id) {
	return _motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;
}

int Cubus::getConnectorParams(int type, int side, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};

	// connection dimensions
	offset[0] = _conn_depth;
	dRSetIdentity(R1);

	// set output parameters
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int Cubus::getFaceParams(int face, dMatrix3 R, double *p) {
	double offset[3] = {0};
	const double *pos = dBodyGetPosition(_body[face]);
	const double *R1 = dBodyGetRotation(_body[face]);
	dMatrix3 R2;

	// get offset and rotation of face connection
	switch (face) {
		case FACE1:
			offset[0] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI);
			break;
		case FACE2:
			offset[1] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], -M_PI/2);
			break;
		case FACE3:
			offset[0] = _face_depth/2;
			dRSetIdentity(R2);
			break;
		case FACE4:
			offset[1] = _face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI/2);
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

int Cubus::initParams(int disabled, int type) {
	_dof = 6;

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
	_disabled = disabled;
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

int Cubus::initDims(void) {
	_body_length = 0.07835;
	_body_width = 0.07835;
	_body_height = 0.07835;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_conn_depth = 0.00380;
	_conn_height = 0.03715;
	_radius = _body_height/2;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;

	// success
	return 0;
}

void Cubus::simPreCollisionThread(void) {
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

void Cubus::simPostCollisionThread(void) {
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
	if (_motor[JOINT1].success && _motor[JOINT2].success && _motor[JOINT3].success)
		COND_SIGNAL(&_success_cond);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
int Cubus::build_body(double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, 0, 0, 0);

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

int Cubus::build_face(int id, double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of body
	if (id == 1 || id == 3)
		dMassSetCylinder(&m, 270, 1, 2*_face_radius, _face_depth);
	else if (id == 2 || id == 4)
		dMassSetCylinder(&m, 270, 2, 2*_face_radius, _face_depth);
	else if (id == 5 || id == 6)
		dMassSetCylinder(&m, 270, 3, 2*_face_radius, _face_depth);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[id], x, y, z);
	dBodySetRotation(_body[id], R);
	dBodySetFiniteRotationMode(_body[id], 1);

	// rotation matrix
	if (id == 1 || id == 3)
	    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	else if (id == 2 || id == 4)
	    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	else if (id == 5 || id == 6)
	    dRFromAxisAndAngle(R1, 0, 0, 1, M_PI/2);		// SWITCHED X AND Y AXIS
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

int Cubus::build_simple(conn_t conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0}, offset[3] = {_conn_depth/2, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 270, _conn_depth, 2*_face_radius, _conn_height);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

