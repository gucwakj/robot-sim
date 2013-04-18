#include "mobotsim.h"

CRobot4::CRobot4(void) {
	for (int i = 0; i < NUM_DOF; i++) {
		_angle[i] = 0;
		_goal[i] = 0;
		_recording[i] = false;
		_success[i] = true;
		_velocity[i] = 0.7854;	// 45 deg/sec
		_maxSpeed[i] = 120;		// deg/sec
	}
	_conn = NULL;
	_id = -1;

	// init locks
	this->simThreadsAngleInit();
	this->simThreadsGoalInit();
	this->simThreadsRecordingInit();
	this->simThreadsSuccessInit();
}

CRobot4::~CRobot4(void) {
	//dSpaceDestroy(_space); //sigsegv
}

int CRobot4::connect(void) {
	return 0;
}

int CRobot4::getJointAngle(int id, dReal &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CRobot4::getJointSpeed(int id, dReal &speed) {
	speed = RAD2DEG(_velocity[id]);

	// success
	return 0;
}

int CRobot4::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = RAD2DEG(_velocity[0]);
	speed2 = RAD2DEG(_velocity[1]);
	speed3 = RAD2DEG(_velocity[2]);
	speed4 = RAD2DEG(_velocity[3]);

	// success
	return 0;
}

int CRobot4::motionArch(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, -angle/2.0);
	this->moveJointToNB(MOBOT_JOINT3, angle/2.0);
	this->moveJointWait(MOBOT_JOINT2);
	this->moveJointWait(MOBOT_JOINT3);

	// success
	return 0;
}

int CRobot4::motionInchwormLeft(int num) {
	this->moveJointToNB(MOBOT_JOINT2, 0);
	this->moveJointToNB(MOBOT_JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(MOBOT_JOINT2, -50);
		this->moveJointTo(MOBOT_JOINT3, 50);
		this->moveJointTo(MOBOT_JOINT2, 0);
		this->moveJointTo(MOBOT_JOINT3, 0);
	}

	// success
	return 0;
}

int CRobot4::motionInchwormRight(int num) {
	this->moveJointToNB(MOBOT_JOINT2, 0);
	this->moveJointToNB(MOBOT_JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(MOBOT_JOINT3, 50);
		this->moveJointTo(MOBOT_JOINT2, -50);
		this->moveJointTo(MOBOT_JOINT3, 0);
		this->moveJointTo(MOBOT_JOINT2, 0);
	}

	// success
	return 0;
}

int CRobot4::motionRollBackward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionRollForward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionSkinny(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, angle);
	this->moveJointToNB(MOBOT_JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionStand(void) {
	this->resetToZero();
	//this->moveToZero();
	this->moveJointTo(MOBOT_JOINT2, -85);
	this->moveJointTo(MOBOT_JOINT3, 70);
	this->moveWait();
	this->moveJointTo(MOBOT_JOINT1, 45);
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif
	this->moveJointTo(MOBOT_JOINT2, 20);

	// success
	return 0;
}

int CRobot4::motionTumbleLeft(int num) {
	this->resetToZero();
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif

	for (int i = 0; i < num; i++) {
		this->moveJointTo(MOBOT_JOINT2, -85);
		this->moveJointTo(MOBOT_JOINT3, 80);
		this->moveJointTo(MOBOT_JOINT2, 0);
		this->moveJointTo(MOBOT_JOINT3, 0);
		this->moveJointTo(MOBOT_JOINT2, 80);
		this->moveJointTo(MOBOT_JOINT2, 45);
		this->moveJointTo(MOBOT_JOINT3, -85);
		this->moveJointTo(MOBOT_JOINT2, 80);
		this->moveJointTo(MOBOT_JOINT3, 0);
		this->moveJointTo(MOBOT_JOINT2, 0);
		this->moveJointTo(MOBOT_JOINT3, 80);
		if (i != (num-1)) {
			this->moveJointTo(MOBOT_JOINT3, 45);
		}
	}
	this->moveJointToNB(MOBOT_JOINT2, 0);
	this->moveJointToNB(MOBOT_JOINT3, 0);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionTumbleRight(int num) {
	this->resetToZero();
#ifndef _WIN32
	usleep(1000000);
#else
	Sleep(1000);
#endif

	for (int i = 0; i < num; i++) {
		this->moveJointTo(MOBOT_JOINT3, 85);
		this->moveJointTo(MOBOT_JOINT2, -80);
		this->moveJointTo(MOBOT_JOINT3, 0);
		this->moveJointTo(MOBOT_JOINT2, 0);
		this->moveJointTo(MOBOT_JOINT3, -80);
		this->moveJointTo(MOBOT_JOINT3, -45);
		this->moveJointTo(MOBOT_JOINT2, 85);
		this->moveJointTo(MOBOT_JOINT3, -80);
		this->moveJointTo(MOBOT_JOINT2, 0);
		this->moveJointTo(MOBOT_JOINT3, 0);
		this->moveJointTo(MOBOT_JOINT2, -80);
		if (i != (num-1)) {
			this->moveJointTo(MOBOT_JOINT2, -45);
		}
	}
	this->moveJointToNB(MOBOT_JOINT3, 0);
	this->moveJointToNB(MOBOT_JOINT2, 0);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionTurnLeft(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionTurnRight(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::motionUnstand(void) {
	this->moveToDirect(0, 0, 0, 0);
	this->moveJointToNB(MOBOT_JOINT3, 45);
	this->moveJointToNB(MOBOT_JOINT2, -85);
	this->moveWait();
	this->moveToDirect(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4::move(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	this->moveNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::moveNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	// store angles into array
	dReal delta[4] = {angle1, angle2, angle3, angle4};

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[0] += DEG2RAD(angle1);
	_goal[1] += DEG2RAD(angle2);
	_goal[2] += DEG2RAD(angle3);
	_goal[3] += DEG2RAD(angle4);

	// enable motor
	this->simThreadsAngleLock();
	for ( int j = 0; j < NUM_DOF; j++ ) {
		dJointEnable(_motor[j]);
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = MOBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _velocity[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = MOBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_velocity[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = MOBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
	}
    dBodyEnable(_body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	_success[0] = false;
	_success[1] = false;
	_success[2] = false;
	_success[3] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4::moveJoint(int id, dReal angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot4::moveJointNB(int id, dReal angle) {
	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[id] += DEG2RAD(angle);

	// enable motor
	this->simThreadsAngleLock();
	dJointEnable(_motor[id]);

	// set motor state and velocity
	if ( angle > 0 ) {
		_state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _velocity[id]);
	}
	else if ( angle < 0 ) {
		_state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_velocity[id]);
	}
	else if ( fabs(angle-0) < EPSILON ) {
		_state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	_success[id] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4::moveJointTo(int id, dReal angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot4::moveJointToNB(int id, dReal angle) {
	// store delta angle
	dReal delta = angle - _angle[id];

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[id] = DEG2RAD(angle);

	// enable motor
	this->simThreadsAngleLock();
	dJointEnable(_motor[id]);

	// set motor state and velocity
	if ( delta > 0 ) {
		_state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _velocity[id]);
	}
	else if ( delta < 0 ) {
		_state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_velocity[id]);
	}
	else if ( fabs(delta-0) < EPSILON ) {
		_state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	_success[id] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4::moveJointWait(int id) {
	// wait for motion to complete
	this->simThreadsSuccessLock();
	while ( !_success[id] ) { this->simThreadsSuccessWait(); }
	_success[id] = true;
	this->simThreadsSuccessUnlock();

	// success
	return 0;
}

int CRobot4::moveTo(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CRobot4::moveToDirect(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
}

int CRobot4::moveToNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	// store angles into array
	dReal delta[4] = {angle1 - _angle[0], angle2 - _angle[1], angle3 - _angle[2], angle4 - _angle[3]};

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[0] = DEG2RAD(angle1);
	_goal[1] = DEG2RAD(angle2);
	_goal[2] = DEG2RAD(angle3);
	_goal[3] = DEG2RAD(angle4);

	// enable motor
	this->simThreadsAngleLock();
	for ( int j = 0; j < NUM_DOF; j++ ) {
		dJointEnable(_motor[j]);
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = MOBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _velocity[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = MOBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_velocity[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = MOBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
	}
    dBodyEnable(_body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	_success[0] = false;
	_success[1] = false;
	_success[2] = false;
	_success[3] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int moveToDirectNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	// success
	return 0;
}

int CRobot4::moveToZero(void) {
	this->moveTo(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4::moveToZeroNB(void) {
	this->moveToNB(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4::moveWait(void) {
	// wait for motion to complete
	this->simThreadsSuccessLock();
	while ( !(_success[0]) && !(_success[1]) && !(_success[2]) && !(_success[3]) ) {
		this->simThreadsSuccessWait();
	}
	_success[0] = true;
	_success[1] = true;
	_success[2] = true;
	_success[3] = true;
	this->simThreadsSuccessUnlock();

	// success
	return 0;
}

int CRobot4::recordAngle(int id, dReal *time, dReal *angle, int num, dReal seconds, dReal threshold) {
	pthread_t recording;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	if (_recording[id]) { return -1; }
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle;
	rArg->id = id;
	rArg->num = num;
	rArg->msecs = 1000*seconds;
	_recording[id] = true;
	pthread_create(&recording, NULL, (void* (*)(void *))&CRobot4::record_angle_thread, (void *)rArg);

	// success
	return 0;
}

int CRobot4::recordAngles(dReal *time, dReal *angle1, dReal *angle2, dReal *angle3, dReal *angle4, int num, dReal seconds, dReal threshold) {
	pthread_t recording;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	for (int i = 0; i < NUM_DOF; i++) {
		if (_recording[i]) { return -1; }
	}
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle1;
	rArg->angle2 = angle2;
	rArg->angle3 = angle3;
	rArg->angle4 = angle4;
	rArg->num = num;
	rArg->msecs = 1000*seconds;
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}
	pthread_create(&recording, NULL, (void* (*)(void *))&CRobot4::record_angles_thread, (void *)rArg);

	// success
	return 0;
}

int CRobot4::recordWait(void) {
	// wait for motion to complete
	this->simThreadsRecordingLock();
	while ( _recording[0] || _recording[1] || _recording[2] || _recording[3] ) {
		this->simThreadsRecordingWait();
	}
	_recording[0] = false;
	_recording[1] = false;
	_recording[2] = false;
	_recording[3] = false;
	this->simThreadsRecordingUnlock();

	// success
	return 0;
}

int CRobot4::resetToZero(void) {
	// reset absolute counter to 0 -> 2M_PI
	this->simThreadsAngleLock();
	int rev = (int)(_angle[LE]/2/M_PI);
	if (rev) _angle[LE] -= 2*rev*M_PI;
	rev = (int)(_angle[RE]/2/M_PI);
	if (rev) _angle[RE] -= 2*rev*M_PI;
	this->simThreadsAngleUnlock();

	// move to zero position
	this->moveToZero();

	// success
	return 0;
}

int CRobot4::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	this->simThreadsAngleLock();
	int rev = (int)(_angle[LE]/2/M_PI);
	if (rev) _angle[LE] -= 2*rev*M_PI;
	rev = (int)(_angle[RE]/2/M_PI);
	if (rev) _angle[RE] -= 2*rev*M_PI;
	this->simThreadsAngleUnlock();

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int CRobot4::setJointSpeed(int id, double speed) {
	_velocity[id] = DEG2RAD((speed > _maxSpeed[id]) ? _maxSpeed[id] : speed);

	// success
	return 0;
}

int CRobot4::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	_velocity[0] = DEG2RAD((speed1 > _maxSpeed[0]) ? _maxSpeed[0] : speed1);
	_velocity[1] = DEG2RAD((speed2 > _maxSpeed[1]) ? _maxSpeed[1] : speed2);
	_velocity[2] = DEG2RAD((speed3 > _maxSpeed[2]) ? _maxSpeed[2] : speed3);
	_velocity[3] = DEG2RAD((speed4 > _maxSpeed[3]) ? _maxSpeed[3] : speed4);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CRobot4::addToSim(dWorldID &world, dSpaceID &space, dReal *clock) {
	_world = world;
    _space = dHashSpaceCreate(space);
	_clock = clock;

	// success
	return 0;
}

int CRobot4::build(bot_t robot) {
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

	// debug printing
	//const dReal *pos = dBodyGetPosition(_body[CENTER]);
	//printf("robot pos: %lf %lf %lf\n", pos[0], pos[1], pos[2]);

	// success
	return 0;
}

int CRobot4::build(bot_t robot, CRobot *base, Conn_t *conn) {
	// build robot
	this->build_attached(robot, base, conn);

	// add connectors
	Conn_t *ctmp = robot->conn;
	while (ctmp) {
		if ( ctmp->robot == _id )
			this->add_connector(ctmp->type, ctmp->face1);
		ctmp = ctmp->next;
	}

	// debug printing
	//const dReal *pos = dBodyGetPosition(_body[CENTER]);
	//printf("robot pos: %lf %lf %lf\n", pos[0], pos[1], pos[2]);

	// success
	return 0;
}

dReal CRobot4::getAngle(int i) {
	if (i == LE || i == RE)
		_angle[i] = mod_angle(_angle[i], dJointGetHingeAngle(_joint[i]), dJointGetHingeAngleRate(_joint[i]));
	else
		_angle[i] = dJointGetHingeAngle(_joint[i]);
    return _angle[i];
}

dBodyID CRobot4::getBodyID(int id) {
    return _body[id];
}

int CRobot4::getConnectionParams(int face, dMatrix3 R, dReal *p) {
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

dBodyID CRobot4::getConnectorBodyID(int face) {
	conn_t ctmp = _conn;
	while (ctmp) {
		if (ctmp->face == face) {
			return ctmp->body;
		}
		ctmp = ctmp->next;
	}
	return NULL;
}

dBodyID CRobot4::getConnectorBodyIDs(int num) {
	conn_t ctmp = _conn;
	int i = 0;
	while (ctmp && i++ < num)
		ctmp = ctmp->next;
	if (ctmp) {
		return ctmp->body;
	}
	return NULL;
}

int CRobot4::getID(void) {
	return _id;
}

dJointID CRobot4::getMotorID(int id) {
    return _motor[id];
}

dReal CRobot4::getPosition(int body, int i) {
	const dReal *pos = dBodyGetPosition(_body[body]);
	return pos[i];
}

dReal CRobot4::getRotation(int body, int i) {
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

bool CRobot4::getSuccess(int i) {
	return _success[i];
}

int CRobot4::getType(void) {
	return _type;
}

bool CRobot4::isHome(void) {
    return ( fabs(_angle[LE]) < EPSILON && fabs(_angle[LB]) < EPSILON && fabs(_angle[RB]) < EPSILON && fabs(_angle[RE]) < EPSILON );
}

int CRobot4::setID(int id) {
	_id = id;
	return 0;
}

void CRobot4::simPreCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	this->simThreadsAngleLock();

	// update angle values for each degree of freedom
	for ( int i = 0; i < NUM_DOF; i++ ) {
		// update current angle
		_angle[i] = getAngle(i);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i], 0, _angle[i]);
		// drive motor to get current angle to match future angle
		if (_angle[i] < _goal[i] - _encoderResolution)
			dJointSetAMotorParam(_motor[i], dParamVel, _velocity[i]);
		else if (_angle[i] > _goal[i] + _encoderResolution)
			dJointSetAMotorParam(_motor[i], dParamVel, -_velocity[i]);
		else
			dJointSetAMotorParam(_motor[i], dParamVel, 0);
	}

	// unlock angle and goal
	this->simThreadsAngleUnlock();
	this->simThreadsGoalRUnlock();
}

void CRobot4::simPostCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	this->simThreadsAngleLock();

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < NUM_DOF; i++) {
		_success[i] = (bool)(!(int)(dJointGetAMotorParam(this->getMotorID(i), dParamVel)*1000) );
	}
	if ( _success[0] && _success[1] && _success[2] && _success[3] ) {
		this->simThreadsSuccessSignal();
	}

	// unlock angle and goal
	this->simThreadsAngleUnlock();
	this->simThreadsGoalRUnlock();
}

#ifdef ENABLE_GRAPHICS
void CRobot4::draw(osg::Group *root) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/texture.png"));
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
	robot->setUpdateCallback(new robot4NodeCallback(this));

	// add to scenegraph
	root->addChild(robot);
}
#endif // ENABLE_GRAPHICS

/**********************************************************
	private functions
 **********************************************************/
int CRobot4::add_connector(int type, int face) {
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

	// debug printing
	/*conn_t ctmp2 = _conn;
	while (ctmp2) {
		printf("on face %d draw a %d connector %p\n", ctmp2->face, ctmp2->type, ctmp2->body);
		ctmp2 = ctmp2->next;
	}*/

	// success
	return 0;
}

int CRobot4::build_individual(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
    _geom[ENDCAP_L] = new dGeomID[7];
    _geom[BODY_L] = new dGeomID[5];
    _geom[CENTER] = new dGeomID[3];
    _geom[BODY_R] = new dGeomID[5];
    _geom[ENDCAP_R] = new dGeomID[7];

	// initialize PID class
	for ( int i = 0; i < NUM_DOF; i++ ) { _pid[i].init(100, 1, 10, 0.1, 0.004); }

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
    dJointSetAMotorParam(_motor[0], dParamFMax, _maxJointForce[LE]);
	dJointDisable(_motor[0]);

    // motor for center to left body
    _motor[1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[1], _body[CENTER], _body[BODY_L]);
    dJointSetAMotorMode(_motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[1], 1);
    dJointSetAMotorAxis(_motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[1], 0, 0);
    dJointSetAMotorParam(_motor[1], dParamCFM, 0);
    dJointSetAMotorParam(_motor[1], dParamFMax, _maxJointForce[LB]);
	dJointDisable(_motor[1]);

    // motor for center to right body
    _motor[2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[2], _body[CENTER], _body[BODY_R]);
    dJointSetAMotorMode(_motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[2], 1);
    dJointSetAMotorAxis(_motor[2], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[2], 0, 0);
    dJointSetAMotorParam(_motor[2], dParamCFM, 0);
    dJointSetAMotorParam(_motor[2], dParamFMax, _maxJointForce[RB]);
	dJointDisable(_motor[2]);

    // motor for right body to endcap
    _motor[3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetAMotorMode(_motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[3], 1);
    dJointSetAMotorAxis(_motor[3], 0, 1, R_rb[0], R_rb[4], R_rb[8]);
    dJointSetAMotorAngle(_motor[3], 0, 0);
    dJointSetAMotorParam(_motor[3], dParamCFM, 0);
    dJointSetAMotorParam(_motor[3], dParamFMax, _maxJointForce[RE]);
	dJointDisable(_motor[3]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CRobot4::build_attached(bot_t robot, CRobot *base, Conn_t *conn) {
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

int CRobot4::build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
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

int CRobot4::build_center(dReal x, dReal y, dReal z, dMatrix3 R) {
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

int CRobot4::build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R) {
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

int CRobot4::build_bigwheel(conn_t conn, int face) {
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

int CRobot4::build_caster(conn_t conn, int face) {
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
	dMassSetBox(&m, 2700, depth, width, height);
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

int CRobot4::build_simple(conn_t conn, int face) {
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

int CRobot4::build_smallwheel(conn_t conn, int face) {
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

int CRobot4::build_square(conn_t conn, int face) {
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

int CRobot4::build_tank(conn_t conn, int face) {
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

int CRobot4::fix_body_to_connector(dBodyID cBody, int face) {
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

int CRobot4::fix_connector_to_body(int face, dBodyID cBody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

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
			dJointAttach(joint, this->getBodyID(BODY_R), cBody);
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

int CRobot4::get_connector_params(Conn_t *conn, dMatrix3 R, dReal *p) {
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

dReal CRobot4::mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate) {
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

/*void CRobot4::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}*/

void* CRobot4::record_angle_thread(void *arg) {
    recordAngleArg_t *rArg = (recordAngleArg_t *)arg;
    double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);
    for (int i = 0; i < rArg->num; i++) {
		rArg->time[i] = *(rArg->robot->_clock)*1000;
        if (i == 0) { start_time = rArg->time[i]; }
        rArg->time[i] = (rArg->time[i] - start_time) / 1000;
		rArg->robot->getJointAngle(rArg->id, rArg->angle1[i]);

		// increment time step
		time += rArg->msecs;
		// pause until next step
		while ( (int)(*(rArg->robot->_clock)*1000) < time ) {}
    }
	rArg->robot->simThreadsRecordingLock();
	rArg->robot->_recording[rArg->id] = false;
	rArg->robot->simThreadsRecordingSignal();
	rArg->robot->simThreadsRecordingUnlock();
	return NULL;
}

void* CRobot4::record_angles_thread(void *arg) {
    recordAngleArg_t *rArg = (recordAngleArg_t *)arg;
    double start_time;
	int time = (int)(*(rArg->robot->_clock)*1000);
    for (int i = 0; i < rArg->num; i++) {
		rArg->time[i] = *(rArg->robot->_clock)*1000;
        if (i == 0) { start_time = rArg->time[i]; }
        rArg->time[i] = (rArg->time[i] - start_time) / 1000;
		rArg->robot->getJointAngle(0, rArg->angle1[i]);
		rArg->robot->getJointAngle(1, rArg->angle2[i]);
		rArg->robot->getJointAngle(2, rArg->angle3[i]);
		rArg->robot->getJointAngle(3, rArg->angle4[i]);

		// increment time step
		time += rArg->msecs;
		// pause until next step
		while ( (int)(*(rArg->robot->_clock)*1000) < time ) {}
    }
	rArg->robot->simThreadsRecordingLock();
    for (int i = 0; i < NUM_DOF; i++) {
        rArg->robot->_recording[i] = false;
    }
	rArg->robot->simThreadsRecordingSignal();
	rArg->robot->simThreadsRecordingUnlock();
	return NULL;
}

#ifdef ENABLE_GRAPHICS
void CRobot4::draw_bigwheel(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

void CRobot4::draw_caster(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

void CRobot4::draw_simple(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

void CRobot4::draw_smallwheel(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

void CRobot4::draw_square(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

void CRobot4::draw_tank(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("data/mobot/conn_texture.png"));
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

/**********************************************************
	CiMobot Class
 **********************************************************/
CiMobot::CiMobot(void) {
	_encoderResolution = DEG2RAD(0.5);
	_maxJointVelocity[IMOBOT_JOINT1] = 6.70;
	_maxJointVelocity[IMOBOT_JOINT2] = 2.61;
	_maxJointVelocity[IMOBOT_JOINT3] = 2.61;
	_maxJointVelocity[IMOBOT_JOINT4] = 6.70;
	_maxJointForce[IMOBOT_JOINT1] = 0.260;
	_maxJointForce[IMOBOT_JOINT2] = 1.059;
	_maxJointForce[IMOBOT_JOINT3] = 1.059;
	_maxJointForce[IMOBOT_JOINT4] = 0.260;
	_center_length = 0.07303;
	_center_width = 0.02540;
	_center_height = 0.06909;
	_center_radius = 0.03554;
	_center_offset = 0;
	_body_length = 0.03785;
	_body_width = 0.07239;
	_body_height = 0.07239;
	_body_radius = 0.03620;
	_body_inner_width_left = 0.02287;
	_body_inner_width_right = 0.02287;
	_body_end_depth = 0.01994;
	_body_mount_center = 0.03792;
	_end_width = 0.07239;
	_end_height = 0.07239;
	_end_depth = 0.00476;
	_end_radius = 0.01778;
	_connector_depth = 0;
	_connector_height = 0;
	_connector_radius = 0;
	_bigwheel_radius = 0;
	_smallwheel_radius = 0;
	_tank_depth = 0;
	_tank_height = 0;
	_type = IMOBOT;
}

/**********************************************************
	CMobot Class
 **********************************************************/
CMobot::CMobot(void) {
	_encoderResolution = DEG2RAD(0.5);
	_maxJointVelocity[MOBOT_JOINT1] = 6.70;
	_maxJointVelocity[MOBOT_JOINT2] = 2.61;
	_maxJointVelocity[MOBOT_JOINT3] = 2.61;
	_maxJointVelocity[MOBOT_JOINT4] = 6.70;
	_maxJointForce[MOBOT_JOINT1] = 0.260;
	_maxJointForce[MOBOT_JOINT2] = 1.059;
	_maxJointForce[MOBOT_JOINT3] = 1.059;
	_maxJointForce[MOBOT_JOINT4] = 0.260;
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
	_type = MOBOT;

}
