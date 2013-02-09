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

	// init locks
	this->simThreadsAngleInit();
	this->simThreadsGoalInit();
	this->simThreadsRecordingInit();
	this->simThreadsSuccessInit();
}

CRobot4::~CRobot4(void) {
	//dSpaceDestroy(_space); //sigsegv
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

void* CRobot4::recordAngleThread(void *arg) {
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
	pthread_create(&recording, NULL, (void* (*)(void *))&CRobot4::recordAngleThread, (void *)rArg);

	// success
	return 0;
}

void* CRobot4::recordAnglesThread(void *arg) {
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
	pthread_create(&recording, NULL, (void* (*)(void *))&CRobot4::recordAnglesThread, (void *)rArg);

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
	private functions
 **********************************************************/
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

int CRobot4::connect(CRobotSim &sim) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobot(this);

	// success
	return 0;
}

int CRobot4::connect(CRobotSim &sim, dReal x, dReal y, dReal z) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobot(this, x, y, z);

	// success
	return 0;
}

int CRobot4::connect(CRobotSim &sim, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobot(this, x, y, z, psi, theta, phi);

	// success
	return 0;
}

int CRobot4::connect(CRobotSim &sim, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobot(this, x, y, z, psi, theta, phi, r_le, r_lb, r_rb, r_re);

	// success
	return 0;
}

int CRobot4::connect(CRobotSim &sim, CRobot4 *base, int face1, int face2) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobotConnected(this, base, face1, face2);

	// success
	return 0;
}

int CRobot4::connect(CRobotSim &sim, CRobot4 *base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	dSpaceID space;
	sim.simAddRobot(_world, space, &_clock);
    _space = dHashSpaceCreate(space);
	sim.addMobotConnected(this, base, face1, face2, r_le, r_lb, r_rb, r_re);

	// success
	return 0;
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

	// set update callback for robot
	robot->setUpdateCallback(new robot4NodeCallback(this));

	// add to scenegraph
	root->addChild(robot);
}
#endif /* ENABLE_GRAPHICS */

dReal CRobot4::getAngle(int i) {
	if (i == LE || i == RE)
		_angle[i] = mod_angle(_angle[i], dJointGetHingeAngle(_joint[i]), dJointGetHingeAngleRate(_joint[i]));
	else
		_angle[i] = dJointGetHingeAngle(_joint[i]);
    return _angle[i];
}

bool CRobot4::getSuccess(int i) {
	return _success[i];
}

dReal CRobot4::getPosition(int body, int i) {
	const dReal *pos = dBodyGetPosition(_body[body]);
	return pos[i];
}

dReal CRobot4::getRotation(int body, int i) {
	const dReal *rot = dBodyGetRotation(_body[body]);
	dMatrix3 rot2 = {rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8], rot[9], rot[10], rot[11]};
	dReal angles[3] = {0};
	extract_euler_angles(rot2, angles[0], angles[1], angles[2]);
	return angles[i];
}

dBodyID CRobot4::getBodyID(int id) {
    return _body[id];
}

dJointID CRobot4::getMotorID(int id) {
    return _motor[id];
}

bool CRobot4::isHome(void) {
    return ( fabs(_angle[LE]) < EPSILON && fabs(_angle[LB]) < EPSILON && fabs(_angle[RB]) < EPSILON && fabs(_angle[RE]) < EPSILON );
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

void CRobot4::create_fixed_joint(CRobot4 *attach, int face1, int face2) {
    int part1, part2;

    switch (face1) {
        case 1:
            part1 = ENDCAP_L;
            break;
        case 2:
        case 3:
            part1 = BODY_L;
            break;
        case 4:
        case 5:
            part1 = BODY_R;
            break;
        case 6:
            part1 = ENDCAP_R;
            break;
    }
    switch (face2) {
        case 1:
            part2 = ENDCAP_L;
            break;
        case 2:
        case 3:
            part2 = BODY_L;
            break;
        case 4:
        case 5:
            part2 = BODY_R;
            break;
        case 6:
            part2 = ENDCAP_R;
            break;
    }

    dJointID joint = dJointCreateFixed(_world, 0);
    dJointAttach(joint, attach->getBodyID(part1), this->getBodyID(part2));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CRobot4::create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
    dReal   sphi = sin(phi),        cphi = cos(phi),
    stheta = sin(theta),    ctheta = cos(theta),
    spsi = sin(psi),        cpsi = cos(psi);

    R[0] =  cphi*ctheta;
    R[1] = -cphi*stheta*spsi - sphi*cpsi;
    R[2] = -cphi*stheta*cpsi + sphi*spsi;
    R[3] = 0;
    R[4] =  sphi*ctheta;
    R[5] = -sphi*stheta*spsi + cphi*cpsi;
    R[6] = -sphi*stheta*cpsi - cphi*spsi;
    R[7] = 0;
    R[8] =  stheta;
    R[9] =  ctheta*spsi;
    R[10] = ctheta*cpsi;
    R[11] = 0;
}

void CRobot4::extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi) {
    if ( fabs(R[8]-1) < DBL_EPSILON ) {         // R_31 == 1; theta = M_PI/2
        psi = atan2(-R[1], -R[2]);
        theta = M_PI/2;
        phi = 0;
    }
    else if ( fabs(R[8]+1) < DBL_EPSILON ) {    // R_31 == -1; theta = -M_PI/2
        psi = atan2(R[1], R[2]);
        theta = -M_PI/2;
        phi = 0;
    }
    else {
        theta = asin(R[8]);
        psi = atan2(R[9]/cos(theta), R[10]/cos(theta));
        phi = atan2(R[4]/cos(theta), R[0]/cos(theta));
    }
}

unsigned int CRobot4::diff_msecs(struct timespec t1, struct timespec t2) {
    unsigned int t;
    t = (t2.tv_sec - t1.tv_sec) * 1000;
    t += (t2.tv_nsec - t1.tv_nsec) / 1000000;
    return t;
}

unsigned int CRobot4::current_msecs(struct timespec t) {
	return t.tv_sec*1000 + t.tv_nsec / 1000000;
}
/*void CRobot4::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}*/

void CRobot4::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
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
    z += _end_height/2;
    // convert input angles to radians
    psi = DEG2RAD(psi);         // roll: x
    theta = DEG2RAD(theta);     // pitch: -y
    phi = DEG2RAD(phi);         // yaw: z

    // create rotation matrix for robot
    dMatrix3 R;
    this->create_rotation_matrix(R, psi, theta, phi);

    // store initial body angles into array
    _angle[LE] = 0;
    _angle[LB] = 0;
    _angle[RB] = 0;
    _angle[RE] = 0;

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-_body_radius - _body_length - _body_end_depth - _end_depth/2, 0, 0, -_body_radius - _body_length - _body_end_depth, 0, 0};
    dReal lb[6] = {-_body_radius - _body_length - _body_end_depth/2, 0, 0, -_center_length/2, _center_width/2, 0};
	dReal ce[3] = {0, _center_offset, 0};
    dReal rb[6] = { _body_radius + _body_length + _body_end_depth/2, 0, 0, _center_length/2, _center_width/2, 0};
    dReal re[6] = { _body_radius + _body_length + _body_end_depth + _end_depth/2, 0, 0,  _body_radius + _body_length + _body_end_depth, 0, 0};

    // build pieces of module
    this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->build_center(R[1]*ce[1] + x, R[5]*ce[1] + y, R[9]*ce[1] + z, R);
    this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // joint for left endcap to body
    _joint[0] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetHingeAnchor(_joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[0], dParamCFM, 0);

    // joint for center to left body 1
    _joint[1] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[1], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[1], R[0]*lb[3] + R[1]*(_center_offset+lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(_center_offset+lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(_center_offset+lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[1], dParamCFM, 0);

    // joint for center to left body 2
    _joint[4] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[4], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[4], R[0]*lb[3] + R[1]*(_center_offset-lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(_center_offset-lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(_center_offset-lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[4], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[4], dParamCFM, 0);

    // joint for center to right body 1
    _joint[2] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[2], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[2], R[0]*rb[3] + R[1]*(_center_offset+rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(_center_offset+rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(_center_offset+rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[2], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[2], dParamCFM, 0);

    // joint for center to right body 2
    _joint[5] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[5], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[5], R[0]*rb[3] + R[1]*(_center_offset-rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(_center_offset-rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(_center_offset-rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[5], dParamCFM, 0);

    // joint for right body to endcap
    _joint[3] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetHingeAnchor(_joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(_joint[3], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[3], dParamCFM, 0);

    // motor for left endcap to body
    _motor[0] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetAMotorMode(_motor[0], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[0], 1);
    dJointSetAMotorAxis(_motor[0], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[0], 0, 0);
    dJointSetAMotorParam(_motor[0], dParamCFM, 0);
    dJointSetAMotorParam(_motor[0], dParamFMax, _maxJointForce[LE]);

    // motor for center to left body
    _motor[1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[1], _body[CENTER], _body[BODY_L]);
    dJointSetAMotorMode(_motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[1], 1);
    dJointSetAMotorAxis(_motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[1], 0, 0);
    dJointSetAMotorParam(_motor[1], dParamCFM, 0);
    dJointSetAMotorParam(_motor[1], dParamFMax, _maxJointForce[LB]);

    // motor for center to right body
    _motor[2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[2], _body[CENTER], _body[BODY_R]);
    dJointSetAMotorMode(_motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[2], 1);
    dJointSetAMotorAxis(_motor[2], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[2], 0, 0);
    dJointSetAMotorParam(_motor[2], dParamCFM, 0);
    dJointSetAMotorParam(_motor[2], dParamFMax, _maxJointForce[RB]);

    // motor for right body to endcap
    _motor[3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetAMotorMode(_motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[3], 1);
    dJointSetAMotorAxis(_motor[3], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[3], 0, 0);
    dJointSetAMotorParam(_motor[3], dParamCFM, 0);
    dJointSetAMotorParam(_motor[3], dParamFMax, _maxJointForce[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);
}

void CRobot4::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
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
    z += _body_height/2;
    // convert input angles to radians
    psi = DEG2RAD(psi);         // roll: x
    theta = DEG2RAD(theta);     // pitch: -y
    phi = DEG2RAD(phi);         // yaw: z
    r_le = DEG2RAD(r_le);       // left end
    r_lb = DEG2RAD(r_lb);       // left body
    r_rb = DEG2RAD(r_rb);       // right body
    r_re = DEG2RAD(r_re);       // right end

    // create rotation matrix for robot
    dMatrix3 R;
    this->create_rotation_matrix(R, psi, theta, phi);

    // store initial body angles into array
    _angle[LE] = r_le;
    _angle[LB] = r_lb;
    _angle[RB] = r_rb;
    _angle[RE] = r_re;

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-_body_radius - _body_length - _body_end_depth - _end_depth/2, 0, 0, -_center_length/2 - _body_length - _body_end_depth, 0, 0};
    dReal lb[6] = {-_body_radius - _body_length - _body_end_depth/2, 0, 0, -_center_length/2, _center_width/2, 0};
	dReal ce[3] = {0, _center_offset, 0};
    dReal rb[6] = {_body_radius + _body_length + _body_end_depth/2, 0, 0, _center_length/2, _center_width/2, 0};
    dReal re[6] = {_body_radius + _body_length + _body_end_depth + _end_depth/2, 0, 0, _center_length/2 + _body_length + _body_end_depth, 0, 0};

    this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->build_center(R[1]*ce[1] + x, R[5]*ce[1] + y, R[5]*ce[1] + z, R);
    this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // joint for left endcap to body
    _joint[0] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetHingeAnchor(_joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[0], dParamCFM, 0);

    // joint for center to left body 1
    _joint[1] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[1], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[1], R[0]*lb[3] + R[1]*(_center_offset+lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(_center_offset+lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(_center_offset+lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[1], dParamCFM, 0);

    // joint for center to left body 2
    _joint[4] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[4], _body[CENTER], _body[BODY_L]);
    dJointSetHingeAnchor(_joint[4], R[0]*lb[3] + R[1]*(_center_offset-lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(_center_offset-lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(_center_offset-lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(_joint[4], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[4], dParamCFM, 0);

    // joint for center to right body 1
    _joint[2] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[2], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[2], R[0]*rb[3] + R[1]*(_center_offset+rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(_center_offset+rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(_center_offset+rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[2], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[2], dParamCFM, 0);

    // joint for center to right body 2
    _joint[5] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[5], _body[CENTER], _body[BODY_R]);
    dJointSetHingeAnchor(_joint[5], R[0]*rb[3] + R[1]*(_center_offset-rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(_center_offset-rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(_center_offset-rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(_joint[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[5], dParamCFM, 0);

    // joint for right body to endcap
    _joint[3] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetHingeAnchor(_joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(_joint[3], -R[0], -R[4], -R[8]);
    dJointSetHingeParam(_joint[3], dParamCFM, 0);

    // create rotation matrices for each body part
    dMatrix3 R_e, R_b, R_le, R_lb, R_rb, R_re;
    dRFromAxisAndAngle(R_b, 0, 1, 0, r_lb);
    dMultiply0(R_lb, R, R_b, 3, 3, 3);
    dRFromAxisAndAngle(R_e, -1, 0, 0, r_le);
    dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
    dRFromAxisAndAngle(R_b, 0, -1, 0, r_rb);
    dMultiply0(R_rb, R, R_b, 3, 3, 3);
    dRFromAxisAndAngle(R_e, 1, 0, 0, r_re);
    dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

    // offset values from center of robot
    dReal le_r[3] = {-_center_length/2 - (_body_length + _body_end_depth + _end_depth/2)*cos(r_lb), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(r_lb)};
    dReal lb_r[3] = {-_center_length/2 - (_body_length + _body_end_depth/2)*cos(r_lb), 0, (_body_length + _body_end_depth/2)*sin(r_lb)};
    dReal rb_r[3] = {_center_length/2 + (_body_length + _body_end_depth/2)*cos(r_rb), 0, (_body_length + _body_end_depth/2)*sin(r_rb)};
    dReal re_r[3] = {_center_length/2 + (_body_length + _body_end_depth + _end_depth/2)*cos(r_rb), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(r_rb)};

    // re-build pieces of module
    this->build_endcap(ENDCAP_L, R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
    this->build_body(BODY_L, R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, r_lb);
    this->build_body(BODY_R, R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, r_rb);
    this->build_endcap(ENDCAP_R, R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);

    // motor for left endcap to body
    _motor[0] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[0], _body[BODY_L], _body[ENDCAP_L]);
    dJointSetAMotorMode(_motor[0], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[0], 1);
    dJointSetAMotorAxis(_motor[0], 0, 1, R_lb[0], R_lb[4], R_lb[8]);
    dJointSetAMotorAngle(_motor[0], 0, 0);
    dJointSetAMotorParam(_motor[0], dParamCFM, 0);
    dJointSetAMotorParam(_motor[0], dParamFMax, _maxJointForce[LE]);

    // motor for center to left body
    _motor[1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[1], _body[CENTER], _body[BODY_L]);
    dJointSetAMotorMode(_motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[1], 1);
    dJointSetAMotorAxis(_motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[1], 0, 0);
    dJointSetAMotorParam(_motor[1], dParamCFM, 0);
    dJointSetAMotorParam(_motor[1], dParamFMax, _maxJointForce[LB]);

    // motor for center to right body
    _motor[2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[2], _body[CENTER], _body[BODY_R]);
    dJointSetAMotorMode(_motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[2], 1);
    dJointSetAMotorAxis(_motor[2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(_motor[2], 0, 0);
    dJointSetAMotorParam(_motor[2], dParamCFM, 0);
    dJointSetAMotorParam(_motor[2], dParamFMax, _maxJointForce[RB]);

    // motor for right body to endcap
    _motor[3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[3], _body[BODY_R], _body[ENDCAP_R]);
    dJointSetAMotorMode(_motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[3], 1);
    dJointSetAMotorAxis(_motor[3], 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
    dJointSetAMotorAngle(_motor[3], 0, 0);
    dJointSetAMotorParam(_motor[3], dParamCFM, 0);
    dJointSetAMotorParam(_motor[3], dParamFMax, _maxJointForce[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);
}

void CRobot4::buildAttached00(CRobot4 *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3] = {0};
    dMatrix3 R, R1, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(CENTER, 0), attach->getRotation(CENTER, 1), attach->getRotation(CENTER, 2));

    if ( face1 == 1 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - 2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length;
        m[1] = 0;
    }
    else if ( face1 == 1 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width;
        m[1] = _body_end_depth + _body_length - _body_mount_center + 0.5*_center_length;
    }
    else if ( face1 == 1 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width;
        m[1] = -_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length;
    }
    else if ( face1 == 1 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width;
        m[1] = -_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length;
    }
    else if ( face1 == 1 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width;
        m[1] = _body_end_depth + _body_length - _body_mount_center + 0.5*_center_length;
    }
    else if ( face1 == 1 && face2 == 6 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - 2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length;
        m[1] = 0;
    }
    else if ( face1 == 2 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center;
        m[1] = -_end_depth - 0.5*_body_width - _body_end_depth - _body_length - 0.5*_center_length;
    }
    else if ( face1 == 2 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) - 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 2 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 2 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 2 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) - 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 2 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center;
        m[1] = -_end_depth - 0.5*_body_width - _body_end_depth - _body_length - 0.5*_center_length;
    }
    else if ( face1 == 3 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center;
        m[1] = _end_depth + 0.5*_body_width + _body_end_depth + _body_length + 0.5*_center_length;
    }
    else if ( face1 == 3 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 3 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) - 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 3 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) - 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 3 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length + 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 3 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center;
        m[1] = _end_depth + 0.5*_body_width + _body_end_depth + _body_length + 0.5*_center_length;
    }
    else if ( face1 == 4 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center;
        m[1] = -_end_depth - 0.5*_body_width - _body_end_depth - _body_length - 0.5*_center_length;
    }
    else if ( face1 == 4 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length - 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 4 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 4 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length +  2*(_body_length + _body_end_depth - _body_mount_center) + 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 4 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length - 0.5*_center_length;
        m[1] = -_body_width;
    }
    else if ( face1 == 4 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center;
        m[1] = -_end_depth - 0.5*_body_width - _body_end_depth - _body_length - 0.5*_center_length;
    }
    else if ( face1 == 5 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center;
        m[1] = _end_depth + 0.5*_body_width + _body_end_depth + _body_length + 0.5*_center_length;
    }
    else if ( face1 == 5 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 5 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length - 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 5 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length - 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 5 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length    +   2*(_body_length + _body_end_depth - _body_mount_center) + 0.5*_center_length;
        m[1] = _body_width;
    }
    else if ( face1 == 5 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center;
        m[1] = _end_depth + 0.5*_body_width + _body_end_depth + _body_length + 0.5*_center_length;
    }
    else if ( face1 == 6 && face2 == 1 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + 2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length;
        m[1] = 0;
    }
    else if ( face1 == 6 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width;
        m[1] = -_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length;
    }
    else if ( face1 == 6 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length +  _body_length + _body_end_depth + _end_depth + 0.5*_body_width;
        m[1] = _body_end_depth + _body_length - _body_mount_center + 0.5*_center_length;
    }
    else if ( face1 == 6 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width;
        m[1] = _body_end_depth + _body_length - _body_mount_center + 0.5*_center_length;
    }
    else if ( face1 == 6 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width;
        m[1] = -_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length;
    }
    else if ( face1 == 6 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + 2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length;
        m[1] = 0;
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(CENTER, 0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(ENDCAP_L, 1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                RAD2DEG(psi), RAD2DEG(theta), RAD2DEG(phi));

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4::buildAttached10(CRobot4 *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(CENTER, 0), attach->getRotation(CENTER, 1), attach->getRotation(CENTER, 2));

    if ( face1 == 1 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth) + R3[0]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth) + R3[4]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth) + R3[8]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth) + R3[0]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth) + R3[4]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth) + R3[8]*(-2*_end_depth - _body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                               + R1[1]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _end_depth - 0.5*_body_width  + R1[5]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                               + R1[9]*(-_body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length                   + R1[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                                      + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length                   + R1[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                                      + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[1]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[9]*(-_body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[1]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[9]*(_body_end_depth + _body_length + 0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length               + R1[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(0.5*_center_length);
        m[2] =                                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length               + R1[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(0.5*_center_length);
        m[2] =                                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[1]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R1[9]*(_body_end_depth + _body_length + 0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[1]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) -  _end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[9]*(-_body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      -_body_width + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length                    + R1[0]*(-0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                                      + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[1]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) -  _end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length - 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[9]*(-_body_end_depth - _body_length - 0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[1]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) +  _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[9]*(_body_end_depth + _body_length + 0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  + _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  + _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[1]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) +  _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R1[9]*(_body_end_depth + _body_length + 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth) + R3[0]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth) + R3[4]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth) + R3[8]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center + 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center - 0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth) + R3[0]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth) + R3[4]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth) + R3[8]*(2*_end_depth + _body_end_depth + _body_length + 0.5*_center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(CENTER, 0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(CENTER, 1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(CENTER, 2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                RAD2DEG(psi), RAD2DEG(theta), RAD2DEG(phi));

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4::buildAttached01(CRobot4 *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    // initialize variables
    dReal psi, theta, phi, r_e, r_b, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(CENTER, 0), attach->getRotation(CENTER, 1), attach->getRotation(CENTER, 2));

    // rotation of body about fixed point
    if ( face2 == 1 ) {
        r_e = DEG2RAD(r_le);
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 2 ) {
        r_e = 0;
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 3 ) {
        r_e = 0;
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 4 ) {
        r_e = 0;
        r_b = DEG2RAD(r_rb);
    }
    else if ( face2 == 5 ) {
        r_e = 0;
        r_b = DEG2RAD(r_rb);
    }
    else if ( face2 == 6 ) {
        r_e = DEG2RAD(r_re);
        r_b = DEG2RAD(r_rb);
    }

    if ( face1 == 1 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - 2*_end_depth + R1[0]*(-_body_end_depth - _body_length) + R3[0]*(-0.5*_center_length);
        m[1] =                                                                   R1[4]*(-_body_end_depth - _body_length) + R3[4]*(-0.5*_center_length);
        m[2] =                                                                   R1[8]*(-_body_end_depth - _body_length) + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width + R1[1]*(0.5*_center_length);
        m[1] = _body_end_depth + _body_length - _body_mount_center + R1[5]*(0.5*_center_length);
        m[2] = R1[9]*(0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width + R1[1]*(-0.5*_center_length);
        m[1] = -_body_end_depth - _body_length + _body_mount_center + R1[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width + R1[1]*(-0.5*_center_length);
        m[1] = -_body_end_depth - _body_length + _body_mount_center + R1[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - _end_depth - 0.5*_body_width + R1[1]*(0.5*_center_length);
        m[1] = _body_end_depth + _body_length - _body_mount_center + R1[5]*(0.5*_center_length);
        m[2] = R1[9]*(0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth - 2*_end_depth + R1[0]*(-_body_end_depth - _body_length) + R3[0]*(-0.5*_center_length);
        m[1] = R1[4]*(-_body_end_depth - _body_length) + R3[4]*(-0.5*_center_length);
        m[2] = R1[8]*(-_body_end_depth - _body_length) + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center + R1[1]*(-_body_end_depth - _body_length) + R3[1]*(-0.5*_center_length);
        m[1] = -_end_depth - 0.5*_body_width  + R1[5]*(-_body_end_depth - _body_length) + R3[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-_body_end_depth - _body_length) + R3[9]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) + R1[0]*(-0.5*_center_length);
        m[1] = -_body_width + R1[4]*(-0.5*_center_length);
        m[2] = R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*_center_length                   + R1[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                                      + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*_center_length                   + R1[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(0.5*_center_length);
        m[2] =                                      + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) + R1[0]*(-0.5*_center_length);
        m[1] = -_body_width + R1[4]*(-0.5*_center_length);
        m[2] = R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center + R1[1]*(-_body_end_depth - _body_length) + R3[1]*(-0.5*_center_length);
        m[1] = -_end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length) + R5[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-_body_end_depth - _body_length) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center + R1[1]*(_body_end_depth + _body_length) + R3[1]*(0.5*_center_length);
        m[1] = _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length) + R2[5]*(0.5*_center_length);
        m[2] = R1[9]*(_body_end_depth + _body_length) + R3[9]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*_center_length               + R1[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(0.5*_center_length);
        m[2] =                                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center + R1[0]*(-0.5*_center_length);
        m[1] = _body_width + R1[4]*(-0.5*_center_length);
        m[2] = R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*_center_length + 2*(-_body_length - _body_end_depth + _body_mount_center) + R1[0]*(-0.5*_center_length);
        m[1] = _body_width + R1[4]*(-0.5*_center_length);
        m[2] = R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*_center_length               + R1[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(0.5*_center_length);
        m[2] =                                  + R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length - _body_length - _body_end_depth + _body_mount_center + R1[1]*(_body_end_depth + _body_length) + R3[1]*(0.5*_center_length);
        m[1] = _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length) + R3[5]*(0.5*_center_length);
        m[2] = R1[9]*(_body_end_depth + _body_length) + R3[9]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center + R1[1]*(-_body_end_depth - _body_length) + R3[1]*(-0.5*_center_length);
        m[1] = -_end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length) + R3[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-_body_end_depth - _body_length) + R3[9]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      -_body_width + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + R1[0]*(0.5*_center_length);
        m[1] = -_body_width + R1[4]*(0.5*_center_length);
        m[2] = R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + R1[0]*(0.5*_center_length);
        m[1] = -_body_width + R1[4]*(0.5*_center_length);
        m[2] = R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*_center_length                    + R1[0]*(-0.5*_center_length);
        m[1] =                      - _body_width    + R1[4]*(-0.5*_center_length);
        m[2] =                                      + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center + R1[1]*(-_body_end_depth - _body_length) + R3[1]*(-0.5*_center_length);
        m[1] = -_end_depth - 0.5*_body_width + R1[5]*(-_body_end_depth - _body_length) + R3[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-_body_end_depth - _body_length) + R3[9]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center + R1[1]*(_body_end_depth + _body_length) + R3[1]*(0.5*_center_length);
        m[1] = _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length) + R3[5]*(0.5*_center_length);
        m[2] = R1[9]*(_body_end_depth + _body_length) + R3[9]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + R1[0]*(0.5*_center_length);
        m[1] = _body_width + R1[4]*(0.5*_center_length);
        m[2] = R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*_center_length                + R1[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R1[4]*(-0.5*_center_length);
        m[2] =                                  + R1[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*_center_length + 2*(_body_length + _body_end_depth - _body_mount_center) + R1[0]*(0.5*_center_length);
        m[1] = _body_width + R1[4]*(0.5*_center_length);
        m[2] = R1[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_e);
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth - _body_mount_center + R1[1]*(_body_end_depth + _body_length) + R3[1]*(0.5*_center_length);
        m[1] = _end_depth + 0.5*_body_width + R1[5]*(_body_end_depth + _body_length) + R3[5]*(0.5*_center_length);
        m[2] = R1[9]*(_body_end_depth + _body_length) + R3[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_e);
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + 2*_end_depth + R1[0]*(_body_end_depth + _body_length) + R3[0]*(0.5*_center_length);
        m[1] = R1[4]*(_body_end_depth + _body_length) + R3[4]*(0.5*_center_length);
        m[2] = R1[8]*(_body_end_depth + _body_length) + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width + R1[1]*(-0.5*_center_length);
        m[1] = -_body_end_depth - _body_length + _body_mount_center + R1[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width + R1[1]*(0.5*_center_length);
        m[1] = _body_end_depth + _body_length - _body_mount_center + R1[5]*(0.5*_center_length);
        m[2] = R1[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width + R1[1]*(0.5*_center_length);
        m[1] = _body_end_depth + _body_length - _body_mount_center + R1[5]*(0.5*_center_length);
        m[2] = R1[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + _end_depth + 0.5*_body_width + R1[1]*(-0.5*_center_length);
        m[1] = -_body_end_depth - _body_length + _body_mount_center + R1[5]*(-0.5*_center_length);
        m[2] = R1[9]*(-0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -r_e);
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_e);
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length + _body_length + _body_end_depth + 2*_end_depth + R1[0]*(_body_end_depth + _body_length) + R3[0]*(0.5*_center_length);
        m[1] = R1[4]*(_body_end_depth + _body_length) + R3[4]*(0.5*_center_length);
        m[2] = R1[8]*(_body_end_depth + _body_length) + R3[8]*(0.5*_center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(CENTER, 0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(CENTER, 1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(CENTER, 2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                RAD2DEG(psi), RAD2DEG(theta), RAD2DEG(phi), r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4::buildAttached11(CRobot4 *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    // initialize variables
    dReal psi, theta, phi, r_e, r_b, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R6, R7, R8, R9, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(CENTER, 0), attach->getRotation(CENTER, 1), attach->getRotation(CENTER, 2));

    // rotation of body about fixed point
    if ( face2 == 1 ) {
        r_e = DEG2RAD(r_le);
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 2 ) {
        r_e = 0;
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 3 ) {
        r_e = 0;
        r_b = DEG2RAD(r_lb);
    }
    else if ( face2 == 4 ) {
        r_e = 0;
        r_b = DEG2RAD(r_rb);
    }
    else if ( face2 == 5 ) {
        r_e = 0;
        r_b = DEG2RAD(r_rb);
    }
    else if ( face2 == 6 ) {
        r_e = DEG2RAD(r_re);
        r_b = DEG2RAD(r_rb);
    }

    if ( face1 == 1 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth) + R3[0]*(-2*_end_depth) + R5[0]*(-_body_end_depth - _body_length) + R7[0]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth) + R3[4]*(-2*_end_depth) + R5[4]*(-_body_end_depth - _body_length) + R7[4]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth) + R3[8]*(-2*_end_depth) + R5[8]*(-_body_end_depth - _body_length) + R7[8]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth - _end_depth - 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 1 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth) + R3[0]*(-2*_end_depth) + R5[0]*(-_body_end_depth - _body_length) + R7[0]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth) + R3[4]*(-2*_end_depth) + R5[4]*(-_body_end_depth - _body_length) + R7[4]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth) + R3[8]*(-2*_end_depth) + R5[8]*(-_body_end_depth - _body_length) + R7[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                               + R3[1]*(-_body_end_depth - _body_length) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _end_depth - 0.5*_body_width  + R3[5]*(-_body_end_depth - _body_length) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                               + R3[9]*(-_body_end_depth - _body_length) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _body_width    + R3[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length                   + R3[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                                      + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length                   + R3[0]*(0.5*_center_length);
        m[1] =                      - _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                                      + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _body_width    + R3[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[1]*(-_body_end_depth - _body_length) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) - _end_depth - 0.5*_body_width + R3[5]*(-_body_end_depth - _body_length) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[9]*(-_body_end_depth - _body_length) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[1]*(_body_end_depth + _body_length) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _end_depth + 0.5*_body_width + R3[5]*(_body_end_depth + _body_length) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[9]*(_body_end_depth + _body_length) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length               + R3[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R3[4]*(0.5*_center_length);
        m[2] =                                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _body_width    + R3[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length   +   2*R1[0]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[0]*(-0.5*_center_length);
        m[1] =                          2*R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _body_width    + R3[4]*(-0.5*_center_length);
        m[2] =                          2*R1[8]*(-_body_length - _body_end_depth + _body_mount_center)                 + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*_center_length               + R3[0]*(0.5*_center_length);
        m[1] =                      _body_width  + R3[4]*(0.5*_center_length);
        m[2] =                                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*_center_length + R1[0]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[1]*(_body_end_depth + _body_length) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(-_body_length - _body_end_depth + _body_mount_center) + _end_depth + 0.5*_body_width + R3[5]*(_body_end_depth + _body_length) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(-_body_length - _body_end_depth + _body_mount_center) +                              R3[9]*(_body_end_depth + _body_length) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[1]*(-_body_end_depth - _body_length) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) -  _end_depth - 0.5*_body_width + R3[5]*(-_body_end_depth - _body_length) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[9]*(-_body_end_depth - _body_length) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length                + R3[0]*(-0.5*_center_length);
        m[1] =                      -_body_width + R3[4]*(-0.5*_center_length);
        m[2] =                                  + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  - _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  - _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length                    + R3[0]*(-0.5*_center_length);
        m[1] =                      - _body_width    + R3[4]*(-0.5*_center_length);
        m[2] =                                      + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[1]*(-_body_end_depth - _body_length) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) -  _end_depth - 0.5*_body_width + R3[5]*(-_body_end_depth - _body_length) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[9]*(-_body_end_depth - _body_length) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[1]*(_body_end_depth + _body_length) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) +  _end_depth + 0.5*_body_width + R3[5]*(_body_end_depth + _body_length) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[9]*(_body_end_depth + _body_length) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  + _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length                + R3[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R3[4]*(-0.5*_center_length);
        m[2] =                                  + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length                + R3[0]*(-0.5*_center_length);
        m[1] =                      _body_width  + R3[4]*(-0.5*_center_length);
        m[2] =                                  + R3[8]*(-0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*_center_length    +   2*R1[0]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[0]*(0.5*_center_length);
        m[1] =                          2*R1[4]*(_body_length + _body_end_depth - _body_mount_center)  + _body_width    + R3[4]*(0.5*_center_length);
        m[2] =                          2*R1[8]*(_body_length + _body_end_depth - _body_mount_center)                  + R3[8]*(0.5*_center_length);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[1]*(_body_end_depth + _body_length) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth - _body_mount_center) +  _end_depth + 0.5*_body_width + R3[5]*(_body_end_depth + _body_length) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth - _body_mount_center) +                               R3[9]*(_body_end_depth + _body_length) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth) + R3[0]*(2*_end_depth) + R5[0]*(_body_end_depth + _body_length) + R7[0]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth) + R3[4]*(2*_end_depth) + R5[4]*(_body_end_depth + _body_length) + R7[4]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth) + R3[8]*(2*_end_depth) + R5[8]*(_body_end_depth + _body_length) + R7[8]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(_body_end_depth + _body_length - _body_mount_center) + R5[1]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(_body_end_depth + _body_length - _body_mount_center) + R5[5]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(_body_end_depth + _body_length - _body_mount_center) + R5[9]*(0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[1]*(-_body_end_depth - _body_length + _body_mount_center) + R5[1]*(-0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[5]*(-_body_end_depth - _body_length + _body_mount_center) + R5[5]*(-0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth + _end_depth + 0.5*_body_width) + R3[9]*(-_body_end_depth - _body_length + _body_mount_center) + R5[9]*(-0.5*_center_length);
    }
    else if ( face1 == 6 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = 0.5*_center_length +  R1[0]*(_body_length + _body_end_depth) + R3[0]*(2*_end_depth) + R5[0]*(_body_end_depth + _body_length) + R7[0]*(0.5*_center_length);
        m[1] =                      R1[4]*(_body_length + _body_end_depth) + R3[4]*(2*_end_depth) + R5[4]*(_body_end_depth + _body_length) + R7[4]*(0.5*_center_length);
        m[2] =                      R1[8]*(_body_length + _body_end_depth) + R3[8]*(2*_end_depth) + R5[8]*(_body_end_depth + _body_length) + R7[8]*(0.5*_center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(CENTER, 0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(CENTER, 1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(CENTER, 2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                RAD2DEG(psi), RAD2DEG(theta), RAD2DEG(phi), r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4::build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
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
}

void CRobot4::build_center(dReal x, dReal y, dReal z, dMatrix3 R) {
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
}

void CRobot4::build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R) {
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
}

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
	_type = MOBOT;
}
