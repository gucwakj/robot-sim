#include "linkbotsim.h"

CLinkbot::CLinkbot(int disabled, int type) {
	// initialize parameters
	init_params(disabled, type);
	// initialize dimensions
	init_dims();
}

CLinkbot::~CLinkbot(void) {
	dSpaceDestroy(_space);
}

int CLinkbot::connect(void) {
	return 0;
}

int CLinkbot::getJointAngle(int id, dReal &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CLinkbot::getJointSpeed(int id, dReal &speed) {
	speed = RAD2DEG(_speed[id]);

	// success
	return 0;
}

int CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	speed1 = RAD2DEG(_speed[0]);
	speed2 = RAD2DEG(_speed[1]);
	speed3 = RAD2DEG(_speed[2]);

	// success
	return 0;
}

int CLinkbot::motionArch(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, -angle/2.0);
	this->moveJointToNB(MOBOT_JOINT3, angle/2.0);
	this->moveJointWait(MOBOT_JOINT2);
	this->moveJointWait(MOBOT_JOINT3);

	// success
	return 0;
}

int CLinkbot::motionInchwormLeft(int num) {
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

int CLinkbot::motionInchwormRight(int num) {
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

int CLinkbot::motionRollBackward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::motionRollForward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::motionSkinny(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, angle);
	this->moveJointToNB(MOBOT_JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::motionStand(void) {
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

int CLinkbot::motionTumbleLeft(int num) {
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

int CLinkbot::motionTumbleRight(int num) {
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

int CLinkbot::motionTurnLeft(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::motionTurnRight(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::motionUnstand(void) {
	this->moveToDirect(0, 0, 0);
	this->moveJointToNB(MOBOT_JOINT3, 45);
	this->moveJointToNB(MOBOT_JOINT2, -85);
	this->moveWait();
	this->moveToDirect(0, 0, 0);

	// success
	return 0;
}

int CLinkbot::move(dReal angle1, dReal angle2, dReal angle3) {
	this->moveNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::moveNB(dReal angle1, dReal angle2, dReal angle3) {
	// store angles into array
	dReal delta[NUM_DOF] = {angle1, angle2, angle3};

	// lock mutexes
	this->simThreadsGoalWLock();
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// loop over joints
	for (int i = 0; i < ((_disabled == -1) ? 3 : 2); i++) {
		int j = _enabled[i];
		_goal[j] += DEG2RAD(delta[j]);
		dJointEnable(_motor[j]);
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = MOBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _speed[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = MOBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_speed[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = MOBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
		_success[j] = false;
	}

	// enable body
    dBodyEnable(_body[BODY]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CLinkbot::moveJoint(int id, dReal angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbot::moveJointNB(int id, dReal angle) {
	// check if disabled joint
	if (_disabled == id-1) return 0;

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[id] += DEG2RAD(angle);

	// enable motor
	MUTEX_LOCK(&_angle_mutex);
	dJointEnable(_motor[id]);

	// set motor state and velocity
	if ( angle > 0 ) {
		_state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _speed[id]);
	}
	else if ( angle < 0 ) {
		_state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_speed[id]);
	}
	else if ( fabs(angle-0) < EPSILON ) {
		_state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = false;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CLinkbot::moveJointTo(int id, dReal angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CLinkbot::moveJointToNB(int id, dReal angle) {
	// check if disabled joint
	if (_disabled == id-1) return 0;

	// store delta angle
	dReal delta = angle - _angle[id];

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	_goal[id] = DEG2RAD(angle);

	// enable motor
	MUTEX_LOCK(&_angle_mutex);
	dJointEnable(_motor[id]);

	// set motor state and velocity
	if ( delta > 0 ) {
		_state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, _speed[id]);
	}
	else if ( delta < 0 ) {
		_state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(_motor[id], dParamVel, -_speed[id]);
	}
	else if ( fabs(delta-0) < EPSILON ) {
		_state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(_motor[id], dParamVel, 0);
	}
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_angle_mutex);

	// set success to false
	MUTEX_LOCK(&_success_mutex);
	_success[id] = false;
	MUTEX_UNLOCK(&_success_mutex);

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CLinkbot::moveJointWait(int id) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ( !_success[id] ) { COND_WAIT(&_success_cond, &_success_mutex); }
	_success[id] = true;
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CLinkbot::moveTo(dReal angle1, dReal angle2, dReal angle3) {
	this->moveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::moveToDirect(dReal angle1, dReal angle2, dReal angle3) {
	// success
	return 0;
}

int CLinkbot::moveToNB(dReal angle1, dReal angle2, dReal angle3) {
	// store angles into array
	dReal delta[3] = {angle1 - _angle[0], angle2 - _angle[1], angle3 - _angle[2]};

	// lock mutexes
	this->simThreadsGoalWLock();
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// loop over joints
	for (int i = 0; i < ((_disabled == -1) ? 3 : 2); i++) {
		int j = _enabled[i];
		_goal[j] += DEG2RAD(delta[j]);
		dJointEnable(_motor[j]);
		dJointSetAMotorAngle(_motor[j], 0, _angle[j]);
		if ( delta[j] > 0 ) {
			_state[j] = LINKBOT_FORWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, _speed[j]);
		}
		else if ( delta[j] < 0 ) {
			_state[j] = LINKBOT_BACKWARD;
			dJointSetAMotorParam(_motor[j], dParamVel, -_speed[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			_state[j] = LINKBOT_HOLD;
			dJointSetAMotorParam(_motor[j], dParamVel, 0);
		}
		_success[j] = false;
	}

	// enable body
    dBodyEnable(_body[BODY]);

	// unlock mutexes
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	this->simThreadsGoalWUnlock();


	// success
	return 0;
}

int moveToDirectNB(dReal angle1, dReal angle2, dReal angle3) {
	// success
	return 0;
}

int CLinkbot::moveToZero(void) {
	this->moveTo(0, 0, 0);

	// success
	return 0;
}

int CLinkbot::moveToZeroNB(void) {
	this->moveToNB(0, 0, 0);

	// success
	return 0;
}

int CLinkbot::moveWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_success_mutex);
	while ( !(_success[0]) && !(_success[1]) && !(_success[2]) ) {
		COND_WAIT(&_success_cond, &_success_mutex);
	}
	_success[0] = true;
	_success[1] = true;
	_success[2] = true;

	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int CLinkbot::recordAngle(int id, dReal *time, dReal *angle, int num, dReal seconds, dReal threshold) {
	THREAD_T recording;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	if (_recording[id]) { return -1; }
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle;
	rArg->id = id;
	rArg->num = num;
	rArg->msecs = 1000*seconds;
	_recording[id] = true;
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbot::record_angle_thread, (void *)rArg);

	// success
	return 0;
}

int CLinkbot::recordAngles(dReal *time, dReal *angle1, dReal *angle2, dReal *angle3, int num, dReal seconds, dReal threshold) {
	THREAD_T recording;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	for (int i = 0; i < NUM_DOF; i++) {
		if (_recording[i]) { return -1; }
	}
	rArg->robot = this;
	rArg->time = time;
	rArg->angle1 = angle1;
	rArg->angle2 = angle2;
	rArg->angle3 = angle3;
	rArg->num = num;
	rArg->msecs = 1000*seconds;
	for (int i = 0; i < NUM_DOF; i++) {
		_recording[i] = true;
	}
	THREAD_CREATE(&recording, (void* (*)(void *))&CLinkbot::record_angles_thread, (void *)rArg);

	// success
	return 0;
}

int CLinkbot::recordWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_recording_mutex);
	while ( _recording[0] || _recording[1] || _recording[2] ) {
		COND_WAIT(&_recording_cond, &_recording_mutex);
	}
	_recording[0] = false;
	_recording[1] = false;
	_recording[2] = false;
	MUTEX_UNLOCK(&_recording_mutex);

	// success
	return 0;
}

int CLinkbot::resetToZero(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_angle_mutex);
	for (int i = 0; i < 3; i++) {
		int rev = (int)(_angle[i]/2/M_PI);
		if (rev) _angle[i] -= 2*rev*M_PI;
	}
	MUTEX_UNLOCK(&_angle_mutex);

	// move to zero position
	this->moveToZero();

	// success
	return 0;
}

int CLinkbot::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_angle_mutex);
	for (int i = 0; i < 3; i++) {
		int rev = (int)(_angle[i]/2/M_PI);
		if (rev) _angle[i] -= 2*rev*M_PI;
	}
	MUTEX_UNLOCK(&_angle_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int CLinkbot::setJointSpeed(int id, double speed) {
	_speed[id] = DEG2RAD((speed > _maxSpeed[id]) ? _maxSpeed[id] : speed);

	// success
	return 0;
}

int CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3) {
	_speed[0] = DEG2RAD((speed1 > _maxSpeed[0]) ? _maxSpeed[0] : speed1);
	_speed[1] = DEG2RAD((speed2 > _maxSpeed[1]) ? _maxSpeed[1] : speed2);
	_speed[2] = DEG2RAD((speed3 > _maxSpeed[2]) ? _maxSpeed[2] : speed3);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CLinkbot::addToSim(dWorldID &world, dSpaceID &space, dReal *clock) {
	_world = world;
    _space = dHashSpaceCreate(space);
	_clock = clock;

	// success
	return 0;
}

int CLinkbot::build(bot_t robot) {
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
			robot->z += ((ctmp->type == SMALLWHEEL) ? _smallwheel_radius : _bigwheel_radius) - _body_height/2;
			break;
		}
		ctmp = ctmp->next;
	}

	// build robot
	this->build_individual(robot->x, robot->y, robot->z, R, robot->angle1, robot->angle2, robot->angle3);

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

int CLinkbot::build(bot_t robot, CRobot *base, Conn_t *conn) {
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
	//const dReal *pos = dBodyGetPosition(_body[BODY]);
	//printf("robot pos: %lf %lf %lf\n", pos[0], pos[1], pos[2]);

	// success
	return 0;
}

dReal CLinkbot::getAngle(int i) {
	_angle[i] = mod_angle(_angle[i], dJointGetHingeAngle(_joint[i]), dJointGetHingeAngleRate(_joint[i]));
    return _angle[i];
}

dBodyID CLinkbot::getBodyID(int id) {
    return _body[id];
}

int CLinkbot::getConnectionParams(int face, dMatrix3 R, dReal *p) {
	const dReal *pos, *R1;
	dMatrix3 R2;
	double offset[3] = {0};

	switch (face) {
		case 1:
			pos = dBodyGetPosition(_body[FACE1]);
			R1 = dBodyGetRotation(_body[FACE1]);
			offset[1] = _face_depth/2;
			p[0] = pos[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[5]*offset[1];
			p[2] = pos[2] + R1[9]*offset[1];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 2:
			pos = dBodyGetPosition(_body[FACE2]);
			R1 = dBodyGetRotation(_body[FACE2]);
			offset[0] = -_face_depth/2;
			p[0] = pos[0] + R1[0]*offset[0];
			p[1] = pos[1] + R1[4]*offset[0];
			p[2] = pos[2] + R1[8]*offset[0];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 3:
			pos = dBodyGetPosition(_body[FACE3]);
			R1 = dBodyGetRotation(_body[FACE3]);
			offset[1] = -_face_depth/2;
			p[0] = pos[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[5]*offset[1];
			p[2] = pos[2] + R1[9]*offset[1];
    		dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], -M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
	}

	// success
	return 0;
}

dBodyID CLinkbot::getConnectorBodyID(int face) {
	conn_t ctmp = _conn;
	while (ctmp) {
		if (ctmp->face == face) {
			return ctmp->body;
		}
		ctmp = ctmp->next;
	}
	return NULL;
}

dBodyID CLinkbot::getConnectorBodyIDs(int num) {
	conn_t ctmp = _conn;
	int i = 0;
	while (ctmp && i++ < num)
		ctmp = ctmp->next;
	if (ctmp) {
		return ctmp->body;
	}
	return NULL;
}

int CLinkbot::getID(void) {
	return _id;
}

dJointID CLinkbot::getMotorID(int id) {
    return _motor[id];
}

dReal CLinkbot::getPosition(int body, int i) {
	const dReal *pos = dBodyGetPosition(_body[body]);
	return pos[i];
}

dReal CLinkbot::getRotation(int body, int i) {
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

bool CLinkbot::getSuccess(int i) {
	return _success[i];
}

int CLinkbot::getType(void) {
	return _type;
}

bool CLinkbot::isHome(void) {
    return ( fabs(_angle[FACE1]) < EPSILON && fabs(_angle[FACE2]) < EPSILON && fabs(_angle[FACE3]) < EPSILON );
}

int CLinkbot::setID(int id) {
	_id = id;
	return 0;
}

void CLinkbot::simPreCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	MUTEX_LOCK(&_angle_mutex);

	// update angle values for each degree of freedom
	for ( int i = 0; i < NUM_DOF; i++ ) {
		// update current angle
		_angle[i] = getAngle(i);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i], 0, _angle[i]);
		// drive motor to get current angle to match future angle
		if (_angle[i] < _goal[i] - _encoderResolution)
			dJointSetAMotorParam(_motor[i], dParamVel, _speed[i]);
		else if (_angle[i] > _goal[i] + _encoderResolution)
			dJointSetAMotorParam(_motor[i], dParamVel, -_speed[i]);
		else
			dJointSetAMotorParam(_motor[i], dParamVel, 0);
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_angle_mutex);
	this->simThreadsGoalRUnlock();
}

void CLinkbot::simPostCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	MUTEX_LOCK(&_angle_mutex);
	MUTEX_LOCK(&_success_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < NUM_DOF; i++) {
		_success[i] = (bool)(!(int)(dJointGetAMotorParam(this->getMotorID(i), dParamVel)*1000) );
	}
	if (_success[0] && _success[1] && _success[2]) {
		COND_SIGNAL(&_success_cond);
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_angle_mutex);
	this->simThreadsGoalRUnlock();
}

#ifdef ENABLE_GRAPHICS
void CLinkbot::draw(osg::Group *root) {
	// initialize variables
	osg::ref_ptr<osg::Group> robot = new osg::Group();
	osg::ref_ptr<osg::Geode> body[NUM_PARTS];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[NUM_PARTS];
	osg::ref_ptr<osg::Texture2D> tex[2];
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for ( int i = 0; i < NUM_PARTS; i++) {
		body[i] = new osg::Geode;
	}

	// body
	pos = dGeomGetOffsetPosition(_geom[0][0]);
	dGeomGetOffsetQuaternion(_geom[0][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _body_length, _body_width, _body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(_geom[0][1]);
	dGeomGetOffsetQuaternion(_geom[0][1], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _body_radius, _body_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));
    
    // face1
	pos = dGeomGetOffsetPosition(_geom[1][0]);
	dGeomGetOffsetQuaternion(_geom[1][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[1]->addDrawable(new osg::ShapeDrawable(cyl));

    // face 2
	pos = dGeomGetOffsetPosition(_geom[2][0]);
	dGeomGetOffsetQuaternion(_geom[2][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[2]->addDrawable(new osg::ShapeDrawable(cyl));

    // face 3
	pos = dGeomGetOffsetPosition(_geom[3][0]);
	dGeomGetOffsetQuaternion(_geom[3][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), _face_radius, _face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[3]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex[0] = new osg::Texture2D(
		osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/body.png"));
	tex[1] = new osg::Texture2D(
		osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/face.png"));
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
	if (_disabled > 0) {
    	body[_disabled+1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	}

	// position each body within robot
	for (int i = 0; i < NUM_PARTS; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		robot->addChild(pat[i].get());
	}

	conn_t ctmp = _conn;
	while (ctmp) {
		switch (ctmp->type) {
			case L_BIGWHEEL:
				this->draw_bigwheel(ctmp, robot);
				break;
			case L_CASTER:
				this->draw_caster(ctmp, robot);
				break;
			case L_SIMPLE:
				this->draw_simple(ctmp, robot);
				break;
			case L_SMALLWHEEL:
				this->draw_smallwheel(ctmp, robot);
				break;
		}
		ctmp = ctmp->next;
	}

	// set update callback for robot
	robot->setUpdateCallback(new linkbotNodeCallback(this));

	// add to scenegraph
	root->addChild(robot);
}
#endif // ENABLE_GRAPHICS

/**********************************************************
	private functions
 **********************************************************/
int CLinkbot::add_connector(int type, int face) {
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
		case L_BIGWHEEL:
			this->build_bigwheel(nc, face);
			break;
		case L_CASTER:
			this->build_caster(nc, face);
			break;
		case L_SIMPLE:
			this->build_simple(nc, face);
			break;
		case L_SMALLWHEEL:
			this->build_smallwheel(nc, face);
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

int CLinkbot::build_individual(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_f1, dReal r_f2, dReal r_f3) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { _body[i] = dBodyCreate(_world); }
    _geom[BODY] = new dGeomID[2];
    _geom[FACE1] = new dGeomID[1];
    _geom[FACE2] = new dGeomID[1];
    _geom[FACE3] = new dGeomID[1];

	// initialize PID class
	//for ( int i = 0; i < NUM_DOF; i++ ) { _pid[i].init(100, 1, 10, 0.1, 0.004); }

    // adjust input height by body height
	if (z < _body_height/2) {
    	x += R[2]*_body_height/2;
    	y += R[6]*_body_height/2;
    	z += R[10]*_body_height/2;
	}

    // convert input angles to radians
    _angle[F1] = DEG2RAD(r_f1);	// face 1
    _angle[F2] = DEG2RAD(r_f2);	// face 2
    _angle[F3] = DEG2RAD(r_f3);	// face 3

	// offset values for each body part[0-2] and joint[3-5] from center
	dReal b[3] = {-_body_length/2, 0, 0};
	dReal f1[6] = {0, _body_width/2 + _face_depth/2, 0, 0, _body_width/2, 0};
	dReal f2[6] = {-_body_length - _face_depth/2, 0, -_body_length, 0, 0};
	dReal f3[6] = {0, -_body_width/2 - _face_depth/2, 0, 0, -_body_width/2, 0};

	// build robot bodies
	this->build_body(R[0]*b[0] + x, R[4]*b[0] + y, R[8]*b[0] + z, R, 0);
	this->build_face(FACE1, R[1]*f1[1] + x, R[5]*f1[1] + y, R[9]*f1[1] + z, R, 0);
	this->build_face(FACE2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R, 0);
	this->build_face(FACE3, R[1]*f3[1] + x, R[5]*f3[1] + y, R[9]*f3[1] + z, R, 0);

    // joint for body to face 1
    _joint[0] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[0], _body[BODY], _body[FACE1]);
    dJointSetHingeAnchor(_joint[0], R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x, 
									R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
									R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
    dJointSetHingeAxis(_joint[0], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(_joint[0], dParamCFM, 0);

    // joint for body to face 2
    _joint[1] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[1], _body[BODY], _body[FACE2]);
    dJointSetHingeAnchor(_joint[1], R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
									R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
									R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
    dJointSetHingeAxis(_joint[1], R[0], R[4], R[8]);
    dJointSetHingeParam(_joint[1], dParamCFM, 0);

    // joint for body to face 3
    _joint[2] = dJointCreateHinge(_world, 0);
    dJointAttach(_joint[2], _body[BODY], _body[FACE3]);
    dJointSetHingeAnchor(_joint[2], R[0]*f3[3] + R[1]*f3[4] + R[2]*f3[5] + x,
									R[4]*f3[3] + R[5]*f3[4] + R[6]*f3[5] + y,
									R[8]*f3[3] + R[9]*f3[4] + R[10]*f3[5] + z);
    dJointSetHingeAxis(_joint[2], R[1], R[5], R[9]);
    dJointSetHingeParam(_joint[2], dParamCFM, 0);

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2, R_f3;
    dRFromAxisAndAngle(R_f, 0, 1, 0, _angle[F1]);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
    dRFromAxisAndAngle(R_f, -1, 0, 0, _angle[F2]);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);
    dRFromAxisAndAngle(R_f, 0, -1, 0, _angle[F3]);
    dMultiply0(R_f3, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _angle[F1] != 0 || _angle[F2] != 0 ||_angle[F3] != 0 ) {
    	// re-build pieces of module
		this->build_face(FACE1, R[1]*f1[1] + x, R[5]*f1[1] + y, R[9]*f1[1] + z, R_f1, 0);
		this->build_face(FACE2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R_f2, 0);
		this->build_face(FACE3, R[1]*f3[1] + x, R[5]*f3[1] + y, R[9]*f3[1] + z, R_f3, 0);
	}

    // motor for body to face 1
    _motor[F1] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F1], _body[BODY], _body[FACE1]);
    dJointSetAMotorMode(_motor[F1], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F1], 1);
    dJointSetAMotorAxis(_motor[F1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(_motor[F1], 0, 0);
    dJointSetAMotorParam(_motor[F1], dParamCFM, 0);
    dJointSetAMotorParam(_motor[F1], dParamFMax, _maxJointForce[F1]);
	dJointDisable(_motor[F1]);

    // motor for body to face 2
    _motor[F2] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F2], _body[BODY], _body[FACE2]);
    dJointSetAMotorMode(_motor[F2], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F2], 1);
    dJointSetAMotorAxis(_motor[F2], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[F2], 0, 0);
    dJointSetAMotorParam(_motor[F2], dParamCFM, 0);
    dJointSetAMotorParam(_motor[F2], dParamFMax, _maxJointForce[F2]);
	dJointDisable(_motor[F2]);

    // motor for body to face 3
    _motor[F3] = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[F3], _body[BODY], _body[FACE3]);
    dJointSetAMotorMode(_motor[F3], dAMotorUser);
    dJointSetAMotorNumAxes(_motor[F3], 1);
    dJointSetAMotorAxis(_motor[F3], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(_motor[F3], 0, 0);
    dJointSetAMotorParam(_motor[F3], dParamCFM, 0);
    dJointSetAMotorParam(_motor[F3], dParamFMax, _maxJointForce[F3]);
	dJointDisable(_motor[F3]); 

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CLinkbot::build_attached(bot_t robot, CRobot *base, Conn_t *conn) {
	// initialize new variables
	int i = 1;
	dReal m[3] = {0}, offset[3] = {0};
	dMatrix3 R, R1, R2, R3, R4, R5, R6;

	// generate parameters for base robot
	base->getConnectionParams(conn->face1, R, m);

	// generate parameters for connector
	this->get_connector_params(conn, R, m);

	// collect data from struct
	dReal r_f1 = robot->angle1;
	dReal r_f2 = robot->angle2;
	dReal r_f3 = robot->angle3;

	switch (conn->face2) {
		case 2:
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], 0);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], DEG2RAD(r_f2));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_f2));
			offset[0] = _body_length + R1[0]*_body_radius;
			offset[1] = R1[4]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 3: case 1:
			i = (conn->face2 == 3) ? -1 : 1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], i*M_PI/2);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -DEG2RAD(r_f2));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_f2));
			offset[0] = _body_width/2;
			offset[1] = i*_body_length + i*R1[0]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
	}

	// adjust position by rotation matrix
	m[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	m[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	m[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];

    // build new module
	this->build_individual(m[0], m[1], m[2], R6, r_f1, r_f2, r_f3);

    // add fixed joint to attach two modules
	this->fix_body_to_connector(base->getConnectorBodyID(conn->face1), conn->face2);

	// success
	return 0;
}

int CLinkbot::build_body(dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 2700, _body_length, _body_height, _body_width);
    dMassAdd(&m, &m1);
    //dMassSetParameters( &m, 500, 1, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[BODY], x, y, z);
    dBodySetRotation(_body[BODY], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - square
    _geom[BODY][0] = dCreateBox(_space, _body_length, _body_width, _body_height);
    dGeomSetBody( _geom[BODY][0], _body[BODY]);
    dGeomSetOffsetPosition(_geom[BODY][0], -m.c[0], -m.c[1], -m.c[2]);
    // set geometry 2 - curve
	_geom[BODY][1] = dCreateCylinder(_space, _body_radius, _body_width);
	dGeomSetBody( _geom[BODY][1], _body[BODY]);
	dGeomSetOffsetPosition(_geom[BODY][1], _body_length/2 - m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation( _geom[BODY][1], R2);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[BODY], &m);

	// success
	return 0;
}

int CLinkbot::build_face(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 270, _face_depth, 2*_face_radius, 2*_face_radius);
    dMassAdd(&m, &m1);
    //dMassSetParameters( &m, 500, 1, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[id], x, y, z);
    dBodySetRotation(_body[id], R);

    // rotation matrix for curves of d-shapes
	if ( id == 2)
	    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);
	else
	    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 2 - curve
	_geom[id][0] = dCreateCylinder(_space, _face_radius, _face_depth);
	dGeomSetBody( _geom[id][0], _body[id]);
	dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation( _geom[id][0], R2);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CLinkbot::build_bigwheel(conn_t conn, int face) {
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
    dMassSetBox(&m, 270, _connector_depth/2, _body_width, _connector_height);
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

int CLinkbot::build_caster(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[10];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double	depth = _connector_depth,
			width = 2*_face_radius,
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
	conn->geom[7] = dCreateBox(_space, 0.04, 0.021, 0.0032);
	dGeomSetBody(conn->geom[7], conn->body);
	dGeomSetOffsetPosition(conn->geom[7], depth/2 + 0.04/2 - m.c[0], -m.c[1], -height/2 + 0.0016 - m.c[2]);

    // set geometry 9 - ball support
    conn->geom[8] = dCreateCylinder(_space, 0.0105, 0.014);
    dGeomSetBody(conn->geom[8], conn->body);
    dGeomSetOffsetPosition(conn->geom[8], depth/2 + 0.04 - m.c[0], -m.c[1], -height/2 - 0.0064 - m.c[2]);

    // set geometry 10 - sphere
    conn->geom[9] = dCreateSphere(_space, 0.0105);
    dGeomSetBody(conn->geom[9], conn->body);
    dGeomSetOffsetPosition(conn->geom[9], depth/2 + 0.04 - m.c[0], -m.c[1], -height/2 - 0.0159 - m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CLinkbot::build_simple(conn_t conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

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
    dMassSetBox(&m, 270, _connector_depth, 2*_face_radius, _connector_height);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // set geometry 1 - box
    conn->geom[0] = dCreateBox(_space, _connector_depth, 2*_face_radius, _connector_height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fix_connector_to_body(face, conn->body);

	// success
	return 0;
}

int CLinkbot::build_smallwheel(conn_t conn, int face) {
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
    dMassSetBox(&m, 270, _connector_depth/2, 2*_face_radius, _connector_height);
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

int CLinkbot::fix_body_to_connector(dBodyID cBody, int face) {
	if (!cBody) { fprintf(stderr,"connector body does not exist\n"); }

	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));
	/*switch (face) {
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
	}*/

	// set joint params
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);

	// success
	return 0;
}

int CLinkbot::fix_connector_to_body(int face, dBodyID cBody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, this->getBodyID(face), cBody);
	/*switch (face) {
		case 2:
			dJointAttach(joint, this->getBodyID(FACE2), cBody);
			break;
		case 3: case 1:
			dJointAttach(joint, this->getBodyID(BODY_L), cBody);
			break;
	}*/

	// set joint params
	dJointSetFixed(joint);
	dJointSetFixedParam(joint, dParamCFM, 0);
	dJointSetFixedParam(joint, dParamERP, 0.9);

	// success
	return 0;
}

int CLinkbot::get_connector_params(Conn_t *conn, dMatrix3 R, dReal *p) {
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
	}
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int CLinkbot::init_params(int disabled, int type) {
	_disabled = disabled;
	_enabled = new int[(_disabled == -1) ? 3 : 2];
	for (int i = 0, j = 0; i < NUM_DOF; i++) {
		_angle[i] = 0;
		_goal[i] = 0;
		_recording[i] = false;
		_success[i] = true;
		_speed[i] = 0.7854;		// 45 deg/sec
		_maxSpeed[i] = 120;		// deg/sec
		if (i != _disabled)
			_enabled[j++] = i;
	}
	_conn = NULL;
	_id = -1;
	_type = type;
	_encoderResolution = DEG2RAD(0.5);
	_maxJointForce[LINKBOT_JOINT1] = 1.059;
	_maxJointForce[LINKBOT_JOINT2] = 1.059;
	_maxJointForce[LINKBOT_JOINT3] = 1.059;

	// init locks
	MUTEX_INIT(&_angle_mutex);
	this->simThreadsGoalInit();
	MUTEX_INIT(&_recording_mutex);
	COND_INIT(&_recording_cond);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);

	// success
	return 0;
}

int CLinkbot::init_dims(void) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_connector_depth = 0.00380;
	_connector_height = 0.03715;
	_bigwheel_radius = 0.04440;
	_smallwheel_radius = 0.04130;

	// success
	return 0;
}

dReal CLinkbot::mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate) {
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

/*void CLinkbot::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}*/

void* CLinkbot::record_angle_thread(void *arg) {
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
	SIGNAL(&rArg->robot->_recording_cond, &rArg->robot->_recording_mutex, rArg->robot->_recording[rArg->id] = false);
	return NULL;
}

void* CLinkbot::record_angles_thread(void *arg) {
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

		// increment time step
		time += rArg->msecs;
		// pause until next step
		while ( (int)(*(rArg->robot->_clock)*1000) < time ) {}
    }
	MUTEX_LOCK(&rArg->robot->_recording_mutex);
    for (int i = 0; i < NUM_DOF; i++) {
        rArg->robot->_recording[i] = false;
    }
	COND_SIGNAL(&rArg->robot->_recording_cond);
	MUTEX_UNLOCK(&rArg->robot->_recording_mutex);
	return NULL;
}

#ifdef ENABLE_GRAPHICS
void CLinkbot::draw_bigwheel(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/conn.png"));
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

void CLinkbot::draw_caster(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	osg::Sphere *sph;
	double	depth = _connector_depth,
			width = 2*_face_radius,
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/conn.png"));
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

void CLinkbot::draw_simple(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;

	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, 2*_face_radius, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/conn.png"));
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

void CLinkbot::draw_smallwheel(conn_t conn, osg::Group *robot) {
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
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/conn.png"));
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

void CLinkbot::draw_square(conn_t conn, osg::Group *robot) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	const dReal *pos;
	dQuaternion quat;
	osg::Box *box;

	// draw geoms
	pos = dGeomGetOffsetPosition(conn->geom[0]);
	dGeomGetOffsetQuaternion(conn->geom[0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, 2*_face_radius, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[1]);
	dGeomGetOffsetQuaternion(conn->geom[1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 2*_face_radius- 2*_connector_depth, _connector_depth, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[2]);
	dGeomGetOffsetQuaternion(conn->geom[2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 2*_face_radius- 2*_connector_depth, _connector_depth, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(conn->geom[3]);
	dGeomGetOffsetQuaternion(conn->geom[3], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), _connector_depth, 2*_face_radius, _connector_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body->addDrawable(new osg::ShapeDrawable(box));

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile("/usr/local/ch/package/chrobotsim/data/linkbot/conn.png"));
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
	CLinkbot(1, LINKBOTI);
}

CLinkbotI::~CLinkbotI(void) {
}

CLinkbotL::CLinkbotL(void) {
	CLinkbot(2, LINKBOTL);
}

CLinkbotL::~CLinkbotL(void) {
}
#endif
