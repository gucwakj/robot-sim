#include "base.h"
#include "robosim.h"

CRobot::CRobot(void) {
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

CRobot::~CRobot(void) {
	// destroy connectors array
	conn_t ctmp = _conn;
	while (ctmp) {
		conn_t tmp = ctmp->next;
		delete [] ctmp->geom;
		delete ctmp;
		ctmp = tmp;
	}

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

int CRobot::blinkLED(double delay, int num) {
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

int CRobot::connect(char *name, int pause) {
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

int CRobot::delay(double milliseconds) {
	// set ending time
	double end = g_sim->getClock() + milliseconds/1000;

	// while clock hasn't reached ending time
	while ((end - g_sim->getClock()) >= EPSILON)
		this->doze(50);

	// success
	return 0;
}

int CRobot::delaySeconds(double seconds) {
	// delay milliseconds
	this->delay(1000 * seconds);

	// success
	return 0;
}

int CRobot::disableRecordDataShift(void) {
	_g_shift_data = 0;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CRobot::disconnect(void) {
	// and we are not connected
	_connected = 0;

	// success
	return 0;
}

int CRobot::enableRecordDataShift(void) {
	_g_shift_data = 1;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int CRobot::getBatteryVoltage(double &voltage) {
	voltage = 100;

	// success
	return 0;
}

int CRobot::getDistance(double &distance, double radius) {
	double angle;
	this->getJointAngle(JOINT1, angle, 2);
	distance = DEG2RAD(angle) * radius;

	// success
	return 0;
}

int CRobot::getFormFactor(int &formFactor) {
	formFactor = _type;

	// success
	return 0;
}

int CRobot::getID(void) {
	// get id of robot
	return _id;
}

int CRobot::getJointAngle(robotJointId_t id, double &angle, int numReadings) {
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

int CRobot::getJointAngleInstant(robotJointId_t id, double &angle) {
	angle = RAD2DEG(this->getAngle(id));

	// success
	return 0;
}

int CRobot::getJointMaxSpeed(robotJointId_t id, double &maxSpeed) {
	maxSpeed = RAD2DEG(_motor[id].omega_max);

	// success
	return 0;
}

int CRobot::getJointSafetyAngle(double &angle) {
	angle = _motor[JOINT1].safety_angle;

	// success
	return 0;
}

int CRobot::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _motor[JOINT1].safety_timeout;

	// success
	return 0;
}

int CRobot::getJointSpeed(robotJointId_t id, double &speed) {
	speed = RAD2DEG(_motor[id].omega);

	// success
	return 0;
}

int CRobot::getJointSpeedRatio(robotJointId_t id, double &ratio) {
	ratio = _motor[id].omega/_motor[id].omega_max;

	// success
	return 0;
}

int CRobot::holdJoint(robotJointId_t id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int CRobot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		this->setJointSpeed(static_cast<robotJointId_t>(i), 0);
	}

	// success
	return 0;
}

int CRobot::holdJointsAtExit(void) {
	// set joint speeds to zero
	this->holdJoints();

	// hold joints still
	this->moveForeverNB();

	// success
	return 0;
}

int CRobot::isConnected(void) {
	// return connected status
	return _connected;
}

int CRobot::isMoving(void) {
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].state == POSITIVE || _motor[i].state == NEGATIVE) {
			return 1;
		}
	}

	// success
	return 0;
}

int CRobot::isNotMoving(void) {
	// oppositve of ismoving
	return !(this->isMoving());
}

int CRobot::moveForeverNB(void) {
	// set joint movements
	for (int i = 0; i < _dof; i++) {
		this->moveJointForeverNB(static_cast<robotJointId_t>(i));
	}

	// success
	return 0;
}

int CRobot::moveJoint(robotJointId_t id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot::moveJointNB(robotJointId_t id, double angle) {
	// check if disabled joint
	if (_disabled == id) return 0;

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

int CRobot::moveJointForeverNB(robotJointId_t id) {
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

int CRobot::moveJointTime(robotJointId_t id, double seconds) {
	// move joint
	this->moveJointForeverNB(id);

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoint(id);

	// success
	return 0;
}

void* CRobot::moveJointTimeNBThread(void *arg) {
	// cast argument
	recordAngleArg_t *rArg = (recordAngleArg_t *)arg;

	// get robot
	CRobot *robot = dynamic_cast<CRobot *>(rArg->robot);
	// sleep
	robot->doze(rArg->msecs);
	// hold all robot motion
	robot->holdJoint(rArg->id);

	// cleanup
	delete rArg;

	// success
	return NULL;
}

int CRobot::moveJointTimeNB(robotJointId_t id, double seconds) {
	// set up threading
	THREAD_T moving;
	recordAngleArg_t *rArg = new recordAngleArg_t;
	rArg->robot = this;
	rArg->msecs = 1000*seconds;
	rArg->id = id;

	// set joint movements
	this->moveJointForeverNB(id);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&CRobot::moveJointTimeNBThread, (void *)rArg);

	// success
	return 0;
}

int CRobot::moveJointTo(robotJointId_t id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot::moveJointToNB(robotJointId_t id, double angle) {
	// check if disabled joint
	if (_disabled == id) return 0;

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

int CRobot::moveJointWait(robotJointId_t id) {
	// wait for motion to complete
	MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int CRobot::setJointSpeed(robotJointId_t id, double speed) {
	if (speed > RAD2DEG(_motor[id].omega_max)) {
		fprintf(stderr, "Warning: Setting the speed for joint %d to %.2lf degrees per second is "
			"beyond the hardware limit of %.2lf degrees per second.\n",
			id+1, speed, RAD2DEG(_motor[id].omega_max));
	}
	_motor[id].omega = DEG2RAD(speed);

	// success
	return 0;
}

int CRobot::setJointSpeedRatio(robotJointId_t id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * RAD2DEG(_motor[(int)id].omega_max));
}

/**********************************************************
	protected functions
 **********************************************************/
dBodyID CRobot::getBodyID(int id) {
	return _body[id];
}

dBodyID CRobot::getConnectorBodyID(int face) {
	conn_t ctmp = _conn;
	while (ctmp) {
		if (ctmp->face == face) {
			return ctmp->body;
		}
		ctmp = ctmp->next;
	}
	return NULL;
}

dBodyID CRobot::getConnectorBodyIDs(int num) {
	conn_t ctmp = _conn;
	int i = 0;
	while (ctmp && i++ < num)
		ctmp = ctmp->next;
	if (ctmp) {
		return ctmp->body;
	}
	return NULL;
}

dJointID CRobot::getMotorID(int id) {
    return _motor[id].id;
}

double CRobot::getAngle(int id) {
	if (_type == MOBOT && (id == JOINT2 || id == JOINT3))
		_motor[id].theta = dJointGetHingeAngle(_joint[id]);
	else if (id == _disabled)
		_motor[id].theta = 0;
	else
		_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;

	// add noise to angle
	//this->noisy(&(_motor[id].theta), 1, 0.0005);

    return _motor[id].theta;
}

double CRobot::getCenter(int i) {
	const double *pos = dBodyGetPosition(_body[0]);
	const double *R = dBodyGetRotation(_body[0]);
	double p[3] = {	R[0]*_center[0] + R[1]*_center[1] + R[2]*_center[2],
					R[4]*_center[0] + R[5]*_center[1] + R[6]*_center[2],
					R[8]*_center[0] + R[9]*_center[1] + R[10]*_center[2]};
	return pos[i] + p[i];
}

double CRobot::getRotation(int body, int i) {
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

int CRobot::addToSim(dWorldID &world, dSpaceID &space) {
	_world = world;
    _space = dHashSpaceCreate(space);

	// success
	return 0;
}

int CRobot::doze(double ms) {
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms*1000);
#endif
	// success
	return 0;
}

int CRobot::fixBodyToGround(dBodyID cbody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, 0, cbody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CRobot::getConnectorParams(int type, int side, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};
/*
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
		case SQUARE:
			if (side == 2) {
				offset[0] = _end_width/2;
				offset[1] = _end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI/2);
			}
			else if (side == 3) {
				offset[0] = _end_width;
				dRSetIdentity(R1);
			}
			else if (side == 4) {
				offset[0] = _end_width/2;
				offset[1] = -_end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], -M_PI/2);
			}
			break;
		case TANK:
			if (side == 2) {
				offset[0] = _tank_depth;
				dRSetIdentity(R1);
			}
			else if (side == 3) {
				offset[0] = _tank_depth/2;
				offset[2] = _tank_height - _connector_height/2;
				dRFromAxisAndAngle(R1, R[1], R[5], R[9], -M_PI/2);
			}
			break;
	}
*/
	// set output parameters
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int CRobot::getRobotID(void) {
	return _id;
}

int CRobot::getType(void) {
	return _type;
}

int CRobot::isShiftEnabled(void) {
	if(_shift_data && !_g_shift_data_en)
		return 1;
	else if (_g_shift_data_en && _g_shift_data)
		return 1;
	else
		return 0;
}

int CRobot::noisy(double *a, int length, double sigma) {
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

int CRobot::setID(int id) {
	_id = id;
	return 0;
}

void* CRobot::simPreCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPreCollisionThread();
	return arg;
}

void* CRobot::simPostCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPostCollisionThread();
	return arg;
}

/**********************************************************
	private functions
 **********************************************************/
double CRobot::mod_angle(double past_ang, double cur_ang, double ang_rate) {
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

double CRobot::normal(double sigma) {
	// compute pair of random uniform data
	double u1 = this->uniform();
	double u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*M_PI*u2));
}

double CRobot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
}

