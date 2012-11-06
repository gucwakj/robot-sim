#include "mobot.h"

CRobot4Sim::CRobot4Sim(void) {
	this->angle[0] = 0;
	this->angle[1] = 0;
	this->angle[2] = 0;
	this->angle[3] = 0;
	this->velocity[0] = 0.7854;	// 45 deg/sec
	this->velocity[1] = 0.7854;	// 45 deg/sec
	this->velocity[2] = 0.7854;	// 45 deg/sec
	this->velocity[3] = 0.7854;	// 45 deg/sec
	this->success[0] = true;
	this->success[1] = true;
	this->success[2] = true;
	this->success[3] = true;

	// init locks
	pthread_mutex_init(&angle_mutex, NULL);
	this->simThreadsGoalInit(NULL);
	pthread_mutex_init(&success_mutex, NULL);
	pthread_cond_init(&success_cond, NULL);
}

CRobot4Sim::~CRobot4Sim(void) {
	//dSpaceDestroy(this->space); //sigsegv
}

void CRobot4Sim::simPreCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	this->simThreadsAngleLock();

	// update angle values for each degree of freedom
	for ( int j = 0; j < NUM_DOF; j++ ) {
		// set motor angle to current angle
		dJointSetAMotorAngle(this->getMotorID(j), 0, this->getAngle(j));
		// drive motor to get current angle to match future angle
		this->update_joint_speed(j);
	}

	// unlock angle and goal
	this->simThreadsAngleUnlock();
	this->simThreadsGoalRUnlock();
}

void CRobot4Sim::simPostCollisionThread(void) {
	// lock angle and goal
	this->simThreadsGoalRLock();
	this->simThreadsAngleLock();

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < NUM_DOF; i++) {
		this->success[i] = this->is_joint_complete(i);
	}
	if ( this->success[0] && this->success[1] && this->success[2] && this->success[3] ) {
		pthread_cond_signal(&(this->success_cond));
	}

	// unlock angle and goal
	this->simThreadsAngleUnlock();
	this->simThreadsGoalRUnlock();
}

void CRobot4Sim::simAddRobot(dWorldID &world, dSpaceID &space) {
	this->world = world;
    this->space = dHashSpaceCreate(space);
}

dReal CRobot4Sim::getAngle(int i) {
	if (i == LE || i == RE)
		this->angle[i] = mod_angle(this->angle[i], dJointGetHingeAngle(this->joint[i]), dJointGetHingeAngleRate(this->joint[i]));
	else
		this->angle[i] = dJointGetHingeAngle(this->joint[i]);
    return this->angle[i];
}

bool CRobot4Sim::getSuccess(int i) {
	return this->success[i];
}

int CRobot4Sim::getJointAngle(int id, dReal &angle) {
	angle = R2D(this->getAngle(id));

	// success
	return 0;
}

dReal CRobot4Sim::getPosition(int i) {
    return this->position[i];
}

dReal CRobot4Sim::getRotation(int i) {
    return this->rotation[i];
}

dBodyID CRobot4Sim::getBodyID(int id) {
    return this->body[id];
}

dJointID CRobot4Sim::getMotorID(int id) {
    return this->motor[id];
}

bool CRobot4Sim::isHome(void) {
    return ( fabs(this->angle[LE]) < EPSILON && fabs(this->angle[LB]) < EPSILON && fabs(this->angle[RB]) < EPSILON && fabs(this->angle[RE]) < EPSILON );
}

/*bool CRobot4Sim::isComplete(void) {
	// initialze loop counters
	int c = 0, i;

	// check if joint speed is zero -> joint has completed step
	for (i = 0; i < NUM_DOF; i++) {
		this->success[i] = this->is_joint_complete(i);
	}
	if ( this->success[0] && this->success[1] && this->success[2] && this->success[3] ) {
		pthread_cond_signal(&(this->success_cond));
		return true;
	}
	return false;
}*/

bool CRobot4Sim::is_joint_complete(int id) {
	// check if joint speed is zero -> joint has completed step
	if ( !(int)(dJointGetAMotorParam(this->getMotorID(id), dParamVel)*1000) ) 
		return true;
	else
		return false;
}

int CRobot4Sim::motionArch(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, -angle/2.0);
	this->moveJointToNB(MOBOT_JOINT3, angle/2.0);
	this->moveJointWait(MOBOT_JOINT2);
	this->moveJointWait(MOBOT_JOINT3);

	// success
	return 0;
}

int CRobot4Sim::motionInchwormLeft(int num) {
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

int CRobot4Sim::motionInchwormRight(int num) {
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

int CRobot4Sim::motionRollBackward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::motionRollForward(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::motionSkinny(dReal angle) {
	this->moveJointToNB(MOBOT_JOINT2, angle);
	this->moveJointToNB(MOBOT_JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::motionStand(void) {
	this->resetToZero();
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

int CRobot4Sim::motionTumbleLeft(int num) {
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

int CRobot4Sim::motionTumbleRight(int num) {
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

int CRobot4Sim::motionTurnLeft(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] - angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::motionTurnRight(dReal angle) {
	dReal motorPosition[2];
	this->getJointAngle(MOBOT_JOINT1, motorPosition[0]);
	this->getJointAngle(MOBOT_JOINT4, motorPosition[1]);
	this->moveJointToNB(MOBOT_JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(MOBOT_JOINT4, motorPosition[1] - angle);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::motionUnstand(void) {
	this->moveToDirect(0, 0, 0, 0);
	this->moveJointToNB(MOBOT_JOINT3, 45);
	this->moveJointToNB(MOBOT_JOINT2, -85);
	this->moveWait();
	this->moveToDirect(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4Sim::move(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	this->moveNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::moveNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	// store angles into array
	dReal delta[4] = {angle1, angle2, angle3, angle4};

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	this->goal[0] += D2R(angle1);
	this->goal[1] += D2R(angle2);
	this->goal[2] += D2R(angle3);
	this->goal[3] += D2R(angle4);

	// enable motor
	this->simThreadsAngleLock();
	for ( int j = 0; j < NUM_DOF; j++ ) {
		dJointEnable(this->motor[j]);
		dJointSetAMotorAngle(this->motor[j], 0, this->angle[j]);
		if ( delta[j] > 0 ) {
			this->state[j] = MOBOT_FORWARD;
			dJointSetAMotorParam(this->motor[j], dParamVel, this->velocity[j]);
		}
		else if ( delta[j] < 0 ) {
			this->state[j] = MOBOT_BACKWARD;
			dJointSetAMotorParam(this->motor[j], dParamVel, -this->velocity[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			this->state[j] = MOBOT_HOLD;
			dJointSetAMotorParam(this->motor[j], dParamVel, 0);
		}
	}
    dBodyEnable(this->body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	this->success[0] = false;
	this->success[1] = false;
	this->success[2] = false;
	this->success[3] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4Sim::moveJoint(int id, dReal angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot4Sim::moveJointNB(int id, dReal angle) {
	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	this->goal[id] += D2R(angle);

	// enable motor
	this->simThreadsAngleLock();
	dJointEnable(this->motor[id]);

	// set motor state and velocity
	if ( angle > 0 ) {
		this->state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(this->motor[id], dParamVel, this->velocity[id]);
	}
	else if ( angle < 0 ) {
		this->state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(this->motor[id], dParamVel, -this->velocity[id]);
	}
	else if ( fabs(angle-0) < EPSILON ) {
		this->state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(this->motor[id], dParamVel, 0);
	}
	dBodyEnable(this->body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	this->success[id] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4Sim::moveJointTo(int id, dReal angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int CRobot4Sim::moveJointToNB(int id, dReal angle) {
	// store delta angle
	dReal delta = angle - this->angle[id];

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	this->goal[id] = D2R(angle);

	// enable motor
	this->simThreadsAngleLock();
	dJointEnable(this->motor[id]);

	// set motor state and velocity
	if ( delta > 0 ) {
		this->state[id] = MOBOT_FORWARD;
		dJointSetAMotorParam(this->motor[id], dParamVel, this->velocity[id]);
	}
	else if ( delta < 0 ) {
		this->state[id] = MOBOT_BACKWARD;
		dJointSetAMotorParam(this->motor[id], dParamVel, -this->velocity[id]);
	}
	else if ( fabs(delta-0) < EPSILON ) {
		this->state[id] = MOBOT_HOLD;
		dJointSetAMotorParam(this->motor[id], dParamVel, 0);
	}
	dBodyEnable(this->body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	this->success[id] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int CRobot4Sim::moveJointWait(int id) {
	// wait for motion to complete
	this->simThreadsSuccessLock();
	while ( !this->success[id] ) { pthread_cond_wait(&(this->success_cond), &(this->success_mutex)); }
	this->success[id] = true;
	this->simThreadsSuccessUnlock();

	// success
	return 0;
}

int CRobot4Sim::moveTo(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CRobot4Sim::moveToDirect(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
}

int CRobot4Sim::moveToNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
	// store angles into array
	dReal delta[4] = {angle1 - this->angle[0], angle2 - this->angle[1], angle3 - this->angle[2], angle4 - this->angle[3]};

	// lock goal
	this->simThreadsGoalWLock();

	// set new goal angles
	this->goal[0] = D2R(angle1);
	this->goal[1] = D2R(angle2);
	this->goal[2] = D2R(angle3);
	this->goal[3] = D2R(angle4);

	// enable motor
	this->simThreadsAngleLock();
	for ( int j = 0; j < NUM_DOF; j++ ) {
		dJointEnable(this->motor[j]);
		dJointSetAMotorAngle(this->motor[j], 0, this->angle[j]);
		if ( delta[j] > 0 ) {
			this->state[j] = MOBOT_FORWARD;
			dJointSetAMotorParam(this->motor[j], dParamVel, this->velocity[j]);
		}
		else if ( delta[j] < 0 ) {
			this->state[j] = MOBOT_BACKWARD;
			dJointSetAMotorParam(this->motor[j], dParamVel, -this->velocity[j]);
		}
		else if ( fabs(delta[j]-0) < EPSILON ) {
			this->state[j] = MOBOT_HOLD;
			dJointSetAMotorParam(this->motor[j], dParamVel, 0);
		}
	}
    dBodyEnable(this->body[CENTER]);
	this->simThreadsAngleUnlock();

	// set success to false
	this->simThreadsSuccessLock();
	this->success[0] = false;
	this->success[1] = false;
	this->success[2] = false;
	this->success[3] = false;
	this->simThreadsSuccessUnlock();

	// unlock goal
	this->simThreadsGoalWUnlock();

	// success
	return 0;
}

int moveToDirectNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4) {
}

int CRobot4Sim::moveToZero(void) {
	this->moveTo(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4Sim::moveToZeroNB(void) {
	this->moveToNB(0, 0, 0, 0);

	// success
	return 0;
}

int CRobot4Sim::moveWait(void) {
	// wait for motion to complete
	this->simThreadsSuccessLock();
	while ( !this->success[0] && !this->success[1] && !this->success[2] && !this->success[3]) {
		pthread_cond_wait(&(this->success_cond), &(this->success_mutex));
	}
	this->success[0] = true;
	this->success[1] = true;
	this->success[2] = true;
	this->success[3] = true;
	this->simThreadsSuccessUnlock();

	// success
	return 0;
}

int CRobot4Sim::resetToZero(void) {
	// reset absolute counter to 0 -> 2M_PI
	this->simThreadsAngleLock();
	int rev = (int)(this->angle[LE]/2/M_PI);
	if (rev) this->angle[LE] -= 2*rev*M_PI;
	rev = (int)(this->angle[RE]/2/M_PI);
	if (rev) this->angle[RE] -= 2*rev*M_PI;
	this->simThreadsAngleUnlock();

	// move to zero position
	this->moveToZero();

	// success
	return 0;
}

/*void CRobot4Sim::updateAngles(void) {
	// must be done for each degree of freedom
	for ( int j = 0; j < NUM_DOF; j++ ) {
		// set motor angle to current angle
		dJointSetAMotorAngle(this->getMotorID(j), 0, this->getAngle(j));
		// drive motor to get current angle to match future angle
		this->updateMotorSpeed(j);
	}
}*/

void CRobot4Sim::update_joint_speed(int i) {
    /*// with PID
    if (this->cur_ang[i] < this->fut_ang[i] - 10*this->m_motor_res)
        dJointSetA*MotorParam(this->motor[i], dParamVel, this->jnt_vel[i]);
    else if (this->cur_ang[i] > this->fut_ang[i] + 10*this->m_motor_res)
        dJointSetAMotorParam(this->motor[i], dParamVel, -this->jnt_vel[i]);
    else if (this->fut_ang[i] - 10*this->m_motor_res < this->cur_ang[i] &&  this->cur_ang[i] < this->fut_ang[i] - this->m_motor_res)
        dJointSetAMotorParam(this->motor[i], dParamVel, this->pid[i].update(this->fut_ang[i] - this->cur_ang[i]));
    else if (this->cur_ang[i] < this->fut_ang[i] + 10*this->m_motor_res && this->cur_ang[i] > this->fut_ang[i] + this->m_motor_res)
        dJointSetAMotorParam(this->motor[i], dParamVel, this->pid[i].update(this->cur_ang[i] - this->fut_ang[i]));
    else
        dJointSetAMotorParam(this->motor[i], dParamVel, 0);*/
    // without PID
    if (this->angle[i] < this->goal[i] - this->m_motor_res)
        dJointSetAMotorParam(this->motor[i], dParamVel, this->velocity[i]);
    else if (this->angle[i] > this->goal[i] + this->m_motor_res)
        dJointSetAMotorParam(this->motor[i], dParamVel, -this->velocity[i]);
    else
        dJointSetAMotorParam(this->motor[i], dParamVel, 0);
}

/*void CRobot4Sim::updateMotorState(int i) {
    if (this->goal[i] - this->m_motor_res < this->angle[i] && this->angle[i] < this->goal[i] + this->m_motor_res) {
        dJointSetAMotorParam(this->motor[i], dParamVel, 0);
		this->state[i] = MOTOR_HOLD;
	}	
}*/

/**********************************************************
	private functions
 **********************************************************/
dReal CRobot4Sim::mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate) {
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

dReal CRobot4Sim::D2R( dReal x ) {
    return x*M_PI/180;
}

dReal CRobot4Sim::R2D( dReal x ) {
    return x/M_PI*180;
}



void CRobot4Sim::create_fixed_joint(CRobot4Sim *attach, int face1, int face2) {
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

    dJointID joint = dJointCreateFixed(this->world, 0);
    dJointAttach(joint, attach->getBodyID(part1), this->getBodyID(part2));
    dJointSetFixed(joint);
    dJointSetFixedParam(joint, dParamCFM, 0);
    dJointSetFixedParam(joint, dParamERP, 0.9);
}

void CRobot4Sim::create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
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

void CRobot4Sim::extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi) {
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

/*void CRobot4Sim::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}*/

CiMobotSim::CiMobotSim(void) {
	this->m_motor_res = D2R(0.5);
	this->m_joint_vel_max[LE] = 6.70;
	this->m_joint_vel_max[LB] = 2.61;
	this->m_joint_vel_max[RB] = 2.61;
	this->m_joint_vel_max[RE] = 6.70;
	this->m_joint_frc_max[LE] = 0.260;
	this->m_joint_frc_max[LB] = 1.059;
	this->m_joint_frc_max[RB] = 1.059;
	this->m_joint_frc_max[RE] = 0.260;
	this->center_length = 0.07303;
	this->center_width = 0.02540;
	this->center_height = 0.06909;
	this->center_radius = 0.03554;
	this->center_offset = 0;
	this->body_length = 0.03785;
	this->body_width = 0.07239;
	this->body_height = 0.07239;
	this->body_radius = 0.03620;
	this->body_inner_width_left = 0.02287;
	this->body_inner_width_right = 0.02287;
	this->body_end_depth = 0.01994;
	this->body_mount_center = 0.03792;
	this->end_width = 0.07239;
	this->end_height = 0.07239;
	this->end_depth = 0.00476;
	this->end_radius = 0.01778;
}

CMobotSim::CMobotSim(void) {
	this->m_motor_res = D2R(0.5);
	this->m_joint_vel_max[LE] = 6.70;
	this->m_joint_vel_max[LB] = 2.61;
	this->m_joint_vel_max[RB] = 2.61;
	this->m_joint_vel_max[RE] = 6.70;
	this->m_joint_frc_max[LE] = 0.260;
	this->m_joint_frc_max[LB] = 1.059;
	this->m_joint_frc_max[RB] = 1.059;
	this->m_joint_frc_max[RE] = 0.260;
	this->center_length = 0.07303;
	this->center_width = 0.02540;
	this->center_height = 0.06909;
	this->center_radius = 0.03554;
	this->center_offset = 0;
	this->body_length = 0.03785;
	this->body_width = 0.07239;
	this->body_height = 0.07239;
	this->body_radius = 0.03620;
	this->body_inner_width_left = 0.02287;
	this->body_inner_width_right = 0.02287;
	this->body_end_depth = 0.01994;
	this->body_mount_center = 0.03792;
	this->end_width = 0.07239;
	this->end_height = 0.07239;
	this->end_depth = 0.00476;
	this->end_radius = 0.01778;
}

void CRobot4Sim::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { this->body[i] = dBodyCreate(this->world); }
    this->geom[ENDCAP_L] = new dGeomID[7];
    this->geom[BODY_L] = new dGeomID[5];
    this->geom[CENTER] = new dGeomID[3];
    this->geom[BODY_R] = new dGeomID[5];
    this->geom[ENDCAP_R] = new dGeomID[7];

	// initialize PID class
	for ( int i = 0; i < NUM_DOF; i++ ) { this->pid[i].init(100, 1, 10, 0.1, 0.004); }

    // adjust input height by body height
    z += this->body_height/2;
    // convert input angles to radians
    psi = D2R(psi);         // roll: x
    theta = D2R(theta);     // pitch: -y
    phi = D2R(phi);         // yaw: z

    // create rotation matrix for robot
    dMatrix3 R;
    this->create_rotation_matrix(R, psi, theta, phi);

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-this->center_length/2 - this->body_length - this->body_end_depth - this->end_depth/2, 0, 0, -this->center_length/2 - this->body_length - this->body_end_depth, 0, 0};
    dReal lb[6] = {-this->center_length/2 - this->body_length - this->body_end_depth/2, 0, 0, -this->center_length/2, this->center_width/2, 0};
	dReal ce[3] = {0, this->center_offset, 0};
    dReal rb[6] = {this->center_length/2 + this->body_length + this->body_end_depth/2, 0, 0, this->center_length/2, this->center_width/2, 0};
    dReal re[6] = {this->center_length/2 + this->body_length + this->body_end_depth + this->end_depth/2, 0, 0, this->center_length/2 + this->body_length + this->body_end_depth, 0, 0};

    // build pieces of module
    this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->build_center(R[0]*ce[0] + x, R[4]*ce[0] + y, R[8]*ce[0] + z, R);
    this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // store position and rotation of center of module
    this->position[0] = x;
    this->position[1] = y;
    this->position[2] = z - this->body_height/2;
    this->rotation[0] = psi;
    this->rotation[1] = theta;
    this->rotation[2] = phi;
    this->orientation[0] = 0;
    this->orientation[1] = 0;
    this->orientation[2] = 0;
    this->orientation[3] = 0;

    // joint for left endcap to body
    this->joint[0] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[0], this->body[BODY_L], this->body[ENDCAP_L]);
    dJointSetHingeAnchor(this->joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(this->joint[0], R[0], R[4], R[8]);
    dJointSetHingeParam(this->joint[0], dParamCFM, 0);

    // joint for center to left body 1
    this->joint[1] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[1], this->body[CENTER], this->body[BODY_L]);
    dJointSetHingeAnchor(this->joint[1], R[0]*lb[3] + R[1]*(this->center_offset+lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(this->center_offset+lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(this->center_offset+lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joint[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joint[1], dParamCFM, 0);

    // joint for center to left body 2
    this->joint[4] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[4], this->body[CENTER], this->body[BODY_L]);
    dJointSetHingeAnchor(this->joint[4], R[0]*lb[3] + R[1]*(this->center_offset-lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(this->center_offset-lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(this->center_offset-lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joint[4], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joint[4], dParamCFM, 0);

    // joint for center to right body 1
    this->joint[2] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[2], this->body[CENTER], this->body[BODY_R]);
    dJointSetHingeAnchor(this->joint[2], R[0]*rb[3] + R[1]*(this->center_offset+rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(this->center_offset+rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(this->center_offset+rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joint[2], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joint[2], dParamCFM, 0);

    // joint for center to right body 2
    this->joint[5] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[5], this->body[CENTER], this->body[BODY_R]);
    dJointSetHingeAnchor(this->joint[5], R[0]*rb[3] + R[1]*(this->center_offset-rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(this->center_offset-rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(this->center_offset-rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joint[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joint[5], dParamCFM, 0);

    // joint for right body to endcap
    this->joint[3] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[3], this->body[BODY_R], this->body[ENDCAP_R]);
    dJointSetHingeAnchor(this->joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(this->joint[3], -R[0], -R[4], -R[8]);
    dJointSetHingeParam(this->joint[3], dParamCFM, 0);

    // motor for left endcap to body
    this->motor[0] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[0], this->body[BODY_L], this->body[ENDCAP_L]);
    dJointSetAMotorMode(this->motor[0], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[0], 1);
    dJointSetAMotorAxis(this->motor[0], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(this->motor[0], 0, 0);
    dJointSetAMotorParam(this->motor[0], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[0], dParamFMax, this->m_joint_frc_max[LE]);

    // motor for center to left body
    this->motor[1] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[1], this->body[CENTER], this->body[BODY_L]);
    dJointSetAMotorMode(this->motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[1], 1);
    dJointSetAMotorAxis(this->motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(this->motor[1], 0, 0);
    dJointSetAMotorParam(this->motor[1], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[1], dParamFMax, this->m_joint_frc_max[LB]);

    // motor for center to right body
    this->motor[2] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[2], this->body[CENTER], this->body[BODY_R]);
    dJointSetAMotorMode(this->motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[2], 1);
    dJointSetAMotorAxis(this->motor[2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(this->motor[2], 0, 0);
    dJointSetAMotorParam(this->motor[2], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[2], dParamFMax, this->m_joint_frc_max[RB]);

    // motor for right body to endcap
    this->motor[3] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[3], this->body[BODY_R], this->body[ENDCAP_R]);
    dJointSetAMotorMode(this->motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[3], 1);
    dJointSetAMotorAxis(this->motor[3], 0, 1, -R[0], -R[4], -R[8]);
    dJointSetAMotorAngle(this->motor[3], 0, 0);
    dJointSetAMotorParam(this->motor[3], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[3], dParamFMax, this->m_joint_frc_max[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->body[i], 0.1, 0.1);
}

void CRobot4Sim::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
	// init body parts
	for ( int i = 0; i < NUM_PARTS; i++ ) { this->body[i] = dBodyCreate(this->world); }
    this->geom[ENDCAP_L] = new dGeomID[7];
    this->geom[BODY_L] = new dGeomID[5];
    this->geom[CENTER] = new dGeomID[3];
    this->geom[BODY_R] = new dGeomID[5];
    this->geom[ENDCAP_R] = new dGeomID[7];

	// initialize PID class
	for ( int i = 0; i < NUM_DOF; i++ ) { this->pid[i].init(100, 1, 10, 0.1, 0.004); }

    // adjust input height by body height
    z += this->body_height/2;
    // convert input angles to radians
    psi = D2R(psi);         // roll: x
    theta = D2R(theta);     // pitch: -y
    phi = D2R(phi);         // yaw: z
    r_le = D2R(r_le);       // left end
    r_lb = D2R(r_lb);       // left body
    r_rb = D2R(r_rb);       // right body
    r_re = D2R(r_re);       // right end

    // create rotation matrix for robot
    dMatrix3 R;
    this->create_rotation_matrix(R, psi, theta, phi);

    // store initial body angles into array
    /*this->cur_ang[LE] = r_le;
    this->cur_ang[LB] = r_lb;
    this->cur_ang[RB] = r_rb;
    this->cur_ang[RE] = r_re;
    this->fut_ang[LE] += r_le;
    this->fut_ang[RE] += r_re;*/
    this->angle[LE] = r_le;
    this->angle[LB] = r_lb;
    this->angle[RB] = r_rb;
    this->angle[RE] = r_re;

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-this->center_length/2 - this->body_length - this->body_end_depth - this->end_depth/2, 0, 0, -this->center_length/2 - this->body_length - this->body_end_depth, 0, 0};
    dReal lb[6] = {-this->center_length/2 - this->body_length - this->body_end_depth/2, 0, 0, -this->center_length/2, this->center_width/2, 0};
	dReal ce[3] = {0, this->center_offset, 0};
    dReal rb[6] = {this->center_length/2 + this->body_length + this->body_end_depth/2, 0, 0, this->center_length/2, this->center_width/2, 0};
    dReal re[6] = {this->center_length/2 + this->body_length + this->body_end_depth + this->end_depth/2, 0, 0, this->center_length/2 + this->body_length + this->body_end_depth, 0, 0};

    this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->build_center(R[0]*ce[0] + x, R[4]*ce[0] + y, R[8]*ce[0] + z, R);
    this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // store position and rotation of center of module
    this->position[0] = x;
    this->position[1] = y;
    this->position[2] = z - this->body_height/2;
    this->rotation[0] = psi;
    this->rotation[1] = theta;
    this->rotation[2] = phi;
    this->orientation[0] = r_le;
    this->orientation[1] = r_lb;
    this->orientation[2] = r_rb;
    this->orientation[3] = r_re;

    // joint for left endcap to body
    this->joint[0] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[0], this->body[BODY_L], this->body[ENDCAP_L]);
    dJointSetHingeAnchor(this->joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(this->joint[0], R[0], R[4], R[8]);
    dJointSetHingeParam(this->joint[0], dParamCFM, 0);

    // joint for center to left body 1
    this->joint[1] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[1], this->body[CENTER], this->body[BODY_L]);
    dJointSetHingeAnchor(this->joint[1], R[0]*lb[3] + R[1]*(this->center_offset+lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(this->center_offset+lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(this->center_offset+lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joint[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joint[1], dParamCFM, 0);

    // joint for center to left body 2
    this->joint[4] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[4], this->body[CENTER], this->body[BODY_L]);
    dJointSetHingeAnchor(this->joint[4], R[0]*lb[3] + R[1]*(this->center_offset-lb[4]) + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*(this->center_offset-lb[4]) + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*(this->center_offset-lb[4]) + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joint[4], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joint[4], dParamCFM, 0);

    // joint for center to right body 1
    this->joint[2] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[2], this->body[CENTER], this->body[BODY_R]);
    dJointSetHingeAnchor(this->joint[2], R[0]*rb[3] + R[1]*(this->center_offset+rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(this->center_offset+rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(this->center_offset+rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joint[2], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joint[2], dParamCFM, 0);

    // joint for center to right body 2
    this->joint[5] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[5], this->body[CENTER], this->body[BODY_R]);
    dJointSetHingeAnchor(this->joint[5], R[0]*rb[3] + R[1]*(this->center_offset-rb[4]) + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*(this->center_offset-rb[4]) + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*(this->center_offset-rb[4]) + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joint[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joint[5], dParamCFM, 0);

    // joint for right body to endcap
    this->joint[3] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joint[3], this->body[BODY_R], this->body[ENDCAP_R]);
    dJointSetHingeAnchor(this->joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(this->joint[3], -R[0], -R[4], -R[8]);
    dJointSetHingeParam(this->joint[3], dParamCFM, 0);

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
    dReal le_r[3] = {-this->center_length/2 - (this->body_length + this->body_end_depth + this->end_depth/2)*cos(r_lb), 0, (this->body_length + this->body_end_depth + this->end_depth/2)*sin(r_lb)};
    dReal lb_r[3] = {-this->center_length/2 - (this->body_length + this->body_end_depth/2)*cos(r_lb), 0, (this->body_length + this->body_end_depth/2)*sin(r_lb)};
    dReal rb_r[3] = {this->center_length/2 + (this->body_length + this->body_end_depth/2)*cos(r_rb), 0, (this->body_length + this->body_end_depth/2)*sin(r_rb)};
    dReal re_r[3] = {this->center_length/2 + (this->body_length + this->body_end_depth + this->end_depth/2)*cos(r_rb), 0, (this->body_length + this->body_end_depth + this->end_depth/2)*sin(r_rb)};

    // re-build pieces of module
    this->build_endcap(ENDCAP_L, R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
    this->build_body(BODY_L, R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, r_lb);
    this->build_body(BODY_R, R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, r_rb);
    this->build_endcap(ENDCAP_R, R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);

    // motor for left endcap to body
    this->motor[0] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[0], this->body[BODY_L], this->body[ENDCAP_L]);
    dJointSetAMotorMode(this->motor[0], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[0], 1);
    dJointSetAMotorAxis(this->motor[0], 0, 1, R_lb[0], R_lb[4], R_lb[8]);
    dJointSetAMotorAngle(this->motor[0], 0, 0);
    dJointSetAMotorParam(this->motor[0], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[0], dParamFMax, this->m_joint_frc_max[LE]);

    // motor for center to left body
    this->motor[1] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[1], this->body[CENTER], this->body[BODY_L]);
    dJointSetAMotorMode(this->motor[1], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[1], 1);
    dJointSetAMotorAxis(this->motor[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(this->motor[1], 0, 0);
    dJointSetAMotorParam(this->motor[1], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[1], dParamFMax, this->m_joint_frc_max[LB]);

    // motor for center to right body
    this->motor[2] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[2], this->body[CENTER], this->body[BODY_R]);
    dJointSetAMotorMode(this->motor[2], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[2], 1);
    dJointSetAMotorAxis(this->motor[2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(this->motor[2], 0, 0);
    dJointSetAMotorParam(this->motor[2], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[2], dParamFMax, this->m_joint_frc_max[RB]);

    // motor for right body to endcap
    this->motor[3] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motor[3], this->body[BODY_R], this->body[ENDCAP_R]);
    dJointSetAMotorMode(this->motor[3], dAMotorUser);
    dJointSetAMotorNumAxes(this->motor[3], 1);
    dJointSetAMotorAxis(this->motor[3], 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
    dJointSetAMotorAngle(this->motor[3], 0, 0);
    dJointSetAMotorParam(this->motor[3], dParamCFM, 0);
    dJointSetAMotorParam(this->motor[3], dParamFMax, this->m_joint_frc_max[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->body[i], 0.1, 0.1);
}

void CRobot4Sim::buildAttached00(CRobot4Sim *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3] = {0};
    dMatrix3 R, R1, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

    if ( face1 == 1 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - 2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length;
        m[1] = 0;
    }
    else if ( face1 == 1 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width;
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length;
    }
    else if ( face1 == 1 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width;
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length;
    }
    else if ( face1 == 1 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width;
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length;
    }
    else if ( face1 == 1 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width;
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length;
    }
    else if ( face1 == 1 && face2 == 6 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - 2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length;
        m[1] = 0;
    }
    else if ( face1 == 2 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center;
        m[1] = -this->end_depth - 0.5*this->body_width - this->body_end_depth - this->body_length - 0.5*this->center_length;
    }
    else if ( face1 == 2 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) - 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 2 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 2 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 2 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) - 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 2 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center;
        m[1] = -this->end_depth - 0.5*this->body_width - this->body_end_depth - this->body_length - 0.5*this->center_length;
    }
    else if ( face1 == 3 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center;
        m[1] = this->end_depth + 0.5*this->body_width + this->body_end_depth + this->body_length + 0.5*this->center_length;
    }
    else if ( face1 == 3 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 3 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) - 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 3 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) - 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 3 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length + 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 3 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center;
        m[1] = this->end_depth + 0.5*this->body_width + this->body_end_depth + this->body_length + 0.5*this->center_length;
    }
    else if ( face1 == 4 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center;
        m[1] = -this->end_depth - 0.5*this->body_width - this->body_end_depth - this->body_length - 0.5*this->center_length;
    }
    else if ( face1 == 4 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length - 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 4 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 4 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length +  2*(this->body_length + this->body_end_depth - this->body_mount_center) + 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 4 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length - 0.5*this->center_length;
        m[1] = -this->body_width;
    }
    else if ( face1 == 4 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center;
        m[1] = -this->end_depth - 0.5*this->body_width - this->body_end_depth - this->body_length - 0.5*this->center_length;
    }
    else if ( face1 == 5 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center;
        m[1] = this->end_depth + 0.5*this->body_width + this->body_end_depth + this->body_length + 0.5*this->center_length;
    }
    else if ( face1 == 5 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 5 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length - 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 5 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length - 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 5 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length    +   2*(this->body_length + this->body_end_depth - this->body_mount_center) + 0.5*this->center_length;
        m[1] = this->body_width;
    }
    else if ( face1 == 5 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center;
        m[1] = this->end_depth + 0.5*this->body_width + this->body_end_depth + this->body_length + 0.5*this->center_length;
    }
    else if ( face1 == 6 && face2 == 1 ) {
		printf("6161616161\n");
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + 2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length;
        m[1] = 0;
    }
    else if ( face1 == 6 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width;
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length;
    }
    else if ( face1 == 6 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length +  this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width;
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length;
    }
    else if ( face1 == 6 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width;
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length;
    }
    else if ( face1 == 6 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width;
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length;
    }
    else if ( face1 == 6 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + 2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length;
        m[1] = 0;
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                R2D(psi), R2D(theta), R2D(phi));

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4Sim::buildAttached10(CRobot4Sim *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth) + R3[0]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth) + R3[4]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth) + R3[8]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth) + R3[0]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth) + R3[4]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth) + R3[8]*(-2*this->end_depth - this->body_end_depth - this->body_length - 0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                               + R1[1]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->end_depth - 0.5*this->body_width  + R1[5]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                               + R1[9]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length                   + R1[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                                      + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length                   + R1[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                                      + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[1]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[9]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[1]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[9]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length               + R1[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(0.5*this->center_length);
        m[2] =                                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length               + R1[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(0.5*this->center_length);
        m[2] =                                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getAngle(LB));
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[1]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R1[9]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[1]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) -  this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[9]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      -this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length                    + R1[0]*(-0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                                      + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[1]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) -  this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[9]*(-this->body_end_depth - this->body_length - 0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[1]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) +  this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[9]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  + this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  + this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getAngle(RB));
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[1]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) +  this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R1[9]*(this->body_end_depth + this->body_length + 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth) + R3[0]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth) + R3[4]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth) + R3[8]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center + 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center - 0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth) + R3[0]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth) + R3[4]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth) + R3[8]*(2*this->end_depth + this->body_end_depth + this->body_length + 0.5*this->center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                R2D(psi), R2D(theta), R2D(phi));

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4Sim::buildAttached01(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    // initialize variables
    dReal psi, theta, phi, r_e, r_b, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

    // rotation of body about fixed point
    if ( face2 == 1 ) {
        r_e = D2R(r_le);
        r_b = D2R(r_lb);
    }
    else if ( face2 == 2 ) {
        r_e = 0;
        r_b = D2R(r_lb);
    }
    else if ( face2 == 3 ) {
        r_e = 0;
        r_b = D2R(r_lb);
    }
    else if ( face2 == 4 ) {
        r_e = 0;
        r_b = D2R(r_rb);
    }
    else if ( face2 == 5 ) {
        r_e = 0;
        r_b = D2R(r_rb);
    }
    else if ( face2 == 6 ) {
        r_e = D2R(r_re);
        r_b = D2R(r_rb);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - 2*this->end_depth + R1[0]*(-this->body_end_depth - this->body_length) + R3[0]*(-0.5*this->center_length);
        m[1] =                                                                   R1[4]*(-this->body_end_depth - this->body_length) + R3[4]*(-0.5*this->center_length);
        m[2] =                                                                   R1[8]*(-this->body_end_depth - this->body_length) + R3[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width + R1[1]*(0.5*this->center_length);
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + R1[5]*(0.5*this->center_length);
        m[2] = R1[9]*(0.5*this->center_length);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width + R1[1]*(-0.5*this->center_length);
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center + R1[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-0.5*this->center_length);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width + R1[1]*(-0.5*this->center_length);
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center + R1[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-0.5*this->center_length);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width + R1[1]*(0.5*this->center_length);
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + R1[5]*(0.5*this->center_length);
        m[2] = R1[9]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth - 2*this->end_depth + R1[0]*(-this->body_end_depth - this->body_length) + R3[0]*(-0.5*this->center_length);
        m[1] = R1[4]*(-this->body_end_depth - this->body_length) + R3[4]*(-0.5*this->center_length);
        m[2] = R1[8]*(-this->body_end_depth - this->body_length) + R3[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center + R1[1]*(-this->body_end_depth - this->body_length) + R3[1]*(-0.5*this->center_length);
        m[1] = -this->end_depth - 0.5*this->body_width  + R1[5]*(-this->body_end_depth - this->body_length) + R3[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-this->body_end_depth - this->body_length) + R3[9]*(-0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) + R1[0]*(-0.5*this->center_length);
        m[1] = -this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] = R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*this->center_length                   + R1[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                                      + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*this->center_length                   + R1[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(0.5*this->center_length);
        m[2] =                                      + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) + R1[0]*(-0.5*this->center_length);
        m[1] = -this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] = R1[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center + R1[1]*(-this->body_end_depth - this->body_length) + R3[1]*(-0.5*this->center_length);
        m[1] = -this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length) + R5[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-this->body_end_depth - this->body_length) + R5[9]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center + R1[1]*(this->body_end_depth + this->body_length) + R3[1]*(0.5*this->center_length);
        m[1] = this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length) + R2[5]*(0.5*this->center_length);
        m[2] = R1[9]*(this->body_end_depth + this->body_length) + R3[9]*(0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*this->center_length               + R1[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(0.5*this->center_length);
        m[2] =                                  + R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center + R1[0]*(-0.5*this->center_length);
        m[1] = this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] = R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*this->center_length + 2*(-this->body_length - this->body_end_depth + this->body_mount_center) + R1[0]*(-0.5*this->center_length);
        m[1] = this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] = R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*this->center_length               + R1[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(0.5*this->center_length);
        m[2] =                                  + R1[8]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length - this->body_length - this->body_end_depth + this->body_mount_center + R1[1]*(this->body_end_depth + this->body_length) + R3[1]*(0.5*this->center_length);
        m[1] = this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length) + R3[5]*(0.5*this->center_length);
        m[2] = R1[9]*(this->body_end_depth + this->body_length) + R3[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center + R1[1]*(-this->body_end_depth - this->body_length) + R3[1]*(-0.5*this->center_length);
        m[1] = -this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length) + R3[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-this->body_end_depth - this->body_length) + R3[9]*(-0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      -this->body_width + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + R1[0]*(0.5*this->center_length);
        m[1] = -this->body_width + R1[4]*(0.5*this->center_length);
        m[2] = R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + R1[0]*(0.5*this->center_length);
        m[1] = -this->body_width + R1[4]*(0.5*this->center_length);
        m[2] = R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*this->center_length                    + R1[0]*(-0.5*this->center_length);
        m[1] =                      - this->body_width    + R1[4]*(-0.5*this->center_length);
        m[2] =                                      + R1[8]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center + R1[1]*(-this->body_end_depth - this->body_length) + R3[1]*(-0.5*this->center_length);
        m[1] = -this->end_depth - 0.5*this->body_width + R1[5]*(-this->body_end_depth - this->body_length) + R3[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-this->body_end_depth - this->body_length) + R3[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center + R1[1]*(this->body_end_depth + this->body_length) + R3[1]*(0.5*this->center_length);
        m[1] = this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length) + R3[5]*(0.5*this->center_length);
        m[2] = R1[9]*(this->body_end_depth + this->body_length) + R3[9]*(0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + R1[0]*(0.5*this->center_length);
        m[1] = this->body_width + R1[4]*(0.5*this->center_length);
        m[2] = R1[8]*(0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*this->center_length                + R1[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R1[4]*(-0.5*this->center_length);
        m[2] =                                  + R1[8]*(-0.5*this->center_length);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*this->center_length + 2*(this->body_length + this->body_end_depth - this->body_mount_center) + R1[0]*(0.5*this->center_length);
        m[1] = this->body_width + R1[4]*(0.5*this->center_length);
        m[2] = R1[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth - this->body_mount_center + R1[1]*(this->body_end_depth + this->body_length) + R3[1]*(0.5*this->center_length);
        m[1] = this->end_depth + 0.5*this->body_width + R1[5]*(this->body_end_depth + this->body_length) + R3[5]*(0.5*this->center_length);
        m[2] = R1[9]*(this->body_end_depth + this->body_length) + R3[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + 2*this->end_depth + R1[0]*(this->body_end_depth + this->body_length) + R3[0]*(0.5*this->center_length);
        m[1] = R1[4]*(this->body_end_depth + this->body_length) + R3[4]*(0.5*this->center_length);
        m[2] = R1[8]*(this->body_end_depth + this->body_length) + R3[8]*(0.5*this->center_length);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width + R1[1]*(-0.5*this->center_length);
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center + R1[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-0.5*this->center_length);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width + R1[1]*(0.5*this->center_length);
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + R1[5]*(0.5*this->center_length);
        m[2] = R1[9]*(0.5*this->center_length);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width + R1[1]*(0.5*this->center_length);
        m[1] = this->body_end_depth + this->body_length - this->body_mount_center + R1[5]*(0.5*this->center_length);
        m[2] = R1[9]*(0.5*this->center_length);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width + R1[1]*(-0.5*this->center_length);
        m[1] = -this->body_end_depth - this->body_length + this->body_mount_center + R1[5]*(-0.5*this->center_length);
        m[2] = R1[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length + this->body_length + this->body_end_depth + 2*this->end_depth + R1[0]*(this->body_end_depth + this->body_length) + R3[0]*(0.5*this->center_length);
        m[1] = R1[4]*(this->body_end_depth + this->body_length) + R3[4]*(0.5*this->center_length);
        m[2] = R1[8]*(this->body_end_depth + this->body_length) + R3[8]*(0.5*this->center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4Sim::buildAttached11(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    // initialize variables
    dReal psi, theta, phi, r_e, r_b, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R6, R7, R8, R9, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

    // rotation of body about fixed point
    if ( face2 == 1 ) {
        r_e = D2R(r_le);
        r_b = D2R(r_lb);
    }
    else if ( face2 == 2 ) {
        r_e = 0;
        r_b = D2R(r_lb);
    }
    else if ( face2 == 3 ) {
        r_e = 0;
        r_b = D2R(r_lb);
    }
    else if ( face2 == 4 ) {
        r_e = 0;
        r_b = D2R(r_rb);
    }
    else if ( face2 == 5 ) {
        r_e = 0;
        r_b = D2R(r_rb);
    }
    else if ( face2 == 6 ) {
        r_e = D2R(r_re);
        r_b = D2R(r_rb);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth) + R3[0]*(-2*this->end_depth) + R5[0]*(-this->body_end_depth - this->body_length) + R7[0]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth) + R3[4]*(-2*this->end_depth) + R5[4]*(-this->body_end_depth - this->body_length) + R7[4]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth) + R3[8]*(-2*this->end_depth) + R5[8]*(-this->body_end_depth - this->body_length) + R7[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[9]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[9]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[9]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth - this->end_depth - 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[9]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth) + R3[0]*(-2*this->end_depth) + R5[0]*(-this->body_end_depth - this->body_length) + R7[0]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth) + R3[4]*(-2*this->end_depth) + R5[4]*(-this->body_end_depth - this->body_length) + R7[4]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth) + R3[8]*(-2*this->end_depth) + R5[8]*(-this->body_end_depth - this->body_length) + R7[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                               + R3[1]*(-this->body_end_depth - this->body_length) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->end_depth - 0.5*this->body_width  + R3[5]*(-this->body_end_depth - this->body_length) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                               + R3[9]*(-this->body_end_depth - this->body_length) + R5[9]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->body_width    + R3[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length                   + R3[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                                      + R3[8]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length                   + R3[0]*(0.5*this->center_length);
        m[1] =                      - this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                                      + R3[8]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->body_width    + R3[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[1]*(-this->body_end_depth - this->body_length) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) - this->end_depth - 0.5*this->body_width + R3[5]*(-this->body_end_depth - this->body_length) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[9]*(-this->body_end_depth - this->body_length) + R5[9]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[1]*(this->body_end_depth + this->body_length) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->end_depth + 0.5*this->body_width + R3[5]*(this->body_end_depth + this->body_length) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[9]*(this->body_end_depth + this->body_length) + R5[9]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length               + R3[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R3[4]*(0.5*this->center_length);
        m[2] =                                  + R3[8]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->body_width    + R3[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length   +   2*R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[0]*(-0.5*this->center_length);
        m[1] =                          2*R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->body_width    + R3[4]*(-0.5*this->center_length);
        m[2] =                          2*R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center)                 + R3[8]*(-0.5*this->center_length);
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
        m[0] = -0.5*this->center_length               + R3[0]*(0.5*this->center_length);
        m[1] =                      this->body_width  + R3[4]*(0.5*this->center_length);
        m[2] =                                  + R3[8]*(0.5*this->center_length);
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
        m[0] = -0.5*this->center_length + R1[0]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[1]*(this->body_end_depth + this->body_length) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(-this->body_length - this->body_end_depth + this->body_mount_center) + this->end_depth + 0.5*this->body_width + R3[5]*(this->body_end_depth + this->body_length) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(-this->body_length - this->body_end_depth + this->body_mount_center) +                              R3[9]*(this->body_end_depth + this->body_length) + R5[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[1]*(-this->body_end_depth - this->body_length) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) -  this->end_depth - 0.5*this->body_width + R3[5]*(-this->body_end_depth - this->body_length) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[9]*(-this->body_end_depth - this->body_length) + R5[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length                + R3[0]*(-0.5*this->center_length);
        m[1] =                      -this->body_width + R3[4]*(-0.5*this->center_length);
        m[2] =                                  + R3[8]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  - this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  - this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length                    + R3[0]*(-0.5*this->center_length);
        m[1] =                      - this->body_width    + R3[4]*(-0.5*this->center_length);
        m[2] =                                      + R3[8]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[1]*(-this->body_end_depth - this->body_length) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) -  this->end_depth - 0.5*this->body_width + R3[5]*(-this->body_end_depth - this->body_length) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[9]*(-this->body_end_depth - this->body_length) + R5[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[1]*(this->body_end_depth + this->body_length) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) +  this->end_depth + 0.5*this->body_width + R3[5]*(this->body_end_depth + this->body_length) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[9]*(this->body_end_depth + this->body_length) + R5[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  + this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length                + R3[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R3[4]*(-0.5*this->center_length);
        m[2] =                                  + R3[8]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length                + R3[0]*(-0.5*this->center_length);
        m[1] =                      this->body_width  + R3[4]*(-0.5*this->center_length);
        m[2] =                                  + R3[8]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length    +   2*R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[0]*(0.5*this->center_length);
        m[1] =                          2*R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center)  + this->body_width    + R3[4]*(0.5*this->center_length);
        m[2] =                          2*R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center)                  + R3[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[1]*(this->body_end_depth + this->body_length) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth - this->body_mount_center) +  this->end_depth + 0.5*this->body_width + R3[5]*(this->body_end_depth + this->body_length) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth - this->body_mount_center) +                               R3[9]*(this->body_end_depth + this->body_length) + R5[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth) + R3[0]*(2*this->end_depth) + R5[0]*(this->body_end_depth + this->body_length) + R7[0]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth) + R3[4]*(2*this->end_depth) + R5[4]*(this->body_end_depth + this->body_length) + R7[4]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth) + R3[8]*(2*this->end_depth) + R5[8]*(this->body_end_depth + this->body_length) + R7[8]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[1]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[5]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(this->body_end_depth + this->body_length - this->body_mount_center) + R5[9]*(0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[1]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[1]*(-0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[5]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[5]*(-0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth + this->end_depth + 0.5*this->body_width) + R3[9]*(-this->body_end_depth - this->body_length + this->body_mount_center) + R5[9]*(-0.5*this->center_length);
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
        m[0] = 0.5*this->center_length +  R1[0]*(this->body_length + this->body_end_depth) + R3[0]*(2*this->end_depth) + R5[0]*(this->body_end_depth + this->body_length) + R7[0]*(0.5*this->center_length);
        m[1] =                      R1[4]*(this->body_length + this->body_end_depth) + R3[4]*(2*this->end_depth) + R5[4]*(this->body_end_depth + this->body_length) + R7[4]*(0.5*this->center_length);
        m[2] =                      R1[8]*(this->body_length + this->body_end_depth) + R3[8]*(2*this->end_depth) + R5[8]*(this->body_end_depth + this->body_length) + R7[8]*(0.5*this->center_length);
    }

    // extract euler angles from rotation matrix
    this->extract_euler_angles(R, psi, theta, phi);

    // build new module
    this->build(attach->getPosition(0) + R_att[0]*m[0] + R_att[1]*m[1] + R_att[2]*m[2],
                attach->getPosition(1) + R_att[4]*m[0] + R_att[5]*m[1] + R_att[6]*m[2],
                attach->getPosition(2) + R_att[8]*m[0] + R_att[9]*m[1] + R_att[10]*m[2],
                R2D(psi), R2D(theta), R2D(phi), r_le, r_lb, r_rb, r_re);

    // add fixed joint to attach two modules
    this->create_fixed_joint(attach, face1, face2);
}

void CRobot4Sim::build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta) {
	int i = 1;
	if ( id == BODY_R )
		i = -1;

    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetZero(&m);
    // create mass 1
    dMassSetBox(&m1, 2700, this->body_end_depth, this->center_height, this->body_width );
    dMassAdd(&m, &m1);
    // create mass 2
    dMassSetBox(&m2, 2700, this->body_inner_width_left, this->end_depth, this->body_width );
    dMassTranslate(&m2, 0.01524*i, -0.0346, 0 );
    dMassAdd(&m, &m2);
    // create mass 3
    dMassSetBox(&m3, 2700, this->body_inner_width_right, this->end_depth, this->body_width );
    dMassTranslate(&m3, 0.01524*i, 0.0346, 0 );
    dMassAdd(&m, &m3);
    //dMassSetParameters( &m, 500, 1, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(this->body[id], x, y, z);
    dBodySetRotation(this->body[id], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - face
    this->geom[id][0] = dCreateBox(this->space, this->body_end_depth, this->body_width, this->body_height);
    dGeomSetBody(this->geom[id][0], this->body[id]);
    dGeomSetOffsetPosition(this->geom[id][0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - side square
    this->geom[id][1] = dCreateBox( this->space, this->body_length, this->body_inner_width_left, this->body_height);
    dGeomSetBody( this->geom[id][1], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][1], i*this->body_length/2 + i*this->body_end_depth/2 - m.c[0], -this->body_radius + this->body_inner_width_left/2 - m.c[1], -m.c[2] );

    // set geometry 3 - side square
    this->geom[id][2] = dCreateBox( this->space, this->body_length, this->body_inner_width_right, this->body_height);
    dGeomSetBody( this->geom[id][2], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][2], i*this->body_length/2 + i*this->body_end_depth/2 - m.c[0], this->body_radius - this->body_inner_width_right/2 - m.c[1], -m.c[2] );

    // set geometry 4 - side curve
    this->geom[id][3] = dCreateCylinder( this->space, this->body_radius, this->body_inner_width_left);
    dGeomSetBody( this->geom[id][3], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][3], i*this->body_length + i*this->body_end_depth/2 - m.c[0], -this->body_radius + this->body_inner_width_left/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][3], R2);

    // set geometry 5 - side curve
    this->geom[id][4] = dCreateCylinder( this->space, this->body_radius, this->body_inner_width_right);
    dGeomSetBody( this->geom[id][4], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][4], i*this->body_length + i*this->body_end_depth/2 - m.c[0], this->body_radius - this->body_inner_width_right/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][4], R2);

    // set mass center to (0,0,0) of this->bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->body[id], &m);
}

void CRobot4Sim::build_center(dReal x, dReal y, dReal z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetZero(&m);
    dMassSetCapsule(&m, 2700, 1, this->center_radius, this->center_length );
    dMassAdjust(&m, 0.24);
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(this->body[CENTER], x, y, z);
    dBodySetRotation(this->body[CENTER], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);

    // set geometry 1 - center rectangle
    this->geom[CENTER][0] = dCreateBox(this->space, this->center_length, this->center_width, this->center_height );
    dGeomSetBody( this->geom[CENTER][0], this->body[CENTER]);
    dGeomSetOffsetPosition( this->geom[CENTER][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - side curve
    this->geom[CENTER][1] = dCreateCylinder(this->space, this->center_radius, this->center_width );
    dGeomSetBody( this->geom[CENTER][1], this->body[CENTER]);
    dGeomSetOffsetPosition( this->geom[CENTER][1], -this->center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geom[CENTER][1], R1);

    // set geometry 3 - side curve
    this->geom[CENTER][2] = dCreateCylinder(this->space, this->center_radius, this->center_width );
    dGeomSetBody( this->geom[CENTER][2], this->body[CENTER]);
    dGeomSetOffsetPosition( this->geom[CENTER][2], this->center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( this->geom[CENTER][2], R1);

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->body[CENTER], &m);
}

void CRobot4Sim::build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 2700, this->end_depth, this->end_width, this->end_height );
    //dMassSetParameters( &m, 500, 0.45, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0);

    // adjust x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(this->body[id], x, y, z);
    dBodySetRotation(this->body[id], R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    this->geom[id][0] = dCreateBox(this->space, this->end_depth, this->end_width - 2*this->end_radius, this->end_height );
    dGeomSetBody( this->geom[id][0], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - left box
    this->geom[id][1] = dCreateBox(this->space, this->end_depth, this->end_radius, this->end_height - 2*this->end_radius );
    dGeomSetBody( this->geom[id][1], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][1], -m.c[0], -this->end_width/2 + this->end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 3 - right box
    this->geom[id][2] = dCreateBox(this->space, this->end_depth, this->end_radius, this->end_height - 2*this->end_radius );
    dGeomSetBody( this->geom[id][2], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][2], -m.c[0], this->end_width/2 - this->end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 4 - fillet upper left
    this->geom[id][3] = dCreateCylinder(this->space, this->end_radius, this->end_depth );
    dGeomSetBody( this->geom[id][3], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][3], -m.c[0], -this->end_width/2 + this->end_radius - m.c[1], this->end_width/2 - this->end_radius - m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][3], R1);

    // set geometry 5 - fillet upper right
    this->geom[id][4] = dCreateCylinder(this->space, this->end_radius, this->end_depth );
    dGeomSetBody( this->geom[id][4], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][4], -m.c[0], this->end_width/2 - this->end_radius - m.c[1], this->end_width/2 - this->end_radius - m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][4], R1);

    // set geometry 6 - fillet lower right
    this->geom[id][5] = dCreateCylinder(this->space, this->end_radius, this->end_depth );
    dGeomSetBody( this->geom[id][5], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][5], -m.c[0], this->end_width/2 - this->end_radius - m.c[1], -this->end_width/2 + this->end_radius - m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][5], R1);

    // set geometry 7 - fillet lower left
    this->geom[id][6] = dCreateCylinder(this->space, this->end_radius, this->end_depth );
    dGeomSetBody( this->geom[id][6], this->body[id]);
    dGeomSetOffsetPosition( this->geom[id][6], -m.c[0], -this->end_width/2 + this->end_radius - m.c[1], -this->end_width/2 + this->end_radius - m.c[2] );
    dGeomSetOffsetRotation( this->geom[id][6], R1);

    // set mass center to (0,0,0) of this->bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(this->body[id], &m);
}
