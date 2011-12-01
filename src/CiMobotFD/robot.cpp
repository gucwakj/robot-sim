#include "robot.h"
using namespace std;

Robot::Robot(dWorldID &world, dSpaceID &space, int num_stp) {
    int j;

    this->world = world;
    this->space = dHashSpaceCreate(space);
    this->m_num_stp = num_stp;

    this->m_motor_res = D2R(0.5);
    this->m_joint_vel_max = new dReal[NUM_DOF];
    this->m_joint_vel_max[LE] = 6.70;
    this->m_joint_vel_max[LB] = 2.61;
    this->m_joint_vel_max[RB] = 2.61;
    this->m_joint_vel_max[RE] = 6.70;
    this->m_joint_vel_min = new dReal[NUM_DOF];
    this->m_joint_vel_min[LE] = 3.22;
    this->m_joint_vel_min[LB] = 1.25;
    this->m_joint_vel_min[RB] = 1.25;
    this->m_joint_vel_min[RE] = 3.22;
    this->m_joint_frc_max = new dReal[NUM_DOF];
    this->m_joint_frc_max[LE] = 0.260;
    this->m_joint_frc_max[LB] = 1.059;
    this->m_joint_frc_max[RB] = 1.059;
    this->m_joint_frc_max[RE] = 0.260;

    this->body = new Body * [NUM_PARTS];
    this->body[ENDCAP_L] = new Body(this->world, this->space, 7);
    this->body[BODY_L] = new Body(this->world, this->space, 5);
    this->body[CENTER] = new Body(this->world, this->space, 3);
    this->body[BODY_R] = new Body(this->world, this->space, 5);
    this->body[ENDCAP_R] = new Body(this->world, this->space, 7);
    this->joints = new dJointID[6];
    this->motors = new dJointID[4];
    this->pid = new PID[NUM_DOF];
    this->ang = new dReal[NUM_DOF*num_stp]();
    this->vel = new dReal[NUM_DOF*num_stp]();
    this->cur_ang = new dReal[NUM_DOF]();
    this->fut_ang = new dReal[NUM_DOF]();
    this->jnt_vel = new dReal[NUM_DOF]();
    this->pos = new dReal[3]();
    this->rot = new dReal[3]();
    this->ori = new dReal[4]();

    for ( j = 0; j < NUM_DOF*num_stp; j++ ) {
        this->vel[j] = this->m_joint_vel_max[j%NUM_DOF];
    }
    for ( j = 0; j < NUM_DOF; j++ ) {
        this->pid[j].init(100, 1, 10, 0.1, 0.004);
    }

    #ifdef ENABLE_DRAWSTUFF
    for ( int j = 0; j < NUM_DOF; j++ ) { this->jnt_vel[j] = this->vel[j]; }
    #endif
}

Robot::~Robot(void) {
    for ( int j = 0; j < NUM_PARTS; j++ ) delete [] this->body[j]->geomID;
    for ( int i = NUM_PARTS - 1; i >= 0; i-- ) { delete this->body[i]; }
    delete [] this->body;
    delete [] this->pid;
    delete [] this->joints;
    delete [] this->motors;
    delete [] this->fut_ang;
    delete [] this->cur_ang;
    delete [] this->jnt_vel;
    delete [] this->ang;
    delete [] this->vel;
    delete [] this->pos;
    delete [] this->rot;
    delete [] this->ori;
    dSpaceDestroy(this->space);
}

void Robot::setAngles(dReal *ang) {
    for ( int j = 0; j < NUM_DOF*this->m_num_stp; j++ ) {
        this->ang[j] = D2R(ang[j]);
    }
    for ( int j = 0; j < NUM_DOF; j++ ) {
        this->fut_ang[j] = this->ang[j];
    }
}

void Robot::setAngularVelocity(dReal *vel) {
    for ( int j = 0; j < NUM_DOF*this->m_num_stp; j++ ) {
        this->vel[j] = this->m_joint_vel_min[j%NUM_DOF] + vel[j]*(this->m_joint_vel_max[j%NUM_DOF] - this->m_joint_vel_min[j%NUM_DOF]);
    }
    for ( int j = 0; j < NUM_DOF; j++ ) {
        this->jnt_vel[j] = this->vel[j];
    }
}

void Robot::setMotorSpeed(int j) {
    /*// with PID
    if (this->cur_ang[j] < this->fut_ang[j] - 10*this->m_motor_res)
        dJointSetA*MotorParam(this->motors[j], dParamVel, this->jnt_vel[j]);
    else if (this->cur_ang[j] > this->fut_ang[j] + 10*this->m_motor_res)
        dJointSetAMotorParam(this->motors[j], dParamVel, -this->jnt_vel[j]);
    else if (this->fut_ang[j] - 10*this->m_motor_res < this->cur_ang[j] &&  this->cur_ang[j] < this->fut_ang[j] - this->m_motor_res)
        dJointSetAMotorParam(this->motors[j], dParamVel, this->pid[j].update(this->fut_ang[j] - this->cur_ang[j]));
    else if (this->cur_ang[j] < this->fut_ang[j] + 10*this->m_motor_res && this->cur_ang[j] > this->fut_ang[j] + this->m_motor_res)
        dJointSetAMotorParam(this->motors[j], dParamVel, this->pid[j].update(this->cur_ang[j] - this->fut_ang[j]));
    else
        dJointSetAMotorParam(this->motors[j], dParamVel, 0);*/
    // without PID
    if (this->cur_ang[j] < this->fut_ang[j] - this->m_motor_res)
        dJointSetAMotorParam(this->motors[j], dParamVel, this->jnt_vel[j]);
    else if (this->cur_ang[j] > this->fut_ang[j] + this->m_motor_res)
        dJointSetAMotorParam(this->motors[j], dParamVel, -this->jnt_vel[j]);
    else
        dJointSetAMotorParam(this->motors[j], dParamVel, 0);
}

dReal Robot::getJointForce(int body) {
    return this->m_joint_frc_max[body];
}

dSpaceID Robot::getSpaceID(void) {
    return this->space;
}

void Robot::enable(void) {
    dBodyEnable(this->body[CENTER]->bodyID);
}

void Robot::resetPID(int joint) {
    if ( joint == 4 ) {
        for ( int i = 0; i < NUM_DOF; i++ ) {
            this->pid[i].restart();
        }
    }
    else {
        this->pid[joint].restart();
    }
}

bool Robot::isDisabled(void) {
    return !(bool)dBodyIsEnabled(this->body[CENTER]->bodyID);
}

inline dReal Robot::D2R( dReal x ) {
    return x*M_PI/180;
}
inline dReal Robot::R2D( dReal x ) {
    return x/M_PI*180;
}
