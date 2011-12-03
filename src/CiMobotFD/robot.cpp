#include "robot.h"

Robot::Robot(dWorldID &world, dSpaceID &space, int num_stp) {
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

    for ( int i = 0; i < NUM_DOF*num_stp; i++ ) {
        this->vel[i] = this->m_joint_vel_max[i%NUM_DOF];
    }

    for ( int i = 0; i < NUM_DOF; i++ ) {
        this->pid[i].init(100, 1, 10, 0.1, 0.004);
        #ifdef ENABLE_DRAWSTUFF
        this->jnt_vel[i] = this->vel[i];
        #endif
    }
}

Robot::~Robot(void) {
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

dReal Robot::getCurrentAngle(int j) {
    return this->cur_ang[j];
}

dBodyID Robot::getBodyID(int body) {
    return this->body[body]->getBodyID();
}

dJointID Robot::getJointID(int joint) {
    return this->joints[joint];
}

dJointID Robot::getMotorID(int motor) {
    return this->motors[motor];
}

bool Robot::isDisabled(void) {
    return !(bool)dBodyIsEnabled(this->body[CENTER]->getBodyID());
}

bool Robot::isJointDisabled(int j, int current_step) {
    return ( (int)(this->ang[NUM_DOF*current_step + j]) == (int)(D2R(123456789)) );
}

void Robot::enable(void) {
    dBodyEnable(this->body[CENTER]->getBodyID());
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

void Robot::updateCurrentAngle(int j) {
    if ( j == LE || j == RE )
        this->cur_ang[j] = mod_angle(this->cur_ang[j], dJointGetHingeAngle(this->joints[j]), dJointGetHingeAngleRate(this->joints[j]));
    else
        this->cur_ang[j] = dJointGetHingeAngle(this->joints[j]);
}

void Robot::updateFutureAngle(int j, int current_step, int enable) {
    if ( enable ) {
        if ( j == LE || j == RE ) {
            this->fut_ang[j] = this->ori[j];
            for ( int k = 0; k <= current_step; k++ ) { this->fut_ang[j] += this->ang[NUM_DOF*k + j]; }
        }
        else {
            this->fut_ang[j] = this->ang[NUM_DOF*current_step + j];
        }
    }
    else {
        this->fut_ang[j] = this->cur_ang[j];
    }
}

void Robot::updateJointVelocity(int j, int current_step) {
    this->jnt_vel[j] = this->vel[NUM_DOF*current_step + j];
}

void Robot::updateMotorSpeed(int j) {
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

void Robot::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) {
    // adjust input height by body height
    z += BODY_HEIGHT/2;
    // convert input angles to radians
    psi = D2R(psi);         // roll: x
    theta = D2R(theta);     // pitch: -y
    phi = D2R(phi);         // yaw: z

    // create rotation matrix for robot
    dMatrix3 R;
    rotation_matrix_from_euler_angles(R, psi, theta, phi);

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH/2, 0, 0, -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH, 0, 0};
    dReal lb[6] = {-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH/2, 0, 0, -CENTER_LENGTH/2, CENTER_WIDTH/2, 0};
    dReal rb[6] = {CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2, 0, 0, CENTER_LENGTH/2, CENTER_WIDTH/2, 0};
    dReal re[6] = {CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2, 0, 0, CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH, 0, 0};

    // build pieces of module
    this->body[ENDCAP_L]->buildEndcap(R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->body[BODY_L]->buildLeftBody(R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->body[CENTER]->buildCenter(x, y, z, R);
    this->body[BODY_R]->buildRightBody(R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->body[ENDCAP_R]->buildEndcap(R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // store position and rotation of center of module
    this->pos[0] = x;
    this->pos[1] = y;
    this->pos[2] = z - BODY_HEIGHT/2;
    this->rot[0] = psi;
    this->rot[1] = theta;
    this->rot[2] = phi;

    // joint for left endcap to body
    this->joints[0] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[0], this->body[BODY_L]->getBodyID(), this->body[ENDCAP_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(this->joints[0], R[0], R[4], R[8]);
    dJointSetHingeParam(this->joints[0], dParamCFM, 0);

    // joint for center to left body 1
    this->joints[1] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[1], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[1], R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joints[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joints[1], dParamCFM, 0);

    // joint for center to left body 2
    this->joints[4] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[4], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[4], R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joints[4], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joints[4], dParamCFM, 0);

    // joint for center to right body 1
    this->joints[2] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[2], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[2], R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joints[2], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joints[2], dParamCFM, 0);

    // joint for center to right body 2
    this->joints[5] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[5], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[5], R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joints[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joints[5], dParamCFM, 0);

    // joint for right body to endcap
    this->joints[3] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[3], this->body[BODY_R]->getBodyID(), this->body[ENDCAP_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(this->joints[3], -R[0], -R[4], -R[8]);
    dJointSetHingeParam(this->joints[3], dParamCFM, 0);

    // motor for left endcap to body
    this->motors[0] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[0], this->body[BODY_L]->getBodyID(), this->body[ENDCAP_L]->getBodyID());
    dJointSetAMotorMode(this->motors[0], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[0], 1);
    dJointSetAMotorAxis(this->motors[0], 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(this->motors[0], 0, 0);
    dJointSetAMotorParam(this->motors[0], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[0], dParamFMax, this->m_joint_frc_max[LE]);

    // motor for center to left body
    this->motors[1] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[1], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetAMotorMode(this->motors[1], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[1], 1);
    dJointSetAMotorAxis(this->motors[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(this->motors[1], 0, 0);
    dJointSetAMotorParam(this->motors[1], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[1], dParamFMax, this->m_joint_frc_max[LB]);

    // motor for center to right body
    this->motors[2] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[2], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetAMotorMode(this->motors[2], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[2], 1);
    dJointSetAMotorAxis(this->motors[2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(this->motors[2], 0, 0);
    dJointSetAMotorParam(this->motors[2], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[2], dParamFMax, this->m_joint_frc_max[RB]);

    // motor for right body to endcap
    this->motors[3] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[3], this->body[BODY_R]->getBodyID(), this->body[ENDCAP_R]->getBodyID());
    dJointSetAMotorMode(this->motors[3], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[3], 1);
    dJointSetAMotorAxis(this->motors[3], 0, 1, -R[0], -R[4], -R[8]);
    dJointSetAMotorAngle(this->motors[3], 0, 0);
    dJointSetAMotorParam(this->motors[3], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[3], dParamFMax, this->m_joint_frc_max[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->body[i]->getBodyID(), 0.1, 0.1);
}

void Robot::build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    // adjust input height by body height
    z += BODY_HEIGHT/2;
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
    rotation_matrix_from_euler_angles(R, psi, theta, phi);

    // store initial body angles into array
    this->cur_ang[LE] = r_le;
    this->cur_ang[LB] = r_lb;
    this->cur_ang[RB] = r_rb;
    this->cur_ang[RE] = r_re;
    this->fut_ang[LE] += r_le;
    this->fut_ang[RE] += r_re;

    // offset values for each body part[0-2] and joint[3-5] from center
    dReal le[6] = {-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH/2, 0, 0, -CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH, 0, 0};
    dReal lb[6] = {-CENTER_LENGTH/2 - BODY_LENGTH - BODY_END_DEPTH/2, 0, 0, -CENTER_LENGTH/2, CENTER_WIDTH/2, 0};
    dReal rb[6] = {CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH/2, 0, 0, CENTER_LENGTH/2, CENTER_WIDTH/2, 0};
    dReal re[6] = {CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2, 0, 0, CENTER_LENGTH/2 + BODY_LENGTH + BODY_END_DEPTH, 0, 0};

    this->body[ENDCAP_L]->buildEndcap(R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
    this->body[BODY_L]->buildLeftBody(R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
    this->body[CENTER]->buildCenter(x, y, z, R);
    this->body[BODY_R]->buildRightBody(R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
    this->body[ENDCAP_R]->buildEndcap(R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

    // store position and rotation of center of module
    this->pos[0] = x;
    this->pos[1] = y;
    this->pos[2] = z - BODY_HEIGHT/2;
    this->rot[0] = psi;
    this->rot[1] = theta;
    this->rot[2] = phi;
    this->ori[0] = r_le;
    this->ori[1] = r_lb;
    this->ori[2] = r_rb;
    this->ori[3] = r_re;

    // joint for left endcap to body
    this->joints[0] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[0], this->body[BODY_L]->getBodyID(), this->body[ENDCAP_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x, R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y, R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
    dJointSetHingeAxis(this->joints[0], R[0], R[4], R[8]);
    dJointSetHingeParam(this->joints[0], dParamCFM, 0);

    // joint for center to left body 1
    this->joints[1] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[1], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[1], R[0]*lb[3] + R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] + R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] + R[9]*lb[4] + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joints[1], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joints[1], dParamCFM, 0);

    // joint for center to left body 2
    this->joints[4] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[4], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetHingeAnchor(this->joints[4], R[0]*lb[3] - R[1]*lb[4] + R[2]*lb[5] + x, R[4]*lb[3] - R[5]*lb[4] + R[6]*lb[5] + y, R[8]*lb[3] - R[9]*lb[4] + R[10]*lb[5] + z);
    dJointSetHingeAxis(this->joints[4], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joints[4], dParamCFM, 0);

    // joint for center to right body 1
    this->joints[2] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[2], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[2], R[0]*rb[3] + R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] + R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] + R[9]*rb[4] + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joints[2], R[1], R[5], R[9]);
    dJointSetHingeParam(this->joints[2], dParamCFM, 0);

    // joint for center to right body 2
    this->joints[5] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[5], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[5], R[0]*rb[3] - R[1]*rb[4] + R[2]*rb[5] + x, R[4]*rb[3] - R[5]*rb[4] + R[6]*rb[5] + y, R[8]*rb[3] - R[9]*rb[4] + R[10]*rb[5] + z);
    dJointSetHingeAxis(this->joints[5], -R[1], -R[5], -R[9]);
    dJointSetHingeParam(this->joints[5], dParamCFM, 0);

    // joint for right body to endcap
    this->joints[3] = dJointCreateHinge(this->world, 0);
    dJointAttach(this->joints[3], this->body[BODY_R]->getBodyID(), this->body[ENDCAP_R]->getBodyID());
    dJointSetHingeAnchor(this->joints[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x, R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y, R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
    dJointSetHingeAxis(this->joints[3], -R[0], -R[4], -R[8]);
    dJointSetHingeParam(this->joints[3], dParamCFM, 0);

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
    dReal le_r[3] = {-CENTER_LENGTH/2 - (BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_lb), 0, (BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_lb)};
    dReal lb_r[3] = {-CENTER_LENGTH/2 - (BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_lb), 0, (BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_lb)};
    dReal rb_r[3] = {CENTER_LENGTH/2 + (BODY_LENGTH + BODY_END_DEPTH/2)*cos(r_rb), 0, (BODY_LENGTH + BODY_END_DEPTH/2)*sin(r_rb)};
    dReal re_r[3] = {CENTER_LENGTH/2 + (BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*cos(r_rb), 0, (BODY_LENGTH + BODY_END_DEPTH + END_DEPTH/2)*sin(r_rb)};

    // re-build pieces of module
    this->body[ENDCAP_L]->buildEndcap(R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
    this->body[BODY_L]->buildLeftBody(R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, r_lb);
    this->body[BODY_R]->buildRightBody(R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, r_rb);
    this->body[ENDCAP_R]->buildEndcap(R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);

    // motor for left endcap to body
    this->motors[0] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[0], this->body[BODY_L]->getBodyID(), this->body[ENDCAP_L]->getBodyID());
    dJointSetAMotorMode(this->motors[0], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[0], 1);
    dJointSetAMotorAxis(this->motors[0], 0, 1, R_lb[0], R_lb[4], R_lb[8]);
    dJointSetAMotorAngle(this->motors[0], 0, 0);
    dJointSetAMotorParam(this->motors[0], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[0], dParamFMax, this->m_joint_frc_max[LE]);

    // motor for center to left body
    this->motors[1] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[1], this->body[CENTER]->getBodyID(), this->body[BODY_L]->getBodyID());
    dJointSetAMotorMode(this->motors[1], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[1], 1);
    dJointSetAMotorAxis(this->motors[1], 0, 1, -R[1], -R[5], -R[9]);
    dJointSetAMotorAngle(this->motors[1], 0, 0);
    dJointSetAMotorParam(this->motors[1], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[1], dParamFMax, this->m_joint_frc_max[LB]);

    // motor for center to right body
    this->motors[2] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[2], this->body[CENTER]->getBodyID(), this->body[BODY_R]->getBodyID());
    dJointSetAMotorMode(this->motors[2], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[2], 1);
    dJointSetAMotorAxis(this->motors[2], 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(this->motors[2], 0, 0);
    dJointSetAMotorParam(this->motors[2], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[2], dParamFMax, this->m_joint_frc_max[RB]);

    // motor for right body to endcap
    this->motors[3] = dJointCreateAMotor(this->world, 0);
    dJointAttach(this->motors[3], this->body[BODY_R]->getBodyID(), this->body[ENDCAP_R]->getBodyID());
    dJointSetAMotorMode(this->motors[3], dAMotorUser);
    dJointSetAMotorNumAxes(this->motors[3], 1);
    dJointSetAMotorAxis(this->motors[3], 0, 1, -R_rb[0], -R_rb[4], -R_rb[8]);
    dJointSetAMotorAngle(this->motors[3], 0, 0);
    dJointSetAMotorParam(this->motors[3], dParamCFM, 0);
    dJointSetAMotorParam(this->motors[3], dParamFMax, this->m_joint_frc_max[RE]);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(this->body[i]->getBodyID(), 0.1, 0.1);
}

/*void Robot::buildAttached(int attNum, int face1, int face2, int mode) {
    switch (mode) {
        case 00:
            this->build_attached_00(attNum, face1, face2);
            break;
        case 10:
            this->build_attached_10(attNum, face1, face2);
            break;
    }
}

void Robot::buildAttached(int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
    switch (mode) {
        case 01:
            this->build_attached_00(attNum, face1, face2, r_le, r_lb, r_rb, r_re);
            break;
        case 11:
            this->build_attached_10(attNum, face1, face2, r_le, r_lb, r_rb, r_re);
            break;
    }
}*/

dReal Robot::mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate) {
    dReal new_ang = 0;
    int stp = (int)( fabs(past_ang) / M_PI );
    dReal past_ang_mod = fabs(past_ang) - stp*M_PI;

    if ( (int)ang_rate == 0 ) {
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

void Robot::rotation_matrix_from_euler_angles(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
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

inline dReal Robot::D2R( dReal x ) {
    return x*M_PI/180;
}

inline dReal Robot::R2D( dReal x ) {
    return x/M_PI*180;
}

#ifdef ENABLE_DRAWSTUFF
void Robot::drawRobot(void) {
    for (int j = 0; j < NUM_PARTS; j++) {
        this->body[j]->drawBody();
    }
}
#endif