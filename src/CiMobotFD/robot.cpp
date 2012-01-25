#include "robot.h"

Robot::Robot(dWorldID &world, dSpaceID &space, int num_stp, int bot_type) {
    this->world = world;
    this->space = dHashSpaceCreate(space);
    this->m_num_stp = num_stp;

    if ( bot_type == IMOBOT ) {
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
    }
    else if ( bot_type == MOBOT ) {
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
    }

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

dReal Robot::getCurrentAngle(int i) {
    return this->cur_ang[i];
}

dReal Robot::getPosition(int i) {
    return this->pos[i];
}

dReal Robot::getRotation(int i) {
    return this->rot[i];
}

dBodyID Robot::getBodyID(int body) {
    return this->body[body]->getBodyID();
}

dJointID Robot::getMotorID(int motor) {
    return this->motors[motor];
}

void Robot::enable(void) {
    dBodyEnable(this->body[CENTER]->getBodyID());
}

void Robot::resetPID(int i) {
    if ( i == NUM_DOF )
        for ( int j = 0; j < NUM_DOF; j++ ) this->pid[j].restart();
    else
        this->pid[i].restart();
}

void Robot::updateCurrentAngle(int i) {
    if ( i == LE || i == RE )
        this->cur_ang[i] = mod_angle(this->cur_ang[i], dJointGetHingeAngle(this->joints[i]), dJointGetHingeAngleRate(this->joints[i]));
    else
        this->cur_ang[i] = dJointGetHingeAngle(this->joints[i]);
}

void Robot::updateFutureAngle(int i, int current_step, int enable) {
    if ( enable ) {
        if ( i == LE || i == RE ) {
            this->fut_ang[i] = this->ori[i];
            for ( int j = 0; j <= current_step; j++ ) { this->fut_ang[i] += this->ang[NUM_DOF*j + i]; }
        }
        else {
            this->fut_ang[i] = this->ang[NUM_DOF*current_step + i];
        }
    }
    else {
        this->fut_ang[i] = this->cur_ang[i];
    }
}

void Robot::updateJointVelocity(int i, int current_step) {
    this->jnt_vel[i] = this->vel[NUM_DOF*current_step + i];
}

void Robot::updateMotorSpeed(int i) {
    /*// with PID
    if (this->cur_ang[i] < this->fut_ang[i] - 10*this->m_motor_res)
        dJointSetA*MotorParam(this->motors[i], dParamVel, this->jnt_vel[i]);
    else if (this->cur_ang[i] > this->fut_ang[i] + 10*this->m_motor_res)
        dJointSetAMotorParam(this->motors[i], dParamVel, -this->jnt_vel[i]);
    else if (this->fut_ang[i] - 10*this->m_motor_res < this->cur_ang[i] &&  this->cur_ang[i] < this->fut_ang[i] - this->m_motor_res)
        dJointSetAMotorParam(this->motors[i], dParamVel, this->pid[i].update(this->fut_ang[i] - this->cur_ang[i]));
    else if (this->cur_ang[i] < this->fut_ang[i] + 10*this->m_motor_res && this->cur_ang[i] > this->fut_ang[i] + this->m_motor_res)
        dJointSetAMotorParam(this->motors[i], dParamVel, this->pid[i].update(this->cur_ang[i] - this->fut_ang[i]));
    else
        dJointSetAMotorParam(this->motors[i], dParamVel, 0);*/
    // without PID
    if (this->cur_ang[i] < this->fut_ang[i] - this->m_motor_res)
        dJointSetAMotorParam(this->motors[i], dParamVel, this->jnt_vel[i]);
    else if (this->cur_ang[i] > this->fut_ang[i] + this->m_motor_res)
        dJointSetAMotorParam(this->motors[i], dParamVel, -this->jnt_vel[i]);
    else
        dJointSetAMotorParam(this->motors[i], dParamVel, 0);
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
    this->create_rotation_matrix(R, psi, theta, phi);

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
    this->create_rotation_matrix(R, psi, theta, phi);

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

void Robot::buildAttached00(Robot *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3] = {0};
    dMatrix3 R, R1, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

    if ( face1 == 1 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = 0;
    }
    else if ( face1 == 1 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 1 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 1 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 1 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH;
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 1 && face2 == 6 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = 0;
    }
    else if ( face1 == 2 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 2 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 2 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 2 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 2 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 2 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 3 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 3 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 3 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 3 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 3 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 3 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER;
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 4 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 4 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 4 && face2 == 3 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 4 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 4 && face2 == 5 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = -BODY_WIDTH;
    }
    else if ( face1 == 4 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 5 && face2 == 1 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 5 && face2 == 2 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 5 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 5 && face2 == 4 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH - 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 5 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH    +   2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + 0.5*CENTER_LENGTH;
        m[1] = BODY_WIDTH;
    }
    else if ( face1 == 5 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER;
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 6 && face2 == 1 ) {
        dRSetIdentity(R1);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
        m[1] = 0;
    }
    else if ( face1 == 6 && face2 == 2 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 6 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 6 && face2 == 4 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 6 && face2 == 5 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH;
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH;
    }
    else if ( face1 == 6 && face2 == 6 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R, R1, R_att, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH;
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

void Robot::buildAttached10(Robot *attach, int face1, int face2) {
    // initialize variables
    dReal psi, theta, phi, m[3];
    dMatrix3 R, R1, R2, R3, R4, R5, R_att;

    // generate rotation matrix for base robot
    this->create_rotation_matrix(R_att, attach->getRotation(0), attach->getRotation(1), attach->getRotation(2));

    if ( face1 == 1 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getCurrentAngle(LE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH - BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                               + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - END_DEPTH - 0.5*BODY_WIDTH  + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                               + R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH                   + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH                   + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH               + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH               + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -  END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH                    + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[1]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -  END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[9]*(-BODY_END_DEPTH - BODY_LENGTH - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +  END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  + BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  + BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[1]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +  END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R1[9]*(BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - 0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getCurrentAngle(RE));
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + 0.5*CENTER_LENGTH);
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

void Robot::buildAttached01(Robot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH + R1[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                                                                   R1[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                                                   R1[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(0.5*CENTER_LENGTH);
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH - 2*END_DEPTH + R1[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] = R1[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] = R1[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[8]*(-0.5*CENTER_LENGTH);
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH  + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] = R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH                   + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH                   + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] = R1[8]*(-0.5*CENTER_LENGTH);
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R2[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH               + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] = BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] = R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = -0.5*CENTER_LENGTH + 2*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] = BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] = R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = -0.5*CENTER_LENGTH               + R1[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(0.5*CENTER_LENGTH);
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
        m[0] = -0.5*CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      -BODY_WIDTH + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
        m[1] = -BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
        m[1] = -BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH                    + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                      + R1[8]*(-0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[1]*(-0.5*CENTER_LENGTH);
        m[1] = -END_DEPTH - 0.5*BODY_WIDTH + R1[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R3[9]*(-0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
        m[1] = BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], r_b);
        dMultiply0(R, R1, R_att, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH                + R1[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R1[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R1[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + 2*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) + R1[0]*(0.5*CENTER_LENGTH);
        m[1] = BODY_WIDTH + R1[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER + R1[1]*(BODY_END_DEPTH + BODY_LENGTH) + R3[1]*(0.5*CENTER_LENGTH);
        m[1] = END_DEPTH + 0.5*BODY_WIDTH + R1[5]*(BODY_END_DEPTH + BODY_LENGTH) + R3[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(BODY_END_DEPTH + BODY_LENGTH) + R3[9]*(0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + R1[0]*(BODY_END_DEPTH + BODY_LENGTH) + R3[0]*(0.5*CENTER_LENGTH);
        m[1] = R1[4]*(BODY_END_DEPTH + BODY_LENGTH) + R3[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(BODY_END_DEPTH + BODY_LENGTH) + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, r_b);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(0.5*CENTER_LENGTH);
        m[1] = BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + R1[5]*(0.5*CENTER_LENGTH);
        m[2] = R1[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 1, 0, 0, -r_b);
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH + R1[1]*(-0.5*CENTER_LENGTH);
        m[1] = -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER + R1[5]*(-0.5*CENTER_LENGTH);
        m[2] = R1[9]*(-0.5*CENTER_LENGTH);
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
        m[0] = 0.5*CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH + 2*END_DEPTH + R1[0]*(BODY_END_DEPTH + BODY_LENGTH) + R3[0]*(0.5*CENTER_LENGTH);
        m[1] = R1[4]*(BODY_END_DEPTH + BODY_LENGTH) + R3[4]*(0.5*CENTER_LENGTH);
        m[2] = R1[8]*(BODY_END_DEPTH + BODY_LENGTH) + R3[8]*(0.5*CENTER_LENGTH);
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

void Robot::buildAttached11(Robot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) {
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
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH) + R5[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH) + R5[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[4]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH) + R5[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH - END_DEPTH - 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 1 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getCurrentAngle(LE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], -attach->getCurrentAngle(LE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[0]*(-2*END_DEPTH) + R5[0]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[4]*(-2*END_DEPTH) + R5[4]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[4]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH) + R3[8]*(-2*END_DEPTH) + R5[8]*(-BODY_END_DEPTH - BODY_LENGTH) + R7[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                               + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - END_DEPTH - 0.5*BODY_WIDTH  + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                               + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - BODY_WIDTH    + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH                   + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH                   + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                                      + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - BODY_WIDTH    + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 2 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) - END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH               + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + BODY_WIDTH    + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], attach->getCurrentAngle(LB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH   +   2*R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + BODY_WIDTH    + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER)                 + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH               + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 3 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(LB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, attach->getCurrentAngle(LB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = -0.5*CENTER_LENGTH + R1[0]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) + END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(-BODY_LENGTH - BODY_END_DEPTH + BODY_MOUNT_CENTER) +                              R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -  END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH                + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      -BODY_WIDTH + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  - BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  - BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH                    + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      - BODY_WIDTH    + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                      + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 4 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[1]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) -  END_DEPTH - 0.5*BODY_WIDTH + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[9]*(-BODY_END_DEPTH - BODY_LENGTH) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +  END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  + BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH                + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[1], R_att[5], R_att[9], -attach->getCurrentAngle(RB));
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], r_b);
        dMultiply0(R, R3, R2, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH                + R3[0]*(-0.5*CENTER_LENGTH);
        m[1] =                      BODY_WIDTH  + R3[4]*(-0.5*CENTER_LENGTH);
        m[2] =                                  + R3[8]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], r_b);
        dMultiply0(R, R5, R4, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], -r_b);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH    +   2*R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[0]*(0.5*CENTER_LENGTH);
        m[1] =                          2*R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)  + BODY_WIDTH    + R3[4]*(0.5*CENTER_LENGTH);
        m[2] =                          2*R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER)                  + R3[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 5 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -r_e);
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[1], R1[5], R1[9], r_e);
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[1]*(BODY_END_DEPTH + BODY_LENGTH) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +  END_DEPTH + 0.5*BODY_WIDTH + R3[5]*(BODY_END_DEPTH + BODY_LENGTH) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH - BODY_MOUNT_CENTER) +                               R3[9]*(BODY_END_DEPTH + BODY_LENGTH) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 1 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], 0);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], -r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH) + R5[0]*(BODY_END_DEPTH + BODY_LENGTH) + R7[0]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH) + R5[4]*(BODY_END_DEPTH + BODY_LENGTH) + R7[4]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH) + R5[8]*(BODY_END_DEPTH + BODY_LENGTH) + R7[8]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 2 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 3 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], -r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 4 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], -M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[1]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[5]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER) + R5[9]*(0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 5 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI/2);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[1], R6[5], R6[9], r_b);
        dMultiply0(R, R7, R6, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -r_b);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[1]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[1]*(-0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[5]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[5]*(-0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH + END_DEPTH + 0.5*BODY_WIDTH) + R3[9]*(-BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER) + R5[9]*(-0.5*CENTER_LENGTH);
    }
    else if ( face1 == 6 && face2 == 6 ) {
        // generate rotation matrix
        dRFromAxisAndAngle(R1, R_att[2], R_att[6], R_att[10], M_PI);
        dMultiply0(R2, R1, R_att, 3, 3, 3);
        dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], attach->getCurrentAngle(RB));
        dMultiply0(R4, R3, R2, 3, 3, 3);
        dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], -attach->getCurrentAngle(RE));
        dMultiply0(R6, R5, R4, 3, 3, 3);
        dRFromAxisAndAngle(R7, R6[0], R6[4], R6[8], -r_e);
        dMultiply0(R8, R7, R6, 3, 3, 3);
        dRFromAxisAndAngle(R9, R8[1], R8[5], R8[9], r_b);
        dMultiply0(R, R9, R8, 3, 3, 3);

        // generate offset for mass center of new module
        dRFromAxisAndAngle(R1, 0, 1, 0, -attach->getCurrentAngle(RB));
        dRFromAxisAndAngle(R2, R1[0], R1[4], R1[8], attach->getCurrentAngle(RE));
        dMultiply0(R3, R2, R1, 3, 3, 3);
        dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], r_e);
        dMultiply0(R5, R4, R3, 3, 3, 3);
        dRFromAxisAndAngle(R6, R5[1], R5[5], R5[9], -r_b);
        dMultiply0(R7, R6, R5, 3, 3, 3);
        m[0] = 0.5*CENTER_LENGTH +  R1[0]*(BODY_LENGTH + BODY_END_DEPTH) + R3[0]*(2*END_DEPTH) + R5[0]*(BODY_END_DEPTH + BODY_LENGTH) + R7[0]*(0.5*CENTER_LENGTH);
        m[1] =                      R1[4]*(BODY_LENGTH + BODY_END_DEPTH) + R3[4]*(2*END_DEPTH) + R5[4]*(BODY_END_DEPTH + BODY_LENGTH) + R7[4]*(0.5*CENTER_LENGTH);
        m[2] =                      R1[8]*(BODY_LENGTH + BODY_END_DEPTH) + R3[8]*(2*END_DEPTH) + R5[8]*(BODY_END_DEPTH + BODY_LENGTH) + R7[8]*(0.5*CENTER_LENGTH);
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

bool Robot::isDisabled(void) {
    return !(bool)dBodyIsEnabled(this->body[CENTER]->getBodyID());
}

bool Robot::isJointDisabled(int i, int current_step) {
    return ( (int)(this->ang[NUM_DOF*current_step + i]) == (int)(D2R(123456789)) );
}

bool Robot::isHome(void) {
    return ( fabs(this->cur_ang[LE]) < DBL_EPSILON && fabs(this->cur_ang[LB]) < DBL_EPSILON && fabs(this->cur_ang[RB]) < DBL_EPSILON && fabs(this->cur_ang[RE]) < DBL_EPSILON );
}

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

dReal Robot::D2R( dReal x ) {
    return x*M_PI/180;
}

dReal Robot::R2D( dReal x ) {
    return x/M_PI*180;
}

void Robot::create_fixed_joint(Robot *attach, int face1, int face2) {
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

void Robot::create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi) {
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

void Robot::extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi) {
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

#ifdef ENABLE_DRAWSTUFF
void Robot::drawRobot(void) {
    for (int i = 0; i < NUM_PARTS; i++) {
        this->body[i]->drawBody();
    }
}
#endif