#include "robot.h"

Robot::Robot(int num_stp) {
    int j;

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

    this->bodyPart = new CiMobotFDPart[NUM_PARTS];
    this->bodyPart[ENDCAP_L].geomID = new dGeomID[7];
    this->bodyPart[BODY_L].geomID = new dGeomID[5];
    this->bodyPart[CENTER].geomID = new dGeomID[3];
    this->bodyPart[BODY_R].geomID = new dGeomID[5];
    this->bodyPart[ENDCAP_R].geomID = new dGeomID[7];
    this->joints = new dJointID[6];
    this->motors = new dJointID[4];
    this->pid = new PID[NUM_DOF];
    this->ang = new dReal[NUM_DOF*num_stp];
    this->vel = new dReal[NUM_DOF*num_stp];
    this->cur_ang = new dReal[NUM_DOF];
    this->fut_ang = new dReal[NUM_DOF];
    this->jnt_vel = new dReal[NUM_DOF];
    this->pos = new dReal[3];
    this->rot = new dReal[3];
    this->ori = new dReal[4];
    //for ( j = 0; j < NUM_DOF; j++ ) { this->pid[j].init(100, 1, 10, 0.1, this->m_t_step); }
    for ( j = 0; j < NUM_DOF; j++ ) { this->pid[j].init(100, 1, 10, 0.1, 0.004); }
    #ifdef ENABLE_DRAWSTUFF
    this->bodyPart[ENDCAP_L].num_geomID = 7;
    this->bodyPart[BODY_L].num_geomID = 5;
    this->bodyPart[CENTER].num_geomID = 3;
    this->bodyPart[BODY_R].num_geomID = 5;
    this->bodyPart[ENDCAP_R].num_geomID = 7;
    for ( j = 0; j < NUM_DOF; j++ ) {
        this->fut_ang[j] = this->ang[j];
        this->jnt_vel[j] = this->vel[j];
    }
    #endif
}

Robot::~Robot(void) {
    for ( int j = 0; j < NUM_PARTS; j++ ) delete [] this->bodyPart[j].geomID;
    delete [] this->bodyPart;
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
}

inline dReal Robot::D2R( dReal x ) {
    return x*M_PI/180;
}
inline dReal Robot::R2D( dReal x ) {
    return x/M_PI*180;
}
