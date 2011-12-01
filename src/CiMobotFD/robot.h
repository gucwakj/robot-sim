#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>
#include <ode/ode.h>
#include "body.h"
#include "pid.h"

enum robot_bodies_e {       // each body which has a degree of freedom
    LE,
    LB,
    RB,
    RE,
    NUM_DOF
};

enum robot_pieces_e {       // each body part which is built
    BODY_L,
    BODY_R,
    CENTER,
    ENDCAP_L,
    ENDCAP_R,
    NUM_PARTS
};

class Robot {
    public:
        Robot(dWorldID &world, dSpaceID &space, int num_stp);
        ~Robot(void);

        void setAngles(dReal *ang);
        void setAngularVelocity(dReal *vel);
        void setMotorSpeed(int j);

        dReal getJointForce(int body);
        dSpaceID getSpaceID(void);

        void resetPID(int joint = NUM_DOF);
        void enable(void);
        bool isDisabled(void);

        Body **body;                            // body parts
        dJointID    *joints,                    // joints between body parts
                    *motors;                    // motors to drive body parts
        double  *cur_ang,                       // current angle of each body part
                *fut_ang,                       // future angle being driven toward
                *jnt_vel,                       // desired joint velocity
                *ang,                           // array of angles
                *vel,                           // array of velocities
                *pos,                           // initial position of robot
                *rot,                           // initial rotation of robot by three Euler angles
                *ori;                           // initial orientation of body parts
    private:
        dWorldID world;                         // world for all robots
        dSpaceID space;                         // space for this robot
        PID *pid;                               // PID control for each joint

        double  m_motor_res,                    // motor angle resolution
                *m_joint_vel_max,               // maximum joint velocity possible
                *m_joint_vel_min,               // minimum joint velocity possible
                *m_joint_frc_max;               // maximum force that can be applied to each body part
        int m_num_stp;

        inline dReal D2R(dReal x);              // convert degrees to radians
        inline dReal R2D(dReal x);              // convert radians to degrees
};

#endif  /* ROBOT_H_ */