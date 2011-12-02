#ifndef ROBOT_H_
#define ROBOT_H_

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

        dBodyID getBodyID(int body);
        dJointID getJointID(int joint);
        dJointID getMotorID(int motor);

        void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
        void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        //void buildAttached(int attNum, int face1, int face2);
        //void buildAttached(int attNum, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

        bool isDisabled(void);
        void enable(void);
        void resetPID(int joint = NUM_DOF);

        #ifdef ENABLE_DRAWSTUFF
        void drawRobot(void);
        #endif

        double  *cur_ang,                       // current angle of each body part
                *fut_ang,                       // future angle being driven toward
                *jnt_vel,                       // desired joint velocity
                *ang,                           // array of angles
                *vel,                           // array of velocities
                *ori;                           // initial orientation of body parts
    private:
        dWorldID world;                         // world for all robots
        dSpaceID space;                         // space for this robot
        dJointID    *joints,                    // joints between body parts
                    *motors;                    // motors to drive body parts
        Body **body;                            // body parts
        PID *pid;                               // PID control for each joint

        double  *pos,                           // initial position of robot
                *rot,                           // initial rotation of robot by three Euler angles
                m_motor_res,                    // motor angle resolution
                *m_joint_vel_max,               // maximum joint velocity possible
                *m_joint_vel_min,               // minimum joint velocity possible
                *m_joint_frc_max;               // maximum force that can be applied to each body part
        int m_num_stp;

        void rotation_matrix_from_euler_angles(dMatrix3 R, dReal psi, dReal theta, dReal phi);

        inline dReal D2R(dReal x);              // convert degrees to radians
        inline dReal R2D(dReal x);              // convert radians to degrees
};

#endif  /* ROBOT_H_ */