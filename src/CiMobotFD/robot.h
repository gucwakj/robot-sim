#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>
#include <ode/ode.h>
//#include "body.h"
#include "pid.h"

// iMobot dimension macros
#ifndef CIMOBOTIK_H_
#define CENTER_LENGTH       0.07303
#define CENTER_WIDTH        0.02540
#define CENTER_HEIGHT       0.06909
#define CENTER_RADIUS       0.03554
#define BODY_LENGTH         0.03785
#define BODY_WIDTH          0.07239
#define BODY_HEIGHT         0.07239
#define BODY_RADIUS         0.03620
#define BODY_INNER_WIDTH    0.02287
#define BODY_END_DEPTH      0.01994
#define BODY_MOUNT_CENTER   0.03792
#define END_WIDTH           0.07239
#define END_HEIGHT          0.07239
#define END_DEPTH           0.00476
#define END_RADIUS          0.01778
#endif

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

enum robot_build_e {        // build or rebuild a part
    BUILD,
    REBUILD
};

class Robot {
    public:
        Robot(int num_stp);
        ~Robot(void);

        void setAngles(dReal *ang);
        void setAngularVelocity(dReal *vel);
        void setMotorSpeed(int j);

        dReal getJointForce(int body);

        typedef struct cimobotfdpart_s {            // information about each body part
            dBodyID bodyID;                         // id of body part
            dGeomID *geomID;                        // ids of geoms which make up each body part
            #ifdef ENABLE_DRAWSTUFF
            float color[3];                         // rgb color for each body part
            int num_geomID;                         // total number of geomID for part
            #endif
        } CiMobotFDPart;

        CiMobotFDPart *bodyPart;                // body parts
        //Body *bodyPart;                         // body parts
        PID *pid;                               // PID control for each joint
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
        double  m_motor_res,                    // motor angle resolution
                *m_joint_vel_max,               // maximum joint velocity possible
                *m_joint_vel_min,               // minimum joint velocity possible
                *m_joint_frc_max;               // maximum force that can be applied to each body part
        int m_num_stp;

        inline dReal D2R(dReal x);              // convert degrees to radians
        inline dReal R2D(dReal x);              // convert radians to degrees
};

#endif  /* ROBOT_H_ */