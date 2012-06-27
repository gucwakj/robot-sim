#ifndef MOBOT_H_
#define MOBOT_H_

#include <ode/ode.h>
#include "pid.h"

enum robot_type_e {         // type of robot in simulation
    IMOBOT,
    MOBOT
};
enum robot_bodies_e {       // each body which has a degree of freedom
    LE,
    LB,
    RB,
    RE,
    NUM_DOF
};
enum robot_pieces_e {       // each body part which is built
    ENDCAP_L,
    BODY_L,
    CENTER,
    BODY_R,
    ENDCAP_R,
    NUM_PARTS
};
struct robot_body_s {
	        dGeomID *geomID;                        // ids of geoms which make up each body part

        #ifdef ENABLE_DRAWSTUFF
        float color[3];                         // rgb color for each body part
        int num_geomID;                         // total number of geomID for part
        #endif
} Body;

class Mobot {
    public:
        Mobot(dWorldID &world, dSpaceID &space, int num_stp, int bot_type);
        ~Mobot(void);

        void setAngles(dReal *ang);
        void setAngularVelocity(dReal *vel);

        dReal getCurrentAngle(int i);
        dReal getPosition(int i);
        dReal getRotation(int i);
        dBodyID getBodyID(int body);
        dJointID getMotorID(int motor);

        void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
        void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        void buildAttached00(Mobot *attach, int face1, int face2);
        void buildAttached10(Mobot *attach, int face1, int face2);
        void buildAttached01(Mobot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
        void buildAttached11(Mobot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

        void enable(void);
        void resetPID(int i = NUM_DOF);
        void updateCurrentAngle(int i);
        void updateFutureAngle(int i, int current_step, int enable);
        void updateJointVelocity(int i, int current_step);
        void updateMotorSpeed(int i);

        bool isDisabled(void);
        bool isJointDisabled(int i, int current_step);
        bool isHome(void);

        #ifdef ENABLE_DRAWSTUFF
        void drawMobot(void);
        #endif
    private:
        dWorldID world;                         // world for all robots
        dSpaceID space;                         // space for this robot
        dJointID *joints,                       // joints between body parts
                 *motors;                       // motors to drive body parts
        Body     **body;                        // body parts
        PID      *pid;                          // PID control for each joint
        double   m_motor_res,                   // motor angle resolution
                 *pos,                          // initial position of robot
                 *rot,                          // initial rotation of robot by three Euler angles
                 *ang,                          // array of angles
                 *vel,                          // array of velocities
                 *ori,                          // initial orientation of body parts
                 *cur_ang,                      // current angle of each body part
                 *fut_ang,                      // future angle being driven toward
                 *jnt_vel,                      // desired joint velocity
                 *m_joint_vel_max,              // maximum joint velocity possible
                 *m_joint_vel_min,              // minimum joint velocity possible
                 *m_joint_frc_max;              // maximum force that can be applied to each body part
        int      m_num_stp;

        dReal D2R(dReal x);              // convert degrees to radians
        dReal R2D(dReal x);              // convert radians to degrees
        dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);                 // modify angle from ODE for endcaps to count continuously
        void create_fixed_joint(Mobot *attach, int face1, int face2);                   // create fixed joint between modules
        void create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi);     // get rotation matrix from euler angles
        void extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi);    // get euler angles from rotation matrix
};

#endif  /* MOBOT_H_ */