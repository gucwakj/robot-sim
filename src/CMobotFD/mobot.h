#ifndef MOBOT_H_
#define MOBOT_H_

#include "config.h"
#include "pid.h"
#include <ode/ode.h>
#ifdef ENABLE_DRAWSTUFF
    #include <drawstuff/drawstuff.h>
    #define DRAWSTUFF_TEXTURE_PATH "../opende/drawstuff/textures"
    #ifdef dDOUBLE
        #define dsDrawSphere dsDrawSphereD
        #define dsDrawBox dsDrawBoxD
        #define dsDrawCylinder dsDrawCylinderD
        #define dsDrawCapsule dsDrawCapsuleD
    #endif
#endif

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
enum robot_faces_e {
	MOBOT_FACE1 = 1,
	MOBOT_FACE2,
	MOBOT_FACE3,
	MOBOT_FACE4,
	MOBOT_FACE5,
	MOBOT_FACE6
};

typedef struct robot_body_s {
	dBodyID bodyID;                         // id of body part
	dGeomID *geomID;                        // ids of geoms which make up each body part
#ifdef ENABLE_DRAWSTUFF
	float color[3];                         // rgb color for each body part
	int num_geomID;                         // total number of geomID for part
#endif
} Body;

class CMobotFD;
class CRobot4Sim {
    public:
        CRobot4Sim(void);
        ~CRobot4Sim(void);
		void addToSim(dWorldID &world, dSpaceID &space, CMobotFD *sim, int type, int num);

        dReal getCurrentAngle(int i);
        dReal getPosition(int i);
        dReal getRotation(int i);
        dBodyID getBodyID(int body);
        dJointID getMotorID(int motor);

		void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void buildAttached00(CRobot4Sim *attach, int face1, int face2);
		void buildAttached10(CRobot4Sim *attach, int face1, int face2);
		void buildAttached01(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		void buildAttached11(CRobot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);

		void move(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		void moveNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		void moveWait(void);

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
        void drawRobot(void);
        #endif
    private:
        dWorldID world;                         // world for all robots
        dSpaceID space;                         // space for this robot
        CMobotFD *sim;		//simulation
        dJointID *joints,                       // joints between body parts
                 *motors;                       // motors to drive body parts
        Body     *body;                        // body parts
        PID      *pid;                          // PID control for each joint
        double   *pos,                          // initial position of robot
                 *rot,                          // initial rotation of robot by three Euler angles
                 *ang,                          // array of angles
                 *vel,                          // array of velocities
                 *ori,                          // initial orientation of body parts
                 *cur_ang,                      // current angle of each body part
                 *fut_ang,                      // future angle being driven toward
                 *jnt_vel;                      // desired joint velocity
        int m_num_stp;
		int m_type;
		int m_num;

		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);                 // modify angle from ODE for endcaps to count continuously
		void build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);
		void build_center(dReal x, dReal y, dReal z, dMatrix3 R);
		void build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R);
		void create_fixed_joint(CRobot4Sim *attach, int face1, int face2);                   // create fixed joint between modules
		void create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi);     // get rotation matrix from euler angles
		void extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi);    // get euler angles from rotation matrix
#ifdef ENABLE_DRAWSTUFF
		void draw_body(int id);
#endif
	protected:
		dReal D2R(dReal x);              // convert degrees to radians
		dReal R2D(dReal x);              // convert radians to degrees
		double	m_motor_res,
                *m_joint_vel_max,              // maximum joint velocity possible
                *m_joint_vel_min,              // minimum joint velocity possible
                *m_joint_frc_max,              // maximum force that can be applied to each body part
				center_length, center_width, center_height, center_radius, center_offset,
				body_length, body_width, body_height, body_radius,
				body_inner_width_left, body_inner_width_right, body_end_depth, body_mount_center,
				end_width, end_height, end_depth, end_radius;
};

class CMobotSim: public CRobot4Sim {
	public:
		CMobotSim(void);
};

class CiMobotSim: public CRobot4Sim {
	public:
		CiMobotSim(void);
};

#endif  /* MOBOT_H_ */