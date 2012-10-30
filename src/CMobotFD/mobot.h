#ifndef MOBOT_H_
#define MOBOT_H_

#include "config.h"
#include "pid.h"
#include "threads.h"
#include <unistd.h>
#include <ode/ode.h>
#ifdef ENABLE_DOUBLE
#define EPSILON DBL_EPSILON
#else
#define EPSILON FLT_EPSILON
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
typedef enum imobot_joints_e {
	IMOBOT_JOINT1,
	IMOBOT_JOINT2,
	IMOBOT_JOINT3,
	IMOBOT_JOINT4
} iMobotJointID_t;
typedef enum mobot_joints_e {
	MOBOT_JOINT1,
	MOBOT_JOINT2,
	MOBOT_JOINT3,
	MOBOT_JOINT4
} mobotJointID_t;
typedef enum mobot_joint_state_e {
	MOBOT_NEUTRAL	= 0,
	MOBOT_FORWARD	= 1,
	MOBOT_BACKWARD	= 2,
	MOBOT_HOLD		= 3
} mobotJointState_t;

typedef struct robot_body_s {
	dBodyID bodyID;                         // id of body part
	dGeomID *geomID;                        // ids of geoms which make up each body part
} mobotBody_t;

class CMobotFD;
class CRobot4Sim : public robotSimThreads {
	public:
        CRobot4Sim(void);
        ~CRobot4Sim(void);
		void addToSim(dWorldID &world, dSpaceID &space);

		dReal getAngle(int i);
		int getJointAngle(int id, dReal &angle);
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

		int motionArch(dReal angle);
		int motionInchwormLeft(int num);
		int motionInchwormRight(int num);
		int motionRollBackward(dReal angle);
		int motionRollForward(dReal angle);
		int motionSkinny(dReal angle);
		int motionStand(void);
		int motionTurnLeft(dReal angle);
		int motionTurnRight(dReal angle);
		int motionTumbleRight(int num);
		int motionTumbleLeft(int num);
		int motionUnstand(void);

		int move(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveJoint(int id, dReal angle);
		int moveJointNB(int id, dReal angle);
		int moveJointTo(int id, dReal angle);
		int moveJointToNB(int id, dReal angle);
		int moveJointWait(int id);
		int moveTo(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveToDirect(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveToNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveToDirectNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);

		int resetToZero(void);

        void resetPID(int i = NUM_DOF);
        void updateMotorSpeed(int i);

		bool isHome(void);
		bool isComplete(void);
    private:
		dWorldID world;				// world for all robots
		dSpaceID space;				// space for this robot
		dJointID motor[4];			// motors
		dJointID joint[6];			// joints between body parts
		PID pid[4];					// PID control for each joint
		mobotJointState_t state[4];	// states
        mobotBody_t body[NUM_PARTS];// body parts
		dReal angle[4];				// angles
		dReal velocity[4];			// velocities
		dReal goal[4];				// goals
		dReal position[3];			// initial position
		dReal rotation[3];			// initial rotation
		dReal orientation[4];		// initial joint orientation
		bool success[4];				// trigger for goal

		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);                 // modify angle from ODE for endcaps to count continuously
		void build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);	// build body of mobot
		void build_center(dReal x, dReal y, dReal z, dMatrix3 R);						// build center
		void build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R);				// build endcap
		void create_fixed_joint(CRobot4Sim *attach, int face1, int face2);				// create fixed joint between modules
		void create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi);		// get rotation matrix from euler angles
		void extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi);	// get euler angles from rotation matrix
		bool is_joint_complete(int id);
	protected:
		dReal D2R(dReal x);              // convert degrees to radians
		dReal R2D(dReal x);              // convert radians to degrees
		double	m_motor_res,
				center_length, center_width, center_height, center_radius, center_offset,
				body_length, body_width, body_height, body_radius,
				body_inner_width_left, body_inner_width_right, body_end_depth, body_mount_center,
				end_width, end_height, end_depth, end_radius;
		dReal m_joint_vel_max[4];
		dReal m_joint_frc_max[4];
};

class CMobotSim : public CRobot4Sim {
	public:
		CMobotSim(void);
};

class CiMobotSim: public CRobot4Sim {
	public:
		CiMobotSim(void);
};

#endif  /* MOBOT_H_ */
