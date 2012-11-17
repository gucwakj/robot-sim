#ifndef MOBOT_H_
#define MOBOT_H_

#include "config.h"
#include "pid.h"
#include "robot.h"
#ifdef ENABLE_DOUBLE
#define EPSILON DBL_EPSILON
#else
#define EPSILON FLT_EPSILON
#endif

typedef enum imobot_faces_e {
	IMOBOT_FACE1 = 1,
	IMOBOT_FACE2,
	IMOBOT_FACE3,
	IMOBOT_FACE4,
	IMOBOT_FACE5,
	IMOBOT_FACE6
} iMobotFaceID_t;
typedef enum mobot_faces_e {
	MOBOT_FACE1 = 1,
	MOBOT_FACE2,
	MOBOT_FACE3,
	MOBOT_FACE4,
	MOBOT_FACE5,
	MOBOT_FACE6
} mobotFaceID_t;
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

class robot4Sim : virtual private robotSim {
	friend class CMobotFD;

	// public api to mimic CMobot clas
	public:
		robot4Sim(void);
		~robot4Sim(void);

		int getJointAngle(int id, dReal &angle);

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
    private:
		enum robot_pieces_e {       // each body part which is built
			ENDCAP_L,
			BODY_L,
			CENTER,
			BODY_R,
			ENDCAP_R,
			NUM_PARTS
		};
		enum robot_bodies_e {       // each body which has a degree of freedom
			LE,
			LB,
			RB,
			RE,
			NUM_DOF
		};

		// private member variables
		dWorldID world;						// world for all robots
		dSpaceID space;						// space for this robot
		dBodyID body[NUM_PARTS];			// body parts
		dGeomID *geom[NUM_PARTS];			// geometries of each body part
		dJointID motor[NUM_DOF];			// motors
		dJointID joint[6];					// joints between body parts
		PID pid[NUM_DOF];					// PID control for each joint
		mobotJointState_t state[NUM_DOF];	// states
		dReal angle[NUM_DOF];				// angles
		dReal velocity[NUM_DOF];			// velocities
		dReal goal[NUM_DOF];				// goals
		dReal rotation[3];					// initial rotation
		bool success[NUM_DOF];				// trigger for goal

		// private functions inherited from robotSim class
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual void buildAttached00(robot4Sim *attach, int face1, int face2);
		virtual void buildAttached10(robot4Sim *attach, int face1, int face2);
		virtual void buildAttached01(robot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual void buildAttached11(robot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual dReal getAngle(int i);
		virtual bool getSuccess(int i);
		virtual dReal getPosition(int i);
		virtual dReal getRotation(int i);
		virtual dBodyID getBodyID(int id);
		virtual dJointID getMotorID(int id);
		virtual bool isHome(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);
		virtual void simAddRobot(dWorldID &world, dSpaceID &space);

		// private functions
        //void resetPID(int i = NUM_DOF);
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);                 // modify angle from ODE for endcaps to count continuously
		void build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);	// build body of mobot
		void build_center(dReal x, dReal y, dReal z, dMatrix3 R);						// build center
		void build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R);				// build endcap
		void create_fixed_joint(robot4Sim *attach, int face1, int face2);				// create fixed joint between modules
		void create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi);		// get rotation matrix from euler angles
		void extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi);	// get euler angles from rotation matrix
	protected:
		double	m_motor_res,
				center_length, center_width, center_height, center_radius, center_offset,
				body_length, body_width, body_height, body_radius,
				body_inner_width_left, body_inner_width_right, body_end_depth, body_mount_center,
				end_width, end_height, end_depth, end_radius;
		dReal m_joint_vel_max[NUM_DOF];
		dReal m_joint_frc_max[NUM_DOF];
};

class mobotSim : public robot4Sim {
	public:
		mobotSim(void);
};

class iMobotSim : public robot4Sim {
	public:
		iMobotSim(void);
};

#endif  /* MOBOT_H_ */
