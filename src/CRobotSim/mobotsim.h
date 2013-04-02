#ifndef MOBOT_H_
#define MOBOT_H_

#include "config.h"
#include "pid.h"
#include "robot.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif /* ENABLE_GRAPHICS */
#include "robotsim.h"
#ifdef ENABLE_DOUBLE
#define EPSILON DBL_EPSILON
#else
#define EPSILON FLT_EPSILON
#endif /* ENABLE_DOUBLE */

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
typedef enum mobot_connector_e {
	SIMPLE,
	CASTER,
	BIGWHEEL,
	SMALLWHEEL,
	L,
	TANK,
	NUM_CONNECTORS
} mobotConnector_t;

class CRobotSim;

class CRobot4 : virtual public CRobot {
	friend class CRobotSim;
	friend class robot4NodeCallback;

	// public api to mimic CMobot clas
	public:
		CRobot4(void);
		~CRobot4(void);

		/*int connect(CRobotSim &sim);
		int connect(CRobotSim &sim, dReal x, dReal y, dReal z);
		int connect(CRobotSim &sim, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		int connect(CRobotSim &sim, dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		int connect(CRobotSim &sim, CRobot4 *base, int face1, int face2);
		int connect(CRobotSim &sim, CRobot4 *base, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);*/
		int connect(void);
		int getJointAngle(int id, dReal &angle);
		int getJointSpeed(int id, double &speed);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
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
		int recordAngle(int id, dReal *time, dReal *angle, int num, dReal seconds, dReal threshold = 0.0);
		int recordAngles(dReal *time, dReal *angle1, dReal *angle2, dReal *angle3, dReal *angle4, int num, dReal seconds, dReal threshold = 0.0);
		int recordWait(void);
		int resetToZero(void);
		int setJointSpeed(int id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
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
		typedef struct recordAngleArg_s {
			CRobot4 *robot;
			int id;
			int num;
			int msecs;
			double *time;
			double *angle1;
			double *angle2;
			double *angle3;
			double *angle4;
		} recordAngleArg_t;
		typedef struct conn_s {
			int face, type;
			dBodyID body;
			dGeomID *geom;
			struct conn_s *next;
		} *conn_t;

		// private member variables
		dWorldID _world;			// world for all robots
		dSpaceID _space;			// space for this robot
		dBodyID  _body[NUM_PARTS];	// body parts
		dGeomID* _geom[NUM_PARTS];	// geometries of each body part
		dJointID _motor[NUM_DOF];	// motors
		dJointID _joint[6];			// joints between body parts
		conn_t _conn;
		dReal* _clock;				// world clock
		dReal _angle[NUM_DOF];		// angles
		dReal _velocity[NUM_DOF];	// velocities
		dReal _goal[NUM_DOF];		// goals
		dReal _maxSpeed[NUM_DOF];	// maximum joint speeds
		PID _pid[NUM_DOF];			// PID control for each joint
		int  _state[NUM_DOF];		// states
		int _id;
		bool _success[NUM_DOF];		// trigger for goal
		bool _recording[NUM_DOF];	// recording in progress

		// private functions inherited from CRobot class
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi);
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual void buildAttached00(CRobot *attach, int face1, int face2);
		virtual void buildAttached10(CRobot *attach, int face1, int face2);
		virtual void buildAttached01(CRobot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual void buildAttached11(CRobot *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		virtual int addConnector(int type, int face);
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root);
#endif /* ENABLE_GRAPHICS */
		virtual int getID(void);
		virtual dReal getAngle(int i);
		virtual bool getSuccess(int i);
		virtual dReal getPosition(int body, int i);
		virtual dReal getRotation(int body, int i);
		virtual dBodyID getBodyID(int id);
		virtual dJointID getMotorID(int id);
		virtual int getType(void);
		virtual int setID(int id);
		virtual bool isHome(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock);

		// private functions
        //void resetPID(int i = NUM_DOF);
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);                 // modify angle from ODE for endcaps to count continuously
		void build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);	// build body of mobot
		void build_center(dReal x, dReal y, dReal z, dMatrix3 R);						// build center
		void build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R);				// build endcap
		void create_fixed_joint(CRobot *attach, int face1, int face2);				// create fixed joint between modules
		void create_rotation_matrix(dMatrix3 R, dReal psi, dReal theta, dReal phi);		// get rotation matrix from euler angles
		void extract_euler_angles(dMatrix3 R, dReal &psi, dReal &theta, dReal &phi);	// get euler angles from rotation matrix
		static void* recordAngleThread(void *arg);
		static void* recordAnglesThread(void *arg);
		unsigned int diff_msecs(struct timespec t1, struct timespec t2);
		unsigned int current_msecs(struct timespec t);
	protected:
		double	_encoderResolution,
				_center_length, _center_width, _center_height, _center_radius, _center_offset,
				_body_length, _body_width, _body_height, _body_radius,
				_body_inner_width_left, _body_inner_width_right, _body_end_depth, _body_mount_center,
				_end_width, _end_height, _end_depth, _end_radius;
		dReal _maxJointVelocity[NUM_DOF];
		dReal _maxJointForce[NUM_DOF];
		int _type;
};

class CMobot : public CRobot4 {
	public:
		CMobot(void);
};

class CiMobot : public CRobot4 {
	public:
		CiMobot(void);
};

#endif  /* MOBOTSIM_H_ */