#ifndef LINKBOTSIM_H_
#define LINKBOTSIM_H_

#ifndef _CH_
#include "config.h"
#include "pid.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS
#include "robotsim.h"
#include "base.h"
#endif // not _CH_

#ifdef ENABLE_DOUBLE
#define EPSILON DBL_EPSILON
#else
#define EPSILON FLT_EPSILON
#endif // ENABLE_DOUBLE

typedef enum linkbot_faces_e {
	LINKBOT_FACE1 = 1,
	LINKBOT_FACE2,
	LINKBOT_FACE3,
} linkbotFaceID_t;
typedef enum linkbot_joints_e {
	LINKBOT_JOINT1,
	LINKBOT_JOINT2,
	LINKBOT_JOINT3,
} linkbotJointID_t;
typedef enum linkbot_joint_state_e {
	LINKBOT_NEUTRAL		= 0,
	LINKBOT_FORWARD		= 1,
	LINKBOT_BACKWARD	= 2,
	LINKBOT_HOLD		= 3
} linkbotJointState_t;
typedef enum linkbot_connector_e {
	L_BIGWHEEL,
	L_CASTER,
	L_CUBE,
	L_SIMPLE,
	L_SMALLWHEEL,
	L_TANK,
	L_NUM_CONNECTORS
} linkbotConnector_t;

#ifndef _CH_
class CLinkbot : virtual public CRobot {
	// public api to mimic CLinkbot class
	public:
		CLinkbot(int disabled = -1, int type = LINKBOT);
		~CLinkbot();

		int connect(void);
		int getJointAngle(int id, dReal &angle);
		int getJointSpeed(int id, double &speed);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int motionArch(dReal angle);
		int motionInchwormLeft(int num);
		int motionInchwormRight(int num);
		int motionRollBackward(dReal angle);
		int motionRollForward(dReal angle);
		int motionSkinny(dReal angle);
		int motionStand(void);
		int motionTumbleRight(int num);
		int motionTumbleLeft(int num);
		int motionTurnLeft(dReal angle);
		int motionTurnRight(dReal angle);
		int motionUnstand(void);
		int move(dReal angle1, dReal angle2, dReal angle3);
		int moveNB(dReal angle1, dReal angle2, dReal angle3);
		int moveJoint(int id, dReal angle);
		int moveJointNB(int id, dReal angle);
		int moveJointTo(int id, dReal angle);
		int moveJointToNB(int id, dReal angle);
		int moveJointWait(int id);
		int moveTo(dReal angle1, dReal angle2, dReal angle3);
		int moveToDirect(dReal angle1, dReal angle2, dReal angle3);
		int moveToNB(dReal angle1, dReal angle2, dReal angle3);
		//int moveToDirectNB(dReal angle1, dReal angle2, dReal angle3, dReal angle4);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
#ifndef _CH_
		int recordAngle(int id, dReal *time, dReal *angle, int num, dReal seconds, dReal threshold = 0.0);
		int recordAngles(dReal *time, dReal *angle1, dReal *angle2, dReal *angle3, int num, dReal seconds, dReal threshold = 0.0);
#else
		int recordAngle(int id, dReal time[:], dReal angle[:], int num, dReal seconds);
		int recordAngles(dReal time[:], dReal angle1[:], dReal angle2[:], dReal angle3[:], int num, dReal seconds);
#endif // not _CH_
		int recordWait(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointSpeed(int id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
    private:
		enum robot_pieces_e {       // each body part which is built
			BODY,
			FACE1,
			FACE2,
			FACE3,
			NUM_PARTS
		};
		enum robot_bodies_e {       // each body which has a degree of freedom
			F1,
			F2,
			F3,
			NUM_DOF
		};
		typedef struct recordAngleArg_s {
			CLinkbot *robot;
			int id;
			int num;
			int msecs;
			double *time;
			double *angle1;
			double *angle2;
			double *angle3;
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
		dJointID _joint[3];			// joints between body parts
		dReal* _clock;				// world clock
		dReal _angle[NUM_DOF];		// angles
		dReal _speed[NUM_DOF];		// speed
		dReal _goal[NUM_DOF];		// goals
		dReal _maxJointForce[NUM_DOF];
		dReal _maxSpeed[NUM_DOF];	// maximum joint speeds
		conn_t _conn;				// connectors
		//PID _pid[NUM_DOF];			// PID control for each joint
		int _id;					// robot id
		int _state[NUM_DOF];		// joint states
		int* _enabled;				// list of enabled motors
		bool _recording[NUM_DOF];	// recording in progress
		bool _success[NUM_DOF];		// trigger for goal
		double	_encoderResolution,
				_body_length, _body_width, _body_height, _body_radius,
				_face_depth, _face_radius;
		double	_connector_depth, _connector_height, _connector_radius,
				_bigwheel_radius, _smallwheel_radius;

		// private functions inherited from CRobot class
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock);
		virtual int build(bot_t robot);
		virtual int build(bot_t robot, CRobot *base, Conn_t *conn);
		virtual dReal getAngle(int i);
		virtual dBodyID getBodyID(int id);
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p);
		virtual dBodyID getConnectorBodyID(int face);
		virtual dBodyID getConnectorBodyIDs(int num);
		virtual int getID(void);
		virtual dJointID getMotorID(int id);
		virtual dReal getPosition(int body, int i);
		virtual dReal getRotation(int body, int i);
		virtual bool getSuccess(int i);
		virtual int getType(void);
		virtual bool isHome(void);
		virtual int setID(int id);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root);
#endif // ENABLE_GRAPHICS

		// private functions
		int add_connector(int type, int face);
		int build_individual(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_f1, dReal r_f2, dReal r_f3);
		int build_attached(bot_t robot, CRobot *base, Conn_t *conn);			// build rotated and attached robot
		int build_body(dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);		// build body of mobot
		int build_face(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);		// build body of mobot
		int build_bigwheel(conn_t conn, int face);								// build big wheel
		int build_caster(conn_t conn, int face);								// build caster
		int build_simple(conn_t conn, int face);								// build simple connector
		int build_smallwheel(conn_t conn, int face);							// build small wheel
		int build_square(conn_t conn, int face);								// build square connector
		int build_tank(conn_t conn, int face);									// build tank connector
		int fix_body_to_connector(dBodyID cBody, int face);				// create fixed joint between body and connector
		int fix_connector_to_body(int face, dBodyID cBody);				// create fixed joint between connector and body
		int get_connector_params(Conn_t *conn, dMatrix3 R, dReal *p);	// get parameters of connector
		int init_params(int disabled, int type);						// initialize robot parameters
		int init_dims(void);											// initialize robot dimensions
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);	// modify angle from ODE for endcaps to count continuously
        //void resetPID(int i = NUM_DOF);
		static void* record_angle_thread(void *arg);
		static void* record_angles_thread(void *arg);
#ifdef ENABLE_GRAPHICS
		void draw_bigwheel(conn_t conn, osg::Group *robot);
		void draw_caster(conn_t conn, osg::Group *robot);
		void draw_simple(conn_t conn, osg::Group *robot);
		void draw_smallwheel(conn_t conn, osg::Group *robot);
		void draw_square(conn_t conn, osg::Group *robot);
		void draw_tank(conn_t conn, osg::Group *robot);
#endif // ENABLE_GRAPHICS
	protected:
		int _disabled;				// which joint is disabled
		int _type;					// type of robot
};
#endif // not _CH_

#ifdef _CH_
class CLinkbotI {
	public:
		CLinkbotI();
		~CLinkbotI();
		int connect(void);
		int getJointAngle(int id, dReal &angle);
		int getJointSpeed(int id, double &speed);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int motionArch(dReal angle);
		int motionInchwormLeft(int num);
		int motionInchwormRight(int num);
		int motionRollBackward(dReal angle);
		int motionRollForward(dReal angle);
		int motionSkinny(dReal angle);
		int motionStand(void);
		int motionTumbleRight(int num);
		int motionTumbleLeft(int num);
		int motionTurnLeft(dReal angle);
		int motionTurnRight(dReal angle);
		int motionUnstand(void);
		int move(dReal angle1, dReal angle2, dReal angle3);
		int moveNB(dReal angle1, dReal angle2, dReal angle3);
		int moveJoint(int id, dReal angle);
		int moveJointNB(int id, dReal angle);
		int moveJointTo(int id, dReal angle);
		int moveJointToNB(int id, dReal angle);
		int moveJointWait(int id);
		int moveTo(dReal angle1, dReal angle2, dReal angle3);
		int moveToDirect(dReal angle1, dReal angle2, dReal angle3);
		int moveToNB(dReal angle1, dReal angle2, dReal angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(int id, dReal time[:], dReal angle[:], int num, dReal seconds);
		int recordAngles(dReal time[:], dReal angle1[:], dReal angle2[:], dReal angle3[:], int num, dReal seconds);
		int recordWait(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointSpeed(int id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
};
#else
class CLinkbotI : public CLinkbot {
	public:
		CLinkbotI(void) : CLinkbot(1, LINKBOTI) {}
};
#endif // _CH_

#ifdef _CH_
class CLinkbotL {
	public:
		CLinkbotL();
		~CLinkbotL();
		int connect(void);
		int getJointAngle(int id, dReal &angle);
		int getJointSpeed(int id, double &speed);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int motionArch(dReal angle);
		int motionInchwormLeft(int num);
		int motionInchwormRight(int num);
		int motionRollBackward(dReal angle);
		int motionRollForward(dReal angle);
		int motionSkinny(dReal angle);
		int motionStand(void);
		int motionTumbleRight(int num);
		int motionTumbleLeft(int num);
		int motionTurnLeft(dReal angle);
		int motionTurnRight(dReal angle);
		int motionUnstand(void);
		int move(dReal angle1, dReal angle2, dReal angle3);
		int moveNB(dReal angle1, dReal angle2, dReal angle3);
		int moveJoint(int id, dReal angle);
		int moveJointNB(int id, dReal angle);
		int moveJointTo(int id, dReal angle);
		int moveJointToNB(int id, dReal angle);
		int moveJointWait(int id);
		int moveTo(dReal angle1, dReal angle2, dReal angle3);
		int moveToDirect(dReal angle1, dReal angle2, dReal angle3);
		int moveToNB(dReal angle1, dReal angle2, dReal angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(int id, dReal time[:], dReal angle[:], int num, dReal seconds);
		int recordAngles(dReal time[:], dReal angle1[:], dReal angle2[:], dReal angle3[:], int num, dReal seconds);
		int recordWait(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointSpeed(int id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
};
#else
class CLinkbotL : public CLinkbot {
	public:
		CLinkbotL(void) : CLinkbot(2, LINKBOTL) {}
};
#endif // _CH_

#endif // LINKBOTSIM_H_
