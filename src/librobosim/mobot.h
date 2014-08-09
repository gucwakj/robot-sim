#ifndef MOBOT_H_
#define MOBOT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CMobot : virtual public Robot {
		friend class mobotNodeCallback;

	// public api
	public:
		CMobot();
		~CMobot();

		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForeverNB(void);
		int driveForward(double);
		int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int drivexy(double, double, double, double);
		int drivexyNB(double, double, double, double);
		int drivexyTo(double, double, double, double);
		int drivexyToNB(double, double, double, double);
		int getJointAngles(double&, double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&, double&);
		int getJointSpeeds(double&, double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&, double&);
		int jumpTo(double, double, double, double);
		int jumpToNB(double, double, double, double);
		int motionArch(double);
		int motionArchNB(double);
		int motionDistance(double, double);
		int motionDistanceNB(double, double);
		int motionInchwormLeft(int);
		int motionInchwormLeftNB(int);
		int motionInchwormRight(int);
		int motionInchwormRightNB(int);
		int motionRollBackward(double);
		int motionRollBackwardNB(double);
		int motionRollForward(double);
		int motionRollForwardNB(double);
		int motionSkinny(double);
		int motionSkinnyNB(double);
		int motionStand(void);
		int motionStandNB(void);
		int motionTumbleLeft(int);
		int motionTumbleLeftNB(int);
		int motionTumbleRight(int);
		int motionTumbleRightNB(int);
		int motionTurnLeft(double);
		int motionTurnLeftNB(double);
		int motionTurnRight(double);
		int motionTurnRightNB(double);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double, double, double, double);
		int moveNB(double, double, double, double);
		int moveTo(double, double, double, double);
		int moveToNB(double, double, double, double);
		int recordAngles(double[], double[], double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int reset(void);
		int setJointSpeeds(double, double, double, double);
		int setJointSpeedRatios(double, double, double, double);
		int setSpeed(double, double);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// inherited functions
    private:
		virtual int build(xml_robot_t);
		virtual int build(xml_robot_t, Robot*, xml_conn_t);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int);
		virtual int getConnectionParams(int, dMatrix3, double*);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
		int add_connector(int, int, double);							// add connector
		int build_bigwheel(conn_t, int);								// build big wheel connector
		int build_body(int, double, double, double, dMatrix3, double);	// build body of mobot
		int build_caster(conn_t, int);									// build caster connector
		int build_center(double, double, double, dMatrix3);				// build center body of mobot
		int build_endcap(int, double, double, double, dMatrix3);		// build endcap of mobot
		int build_simple(conn_t, int);									// build simple connector
		int build_smallwheel(conn_t, int);								// build small wheel connector
		int build_square(conn_t, int);									// build square connector
		int build_tank(conn_t, int);									// build tank connector
		int build_wheel(conn_t, int, double);							// build custom wheel connector
#ifdef ENABLE_GRAPHICS
		void draw_bigwheel(conn_t, osg::Group*);						// draw big wheel
		void draw_caster(conn_t, osg::Group*);							// draw caster
		void draw_simple(conn_t, osg::Group*);							// draw simple
		void draw_smallwheel(conn_t, osg::Group*);						// draw small wheel
		void draw_square(conn_t, osg::Group*);							// draw square
		void draw_tank(conn_t, osg::Group*);							// draw tank
		void draw_wheel(conn_t, osg::Group*);							// draw custom wheel
#endif // ENABLE_GRAPHICS
		int fix_body_to_connector(dBodyID, int);						// fix second body to connector
		int fix_connector_to_body(int, dBodyID);						// fix connector to robot body
		int get_connector_params(int, int, dMatrix3, double*);			// get params of connector face
		static void* drivexyThread(void*);								// thread for driving
		static void* drivexyToThread(void*);							// thread for driving to a global point
		static void* motionArchThread(void*);							// thread for arching
		static void* motionDistanceThread(void*);						// thread for driving a distance
		static void* motionInchwormLeftThread(void*);					// thread for inchworming left
		static void* motionInchwormRightThread(void*);					// thread for inchworming right
		static void* motionRollBackwardThread(void*);					// thread for rolling backward
		static void* motionRollForwardThread(void*);					// thread for rolling forward
		static void* motionSkinnyThread(void*);							// thread for skinny driving
		static void* motionStandThread(void*);							// thread for standing
		static void* motionTurnLeftThread(void*);						// thread for turning left
		static void* motionTurnRightThread(void*);						// thread for turning right
		static void* motionTumbleLeftThread(void*);						// thread for for tumbling left
		static void* motionTumbleRightThread(void*);					// thread for tumbling right
		static void* motionUnstandThread(void*);						// thread for unstanding

	// private data
	private:
		// robot body parts
		enum robot_pieces_e {
			CENTER,
			ENDCAP_L,
			BODY_L,
			BODY_R,
			ENDCAP_R,
			NUM_PARTS
		};

		// dimensions
		double	_body_end_depth,
				_body_inner_width_left,
				_body_inner_width_right,
				_body_mount_center,
				_center_length,
				_center_height,
				_center_offset,
				_center_radius,
				_center_width,
				_end_depth,
				_end_height,
				_end_radius,
				_end_width,
				_tank_height,
				_tank_depth;
};

class CMobotGroup {
	public:
		CMobotGroup();
		virtual ~CMobotGroup();

		int addRobot(CMobot& robot);
		int addRobots(CMobot robots[], int num);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);
		int jumpJointToNB(robotJointId_t id, double angle);
		int jumpTo(double angle1, double angle2, double angle3, double angle4);
		int jumpToNB(double angle1, double angle2, double angle3, double angle4);
		int motionArch(double angle);
		int motionArchNB(double angle);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionInchwormLeftNB(int num);
		int motionInchwormRight(int num);
		int motionInchwormRightNB(int num);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForwardNB(double angle);
		int motionSkinny(double angle);
		int motionSkinnyNB(double angle);
		int motionRollForward(double angle);
		int motionInchwormLeft(int num);
		int motionStand(void);
		int motionStandNB(void);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionTumbleRight(int num);
		int motionTumbleRightNB(int num);
		int motionTumbleLeft(int num);
		int motionTumbleLeftNB(int num);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double angle1, double angle2, double angle3, double angle4);
		int moveNB(double angle1, double angle2, double angle3, double angle4);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointForeverNB(robotJointId_t id);
		int moveJointTime(robotJointId_t id, double seconds);
		int moveJointTimeNB(robotJointId_t id, double seconds);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTime(double seconds);
		int moveTimeNB(double seconds);
		int moveTo(double angle1, double angle2, double angle3, double angle4);
		int moveToNB(double angle1, double angle2, double angle3, double angle4);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointPower(robotJointId_t id, int power);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double angle);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setSpeed(double speed, double radius);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
	private:
		typedef struct robots_s {
			CMobot *robot;
			struct robots_s *next;
		} *robots_t;
		robots_t _robots;
		int _i;
		int _motion;
		double _d;
		THREAD_T *_thread;

		static void* motionArchThread(void *arg);
		static void* motionDistanceThread(void *arg);
		static void* motionInchwormLeftThread(void *arg);
		static void* motionInchwormRightThread(void *arg);
		static void* motionRollBackwardThread(void *arg);
		static void* motionRollForwardThread(void *arg);
		static void* motionSkinnyThread(void *arg);
		static void* motionStandThread(void *arg);
		static void* motionTurnLeftThread(void *arg);
		static void* motionTurnRightThread(void *arg);
		static void* motionTumbleRightThread(void *arg);
		static void* motionTumbleLeftThread(void *arg);
		static void* motionUnstandThread(void *arg);
};

// global structs for threading
typedef struct mobotMotionArg_s {
	int i;
	double d;
	CMobot *robot;
} mobotMotionArg_t;
typedef struct mobotMoveArg_s {
	double x, y, radius, trackwidth;
	CMobot *robot;
} mobotMoveArg_t;

// simulation
extern RoboSim *g_sim;

#endif  // MOBOT_H_

