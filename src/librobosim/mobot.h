#ifndef MOBOT_H_
#define MOBOT_H_

#include "config.h"
#include "robosim.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

class DLLIMPORT CMobot : public ModularRobot {
		friend class mobotNodeCallback;

	// public api
	public:
		CMobot();
		~CMobot();

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

	// inherited functions from ModularRobot class
	private:
		virtual int addConnector(int, int, double);
		virtual int build(xml_robot_t, dMatrix3, double*, dBodyID, xml_conn_t);
#ifdef ENABLE_GRAPHICS
		virtual int drawConnector(conn_t, osg::Group*);
#endif // ENABLE_GRAPHICS
		virtual int fixBodyToConnector(dBodyID, int);
		virtual int fixConnectorToBody(int, dBodyID, int = -1);
		virtual int getConnectorParams(int, int, dMatrix3, double*);
		virtual int getFaceParams(int, dMatrix3, double*);

	// inherited functions from Robot class
	private:
		virtual int build(xml_robot_t);
		virtual int buildIndividual(double, double, double, dMatrix3, double*);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int);
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int);
		virtual int initParams(int, int);
		virtual int initDims(void);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

	// private functions
	private:
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
	// public api
	public:
		CMobotGroup();
		virtual ~CMobotGroup();
		int addRobot(CMobot&);
		int addRobots(CMobot[], int);

		int blinkLED(double, int);
		int connect(void);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForeverNB(void);
		int driveForward(double);
		int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t, double);
		int jumpJointToNB(robotJointId_t, double);
		int jumpTo(double, double, double, double);
		int jumpToNB(double, double, double, double);
		int motionArch(double);
		int motionArchNB(double);
		int motionDistance(double, double);
		int motionDistanceNB(double, double);
		int motionInchwormLeftNB(int);
		int motionInchwormRight(int);
		int motionInchwormRightNB(int);
		int motionRollBackward(double);
		int motionRollBackwardNB(double);
		int motionRollForwardNB(double);
		int motionSkinny(double);
		int motionSkinnyNB(double);
		int motionRollForward(double);
		int motionInchwormLeft(int);
		int motionStand(void);
		int motionStandNB(void);
		int motionTurnLeft(double);
		int motionTurnLeftNB(double);
		int motionTurnRight(double);
		int motionTurnRightNB(double);
		int motionTumbleRight(int);
		int motionTumbleRightNB(int);
		int motionTumbleLeft(int);
		int motionTumbleLeftNB(int);
		int motionUnstand(void);
		int motionUnstandNB(void);
		int motionWait(void);
		int move(double, double, double, double);
		int moveNB(double, double, double, double);
		int moveForeverNB(void);
		int moveJoint(robotJointId_t, double);
		int moveJointNB(robotJointId_t, double);
		int moveJointForeverNB(robotJointId_t);
		int moveJointTime(robotJointId_t, double);
		int moveJointTimeNB(robotJointId_t, double);
		int moveJointTo(robotJointId_t, double);
		int moveJointToNB(robotJointId_t, double);
		int moveJointWait(robotJointId_t);
		int moveTime(double);
		int moveTimeNB(double);
		int moveTo(double, double, double, double);
		int moveToNB(double, double, double, double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t);
		int reset(void);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointPower(robotJointId_t, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeeds(double, double, double, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int setJointSpeedRatios(double, double, double, double);
		int setSpeed(double, double);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// private functions
	private:
		static void* motionArchThread(void*);
		static void* motionDistanceThread(void*);
		static void* motionInchwormLeftThread(void*);
		static void* motionInchwormRightThread(void*);
		static void* motionRollBackwardThread(void*);
		static void* motionRollForwardThread(void*);
		static void* motionSkinnyThread(void*);
		static void* motionStandThread(void*);
		static void* motionTurnLeftThread(void*);
		static void* motionTurnRightThread(void*);
		static void* motionTumbleRightThread(void*);
		static void* motionTumbleLeftThread(void*);
		static void* motionUnstandThread(void*);

	// private data
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

