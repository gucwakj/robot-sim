#ifndef MOBOT_H_
#define MOBOT_H_

#include "robosim.h"
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
#endif // ENABLE_GRAPHICS

#define NUM_DOF 4

class DLLIMPORT CMobot : virtual public CRobot {
	public:
		CMobot();
		~CMobot();

		int blinkLED(double delay, int num);
		int connect(char *name = NULL, int pause = 3);
		int delay(double milliseconds);
		int delaySeconds(double seconds);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveBackward(double angle);
		int driveBackwardNB(double angle);
		int driveDistance(double distance, double radius);
		int driveDistanceNB(double distance, double radius);
		int driveForeverNB(void);
		int driveForward(double angle);
		int driveForwardNB(double angle);
		int driveTime(double seconds);
		int driveTimeNB(double seconds);
		int drivexy(double x, double y, double radius, double trackwidth);
		int drivexyNB(double x, double y, double radius, double trackwidth);
		int drivexyTo(double x, double y, double radius, double trackwidth);
		int drivexyToNB(double x, double y, double radius, double trackwidth);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getDistance(double &distance, double radius);
		int getFormFactor(int &formFactor);
		int getJointAngle(robotJointId_t id, double &angle, int numReadings=10);
		int getJointAngleInstant(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings=10);
		int getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
		int getxy(double &x, double &y);
		int holdJoint(robotJointId_t id);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t id, double angle);
		int jumpJointToNB(robotJointId_t id, double angle);
		int jumpTo(double angle1, double angle2, double angle3, double angle4);
		int jumpToNB(double angle1, double angle2, double angle3, double angle4);
#ifdef ENABLE_GRAPHICS
		int line(double x1, double y1, double z1, double x2, double y2, double z2, int linewidth, char *color);
#endif // ENABLE_GRAPHICS
		int motionArch(double angle);
		int motionArchNB(double angle);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionInchwormLeft(int num);
		int motionInchwormLeftNB(int num);
		int motionInchwormRight(int num);
		int motionInchwormRightNB(int num);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionSkinny(double angle);
		int motionSkinnyNB(double angle);
		int motionStand(void);
		int motionStandNB(void);
		int motionTumbleLeft(int num);
		int motionTumbleLeftNB(int num);
		int motionTumbleRight(int num);
		int motionTumbleRightNB(int num);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
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
#ifdef ENABLE_GRAPHICS
		int point(double x, double y, double z, int pointsize, char *color);
#endif // ENABLE_GRAPHICS
		int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData = 1);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[], double angle1[], double angle2[], double angle3[], double angle4[], int num, double seconds, int shiftData = 1);
		int recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, robotRecordData_t &angle4, double seconds, int shiftData = 1);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistanceOffset(double distance);
		int recordDistancesBegin(robotRecordData_t &t, robotRecordData_t &d1, robotRecordData_t &d2, robotRecordData_t &d3, robotRecordData_t &d4, double radius, double seconds, int shiftData = 1);
		int recordDistancesEnd(int &num);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int recordTrace = 0, int shiftData = 1);
		int recordxyEnd(int &num);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int reset(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setJointPower(robotJointId_t id, int power);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int systemTime(double &time);
#ifdef ENABLE_GRAPHICS
		int text(double x, double y, double z, char *text);
#endif // ENABLE_GRAPHICS
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double angle, double radius, double trackwidth);
		int turnLeftNB(double angle, double radius, double trackwidth);
		int turnRight(double angle, double radius, double trackwidth);
		int turnRightNB(double angle, double radius, double trackwidth);
    private:
		enum robot_pieces_e {		// each body part which is built
			ENDCAP_L,
			BODY_L,
			CENTER,
			BODY_R,
			ENDCAP_R,
			NUM_PARTS
		};

		// private functions inherited from CRobot class
		virtual int addToSim(dWorldID &world, dSpaceID &space);
		virtual int build(xml_robot_t robot);
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn);
		virtual double getAngle(int i);
		virtual double getAngularRate(int i);
		virtual dBodyID getBodyID(int id);
		virtual double getCenter(int i);
		virtual int getConnectionParams(int face, dMatrix3 R, double *p);
		virtual dBodyID getConnectorBodyID(int face);
		virtual dBodyID getConnectorBodyIDs(int num);
		virtual int getRobotID(void);
		virtual dJointID getMotorID(int id);
		virtual double getPosition(int body, int i);
		virtual double getRotation(int body, int i);
		virtual bool getSuccess(int i);
		virtual int getType(void);
		virtual bool isHome(void);
		virtual int isShiftEnabled(void);
		virtual int setID(int id);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

		// private functions
		int add_connector(int type, int face, double size);
		int build_individual(double x, double y, double z, dMatrix3 R, double r_le, double r_lb, double r_rb, double r_re);
		int build_attached(xml_robot_t robot, CRobot *base, xml_conn_t conn);
		int build_body(int id, double x, double y, double z, dMatrix3 R, double theta);
		int build_center(double x, double y, double z, dMatrix3 R);
		int build_endcap(int id, double x, double y, double z, dMatrix3 R);
		int build_bigwheel(conn_t conn, int face);
		int build_caster(conn_t conn, int face);
		int build_simple(conn_t conn, int face);
		int build_smallwheel(conn_t conn, int face);
		int build_square(conn_t conn, int face);
		int build_tank(conn_t conn, int face);
		int build_wheel(conn_t conn, int face, double size);
		int fix_body_to_connector(dBodyID cBody, int face);
		int fix_connector_to_body(int face, dBodyID cBody);
		int get_connector_params(xml_conn_t conn, dMatrix3 R, double *p);
		int init_params(void);
		int init_dims(void);
		double mod_angle(double past_ang, double cur_ang, double ang_rate);
		static void* drivexyThread(void *arg);
		static void* drivexyToThread(void *arg);
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
		static void* recordAngleThread(void *arg);
		static void* recordAngleBeginThread(void *arg);
		static void* recordAnglesThread(void *arg);
		static void* recordAnglesBeginThread(void *arg);
		static void* recordxyBeginThread(void *arg);
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking);
		void draw_bigwheel(conn_t conn, osg::Group *robot);
		void draw_caster(conn_t conn, osg::Group *robot);
		void draw_simple(conn_t conn, osg::Group *robot);
		void draw_smallwheel(conn_t conn, osg::Group *robot);
		void draw_square(conn_t conn, osg::Group *robot);
		void draw_tank(conn_t conn, osg::Group *robot);
		void draw_wheel(conn_t conn, osg::Group *robot);
#endif // ENABLE_GRAPHICS
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
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
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

typedef struct mobotMotionArg_s {
	int i;
	double d;
	CMobot *robot;
} mobotMotionArg_t;
typedef struct mobotMoveArg_s {
	double x, y, radius, trackwidth;
	CMobot *robot;
} mobotMoveArg_t;

extern RoboSim *g_sim;

#endif  // MOBOT_H_
