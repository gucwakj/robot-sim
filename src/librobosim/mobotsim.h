#ifndef MOBOT_H_
#define MOBOT_H_

#include "robosim.h"

#ifdef _CH_
#include <array.h>
class DLLIMPORT CMobot {
#else
#include "config.h"
#ifdef ENABLE_GRAPHICS
#include "graphics.h"
//#include "pid.h"
#endif // ENABLE_GRAPHICS
class DLLIMPORT CMobot : virtual public CRobot {
#endif // _CH_
	public:
		CMobot();
		~CMobot();

		int blinkLED(double delay, int num);
		int connect(void);
		int disconnect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3, double angle4);
		int driveToDirect(double angle1, double angle2, double angle3, double angle4);
		int driveToDirectNB(double angle1, double angle2, double angle3, double angle4);
		int driveToNB(double angle1, double angle2, double angle3, double angle4);
		int getFormFactor(int &formFactor);
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
#ifdef _CH_
		int getJointAngleAverage(robotJointId_t id, double &angle, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, ...);
#else
		int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings=10);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings=10);
#endif
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isConnected();
		int isMoving();
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
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1,
							 robotJointState_t dir2,
							 robotJointState_t dir3,
							 robotJointState_t dir4);
		int moveContinuousTime(robotJointState_t dir1,
							   robotJointState_t dir2,
							   robotJointState_t dir3,
							   robotJointState_t dir4, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
		int moveJointContinuousTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3, double angle4);
		int moveToDirect(double angle1, double angle2, double angle3, double angle4);
		int moveToDirectNB(double angle1, double angle2, double angle3, double angle4);
		int moveToNB(double angle1, double angle2, double angle3, double angle4);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
#ifdef _CH_
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, ...);
		int recordAngles(double time[:],
						 double angle1[:],				
						 double angle2[:],
						 double angle3[:], 
						 double angle4[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3,
							  robotRecordData_t &angle4, double seconds, ...);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3,
								 robotRecordData_t &distance4, double radius, double seconds, ...);
#else
		int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, int shiftData = 1);
		int recordAngles(double time[],
						 double angle1[],
						 double angle2[],
						 double angle3[],
						 double angle4[], int num, double seconds, int shiftData = 1);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3,
							  robotRecordData_t &angle4, double seconds, int shiftData = 1);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3,
								 robotRecordData_t &distance4, double radius, double seconds, int shiftData = 1);
#endif
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordAnglesEnd(int &num);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1,
							   robotJointState_t dir2,
							   robotJointState_t dir3,
							   robotJointState_t dir4);
		int setMovementStateTime(robotJointState_t dir1,
								 robotJointState_t dir2,
								 robotJointState_t dir3,
								 robotJointState_t dir4, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1,
								   robotJointState_t dir2,
								   robotJointState_t dir3,
								   robotJointState_t dir4, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stop(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int stopAllJoints(void);
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
#ifndef _CH_
    private:
		enum robot_pieces_e {		// each body part which is built
			ENDCAP_L,
			BODY_L,
			CENTER,
			BODY_R,
			ENDCAP_R,
			NUM_PARTS
		};
		enum robot_bodies_e {		// each body which has a degree of freedom
			LE,
			LB,
			RB,
			RE,
			NUM_DOF
		};

		// private functions inherited from CRobot class
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock);
		virtual int build(bot_t robot);
		virtual int build(bot_t robot, CRobot *base, Conn_t *conn);
		virtual dReal getAngle(int i);
		virtual dBodyID getBodyID(int id);
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p);
		virtual dBodyID getConnectorBodyID(int face);
		virtual dBodyID getConnectorBodyIDs(int num);
		virtual int getRobotID(void);
		virtual dJointID getMotorID(int id);
		virtual dReal getPosition(int body, int i);
		virtual dReal getRotation(int body, int i);
		virtual bool getSuccess(int i);
		virtual int getType(void);
		virtual bool isHome(void);
		virtual int setID(int id);
		virtual void simPreCollisionThread(void);
		virtual void simPostCollisionThread(void);

		// private functions
		int add_connector(int type, int face);
		int build_individual(dReal x, dReal y, dReal z, dMatrix3 R, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re);
		int build_attached(bot_t robot, CRobot *base, Conn_t *conn);
		int build_body(int id, dReal x, dReal y, dReal z, dMatrix3 R, dReal theta);
		int build_center(dReal x, dReal y, dReal z, dMatrix3 R);
		int build_endcap(int id, dReal x, dReal y, dReal z, dMatrix3 R);
		int build_bigwheel(conn_t conn, int face);
		int build_caster(conn_t conn, int face);
		int build_simple(conn_t conn, int face);
		int build_smallwheel(conn_t conn, int face);
		int build_square(conn_t conn, int face);
		int build_tank(conn_t conn, int face);
		int fix_body_to_connector(dBodyID cBody, int face);
		int fix_connector_to_body(int face, dBodyID cBody);
		int get_connector_params(Conn_t *conn, dMatrix3 R, dReal *p);
		int init_params(void);
		int init_dims(void);
		dReal mod_angle(dReal past_ang, dReal cur_ang, dReal ang_rate);
		void resetPID(int i = NUM_DOF);
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
		static void* setJointMovementStateTimeNBThread(void *arg);
		static void* setMovementStateTimeNBThread(void *arg);
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root);
		void draw_bigwheel(conn_t conn, osg::Group *robot);
		void draw_caster(conn_t conn, osg::Group *robot);
		void draw_simple(conn_t conn, osg::Group *robot);
		void draw_smallwheel(conn_t conn, osg::Group *robot);
		void draw_square(conn_t conn, osg::Group *robot);
		void draw_tank(conn_t conn, osg::Group *robot);
#endif // ENABLE_GRAPHICS
#endif // not _CH_
};

class CMobotGroup {
	public:
		CMobotGroup();
		virtual ~CMobotGroup();

		int addRobot(CMobot& robot);
#ifdef _CH_
		int addRobots(array CMobot robots[], ...);
#else
		int addRobots(CMobot robots[], int num);
#endif
		int blinkLED(double delay, int num);
		int connect(void);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveToDirect(double angle1, double angle2, double angle3, double angle4);
		int driveTo(double angle1, double angle2, double angle3, double angle4);
		int driveToDirectNB(double angle1, double angle2, double angle3, double angle4);
		int driveToNB(double angle1, double angle2, double angle3, double angle4);
		int isMoving(void);
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
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3, double angle4);
		int moveToDirect(double angle1, double angle2, double angle3, double angle4);
		int moveToNB(double angle1, double angle2, double angle3, double angle4);
		int moveToDirectNB(double angle1, double angle2, double angle3, double angle4);
		int moveWait(void);
		int moveToZero(void);
		int moveToZeroNB(void);
		int reset(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointMovementStateTimeNB(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double angle);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3, double speed4);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4);
		int setMovementStateNB(robotJointState_t dir1,
							   robotJointState_t dir2,
							   robotJointState_t dir3,
							   robotJointState_t dir4);
		int setMovementStateTime(robotJointState_t dir1,
								 robotJointState_t dir2,
								 robotJointState_t dir3,
								 robotJointState_t dir4, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1,
								   robotJointState_t dir2,
								   robotJointState_t dir3,
								   robotJointState_t dir4, double seconds);
		int setTwoWheelRobotSpeed(double speed, double radius);
		int stopAllJoints(void);
		int stopOneJoint(robotJointId_t id);
		int stopTwoJoints(robotJointId_t id1, robotJointId_t id2);
		int stopThreeJoints(robotJointId_t id1, robotJointId_t id2, robotJointId_t id3);
		int turnLeft(double angle, double radius, double tracklength);
		int turnLeftNB(double angle, double radius, double tracklength);
		int turnRight(double angle, double radius, double tracklength);
		int turnRightNB(double angle, double radius, double tracklength);
#ifndef _CH_
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
#endif // not _CH_
};

typedef struct motionArg_s {
	int i;
	double d;
	CMobot *robot;
} motionArg_t;

#ifdef _CH_
#ifndef ROBOSIM_DLHANDLE
#define ROBOSIM_DLHANDLE
void* RoboSim::g_chrobosim_dlhandle = NULL;
int RoboSim::g_chrobosim_dlcount = 0;
#pragma importf "chrobosim.chf"
#endif
#pragma importf "chmobotsim.chf"
#else
extern RoboSim _simObject;
#endif

#endif  // MOBOT_H_
