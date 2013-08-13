#ifndef LINKBOT_H_
#define LINKBOT_H_

#include "robotsim.h"

#ifdef _CH_
#include <array.h>
class DLLIMPORT CLinkbotT {
#else
#include "config.h"
//#include "pid.h"
#include "graphics.h"
class DLLIMPORT CLinkbotT : virtual public CRobot {
#endif
	public:
#ifdef _CH_
		CLinkbotT();
#else
		CLinkbotT(int disabled = -1, int type = LINKBOTT);
#endif
		~CLinkbotT();

		int blinkLED(double delay, int num);
		int connect();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngles(double &angle1, double &angle2, double &angle3);
#ifdef _CH_
		int getJointAngleAverage(robotJointId_t id, double &angle, ...);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
#else
		int getJointAngleAverage(robotJointId_t id, double &angle, int numReadings = 10);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, int numReadings = 10);
#endif
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isConnected(void);
		int isMoving(void);
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
#ifdef _CH_
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, ...);
		int recordAngles(double time[:],
						 double angle1[:],
						 double angle2[:],
						 double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
#else
		int recordAngle(robotJointId_t id, double time[], double angle[], int num, double seconds, int shiftData = 1);
		int recordAngleBegin(robotJointId_t id,
							 robotRecordData_t &time,
							 robotRecordData_t &angle, double seconds, int shiftData = 1);
		int recordAngles(double time[], double angle1[],
										double angle2[],
										double angle3[], int num, double seconds, int shiftData = 1);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, int shiftData = 1);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, int shiftData = 1);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, int shiftData = 1);
#endif
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAnglesEnd(int &num);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
			CLinkbotT *robot;
			robotJointId_t id;
			int num;
			int msecs;
			double *time;
			double **ptime;
			double *angle1;
			double **pangle1;
			double *angle2;
			double **pangle2;
			double *angle3;
			double **pangle3;
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
		int _connected;
		int* _enabled;				// list of enabled motors
		double _radius;				// wheel radius
		double **_recording_angles[NUM_DOF];
		bool _recording[NUM_DOF];	// recording in progress
		bool _active[NUM_DOF];		// actively recording a new value
		int _recording_num[NUM_DOF];// recording data points
		bool _success[NUM_DOF];		// trigger for goal
		bool _seek[NUM_DOF];		// currently seeking goal?
		int _rgb[3];				// rgb of 'led'
		double _joint_safety_angle;
		double _joint_safety_time;
		double _offset[NUM_DOF];
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
		static void* motionDistanceThread(void *arg);
		static void* motionRollBackwardThread(void *arg);
		static void* motionRollForwardThread(void *arg);
		static void* motionTurnLeftThread(void *arg);
		static void* motionTurnRightThread(void *arg);
		static void* recordAngleThread(void *arg);
		static void* recordAngleBeginThread(void *arg);
		static void* recordAnglesThread(void *arg);
		static void* recordAnglesBeginThread(void *arg);
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root);
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
#endif // not _CH_
};

class DLLIMPORT CLinkbotTGroup {
	public:
		CLinkbotTGroup();
		virtual ~CLinkbotTGroup();

		int blinkLED(double delay, int num);
		int addRobot(CLinkbotT& robot);
#ifdef _CH_
		int addRobots(array CLinkbotT robots[], ...);
#else
		int addRobots(CLinkbotT robots[], int num);
#endif
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
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
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
		typedef struct robots_s {
			CLinkbotT *robot;
			struct robots_s *next;
		} *robots_t;
		double _d;
		int _i;
		int _motion;
		robots_t _robots;
		THREAD_T *_thread;

		static void* motionDistanceThread(void*);
		static void* motionRollBackwardThread(void*);
		static void* motionRollForwardThread(void*);
		static void* motionTurnLeftThread(void*);
		static void* motionTurnRightThread(void*);
#endif // not _CH_
};

#ifdef _CH_
class DLLIMPORT CLinkbotI {
	public:
		CLinkbotI();
		~CLinkbotI();

		int blinkLED(double delay, int num);
		int connect();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isConnected();
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointContinuousNB(robotJointId_t id, robotJointState_t dir);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
};
class DLLIMPORT CLinkbotIGroup {
	public:
		CLinkbotIGroup();
		virtual ~CLinkbotIGroup();

		int addRobot(CLinkbotI& robot);
		int addRobots(array CLinkbotI robots[], ...);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
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
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
};

class DLLIMPORT CLinkbotL {
	public:
		CLinkbotL();
		~CLinkbotL();

		int blinkLED(double delay, int num);
		int connect();
		int disconnect();
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getAccelerometerData(double &accel_x, double &accel_y, double &accel_z);
		int getBatteryVoltage(double &voltage);
		int getColorRGB(int &r, int &g, int &b);
		int getFormFactor(int &formFactor);
		int getID();
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isConnected();
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
		int recordAngle(robotJointId_t id, double time[:], double angle[:], int num, double seconds, ...);
		int recordAngleBegin(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, ...);
		int recordAngleEnd(robotJointId_t id, int &num);
		int recordAngles(double time[:], double angle1[:], double angle2[:], double angle3[:], int num, double seconds, ...);
		int recordAnglesBegin(robotRecordData_t &time,
							  robotRecordData_t &angle1,
							  robotRecordData_t &angle2,
							  robotRecordData_t &angle3, double seconds, ...);
		int recordAnglesEnd(int &num);
		int recordDistanceBegin(robotJointId_t id,
								robotRecordData_t &time,
								robotRecordData_t &distance, double radius, double seconds, ...);
		int recordDistanceEnd(robotJointId_t id, int &num);
		int recordDistancesBegin(robotRecordData_t &time,
								 robotRecordData_t &distance1,
								 robotRecordData_t &distance2,
								 robotRecordData_t &distance3, double radius, double seconds, ...);
		int recordDistancesEnd(int &num);
		int recordWait();
		int reset();
		int resetToZero();
		int resetToZeroNB();
		int setBuzzerFrequency(int frequency, double time);
		int setBuzzerFrequencyOn(int frequency);
		int setBuzzerFrequencyOff();
		int setColorRGB(int r, int g, int b);
		int setExitState(robotJointState_t exitState);
		int setJointMovementStateNB(robotJointId_t id, robotJointState_t dir);
		int setJointMovementStateTime(robotJointId_t id, robotJointState_t dir, double seconds);
		int setJointSafetyAngle(double angle);
		int setJointSafetyAngleTimeout(double seconds);
		int setJointSpeed(robotJointId_t id, double speed);
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
};
class DLLIMPORT CLinkbotLGroup {
	public:
		CLinkbotLGroup();
		virtual ~CLinkbotLGroup();

		int addRobot(CLinkbotL& robot);
		int addRobots(array CLinkbotL robots[], ...);
		int blinkLED(double delay, int num);
		int connect(void);
		int driveJointTo(robotJointId_t id, double angle);
		int driveJointToDirect(robotJointId_t id, double angle);
		int driveJointToDirectNB(robotJointId_t id, double angle);
		int driveJointToNB(robotJointId_t id, double angle);
		int driveTo(double angle1, double angle2, double angle3);
		int driveToDirect(double angle1, double angle2, double angle3);
		int driveToDirectNB(double angle1, double angle2, double angle3);
		int driveToNB(double angle1, double angle2, double angle3);
		int getJointAngle(robotJointId_t id, double &angle);
		int getJointAngleAverage(robotJointId_t id, double &angle, ... );
		int getJointAngles(double &angle1, double &angle2, double &angle3);
		int getJointAnglesAverage(double &angle1, double &angle2, double &angle3, ...);
		int getJointMaxSpeed(robotJointId_t id, double &maxSpeed);
		int getJointSafetyAngle(double &angle);
		int getJointSafetyAngleTimeout(double &seconds);
		int getJointSpeed(robotJointId_t id, double &speed);
		int getJointSpeedRatio(robotJointId_t id, double &ratio);
		int getJointSpeeds(double &speed1, double &speed2, double &speed3);
		int getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
		int getJointState(robotJointId_t id, robotJointState_t &state);
		int isMoving();
		int motionDistance(double distance, double radius);
		int motionDistanceNB(double distance, double radius);
		int motionRollBackward(double angle);
		int motionRollBackwardNB(double angle);
		int motionRollForward(double angle);
		int motionRollForwardNB(double angle);
		int motionTurnLeft(double angle);
		int motionTurnLeftNB(double angle);
		int motionTurnRight(double angle);
		int motionTurnRightNB(double angle);
		int motionWait();
		int move(double angle1, double angle2, double angle3);
		int moveNB(double angle1, double angle2, double angle3);
		int moveBackward(double angle);
		int moveBackwardNB(double angle);
		int moveContinuousNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int moveContinuousTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int moveDistance(double distance, double radius);
		int moveDistanceNB(double distance, double radius);
		int moveForward(double angle);
		int moveForwardNB(double angle);
		int moveJoint(robotJointId_t id, double angle);
		int moveJointNB(robotJointId_t id, double angle);
		int moveJointTo(robotJointId_t id, double angle);
		int moveJointToDirect(robotJointId_t id, double angle);
		int moveJointToDirectNB(robotJointId_t id, double angle);
		int moveJointToNB(robotJointId_t id, double angle);
		int moveJointWait(robotJointId_t id);
		int moveTo(double angle1, double angle2, double angle3);
		int moveToDirect(double angle1, double angle2, double angle3);
		int moveToDirectNB(double angle1, double angle2, double angle3);
		int moveToNB(double angle1, double angle2, double angle3);
		int moveToZero();
		int moveToZeroNB();
		int moveWait();
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
		int setJointSpeeds(double speed1, double speed2, double speed3);
		int setJointSpeedRatio(robotJointId_t id, double ratio);
		int setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
		int setMotorPower(robotJointId_t id, int power);
		int setMovementStateNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3);
		int setMovementStateTime(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
		int setMovementStateTimeNB(robotJointState_t dir1, robotJointState_t dir2, robotJointState_t dir3, double seconds);
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
};
#ifndef ROBOTSIM_DLHANDLE
#define ROBOTSIM_DLHANDLE
void* CRobotSim::g_chrobotsim_dlhandle = NULL;
int CRobotSim::g_chrobotsim_dlcount = 0;
#pragma importf "chrobotsim.chf"
#endif
#pragma importf "chlinkbottsim.chf"
#pragma importf "chlinkbotisim.chf"
#pragma importf "chlinkbotlsim.chf"
#else
typedef struct motionArg_s {
	int i;
	double d;
	CLinkbotT *robot;
} motionArg_t;
class DLLIMPORT CLinkbotI : public CLinkbotT {
	public:
		CLinkbotI(void) : CLinkbotT(1, LINKBOTI) {}
};
class DLLIMPORT CLinkbotL : public CLinkbotT {
	public:
		CLinkbotL(void) : CLinkbotT(2, LINKBOTL) {}
};
class DLLIMPORT CLinkbotIGroup : public CLinkbotTGroup {};
class DLLIMPORT CLinkbotLGroup : public CLinkbotTGroup {};
extern CRobotSim _simObject;
#endif // _CH_

#endif // LINKBOT_H_
