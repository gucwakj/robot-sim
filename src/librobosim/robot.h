#ifndef ROBOT_H_
#define ROBOT_H_

#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#include <Shlwapi.h>
#else
#include <pthread.h>
#include <unistd.h>
#endif // _WIN32

#include <ode/ode.h>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <ctime>
#include "config.h"
#include "macros.h"
#include "rgbhashtable.h"

#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#include <osg/ShapeDrawable>
#include "graphics.h"
#endif // ENABLE_GRAPHICS

// forward declare friends
class RoboSim;

class DLLIMPORT Robot {
		friend class RoboSim;

	// common public api
	public:
		Robot(void);
		virtual ~Robot(void);

		int blinkLED(double, int);
		int connect(char* = NULL, int = 3);
		int delay(double);
		int delaySeconds(double);
		int disableRecordDataShift(void);
		int disconnect(void);
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
		int drivexyToFunc(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncNB(double, double, int, double (*func)(double), double, double);
		int drivexyToPoly(double, double, int, char*, double, double);
		int drivexyToPolyNB(double, double, int, char*, double, double);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getBatteryVoltage(double&);
		int getDistance(double&, double);
		int getFormFactor(int&);
		int getID(void);
		int getJointAngle(robotJointId_t, double&, int = 10);
		int getJointAngleInstant(robotJointId_t, double&);
		int getJointMaxSpeed(robotJointId_t, double&);
		int getJointSafetyAngle(double&);
		int getJointSafetyAngleTimeout(double&);
		int getJointSpeed(robotJointId_t, double&);
		int getJointSpeedRatio(robotJointId_t, double&);
		int getxy(double&, double&);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t, double);
		int jumpJointToNB(robotJointId_t, double);
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
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(robotJointId_t, double[], double[], int, double, int = 1);
		int recordAngleBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordAngleEnd(robotJointId_t, int&);
		int recordAnglesEnd(int&);
		int recordDistanceBegin(robotJointId_t, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int recordDistanceEnd(robotJointId_t, int&);
		int recordDistanceOffset(double);
		int recordDistancesEnd(int&);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordxyEnd(int&);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointPower(robotJointId_t, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int systemTime(double&);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// condensed argument versions of function calls
	protected:
		int moveNB(double*);
		int moveToNB(double*);
		int recordAngles(double*, double**, int, double, int);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t*&, double, int = 1);

	// utility functions for inherited and friend classes
	protected:
		int addToSim(dWorldID&, dSpaceID&, int);
		int doze(double);
		int fixBodyToGround(dBodyID);
		dBodyID getBodyID(int);
		double getCenter(int);
		double getRotation(int, int);
		double mod_angle(double, double, double);
		int noisy(double*, int, double);
		static void* simPreCollisionThreadEntry(void*);
		static void* simPostCollisionThreadEntry(void*);

	// virual functions for inherited classes
	protected:
		virtual int build(xml_robot_t) {};
		virtual int buildIndividual(double, double, double, dMatrix3, double*) {};
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group*, int) {};
#endif // ENABLE_GRAPHICS
		virtual double getAngle(int) {};
		virtual int initParams(int, int) {};
		virtual int initDims(void) {};
		virtual void simPreCollisionThread(void) {};
		virtual void simPostCollisionThread(void) {};

	// data members
	protected:
		// motor motion directions
		typedef enum motor_state_e {
			NEUTRAL = 0,
			HOLD,
			POSITIVE,
			NEGATIVE,
		} motorState_t;
		// motor motion profiles
		typedef enum motor_mode_e {
			ACCEL_CONSTANT = 0,
			ACCEL_CYCLOIDAL,
			ACCEL_HARMONIC,
			CONTINUOUS,
			SEEK,
		} motorMode_t;

		// recording
		typedef struct recArg_s {
			Robot *robot;			// robot
			robotJointId_t id;		// joint to record
			int num;				// number of points
			int msecs;				// ms between data points
			double *time;			// array for time
			double **ptime;			// pointer to time array
			double **angle;			// array of angles
			double ***pangle;		// point to array of angles
		} recArg_t;
		// motor accelerations
		typedef struct accel_s {
			double init;			// motion initial angle
			double start;			// motion start time
			double period;			// motion period
			double run;				// number of motions
		} accel_t;
		// motor
		typedef struct motor_s {
			bool success;			// trigger for motion completion
			dJointID id;			// motors
			double alpha;			// angular acceleration
			double encoder;			// encoder resolution
			double goal;			// goal theta value
			double offset;			// offset from zero for resetting
			double omega;			// angular rate
			double omega_max;		// maximum rate
			double safety_angle;	// safety angle
			double safety_timeout;	// safety timeout
			double tau_max;			// maximum force
			double theta;			// theta
			int mode;				// modes
			int timeout;			// mode timeout
			int starting;			// starting movement
			int stopping;			// stopping movement
			int state;				// state
			accel_t accel;			// acceleration variables
			MUTEX_T success_mutex;	// motion successful mutex
			COND_T success_cond;	// motion successful condition
		} *motor_t;

		dBodyID *_body;				// body parts
		dGeomID **_geom;			// geometries of each body part
		dJointID *_joint;			// joints between body parts
		dSpaceID _space;			// space for this robot
		dWorldID _world;			// world for all robots
		motor_t _motor;				// motors
		bool _motion;				// motion in progress
		bool *_recording;			// recording in progress
		bool *_rec_active;			// actively recording a new value
		double _accel[3];			// accelerometer data
		double _bigwheel_radius;	// dimension: big wheel radius
		double _body_length;		// dimension: body length
		double _body_height;		// dimension: body height
		double _body_radius;		// dimension: body radius
		double _body_width;			// dimension: body width
		double _center[3];			// offset of body from initial (x,y,z)
		double _distOffset;			// offset for recorded distance
		double _radius;				// wheel radius
		double ***_rec_angles;		// recorded angles from thread
		double _rgb[3];				// rgb of 'led'
		double _trackwidth;			// trackwidth of robot
		double _wheel_depth;		// dimension: wheel depth
		double _wheel_radius;		// dimension: custom wheel radius
		int _connected;				// connected to controller
		int _disabled;				// which joint is disabled
		int _dof;					// number of DOF
		int *_enabled;				// list of enabled motors
		int _id;					// robot id
		int *_rec_num;				// recording data points
		int _seed;					// seed for random number generation
		int _shift_data;			// shift recorded data or not
		int _g_shift_data;			// globally shift data for robot
		int _g_shift_data_en;		// globally shift data for robot enable/disable flag
		int _trace;					// tracing on or off
		int _type;					// type of robot
#ifdef ENABLE_GRAPHICS
		osg::Group *_robot;			// all graphical objects for drawing
		osg::ShapeDrawable *_led;	// led object
#endif // ENABLE_GRAPHICS
		MUTEX_T _active_mutex;		// active recording
		COND_T _active_cond;		// active recording
		MUTEX_T _goal_mutex;		// goal value being written
		MUTEX_T _motion_mutex;		// motion in progress
		COND_T _motion_cond;		// motion in progress
		MUTEX_T _recording_mutex;	// recording data point
		COND_T _recording_cond;		// recording data  point
		MUTEX_T _success_mutex;		// completed step
		COND_T _success_cond;		// completed step
		MUTEX_T _theta_mutex;		// theta value being written

	// private functions
	private:
		bool is_shift_enabled(void);
		double normal(double);
		double uniform(void);
		static void* driveTimeNBThread(void*);
		static void* drivexyToThread(void*);
		static void* drivexyToFuncThread(void*);
		static void* drivexyToPolyThread(void*);
		static void* moveJointTimeNBThread(void*);
		static void* moveTimeNBThread(void*);
		static void* recordAngleThread(void*);
		static void* recordAngleBeginThread(void*);
		static void* recordAnglesThread(void*);
		static void* recordAnglesBeginThread(void*);
		static void* recordxyBeginThread(void*);
};

class DLLIMPORT RobotGroup {
	// public api
	public:
		RobotGroup(void);
		virtual ~RobotGroup(void);
		int addRobot(Robot&);
		int addRobots(Robot[], int);

		int blinkLED(double, int);
		int connect(void);
		int disconnect(void);
		int holdJoint(robotJointId_t);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isMoving(void);
		int isNotMoving(void);
		int jumpJointTo(robotJointId_t, double);
		int jumpJointToNB(robotJointId_t, double);
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
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int relaxJoint(robotJointId_t id);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setJointPower(robotJointId_t, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(robotJointId_t, double);
		int setJointSpeedRatio(robotJointId_t, double);
		int traceOff(void);
		int traceOn(void);
		int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	// data members
	private:
		typedef struct robots_s {
			Robot *robot;
			struct robots_s *next;
		} *robots_t;
		double _d;
		int _i;
		robots_t _robots;
		THREAD_T *_thread;
};

// global structs for threading
typedef struct moveArg_s {
	double x, y, radius, trackwidth;
	int i;
	double (*func)(double x);
	char *expr;
	Robot *robot;
} moveArg_t;

#endif // ROBOT_H_

