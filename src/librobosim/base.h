#ifndef BASE_H_
#define BASE_H_

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
#include "rgbhashtable.h"
#include "config.h"
#include "macros.h"

#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#include <osg/ShapeDrawable>
#include "graphics.h"
#endif // ENABLE_GRAPHICS

// connector
typedef struct conn_s {
	int face, type;
	int d_side, d_type;
	dBodyID body;
	dGeomID *geom;
	struct conn_s *next;
} *conn_t;

class DLLIMPORT CRobot {
	public:
		CRobot(void);
		~CRobot(void);

		// public functions
		dBodyID getBodyID(int body);
		dBodyID getConnectorBodyID(int face);
		dBodyID getConnectorBodyIDs(int num);
		dJointID getMotorID(int motor);
		double getAngle(int id);
		double getCenter(int i);
		double getNormal(double sigma);
		double getRotation(int body, int i);
		double getUniform(void);
		int addToSim(dWorldID &world, dSpaceID &space);
		int doze(double ms);
		int fixBodyToGround(dBodyID cbody);
		int getConnectorParams(int type, int side, dMatrix3 R, double *p);
		int getRobotID(void);
		int getType(void);
		int isShiftEnabled(void);
		int noisy(double *a, int length, double sigma);
		int setID(int id);
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int build(xml_robot_t robot) = 0;
		virtual int build(xml_robot_t robot, CRobot *base, xml_conn_t conn) = 0;
		virtual int buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) = 0;
#ifdef ENABLE_GRAPHICS
		virtual int draw(osg::Group *root, int tracking) = 0;
#endif // ENABLE_GRAPHICS
		virtual int getConnectionParams(int face, dMatrix3 R, double *p) = 0;
		virtual int initParams(int disabled, int type) = 0;
		virtual int initDims(void) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;

		typedef enum robot_joint_state_e {
			NEUTRAL = 0,
			HOLD,
			POSITIVE,
			NEGATIVE,
		} robotJointState_t;
		typedef enum robot_motor_mode_e {
			ACCEL_CONSTANT = 0,
			ACCEL_CYCLOIDAL,
			ACCEL_HARMONIC,
			CONTINUOUS,
			SEEK,
		} robotMotorMode_t;

		// recording
		typedef struct recordAngleArg_s {
			CRobot *robot;
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
			double *angle4;
			double **pangle4;
		} recordAngleArg_t;
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
			MUTEX_T success_mutex;
			COND_T success_cond;
		} *motor_t;

		conn_t _conn;			// connectors
		dBodyID *_body;			// body parts
		dGeomID **_geom;		// geometries of each body part
		dJointID *_joint;		// joints between body parts
		dSpaceID _space;		// space for this robot
		dWorldID _world;		// world for all robots
		motor_t _motor;			// motors
		bool _motion;			// motion in progress
		bool *_recording;		// recording in progress
		bool *_rec_active;		// actively recording a new value
		double _accel[3];		// accelerometer data
		double _center[3];		// offset of body from initial (x,y,z)
		double _distOffset;		// offset for recorded distance
		double _radius;			// wheel radius
		double ***_rec_angles;	// recorded angles from thread
		double _rgb[3];			// rgb of 'led'
		double _trackwidth;		// trackwidth of robot
		double	_center_length, _center_width, _center_height, _center_radius, _center_offset,
				_body_length, _body_width, _body_height, _body_radius,
				_body_inner_width_left, _body_inner_width_right, _body_end_depth, _body_mount_center,
				_end_width, _end_height, _end_depth, _end_radius,
				_face_depth, _face_radius;
		double	_connector_depth, _connector_height, _connector_radius,
				_bigwheel_radius, _smallwheel_radius, _wheel_depth, _wheel_radius,
				// mobot
				_tank_height, _tank_depth,
				// linkbot
				_bridge_length, _cubic_length, _omni_length, _tinywheel_radius;
		int _connected;			// connected to controller
		int _disabled;			// which joint is disabled
		int _dof;				// number of DOF
		int *_enabled;			// list of enabled motors
		int _id;				// robot id
		int *_rec_num;			// recording data points
		int _seed;				// seed for random number generation
		int _shift_data;		// shift recorded data or not
		int _g_shift_data;		// globally shift data for robot
		int _g_shift_data_en;	// globally shift data for robot enable/disable flag
		int _trace;				// tracing on or off
		int _type;				// type of robot

		// threading locks for each robot
		MUTEX_T _active_mutex;
		COND_T _active_cond;
		MUTEX_T _goal_mutex;
		MUTEX_T _motion_mutex;
		COND_T _motion_cond;
		MUTEX_T _recording_mutex;
		COND_T _recording_cond;
		MUTEX_T _success_mutex;
		COND_T _success_cond;
		MUTEX_T _theta_mutex;

#ifdef ENABLE_GRAPHICS
		osg::Group *_robot;
		osg::ShapeDrawable *_led;
#endif // ENABLE_GRAPHICS
	private:
		double mod_angle(double past_ang, double cur_ang, double ang_rate);
};
#endif // BASE_H_
