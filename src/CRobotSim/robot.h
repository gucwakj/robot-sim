#ifndef ROBOT_H_
#define ROBOT_H_

#include "config.h"
#include <ode/ode.h>
#include <unistd.h>
#include <pthread.h>
#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#endif /* ENABLE_GRAPHICS */
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// types of robots available for simulation
enum robot_types_e {
	IMOBOT,
	MOBOT,
	//KIDBOT,
	//NXT,
	NUM_TYPES
};

// connector
typedef struct Conn_s {
	int robot, type, c_face, face1, face2;
	struct Conn_s *next;
} Conn_t;
// robot
typedef struct bot_s {
	int type;
	int id;
	double x, y, z;
	double psi, theta, phi;
	double angle1, angle2, angle3, angle4;
	struct Conn_s *conn;
	struct bot_s *next;
} *bot_t;

// classes forward declared
class CRobot4;

class CRobot {
	public:
		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// threading functions to control variables
		void simThreadsAngleInit(void);
		void simThreadsAngleLock(void);
		void simThreadsAngleUnlock(void);
		void simThreadsRecordingInit(void);
		void simThreadsRecordingLock(void);
		void simThreadsRecordingUnlock(void);
		void simThreadsRecordingSignal(void);
		void simThreadsRecordingWait(void);
		void simThreadsSuccessInit(void);
		void simThreadsSuccessLock(void);
		void simThreadsSuccessUnlock(void);
		void simThreadsSuccessSignal(void);
		void simThreadsSuccessWait(void);
		int simThreadsGoalInit(void);
		int simThreadsGoalRLock(void);
		int simThreadsGoalRUnlock(void);
		int simThreadsGoalWLock(void);
		int simThreadsGoalWUnlock(void);		

		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock) = 0;
		virtual int build(bot_t robot) = 0;
		virtual int build(bot_t robot, CRobot *base, Conn_t *conn) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual int getID(void) = 0;
		virtual int getType(void) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dBodyID getConnectorBodyID(int face) = 0;
		virtual int getConnectionParams(int face, dMatrix3 R, dReal *p) = 0;
		virtual dJointID getMotorID(int motor) = 0;
		virtual dReal getAngle(int i) = 0;
		virtual dReal getPosition(int body, int i) = 0;
		virtual dReal getRotation(int body, int i) = 0;
		virtual bool isHome(void) = 0;
		virtual int setID(int id) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;
#ifdef ENABLE_GRAPHICS
		virtual void draw(osg::Group *root) = 0;
#endif // ENABLE_GRAPHICS
	private:
		// single access read/write lock
		typedef struct rw_var {
			bool lock;
			pthread_mutex_t mutex;
			pthread_cond_t cond;
		} pthread_rw_t;

		// pthread single access r/w lock functions
		int simThreadsRWInit(pthread_rw_t *rwp);
		int simThreadsRWRLock(pthread_rw_t *rwp);
		int simThreadsRWRUnlock(pthread_rw_t *rwp);
		int simThreadsRWWLock(pthread_rw_t *rwp);
		int simThreadsRWWUnlock(pthread_rw_t *rwp);

		// threading locks for each robot
		pthread_mutex_t _angle_mutex;
		pthread_rw_t _goal_rwlock;
		pthread_mutex_t _success_mutex;
		pthread_cond_t _success_cond;
		pthread_mutex_t _recording_mutex;
		pthread_cond_t _recording_cond;
};

#endif /* ROBOT_H_ */
