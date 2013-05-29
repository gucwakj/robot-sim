#ifndef BASE_H_
#define BASE_H_

#include <unistd.h>

#ifndef _CH_
#include "config.h"
#include <ode/ode.h>
#include <pthread.h>
#ifdef ENABLE_GRAPHICS
#include <osg/Group>
#endif // ENABLE_GRAPHICS
#endif // no _CH_

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

#ifdef _WIN32
//   MUTEX
#define MUTEX_T HANDLE
#define MUTEX_INIT(mutex) *mutex = CreateMutex(NULL, FALSE, NULL)
#define MUTEX_DESTROY(mutex)
#define MUTEX_LOCK(mutex) WaitForSingleObject(*mutex, INFINITE)
#define MUTEX_UNLOCK(mutex) ReleaseMutex(*mutex)
//   COND
#define COND_T HANDLE
#define COND_INIT(cond) *cond = CreateEvent(NULL, TRUE, TRUE, NULL); ResetEvent(*cond)
#define COND_DESTROY(cond)
#define COND_WAIT(cond , mutex) ResetEvent(*cond); ReleaseMutex(*mutex); WaitForSingleObject(*cond, INFINITE)
#define SIGNAL(cond, mutex, action) action; SetEvent(*cond)
#define COND_SIGNAL(cond) SetEvent(*cond)
#else
//   MUTEX
#define MUTEX_T pthread_mutex_t
#define MUTEX_INIT(mutex) pthread_mutex_init(mutex, NULL)
#define MUTEX_DESTROY(mutex) pthread_mutex_destroy(mutex)
#define MUTEX_LOCK(mutex) \
	if (pthread_mutex_lock(mutex)) { \
		fprintf(stderr, "pthread lock error: %s:%d\n", __FILE__, __LINE__); \
	}
#define MUTEX_UNLOCK(mutex) pthread_mutex_unlock(mutex)
//   COND
#define COND_T pthread_cond_t
#define COND_INIT(cond) pthread_cond_init(cond, NULL)
#define COND_DESTROY(cond) pthread_cond_destroy(cond)
#define COND_WAIT(cond, mutex) pthread_cond_wait(cond, mutex)
#define SIGNAL(cond, mutex, action) pthread_mutex_lock(mutex); action; pthread_cond_signal(cond); pthread_mutex_unlock(mutex)
#define COND_SIGNAL(cond) pthread_cond_signal(cond)
#endif

// types of robots available for simulation
enum robot_types_e {
	MOBOT,
	LINKBOT,
	LINKBOTI,
	LINKBOTL,
	NUM_TYPES
};

// connector
typedef struct Conn_s {
	int robot, type, side, face1, face2;
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

class CRobot {
	public:
		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// threading functions to control variables
		/*void simThreadsRecordingInit(void);
		void simThreadsRecordingLock(void);
		void simThreadsRecordingUnlock(void);
		void simThreadsRecordingSignal(void);
		void simThreadsRecordingWait(void);*/
		/*void simThreadsSuccessInit(void);
		void simThreadsSuccessLock(void);
		void simThreadsSuccessUnlock(void);
		void simThreadsSuccessSignal(void);
		void simThreadsSuccessWait(void);*/
		int simThreadsGoalInit(void);
		int simThreadsGoalRLock(void);
		int simThreadsGoalRUnlock(void);
		int simThreadsGoalWLock(void);
		int simThreadsGoalWUnlock(void);		

#ifndef _CH_
		// pure virtual functions to be overridden by inherited classes of each robot
		virtual int addToSim(dWorldID &world, dSpaceID &space, dReal *clock) = 0;
		virtual int build(bot_t robot) = 0;
		virtual int build(bot_t robot, CRobot *base, Conn_t *conn) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual int getID(void) = 0;
		virtual int getType(void) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dBodyID getConnectorBodyID(int face) = 0;
		virtual dBodyID getConnectorBodyIDs(int num) = 0;
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

		// threading locks for each robot
		MUTEX_T _angle_mutex;
		//pthread_mutex_t _recording_mutex;
		MUTEX_T _recording_mutex;
		//pthread_cond_t _recording_cond;
		COND_T _recording_cond;
		//pthread_mutex_t _success_mutex;
		MUTEX_T _success_mutex;
		//pthread_cond_t _success_cond;
		COND_T _success_cond;
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
		pthread_rw_t _goal_rwlock;
		//RWLOCK_T _goal_rwlock;
#endif
};

#endif // BASE_H_
