#ifndef ROBOT_H_
#define ROBOT_H_

#include <ode/ode.h>
#include <unistd.h>
#include <pthread.h>
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

enum robot_types_e {
	IMOBOT,
	MOBOT,
	KIDBOT,
	NXT,
	NUM_TYPES
};

class robot4Sim;

class robotSim {
	public:
		// entry functions for pre- and post-collision updates
		static void* simPreCollisionThreadEntry(void *arg);
		static void* simPostCollisionThreadEntry(void *arg);

		// threading functions to control variables
		void simThreadsAngleInit(void);
		void simThreadsAngleLock(void);
		void simThreadsAngleUnlock(void);
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
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi) = 0;
		virtual void build(dReal x, dReal y, dReal z, dReal psi, dReal theta, dReal phi, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached00(robot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached10(robot4Sim *attach, int face1, int face2) = 0;
		virtual void buildAttached01(robot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual void buildAttached11(robot4Sim *attach, int face1, int face2, dReal r_le, dReal r_lb, dReal r_rb, dReal r_re) = 0;
		virtual dReal getAngle(int i) = 0;
		virtual bool getSuccess(int i) = 0;
		virtual dReal getPosition(int body, int i) = 0;
		virtual dReal getRotation(int body, int i) = 0;
		virtual dBodyID getBodyID(int body) = 0;
		virtual dJointID getMotorID(int motor) = 0;
		virtual bool isHome(void) = 0;
		virtual void simAddRobot(dWorldID &world, dSpaceID &space) = 0;
		virtual void simPreCollisionThread(void) = 0;
		virtual void simPostCollisionThread(void) = 0;
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
};

#endif /* ROBOT_H_ */
