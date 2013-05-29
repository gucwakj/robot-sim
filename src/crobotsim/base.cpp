#include "base.h"

void* CRobot::simPreCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPreCollisionThread();
}

void* CRobot::simPostCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPostCollisionThread();
}

int CRobot::simThreadsRWInit(pthread_rw_t *rwp) {
	rwp->lock = 0;
	pthread_mutex_init(&(rwp->mutex), NULL);
	pthread_cond_init(&(rwp->cond), NULL);
	return 0;
}

int CRobot::simThreadsRWRLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int CRobot::simThreadsRWWLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int CRobot::simThreadsRWRUnlock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	if (!rwp->lock) {
		pthread_mutex_unlock(&(rwp->mutex));
		return -1;
	}
	else {
		rwp->lock = false;
		pthread_cond_signal(&(rwp->cond));
		pthread_mutex_unlock(&(rwp->mutex));
		return 0;
	}
}

int CRobot::simThreadsRWWUnlock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	if (!rwp->lock) {
		pthread_mutex_unlock(&(rwp->mutex));
		return -1;
	}
	else {
		rwp->lock = false;
		pthread_cond_broadcast(&(rwp->cond));
		pthread_mutex_unlock(&(rwp->mutex));
		return 0;
	}
}

int CRobot::simThreadsGoalInit(void) { simThreadsRWInit(&_goal_rwlock); }
int CRobot::simThreadsGoalRLock(void) { simThreadsRWRLock(&_goal_rwlock); }
int CRobot::simThreadsGoalRUnlock(void) { simThreadsRWRUnlock(&_goal_rwlock); }
int CRobot::simThreadsGoalWLock(void) { simThreadsRWWLock(&_goal_rwlock); }
int CRobot::simThreadsGoalWUnlock(void) { simThreadsRWWUnlock(&_goal_rwlock); }

