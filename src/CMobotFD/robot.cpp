#include "robot.h"

void robotSim::simThreadsAngleInit(void) {
	pthread_mutex_init(&angle_mutex, NULL);
}

void robotSim::simThreadsAngleLock(void) {
	pthread_mutex_lock(&(this->angle_mutex));
}

void robotSim::simThreadsAngleUnlock(void) {
	pthread_mutex_unlock(&(this->angle_mutex));
}

void robotSim::simThreadsSuccessInit(void) {
	pthread_mutex_init(&success_mutex, NULL);
	pthread_cond_init(&success_cond, NULL);
}

void robotSim::simThreadsSuccessLock(void) {
	pthread_mutex_lock(&(this->success_mutex));
}

void robotSim::simThreadsSuccessUnlock(void) {
	pthread_mutex_unlock(&(this->success_mutex));
}

void robotSim::simThreadsSuccessSignal(void) {
	pthread_cond_signal(&(this->success_cond));
}

void robotSim::simThreadsSuccessWait(void) {
	pthread_cond_wait(&(this->success_cond), &(this->success_mutex));
}

int robotSim::simThreadsRWInit(pthread_rw_t *rwp, pthread_rwattr_t *attrp) {
	rwp->lock = 0;
	pthread_mutex_init(&(rwp->mutex), NULL);
	pthread_cond_init(&(rwp->cond), NULL);
	return 0;
}

int robotSim::simThreadsRWRLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int robotSim::simThreadsRWWLock(pthread_rw_t *rwp) {
	pthread_mutex_lock(&(rwp->mutex));
	while (rwp->lock) {
		pthread_cond_wait(&(rwp->cond), &(rwp->mutex));
	}
	rwp->lock = true;
	pthread_mutex_unlock(&(rwp->mutex));
	return 0;
}

int robotSim::simThreadsRWRUnlock(pthread_rw_t *rwp) {
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

int robotSim::simThreadsRWWUnlock(pthread_rw_t *rwp) {
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

int robotSim::simThreadsGoalInit(pthread_rwattr_t *attrp) { simThreadsRWInit(&(this->goal_rwlock), attrp); }
int robotSim::simThreadsGoalRLock(void) { simThreadsRWRLock(&(this->goal_rwlock)); }
int robotSim::simThreadsGoalRUnlock(void) { simThreadsRWRUnlock(&(this->goal_rwlock)); }
int robotSim::simThreadsGoalWLock(void) { simThreadsRWWLock(&(this->goal_rwlock)); }
int robotSim::simThreadsGoalWUnlock(void) { simThreadsRWWUnlock(&(this->goal_rwlock)); }

