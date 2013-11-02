#include "base.h"

CRobot::CRobot(void) {
	MUTEX_INIT(&_angle_mutex);
	MUTEX_INIT(&_goal_mutex);
	MUTEX_INIT(&_motion_mutex);
	COND_INIT(&_motion_cond);
	MUTEX_INIT(&_recording_mutex);
	COND_INIT(&_recording_cond);
	MUTEX_INIT(&_active_mutex);
	COND_INIT(&_active_cond);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);

	_seed = time(NULL);
}

CRobot::~CRobot(void) {
	// destroy connectors array
	conn_t ctmp = _conn;
	while (ctmp) {
		conn_t tmp = ctmp->next;
		delete [] ctmp->geom;
		delete ctmp;
		ctmp = tmp;
	}

	// delete all arrays
	delete [] _angle;
	delete [] _body;
	delete [] _enabled;
	delete [] _geom;
	delete [] _goal;
	delete [] _joint;
	delete [] _max_force;
	delete [] _max_speed;
	delete [] _motor;
	delete [] _offset;
	delete [] _rec_active;
	delete [] _rec_angles;
	delete [] _rec_num;
	delete [] _recording;
	delete [] _seek;
	delete [] _speed;
	delete [] _state;
	delete [] _success;

	// destroy mutexes
	MUTEX_DESTROY(&_angle_mutex);
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_motion_mutex);
	COND_DESTROY(&_motion_cond);
	MUTEX_DESTROY(&_recording_mutex);
	COND_DESTROY(&_recording_cond);
	MUTEX_DESTROY(&_active_mutex);
	COND_DESTROY(&_active_cond);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
}

void* CRobot::simPreCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPreCollisionThread();
	return arg;
}

void* CRobot::simPostCollisionThreadEntry(void *arg) {
	CRobot *p = (CRobot *)arg;
	p->simPostCollisionThread();
	return arg;
}

// generate uniform random numbers
double CRobot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
}

// generate gaussian random number
double CRobot::normal(double sigma) {
	// compute pair of random uniform data
	double u1 = this->uniform();
	double u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*M_PI*u2));
}

int CRobot::noisy(double *a, int length, double sigma) {
	// initialize variables
	double *rand = new double[length];
	double sum = 0;

	if (length == 1)
		a[0] += this->normal(sigma);
	else {
		// compute magnitude of randomized vector
		for (int i = 0; i < length; i++) {
			rand[i] = this->normal(sigma);
			sum += (a[i] + rand[i]) * (a[i] + rand[i]);
		}
		double mag = sqrt(sum);
	
		// normalize vector
		for (int i = 0; i < length; i++) {
			a[i] = (a[i] + rand[i])/mag;
		}
	}

	// clean up array
	delete [] rand;

	// success
	return 0;
}

