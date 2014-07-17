#include "base.h"

CRobot::CRobot(void) {
	MUTEX_INIT(&_active_mutex);
	COND_INIT(&_active_cond);
	MUTEX_INIT(&_goal_mutex);
	MUTEX_INIT(&_motion_mutex);
	COND_INIT(&_motion_cond);
	MUTEX_INIT(&_recording_mutex);
	COND_INIT(&_recording_cond);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);
	MUTEX_INIT(&_theta_mutex);

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
	delete [] _body;
	delete [] _enabled;
	delete [] _geom;
	delete [] _joint;
	delete [] _motor;
	delete [] _rec_active;
	delete [] _rec_angles;
	delete [] _rec_num;
	delete [] _recording;

	// destroy mutexes
	MUTEX_DESTROY(&_active_mutex);
	COND_DESTROY(&_active_cond);
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_motion_mutex);
	COND_DESTROY(&_motion_cond);
	MUTEX_DESTROY(&_recording_mutex);
	COND_DESTROY(&_recording_cond);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
	MUTEX_DESTROY(&_theta_mutex);
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

// generate gaussian random number
double CRobot::normal(double sigma) {
	// compute pair of random uniform data
	double u1 = this->uniform();
	double u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*M_PI*u2));
}

// generate uniform random numbers
double CRobot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
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

void CRobot::doze(double ms) {
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms*1000);
#endif
}

int CRobot::addConnector(int type, int face, double size) {
	// create new connector
	conn_t nc = new struct conn_s;
	nc->d_side = -1;
	nc->d_type = -1;
	nc->face = face;
	nc->type = type;
	nc->next = NULL;

	// add to list of connectors
	conn_t ctmp = _conn;
	if ( _conn == NULL )
		_conn = nc;
	else {
		while (ctmp->next)
			ctmp = ctmp->next;
		ctmp->next = nc;
	}

	// build connector
	switch (type) {
		case BIGWHEEL:
			this->build_bigwheel(nc, face);
			break;
		case BRIDGE:
			this->build_bridge(nc, face);
			break;
		case CASTER:
			this->build_caster(nc, face);
			break;
		case CUBE:
			this->build_cube(nc, face);
			break;
		case FACEPLATE:
			this->build_faceplate(nc, face);
			break;
		case GRIPPER:
			this->build_gripper(nc, 1);
			break;
		case OMNIDRIVE:
			this->build_omnidrive(nc, face);
			break;
		case SIMPLE:
			this->build_simple(nc, face);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(nc, face);
			break;
		case SQUARE:
			this->build_square(nc, face);
			break;
		case TANK:
			this->build_tank(nc, face);
			break;
		case TINYWHEEL:
			this->build_tinywheel(nc, face);
			break;
		case WHEEL:
			this->build_wheel(nc, face, size);
			break;
	}

	if (type == GRIPPER) {
		conn_t nc2 = new struct conn_s;
		nc2->face = 3;
		nc2->type = GRIPPER;
		nc2->next = NULL;

		// add to list of connectors
		conn_t ctmp = _conn;
		while (ctmp->next)
			ctmp = ctmp->next;
		ctmp->next = nc2;

		// build
		this->build_gripper(nc2, 3);
	}

	// success
	return 0;
}

int CRobot::addToSim(dWorldID &world, dSpaceID &space) {
	_world = world;
    _space = dHashSpaceCreate(space);

	// success
	return 0;
}

