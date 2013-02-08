#ifndef PID_H_
#define PID_H_

#include <ode/ode.h>
#include "config.h"

class PID {
	public:
		PID();
		~PID();
		void init(dReal kp, dReal ki, dReal kd, dReal error_thresh, dReal step_time);
		void restart();
		dReal update(dReal error);

	private:
		bool m_started;
		dReal	m_kp,				// proportional control constant
				m_ki,				// integral control constant
				m_kd,				// derivative control constant
				m_dt,				// derivative
				m_dt_inv,			// derivative inverse
				m_prev_error,		// previous error value
				m_error_thresh,		// threshold for error
				m_integral;			// integral value (for wind-up correction)
};

#endif	/* PID_H_ */