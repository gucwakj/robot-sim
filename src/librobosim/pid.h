#ifndef PID_H_
#define PID_H_

#include <ode/ode.h>
#include "config.h"

class PID {
	public:
		PID();
		~PID();
		void init(double kp, double ki, double kd, double error_thresh, double step_time);
		void restart();
		double update(double error);

	private:
		bool m_started;
		double	m_kp,				// proportional control constant
				m_ki,				// integral control constant
				m_kd,				// derivative control constant
				m_dt,				// derivative
				m_dt_inv,			// derivative inverse
				m_prev_error,		// previous error value
				m_error_thresh,		// threshold for error
				m_integral;			// integral value (for wind-up correction)
};

#endif	/* PID_H_ */
