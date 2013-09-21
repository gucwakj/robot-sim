#include "pid.h"

using namespace std;

PID::PID() {
	// Initialize controller parameters
	this->m_kp = 0;
	this->m_ki = 0;
	this->m_kd = 0;
	this->m_error_thresh = 0;

	// Controller step time and its inverse
	this->m_dt = 0.01;
	this->m_dt_inv = 1 / 0.01;

	// Initialize integral and derivative calculations
	this->m_integral = 0;
	this->m_started = false;
}

PID::~PID() {
}

void PID::init(double kp, double ki, double kd, double error_thresh, double step_time) {
	// initialize controller parameters
	this->m_kp = kp;
	this->m_ki = ki;
	this->m_kd = kd;
	this->m_error_thresh = error_thresh;

	// controller step time and its inverse
	this->m_dt = step_time;
	this->m_dt_inv = 1 / step_time;

	// initialize integral and derivative calculations
	this->m_integral = 0;
	this->m_started = false;
}

void PID::restart() {
	// initialize integral and derivative calculations
	this->m_integral = 0;
	this->m_started = false;
}

double PID::update(double error) {
	// update the error integral if the error magnitude is below the threshold
	if (fabs(error) < m_error_thresh)
		m_integral += this->m_dt*error;

	// compute the error derivative
	double deriv = 0;
	if (!this->m_started)
		this->m_started = true;
	else
		deriv = this->m_dt_inv * (error - this->m_prev_error);

	// store error for next step
	this->m_prev_error = error;

	// return the PID controller actuator command
	return this->m_kp*(error + this->m_ki*this->m_integral + this->m_kd*deriv);
}
