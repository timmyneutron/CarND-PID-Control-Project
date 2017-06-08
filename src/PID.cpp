#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// initialize gains
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	// initialize parameter array
	p_[0] = &Kp_;
	p_[1] = &Ki_;
	p_[2] = &Kd_;

	// initialize delta values for parameters
	dp_[0] = 0.1;
	dp_[1] = 0.1;
	dp_[2] = 1.0;

	// initialize error
	p_error_ = 0;
	i_error_ = 0;
	d_error_ = 0;

	// initialize previous cte to calculate d_error
	prev_cte_ = 0;

	// initialize total error
	err_ = 0;
	best_err_ = 1e6;

	// start timer
	orig_time_ = chrono::high_resolution_clock::now();
	prev_time_ = orig_time_;

	// flag for starting twiddle algorithm
	twiddle_is_init_ = false;

	// variables to cycle through and modify gains
	which_K_ = 0;
	up_or_down_ = 0;
}

void PID::UpdateError(double cte) {
	// log time since last measurement
	time_ = std::chrono::high_resolution_clock::now();
	chrono::duration<double> d_time = chrono::duration_cast<chrono::duration<double>>(time_ - prev_time_);
	prev_time_ = time_;

	//update errors
	p_error_ = cte;
	i_error_ += cte * d_time.count();
	d_error_ = (cte - prev_cte_) / d_time.count();

	// update total error
	err_ += fabs(i_error_);

	// update previous cte
	prev_cte_ = cte;
}

double PID::Steer() {
	// set steer value from errors
	double steer_value = -(Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_);

	// if steer value is outside the range [-1, 1],
	// set equal to -1 or 1
	if (steer_value < -1.0)
	{
		steer_value = -1.0;
	}
	else if (steer_value > 1.0)
	{
		steer_value = 1.0;
	}

	return steer_value;
}

double PID::Throttle() {
	// set throttle
	double throttle = 0.5 + .1 * (50 - speed_);

	return throttle;
}

void PID::Twiddle() {
	// set time since last reset of Twiddle algorithm
	time_ = std::chrono::high_resolution_clock::now();
	chrono::duration<double> d_time = chrono::duration_cast<chrono::duration<double>>(time_ - orig_time_);

	// print out information
	cout << endl;
	cout << "Time: " << d_time.count() << endl;
	cout << "Speed: " << speed_ << endl;
	cout << "Kp: " << Kp_ << endl;
	cout << "Ki: " << Ki_ << endl;
	cout << "Kd: " << Kd_ << endl;
	cout << "CTE: " << p_error_ << endl;
	cout << "Current error: " << err_ << endl;
	cout << "Best error: " << best_err_ << endl;

	// wait 10 seconds for car to get up to speed
	if (!twiddle_is_init_ && d_time.count() < 10.0)
	{
		return;
	}
	// start twiddle algorithm
	else if (!twiddle_is_init_)
	{
		twiddle_is_init_ = true;
		orig_time_ = chrono::high_resolution_clock::now();
		// *p_[which_K_] += dp_[which_K_];
		return;
	}
	// reset algorithm if total error is greater than current best error,
	// or if time elapsed is over 30 seconds
	else if (err_ > best_err_ || d_time.count() > 30.0)
	{
		// if error is lower than current best error,
		// set new values for best gains and update
		// delta values and shift to next gain
		if (err_ < best_err_)
		{
			best_err_ = err_;
			best_Kp_ = Kp_;
			best_Ki_ = Ki_;
			best_Kd_ = Kd_;
			dp_[which_K_] *= 1.1;
			which_K_ = (which_K_ + 1) % 3;
			*p_[which_K_] += dp_[which_K_];
			up_or_down_ = 0;
		}
		else
		{
			// if error is higher than current best error,
			// try higher or lower than current gain
			// or shift to next gain
			if (up_or_down_ == 0)
			{
				*p_[which_K_] -= 2 * dp_[which_K_];
				up_or_down_ = 1;
			}
			else if (up_or_down_ == 1)
			{
				*p_[which_K_] += dp_[which_K_];
				dp_[which_K_] *= 0.9;
				which_K_ = (which_K_ + 1) % 3;
				*p_[which_K_] += dp_[which_K_];
				up_or_down_ = 0;
			}
		}
		// reset total error
		err_ = 0;

		// reset time
		orig_time_ = time_;
	}
}