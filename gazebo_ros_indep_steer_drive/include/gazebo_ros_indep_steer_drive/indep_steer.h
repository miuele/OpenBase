#ifndef INDEP_STEER_H_INCLUDED
#define INDEP_STEER_H_INCLUDED

#include <cmath>

namespace indep_steer {

inline constexpr double pi = 3.1415926535;

struct unicycle_state {
	double theta, v;
};

double angle_plus_delta(double theta, double delta) {
	return std::fmod(theta + (delta < 0 ? 2*pi + delta : delta), 2*pi);
}

void closest_state_realization(unicycle_state &realization, double angle, double magnitude, const unicycle_state &current_state) {
	double delta = std::remainder(angle - current_state.theta, 2*pi);
	if (std::abs(delta) > pi/2) {
		delta += pi;
		realization.v = -magnitude;
	} else {
		realization.v = magnitude;
	}
	realization.theta = angle_plus_delta(current_state.theta, delta);
}

double clerp(double a, double b, double t) {
	double delta = std::remainder(b - a, 2*pi);
	return angle_plus_delta(a, delta * t);
}

}

#endif
