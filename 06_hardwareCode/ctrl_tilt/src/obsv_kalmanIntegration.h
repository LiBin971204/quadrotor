#pragma once

#include "ctrl_tilt.h"
#include <math.h>

void filterKalman(double& udot, double& vdot, double& wdot, double& roll_meas,
				 double& roll_est, double& pitch_meas, double& pitch_est,
				 double& bias_p, double& bias_q, double& p, double& q);


//int filterKalman(double* states, double* outputMeasured, double* ctrlInput, double* outputEstimated);
