/*
 * obsv_kalmanIntegration.h
 *
 *  Created on: 13.10.2018
 *      Author: Nu
 */

#ifndef INCLUDES_OBSV_KALMANINTEGRATION_H_
#define INCLUDES_OBSV_KALMANINTEGRATION_H_

#include "ctrl_tilt.h"
#include <math.h>

void filterKalman(double& udot, double& vdot, double& wdot, double& roll_meas,
				 double& roll_est, double& pitch_meas, double& pitch_est,
				 double& bias_p, double& bias_q, double& p, double& q);


//int filterKalman(double* states, double* outputMeasured, double* ctrlInput, double* outputEstimated);

#endif /* INCLUDES_OBSV_KALMANINTEGRATION_H_ */
