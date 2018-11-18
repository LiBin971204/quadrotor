#include "obsv_kalmanIntegration.h"

void filterKalman(double& udot, double& vdot, double& wdot, double& roll_meas,
				 double& roll_est, double& pitch_meas, double& pitch_est,
				 double& bias_p, double& bias_q, double& p, double& q){
	// Inclinometer
	roll_meas = atan2(-vdot, -wdot);
	double buff1 = sqrt( vdot*vdot + wdot*wdot );
	pitch_meas = atan2(udot, buff1);

	// Kalman filter phi
	double delta_roll = roll_meas - roll_est;
	roll_est += sampleTime*(p-bias_p) + 0.0258*delta_roll;
	bias_p += -0.0221*delta_roll;

	// Kalman filter theta
	double delta_pitch = pitch_meas - pitch_est;
	pitch_est += sampleTime*(q-bias_q) + 0.0258*delta_pitch;
	bias_q += -0.0221*delta_pitch;
}

/*
int filterKalman(double* states, double* outputMeasured, double* ctrlInput, double* outputEstimated){
	xk1 = A*states + B*ctrlInput + L*(outputEstimated-outputMeasured);
	outputEstimated = C*xk1;
	return 0;
}
*/
