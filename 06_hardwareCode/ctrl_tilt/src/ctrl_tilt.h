#pragma once

#define sampleTime 0.004
#define PIN_CLK 6
#define PIN_MOSI 7
#define PIN_CS_G 8
#define PIN_CS_XM 9
#define PIN_MISO 10


struct bus_states {
	double Pos_z_mtr;
	double V_w_mtrPerScnd;
	double Ag_phi_rad;
	double AgVel_p_radPerScnd;
	double Ag_theta_rad;
	double AgVel_q_radPerScnd;
};

struct bus_lqr {
	double P;
	double D;
};

/*
double A[2][2] = { {1, sampleTime},
				   {0, 1} };
double B[2] = {sampleTime, 0};
double C[2] = {1, 0};
double L[2] = {0.0258, -0.0221};
*/
