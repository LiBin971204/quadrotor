#include "ctrl_PD.h"

void mdl_control_lqr(struct bus_states quadState, struct bus_lqr ctrlTilt)
{
	double u1 = quadState.Pos_z_mtr * ctrlTilt.P+
				quadState.V_w_mtrPerScnd * ctrlTilt.D;

	double u2 = quadState.Ag_phi_rad * -2.0 +
				quadState.AgVel_p_radPerScnd * -5.0;

	double u3 = quadState.Ag_theta_rad * -2.0 +
				quadState.AgVel_q_radPerScnd * -5.0;
     
 	double u4 = 0;

 	// Matrix multiplication
}
