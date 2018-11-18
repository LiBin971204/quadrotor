/*
 * ctrl_PD.h
 *
 *  Created on: 13.10.2018
 *      Author: Nu
 */

#ifndef CTRL_PD_H_
#define CTRL_PD_H_

#include "ctrl_tilt.h"

void mdl_control_lqr(struct bus_states quadState, struct bus_lqr ctrlTilt);


#endif /* CTRL_PD_H_ */
