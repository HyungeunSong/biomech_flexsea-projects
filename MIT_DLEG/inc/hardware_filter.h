/*
 * mit_filters.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Seong Ho Yeon
 */

//****************************************************************************
// Include(s)
//****************************************************************************
#include "arm_math.h"
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "user-mn-MIT-DLeg.h"
#include "walking_state_machine.h"
#include "state_variables.h"

//****************************************************************************
// Definitions
//****************************************************************************

#ifndef BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_
#define BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_

//#define LPF1 // Passband 100Hz, Stopband 200Hz
//#define LPF2 // Passband 50Hz, Stopband 100Hz
#define LPF3 // Passband 35Hz, Stopband 70Hz
//#define LPF4 // Passband 35Hz, Stopband 70Hz

extern uint16_t lpf_index;

//float32_t lpf_out;
void  initLPF(void);
void  updateLPF(float val);
float filterLPF(float val);



#endif /* BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_ */

