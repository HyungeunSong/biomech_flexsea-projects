/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user_ankle_2dof: 2-DoF Ankle Functions
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_DLEG
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_MIT_DLEG
#define INC_MIT_DLEG

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************


//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_MIT_DLeg(void);
void MIT_DLeg_fsm_1(void);
void MIT_DLeg_fsm_2(void);

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
int8_t safetyFailure(void);
int8_t findPoles(void);
float getJointAngle(void);
float getAxialForce(void);
float getLinkageMomentArm(float);
float getJointTorque(void);
void openSpeedFSM(void);
void twoPositionFSM(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees
#define IS_ANKLE
//#define IS_KNEE
#define DEVICE_TF08_A01			// Define specific actuator configuration.
//#define DEVICE_TF08_A02			// Define specific actuator configuration.


#ifdef DEVICE_TF08_A01
#define JOINT_ZERO_OFFSET 	0 		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// CW = 1 (knee orientation), CCW = -1 (ankle orientation)
#define JOINT_CPR 			16384	// Counts per revolution
#define JOINT_MIN_ABS		10967	// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		5444	// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS - 30 * JOINT_CPR/360 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 		// counts for actual angle.
#endif

#ifdef DEVICE_TF08_A02
#define JOINT_ZERO_OFFSET 	20 		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle, fittings, etc.
#define JOINT_ENC_DIR 		-1		// CW = 1 (knee orientation), CCW = -1 (ankle orientation)
#define JOINT_CPR 			16384	// Counts per revolution
#define JOINT_MIN_ABS		10967	// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		5444	// Absolute encoder reading at MAX (Max Flexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS - 30 * JOINT_CPR/360 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 		// counts for actual angle.
#endif

//Joint software limits [Degrees]
#ifdef IS_ANKLE
#define JOINT_MIN 			-20  	// [deg] Actuator physical limit min = -30deg dorsiflexion
#define JOINT_MAX 			60   	// [deg] Actuator physical limit  max = +90deg plantarflex
#endif

#ifdef IS_KNEE
#define JOINT_MIN 			-20		// [deg] Actuator physical limit min = -30deg extension
#define JOINT_MAX 			20		// [deg] Actuator physical limit max = +90deg flexion
#endif

//Force Sensor
#define FORCE_DIR			1		// Direction of positive force, Plantar Flexion is (+)
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define	FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		(FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65536	// max ticks expected
#define FORCE_MIN_TICKS		(FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65536	// min ticks expected
#define FORCE_PER_TICK		2*FORCE_MAX / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	// Newtons/Tick

//safety limits
#define MOTOR_TEMP_LIMIT 50
#define TORQUE_LIMIT	 50
#define CURRENT_LIMIT    10

#define SECONDS			1000

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
