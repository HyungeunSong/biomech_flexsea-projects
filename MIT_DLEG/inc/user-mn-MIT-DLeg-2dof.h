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
extern struct act_s act1;	//define actuator structure shared

//****************************************************************************
// Structure(s)
//****************************************************************************

// Actuator structure to track sensor values, initially built for the TF08 style actuator
struct act_s
{
	float jointAngle;
	float jointAngleVel;
	float jointAngleAcc;
	float linkageMomentArm;
	float axialForce;
	float jointTorque;
	int8_t safetyFlag;			// todo: consider if we want these flags here.
	int16_t regTemp;		// regulate temperature
	int16_t motTemp;		// motor temperature
};

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

//float getJointAngle(void);
float * getJointAngleKinematic(void);
float getJointAngularVelocity(void);
float getAxialForce(void);
float getLinkageMomentArm(float);
float getJointTorque(void);
void updateSensorValues(struct act_s *actx);
void biomControlTorque(float theta_set, float k1, float k2, float b);

void openSpeedFSM(void);
void twoPositionFSM(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees
#define IS_ANKLE
//#define IS_KNEE
#define DEVICE_TF08_A01			// Define specific actuator configuration. Ankle 01
//#define DEVICE_TF08_A02			// Define specific actuator configuration. Ankle 02
//#define DEVICE_TF08_K01			// Define specific actuator configuration. Knee 01
//#define DEVICE_TF08_K02			// Define specific actuator configuration. Knee 02
#define ANG_UNIT			2*M_PI 	// Use Radians 2*M_PI

#ifdef DEVICE_TF08_A01
//Encoder
#define JOINT_ZERO_OFFSET 	0 		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	-1		// Joint angle direction. Std convention is Ankle: Dorsiflexion (+), Plantarflex (-)
#define JOINT_CPR 			16383	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_HS_MIN		( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		10967	// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		5444	// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 		// counts for actual angle.

//Force Sensor
#define FORCE_DIR			1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+)
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define	FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)// Newtons/Tick

#endif

#ifdef DEVICE_TF08_A02
// copy from above and update, when ready.
#endif

//Joint software limits [Degrees]
#ifdef IS_ANKLE
#define JOINT_MIN 			-20  * (ANG_UNIT)/360	// [deg] Actuator physical limit min = -30deg dorsiflexion
#define JOINT_MAX 			60   * (ANG_UNIT)/360	// [deg] Actuator physical limit  max = +90deg plantarflex
#endif

#ifdef IS_KNEE
#define JOINT_MIN 			-20	* (ANG_UNIT)/360	// [deg] Actuator physical limit min = -30deg extension
#define JOINT_MAX 			20	* (ANG_UNIT)/360	// [deg] Actuator physical limit max = +90deg flexion
#endif

// Motor Parameters
#define MOT_KT 			0.0951		// Kt value
#define MOT_L			0.068		// mH

//safety limits
#define PCB_TEMP_LIMIT   50
#define MOTOR_TEMP_LIMIT 50
#define TORQUE_LIMIT	 50
#define CURRENT_LIMIT    10

enum {
	SAFETY_OKAY		=		0,
	SAFETY_ANGLE	=		1,
	SAFETY_TORQUE	=		2,
	SAFETY_FLEX_TEMP=		3,
	SAFETY_TEMP		=		4,
};

#define SECONDS			1000



#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
