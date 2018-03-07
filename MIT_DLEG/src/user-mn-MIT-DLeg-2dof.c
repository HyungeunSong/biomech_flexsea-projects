/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
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
	[This file] user-ex-MIT_2DoF_Ankle_v1: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	* 2016-11-16 | jfduval | Cleaned code, improved formatting
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_DLEG
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include <user-mn-MIT-DLeg-2dof.h>
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"

#include "filters.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t mitDlegInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//SAFETY FLAGS
static int8_t isSafetyFlag = 0;
static int8_t isAngleLimit = 0;
static int8_t isTorqueLimit = 0;

struct act_s act1;		//actuator sensor structure.

struct diffarr_s jnt_ang_clks;		//maybe used for velocity and accel calcs.

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_DLeg(void)
{

}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_DLeg_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t time = 0, state = -2;

//    struct act_s *ac1 = &act1;	// make a pointer to act1 structure.


    //Increment time (1 tick = 1ms)
    time++;
    static float test = 0;


	//begin safety check
    if (safetyFailure()) {
    	//motor behavior changes based on failure mode
    	//bypasses the switch statement if return true

    	return;
    }
	//end safety check


	switch(state)
	{
		case -2:
			//Same power-on delay as FSM2:
			if(time >= AP_FSM2_POWER_ON_DELAY+3000)
			{
				state = -1;
				time = 0;
			}

			break;

		case -1:
			//turned off for testing without Motor usage
//			if(findPoles()) {
//				state = 0;
//				time = 0;
//			}
			state = 0;

			break;

		case 0:
			//reserve for additional initialization
			state = 1;
			time = 0;
			break;
		case 1:
			//Pick one of those demos:
			//openSpeedFSM();
//			twoPositionFSM();

			//test passing passing reference

			updateSensorValues( &act1 );  // This will throw safetyFlag is sense out of limits

			rigid1.mn.genVar[4] = act1.safetyFlag;
			rigid1.mn.genVar[5] = act1.jointAngle * 360/(ANG_UNIT);
			rigid1.mn.genVar[6] = act1.axialForce;
			rigid1.mn.genVar[7] = 1000*act1.linkageMomentArm; //mm
			rigid1.mn.genVar[8] = act1.jointTorque;



			break;

        default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}


//Second state machine for the DLeg project
void MIT_DLeg_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
int8_t safetyFailure(void) {

	switch(isSafetyFlag)
	{
	case SAFETY_OKAY:
		return 0;
	case SAFETY_ANGLE:
			//SHUT MOTORS
		return 0;//1
	case SAFETY_TORQUE:
			//SHUT MOTORS
		return 0;//1
	default:
		break;
	}


	//check joint angles
//	if (*rigid1.ex.joint_ang <= )
	//check torque sensor

	//check motor and board temps
	//return 1;
	return 0;	//return 0 if everything checks out. not yet checking anything!
}

/*
 * Collect all sensor values and update the actuator structure.
 */
void updateSensorValues(struct act_s *actx)
{
//	actx->jointAngle = getJointAngle();
	//todo: calc jointVel;

	float *jointKinematic;
	jointKinematic = getJointAngleKinematic();

	actx->jointAngle = *(jointKinematic + 0);
	actx->jointAngleVel = *(jointKinematic + 1);
	actx->jointAngleAcc = *(jointKinematic + 2);
	actx->linkageMomentArm = getLinkageMomentArm( actx->jointAngle );
	actx->axialForce = getAxialForce();
	actx->jointTorque = getJointTorque();

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = rigid1.mn.analog[0];	//motor temp on AI0

	actx->safetyFlag = isSafetyFlag;

//	if(actx->regTemp > PCB_TEMP_LIMIT || actx->motTemp > MOTOR_TEMP_LIMIT)
//	{
//		isSafetyFlag = SAFETY_TEMP;
//	}

}

//
/*
 * Output joint angle, vel, accel in ANG_UNIT, measured from joint zero,
 * Return:
 * 		joint[0] = angle
 * 		joint[1] = velocity
 * 		joint[2] = acceleration
 * 		todo: pass a reference to the act_s structure to set flags.
 */
float * getJointAngleKinematic( void )
{
	static uint32_t timer = 0;	//probably don't need this state business.
	static int8_t jointAngleState = -1;
	static int32_t jointAngleCnts = 0;
	static float jointAccel = 0;
	static float jointAngle = 0, last_jointVel = 0;
	static int32_t jointAngleCntsAbsolute = 0;
	static float jointAngleAbsolute = 0;
	static float joint[3];

	switch(jointAngleState)
		{
			case -1:
				//We give FSM2 some time to refresh values, first time -- likley not necessary?
				timer++;
				if(timer > 25)
				{
					jointAngleCnts = *(rigid1.ex.joint_ang);
					jointAngleState = 0;
				}
				break;
			case 0:
				//ANGLE
				//Configuration orientation
				jointAngleCnts = JOINT_ANGLE_DIR * ( JOINT_ZERO + JOINT_ENC_DIR * (*(rigid1.ex.joint_ang)) );
				jointAngle = ( (float) jointAngleCnts)  * (ANG_UNIT)/JOINT_CPR;
				joint[0] = jointAngle;

				//Absolute orientation to evaluate against soft-limits
				jointAngleCntsAbsolute = JOINT_ANGLE_DIR * ( JOINT_ZERO_ABS + JOINT_ENC_DIR * (*(rigid1.ex.joint_ang)) );
				jointAngleAbsolute = ( (float) jointAngleCnts)  * (ANG_UNIT)/JOINT_CPR;

				rigid1.mn.genVar[9] = jointAngleAbsolute;


				//Check angle limits, raise flag for safety check
				if( jointAngleAbsolute <= JOINT_MIN  || jointAngleAbsolute >= JOINT_MAX)
				{
					isSafetyFlag = SAFETY_ANGLE;
					isAngleLimit = 1;		//these are all redundant, choose if we want the struct thing.

				}
				else
				{
					isAngleLimit = 0;
				}

				//VELOCITY
				joint[1] = 	*(rigid1.ex.joint_ang_vel) * (ANG_UNIT)/JOINT_CPR * SECONDS;

				//ACCEL  -- todo: not workign yet, need to evaluate timer thing
				jointAccel = ( ( joint[1] - last_jointVel ) / timer ) * (ANG_UNIT)/JOINT_CPR * SECONDS;
				last_jointVel = joint[1];
				joint[2] = jointAccel;

				break;
			default:
				//do nothing, if something went wrong.
				break;
		}

	return joint;
}

/*
 * Return joint angular velocity, don't think it's working, wrapping this into Joint Angle
 * NOT IN USE, but tries to use downsampled trick from execute
 */
//float getJointAngularVelocity(void)
//{
//	static uint32_t timer = 0;	//probably don't need this state business.
//	static int8_t jointAngleState = -1;
//	static int32_t jointAngleVelCnts = 0;
//	static float jointAngleVel = 0;
//	static float jointAngleAcc = 0;
//
//	switch(jointAngleState)
//			{
//				case -1:
//					//We give FSM2 some time to refresh values, first time -- likley not necessary?
//					timer++;
//					if(timer > 25)
//					{
//						jointAngleVelCnts = *(rigid1.ex.joint_ang_vel);
//						jointAngleState = 0;
//					}
//					break;
//				case 0:
//					//Configuration orientation
//					jointAngleVelCnts =  ( *(rigid1.ex.joint_ang_vel) ); // * JOINT_ENC_DIR, don't think we need this
//					jointAngleVel = ( (float) jointAngleVelCnts)  * (ANG_UNIT)/JOINT_CPR *1000; // [ANG_UNIT/s]
//
//					// copied from user-ex-ActPack.c
//					update_diffarr(&jnt_ang_clks, act1.jointAngle, 10);
//					jointAngleAcc = get_accl_1k_5samples_downsampled(&jnt_ang_clks)/2609; //rad/s^2
//					//end copied
//
//					// found in user-ex-ActPack
////					mot_ang = *rigid1.ex.enc_ang - mot_ang_offset;
////					mot_vel = *exec1.enc_ang_vel; //cpms
////
////					update_diffarr(&mot_ang_clks, mot_ang, 10);
////
////					mot_acc = get_accl_1k_5samples_downsampled(&mot_ang_clks)/2609; //rad/s^2
////					rigid1.ex.mot_acc = mot_acc;
//
//
//					break;
//				default:
//					//do nothing, if something went wrong.
//					break;
//			}
//		return jointAngleAcc;
//}

// Output axial force on screw, Returns [Newtons]
float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static uint16_t strainReading = 0;
	static uint16_t tareOffset = 0;
//	static uint16_t strainMeasBias = 32768;	// centered around 2.5V, or 2^16/2 ticks.
	static float axialForce = 0;

	strainReading = (rigid1.ex.strain);

//	//Check for over force reading, set flag, and exit.
//	if (strainReading >= FORCE_MAX_TICKS || strainReading <= FORCE_MIN_TICKS)
//	{
//		isForceOutOfRange = 1;			// these are all redundant.
//		act1.jointTorqueLimit = 1;
//		act1.safetyFlag = 1;
//	}

	switch(tareState)
	{
		case -1:
			//Tare the balance the first time this gets called.
			timer++;
			if(timer > 250)
			{
				strainReading = (rigid1.ex.strain);
				tareOffset = strainReading;
				tareState = 0;
			}
			break;
		case 0:
//			axialForce =  ( strainReading - strainMeasBias - tareOffset ) * FORCE_PER_TICK;	// bias = 2.5V, or 32768 ticks
			axialForce =  FORCE_DIR * ( strainReading - tareOffset ) * FORCE_PER_TICK;	// Looks correct with simple weight, need to test with a scale
			break;
		default:
			//problem occurred
			break;
	}

	return axialForce;
}

// Linear Actuator Actual Moment Arm,
// input( jointAngle, theta [deg] )
// return moment arm projected length  [m]
float getLinkageMomentArm(float theta)
{
	static float a = 0, b = 0, T = 0, F = 0;
	static float A = 0, c = 0, r = 0, C_ang = 0;
	static float theta_r = 0;
//	theta_r = ANG_UNIT % 360 ? (theta*M_PI/180) : theta; 	// convert deg to radians if necessary.
//	theta_r = theta * M_PI / 180;	// convert deg to radians.

    static const float t = 47; 		// [mm] tibial offset
    static const float t_k = 140; 	// [mm] offset from knee along tibia
    static const float f = 39;  	// [mm] femur offset
    static const float f_k = 18;	// [mm] offset from knee along femur

    theta_r = theta;

    a = sqrt( t*t + t_k*t_k);
    b = sqrt( f*f + f_k*f_k);

    T = atan(t/t_k);
    F = atan(f/f_k);

    C_ang = M_PI - theta_r - (T + F); 	// angle
    c = sqrt(a*a + b*b - 2 * a * b * cos( C_ang ) );  // length of actuator from pivot to output
    A = acos( ( a*a - ( c*c + b*b ) ) / ( -2 * b * c ) );
    r = b * sin(A);

    return r/1000;
}

/*
 *  Determine torque at joint due to moment arm and axial force
 *  input:	moment arm [m]
 *  return: 	joint torque [Nm]
 */
float getJointTorque()
{
	float *p;
	static float torque = 0;
	p = getJointAngleKinematic();



//	return getLinkageMomentArm( * getJointAngle() ) * getAxialForce();
	torque = getLinkageMomentArm( * ( p + 0 ) ) * getAxialForce();

	if(torque >= TORQUE_LIMIT || torque <= -TORQUE_LIMIT)
	{
		isSafetyFlag = SAFETY_TORQUE;
		isTorqueLimit = 1;
	}
	else
	{
		isTorqueLimit = 0;
	}
	return torque;
}


// not workign yet
void biomControlTorque(float theta_set, float k1, float k2, float b)
{
	static float theta = 0, theta_d = 0;
	static int32_t cur = 0;
	static float tor_d = 0;

	theta = act1.jointAngle;
	theta_d = act1.jointAngleVel;
	tor_d = k1 *(theta - theta_set) + k2 * (theta-theta_set)*(theta-theta_set)*(theta-theta_set) + b*theta_d;

	cur = tor_d / MOT_KT;

}

int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 10)
			{
				polesState = 1;
				timer = 0;
			}
			return 0;

			break;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitDlegInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;
			return 0;

			break;

		case 2:
			//Wait 60s... (conservative)

			if(timer >= 60*SECONDS)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;

			break;
	}

	return 0;
}

void openSpeedFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static uint8_t fsm1State = 0;

	switch(fsm1State)
	{
		case 0:
			setControlMode(CTRL_OPEN);
			setMotorVoltage(0);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorVoltage(0);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(1000);
			break;
	}
}

void twoPositionFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = *(rigid1.ex.enc_ang);
				fsm1State = 0;
			}
			break;
		case 0:
			setControlMode(CTRL_POSITION);
			setControlGains(20, 6, 0, 0);	//kp = 20, ki = 6
			setMotorPosition(initPos);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorPosition(initPos + 10000);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorPosition(initPos);
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
