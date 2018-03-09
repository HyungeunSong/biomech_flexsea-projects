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

//SAFETY FLAGS - in addition to enum, so can be cleared but don't lose other flags that may exist.
static int8_t isSafetyFlag = 0;
static int8_t isAngleLimit = 0;
static int8_t isTorqueLimit = 0;
static int8_t isTempLimit = 0;

static int32_t currentOpLimit = CURRENT_LIMIT; 	//operational limit for current.

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

    static uint32_t time = 0;
    static int32_t state = -2;

//    struct act_s *ac1 = &act1;	// make a pointer to act1 structure.


    //Increment time (1 tick = 1ms)
    time++;

    //Must run every loop, but needs to stabilize before initializing the first time.
    if(state > -1)
    {
    	updateSensorValues( &act1 );	// updates all actuator sensors, will throw safety flags.
    }

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
			mit_init_current_controller();		//initialize Current Controller with gains

			state = 1;
			time = 0;
			break;
		case 1:
			//Pick one of those demos:
			//openSpeedFSM();
//			twoPositionFSM();
			oneTorqueFSM( &act1 );
//			twoTorqueFSM( &act1);

			rigid1.mn.genVar[0] = act1.safetyFlag;
			rigid1.mn.genVar[1] = state;
			rigid1.mn.genVar[2] = 0;


			rigid1.mn.genVar[5] = act1.jointAngle * 360/(ANG_UNIT);


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
/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t safetyFailure(void)
{
	switch(isSafetyFlag)
	{
		case SAFETY_OK:
			return 0;
		case SAFETY_ANGLE:

			//check if flag is not still active to be released, else do something about problem.
			if( !isAngleLimit )
			{
				isSafetyFlag = SAFETY_OK;
				break;
			} else // remedy the problem
			{
				setMotorCurrent(0); // turn off motor. might need something better than this.

			}

			return 0;//1
		case SAFETY_TORQUE:

			//check if flag is not still active to be released, else do something about problem.
			if( !isTorqueLimit )
			{
				isSafetyFlag = SAFETY_OK;
				break;
			} else // remedy the problem
			{
				setMotorTorque(&act1, 0);
			}

			return 0;//1

		case SAFETY_TEMP:
			//check if flag is not still active to be released, else do something about problem.
			if( !isTempLimit )
			{
				currentOpLimit = CURRENT_LIMIT;		// return to full power todo: may want to gradually increase
				isSafetyFlag = SAFETY_OK;
				break;
			} else // remedy the problem
			{
				if (currentOpLimit > 0)
				{
					currentOpLimit--;	//reduce current limit every cycle until we cool down.
				}
			}

			return 0;

		default:
			break;
	}


	return 0;	//return 0 if everything checks out. not yet checking anything!
}

/*
 * Collect all sensor values and update the actuator structure.
 * Throws safety flags on Joint Angle, Joint Torque, since these functions look at transformed values.
 */
void updateSensorValues(struct act_s *actx)
{
//	actx->jointAngle = getJointAngle();
	//todo: calc jointVel;

	float *jointKinematic;
	jointKinematic = getJointAngleKinematic();

	actx->jointAngle = *(jointKinematic + 0);
	actx->jointVel = *(jointKinematic + 1);
	actx->jointAcc = *(jointKinematic + 2);
	actx->linkageMomentArm = getLinkageMomentArm( actx->jointAngle );
	actx->axialForce = getAxialForce();
	actx->jointTorque = getJointTorque();

	actx->motorVel =  *rigid1.ex.enc_ang_vel * 1 / 16.384 * (ANG_UNIT);	// rad/s
	actx->motorAcc = rigid1.ex.mot_acc;	// rad/s/s

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = getMotorTempSensor();
	actx->motCurr = rigid1.ex.mot_current;

	actx->safetyFlag = isSafetyFlag;

	if(actx->regTemp > PCB_TEMP_LIMIT || actx->motTemp > MOTOR_TEMP_LIMIT)
	{
		isSafetyFlag = SAFETY_TEMP;
		isTempLimit = 1;
	}

}

//The ADC reads the motor Temp sensor - MCP9700 T0-92. This function converts the result to degrees.
int16_t getMotorTempSensor(void)
{
	static int16_t mot_temp = 0;
//	mot_temp = ((VSENSE_SLOPE * (rigid1.mn.analog[0] - V25_TICKS) \
//					/ TICK_TO_V) + 25);
	mot_temp = (rigid1.mn.analog[0] * (3.3/4096) - 500) / 10; 	//celsius
	rigid1.mn.mot_temp = mot_temp;

	return mot_temp;
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

				//Check angle limits, raise flag for safety check
				if( jointAngleAbsolute <= JOINT_MIN  || jointAngleAbsolute >= JOINT_MAX)
				{
					isSafetyFlag = SAFETY_ANGLE;
					isAngleLimit = 1;		//these are all redundant, choose if we want the struct thing.
				} else
				{
					isAngleLimit = 0;
				}

				//VELOCITY
				joint[1] = 	*(rigid1.ex.joint_ang_vel) * (ANG_UNIT)/JOINT_CPR * SECONDS;

				//ACCEL  -- todo: not workign yet, need to evaluate timer thing, maybe better to reflect motor accl to joint (more counts available)
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


// Output axial force on screw, Returns [Newtons]
float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static uint16_t strainReading = 0;
	static uint16_t tareOffset = 0;
	static float axialForce = 0;

	strainReading = (rigid1.ex.strain);

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
			axialForce =  FORCE_DIR * ( strainReading - tareOffset ) * FORCE_PER_TICK;	// Looks correct with simple weight, need to test with a scale
			break;
		default:
			//problem occurred
			break;
	}

	return axialForce;
}

// Linear Actuator Actual Moment Arm,
// input( jointAngle, theta [rad] )
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
 *  return: joint torque [Nm]
 */
float getJointTorque()
{
	float *p;
	static float torque = 0;
	p = getJointAngleKinematic();	//need the pointer to get to angle, at index 0

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

/*
 * Calculate required motor torque, based on joint torque
 * input:	*actx,  actuator structure reference
 * 			tor_d, 	desired torque at joint [Nm]
 * return:	set motor torque
 * 			Motor Torque request, or maybe current
 */
void setMotorTorque(struct act_s *actx, float tau_des)
{
	static int8_t time = 1; 		// ms
	static float N = 0;				// moment arm [m]
	static float tau_meas = 0;  	//joint torque reflected to motor.
	static float tau_err = 0, tau_err_last = 0;
	static float tau_err_dot = 0, tau_err_int = 0;
	static float tau_motor = 0;		// motor torque signal
	static int32_t dtheta_m = 0, ddtheta_m = 0;	//motor vel, accel
	static int32_t I = 0;			// motor current signal

	N = actx->linkageMomentArm * N_SCREW;
	dtheta_m = actx->motorVel;
	ddtheta_m = actx->motorAcc;


	// todo: better fidelity may be had if we modeled N_ETA as a function of torque, long term goal, if necessary
	tau_meas =  actx->jointTorque / ( N ) * 1000;	// measured torque reflected to motor [mNm]
	tau_des = tau_des / (N * N_ETA) *1000;					// desired joint torque, reflected to motor [mNm]

	rigid1.mn.genVar[3] = tau_meas;
	rigid1.mn.genVar[6] = tau_des;

	tau_err = (tau_des - tau_meas);
	tau_err_dot = (tau_err - tau_err_last)/time;
	tau_err_int = tau_err_int + tau_err;
	tau_err_last = tau_err;



	//PID around motor torque.
	tau_motor = tau_err * TORQ_KP + (tau_err_dot) * TORQ_KD + (tau_err_int) * TORQ_KI;

	I = 1 / MOT_KT * ( (int32_t) tau_motor + (MOT_J + MOT_TRANS)*ddtheta_m + MOT_B*dtheta_m);		// + (J_rotor + J_screw)*ddtheta_m + B*dtheta_m
	//I think I needs to be scaled to mA, but not sure yet.

	rigid1.mn.genVar[7] = I; // mA

	//Saturate I for our current operational limits -- limit can be reduced by safetyFailure() due to heating
	if(I > currentOpLimit )
	{
		I = currentOpLimit;
	} else if (I < -currentOpLimit)
	{
		I = -currentOpLimit;
	}

	rigid1.mn.genVar[4] = I;

	setMotorCurrent(I);				// send current command to comm buffer to Execute
}

/*
 * Simple Biom controller
 * input:	theta_set, desired theta
 * 			k1,k2,b, impedance parameters
 * return: 	tor_d, desired torque
 */
float biomControlImpedance(float theta_set, float k1, float k2, float b)
{
	static float theta = 0, theta_d = 0;
	static float tor_d = 0;

	theta = act1.jointAngle;
	theta_d = act1.jointVel;
	tor_d = k1 *(theta - theta_set) + k2 * (theta-theta_set)*(theta-theta_set)*(theta-theta_set) + b*theta_d;

	return tor_d;

}

void mit_init_current_controller(void)
{

	setControlMode(CTRL_CURRENT);
	writeEx.setpoint = 0;			// wasn't included in setControlMode, could be safe for init
	setControlGains( ACTRL_I_KP, ACTRL_I_KI, ACTRL_I_KD, 0 );

// there is another example of this may have been an old initializtion, copied from user-mn-ActPack

}

int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 100)
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

			if(timer >= 45*SECONDS)
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

void twoTorqueFSM(struct act_s *actx)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	rigid1.mn.genVar[9] = fsm1State;


	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = actx->jointAngle;
				fsm1State = 0;
			}
			break;
		case 0:
//			mit_init_current_controller();
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
			setMotorTorque( actx, 2);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorTorque( actx, -2);
			break;
	}
}

void oneTorqueFSM(struct act_s *actx)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	rigid1.mn.genVar[9] = fsm1State;


	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = actx->jointAngle;
				fsm1State = 0;
			}
			break;
		case 0:
//			mit_init_current_controller();
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
			setMotorTorque( actx, 2);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorTorque( actx, 0);
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
