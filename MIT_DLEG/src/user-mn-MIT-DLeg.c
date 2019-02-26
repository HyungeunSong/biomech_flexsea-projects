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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Tony Shu, tony shu at mit dot edu, Matthew Carney mcarney at mit dot edu
*****************************************************************************
	[This file] user-mn-MIT_DLeg_2dof: User code running on Manage
*****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT-DLeg.h"
#include "actuator_functions.h"
#include "safety_functions.h"
#include "walking_state_machine.h"	// Included to allow UserWrites to update walking machine controller.
#include "run_main_user_application.h"	// This is where user application functions live
#include "ui.h"
#include "task_machine.h"
#include "terrain_state_machine.h"

#define SCALE_FACTOR_1000 1000.0
#define SCALE_FACTOR_ONE 1.0


//****************************************************************************
// Variable(s)
//****************************************************************************


float freqInput = 0.0;
float freqRad = 0.0;
float torqInput = 0.0;

int8_t onEntry = 0;
Act_s act1;

// EXTERNS
extern uint8_t calibrationFlags, calibrationNew;
extern int32_t currentOpLimit;

extern float torqueKp;
extern float torqueKi;
extern float torqueKd;

static int gui_mode = GUI_MODE_NOM_CONTROL;
static int gui_mode_prev = GUI_MODE_NOM_CONTROL;


//****************************************************************************
// Macro(s)
//****************************************************************************

static void reInitializeUserWrites(){


	switch (gui_mode){		
	    case GUI_MODE_FL_CONTROL:
	    case GUI_MODE_UR_CONTROL:
	    case GUI_MODE_DR_CONTROL:
	    case GUI_MODE_US_CONTROL:
	    case GUI_MODE_DS_CONTROL:
	    	get_task_machine()->control_mode = gui_mode;
	    	user_data_1.w[1] = (int32_t)(get_control_params()->adaptive.hard_stop_theta_rad[get_task_machine()->control_mode]*SCALE_FACTOR_1000);
	    	user_data_1.w[2] = (int32_t)(get_control_params()->adaptive.hard_stop_k_Nm_p_rad[get_task_machine()->control_mode]);
	    	user_data_1.w[3] = (int32_t)(get_control_params()->adaptive.lst_theta_rad[get_task_machine()->control_mode]*SCALE_FACTOR_1000);
	    	user_data_1.w[4] = (int32_t)(get_control_params()->adaptive.est_k_Nm_p_rad[get_task_machine()->control_mode]);
	    	user_data_1.w[5] = (int32_t)(get_control_params()->adaptive.est_b_Nm_p_rps[get_task_machine()->control_mode]);
	    	user_data_1.w[6] = (int32_t)(get_control_params()->adaptive.lst_k_Nm_p_rad[get_task_machine()->control_mode]);
	    	user_data_1.w[7] = (int32_t)(get_control_params()->adaptive.lst_b_Nm_p_rps[get_task_machine()->control_mode]);
	    	user_data_1.w[8] = (int32_t)(get_control_params()->adaptive.lst_theta_rad[get_task_machine()->control_mode]*SCALE_FACTOR_1000);
	    	user_data_1.w[9] = (int32_t)(get_control_params()->adaptive.est_lst_min_theta_rad[get_task_machine()->control_mode]*SCALE_FACTOR_1000);
		break;
		case GUI_MODE_NOM_CONTROL:
		user_data_1.w[1] = MODE_NOMINAL;
			user_data_1.w[2] = (int32_t)(get_control_params()->nominal.theta_rad*SCALE_FACTOR_1000);
			user_data_1.w[3] = (int32_t)(get_control_params()->nominal.k_Nm_p_rad);
			user_data_1.w[4] = (int32_t)(get_control_params()->nominal.b_Nm_p_rps);
		break;
		case GUI_MODE_SW_CONTROL:
			user_data_1.w[1] = MODE_POSITION;
			user_data_1.w[2] = (int32_t)(get_control_params()->active.esw_theta_rad*SCALE_FACTOR_1000);
			user_data_1.w[3] = (int32_t)(get_control_params()->active.sw_k_Nm_p_rad);
			user_data_1.w[4] = (int32_t)(get_control_params()->active.sw_b_Nm_p_rps);
		break;
		case GUI_MODE_GAIT_EVENTS:
	    case GUI_MODE_KINEMATICS:
	    case GUI_MODE_LEARNING:
	    case GUI_MODE_FEATURES:
	    case GUI_MODE_PREDICTION:
	    	user_data_1.w[1] = MODE_NOMINAL;
		break;
	}
		
}
static void updateGenVars(){


	int16_t guimode_state_inswing = gui_mode*100 + get_walking_state()*10 + get_task_machine()->in_swing;
	rigid1.mn.genVar[0] = guimode_state_inswing;
	rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*10.0);
	rigid1.mn.genVar[2] = (int16_t) (act1.jointAngle*SCALE_FACTOR_1000);

	switch (gui_mode){
		
	    case GUI_MODE_FL_CONTROL:
	    case GUI_MODE_UR_CONTROL:
	    case GUI_MODE_DR_CONTROL:
	    case GUI_MODE_US_CONTROL:
	    case GUI_MODE_DS_CONTROL:
	    case GUI_MODE_NOM_CONTROL:
	    {
	    	int16_t withinBiologicalBounds = (int16_t)get_task_machine()->net_work_within_bounds*100 + (int16_t)get_task_machine()->stance_rom_within_bounds*10 + (int16_t)get_task_machine()->peak_power_timing_within_bounds*1;
			rigid1.mn.genVar[3] = (int16_t) (get_task_machine()->net_work_j_p_kg);
			rigid1.mn.genVar[4] = (int16_t) (get_task_machine()->net_work_error_j_p_kg[gui_mode-1]);
			rigid1.mn.genVar[5] = (int16_t) (get_task_machine()->stance_rom_rad*SCALE_FACTOR_1000);
			rigid1.mn.genVar[6] = (int16_t) (get_task_machine()->stance_rom_error_rad[gui_mode-1]*SCALE_FACTOR_1000);
			rigid1.mn.genVar[7] = (int16_t) (get_task_machine()->peak_power_timing_percent);
			rigid1.mn.genVar[8] = (int16_t) (get_task_machine()->peak_power_timing_error_percent[gui_mode-1]);
			rigid1.mn.genVar[9] = withinBiologicalBounds;
	    }
		break;
		case GUI_MODE_SW_CONTROL:
			rigid1.mn.genVar[3] = (int16_t) (get_control_params()->active.esw_theta_rad*SCALE_FACTOR_1000);
		break;
		case GUI_MODE_GAIT_EVENTS:
			rigid1.mn.genVar[3] = (int16_t) (get_task_machine()->elapsed_samples);
			rigid1.mn.genVar[4] = (int16_t) (get_task_machine()->latest_foot_static_samples);
			rigid1.mn.genVar[5] = (int16_t) (get_task_machine()->latest_foot_off_samples);
			rigid1.mn.genVar[6] = (int16_t) (get_task_machine()->aa_dot*SCALE_FACTOR_1000);
			rigid1.mn.genVar[7] = (int16_t) (get_kinematics()->aOmegaX*SCALE_FACTOR_1000);
			rigid1.mn.genVar[8] = (int16_t) (get_task_machine()->aa_dot_aOmegaX_error*SCALE_FACTOR_1000);
	    case GUI_MODE_KINEMATICS:
			rigid1.mn.genVar[3] = (int16_t) (get_task_machine()->latest_foot_static_samples);
			rigid1.mn.genVar[4] = (int16_t) (get_kinematics()->sinSqAttackAngle*SCALE_FACTOR_1000);
			rigid1.mn.genVar[5] = (int16_t) (get_kinematics()->rot1*SCALE_FACTOR_1000);
			rigid1.mn.genVar[6] = (int16_t) (get_kinematics()->rot3*SCALE_FACTOR_1000);
			rigid1.mn.genVar[7] = (int16_t) (get_kinematics()->aAy*SCALE_FACTOR_1000);
			rigid1.mn.genVar[8] = (int16_t) (get_kinematics()->aAz*SCALE_FACTOR_1000);
			rigid1.mn.genVar[9] = (int16_t) (get_kinematics()->pAz*SCALE_FACTOR_1000);
	    case GUI_MODE_LEARNING:
			rigid1.mn.genVar[3] = (int16_t) (get_statistics()->k_est);
			rigid1.mn.genVar[4] = (int16_t) (get_predictor()->A[0]);
			rigid1.mn.genVar[5] = (int16_t) (get_predictor()->A[73]);
			rigid1.mn.genVar[6] = (int16_t) (get_predictor()->A[146]);
			rigid1.mn.genVar[7] = (int16_t) (get_predictor()->A[219]);
			rigid1.mn.genVar[8] = (int16_t) (get_predictor()->A[292]);
			rigid1.mn.genVar[9] = (int16_t) (get_predictor()->B[0]);
	    case GUI_MODE_FEATURES:
			rigid1.mn.genVar[3] = (int16_t) (get_task_machine()->latest_foot_static_samples);
			rigid1.mn.genVar[4] = (int16_t) (get_task_machine()->latest_foot_off_samples);
			rigid1.mn.genVar[5] = (int16_t) (get_task_machine()->do_learning_for_stride);
			rigid1.mn.genVar[6] = (int16_t) (get_curr_features()->max[0]);
			rigid1.mn.genVar[7] = (int16_t) (get_curr_features()->min[1]);
			rigid1.mn.genVar[8] = (int16_t) (get_curr_features()->rng[2]);
			rigid1.mn.genVar[9] = (int16_t) (get_curr_features()->fin[3]);
	    case GUI_MODE_PREDICTION:
			rigid1.mn.genVar[3] = (int16_t) (get_predictor()->k_pred);
		break;

	}
}

/*
 * Updates the Input values based off of user data
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
static void updateUserWrites(Act_s *actx, struct taskmachine_s* tm, WalkParams *wParams){

	gui_mode = user_data_1.w[0];
	switch (gui_mode){
		
	    case GUI_MODE_FL_CONTROL:
	    case GUI_MODE_UR_CONTROL:
	    case GUI_MODE_DR_CONTROL:
	    case GUI_MODE_US_CONTROL:
	    case GUI_MODE_DS_CONTROL:
	    	tm->control_mode = gui_mode;
			set_hard_stop_theta_rad((float) user_data_1.w[1]/SCALE_FACTOR_1000, tm->control_mode);
			set_hard_stop_k_Nm_p_rad((float) user_data_1.w[2], tm->control_mode);
			set_lst_theta_rad(((float) user_data_1.w[3])/SCALE_FACTOR_1000, tm->control_mode);
			set_est_k_Nm_p_rad(((float) user_data_1.w[4]), tm->control_mode);
			set_est_b_Nm_p_rps((float) user_data_1.w[5], tm->control_mode);
			set_lst_k_Nm_p_rad((float) user_data_1.w[6], tm->control_mode);
			set_lst_b_Nm_p_rps((float) user_data_1.w[7], tm->control_mode);
			set_lst_theta_rad((float) user_data_1.w[8]/SCALE_FACTOR_1000, tm->control_mode);
			set_est_lst_min_theta_rad((float) user_data_1.w[9]/SCALE_FACTOR_1000, tm->control_mode);
		break;
		case GUI_MODE_NOM_CONTROL:
			tm->control_mode = user_data_1.w[1];
			set_nominal_theta_rad((float) user_data_1.w[2]  /SCALE_FACTOR_1000);
			set_nominal_k_Nm_p_rad((float) user_data_1.w[3] );
			set_nominal_b_Nm_p_rps((float) user_data_1.w[4] );
		break;
		case GUI_MODE_SW_CONTROL:
			tm->control_mode = user_data_1.w[1];
			set_esw_theta_rad(((float) user_data_1.w[2])/SCALE_FACTOR_1000);
			set_sw_k_Nm_p_rad(((float) user_data_1.w[3]));
			set_sw_b_Nm_p_rps(((float) user_data_1.w[4]));
		break;
		case GUI_MODE_GAIT_EVENTS:
	    case GUI_MODE_KINEMATICS:
	    case GUI_MODE_LEARNING:
	    case GUI_MODE_FEATURES:
	    case GUI_MODE_PREDICTION:
	    	tm->control_mode = user_data_1.w[1];
		break;

	}
	
	if (gui_mode_prev != gui_mode){
		reInitializeUserWrites();
	}
	gui_mode_prev = gui_mode;
}

static void initializeUserWrites(){
	user_data_1.w[0] = (int32_t) (gui_mode);
	for (int i=1; i < 10; i++)
		user_data_1.w[i] = 0;
}

#define FINDPOLES_DONE (calibrationFlags == 0) && (calibrationNew == 0)
//****************************************************************************
// Public Function(s)
//****************************************************************************

//MIT DLeg Finite State Machine.

/*
 *  Initialize the Finite State Machine
 */
void initMITDLeg(void) {


}


/*
 *  Finite State Machine with multiple states.
 *  Call this function in one of the main while fsmTime slots.
 *  The possible states are as follows:
 *  	STATE_POWER_ON
 *  	STATE_INITIALIZE_SENSORS
 *  	STATE_FIND_POLES
 *  	STATE_INIT_USER_WRITES
 *  	STATE_MAIN
 *  	STATE_DEBUG
 */
void MITDLegFsm1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t fsmTime = 0;
    fsmTime++;

    updateGenVars();

    //begin main FSM
	switch(fsm1State)
	{
		//this state is always reached
		case STATE_POWER_ON:
			stateMachine.currentState = STATE_IDLE;
			//Same power-on delay as FSM2:
			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {
				//sensor update happens in mainFSM2(void) in main_fsm.c

				isEnabledUpdateSensors = 1;
				onEntry = 1;
				fsmTime = 0;
				fsm1State = STATE_INITIALIZE_SENSORS;
			}

			break;

		case STATE_INITIALIZE_SENSORS:

			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {

				#ifndef NO_DEVICE
					fsm1State = STATE_FIND_POLES;
				#else
					fsm1State = STATE_DEBUG;
				#endif
				act1.currentOpLimit = CURRENT_LIMIT_INIT;
				onEntry = 1;
			}


			break;


		case STATE_FIND_POLES:

//			stateMachine.current_state = STATE_INIT;
			if (!actuatorIsCorrect(&act1)){
				setLEDStatus(0,0,1); //flash red; needs reset
			} else{
				if (onEntry) {
					// USE these to to TURN OFF FIND POLES set these = 0 for OFF, or =1 for ON
					calibrationFlags = 1, calibrationNew = 1;
					isEnabledUpdateSensors = 0;
					onEntry = 0;

				}

				// Check if FindPoles has completed, if so then go ahead. This is done in calibration_tools.c
				if (FINDPOLES_DONE){
					fsm1State = STATE_INIT_USER_WRITES;
					fsmTime = 0;
					isEnabledUpdateSensors = 1;

				}
			}



			break;

		case STATE_INIT_USER_WRITES:

			/*reserve for additional initialization*/

			mitInitCurrentController();		//initialize Current Controller with gains
//					setControlMode(CTRL_OPEN, 0);		//open control for alternative testing


			//Set usewrites to initial values
			walkParams.initializedStateMachineVariables = 0;
			if (!walkParams.initializedStateMachineVariables){
				initializeUserWrites(&act1, &walkParams);

			}

			//absolute torque limit scaling factor TODO: possibly remove
			act1.safetyTorqueScalar = 1.0;

			fsm1State = STATE_MAIN;
			fsmTime = 0;
			onEntry = 1;

			break;


		case STATE_MAIN:
			{
				updateUserWrites(&act1, get_task_machine(), &walkParams);
				//TODO consider changing logic so onEntry is only true for one cycle.
				if (onEntry && fsmTime > DELAY_TICKS_AFTER_FIND_POLES) {
					act1.currentOpLimit = CURRENT_LIMIT_INIT;
					onEntry = 0;
				}

				// Inside here is where user code goes
				if (getMotorMode() == MODE_ENABLED || getMotorMode() == MODE_OVERTEMP ){

					runMainUserApplication( &rigid1, &act1);

				}

				break;
			}
		case STATE_DEBUG:
			updateUserWrites(&act1, get_task_machine(), &walkParams);
			runMainUserApplication(&rigid1, &act1);
			break;

        	default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF

}





/*
 *  Second state machine for the DLeg project
 *  Currently seems unused
 */
void MITDLegFsm2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

/*UserWrites are inputs from Plan. They are initailized to teh values shown below.
 * Their values are then used by udpateUserWrites to set function values.
 * These can be updated as necessary.
 * Do keep care to initialize and upate correctly.
 * Also note, the initial values will not show up in Plan, that must be manually entered
 */




#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
