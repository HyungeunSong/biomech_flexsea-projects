#ifdef __cplusplus
extern "C" {
#endif

#include "state_variables.h"
#include "biom_state_machine.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "free_ankle_EMG.h"
#include "stand_ankle_EMG.h"
#include "cmd-DLeg.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include <math.h>

WalkingStateMachine stateMachine;
Act_s act1;
WalkParams walkParams;

// Gain Parameters are modified to match our joint angle convention (RHR for right ankle, wearer's perspective)
GainParams eswGains = {3.0, 0.0, 0.2, -10.0};
GainParams lswGains = {1.5, 0.0, 0.2, 0.0};
GainParams estGains = {0.0, 0.0, 0.05, 0.0};
GainParams lstGains = {0.0, 0.0, 0.0, 0.0}; //currently unused in simple implementation
GainParams lstPowerGains = {4.5, 0.0, 0.2, JNT_ORIENT * -14};

GainParams emgStandGains = {2, 0.025, 0.04, 0};
GainParams emgFreeGains  = {1.2, 0, 0.02, 0};


#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Static Functions:
//****************************************************************************

static void updatePFDFState(Act_s *actx);

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams);
static void updateUserWrites(Act_s *actx, WalkParams *wParams);
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams);
static void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams);
static void initializeUserWrites(WalkParams *wParams);


/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(Act_s *actx) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;


    if (!walkParams.initializedStateMachineVariables){
    	initializeUserWrites(&walkParams);
    }

    updateUserWrites(actx, &walkParams);

    stateMachine.on_entry_sm_state = stateMachine.current_state; // save the state on entry, assigned to last_current_state on exit

    actx->tauDes = 0;

    // Check for state change, then set isTransitioning flag
    if (stateMachine.current_state == stateMachine.last_sm_state) {
        isTransitioning = 0;
        time_in_state++;
    } else {
        // State transition, reset timers and set entry flag
        time_in_state = 0;
        isTransitioning = 1;
    }

    switch (stateMachine.current_state) {

        case STATE_IDLE: //0
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT: //1

            stateMachine.current_state = STATE_LATE_SWING;

            break;
					
        case STATE_EARLY_SWING: //2

            //Put anything you want to run ONCE during state entry.
			if (isTransitioning) {
				walkParams.virtual_hardstop_tq = 0.0;
			}

            actx->tauDes = calcJointTorque(eswGains, actx, &walkParams);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) {
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING: //3

        	if (isTransitioning) {
        		walkParams.transition_id = 0;

        	}

            updateVirtualHardstopTorque(actx, &walkParams);

            actx->tauDes = calcJointTorque(lswGains, actx, &walkParams);

            //---------------------- LATE SWING TRANSITION VECTORS ----------------------//
            if(time_in_state > ESW_TO_LSW_DELAY){

				// VECTOR (1): Late Swing -> Early Stance (hard heal strike) - Condition 1
				if (actx->jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH && actx->jointTorqueRate > HARD_HEELSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 1;
				}
				// VECTOR (1): Late Swing -> Early Stance (gentle heal strike) - Condition 2 -
				else if (actx->jointTorqueRate > GENTLE_HEALSTRIKE_TORQ_RATE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 2;
				}
				// VECTOR (1): Late Swing -> Early Stance (toe strike) - Condition 3
				else if (actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH) {
					stateMachine.current_state = STATE_EARLY_STANCE;
					walkParams.transition_id = 3;
				}
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//
            break;

        case STATE_EARLY_STANCE: //4

            if (isTransitioning) {
                walkParams.scaleFactor = 1.0;
                estGains.k1 = walkParams.earlyStanceK0;
                estGains.thetaDes = actx->jointAngleDegrees;
            }

          	updateVirtualHardstopTorque(actx, &walkParams);
            updateImpedanceParams(actx, &walkParams);

            actx->tauDes = calcJointTorque(estGains, actx, &walkParams);

            //Early Stance transition vectors
            // VECTOR (1): Early Stance -> Late Stance POWER!
            //---------------------- EARLY STANCE TRANSITION VECTORS ----------------------//

            // VECTOR (A): Early Stance -> Late Stance (foot flat) - Condition 1
            if (actx->jointAngleDegrees < EST_TO_LST_FOOT_FLAT_HS_ANGLE_LIMIT && actx->jointTorqueRate < EST_TO_LST_FOOT_FLAT_TORQ_RATE) {
            	stateMachine.current_state = STATE_LATE_STANCE;      //Transition occurs even if the early swing motion is not finished
            }
            // VECTOR (A): Early Stance -> Late Stance (toe strike) - Condition 2
            else if(actx->jointAngleDegrees < HARD_TOESTRIKE_ANGLE_THRESH) {
                stateMachine.current_state = STATE_LATE_STANCE;
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//        
            break;

        case STATE_LATE_STANCE: //5

            if (isTransitioning) {
//               lstGains.k1 = walkParams.earlyStanceKF;
//               lstGains.thetaDes = actx->jointAngleDegrees;
            }

            updateVirtualHardstopTorque(actx, &walkParams);
            updateImpedanceParams(actx, &walkParams);
            actx->tauDes = calcJointTorque(estGains, actx, &walkParams);

            //---------------------- LATE STANCE TRANSITION VECTORS ----------------------//

            // VECTOR (1): Late Stance -> Late Stance POWER
            if (actx->jointTorque > walkParams.lspEngagementTorque) {
            	stateMachine.current_state = STATE_LATE_STANCE_POWER;      //Transition occurs even if the early swing motion is not finished
            }

            //------------------------- END OF TRANSITION VECTORS ------------------------//     
            break;

        case STATE_LATE_STANCE_POWER: //6

            if (isTransitioning) {
                walkParams.samplesInLSP = 0.0;
                walkParams.lsp_entry_tq = actx->jointTorque;
//                walkParams.virtual_hardstop_tq = 0.0;
//                walkParams.pff_lumped_gain = walkParams.pff_gain * powf(1.0/(PCI - walkParams.lsp_entry_tq), walkParams.pff_exponent);
            }

            if (walkParams.samplesInLSP < walkParams.lstPGDelTics){
            	walkParams.samplesInLSP = walkParams.samplesInLSP + 1.0;
            }

            updateVirtualHardstopTorque(actx, &walkParams);

            //Linear ramp push off
            actx->tauDes = -1.0*actx->jointTorque + (walkParams.samplesInLSP/walkParams.lstPGDelTics) * calcJointTorque(lstPowerGains, actx, &walkParams);

            //Late Stance Power transition vectors
            // VECTOR (1): Late Stance Power -> Early Swing - Condition 1


            if (abs(actx->jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH && time_in_state > 200) {
                stateMachine.current_state = STATE_EARLY_SWING;
            }

            break;

        case STATE_EMG_STAND_ON_TOE:
        	//untested and unused. If would like to use, need to check implementation on ts_projects

        	//disable virtual hardstop while in EMG mode
        	walkParams.virtual_hardstop_tq = 0;

        	if (MIT_EMG_getState() == 1) {

				updateStandJoint(&emgStandGains);

				//if we're still in EMG control even after updating EMG control vars
				if (stateMachine.current_state == STATE_EMG_STAND_ON_TOE) {
					actx->tauDes = calcJointTorque(emgStandGains, actx, &walkParams);

				//actually in early stance (explicitly declared)
				} else if (stateMachine.current_state == STATE_EARLY_STANCE) {
					actx->tauDes = calcJointTorque(estGains, actx, &walkParams);
				}

			} else {
				stateMachine.current_state = STATE_EARLY_STANCE;
				reset_EMG_stand(actx->jointAngleDegrees);
				actx->tauDes = calcJointTorque(estGains, actx, &walkParams); //do early stance control if EMG disconnected
			}

            break;
		
        case STATE_LSW_EMG:
        	//toDo: not integrated yet
        	//upon entering, make sure virtual joint and robot joint match
        	if (isTransitioning) {
        		updatePFDFState(actx);
        	}

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
				updateVirtualJoint(&emgFreeGains);
				actx->tauDes = calcJointTorque(emgFreeGains, actx, &walkParams);
        	} else {
        		//reset and command EST torque
        		updatePFDFState(actx);
        		actx->tauDes = calcJointTorque(estGains, actx, &walkParams);
        	}

        	//toDo: Late Swing EMG transition vectors to Early Stance HOW?! Perhaps load cell

        	break;
		
        default:

            //turn off control.
            actx->tauDes = 0;
            stateMachine.current_state = STATE_LATE_SWING;

            break;
	
    }

    //update last state in preparation for next loop
    stateMachine.last_sm_state = stateMachine.on_entry_sm_state;



}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
	@param	actx Actuator params
	@param  wParams Parameters relating to walking states
    @return float desired torque at joint (Nm)
*/
static float calcJointTorque(GainParams gainParams, Act_s *actx, WalkParams *wParams) {

//	if (fabs(actx->jointVelDegrees) > 6){
		return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) \
         - gainParams.b * actx->jointVelDegrees  + wParams->virtual_hardstop_tq;
//	}else{
//    	return gainParams.k1 * (gainParams.thetaDes - actx->jointAngleDegrees) + wParams->virtual_hardstop_tq;
//	}
}

static void updateUserWrites(Act_s *actx, WalkParams *wParams){

    eswGains.thetaDes  = ((float) user_data_1.w[0])/OUTPUT_DIVISOR0;   					//-10 x 1
    wParams->lstPGDelTics = ((float) user_data_1.w[1])/OUTPUT_DIVISOR1;					//1 x 100
    lswGains.b = ((float) user_data_1.w[2])/OUTPUT_DIVISOR2;					//0.2 x 100
    wParams->earlyStanceDecayConstant = ((float) user_data_1.w[3])/OUTPUT_DIVISOR3;    	//0.995 x 10000
    estGains.b = ((float) user_data_1.w[4])/OUTPUT_DIVISOR4;  							//0.1 x 100
    wParams->virtualHardstopK = ((float) user_data_1.w[5])/OUTPUT_DIVISOR5;				//70 x 100
    wParams->virtualHardstopEngagementAngle = ((float) user_data_1.w[6])/OUTPUT_DIVISOR6;	//0.0 x 1
    lstPowerGains.thetaDes = ((float) user_data_1.w[7])/OUTPUT_DIVISOR7;				//18.0 x 1
    lstPowerGains.k1 = ((float) user_data_1.w[8])/OUTPUT_DIVISOR8; 						//4.5 x 100
    wParams->lspEngagementTorque = ((float) user_data_1.w[9])/OUTPUT_DIVISOR9;				//45 x 1

//    lswGains.k1 = ((float) user_data_1.w[7])/OUTPUT_DIVISOR7;				//6.0 x 100
//    eswGains.k1 = ((float) user_data_1.w[8])/OUTPUT_DIVISOR8; 			//3.0 x 100
//    eswGains.b = ((float) user_data_1.w[9])/OUTPUT_DIVISOR9;				//0.2 x 100
}

static void initializeUserWrites(WalkParams *wParams){

	wParams->earlyStanceK0 = 5.23;
	wParams->earlyStanceKF = 0.05;


	user_data_1.w[0] = -10;
	user_data_1.w[1] = 100;
	user_data_1.w[2] = 20;
	user_data_1.w[3] = 9950;
	user_data_1.w[4] = 10;
	user_data_1.w[5] = 2000; //Jim's was defaulted to 700;
	user_data_1.w[6] = 0;
	user_data_1.w[7] = 14;
	user_data_1.w[8] = 450;
	user_data_1.w[9] = 70; //Jim's default was 45
//	user_data_1.w[7] = 300;
//	user_data_1.w[8] = 600;
//	user_data_1.w[9] = 20;

	wParams->initializedStateMachineVariables = 1;
}

static void updateImpedanceParams(Act_s *actx, WalkParams *wParams) {

	wParams->scaleFactor = wParams->scaleFactor * wParams->earlyStanceDecayConstant;

    estGains.k1 = wParams->earlyStanceKF + wParams->scaleFactor * (wParams->earlyStanceK0 - wParams->earlyStanceKF);

    if (actx->jointAngleDegrees < estGains.thetaDes) {
    	estGains.thetaDes = actx->jointAngleDegrees;
    }
}


static void updateVirtualHardstopTorque(Act_s *actx, WalkParams *wParams) {

	if (JNT_ORIENT*actx->jointAngleDegrees > wParams->virtualHardstopEngagementAngle) {
		wParams->virtual_hardstop_tq = wParams->virtualHardstopK * ((JNT_ORIENT * actx->jointAngleDegrees) - wParams->virtualHardstopEngagementAngle);
	} else {
		wParams->virtual_hardstop_tq = 0.0;
	}
}

//reset virtual joint to robot joint state
void updatePFDFState(struct act_s *actx) {
	PFDF_state[0] = equilibriumAngle;
	PFDF_state[1] = 0;
	PFDF_state[2] = 0;
}

#endif //BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif


