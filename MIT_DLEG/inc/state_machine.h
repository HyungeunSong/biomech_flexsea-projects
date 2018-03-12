#ifdef __cplusplus
extern "C" {
#endif

#if (defined INCLUDE_UPROJ_MIT_DLEG && defined BOARD_TYPE_FLEXSEA_MANAGE) || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef STATE_MACHINE
#define STATE_MACHINE

//****************************************************************************
// Include(s)
//****************************************************************************
#include "state_variables.h"

//****************************************************************************
// Definition(s):
//****************************************************************************
#define ESW_TO_LSW_DELAY              			  100   // Transition time: 2->3 in ms

//      THRESHOLD / LIMIT NAME                    VALUE          UNITS           BRIEF DESCRIPTION                             TRANSITION(S)
#define HARD_HEALSTRIKE_SS_TORQ_RATE_THRESH      -180		// Nm/sec      - Foot-strike detector                          3->4
#define LSTPWR_HS_TORQ_TRIGGER_THRESH             -5				// Nm          - The ONLY entry to Late Stance Power           4->5
#define ANKLE_UNLOADED_TORQUE_THRESH              -5			// Nm          - Foot unloaded threshold                       5->2

//****************************************************************************
// Shared Variable(s):
//****************************************************************************
extern GainParams eswGains;
extern GainParams lswGains;
extern GainParams estGains;
extern GainParams lstGains; //currently unused in simple implementation
extern GainParams lstPowerGains;
extern WalkingStateMachine stateMachine;
extern Act_s act1;

//****************************************************************************
// Prototype(s):
//****************************************************************************

void runFlatGroundFSM(float* ptorqueDes);

#endif //STATE_MACHINE
#endif //(INCLUDE_UPROJ_MIT_DLEG && BOARD_TYPE_FLEXSEA_MANAGE) || BOARD_TYPE_FLEXSEA_PLAN
#ifdef __cplusplus
}
#endif