/*
 * safety_functions.c
 *
 *  Created on: Jan 8, 2019
 *      Author: matt
 */

#include "safety_functions.h"

//bitshift macros TODO: test
#define IS_FIELD_HIGH(i, map) ( (map) & (1 << ((i)%16)) )
#define SET_MAP_HIGH(i, map) ( (map) |= (1 << ((i)%16)) )
#define SET_MAP_LOW(i, map) ( (map) &= (~(1 << ((i)%16))) )

//variables
static int8_t errorConditions[ERROR_ARRAY_SIZE]; //massage extern until it works
static int16_t safetyFlags; //bitmap of all errors. Also serves as boolean for error existence
static int8_t motorMode;
static const int16_t stm32ID[] = STM32ID;

//check connected/disconnect status
//TODO: find ways of actually checking these
static void checkLoadcell(Act_s *actx) {
	static uint16_t previousStrainValue = 0;
	static int8_t tripped = 0;
	static uint32_t delayTimer; //TODO: possible to disconnect load cell at t = 0 upon rollover

	//check to see if loadcell is disconnected
	if (abs((int32_t)rigid1.ex.strain - (int32_t)previousStrainValue) >= LOADCELL_DISCONNECT_STRAIN_DIFFERENCE
			&& !tripped && delayTimer >= LOADCELL_DISCONNECT_COUNT_THRESHOLD) {
		errorConditions[ERROR_LDC] = SENSOR_DISCONNECT;
		tripped = 1;
	} else if (!tripped) {
		errorConditions[ERROR_LDC] = SENSOR_NOMINAL;
	}

	previousStrainValue = rigid1.ex.strain;
	delayTimer++;
}

static void checkJointEncoder(Act_s *actx) {
	static uint16_t disconnectCounter = 0;
	static int16_t previousJointValue = 0;

	if (abs(*rigid1.ex.joint_ang - previousJointValue) <= JOINT_ANGLE_DIFF_VALUE) {
		disconnectCounter++;
	} else {
		disconnectCounter = 0;
	}

	if (disconnectCounter >= JOINT_ANGLE_COUNT_THRESHOLD) {
		errorConditions[ERROR_JOINT_ENCODER] = SENSOR_DISCONNECT;
	} else {
		errorConditions[ERROR_JOINT_ENCODER] = SENSOR_NOMINAL;
	}

	previousJointValue = *rigid1.ex.joint_ang;
}

static void checkMotorEncoder(Act_s *actx) {
	static uint16_t disconnectCounter = 0;
	static int32_t previousMotorValue = 0;

	if ((abs(*rigid1.ex.enc_ang - previousMotorValue) <= MOTOR_ANGLE_DIFF_VALUE)
			&& (fabs(actx->axialForce) >= MOTOR_ENCODER_DISCONNECT_AXIAL_FORCE_THRESHOLD_N)) {
		disconnectCounter++;
	} else {
		disconnectCounter = 0;
	}

	if (disconnectCounter >= MOTOR_ANGLE_COUNT_THRESHOLD ||
			abs(*rigid1.ex.enc_ang) > MOTOR_ENCODER_DISCONNECT_BOUND) {
		errorConditions[ERROR_MOTOR_ENCODER] = SENSOR_DISCONNECT;
	} else if (errorConditions[ERROR_MOTOR_ENCODER] != SENSOR_DISCONNECT){
		errorConditions[ERROR_MOTOR_ENCODER] = SENSOR_NOMINAL;
	}

	previousMotorValue = *rigid1.ex.enc_ang;
}

static void checkMotorThermo(Act_s *actx) {
	errorConditions[ERROR_MOTOR_THERMO] = SENSOR_NOMINAL;
}

static void checkPCBThermo(Act_s *actx) {
	errorConditions[ERROR_PCB_THERMO] = SENSOR_NOMINAL;
}

static void checkEMG(Act_s *actx) {
	errorConditions[ERROR_EMG] = SENSOR_NOMINAL;
}

//check values against limits
static void checkBatteryBounds(Act_s *actx) {

	if (rigid1.re.vb <= UVLO_NOTIFY && rigid1.re.vb >= UV_USB_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_BELOW;
	} else if (rigid1.re.vb >= UVHI_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_ABOVE;
	} else {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_NOMINAL;
	}
}

static void checkTorqueMeasuredBounds(Act_s *actx) {
	//if sensors are invalid, torque value is invalid
	if (errorConditions[ERROR_LDC] || errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[WARNING_TORQUE_MEASURED] = SENSOR_INVALID;
	} else {
		if (actx->jointTorque >= ABS_TORQUE_LIMIT_INIT) {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_ABOVE;
		} else if (actx->jointTorque <= -ABS_TORQUE_LIMIT_INIT) {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_BELOW;
		} else {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_NOMINAL;
		}
	}
}

static void checkCurrentMeasuredBounds(Act_s *actx) {
	if (abs(actx->motCurr) >= CURRENT_LIMIT_INIT) {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_ABOVE;
	} else {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_NOMINAL;
	}
}

static void checkTemperatureBounds(Act_s *actx) {
	if (actx->regTemp >= PCB_TEMP_LIMIT_INIT){
		errorConditions[ERROR_PCB_THERMO] = VALUE_ABOVE;
	}else{
		errorConditions[ERROR_PCB_THERMO] = VALUE_NOMINAL;
	}
}


static void checkJointAngleBounds(Act_s *actx) {
	if (errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[WARNING_JOINTANGLE_SOFT] = SENSOR_INVALID;
		errorConditions[WARNING_JOINTANGLE_SOFT] = SENSOR_INVALID;
	} else {
		//soft angle check
		if (actx->jointAngleDegrees <= JOINT_MIN_SOFT_DEGREES) {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_BELOW;
		} else if (actx->jointAngleDegrees >= JOINT_MAX_SOFT_DEGREES) {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_ABOVE;
		} else {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_NOMINAL;
		}

		//hard angle check
		if (actx->jointAngleDegrees <= JOINT_MIN_HARD_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_BELOW;
		} else if (actx->jointAngleDegrees >= JOINT_MAX_HARD_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_ABOVE;
		} else {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_NOMINAL;
		}
	}
}

static void checkPersistentError(Act_s *actx) {
	//time limit for errors
}
/*
 * PUBLIC FUNCTIONS
 */

void setLEDStatus(uint8_t l1_status, uint8_t l2_status, uint8_t l3_status) {
	l1 |= l1_status;
	l2 |= l2_status;
	l3 |= l3_status;
}

void clearLEDStatus(void) {
	l1 = 0;
	l2 = 0;
	l3 = 0;
}

void overrideLED(uint8_t r, uint8_t g, uint8_t b) {
	if (r >= 1) {
		LEDR(1);
	} else {
		LEDR(0);
	}

	if (g >= 1) {
		LEDG(1);
	} else {
		LEDG(0);
	}

	if (b >= 1) {
		LEDB(1);
	} else {
		LEDB(0);
	}
}


int8_t getMotorMode(void){
	return motorMode;
}

int8_t* getSafetyConditions(void) {
	return errorConditions;
}

int16_t getSafetyFlags(void) {
	return safetyFlags;
}


//check for general errors
int actuatorIsCorrect() {
	int16_t* devID16 = getDeviceId16();

	if (safetyFlags) {
		return 0;
	}

	//check all six values of stm32ID match
	for (int i = 0; i < 6; i++) {
		if (*(devID16 + i) != stm32ID[i]) {
			return 0;
		}
	}

	return 1;
}



void checkSafeties(Act_s *actx) {
	safetyFlags = 0; //reset this upon entering a check
	//TODO:

	checkLoadcell(actx);
	checkJointEncoder(actx);
	checkMotorEncoder(actx);
	checkMotorThermo(actx);
	checkPCBThermo(actx);
	checkEMG(actx);

	checkBatteryBounds(actx);
	checkTorqueMeasuredBounds(actx);
	checkCurrentMeasuredBounds(actx);
	checkJointAngleBounds(actx); //hard and soft limits
	checkTemperatureBounds(actx);

	checkPersistentError(actx);

	//set our safety bitmap for streaming and checking purposes
	//TODO: consider optimizing if there are future processing constraints
	for (int i = 0; i < ERROR_ARRAY_SIZE; i++) {
		if (errorConditions[i]) {
			SET_MAP_HIGH(i, safetyFlags);
		}
	}
}

/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t handleSafetyConditions(void) {
	int8_t isBlocking = 1;

	//TODO figure out if MODE_DISABLED should be blocking/ how to do it
	if (errorConditions[ERROR_MOTOR_ENCODER] != SENSOR_NOMINAL)
		motorMode = MODE_DISABLED;
	else if (errorConditions[ERROR_JOINT_ENCODER] ||
			errorConditions[ERROR_LDC])
		motorMode = MODE_PASSIVE;
	else if (errorConditions[ERROR_PCB_THERMO] == VALUE_ABOVE ||
			errorConditions[ERROR_MOTOR_THERMO] != VALUE_NOMINAL)
		motorMode = MODE_THROTTLED;
	else
		motorMode = MODE_ENABLED;

	if (errorConditions[WARNING_TORQUE_MEASURED] != VALUE_NOMINAL){
		setLEDStatus(0,1,0); //flashing yellow
	}

	if (errorConditions[WARNING_JOINTANGLE_SOFT] != VALUE_NOMINAL){
		setLEDStatus(0,1,0);//flashing yellow
	}

	if (errorConditions[WARNING_BATTERY_VOLTAGE] != VALUE_NOMINAL){
		setLEDStatus(0,1,0);//flashing yellow (TODO LED function is not working)
	}

	if (errorConditions[ERROR_PCB_THERMO] != VALUE_NOMINAL){
		setLEDStatus(0,1,0);//flashing yellow
	}

	switch (motorMode){
		case MODE_DISABLED:
			disable_motor();
			break;
		case MODE_PASSIVE:
			actuate_passive_mode(); //position control to neutral angle
			break;
		case MODE_THROTTLED:
			throttle_current();
			isBlocking = 0;
			break;
		case MODE_ENABLED:
			isBlocking = 0;
			break;
		default:
			break;
	}

	return isBlocking;

}
