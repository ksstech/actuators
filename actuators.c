/*
 * actuators.c - actuator support for LEDs, SSRs, Relays etc with hard & soft PWM support
 */

#include	"hal_variables.h"

#if		(configHAL_XXX_XXX_OUT > 0)
#include 	"actuators.h"
#include	"FreeRTOS_Support.h"
#include	"endpoints.h"
#include	"rules_engine.h"

#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"
#include	"x_errors_events.h"

#include	"hal_gpio.h"

#if		(halHAS_PCA9555 > 0)
	#include	"pca9555.h"
#endif

#include	"esp_attr.h"

#include	<string.h>
#include	<float.h>
#include	<limits.h>

// ############################### BUILD: debug configuration options ##############################

#define	debugFLAG					0xC000

#define	debugPHYS					(debugFLAG & 0x0001)
#define	debugFUNC					(debugFLAG & 0x0002)
#define	debugUSER					(debugFLAG & 0x0004)
#define	debugDUTY					(debugFLAG & 0x0008)

#define	debugDUTY_CYCLE				(debugFLAG & 0x0010)
#define	debugTIMING_TASK			(debugFLAG & 0x0020)
#define	debugTIMING_STAGES			(debugFLAG & 0x0040)
#define	debugREMTIME				(debugFLAG & 0x0080)

#define	debugFUNCTIONS				(debugFLAG & 0x0100)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)


// ############################################ Macros #############################################

#define	systimerACT_ALL_MASK (1<<systimerACT_S0 | 1<<systimerACT_S1 | 1<<systimerACT_S2 | 1<<systimerACT_S3 | 1<<systimerACT_SX)

// #################################### Global & Local variables ###################################

const char * const StageNames[]	= { "FI ", "ON ", "FO ", "OFF" } ;
const char * const ActTypeNames[]	= { "SoC/DIG", "SoC/PWM", "SoC/ANA", "I2C/DIG", "I2C/PWM", "I2C/ANA", "SPI/DIG", "SPI/PWM", "SPI/ANA" } ;

act_init_t	ActInit[actNUMBER] = {						// Static configuration info
#if		(HW_VARIANT == HW_AC00)
	[ LED0 ] = {	actI2C_DIG,	pinPCA9555_7,	},
	[ LED1 ] = {	actI2C_DIG,	pinPCA9555_6,	},
	[ LED2 ] = {	actI2C_DIG,	pinPCA9555_5,	},
	[ LED3 ] = {	actI2C_DIG,	pinPCA9555_4,	},
	[ LED4 ] = {	actI2C_DIG,	pinPCA9555_3,	},
	[ LED5 ] = {	actI2C_DIG,	pinPCA9555_2,	},
	[ LED6 ] = {	actI2C_DIG,	pinPCA9555_1,	},
	[ LED7 ] = {	actI2C_DIG,	pinPCA9555_0,	},

	[RELAY0] = {	actI2C_DIG,	pinPCA9555_8,	},
	[RELAY1] = {	actI2C_DIG,	pinPCA9555_9,	},
	[RELAY2] = {	actI2C_DIG,	pinPCA9555_10,	},
	[RELAY3] = {	actI2C_DIG,	pinPCA9555_11,	},
	[RELAY4] = {	actI2C_DIG,	pinPCA9555_12,	},
	[RELAY5] = {	actI2C_DIG,	pinPCA9555_13,	},
	[RELAY6] = {	actI2C_DIG,	pinPCA9555_14,	},
	[RELAY7] = {	actI2C_DIG,	pinPCA9555_15,	},

#elif	(HW_VARIANT == HW_AC01)
	[ LED0 ] = {	actI2C_DIG,	pinPCA9555_0,	},
	[ LED1 ] = {	actI2C_DIG,	pinPCA9555_1,	},
	[ LED2 ] = {	actI2C_DIG,	pinPCA9555_2,	},
	[ LED3 ] = {	actI2C_DIG,	pinPCA9555_3,	},
	[ LED4 ] = {	actI2C_DIG,	pinPCA9555_4,	},
	[ LED5 ] = {	actI2C_DIG,	pinPCA9555_5,	},
	[ LED6 ] = {	actI2C_DIG,	pinPCA9555_6,	},
	[ LED7 ] = {	actI2C_DIG,	pinPCA9555_7,	},

	[RELAY0] = {	actI2C_DIG,	pinPCA9555_8,	},
	[RELAY1] = {	actI2C_DIG,	pinPCA9555_9,	},
	[RELAY2] = {	actI2C_DIG,	pinPCA9555_10,	},
	[RELAY3] = {	actI2C_DIG,	pinPCA9555_11,	},
	[RELAY4] = {	actI2C_DIG,	pinPCA9555_12,	},
	[RELAY5] = {	actI2C_DIG,	pinPCA9555_13,	},
	[RELAY6] = {	actI2C_DIG,	pinPCA9555_14,	},
	[RELAY7] = {	actI2C_DIG,	pinPCA9555_15,	},

#elif	(HW_VARIANT == HW_EM1P2) || (HW_VARIANT == HW_EM3P2)
	// [ LED0 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_0,	}, // cannot use, pin conflicts with SCL

#elif	(HW_VARIANT == HW_WROVERKIT)			// WROVER-KIT
	[ LED0 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_0,	},
	[ LED1 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_1,	},
	[ LED2 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_2,	},

#elif	(HW_VARIANT == HW_DOITDEVKIT)			// DoIT DevKIt v1
	[ LED0 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_0,	},

#elif	(HW_VARIANT == HW_EBOX)
	[ LED1 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_0,	},
	[ LED2 ] = {	actSOC_DIG,	halGPIO_DIG_OUT_1,	},

#endif
} ;

/* In order to optimise MCU utilization, especially since the actuator task is set to run EVERY 1mS,
 * we try to only start the task if there is something to do. Hence, the task is started every time
 * a LOAD command is executed. During the running of the task the 'ActuatorsRunning' variable is set
 * to the number of actuators serviced during that task cycle. At the end of the task cycle, if NO
 * actuators were serviced, the task RUN status is cleared, only to be restarted with the next LOAD.
 */
uint8_t	ActuatorsRunning = 0 ;
act_info_t	sAI[actNUMBER] ;

/* Objective is to facilitate a number of predefined sequences with a simple single actuation command.
 * SEQUENCE Ch# m0 m1 m2 m3 etc mZ will result in the first mode (m0) being loaded immediately with the
 * rest of the mode numbers going into a queue to be loaded sequentially after completion of the previous
 */
act_seq_t	sAS[actSEQ_NUM]	= {
//					Rpt		tFI			tON			tFO			tOFF
	[actSEQ_0]	= { 5,		0,			1000,		0,			1000,		} ,		// LED 0.50Hz	x5	10Sec		OK
	[actSEQ_1]	= { 1,		0,			0,			0,			25000, 		} ,		// LED OFF		x1	25Sec		BUSY

	[actSEQ_2]	= { 5,		0,			1000,		0,			1000,		} ,		// LED 0.50Hz	x5	10Sec		OK
	[actSEQ_3]	= { 1,		0,			0,			0,			275000, 	} ,		// LED OFF		x1	275Sec		BUSY
	[actSEQ_4]	= { 30,		0,			500,		0,			500,		} ,		// LED 1.00Hz	x30 30Sec		WARNING
	[actSEQ_5]	= {	15,		500,		0,			500,		0,			} ,		// LED 1.00Hz	x15	15Sec		WAIT !!!

	[actSEQ_6]	= { 8,		0,			250,		0,			250,		} ,		// LED 2.00 Hz	x8	4Sec
	[actSEQ_7]	= { 6,		0,			500,		0,			500,		} ,		// LED 1.00 Hz	x6	6Sec
	[actSEQ_8]	= { 4,		0,			750,		0,			750,		} ,		// LED 0.67 Hz	x4	6Sec
	[actSEQ_9]	= { 3,		0,			1000,		0,			1000,		} ,		// LED 0.50 Hz	x3	2Sec
} ;

// ################################# local/static functions ########################################

int32_t	xActuatorLogError(const char * pFname, uint8_t eChan) {
	xSyslog(SL_MOD2LOCAL(SL_SEV_ERROR),	pFname, "type=%d '%s'", ActInit[eChan].Type, ActTypeNames[ActInit[eChan].Type]) ;
	return erFAILURE ;
}

// ##################### Hardware dependent (DIG/PWM/ANA) Actuator functions #######################

void IRAM_ATTR vActuateSetLevelDIG(uint8_t eChan, uint8_t NewState) {
	switch(ActInit[eChan].Type) {					// handle hardware dependent component
#if		(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
		halGPIO_DIG_OUT_SetState(ActInit[eChan].halGPIO, NewState) ;
		break ;
#endif

#if		(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
		halGPIO_PWM_OUT_SetState(ActInit[eChan].halGPIO, NewState) ;
		break ;
#endif

#if		(configHAL_I2C_DIG_OUT > 0)
	case actI2C_DIG:
	#if	(halHAS_PCA9555 == 1)							// To minimise I2C traffic, update bit array but do NOT write to device
		pca9555DIG_OUT_SetState(ActInit[eChan].halGPIO, NewState, 0) ;
	#else
		myASSERT(0) ;
	#endif
		break ;
#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan) ;
	}
}

int32_t	xActuateGetLevelDIG(uint8_t eChan) {
	int32_t iRV = erFAILURE ;
	switch(ActInit[eChan].Type) {						// handle hardware dependent component
#if		(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
		iRV = halGPIO_DIG_OUT_GetState(ActInit[eChan].halGPIO) ;
		break ;
#endif

#if		(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
		iRV = halGPIO_PWM_OUT_GetState(ActInit[eChan].halGPIO) ;
		break ;
#endif

#if		(configHAL_I2C_DIG_OUT > 0)
	case actI2C_DIG:
	#if	(halHAS_PCA9555 == 1)
		iRV = pca9555DIG_OUT_GetState(ActInit[eChan].halGPIO) ;
	#else
		myASSERT(0) ;
	#endif
		break ;
#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan) ;
	}
	return iRV ;
}

/**
 * vActuatorSetFrequency() - configure a PWM channel for a specific frequency
 * \brief		The timer will be stopped and a new frequency will be configured
 * 				The timer will NOT be restarted until a new duty cycle is configured
 * \param[in]	Chan - logical PWM channel
 * \param[in]	Frequency - desired frequency trimmed to be within the supported range
 * \return		none
 */
int32_t	xActuatorSetFrequency(uint8_t eChan, uint32_t Frequency) {
	if (eChan >= NumActuator) {
		return erFAILURE ;
	}
	switch(ActInit[eChan].Type) {			// handle hardware dependent component
#if		(configHAL_XXX_DIG_OUT > 0)
	#if		(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
	#endif

	#if		(configHAL_I2C_DIG_OUT > 0)
	case actI2C_DIG:
	#endif

	#if		(configHAL_SPI_DIG_OUT > 0)
	case actSPI_DIG:
	#endif

		FIT2RANGE(configACTUATE_DIG_MIN_FREQ, Frequency, configACTUATE_DIG_MAX_FREQ, uint32_t) ;
		sAI[eChan].Divisor	= (MILLIS_IN_SECOND / ACTUATE_TASK_PERIOD) / Frequency ;
 		break ;
#endif

#if		(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
		FIT2RANGE(configHAL_PWM_MIN_FREQ, Frequency, configHAL_PWM_MAX_FREQ, uint32_t) ;
		sAI[eChan].Divisor	= (configHAL_PWM_CLOCK_HZ / Frequency) ;
		halGPIO_PWM_OUT_SetFrequency(ActInit[eChan].halGPIO, sAI[eChan].Divisor - 1) ;
		break ;
#endif

	default:
		return xActuatorLogError(__FUNCTION__, eChan) ;
	}
	return erSUCCESS ;
}

/**
 * vActuatorSetDC() - Recalc & set duty cycle (brightness/speed level)
 * @param Chan		logical (soft) PWM channel
 */
void IRAM_ATTR vActuatorSetDC(uint8_t eChan, int8_t CurDC) {
	act_info_t * pAI = &sAI[eChan] ;
	pAI->CurDC	= CurDC ;

	switch(ActInit[eChan].Type) {
#if		(configHAL_XXX_DIG_OUT > 0)						// All (GPIO + I2C + SPI) DIGital type actuators
	#if	(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
	#endif

	#if	(configHAL_I2C_DIG_OUT > 0) && (halHAS_PCA9555 == 1)
	case actI2C_DIG:
	#endif

	#if	(configHAL_SPI_DIG_OUT > 0)
	case actSPI_DIG:
	#endif

		switch(pAI->StageNow) {
		case actSTAGE_FI:			pAI->Match	= pAI->MaxDC - pAI->CurDC ;			break ;
		case actSTAGE_ON:			pAI->Match	= pAI->MinDC ;						break ;
		case actSTAGE_FO:			pAI->Match	= pAI->MaxDC - pAI->CurDC ;			break ;
		case actSTAGE_OFF:			pAI->Match	= pAI->MaxDC ;						break ;
		}
		vActuateSetLevelDIG(eChan, (pAI->Count >= pAI->Match) ? 1 : 0) ;
		break ;
#endif

#if		(configHAL_XXX_PWM_OUT > 0)						// All (GPIO + I2C + SPI) PWM type actuators
	#if	(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
		pAI->Match = u32ScaleValue(CurDC, pAI->MinDC, pAI->MaxDC, configHAL_PWM_MIN_FREQ, configHAL_PWM_MAX_FREQ) ;
		halGPIO_PWM_OUT_SetCycle(ActInit[eChan].halGPIO, pAI->Match) ;
		break ;
	#endif

	#if	(configHAL_I2C_PWM_OUT > 0)
	case actSOC_ANA:	myASSERT(0);		break ;
	#endif

	#if	(configHAL_SPI_PWM_OUT > 0)
	case actSOC_ANA:	myASSERT(0);		break ;
	#endif
#endif

#if		(configHAL_XXX_ANA_OUT > 0)						// All (GPIO + I2C + SPI) ANAlog type actuators
	#if	(configHAL_GPIO_ANA_OUT > 0)
	case actSOC_ANA:
		myASSERT(0);
		break ;
	#endif

	#if	(configHAL_I2C_ANA_OUT > 0)
	case actI2C_ANA:
		myASSERT(0);
		break ;
	#endif

	#if	(configHAL_SPI_ANA_OUT > 0)
	case actSPI_ANA:
		myASSERT(0);
		break ;
	#endif
#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan) ;
	}
	IF_EXEC_1(debugDUTY, vActuatorReportChan, eChan) ;
}

void IRAM_ATTR vActuatorUpdateCurDC(act_info_t * pAI) {
	pAI->CurDC	= pAI->MinDC ;
	pAI->Match	= pAI->tNOW ;
	if (pAI->tXXX[pAI->StageNow]) {
		pAI->CurDC	+= ((pAI->tNOW * pAI->DelDC) / pAI->tXXX[pAI->StageNow]) ;
		pAI->Match	= pAI->tNOW / ( pAI->tXXX[pAI->StageNow] / pAI->Divisor) ;
	}
}

/**
 * xActuatorConfig() -  configure the hardware pin associated with a channel
 * \brief		Uses the definitions in the hal_gpio module to define the specific pin,
 * 				its configuration and (optionally) the associated timer module for hard PWM
 * \param[in]	Channel
 * \return		None
 */
int32_t	xActuatorConfig(uint8_t Chan) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	switch(ActInit[Chan].Type) {					// handle hardware dependent component
#if		(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
		halGPIO_DIG_OUT_Config(ActInit[Chan].halGPIO) ;
		xActuatorSetFrequency(Chan, configACTUATE_DIG_DEF_FREQ) ;
		break ;
#endif

#if		(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
		halGPIO_PWM_OUT_Config(ActInit[Chan].halGPIO) ;
		xActuatorSetFrequency(Chan, configHAL_PWM_DEF_FREQ) ;
		break ;
#endif

#if		(configHAL_I2C_DIG_OUT > 0)
	case actI2C_DIG:
	#if	 (halHAS_PCA9555 == 1)
		pca9555DIG_OUT_Config(ActInit[Chan].halGPIO) ;
		xActuatorSetFrequency(Chan, configACTUATE_DIG_DEF_FREQ) ;
	#else
		myASSERT(0) ;
	#endif
		break ;
#endif

	default:
		return xActuatorLogError(__FUNCTION__, Chan) ;
	}
	act_info_t * psAI = &sAI[Chan] ;
	memset(psAI->Seq, 0xFF, SIZEOF_MEMBER(act_info_t, Seq)) ;
	psAI->CurDC		= psAI->MinDC		= actMIN_DUTYCYCLE ;
	psAI->MaxDC		= psAI->DelDC		= actMAX_DUTYCYCLE ;
	psAI->StageBeg	= psAI->StageNow	= actSTAGE_FI ;
	psAI->ChanNum	= Chan ;
	psAI->ConfigOK	= true ;
	vActuatorSetDC(Chan, actMIN_DUTYCYCLE) ;
	IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	return erSUCCESS ;
}

void	vActuatorsConfig(void) {
	for(uint8_t Chan = 0; Chan < NumActuator; xActuatorConfig(Chan++)) ;
}

/* ############################ Actuator alerting support functions ################################
 * Start (OFF -> FI/ON)
 * Stop (ON/FO -> OFF)
 * Stage (OFF -> FI -> ON -> FO -> OFF)
 * Event Blocked due to any reason, especially pending restart
 */

/**
 * xActuatorAlert() - UNTESTED
 * @param pAI
 * @return
 */
int32_t	IRAM_ATTR xActuatorAlert(act_info_t * pAI, uint8_t Type, uint8_t Level) {
	IF_myASSERT(debugPARAM, URI_ACT < URI_MAX_VAL && Level < alertLEVEL_NUM) ;
	epi_t	sEI = { 0 } ;
	event_t	sEvent	= { 0 } ;
	alert_t	sAlert	= { 0 } ;
	ubuf_t	sBuf	= { 0 } ;
	vEpGetInfoWithIndex(&sEI, URI_ACT) ;
	if (sEI.psES == NULL || sEI.psEW == NULL) {
		return erFAILURE ;
	}
	sEI.psEvent		= &sEvent ;
	sEI.psAlert		= &sAlert ;
	sEI.psUB		= &sBuf ;
	// configure the type, level and supporting field/channel info
	sAlert.Type		= Type ;
	sAlert.Level	= Level ;
	sAlert.pvValue	= pAI ;
	xEpGenerateAlert(&sEI) ;
	return erSUCCESS ;
}

// XXX add API support to en/disable from rules
int32_t	xActuatorSetAlertStage(uint8_t Chan, uint8_t State) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	sAI[Chan].alertStage = State ? 1 : 0 ;
	return erSUCCESS ;
}

// XXX add API support to en/disable from rules
int32_t	xActuatorSetAlertDone(uint8_t Chan, uint8_t State) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	sAI[Chan].alertDone = State ? 1 : 0 ;
	return erSUCCESS ;
}

// XXX add API support to en/disable from rules
int32_t	xActuatorSetStartStage(uint8_t Chan, uint8_t Stage) {
	if (Chan >= NumActuator || Stage >= actSTAGE_NUM || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	sAI[Chan].StageBeg	= TRIMMED(actSTAGE_OFF, Stage, actSTAGE_FO, int8_t) ;
	return erSUCCESS ;
}

// XXX add API support to re/set from rules
int32_t	vActuatorSetMinMaxDC(uint8_t Chan, int8_t iMin, int8_t iMax) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	if (iMin > iMax) {
		SWAP(iMin, iMax, uint8_t) ;
	}
	act_info_t	* pAI = &sAI[Chan] ;
	pAI->MinDC = iMin > actMAX_DUTYCYCLE ? actMAX_DUTYCYCLE : iMin ;
	pAI->MaxDC = iMax > actMAX_DUTYCYCLE ? actMAX_DUTYCYCLE : iMax ;
	IF_PRINT(debugDUTY_CYCLE, "Done C=%d  Min=%d -> %d  Max=%d -> %d\n", Chan, iMin, pAI->MinDC, iMax, pAI->MaxDC) ;
	return erSUCCESS ;
}

// ########################## Hardware INDEPENDENT Actuator functions ##############################

/**
 * ActuatorSetTiming()
 * @brief		timing values are supplied in mSec, converted and stored as ticks
 * @param[in]	Chan, tOFF, tFI, tON, tFO
 * @return		erFAILURE or erSUCCESS
 */
int32_t	xActuatorSetTiming(uint8_t Chan, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) {
	act_info_t	* pAI = &sAI[Chan] ;
	if (Chan >= NumActuator || pAI->ConfigOK == 0) {
		return erFAILURE ;
	}
	if (pAI->Blocked) {
		xActuatorAlert(pAI, alertTYPE_ACT_BLOCKED, alertLEVEL_ERROR) ;
		return erFAILURE ;
	}
	while (pAI->Busy) {
		vTaskDelay(pdMS_TO_TICKS(1)) ;
	}
	pAI->Busy = 1 ;
	// set configuration to max 1 day...
	pAI->tFI		= pdMS_TO_TICKS(tFI > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFI) ;
	pAI->tON		= pdMS_TO_TICKS(tON > MILLIS_IN_DAY ? MILLIS_IN_DAY : tON) ;
	pAI->tFO		= pdMS_TO_TICKS(tFO > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFO) ;
	pAI->tOFF		= pdMS_TO_TICKS(tOFF > MILLIS_IN_DAY ? MILLIS_IN_DAY : tOFF) ;
	pAI->Busy = 0 ;
	IF_PRINT(debugTRACK, "[ACT] SetTiming C=%d  %u -> %u -> %u -> %u\n", Chan, tFI, tON, tFO, tOFF) ;
	return erSUCCESS ;
}

int32_t	xActuatorStart(uint8_t Chan, uint32_t Repeats) {
	act_info_t	* pAI = &sAI[Chan] ;
	if (Chan >= NumActuator || pAI->ConfigOK == 0) {
		return erFAILURE ;
	}
	if (pAI->Blocked) {
		xActuatorAlert(pAI, alertTYPE_ACT_BLOCKED, alertLEVEL_ERROR) ;
		return erFAILURE ;
	}
	pAI->tNOW		= pAI->Count		= 0 ;
	pAI->StageNow	= pAI->StageBeg ;
	vActuatorUpdateCurDC(pAI) ;
	vActuatorSetDC(Chan, pAI->CurDC) ;
	pAI->Rpt		= Repeats ;
	IF_PRINT(debugTRACK, "[ACT] Start C=%d R=%d\n", Chan, Repeats) ;
	xRtosSetStateRUN(taskACTUATE) ;
	return erSUCCESS ;
}

int32_t	xActuatorStop(uint8_t Chan) {
	act_info_t * pAI = &sAI[Chan] ;
	if (Chan >= NumActuator || pAI->Blocked) {
		return erFAILURE ;
	}
	// reset ONLY the tXXX values (incl Rpt + tNow)
	memset(&pAI->tXXX, 0, SIZEOF_MEMBER(act_info_t, tXXX)) ;
	memset(&pAI->Seq, 0xFF, SIZEOF_MEMBER(act_info_t, Seq)) ;
	pAI->StageNow	= pAI->StageBeg ;
	pAI->alertDone	= pAI->alertStage	= 0 ;
	vActuatorSetDC(Chan, 0) ;
	IF_PRINT(debugTRACK, "[ACT] Stop C=%d\n", Chan) ;
	return erSUCCESS ;
}

uint32_t xActuatorPause(uint8_t Chan) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return 0xFFFFFFFF ;
	}
	taskDISABLE_INTERRUPTS() ;
	uint32_t CurRpt		= sAI[Chan].Rpt ;
	sAI[Chan].Rpt	= 0 ;
	taskENABLE_INTERRUPTS() ;
	return CurRpt;
}

int32_t	xActuatorUnPause(uint8_t Chan, uint32_t CurRpt) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	sAI[Chan].Rpt = CurRpt ;
	return erSUCCESS ;
}

int32_t	xActuatorBlock(uint8_t Chan) {
	if (Chan >= NumActuator) {
		return erFAILURE ;
	}
	sAI[Chan].Blocked = 1 ;
	return erSUCCESS ;
}

int32_t	xActuatorUnBlock(uint8_t Chan) {
	if (Chan >= NumActuator) {
		return erFAILURE ;
	}
	sAI[Chan].Blocked = 0 ;
	return erSUCCESS ;
}

int32_t	xActuatorLoad(uint8_t Chan, uint32_t Rpt, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) {
	int32_t iRV = xActuatorStop(Chan) ;
	if (iRV == erSUCCESS) {
		iRV = xActuatorSetTiming(Chan, tFI, tON, tFO, tOFF) ;
		if (iRV == erSUCCESS) {
			iRV = xActuatorStart(Chan, Rpt) ;
		}
		IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	}
	return iRV ;
}

/**
 * xActuatorLoadSequence()	load up to a maximum of actMAX_SEQUENCE sequence numbers to the
 * 							sequence table, overwriting any existing (pending) sequences
 *							Expects a full array of actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param Chan		Channel number
 * @param paSeq		pointer to array of sequence numbers
 * @return
 */
int32_t	xActuatorLoadSequence(uint8_t Chan, uint8_t * paSeq) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	act_info_t * psAI = &sAI[Chan] ;
	int32_t Idx ;
	for (Idx = 0; Idx < actMAX_SEQUENCE; ++Idx) {
		if (*paSeq < actSEQ_NUM) {						// if a valid SEQuence number
			psAI->Seq[Idx] = *paSeq++ ;					// store it
		} else {
			psAI->Seq[Idx] = 0xFF ;						// if not, mark it an unused terminator
			break ;										// and go no further
		}
	}
	IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	return Idx ;
}

/**
 * xActuatorQueSequence()	append additional sequence numbers to the end of the sequence table
 *							Expects an array of up to actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param Chan		Channel number
 * @param paSeq		pointer to array of sequence numbers
 * @return			erFAILURE if none appended else number of sequences appended
 */
int32_t	xActuatorQueSequence(uint8_t Chan, uint8_t * paSeq) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	act_info_t * psAI = &sAI[Chan] ;
	uint8_t * paSeqBeg = &psAI->Seq[0] ;
	uint8_t * paSeqEnd = &psAI->Seq[actMAX_SEQUENCE] ;

	int32_t Idx = 0 ;
	while (*paSeqBeg != 0xFF) {							// skip over used space at start of sequence buffer
		if (paSeqBeg == paSeqEnd)
			return erFAILURE ;
		++paSeqBeg ;
		++Idx ;
	}
	for (; Idx < actMAX_SEQUENCE; ++Idx) {
		if (*paSeq < actSEQ_NUM) {						// if a valid SEQuence number
			*paSeqBeg++	= *paSeq++ ;					// store it
		} else {
			*paSeqBeg++ = 0xFF ;						// if not, mark it an unused terminator
			break ;										// and go no further
		}
	}

	IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	return Idx ;
}

int32_t	xActuatorUpdate(uint8_t Chan, int32_t Rpt, int32_t tFI, int32_t tON, int32_t tFO, int32_t tOFF) {
	if (Chan >= NumActuator || sAI[Chan].Blocked) {
		return erFAILURE ;
	}
	uint32_t CurRpt = xActuatorPause(Chan) ;
	act_info_t * pAI = &sAI[Chan] ;
	while (pAI->Busy) vTaskDelay(pdMS_TO_TICKS(1)) ;
	pAI->Busy = 1 ;
	pAI->tFI		+= (tFI * configTICK_RATE_HZ) / MILLIS_IN_SECOND ;
	pAI->tON		+= (tON * configTICK_RATE_HZ) / MILLIS_IN_SECOND ;
	pAI->tFO		+= (tFO * configTICK_RATE_HZ) / MILLIS_IN_SECOND ;
	pAI->tOFF		+= (tOFF * configTICK_RATE_HZ) / MILLIS_IN_SECOND ;
	pAI->Busy = 0 ;
	int32_t	iRV ;
	if (CurRpt == UINT32_MAX) {
		iRV = xActuatorUnPause(Chan, CurRpt) ;
	} else {
		iRV = xActuatorUnPause(Chan, CurRpt + Rpt) ;
	}
	IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	return iRV ;
}

int32_t	xActuatorAdjust(uint8_t Chan, uint32_t Stage, int32_t Adjust) {
	act_info_t * pAI = &sAI[Chan] ;
	if (Chan >= NumActuator || Stage >= actSTAGE_NUM || pAI->Blocked) {
		return erFAILURE ;
	}
	taskDISABLE_INTERRUPTS() ;					// XXX might be able to remove if Busy flag works
	Adjust = (Adjust * configTICK_RATE_HZ) / MILLIS_IN_SECOND ;	// convert adjustment to Ticks
	while (pAI->Busy) {
		vTaskDelay(pdMS_TO_TICKS(1)) ;
	}
	pAI->Busy = 1 ;
	uint32_t CurVal = pAI->tXXX[Stage] ;			// save the selected stage value
	uint32_t NewVal = CurVal + Adjust ;
	if (Adjust < 0) {
		pAI->tXXX[Stage]	= NewVal < CurVal ? NewVal : 0 ;
	} else {
		pAI->tXXX[Stage]	= NewVal > CurVal ? NewVal : UINT32_MAX ;
	}
	pAI->Busy = 0 ;
	taskENABLE_INTERRUPTS() ;
	IF_EXEC_1(debugTRACK, vActuatorReportChan, Chan) ;
	return erSUCCESS ;
}

int32_t	xActuatorToggle(uint8_t Chan) {
	act_info_t * pAI = &sAI[Chan] ;
	if (Chan >= NumActuator || pAI->Blocked) {
		return erFAILURE ;
	}
	taskDISABLE_INTERRUPTS() ;					// XXX might be able to remove if Busy flag works
	while (pAI->Busy) {
		vTaskDelay(pdMS_TO_TICKS(1)) ;
	}
	pAI->Busy = 1 ;
	SWAP(pAI->tFI, pAI->tFO, uint32_t) ;
	SWAP(pAI->tON, pAI->tOFF, uint32_t) ;
	pAI->Busy = 0 ;
	taskENABLE_INTERRUPTS() ;
	return erSUCCESS ;
}

int32_t	xActuatorBreath(uint8_t Chan) {
	return xActuatorLoad(Chan, UINT32_MAX, 750, 750, 750, 750) ;
}

int32_t	vActuatorPanic(uint8_t Chan) {
	return xActuatorLoad(Chan, UINT32_MAX, 150, 150, 150, 150) ;
}

int32_t	vActuatorOn(uint8_t Chan) {
	return xActuatorLoad(Chan, UINT32_MAX, 0, UINT32_MAX, 0, 0) ;
}

int32_t	vActuatorOff(uint8_t Chan) {
	return xActuatorLoad(Chan, UINT32_MAX, 0, 0, 0, UINT32_MAX) ;
}

// ############################## Rules interface to Actuator table ################################

int32_t	xActuatorVerifyParameters(uint8_t Chan, uint8_t Field) {
	if (Chan >= NumActuator || OUTSIDE(selACT_FIRST, Field, selACT_LAST, uint8_t) || sAI[Chan].Blocked) {
		SL_ERR("Invalid actuator(%d) / field (%d) / status (%d)", Chan, Field, sAI[Chan].Blocked) ;
		return erFAILURE ;
	}
	return erSUCCESS ;
}

double	dActuatorGetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE) {
		return 0.0 ;
	}
	px64Var->def.cv.vs	= vs32B ;
	px64Var->def.cv.vt	= vtVALUE ;
	px64Var->def.cv.vf	= vfUXX ;
	px64Var->def.cv.vc	= 1 ;
	act_info_t * psAI = &sAI[Chan] ;
	x64_t x64Value ;
	if (Field < selACT_T_REM) {							// all these are real tXXX fields/stages
		x64Value.f64 					= psAI->tXXX[Field-selACT_FIRST] ;
		px64Var->val.x64.x32[0].u32 	= psAI->tXXX[Field-selACT_FIRST] ;
	} else {
		x64Value.f64 = (double) xActuatorGetRemainingTime(Chan) ;
		IF_PRINT(debugREMTIME, "F64=%f", x64Value.f64) ;
		px64Var->val.x64.x32[0].u32 = (uint32_t) x64Value.f64 ;
	}
	IF_PRINT(debugFUNCTIONS, "%s: C=%d  F=%d  I=%d  V=%'u\n", __FUNCTION__, Chan, Field, Field-selACT_FIRST, px64Var->val.x64.x32[0].u32) ;
	return x64Value.f64 ;
}

int32_t	xActuatorSetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE) {
		return erFAILURE ;
	}
	sAI[Chan].tXXX[Field-selACT_FIRST] = px64Var->val.x64.x32[0].u32 ;
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-selACT_FIRST, sAI[Chan].tXXX[Field-selACT_FIRST]) ;
	return erSUCCESS ;
}

int32_t	xActuatorUpdateFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE) {
		return erFAILURE ;
	}
	uint32_t CurVal = sAI[Chan].tXXX[Field-selACT_FIRST] ;
	if ((px64Var->val.x64.x32[0].i32 < 0) && (CurVal >= abs(px64Var->val.x64.x32[0].i32))) {
		CurVal	+= px64Var->val.x64.x32[0].i32 ;
	} else {
		CurVal	= 0 ;
	}
	sAI[Chan].tXXX[Field-selACT_FIRST] = CurVal ;
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-selACT_FIRST, sAI[Chan].tXXX[Field-selACT_FIRST]) ;
	return erSUCCESS ;
}

// ######################################## reporting functions ####################################

uint32_t xActuatorRunningCount (void) { return ActuatorsRunning ; }

uint64_t xActuatorGetRemainingTime(uint8_t Chan) {
	act_info_t * psAI = &sAI[Chan] ;
	if (psAI->Rpt == UINT32_MAX) {
		return UINT64_MAX ;								// indefinite/unlimited repeat ?
	}
	if (psAI->Rpt == 0) {
		return 0 ;
	}
	// calculate remaining time for full repeats
	taskDISABLE_INTERRUPTS() ;
	uint64_t u64Value = (psAI->Rpt > 1) ? (psAI->tFI + psAI->tON + psAI->tFO + psAI->tOFF) * (psAI->Rpt - 1) : 0 ;
	IF_PRINT(debugREMTIME, "Ch#%d: %llu", Chan, u64Value) ;

	// now add remaining time in current stage
	uint8_t Stage = psAI->StageNow ;
	do {
		u64Value	+= (Stage == psAI->StageNow) ? psAI->tXXX[Stage] - psAI->tNOW : psAI->tXXX[Stage] ;
		IF_PRINT(debugREMTIME, " -> s(%d): %llu", Stage, u64Value) ;
		++Stage ;
		Stage %= actSTAGE_NUM ;
	} while (Stage != psAI->StageBeg) ;

	// now add the time for the (optional) sequences
	for(int32_t Idx = 0; psAI->Seq[Idx] < actSEQ_NUM; ++Idx) {
		act_seq_t * psAS = &sAS[psAI->Seq[Idx]] ;
		u64Value	+= psAS->Rpt * (psAS->tFI + psAS->tON + psAS->tFO + psAS->tOFF) ;
		IF_PRINT(debugREMTIME, " -> I(%d): %llu", Idx, u64Value) ;
	}
	taskENABLE_INTERRUPTS() ;
	IF_PRINT(debugREMTIME, "\n") ;
	return u64Value ;
}

/**
 * Determines the longest remaining running time, not the sum of ALL
 * @return Remaining maximum actuator running time in uSecs
 */
uint64_t xActuatorGetMaxRemainingTime (void) {
	uint64_t u64Now, u64Max = 0.0 ;
	for (int32_t Chan = 0; Chan < NumActuator; ++Chan) {
		u64Now = xActuatorGetRemainingTime(Chan) ;
		if (u64Now > u64Max) {
			u64Max = u64Now ;
		}
	}
	return u64Max * MICROS_IN_MILLISEC ;
}

void	vActuatorReportSeq(uint8_t Seq) {
	act_seq_t * psAS = &sAS[Seq] ;
	if (Seq == 0) {
		printfx("%CSeq |Repeat|  tFI  |  tON  |  tFO  |  tOFF |%C\n", xpfSGR(colourFG_CYAN, 0, 0, 0), xpfSGR(attrRESET, 0, 0, 0)) ;
	}
	printfx(" %2d | %'#5u|%'#7u|%'#7u|%'#7u|%'#7u|\n", Seq, psAS->Rpt, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF) ;
}

void	vActuatorReportChan(uint8_t Chan) {
	act_info_t * psAI = &sAI[Chan] ;
	bool	bLevel = xActuateGetLevelDIG(Chan) ;
	if (Chan == 0) {
		printfx("%C Ch| LBb |Stage| Repeat|  tFI  |  tON  |  tFO  |  tOFF |  tNOW | Div Cnt Mtch| Min DC Max | Sequence%C\n",
				xpfSGR(colourFG_CYAN, 0, 0, 0), xpfSGR(attrRESET, 0, 0, 0)) ;
	}
	printfx(" %2d| %c%c%c | %s | %'#5d |%'#7d|%'#7d|%'#7d|%'#7d|%'#7d| %3d %3d %3d | %3d %3d %3d|",
						psAI->ChanNum,
						bLevel ? CHR_1 : CHR_0,
						psAI->Blocked ? CHR_B : CHR_SPACE,
						psAI->Busy ? CHR_b : CHR_SPACE,
						StageNames[psAI->StageNow], psAI->Rpt,
						psAI->tFI, psAI->tON, psAI->tFO, psAI->tOFF, psAI->tNOW,
						psAI->Divisor, psAI->Count, psAI->Match, psAI->MinDC, psAI->CurDC, psAI->MaxDC) ;
	if (psAI->Blocked == 0 && psAI->Seq[0] != 0xFF) {
		for(int32_t Idx = 0; Idx < actMAX_SEQUENCE && psAI->Seq[Idx] != 0xFF; ++Idx) {
			printfx("%02x ", psAI->Seq[Idx]) ;
		}
	}
	printfx("\n") ;
}

void	vTaskActuatorReport(void) {
	for (uint8_t Chan = 0; Chan < NumActuator; ++Chan) {
		vActuatorReportChan(Chan) ;
	}
	for (uint8_t Seq = 0; Seq < actSEQ_NUM; ++Seq) {
		vActuatorReportSeq(Seq) ;
	}
	printfx("Running=%u  maxDelay=%!.R\n\n", xActuatorRunningCount(), xActuatorGetMaxRemainingTime()) ;
}

/* ############################### actuator task & support functions ###############################
 * To provide transparent "soft" PWM support for DIGital OUTput pins.
 *	Background as follows:
 * 		To simulate PWM we will use a regularly running FreeRTOS task to toggle pins on/off
 * 		The brightness of the LED attached will be controlled by the ratio of pin LOW vs HIGH state
 * 		The smoothness of the LED change in brightness will be determined by ???
 * 	Logic as follows:
 * 		Working on ranges of 100 / 1000 / 10000 ticks per second
 * 		MaxCount	= configTICK_RATE_HZ / Frequency ~ number of ticks in a single (ON - OFF) cycle
 * 		Divisor		= Frequency
 */

int32_t	IRAM_ATTR xActuatorNextSequence(act_info_t * psAI) {
	uint8_t	NxtSeq = psAI->Seq[0] ;
	if (NxtSeq >= actSEQ_NUM) {
		return erFAILURE ;
	}
	IF_EXEC_1(debugTRACK, vActuatorReportSeq, NxtSeq) ;
	act_seq_t * psAS = &sAS[NxtSeq] ;
	// load values from sequence #
	int32_t iRV = xActuatorSetTiming(psAI->ChanNum, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF) ;
	if (iRV == erSUCCESS) {
		iRV = xActuatorStart(psAI->ChanNum, psAS->Rpt) ;
		if (iRV == erSUCCESS) {
			int32_t Idx ;
			for (Idx = 0 ; Idx < (actMAX_SEQUENCE - 1); ++Idx) {
				psAI->Seq[Idx] = psAI->Seq[Idx+1] ;
			}
			psAI->Seq[Idx] = 0xFF ;
		} else {
			myASSERT(0) ;								// what do we need to do if the START failed???
		}
	} else {
		myASSERT(0) ;									// what do we need to do if the SETTIMING failed???
	}
	return iRV ;
}

int32_t	IRAM_ATTR xActuatorNextStage(act_info_t * pAI) {
	int32_t iRV = erSUCCESS ;
	if ((pAI->alertStage == 1) && (pAI->tXXX[pAI->StageNow] > 0)) {
		xActuatorAlert(pAI, alertTYPE_ACT_STAGE, alertLEVEL_INFO) ;
	}
	if (++pAI->StageNow == actSTAGE_NUM) {
		pAI->StageNow = actSTAGE_FI ;
	}
	if (pAI->StageNow == pAI->StageBeg) {				// back at starting stage?
		if (pAI->Rpt != UINT32_MAX) {					// yes, but running unlimited repeats ?
			--pAI->Rpt ;								// No, decrement the repeat count
			if (pAI->Rpt == 0) {						// all repeats done?
				if (pAI->alertDone) {					// yes, check if we should raise alert
					xActuatorAlert(pAI, alertTYPE_ACT_DONE, alertLEVEL_WARNING) ;
				}
				if (pAI->Seq[0] != 0xFF) {				// another sequence in the queue?
					iRV = xActuatorNextSequence(pAI) ;	// yes, load it
				} else {								// no, all sequences done
					xActuatorStop(pAI->ChanNum) ;		// stop & reset all values..
				}
			}
		}
	}
	return iRV ;
}

void IRAM_ATTR vActuatorUpdateTiming(act_info_t * pAI) {
	pAI->Count	+= ACTUATE_TASK_PERIOD ;
	if (pAI->Count >= pAI->Divisor) {
		pAI->Count	= 0 ;
	}
	pAI->tNOW	+= ACTUATE_TASK_PERIOD ;
	if (pAI->tNOW >= pAI->tXXX[pAI->StageNow]) {
		pAI->tNOW	= pAI->Count	= 0 ;
		xActuatorNextStage(pAI) ;
	}
}

void IRAM_ATTR vTaskActuator(void * pvPara) {
	IF_TRACK(debugAPPL_THREADS, debugAPPL_MESS_UP) ;
#if		(configHAL_I2C_XXX_OUT > 0)
	xRtosWaitStatus(flagAPP_I2C, portMAX_DELAY) ;
#endif
	vActuatorsConfig() ;
	IF_SYSTIMER_INIT(debugTIMING_STAGES, systimerACT_S0, systimerCLOCKS, "ActS0_FI", 50, 50000) ;
	IF_SYSTIMER_INIT(debugTIMING_STAGES, systimerACT_S1, systimerCLOCKS, "ActS1_ON", 50, 50000) ;
	IF_SYSTIMER_INIT(debugTIMING_STAGES, systimerACT_S2, systimerCLOCKS, "ActS2_FO", 50, 50000) ;
	IF_SYSTIMER_INIT(debugTIMING_STAGES, systimerACT_S3, systimerCLOCKS, "ActS3_OF", 50, 50000) ;
	IF_SYSTIMER_INIT(debugTIMING_TASK, systimerACT_SX, systimerCLOCKS, "ActSXall", 50, 500000) ;
	xRtosSetStateRUN(taskACTUATE) ;

	while(bRtosVerifyState(taskACTUATE)) {
		TickType_t	ActLWtime = xTaskGetTickCount();    // Get the ticks as starting reference
		IF_EXEC_1(debugTIMING_TASK, xSysTimerStart, systimerACT_SX) ;
		act_info_t * pAI = &sAI[0] ;
		ActuatorsRunning = 0 ;
		for (uint8_t Chan = 0; Chan < NumActuator ; ++Chan, ++pAI) {
			if (pAI->Rpt == 0 ||						// no repeats left
				pAI->Blocked ||							// Reserved for something else
				pAI->ConfigOK == 0) {					// not yet configured
				continue ;
			}
			++ActuatorsRunning ;
			if (pAI->Busy) {
				continue ;								// being changed from somewhere else
			}
			pAI->Busy = 1 ;
			switch(pAI->StageNow) {
			case actSTAGE_FI:							// Step UP from 0% to 100% over tFI mSec
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStart, systimerACT_S0) ;
				if (pAI->tFI > 0) {
					vActuatorSetDC(Chan, pAI->MinDC + ((pAI->tNOW * pAI->DelDC) / pAI->tFI)) ;
					vActuatorUpdateTiming(pAI) ;
					IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S0) ;
					break ;
				}
				xActuatorNextStage(pAI) ;
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S0) ;
				/* FALLTHRU */ /* no break */
			case actSTAGE_ON:							// remain on 100% for tON mSec
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStart, systimerACT_S1) ;
				if (pAI->tON > 0) {
					if (pAI->tNOW == 0) {
						vActuatorSetDC(Chan, pAI->MaxDC) ;
					}
					vActuatorUpdateTiming(pAI) ;
					IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S1) ;
					break ;
				}
				xActuatorNextStage(pAI) ;
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S1) ;
				/* FALLTHRU */ /* no break */
			case actSTAGE_FO:							// Step DOWN 100% -> 0% over tFO mSec
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStart, systimerACT_S2) ;
				if (pAI->tFO > 0) {
					vActuatorSetDC(Chan, pAI->MaxDC - ((pAI->tNOW * pAI->DelDC) / pAI->tFO)) ;
					vActuatorUpdateTiming(pAI) ;
					IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S2) ;
					break ;
				}
				xActuatorNextStage(pAI) ;
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S2) ;
				/* FALLTHRU */ /* no break */
			case actSTAGE_OFF:							// remain off 0% for tOFF mSec
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStart, systimerACT_S3) ;
				if (pAI->tOFF > 0) {
					if (pAI->tNOW == 0) {
						vActuatorSetDC(Chan, pAI->MinDC) ;
					}
					vActuatorUpdateTiming(pAI) ;
					IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S3) ;
					break ;
				}
				xActuatorNextStage(pAI) ;
				IF_EXEC_1(debugTIMING_STAGES, xSysTimerStop, systimerACT_S3) ;
				break ;
			}
			pAI->Busy = 0 ;
		}

#if		(halHAS_PCA9555 == 1)
		/* Considering that we might be running actuator task every 1mS and
		 * that it is possible for every I2C connected actuator pin to change
		 * state every 1mS, we could be trying to write each bit each 1mS,
		 * hence 16x I2C writes per 1mS.
		 * Here we are (maybe) doing a batch write once per cycle */
		pca9555DIG_OUT_WriteAll() ;
		/* With both water valves and door strikers we have a situation where a reverse EMF is induced
		 * in the solenoid when power is removed from the actuator. This EMF can, if left undamped,
		 * reflect back along the cabling to the controller and has been knows to cause I2C bus problems.
		 * In order to damp the EMF right at the source a reverse biased signal diode should be wired
		 * across. the solenoid connectors, as close as possible to the source. To diagnose possible
		 * diode absence or failure we regularly perform a check to verify the actual I2C device state
		 * against what we believe it should be */
		pca9555Check(ACTUATE_TASK_PERIOD) ;
#endif

		IF_EXEC_1(debugTIMING_TASK, xSysTimerStop, systimerACT_SX) ;
		if (ActuatorsRunning) {							// Some active actuators, delay till next cycle
			vTaskDelayUntil(&ActLWtime, pdMS_TO_TICKS(ACTUATE_TASK_PERIOD)) ;
		} else {										// NO active actuators
			xRtosClearStateRUN(taskACTUATE) ;			// clear RUN state & wait at top....
		}
	}
	IF_TRACK(debugAPPL_THREADS, debugAPPL_MESS_DN) ;
	vTaskDelete(NULL) ;
}

void	vTaskActuatorInit(void * pvPara) {
	xRtosTaskCreate(vTaskActuator, "Actuator", ACTUATE_STACK_SIZE, pvPara, 6, NULL, INT_MAX) ;
}

// ##################################### functional tests ##########################################

#define	SCALE		4
#define	tBASE		(3000 * SCALE)
#define	tSTEP		(500 * SCALE)
#define	tLATCH		5000

void	vActuatorsIdent(void) {
#if		(configHAL_XXX_XXX_OUT > 0) && (HW_VARIANT == HW_AC00 || HW_VARIANT == HW_AC01)
	for (uint8_t Chan = LED0; Chan <= LED7; ++Chan) {
		int32_t iRV =xActuatorLoad(Chan, 2, tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP)) ;
		IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
		vActuatorReportChan(Chan) ;
	}
	for (uint8_t Chan = RELAY0; Chan <= RELAY7; ++Chan) {
		int32_t iRV = xActuatorLoad(Chan, 2, 0, (tLATCH * (RELAY7 - Chan + 1)), 0, 0) ;
		IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
		vActuatorReportChan(Chan) ;
	}

#elif		(configHAL_XXX_XXX_OUT > 0) && (HW_VARIANT == HW_WROVERKIT || HW_VARIANT == HW_DOITDEVKIT)
	for (uint8_t Chan = 0; Chan < NumActuator; ++Chan) {
		xActuatorLoad(Chan, 2, tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP), tBASE+(Chan*tSTEP)) ;
		vActuatorReportChan(Chan) ;
	}
#endif
}

void	vActuatorTestReport(uint8_t Chan, char * pcMess) {
	IF_myASSERT(debugPARAM, Chan < NumActuator) ;
	act_info_t * pAI = &sAI[0] ;
	printfx("%s #%d Stage:%d Rpt:%d tFI:%d tON:%d tFO:%d tOFF:%d tNOW:%d ",
				pcMess, Chan, pAI->StageNow, pAI->Rpt,
				pAI->tFI, pAI->tON, pAI->tFO, pAI->tOFF, pAI->tNOW) ;
	switch(ActInit[Chan].Type) {
#if	(configHAL_XXX_DIG_OUT > 0)
	#if	(configHAL_GPIO_DIG_OUT > 0)
	case actSOC_DIG:
	#endif

	#if	(configHAL_I2C_DIG_OUT > 0)
	case actI2C_DIG:
	#endif

	#if	(configHAL_SPI_DIG_OUT > 0)
	case actSPI_DIG:
	#endif

	#if	(configHAL_GPIO_PWM_OUT > 0)
	case actSOC_PWM:
	#endif

		printfx("(%s) Div:%d Match:%d\n", ActTypeNames[ActInit[Chan].Type], pAI->Divisor, pAI->Match) ;
		break ;
#endif

	default:
		printfx("Invalid actuator type=%d\n", ActInit[Chan].Type) ;
		break ;
	}
}

void	vActuatorTest(void) {
// Test PHYSical level functioning
#if		(debugPHYS) || (debugFUNC) || (debugUSER)
	xRtosWaitStatus(flagAPP_I2C_READY, portMAX_DELAY) ;
#endif

#if		(debugPHYS)
	for(uint8_t Chan = 0; Chan < NumActuator; Chan++) {
		xActuatorConfig(Chan) ;
		vActuateSetLevelDIG(Chan, true) ;
		vTaskDelay(1000) ;
		vActuateSetLevelDIG(Chan, false) ;
	}
#endif

#if		(debugFUNC)
	for(uint8_t eChan = 0; eChan < NumActuator; ++eChan) {
		xActuatorConfig(eChan) ;
		for(uint32_t Freq = configACTUATE_DIG_MIN_FREQ ; Freq <= configACTUATE_DIG_MAX_FREQ; Freq *= 5) {
			xActuatorSetFrequency(eChan, Freq) ;
			xActuatorSetTiming(eChan, 0, 0, UINT32_MAX, 0) ;
			xActuatorStart(eChan, UINT32_MAX) ;
			for(int8_t CurDC = actMIN_DUTYCYCLE; CurDC <= actMAX_DUTYCYCLE ; CurDC = (CurDC == 0) ? 1 : CurDC * 2) {
				vActuatorSetDC(eChan, CurDC) ;
				SL_INFO("DIG: Chan=%d  Freq=%'u  Lev=%'u\n", eChan, Freq, CurDC) ;
				getchar() ;
			}
		}
	}
#endif

#if		(debugUSER)
	for(uint8_t eChan = 0; eChan < NumActuator; ++eChan) {
		xActuatorConfig(eChan) ;
		xActuatorSetFrequency(eChan, 1000) ;

		xActuatorLoad(eChan, 1, 1000, 1000, 1000, 1000) ;
		vActuatorTestReport(eChan, "1s On/Off each, 0->100%") ;
		vTaskDelay(pdMS_TO_TICKS(4000)) ;

		xActuatorLoad(eChan, 1, 2000, 2000, 2000, 2000) ;
		vActuatorTestReport(eChan, "2s per phase, 0->100%") ;
		vTaskDelay(pdMS_TO_TICKS(8000)) ;

		xActuatorLoad(eChan, 1, 5000, 5000, 5000, 5000) ;
		vActuatorTestReport(eChan, "5s per phase, 0->100%") ;
		vTaskDelay(pdMS_TO_TICKS(20000)) ;
	}
#endif
}

#endif
