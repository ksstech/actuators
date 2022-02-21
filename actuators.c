/*
 * actuators.c - actuator support for LEDs, SSRs, Relays etc with hard & soft PWM support
 */

#include	"hal_variables.h"
#include 	"actuators.h"
#include	"endpoints.h"
#include	"options.h"
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

#define	debugFLAG					0xF000

#define	debugPHYS					(debugFLAG & 0x0001)
#define	debugFUNC					(debugFLAG & 0x0002)
#define	debugUSER					(debugFLAG & 0x0004)
#define	debugDUTY					(debugFLAG & 0x0008)

#define	debugDUTY_CYCLE				(debugFLAG & 0x0010)
#define	debugREMTIME				(debugFLAG & 0x0080)

#define	debugFUNCTIONS				(debugFLAG & 0x0100)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ############################################ Macros #############################################

#define	stACT_ALL_MASK (1<<stACT_S0 | 1<<stACT_S1 | 1<<stACT_S2 | 1<<stACT_S3 | 1<<stACT_SX)

// #################################### Global & Local variables ###################################

const char * const StageNames[]	= { "FI ", "ON ", "FO ", "OFF" };
const char * const ActTypeNames[]	= { "SoC/DIG", "SoC/PWM", "SoC/ANA", "I2C/DIG", "I2C/PWM", "I2C/ANA", "SPI/DIG", "SPI/PWM", "SPI/ANA" };

act_init_t	ActInit[halXXX_XXX_OUT] = {						// Static configuration info
#if		(HW_VARIANT == HW_AC00)
	{	actI2C_DIG,	7,	},
	{	actI2C_DIG,	6,	},
	{	actI2C_DIG,	5,	},
	{	actI2C_DIG,	4,	},
	{	actI2C_DIG,	3,	},
	{	actI2C_DIG,	2,	},
	{	actI2C_DIG,	1,	},
	{	actI2C_DIG,	0,	},

	{	actI2C_DIG,	8,	},
	{	actI2C_DIG,	9,	},
	{	actI2C_DIG,	10,	},
	{	actI2C_DIG,	11,	},
	{	actI2C_DIG,	12,	},
	{	actI2C_DIG,	13,	},
	{	actI2C_DIG,	14,	},
	{	actI2C_DIG,	15,	},

#elif	(HW_VARIANT == HW_AC01)
	{	actI2C_DIG,	0,	},
	{	actI2C_DIG,	1,	},
	{	actI2C_DIG,	2,	},
	{	actI2C_DIG,	3,	},
	{	actI2C_DIG,	4,	},
	{	actI2C_DIG,	5,	},
	{	actI2C_DIG,	6,	},
	{	actI2C_DIG,	7,	},

	{	actI2C_DIG,	8,	},
	{	actI2C_DIG,	9,	},
	{	actI2C_DIG,	10,	},
	{	actI2C_DIG,	11,	},
	{	actI2C_DIG,	12,	},
	{	actI2C_DIG,	13,	},
	{	actI2C_DIG,	14,	},
	{	actI2C_DIG,	15,	},

#elif	(HW_VARIANT == HW_EM1P2) || (HW_VARIANT == HW_EM3P2)
//	{	actSOC_DIG,	halSOC_DIG_OUT_0,	}, // cannot use, pin conflicts with SCL
#elif	(HW_VARIANT == HW_WROVERKIT)			// WROVER-KIT
	{	actSOC_DIG,	0,	},
	{	actSOC_DIG,	1,	},
	{	actSOC_DIG,	2,	},

#elif	(HW_VARIANT == HW_DOITDEVKIT)			// DoIT DevKIt v1
	{	actSOC_DIG,	0,	},

#elif	(HW_VARIANT == HW_EBOX)
	{	actSOC_DIG,	0,	},
	{	actSOC_DIG,	1,	},

#endif
};

/* In order to optimise MCU utilization, especially since the actuator task is set to run EVERY 1mS,
 * we try to only start the task if there is something to do. Hence, the task is started every time
 * a LOAD command is executed. During the running of the task the 'ActuatorsRunning' variable is set
 * to the number of actuators serviced during that task cycle. At the end of the task cycle, if NO
 * actuators were serviced, the task RUN status is cleared, only to be restarted with the next LOAD.
 */
uint8_t	ActuatorsRunning = 0;
act_info_t	sAI[halXXX_XXX_OUT];

/* Objective is to facilitate a number of predefined sequences with a simple single actuation command.
 * SEQUENCE Ch# m0 m1 m2 m3 etc mZ will result in the first mode (m0) being loaded immediately with the
 * rest of the mode numbers going into a queue to be loaded sequentially after completion of the previous
 */
act_seq_t	sAS[actNUM_SEQUENCES]	= {
//	  Rpt	tFI		tON		tFO		tOFF
	{ 5,	0,		1000,	0,		1000,	} ,		// 0.50Hz	x5	10Sec		OK
	{ 1,	0,		0,		0,		25000, 	} ,		// OFF		x1	25Sec		BUSY
	{ 5,	0,		1000,	0,		1000,	} ,		// 0.50Hz	x5	10Sec		OK
	{ 1,	0,		0,		0,		275000, } ,		// OFF		x1	275Sec		BUSY
	{ 30,	0,		500,	0,		500,	} ,		// 1.00Hz	x30 30Sec		WARNING
	{ 15,	500,	0,		500,	0,		} ,		// 1.00Hz	x15	15Sec		WAIT !!!
	{ 8,	0,		250,	0,		250,	} ,		// 2.00 Hz	x8	4Sec
	{ 6,	0,		500,	0,		500,	} ,		// 1.00 Hz	x6	6Sec
	{ 4,	0,		750,	0,		750,	} ,		// 0.67 Hz	x4	6Sec
	{ 3,	0,		1000,	0,		1000,	} ,		// 0.50 Hz	x3	6Sec
};

// ################################# local/static functions ########################################

int	xActuatorLogError(const char * pFname, uint8_t eChan) {
	SL_LOG(SL_SEV_ERROR, "type=%d '%s'", ActInit[eChan].Type, ActTypeNames[ActInit[eChan].Type]) ;
	return erFAILURE ;
}

// ##################### Hardware dependent (DIG/PWM/ANA) Actuator functions #######################

void IRAM_ATTR vActuateSetLevelDIG(uint8_t eChan, uint8_t NewState) {
	switch(ActInit[eChan].Type) {					// handle hardware dependent component
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
		halGPIO_DIG_OUT_SetState(ActInit[eChan].halGPIO, NewState);
		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		halGPIO_PWM_OUT_SetState(ActInit[eChan].halGPIO, NewState);
		break;
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
		#if	(halHAS_PCA9555 == 1)		// To minimise I2C traffic, update bit array but do NOT write to device
		pca9555DIG_OUT_SetState(ActInit[eChan].halGPIO, NewState, 0);
		#else
		myASSERT(0);
		#endif
		break;
	#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan);
	}
}

static int xActuateGetLevelDIG(uint8_t eChan) {
	int iRV = erFAILURE;
	switch(ActInit[eChan].Type) {						// handle hardware dependent component
#if		(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
		iRV = halGPIO_DIG_OUT_GetState(ActInit[eChan].halGPIO);
		break;
#endif

#if		(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		iRV = halGPIO_PWM_OUT_GetState(ActInit[eChan].halGPIO);
		break;
#endif

#if		(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
	#if	(halHAS_PCA9555 == 1)
		iRV = pca9555DIG_OUT_GetState(ActInit[eChan].halGPIO);
	#else
		myASSERT(0);
	#endif
		break;
#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan);
	}
	return iRV;
}

/**
 * vActuatorSetFrequency() - configure channel for a specific [soft] PWM frequency
 * @brief		The timer will be stopped and a new frequency will be configured
 * 				The timer will NOT be restarted until a new duty cycle is configured
 * @param[in]	Chan - logical PWM channel
 * @param[in]	Frequency - desired frequency trimmed to be within the supported range
 * @return		none
 */
static int xActuatorSetFrequency(uint8_t eChan, uint32_t Frequency) {
	switch(ActInit[eChan].Type) {			// handle hardware dependent component
	#if	(halXXX_DIG_OUT > 0)
	case actSOC_DIG:
	case actI2C_DIG:
	case actSPI_DIG:
		FIT2RANGE(actDIG_MIN_FREQ, Frequency, actDIG_MAX_FREQ, uint32_t);
		sAI[eChan].Divisor	= (MILLIS_IN_SECOND / ACTUATE_TASK_PERIOD) / Frequency;
 		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
	case actI2C_PWM:
	case actSPI_PWM:
		FIT2RANGE(halPWM_MIN_FREQ, Frequency, halPWM_MAX_FREQ, uint32_t);
		sAI[eChan].Divisor	= (halPWM_CLOCK_HZ / Frequency);
		halGPIO_PWM_OUT_SetFrequency(ActInit[eChan].halGPIO, sAI[eChan].Divisor - 1);
		break;
	#endif

	default:
		return xActuatorLogError(__FUNCTION__, eChan);
	}
	return erSUCCESS;
}

/**
 * vActuatorSetDC() - Recalc & set duty cycle (brightness/speed level)
 * @param Chan		logical (soft) PWM channel
 */
static void IRAM_ATTR vActuatorSetDC(uint8_t eChan, int8_t CurDC) {
	act_info_t * psAI = &sAI[eChan];
	psAI->CurDC	= CurDC;

	switch(ActInit[eChan].Type) {
	#if	(halXXX_DIG_OUT > 0)		// All (SOC + I2C + SPI) DIGital type actuators
	case actSOC_DIG:
	case actI2C_DIG:
	case actSPI_DIG:
		switch(psAI->StageNow) {
		case actSTAGE_FI: psAI->Match	= psAI->MaxDC - psAI->CurDC; 	break;
		case actSTAGE_ON: psAI->Match	= psAI->MinDC; 				break;
		case actSTAGE_FO: psAI->Match	= psAI->MaxDC - psAI->CurDC; 	break;
		case actSTAGE_OFF: psAI->Match	= psAI->MaxDC; 				break;
		}
		vActuateSetLevelDIG(eChan, (psAI->Count >= psAI->Match) ? 1 : 0);
		break;
	#endif

	#if	(halXXX_PWM_OUT > 0)		// All (SOC + I2C + SPI) PWM type actuators
	case actSOC_PWM:
		psAI->Match = u32ScaleValue(CurDC, psAI->MinDC, psAI->MaxDC, halPWM_MIN_FREQ, halPWM_MAX_FREQ);
		halGPIO_PWM_OUT_SetCycle(ActInit[eChan].halGPIO, psAI->Match);
		break;
	case actSOC_ANA:
	case actSOC_ANA:
		myASSERT(0);
		break;
	#endif

	#if	(halXXX_ANA_OUT > 0)		// All (SOC + I2C + SPI) ANAlog type actuators
	case actSOC_ANA:
	case actI2C_ANA:
	case actSPI_ANA:
		myASSERT(0);
		break;
	#endif

	default:
		xActuatorLogError(__FUNCTION__, eChan);
	}
	IF_EXEC_1(debugDUTY, vActuatorReportChan, eChan);
}

// ################################# local/static functions ########################################

/**
 * @brief		UNTESTED
 * @param		psAI
 * @return
 */
static int IRAM_ATTR xActuatorAlert(act_info_t * psAI, uint8_t Type, uint8_t Level) {
	epi_t	sEI = { 0 };
	event_t	sEvent	= { 0 };
	alert_t	sAlert	= { 0 };
	ubuf_t	sBuf	= { 0 };
	vEpGetInfoWithIndex(&sEI, URI_ACT);
	IF_RETURN(sEI.psES == NULL, erFAILURE);
	sEI.psEvent		= &sEvent;
	sEI.psAlert		= &sAlert;
	sEI.psUB		= &sBuf;
	// configure the type, level and supporting field/channel info
	sAlert.Type		= Type;
	sAlert.Level	= Level;
	sAlert.pvValue	= psAI;
	return xEpGenerateAlert(&sEI);
}

static int xActuatorVerifyParameters(uint8_t Chan, uint8_t Field) {
	#if	(SW_AEP == 1)
	if (Chan >= NumActuator || OUTSIDE(oldACT_T_FI, Field, oldACT_T_REM, uint8_t) || sAI[Chan].Blocked) {
		SL_ERR("Invalid actuator(%d) / field (%d) / status (%d)", Chan, Field, sAI[Chan].Blocked);
		return erFAILURE;
	}
	#elif (SW_AEP == 2)
	if (Chan >= NumActuator || OUTSIDE(selACT_FIRST, Field, selACT_LAST, uint8_t) || sAI[Chan].Blocked) {
		SL_ERR("Invalid actuator(%d) / field (%d) / status (%d)", Chan, Field, sAI[Chan].Blocked);
		return erFAILURE;
	}
	#endif
	return erSUCCESS;
}

static void IRAM_ATTR vActuatorUpdateCurDC(act_info_t * psAI) {
	psAI->CurDC	= psAI->MinDC;
	psAI->Match	= psAI->tNOW;
	if (psAI->tXXX[psAI->StageNow]) {
		psAI->CurDC	+= ((psAI->tNOW * psAI->DelDC) / psAI->tXXX[psAI->StageNow]);
		psAI->Match	= psAI->tNOW / ( psAI->tXXX[psAI->StageNow] / psAI->Divisor);
	}
}

/**
 * @brief		configure the hardware pin associated with a channel
 *				Uses the definitions in the hal_gpio module to define the specific pin,
 * 				its configuration and (optionally) the associated timer module for hard PWM
 * @param[in]	Channel
 * @return		None
 */
static int xActuatorConfig(uint8_t Chan) {
	IF_RETURN(sAI[Chan].Blocked, erFAILURE);
	switch(ActInit[Chan].Type) {					// handle hardware dependent component
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
		halGPIO_DIG_OUT_Config(ActInit[Chan].halGPIO);
		xActuatorSetFrequency(Chan, actDIG_DEF_FREQ);
		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		halGPIO_PWM_OUT_Config(ActInit[Chan].halGPIO);
		xActuatorSetFrequency(Chan, halPWM_DEF_FREQ);
		break;
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
	#if	 (halHAS_PCA9555 == 1)
		pca9555DIG_OUT_Config(ActInit[Chan].halGPIO);
		xActuatorSetFrequency(Chan, actDIG_DEF_FREQ);
	#else
		myASSERT(0);
	#endif
		break;
	#endif

	default:
		return xActuatorLogError(__FUNCTION__, Chan);
	}
	act_info_t * psAI = &sAI[Chan];
	memset(psAI->Seq, 0xFF, SO_MEM(act_info_t, Seq));
	psAI->CurDC		= psAI->MinDC		= 0;
	psAI->MaxDC		= psAI->DelDC		= 100;
	psAI->StageBeg	= psAI->StageNow	= actSTAGE_FI;
	psAI->ChanNum	= Chan;
	psAI->ConfigOK	= 1;
	vActuatorSetDC(Chan, 0);
	IF_EXEC_1(debugTRACK && ioB1GET(ioActuate), vActuatorReportChan, Chan);
	return erSUCCESS;
}

/* ########################## Hardware INDEPENDENT Actuator functions ##############################
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

/**
 * ActuatorSetTiming()
 * @brief		timing values are supplied in mSec, converted and stored as ticks
 * @param[in]	Chan, tOFF, tFI, tON, tFO
 * @return		erFAILURE or erSUCCESS
 */
static int xActuatorSetTiming(uint8_t Chan, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) {
	act_info_t	* psAI = &sAI[Chan];
	while (psAI->Busy)
		vTaskDelay(pdMS_TO_TICKS(1));
	psAI->Busy = 1;
	// set configuration to max 1 day...
	psAI->tFI = pdMS_TO_TICKS(tFI > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFI);
	psAI->tON = pdMS_TO_TICKS(tON > MILLIS_IN_DAY ? MILLIS_IN_DAY : tON);
	psAI->tFO  = pdMS_TO_TICKS(tFO > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFO);
	psAI->tOFF = pdMS_TO_TICKS(tOFF > MILLIS_IN_DAY ? MILLIS_IN_DAY : tOFF);
	psAI->Busy = 0;
	IF_PRINT(debugTRACK && ioB1GET(ioActuate), "[ACT] SetTiming C=%d  %u -> %u -> %u -> %u\n", Chan, tFI, tON, tFO, tOFF);
	return erSUCCESS;
}

static int xActuatorStart(uint8_t Chan, uint32_t Repeats) {
	act_info_t	* psAI = &sAI[Chan];
	psAI->tNOW = psAI->Count = 0;
	psAI->StageNow = psAI->StageBeg;
	vActuatorUpdateCurDC(psAI);
	vActuatorSetDC(Chan, psAI->CurDC);
	psAI->Rpt = Repeats;
	xRtosSetStateRUN(taskACTUATE_MASK);
	IF_PRINT(debugTRACK && ioB1GET(ioActuate), "[ACT] Start C=%d R=%d\n", Chan, Repeats);
	return erSUCCESS;
}

static int xActuatorStop(uint8_t Chan) {
	act_info_t * psAI = &sAI[Chan];
	// reset ONLY the tXXX values (incl Rpt + tNow)
	memset(&psAI->tXXX, 0, sizeof(sAI[0].tXXX));
	memset(&psAI->Seq, 0xFF, sizeof(sAI[0].Seq));
	psAI->StageNow	= psAI->StageBeg;
	psAI->alertDone	= psAI->alertStage	= 0;
	vActuatorSetDC(Chan, 0);
	IF_PRINT(debugTRACK && ioB1GET(ioActuate), "[ACT] Stop C=%d\n", Chan);
	return erSUCCESS;
}

static uint32_t xActuatorPause(uint8_t Chan) {
	taskDISABLE_INTERRUPTS();
	uint32_t CurRpt = sAI[Chan].Rpt;
	sAI[Chan].Rpt = 0;
	taskENABLE_INTERRUPTS();
	return CurRpt;
}

static int xActuatorUnPause(uint8_t Chan, uint32_t CurRpt) {
	sAI[Chan].Rpt = CurRpt;
	return erSUCCESS;
}

static int xActuatorAddSequences(uint8_t Chan, int Idx, uint8_t * paSeq) {
	act_info_t * psAI = &sAI[Chan];
	for (; Idx < actMAX_SEQUENCE; ++Idx) {
		if (*paSeq < NO_MEM(sAS)) {						// if a valid SEQuence number
			psAI->Seq[Idx] = *paSeq++; 					// store it
		} else {
			psAI->Seq[Idx] = 0xFF; 						// if not, mark it unused
			break; 										// and go no further
		}
	}
	IF_EXEC_1(debugTRACK && ioB1GET(ioActuate), vActuatorReportChan, Chan);
	return Idx;
}

static void vActuatorReportSeq(uint8_t Seq) {
	act_seq_t * psAS = &sAS[Seq];
	if (Seq == 0) {
		printfx("%CSeq |Repeat|  tFI  |  tON  |  tFO  |  tOFF |%C\n", colourFG_CYAN, attrRESET);
	}
	printfx(" %2d | %'#5u|%'#7u|%'#7u|%'#7u|%'#7u|\n", Seq, psAS->Rpt, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
}

static int IRAM_ATTR xActuatorNextSequence(act_info_t * psAI) {
	uint8_t	NxtSeq = psAI->Seq[0];
	IF_RETURN(NxtSeq >= NO_MEM(sAS), erFAILURE);
	IF_EXEC_1(debugTRACK && ioB1GET(ioActuate), vActuatorReportSeq, NxtSeq);
	act_seq_t * psAS = &sAS[NxtSeq];
	// load values from sequence #
	int iRV = xActuatorSetTiming(psAI->ChanNum, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
	if (iRV == erSUCCESS) {
		iRV = xActuatorStart(psAI->ChanNum, psAS->Rpt);
		if (iRV == erSUCCESS) {
			int Idx;
			for (Idx = 0;  Idx < (actMAX_SEQUENCE - 1); ++Idx)
				psAI->Seq[Idx] = psAI->Seq[Idx+1];
			psAI->Seq[Idx] = 0xFF;
		} else {
			myASSERT(0); 								// what do we need to do if the START failed???
		}
	} else {
		myASSERT(0); 									// what do we need to do if the SETTIMING failed???
	}
	return iRV;
}

static int IRAM_ATTR xActuatorNextStage(act_info_t * psAI) {
	int iRV = erSUCCESS;
	if ((psAI->alertStage == 1) && (psAI->tXXX[psAI->StageNow] > 0))
		xActuatorAlert(psAI, alertTYPE_ACT_STAGE, alertLEVEL_INFO);
	if (++psAI->StageNow == actSTAGE_NUM)
		psAI->StageNow = actSTAGE_FI;
	if (psAI->StageNow == psAI->StageBeg) {				// back at starting stage?
		if (psAI->Rpt != UINT32_MAX) {					// yes, but running unlimited repeats ?
			--psAI->Rpt; 								// No, decrement the repeat count
			if (psAI->Rpt == 0) {						// all repeats done?
				if (psAI->alertDone)					// yes, check if we should raise alert
					xActuatorAlert(psAI, alertTYPE_ACT_DONE, alertLEVEL_WARNING);
				if (psAI->Seq[0] != 0xFF) 				// another sequence in the queue?
					iRV = xActuatorNextSequence(psAI); 	// yes, load it
				else 									// no, all sequences done
					iRV = xActuatorStop(psAI->ChanNum);	// stop & reset all values..
			}
		}
	}
	return iRV;
}

static void IRAM_ATTR vActuatorUpdateTiming(act_info_t * psAI) {
	psAI->Count	+= ACTUATE_TASK_PERIOD;
	if (psAI->Count >= psAI->Divisor)
		psAI->Count	= 0;
	psAI->tNOW	+= ACTUATE_TASK_PERIOD;
	if (psAI->tNOW >= psAI->tXXX[psAI->StageNow]) {
		psAI->tNOW = psAI->Count = 0;
		xActuatorNextStage(psAI);
	}
}

static void IRAM_ATTR vTaskActuator(void * pvPara) {
	vTaskSetThreadLocalStoragePointer(NULL, 1, (void *)taskACTUATE_MASK);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S0, stMICROS, "ActS0_FI", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S1, stMICROS, "ActS1_ON", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S2, stMICROS, "ActS2_FO", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S3, stMICROS, "ActS3_OF", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_SX, stMICROS, "ActSXall", 1, 100);
	// ensure I2C config is done before initialising
	bRtosWaitStatusALL(flagAPP_I2C, portMAX_DELAY);
	for(uint8_t Chan = 0; Chan < NumActuator; xActuatorConfig(Chan++));
	xRtosSetStateRUN(taskACTUATE_MASK);

	while(bRtosVerifyState(taskACTUATE_MASK)) {
		TickType_t	ActLWtime = xTaskGetTickCount();    // Get the ticks as starting reference
		IF_SYSTIMER_START(debugTIMING, stACT_SX);
		act_info_t * psAI = &sAI[0];
		ActuatorsRunning = 0;
		for (uint8_t Chan = 0; Chan < NumActuator;  ++Chan, ++psAI) {
			if (psAI->Rpt == 0 ||						// no repeats left
				psAI->Blocked ||							// Reserved for something else
				psAI->ConfigOK == 0) {					// not yet configured
				continue;
			}
			++ActuatorsRunning;
			if (psAI->Busy)
				continue;								// being changed from somewhere else
			psAI->Busy = 1;
			switch(psAI->StageNow) {
			case actSTAGE_FI:							// Step UP from 0% to 100% over tFI mSec
				IF_SYSTIMER_START(debugTIMING, stACT_S0);
				if (psAI->tFI > 0) {
					vActuatorSetDC(Chan, psAI->MinDC + ((psAI->tNOW * psAI->DelDC) / psAI->tFI));
					vActuatorUpdateTiming(psAI);
					IF_SYSTIMER_STOP(debugTIMING, stACT_S0);
					break;
				}
				xActuatorNextStage(psAI);
				IF_SYSTIMER_STOP(debugTIMING, stACT_S0);
				/* FALLTHRU */ /* no break */
			case actSTAGE_ON:							// remain on 100% for tON mSec
				IF_SYSTIMER_START(debugTIMING, stACT_S1);
				if (psAI->tON > 0) {
					if (psAI->tNOW == 0)
						vActuatorSetDC(Chan, psAI->MaxDC);
					vActuatorUpdateTiming(psAI);
					IF_SYSTIMER_STOP(debugTIMING, stACT_S1);
					break;
				}
				xActuatorNextStage(psAI);
				IF_SYSTIMER_STOP(debugTIMING, stACT_S1);
				/* FALLTHRU */ /* no break */
			case actSTAGE_FO:							// Step DOWN 100% -> 0% over tFO mSec
				IF_SYSTIMER_START(debugTIMING, stACT_S2);
				if (psAI->tFO > 0) {
					vActuatorSetDC(Chan, psAI->MaxDC - ((psAI->tNOW * psAI->DelDC) / psAI->tFO));
					vActuatorUpdateTiming(psAI);
					IF_SYSTIMER_STOP(debugTIMING, stACT_S2);
					break;
				}
				xActuatorNextStage(psAI);
				IF_SYSTIMER_STOP(debugTIMING, stACT_S2);
				/* FALLTHRU */ /* no break */
			case actSTAGE_OFF:							// remain off 0% for tOFF mSec
				IF_SYSTIMER_START(debugTIMING, stACT_S3);
				if (psAI->tOFF > 0) {
					if (psAI->tNOW == 0)
						vActuatorSetDC(Chan, psAI->MinDC);
					vActuatorUpdateTiming(psAI);
					IF_SYSTIMER_STOP(debugTIMING, stACT_S3);
					break;
				}
				xActuatorNextStage(psAI);
				IF_SYSTIMER_STOP(debugTIMING, stACT_S3);
				break;
			}
			psAI->Busy = 0;
		}

#if		(halHAS_PCA9555 == 1)
		/* Considering that we might be running actuator task every 1mS and
		 * that it is possible for every I2C connected actuator pin to change
		 * state every 1mS, we could be trying to write each bit each 1mS,
		 * hence 16x I2C writes per 1mS.
		 * Here we are (maybe) doing a batch write once per cycle */
		pca9555DIG_OUT_WriteAll();
		/* With both water valves and door strikers we have a situation where a reverse EMF is induced
		 * in the solenoid when power is removed from the actuator. This EMF can, if left undamped,
		 * reflect back along the cabling to the controller and has been knows to cause I2C bus problems.
		 * In order to damp the EMF right at the source a reverse biased signal diode should be wired
		 * across. the solenoid connectors, as close as possible to the source. To diagnose possible
		 * diode absence or failure we regularly perform a check to verify the actual I2C device state
		 * against what we believe it should be */
		pca9555Check(ACTUATE_TASK_PERIOD);
#endif

		IF_SYSTIMER_STOP(debugTIMING, stACT_SX);
		if (ActuatorsRunning) {							// Some active actuators, delay till next cycle
			vTaskDelayUntil(&ActLWtime, pdMS_TO_TICKS(ACTUATE_TASK_PERIOD));
		} else {										// NO active actuators
			xRtosClearStateRUN(taskACTUATE_MASK); 			// clear RUN state & wait at top....
		}
	}
	vRtosTaskDelete(NULL);
}

// ######################################### Public APIs ###########################################

uint32_t xActuatorRunningCount (void) { return ActuatorsRunning;  }

uint64_t xActuatorGetRemainingTime(uint8_t Chan) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	act_info_t * psAI = &sAI[Chan];
	IF_RETURN(psAI->Rpt == UINT32_MAX, UINT64_MAX);		// indefinite/unlimited repeat ?
	IF_RETURN(psAI->Rpt == 0, 0);
	// calculate remaining time for full repeats
	taskDISABLE_INTERRUPTS();
	uint64_t u64Value = (psAI->Rpt > 1) ? (psAI->tFI + psAI->tON + psAI->tFO + psAI->tOFF) * (psAI->Rpt - 1) : 0;
	IF_PRINT(debugREMTIME, "Ch#%d: %llu", Chan, u64Value);

	// now add remaining time in current stage
	uint8_t Stage = psAI->StageNow;
	do {
		u64Value	+= (Stage == psAI->StageNow) ? psAI->tXXX[Stage] - psAI->tNOW : psAI->tXXX[Stage];
		IF_PRINT(debugREMTIME, " -> s(%d): %llu", Stage, u64Value);
		++Stage;
		Stage %= actSTAGE_NUM;
	} while (Stage != psAI->StageBeg);

	// now add the time for the (optional) sequences
	for (int Idx = 0; psAI->Seq[Idx] < NO_MEM(sAS); ++Idx) {
		act_seq_t * psAS = &sAS[psAI->Seq[Idx]];
		u64Value	+= psAS->Rpt * (psAS->tFI + psAS->tON + psAS->tFO + psAS->tOFF);
		IF_PRINT(debugREMTIME, " -> I(%d): %llu", Idx, u64Value);
	}
	taskENABLE_INTERRUPTS();
	IF_PRINT(debugREMTIME, "\n");
	return u64Value;
}

/**
 * Determines the longest remaining running time, not the sum of ALL
 * @return Remaining maximum actuator running time in uSecs
 */
uint64_t xActuatorGetMaxRemainingTime (void) {
	uint64_t u64Now, u64Max = 0.0;
	for (int Chan = 0; Chan < NumActuator; ++Chan) {
		u64Now = xActuatorGetRemainingTime(Chan);
		if (u64Now > u64Max)
			u64Max = u64Now;
	}
	return u64Max * MICROS_IN_MILLISEC;
}

void vTaskActuatorInit(void * pvPara) {
	xRtosTaskCreate(vTaskActuator, "Actuator", ACTUATE_STACK_SIZE, pvPara, ACTUATE_PRIORITY, NULL, tskNO_AFFINITY);
}

/* ############################ Actuator alerting support functions ################################
 * Start (OFF -> FI/ON)
 * Stop (ON/FO -> OFF)
 * Stage (OFF -> FI -> ON -> FO -> OFF)
 * Event Blocked due to any reason, especially pending restart
 */

int	xActuatorLoad(uint8_t Chan, uint32_t Rpt, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	IF_RETURN(sAI[Chan].ConfigOK == 0, erINVALID_CONFIG);
	IF_RETURN(sAI[Chan].Blocked, erINVALID_STATE);
	int iRV = xActuatorStop(Chan);
	if (iRV == erSUCCESS) {
		iRV = xActuatorSetTiming(Chan, tFI, tON, tFO, tOFF);
		if (iRV == erSUCCESS) {
			iRV = xActuatorStart(Chan, Rpt);
			IF_EXEC_1(debugTRACK && ioB1GET(ioActuate), vActuatorReportChan, Chan);
		}
	}
	if (iRV < erSUCCESS)
		xSyslogError(__FUNCTION__, iRV);
	return iRV;
}

int	xActuatorBreath(uint8_t Chan) { return xActuatorLoad(Chan, UINT32_MAX, 750, 750, 750, 750); }

int	vActuatorPanic(uint8_t Chan) { return xActuatorLoad(Chan, UINT32_MAX, 150, 150, 150, 150); }

int	vActuatorOn(uint8_t Chan) { return xActuatorLoad(Chan, UINT32_MAX, 0, UINT32_MAX, 0, 0); }

int	vActuatorOff(uint8_t Chan) { return xActuatorLoad(Chan, UINT32_MAX, 0, 0, 0, UINT32_MAX); }

int	xActuatorSetAlertStage(uint8_t Chan, int OnOff) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	act_info_t	* psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
	psAI->alertStage = OnOff ? 1 : 0;
	return erSUCCESS;
}

int	xActuatorSetAlertDone(uint8_t Chan, int OnOff) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	act_info_t	* psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
	psAI->alertDone = OnOff ? 1 : 0;
	return erSUCCESS;
}

int	xActuatorSetStartStage(uint8_t Chan, int Stage) {
	IF_RETURN(Chan >= NumActuator || OUTSIDE(actSTAGE_FI, Stage, actSTAGE_OFF, int), erINVALID_PARA);
	act_info_t	* psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
	psAI->StageBeg = Stage;
	return erSUCCESS;
}

int	vActuatorSetMinMaxDC(uint8_t Chan, int iMin, int iMax) {
	IF_RETURN(Chan >= NumActuator || OUTSIDE(0, iMin, 100, int) || OUTSIDE(0, iMax, 100, int), erINVALID_PARA);
	act_info_t	* psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
	if (iMin > iMax)
		SWAP(iMin, iMax, uint8_t);
	psAI->MinDC = iMin;
	psAI->MaxDC = iMax;
	IF_PRINT(debugDUTY_CYCLE, "Done C=%d  Min=%d -> %d  Max=%d -> %d\n", Chan, iMin, psAI->MinDC, iMax, psAI->MaxDC);
	return erSUCCESS;
}

int	xActuatorBlock(uint8_t Chan) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	sAI[Chan].Blocked = 1;
	return erSUCCESS;
}

int	xActuatorUnBlock(uint8_t Chan) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	sAI[Chan].Blocked = 0;
	return erSUCCESS;
}

/**
 * @brief	Load up to a maximum of actMAX_SEQUENCE sequence numbers to the
 * 			sequence table, overwriting any existing (pending) sequences
 *			Expects a full array of actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param	Chan		Channel number
 * @param	paSeq		pointer to array of sequence numbers
 * @return
 */
int	xActuatorLoadSequences(uint8_t Chan, uint8_t * paSeq) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	IF_RETURN(sAI[Chan].Blocked, erINVALID_STATE);
	return xActuatorAddSequences(Chan, 0, paSeq);
}

int	xActuatorStartSequence(uint8_t Chan, int Seq) {
	IF_RETURN(OUTSIDE(0, Seq, actMAX_SEQUENCE-1, int), erINVALID_PARA);
	act_seq_t * psAS = &sAS[Seq];
	return xActuatorLoad(Chan, psAS->Rpt, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
}

/**
 * @brief	Append additional sequence numbers to the end of the sequence table
 *			Expects an array of up to actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param	Chan		Channel number
 * @param	paSeq		pointer to array of sequence numbers
 * @return	erFAILURE if none appended else number of sequences appended
 */
int	xActuatorQueSequences(uint8_t Chan, uint8_t * paSeq) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	IF_RETURN(sAI[Chan].Blocked, erINVALID_STATE);
	for (int Idx = 0; Idx < actMAX_SEQUENCE; ++Idx) {
		if (sAI[Chan].Seq[Idx] == 0xFF) {
			return xActuatorAddSequences(Chan, Idx, paSeq);
		}
	}
	return erFAILURE;
}

int	xActuatorUpdate(uint8_t Chan, int Rpt, int tFI, int tON, int tFO, int tOFF) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	act_info_t * psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
	uint32_t CurRpt = xActuatorPause(Chan);
	while (psAI->Busy)
		vTaskDelay(pdMS_TO_TICKS(1));
	psAI->Busy = 1;
	// XXX: Add range checking to not wrap around any member
	psAI->tFI += (tFI * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tON += (tON * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tFO += (tFO * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tOFF += (tOFF * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->Busy = 0;
	return xActuatorUnPause(Chan, (CurRpt == 0xFFFFFFFF) ? CurRpt : CurRpt + Rpt);
}

int	xActuatorAdjust(uint8_t Chan, int Stage, int Adjust) {
	IF_RETURN(Chan >= NumActuator || OUTSIDE(0, Stage, actSTAGE_OFF, int), erINVALID_PARA);
	act_info_t * psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
//	taskDISABLE_INTERRUPTS(); 					// XXX might be able to remove if Busy flag works
	Adjust = (Adjust * configTICK_RATE_HZ) / MILLIS_IN_SECOND; 	// convert adjustment to Ticks
	while (psAI->Busy)
		vTaskDelay(pdMS_TO_TICKS(1));
	psAI->Busy = 1;
	uint32_t CurVal = psAI->tXXX[Stage]; 			// save the selected stage value
	uint32_t NewVal = CurVal + Adjust;
	if (Adjust < 0) {
		psAI->tXXX[Stage]	= NewVal < CurVal ? NewVal : 0;
	} else {
		psAI->tXXX[Stage]	= NewVal > CurVal ? NewVal : UINT32_MAX;
	}
	psAI->Busy = 0;
//	taskENABLE_INTERRUPTS();
	IF_EXEC_1(debugTRACK && ioB1GET(ioActuate), vActuatorReportChan, Chan);
	return erSUCCESS;
}

int	xActuatorToggle(uint8_t Chan) {
	IF_RETURN(Chan >= NumActuator, erINVALID_PARA);
	act_info_t * psAI = &sAI[Chan];
	IF_RETURN(psAI->Blocked, erINVALID_STATE);
//	taskDISABLE_INTERRUPTS(); 					// XXX might be able to remove if Busy flag works
	while (psAI->Busy)
		vTaskDelay(pdMS_TO_TICKS(1));
	psAI->Busy = 1;
	SWAP(psAI->tFI, psAI->tFO, uint32_t);
	SWAP(psAI->tON, psAI->tOFF, uint32_t);
	psAI->Busy = 0;
//	taskENABLE_INTERRUPTS();
	return erSUCCESS;
}

// ############################## Rules interface to Actuator table ################################

double	dActuatorGetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE)
		return 0.0;
	px64Var->def.cv.vs	= vs32B;
	px64Var->def.cv.vt	= vtVALUE;
	px64Var->def.cv.vf	= vfUXX;
	px64Var->def.cv.vc	= 1;
	act_info_t * psAI = &sAI[Chan];
	x64_t x64Value;
#if		(SW_AEP == 1)
	if (Field < oldACT_T_REM) {							// all these are real tXXX fields/stages
		x64Value.f64 				= psAI->tXXX[Field-oldACT_T_FI];
		px64Var->val.x64.x32[0].u32 = psAI->tXXX[Field-oldACT_T_FI];
	} else {
		x64Value.f64 = (double) xActuatorGetRemainingTime(Chan);
		px64Var->val.x64.x32[0].u32 = (uint32_t) x64Value.f64;
		IF_PRINT(debugREMTIME, "F64=%f", x64Value.f64);
	}
	IF_PRINT(debugFUNCTIONS, "%s: C=%d  F=%d  I=%d  V=%'u\n", __FUNCTION__, Chan, Field, Field-oldACT_T_FI, px64Var->val.x64.x32[0].u32);
#elif	(SW_AEP == 2)
	if (Field < selACT_T_REM) {							// all these are real tXXX fields/stages
		x64Value.f64 				= psAI->tXXX[Field-selACT_FIRST];
		px64Var->val.x64.x32[0].u32 = psAI->tXXX[Field-selACT_FIRST];
	} else {
		x64Value.f64 = (double) xActuatorGetRemainingTime(Chan);
		px64Var->val.x64.x32[0].u32 = (uint32_t) x64Value.f64;
		IF_PRINT(debugREMTIME, "F64=%f", x64Value.f64);
	}
	IF_PRINT(debugFUNCTIONS, "%s: C=%d  F=%d  I=%d  V=%'u\n", __FUNCTION__, Chan, Field, Field-selACT_FIRST, px64Var->val.x64.x32[0].u32);
#endif
	return x64Value.f64;
}

int	xActuatorSetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE)
		return erFAILURE;
#if		(SW_AEP == 1)
	sAI[Chan].tXXX[Field-oldACT_T_FI] = px64Var->val.x64.x32[0].u32;
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-oldACT_T_FI, sAI[Chan].tXXX[Field-oldACT_T_FI]);
#elif	(SW_AEP == 2)
	sAI[Chan].tXXX[Field-selACT_FIRST] = px64Var->val.x64.x32[0].u32;
#else
	#error "NO/invalid AEP selected"
#endif
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-selACT_FIRST, sAI[Chan].tXXX[Field-selACT_FIRST]);
	return erSUCCESS;
}

int	xActuatorUpdateFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) {
	if (xActuatorVerifyParameters(Chan, Field) == erFAILURE)
		return erFAILURE;
#if		(SW_AEP == 1)
	uint32_t CurVal = sAI[Chan].tXXX[Field-oldACT_T_FI];
	if ((px64Var->val.x64.x32[0].i32 < 0) && (CurVal >= abs(px64Var->val.x64.x32[0].i32))) {
		CurVal	+= px64Var->val.x64.x32[0].i32;
	} else {
		CurVal	= 0;
	}
	sAI[Chan].tXXX[Field-oldACT_T_FI] = CurVal;
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-oldACT_T_FI, sAI[Chan].tXXX[Field-oldACT_T_FI]);
#elif	(SW_AEP == 2)
	uint32_t CurVal = sAI[Chan].tXXX[Field-selACT_FIRST];
	if ((px64Var->val.x64.x32[0].i32 < 0) && (CurVal >= abs(px64Var->val.x64.x32[0].i32))) {
		CurVal	+= px64Var->val.x64.x32[0].i32;
	} else {
		CurVal	= 0;
	}
	sAI[Chan].tXXX[Field-selACT_FIRST] = CurVal;
	IF_PRINT(debugFUNCTIONS, "F=%d  I=%d  V=%'u\n", Field, Field-selACT_FIRST, sAI[Chan].tXXX[Field-selACT_FIRST]);
#else
	#error "NO/invalid AEP selected"
#endif
	return erSUCCESS;
}

/**
 * @brief	DUMMY stub for actuator MODE support
 */
#define	SCALE		4
#define	tBASE		(3000 * SCALE)
#define	tSTEP		(500 * SCALE)

int xActuatorsConfigMode(rule_t * psR, int Xcur, int Xmax) {
	int iRV = erSUCCESS;
	do {
		uint32_t tXX = tBASE + (Xcur * tSTEP);
		iRV =xActuatorLoad(Xcur, 2, 0, tXX, 0, tXX);
		IF_myASSERT(debugRESULT, iRV == erSUCCESS);
		vActuatorReportChan(Xcur);
	} while (++Xcur < Xmax);
	return iRV;
}

void vTaskActuatorReport(void) {
	for (uint8_t Chan = 0; Chan < NumActuator;  vActuatorReportChan(Chan++));
	for (uint8_t Seq = 0; Seq < NO_MEM(sAS); vActuatorReportSeq(Seq++));
	printfx("Running=%u  maxDelay=%!.R\n\n", xActuatorRunningCount(), xActuatorGetMaxRemainingTime());
}

// ##################################### functional tests ##########################################

void vActuatorTestReport(uint8_t Chan, char * pcMess) {
	IF_myASSERT(debugPARAM, Chan < NumActuator);
	act_info_t * psAI = &sAI[0];
	printfx("%s #%d Stage:%d Rpt:%d tFI:%d tON:%d tFO:%d tOFF:%d tNOW:%d ",
				pcMess, Chan, psAI->StageNow, psAI->Rpt,
				psAI->tFI, psAI->tON, psAI->tFO, psAI->tOFF, psAI->tNOW);
	switch(ActInit[Chan].Type) {
#if	(halXXX_DIG_OUT > 0)
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
	#endif

	#if	(halSPI_DIG_OUT > 0)
	case actSPI_DIG:
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
	#endif

		printfx("(%s) Div:%d Match:%d\n", ActTypeNames[ActInit[Chan].Type], psAI->Divisor, psAI->Match);
		break;
#endif

	default:
		printfx("Invalid actuator type=%d\n", ActInit[Chan].Type);
		break;
	}
}

void vActuatorTest(void) {
// Test PHYSical level functioning
#if	(debugPHYS || debugFUNC || debugUSER)
	bRtosWaitStatusALL(flagAPP_I2C, portMAX_DELAY);
#endif

#if	(debugPHYS)
	for(uint8_t Chan = 0; Chan < NumActuator; ++Chan) {
		xActuatorConfig(Chan);
		vActuateSetLevelDIG(Chan, 1);
		vTaskDelay(1000);
		vActuateSetLevelDIG(Chan, 0);
	}
#endif

#if	(debugFUNC)
	for(uint8_t eChan = 0; eChan < NumActuator; ++eChan) {
		xActuatorConfig(eChan);
		for(uint32_t Freq = actDIG_MIN_FREQ;  Freq <= actDIG_MAX_FREQ; Freq *= 5) {
			xActuatorSetFrequency(eChan, Freq);
			xActuatorSetTiming(eChan, 0, 0, UINT32_MAX, 0);
			xActuatorStart(eChan, UINT32_MAX);
			for(int8_t CurDC = 0; CurDC <= 100;  CurDC = (CurDC == 0) ? 1 : CurDC * 2) {
				vActuatorSetDC(eChan, CurDC);
				SL_INFO("DIG: Chan=%d  Freq=%'u  Lev=%'u\n", eChan, Freq, CurDC);
				getchar();
			}
		}
	}
#endif

#if	(debugUSER)
	for(uint8_t eChan = 0; eChan < NumActuator; ++eChan) {
		xActuatorConfig(eChan);
		xActuatorSetFrequency(eChan, 1000);

		xActuatorLoad(eChan, 1, 1000, 1000, 1000, 1000);
		vActuatorTestReport(eChan, "1s On/Off each, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(4000));

		xActuatorLoad(eChan, 1, 2000, 2000, 2000, 2000);
		vActuatorTestReport(eChan, "2s per phase, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(8000));

		xActuatorLoad(eChan, 1, 5000, 5000, 5000, 5000);
		vActuatorTestReport(eChan, "5s per phase, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(20000));
	}
#endif
}
