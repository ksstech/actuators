/*
 * actuators.c - a soft PWM module to control GPIO outputs driving LEDs and relays
 * Copyright (c) 2016-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halXXX_XXX_OUT > 0)
#include "actuators.h"
#include "endpoints.h"
#include "hal_device_includes.h"
#if (halUSE_I2C > 0)
	#include "hal_i2c_common.h"
#endif
#include "hal_options.h"
#include "printfx.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"
#include "x_errors_events.h"

#include "esp_attr.h"

// ############################### BUILD: debug configuration options ##############################

#define	debugFLAG					0xF000

#define	debugPHYS					(debugFLAG & 0x0001)
#define	debugFUNC					(debugFLAG & 0x0002)
#define	debugUSER					(debugFLAG & 0x0004)
#define	debugDUTY					(debugFLAG & 0x0008)

#define	debugDUTY_CYCLE				(debugFLAG & 0x0010)
#define	debugFUNC_RULES				(debugFLAG & 0x0040)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

/*
 * Theory of operation:	DIGital outputs
 * ------------------------------------
 * Minimum interval for ANYTHING is 1mS (1 or more ticks)
 * Fixed number of levels = 100 to provide 0->100 %
 * Therefore, any stage being OFF -> FI -> ON -> FO must a multiple of 100mS
 * To control the intensity of an LED using a DIGital 0/1 pin we vary
 * the number of mSec (of this fixed 100mS period) that pin is 0/1
 *
 * Logic per mS/tick (FI & FO stages):
 * -----------------------------------
 *	if tNow = 0 (1st time in stage)
 *
 *  Update tNow counter.
 *  If tNow = tXXX
 *  	reset tNow and step to next stage
 * 	Update the level count.
 *	If level count equal match count
 *		switch the output
 *	If level count wrapped (100)
 *		reset level to 0
 *		recalc new match level
 *
 * Logic per mS/tick (ON & OFF stages):
 *  Update tNow counter.
 *  If tNow = tXXX
 *  	reset tNow and step to next stage
 * 	Update the level count.
 *	If level count equal 0 (OFF) or 100(ON)
 *		switch the output OFF or ON
 *	If level count wrapped (100)
 *		reset level to 0
 *
 * Theory of operation:	ANA outputs
 * --------------------------------
 *
 *
 * Theory of operation:	PWM outputs
 * --------------------------------
 *
 *
 */

// ############################################ Macros #############################################

/* DIGital default was 10, timing worked exactly but flashing is prominent
 * 25 reduces flashing, provides more steps in intensity, smoother FI & FO
 * 50 is very smooth, no/minimal flicker, but slightly abrupt at end of FO
 * 100 is smooth but FI/FO look abrupt at start & end
 */
#define	actFREQ_DEF_DIG				33
#define	actFREQ_DEF_ANA				250
#define	halFREQ_DEF_PWM				1000
#define	actFREQ_MIN					1
#define	actFREQ_MAX					configTICK_RATE_HZ

#define	halPWM_MAX_COUNT			(1 << 24)			// 24 bit register
#define	halPWM_CLOCK_HZ				configCLOCKS_PER_SEC
#define	halPWM_MIN_FREQ				((halPWM_CLOCK_HZ / halPWM_MAX_COUNT) + 1)
#define	halPWM_MAX_FREQ				(halPWM_CLOCK_HZ / 8)


#define	stACT_ALL_MASK (1<<stACT_S0 | 1<<stACT_S1 | 1<<stACT_S2 | 1<<stACT_S3 | 1<<stACT_SX)

// #################################### Global & Local variables ###################################

const char * const StageNames[]	= { "FI ", "ON ", "FO ", "OFF" };
const char * const ActTypeNames[]	= { "SoC/DIG", "SoC/PWM", "SoC/ANA", "I2C/DIG", "I2C/PWM", "I2C/ANA", "SPI/DIG", "SPI/PWM", "SPI/ANA" };

/* Objective is to facilitate a number of predefined sequences with a simple single actuation command.
 * SEQUENCE Ch# m0 m1 m2 m3 etc mZ will result in the first mode (m0) being loaded immediately with the
 * rest of the mode numbers going into a queue to be loaded sequentially after completion of the previous
 */
const act_seq_t sAS[actNUM_SEQUENCES] = {
//	  Rpt	tFI		tON		tFO		tOFF
	{ 5,	0,		1000,	0,		1000,	},			// 0.50Hz	x5	10Sec		OK
	{ 1,	0,		0,		0,		25000, 	},			// OFF		x1	25Sec		BUSY
	{ 5,	0,		1000,	0,		1000,	},			// 0.50Hz	x5	10Sec		OK
	{ 1,	0,		0,		0,		275000, },			// OFF		x1	275Sec		BUSY
	{ 30,	0,		500,	0,		500,	},			// 1.00Hz	x30 30Sec		WARNING
	{ 15,	500,	0,		500,	0,		},			// 1.00Hz	x15	15Sec		WAIT !!!
	{ 8,	0,		250,	0,		250,	},			// 2.00 Hz	x8	4Sec
	{ 6,	0,		500,	0,		500,	},			// 1.00 Hz	x6	6Sec
	{ 4,	0,		750,	0,		750,	},			// 0.67 Hz	x4	6Sec
	{ 3,	0,		1000,	0,		1000,	},			// 0.50 Hz	x3	6Sec
};

const act_init_t ActInit[halXXX_XXX_OUT] = {			// Static configuration info
	#if	(cmakePLTFRM == HW_AC00)
	{	actI2C_DIG,	7, },
	{	actI2C_DIG,	6, },
	{	actI2C_DIG,	5, },
	{	actI2C_DIG,	4, },
	{	actI2C_DIG,	3, },
	{	actI2C_DIG,	2, },
	{	actI2C_DIG,	1, },
	{	actI2C_DIG,	0, },

	{	actI2C_DIG,	8, },
	{	actI2C_DIG,	9, },
	{	actI2C_DIG,	10,	},
	{	actI2C_DIG,	11,	},
	{	actI2C_DIG,	12,	},
	{	actI2C_DIG,	13,	},
	{	actI2C_DIG,	14,	},
	{	actI2C_DIG,	15,	},

	#elif (cmakePLTFRM == HW_AC01)
	{	actI2C_DIG,	0, },
	{	actI2C_DIG,	1, },
	{	actI2C_DIG,	2, },
	{	actI2C_DIG,	3, },
	{	actI2C_DIG,	4, },
	{	actI2C_DIG,	5, },
	{	actI2C_DIG,	6, },
	{	actI2C_DIG,	7, },

	{	actI2C_DIG,	8, },
	{	actI2C_DIG,	9, },
	{	actI2C_DIG,	10,	},
	{	actI2C_DIG,	11,	},
	{	actI2C_DIG,	12,	},
	{	actI2C_DIG,	13,	},
	{	actI2C_DIG,	14,	},
	{	actI2C_DIG,	15,	},

	#elif (cmakePLTFRM == HW_EM1P2) || (cmakePLTFRM == HW_EM3P2)
//	{	actSOC_DIG,	halSOC_DIG_OUT_0, }, // cannot use, pin conflicts with SCL

	#elif (cmakePLTFRM == HW_KC868A4)					// Kincony KC868-A4
	{	actSOC_DIG,	0, },
	{	actSOC_DIG,	1, },
	{	actSOC_DIG,	2, },
	{	actSOC_DIG,	3, },

	#elif (cmakePLTFRM == HW_KC868A6)					// Kincony KC868-A6
	{	actI2C_DIG,	pcf8574IO8, },
	{	actI2C_DIG,	pcf8574IO9, },
	{	actI2C_DIG,	pcf8574IO10, },
	{	actI2C_DIG,	pcf8574IO11, },
	{	actI2C_DIG,	pcf8574IO12, },
	{	actI2C_DIG,	pcf8574IO13, },
	{	actSOC_ANA,	0, },
	{	actSOC_ANA,	1, },

	#elif (cmakePLTFRM == HW_DK41)						// WROVER-KIT
	{	actSOC_DIG,	0, },
	{	actSOC_DIG,	1, },
	{	actSOC_DIG,	2, },

	#elif (cmakePLTFRM == HW_SP2PM)						// Shelly Plus 2PM
	{	actSOC_DIG,	0, },								// Relay 1
	{	actSOC_DIG,	1, },								// Relay 2
	{	actSOC_DIG,	2, },								// Red LED
	#endif
};

StaticTask_t ttsACT = { 0 };
StackType_t tsbACT[actuateSTACK_SIZE] = { 0 };

/* In order to optimise MCU utilisation, especially since the actuator task is set to run EVERY 1mS,
 * we try to only start the task if there is something to do. Hence, the task is started every time
 * a LOAD command is executed. During the running of the task the 'ActuatorsRunning' variable is set
 * to the number of actuators serviced during that task cycle. At the end of the task cycle, if NO
 * actuators were serviced, the task RUN status is cleared, only to be restarted with the next LOAD.
 */
u8_t ActuatorsRunning = 0;
act_info_t sAI[halXXX_XXX_OUT];

// ###################################### Forward declarations #####################################

static int xActuateGetLevelDIG(u8_t eCh);
static void vActuatorReportChan(report_t * psR, u8_t eCh);

// #################################### Common support functions ###################################

static int xActuatorLogError(const char * pFname, u8_t eCh) {
	vSyslog(SL_SEV_ERROR, pFname, "#=%d t=%d (s)", eCh, ActInit[eCh].ioType, ActTypeNames[ActInit[eCh].ioType]);
	return erFAILURE;
}

static void vActuatorBusySET(act_info_t * psAI) {
	while (psAI->Busy) vTaskDelay(pdMS_TO_TICKS(2));
	psAI->Busy = 1;
}

static void vActuatorBusyCLR(act_info_t	* psAI) { psAI->Busy = 0; }

/**
 * @brief	UNTESTED
 * @param	psAI
 * @return
 */
static int IRAM_ATTR xActuatorAlert(act_info_t * psAI, u8_t Type, u8_t Level) {
	epi_t	sEI = { 0 };
	event_t	sEvent = { 0 };
	alert_t	sAlert = { 0 };
	ubuf_t	sBuf = { 0 };
	vEpGetInfoWithIndex(&sEI, URI_ACT);
	IF_RETURN_X(sEI.psES == NULL, erFAILURE);
	sEI.psEvent = &sEvent;
	sEI.psAlert = &sAlert;
	sEI.psUB = &sBuf;
	// configure the type, level and supporting field/channel info
	sAlert.Type = Type;
	sAlert.Level = Level;
	sAlert.pvValue = psAI;
	return xEpGenerateAlert(&sEI);
}

static int xActuatorVerifyParameters(u8_t eCh, u8_t Field) {
	if (eCh >= halXXX_XXX_OUT || sAI[eCh].Blocked || OUTSIDE(selACT_T_FI, Field, selACT_T_REM)) {
		SL_ERR("Invalid actuator(%d) / field (%d) / status (%d)", eCh, Field, sAI[eCh].Blocked);
		return erFAILURE;
	}
	return erSUCCESS;
}

// ##################### Hardware dependent (DIG/PWM/ANA) local-only functions #####################

#if	(halXXX_DIG_OUT > 0)		// All (SOC + I2C + SPI) DIGital type actuators
/**
 * @brief	LL=NL
 */
static void IRAM_ATTR vActuateSetLevelDIG(u8_t eCh, u8_t NewState) {
	switch(ActInit[eCh].ioType) {					// handle hardware dependent component
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG: halGDO_SetState(ActInit[eCh].ioNum, NewState); break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM: halGPO_SetState(ActInit[eCh].ioNum, NewState); break;
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
		#if	(halHAS_PCA9555 > 0) && (halHAS_PCF8574 == 0)
		pca9555DIG_OUT_SetStateLazy(ActInit[eCh].ioNum, NewState);

		#elif (halHAS_PCF8574 > 0) && (halHAS_PCA9555 == 0)
		pcf8574DIG_OUT_SetState(ActInit[eCh].ioNum, NewState);

		#else
		#error "Can only support 1 or the other at any stage"
		#endif
		break;
	#endif

	default: xActuatorLogError(__FUNCTION__, eCh);
	}
}

/**
 * @brief	LL=NL
 */
static int xActuateGetLevelDIG(u8_t eCh) {
	int iRV = erFAILURE;
	switch(ActInit[eCh].ioType) {						// handle hardware dependent component
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
		iRV = halGDO_GetState(ActInit[eCh].ioNum);
		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		iRV = halGPIO_PWM_OUT_GetState(ActInit[eCh].ioNum);
		break;
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
		#if	(halHAS_PCA9555 > 0)
		iRV = pca9555DIG_OUT_GetState(ActInit[eCh].ioNum);

		#elif	(halHAS_PCF8574 > 0)
		iRV = pcf8574DIG_IO_GetState(ActInit[eCh].ioNum);

		#else
		myASSERT(0);
		#endif
		break;
	#endif

	default: xActuatorLogError(__FUNCTION__, eCh);
	}
	return iRV;
}
#endif

/**
 * @brief	LL=NL configure channel for a specific [soft] PWM frequency
 * @brief	The timer will be stopped and a new frequency will be configured
 * 			The timer will NOT be restarted until a new duty cycle is configured
 * @param	eCh - logical PWM channel
 * @param	Frequency - desired frequency trimmed to be within the supported range
 * @return	none
 */
static void vActuatorSetFrequency(u8_t eCh, u32_t Frequency) {
	switch(ActInit[eCh].ioType) {			// handle hardware dependent component
	#if	(halXXX_DIG_OUT > 0)
	case actSOC_DIG:
	case actI2C_DIG:
	case actSPI_DIG:
		FIT2RANGE(actFREQ_MIN, Frequency, actFREQ_MAX, u32_t);
		sAI[eCh].Divisor = (MILLIS_IN_SECOND / actuateTASK_PERIOD) / Frequency;
 		break;
	#endif

	#if	(halSOC_ANA_OUT > 0)
	case actSOC_ANA:
		FIT2RANGE(actFREQ_MIN, Frequency, actFREQ_MAX, u32_t);
		sAI[eCh].Divisor = (MILLIS_IN_SECOND / actuateTASK_PERIOD) / Frequency;
		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		FIT2RANGE(halPWM_MIN_FREQ, Frequency, halPWM_MAX_FREQ, u32_t);
		sAI[eCh].Divisor = (halPWM_CLOCK_HZ / Frequency);
		halGPIO_PWM_OUT_SetFrequency(ActInit[eCh].ioNum, sAI[eCh].Divisor - 1);
		break;
	#endif

	#if	(halI2C_PWM_OUT > 0)
	case actI2C_PWM:
	#endif

	#if	(halSPI_PWM_OUT > 0)
	case actSPI_PWM:
	#endif

	default: xActuatorLogError(__FUNCTION__, eCh);
	}
}

/**
 * @brief	LL=NL Recalc & set duty cycle (brightness/speed level)
 * @param	logical (soft) PWM channel
 */
static void IRAM_ATTR vActuatorSetDC(u8_t eCh, u8_t CurDC) {
	act_info_t * psAI = &sAI[eCh];
	psAI->CurDC = CurDC;
	switch(ActInit[eCh].ioType) {
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG:
	#endif
	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG:
	#endif
	#if	(halSPI_DIG_OUT > 0)
	case actSPI_DIG:
	#endif
	#if	(halXXX_DIG_OUT > 0)		// All (SOC + I2C + SPI) DIGital type actuators
		switch(psAI->StageNow) {
		case actSTAGE_FI: psAI->Match	= psAI->MaxDC - psAI->CurDC; break;
		case actSTAGE_ON: psAI->Match	= psAI->MinDC; break;
		case actSTAGE_FO: psAI->Match	= psAI->MaxDC - psAI->CurDC; break;
		case actSTAGE_OFF: psAI->Match	= psAI->MaxDC; break;
		}
		vActuateSetLevelDIG(eCh, (psAI->Count >= psAI->Match) ? 1 : 0);
		break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM:
		psAI->Match = u32ScaleValue(CurDC, psAI->MinDC, psAI->MaxDC, halPWM_MIN_FREQ, halPWM_MAX_FREQ);
		halGPIO_PWM_OUT_SetCycle(ActInit[eCh].ioNum, psAI->Match);
		break;
	#endif

	#if	(halI2C_PWM_OUT > 0)
		case actI2C_PWM: myASSERT(0); break;
	#endif

	#if	(halSPI_PWM_OUT > 0)
		case actSPI_PWM: myASSERT(0); break;
	#endif

	#if	(halSOC_ANA_OUT > 0)
	case actSOC_ANA:
		switch(psAI->StageNow) {
		case actSTAGE_FI: psAI->Match	= psAI->MaxDC - psAI->CurDC; break;
		case actSTAGE_ON: psAI->Match	= psAI->MinDC; break;
		case actSTAGE_FO: psAI->Match	= psAI->MaxDC - psAI->CurDC; break;
		case actSTAGE_OFF: psAI->Match	= psAI->MaxDC; break;
		}
		halGAO_WriteRAW(ActInit[eCh].ioNum, (255 * psAI->CurDC) / 100);
		break;
	#endif

	#if	(halI2C_ANA_OUT > 0)
	case actI2C_ANA: myASSERT(0); break;
	#endif

	#if	(halSPI_ANA_OUT > 0)
	case actSPI_ANA: myASSERT(0); break;
	#endif

	default: xActuatorLogError(__FUNCTION__, eCh);
	}
	IF_EXEC_2(debugDUTY, vActuatorReportChan, NULL, eCh);
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
 * @brief	configure the hardware pin associated with a channel
 *			Uses the definitions in the hal_gpio module to define the specific pin,
 * 			its configuration and (optionally) the associated timer module for hard PWM
 * @param	Channel
 * @return	None
 */
static void vActuatorConfig(u8_t eCh) {
	const act_init_t * psAIS = &ActInit[eCh];
	IF_RETURN(sAI[eCh].Blocked);
	switch(psAIS->ioType) {					// handle hardware dependent component
	#if	(halSOC_DIG_OUT > 0)
	case actSOC_DIG: vActuatorSetFrequency(eCh, actFREQ_DEF_DIG); break;
	#endif

	#if	(halSOC_PWM_OUT > 0)
	case actSOC_PWM: vActuatorSetFrequency(eCh, halFREQ_DEF_PWM); break;
	#endif

	#if	(halSOC_ANA_OUT > 0)
	case actSOC_ANA: vActuatorSetFrequency(eCh, actFREQ_DEF_ANA); break;
	#endif

	#if	(halI2C_DIG_OUT > 0)
	case actI2C_DIG: vActuatorSetFrequency(eCh, actFREQ_DEF_DIG); break;
	#endif

	#if	(halI2C_PWM_OUT > 0)							// Not yet supported/tested
	case actI2C_PWM: vActuatorSetFrequency(eCh, actFREQ_DEF_PWM); break;
	#endif

	#if	(halI2C_ANA_OUT > 0)							// Not yet supported/tested
	case actI2C_ANA: vActuatorSetFrequency(eCh, actFREQ_DEF_ANA); break;
	#endif

	default: xActuatorLogError(__FUNCTION__, eCh); return;
	}
	act_info_t * psAID = &sAI[eCh];
	memset(psAID->Seq, 0xFF, SO_MEM(act_info_t, Seq));
	psAID->StageBeg = psAID->StageNow	= actSTAGE_FI;
	psAID->ChanNum = eCh;
	psAID->MaxDC = psAID->DelDC = 100;
	psAID->ConfigOK = 1;
	vActuatorSetDC(eCh, psAID->MinDC = 0);
	IF_EXEC_2(debugTRACK && (ioB2GET(dbgActuate) & 2), vActuatorReportChan, NULL, eCh);
}

/**
 * @brief	LowLevel-NoLocking timing values are supplied in mSec, converted and stored as ticks
 * @param	eCh, tOFF, tFI, tON, tFO
 * @return	erFAILURE or erSUCCESS
 */
static void vActuatorSetTiming(u8_t eCh, u32_t tFI, u32_t tON, u32_t tFO, u32_t tOFF) {
	act_info_t	* psAI = &sAI[eCh];
	// set configuration to max 1 day...
	psAI->tFI = pdMS_TO_TICKS(tFI > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFI);
	psAI->tON = pdMS_TO_TICKS(tON > MILLIS_IN_DAY ? MILLIS_IN_DAY : tON);
	psAI->tFO  = pdMS_TO_TICKS(tFO > MILLIS_IN_DAY ? MILLIS_IN_DAY : tFO);
	psAI->tOFF = pdMS_TO_TICKS(tOFF > MILLIS_IN_DAY ? MILLIS_IN_DAY : tOFF);
	IF_PT(debugTRACK && (ioB2GET(dbgActuate) & 2), "[ACT] SetTiming Ch=%d tFI=%u tON=%u tFO=%u tOFF=%u\r\n", eCh, tFI, tON, tFO, tOFF);
}

/**
 * @brief	LL-NL
 */
static void vActuatorStart(u8_t eCh, u32_t Repeats) {
	act_info_t * psAI = &sAI[eCh];
	psAI->tNOW = psAI->Count = 0;
	psAI->StageNow = psAI->StageBeg;
	psAI->CurDC	= psAI->MinDC;
	psAI->Match	= psAI->tNOW;
	if (psAI->tXXX[psAI->StageNow]) {
		psAI->CurDC	+= ((psAI->tNOW * psAI->DelDC) / psAI->tXXX[psAI->StageNow]);
		psAI->Match	= psAI->tNOW / ( psAI->tXXX[psAI->StageNow] / psAI->Divisor);
	}
	vActuatorSetDC(eCh, psAI->CurDC);
	psAI->Rpt = Repeats;
	xRtosSetTaskRUN(taskACTUATE_MASK);
	IF_PT(debugTRACK && (ioB2GET(dbgActuate) & 2), "[ACT] Start Ch=%d Rpt=%d\r\n", eCh, Repeats);
}

/**
 * @brief	LL-NL
 */
static void vActuatorStop(u8_t eCh) {
	act_info_t * psAI = &sAI[eCh];
	// reset ONLY the tXXX values (incl Rpt + tNow)
	memset(&psAI->tXXX, 0, sizeof(sAI[0].tXXX));
	memset(&psAI->Seq, 0xFF, sizeof(sAI[0].Seq));
	psAI->StageNow	= psAI->StageBeg;
	psAI->alertDone	= psAI->alertStage	= 0;
	vActuatorSetDC(eCh, 0);
	IF_PT(debugTRACK && (ioB2GET(dbgActuate) & 2), "[ACT] Stop Ch=%d\r\n", eCh);
}

/**
 * @brief	LL-NL
 */
static void vActuatorAddSequences(u8_t eCh, int Idx, u8_t * paSeq) {
	act_info_t * psAI = &sAI[eCh];
	for (; Idx < actMAX_SEQUENCE; ++Idx) {
		if (*paSeq < NO_MEM(sAS)) {						// if a valid SEQuence number
			psAI->Seq[Idx] = *paSeq++; 					// store it
		} else {
			psAI->Seq[Idx] = 0xFF; 						// if not, mark it unused
			break; 										// and go no further
		}
	}
	IF_EXEC_2(debugTRACK && (ioB2GET(dbgActuate) & 2), vActuatorReportChan, NULL, eCh);
}

/**
 * @brief	LL-NL
 */
static void IRAM_ATTR xActuatorNextStage(act_info_t * psAI) {
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
				if (psAI->Seq[0] != 0xFF) {				// another sequence in the queue?
					const act_seq_t * psAS = &sAS[psAI->Seq[0]];	// load values from sequence #
					vActuatorSetTiming(psAI->ChanNum, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
					vActuatorStart(psAI->ChanNum, psAS->Rpt);
					int Idx;
					for (Idx = 0;  Idx < (actMAX_SEQUENCE - 1); ++Idx)
						psAI->Seq[Idx] = psAI->Seq[Idx+1];
					psAI->Seq[Idx] = 0xFF;
				} else { 								// no, all sequences done
					vActuatorStop(psAI->ChanNum);		// stop & reset all values..
				}
			}
		}
	}
}

/**
 * @brief	LL-NL
 */
static void IRAM_ATTR vActuatorUpdateTiming(act_info_t * psAI) {
	psAI->Count	+= actuateTASK_PERIOD;
	if (psAI->Count >= psAI->Divisor) psAI->Count = 0;
	psAI->tNOW	+= actuateTASK_PERIOD;
	if (psAI->tNOW >= psAI->tXXX[psAI->StageNow]) {
		psAI->tNOW = psAI->Count = 0;
		xActuatorNextStage(psAI);
	}
}

// ####################################### Actual task #############################################

static void IRAM_ATTR vTaskActuator(void * pvPara) {
	IF_SYSTIMER_INIT(debugTIMING, stACT_S0, stMICROS, "ActS0_FI", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S1, stMICROS, "ActS1_ON", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S2, stMICROS, "ActS2_FO", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_S3, stMICROS, "ActS3_OF", 1, 10);
	IF_SYSTIMER_INIT(debugTIMING, stACT_SX, stMICROS, "ActSXall", 1, 100);
	vTaskSetThreadLocalStoragePointer(NULL, buildFRTLSP_EVT_MASK, (void *)taskACTUATE_MASK);
	#if (halUSE_I2C == 1)
	(void)xRtosWaitStatus(flagAPP_I2C, portMAX_DELAY);	// ensure I2C config done before initialising
	#endif
	for(u8_t eCh = 0; eCh < halXXX_XXX_OUT; vActuatorConfig(eCh++));
	xRtosSetTaskRUN(taskACTUATE_MASK);
	while(bRtosTaskWaitOK(taskACTUATE_MASK, portMAX_DELAY)) {
		TickType_t	ActLWtime = xTaskGetTickCount();    // Get the ticks as starting reference
		IF_SYSTIMER_START(debugTIMING, stACT_SX);
		act_info_t * psAI = &sAI[0];
		ActuatorsRunning = 0;
		for (u8_t eCh = 0; eCh < halXXX_XXX_OUT;  ++eCh, ++psAI) {
			if (!psAI->Rpt || psAI->Blocked || !psAI->ConfigOK)	continue;
			++ActuatorsRunning;
			if (psAI->Busy) continue;					// being changed from somewhere else
			psAI->Busy = 1;
			switch(psAI->StageNow) {
			case actSTAGE_FI:							// Step UP from 0% to 100% over tFI mSec
				IF_SYSTIMER_START(debugTIMING, stACT_S0);
				if (psAI->tFI > 0) {
					vActuatorSetDC(eCh, psAI->MinDC + ((psAI->tNOW * psAI->DelDC) / psAI->tFI));
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
					if (psAI->tNOW == 0) vActuatorSetDC(eCh, psAI->MaxDC);
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
					vActuatorSetDC(eCh, psAI->MaxDC - ((psAI->tNOW * psAI->DelDC) / psAI->tFO));
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
					if (psAI->tNOW == 0) vActuatorSetDC(eCh, psAI->MinDC);
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

		#if	(halHAS_PCA9555 == 1)
		/* Considering that we might be running actuator task every 1mS and that it is
		 * possible for every I2C connected actuator pin to change state every 1mS, we
		 * could be trying to write each bit each 1mS, hence 16x I2C writes per 1mS.
		 * Hence we are doing a batch max write once per cycle, if any bit changed */
		if (pca9555DIG_OUT_WriteAll() == 1) {			// if it was a dirty write, check if write OK...
			/* With both water valves and door strikers we have a situation where a reverse EMF is induced
			 * in the solenoid when power is removed from the actuator. This EMF can, if left undamped,
			 * reflect back along the cabling to the controller and has been knows to cause I2C bus problems.
			 * In order to damp the EMF right at the source a reverse biased signal diode should be wired
			 * across. the solenoid connectors, as close as possible to the source. To diagnose possible
			 * diode absence or failure we regularly perform a check to verify the actual I2C device state
			 * against what we believe it should be */
			pca9555Check();
		}
		#endif

		IF_SYSTIMER_STOP(debugTIMING, stACT_SX);
		if (ActuatorsRunning) {							// Some active actuators, delay till next cycle
			vTaskDelayUntil(&ActLWtime, actuateTASK_PERIOD);
		} else {										// NO active actuators
			xRtosClearTaskRUN(taskACTUATE_MASK); 		// clear RUN state & wait at top....
		}
	}
	vRtosTaskDelete(NULL);
}

// ######################################### Public APIs ###########################################

void vTaskActuatorInit(void) {
	xRtosTaskCreateStatic(vTaskActuator, "actuate", actuateSTACK_SIZE, NULL, actuateTASK_PRIORITY, tsbACT, &ttsACT, tskNO_AFFINITY);
}

void vActuatorLoad(u8_t eCh, u32_t Rpt, u32_t tFI, u32_t tON, u32_t tFO, u32_t tOFF) {
	IF_myASSERT(debugTRACK, eCh < halXXX_XXX_OUT);
	IF_myASSERT(debugTRACK, sAI[eCh].ConfigOK);
	IF_myASSERT(debugTRACK, !sAI[eCh].Blocked);
	vActuatorBusySET(&sAI[eCh]);
	vActuatorStop(eCh);
	vActuatorSetTiming(eCh, tFI, tON, tFO, tOFF);
	vActuatorStart(eCh, Rpt);
	vActuatorBusyCLR(&sAI[eCh]);
	IF_EXEC_2(debugTRACK && (ioB2GET(dbgActuate) & 2), vActuatorReportChan, NULL, eCh);
}

void vActuatorUpdate(u8_t eCh, int Rpt, int tFI, int tON, int tFO, int tOFF) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && (sAI[eCh].Blocked == 0));
	act_info_t * psAI = &sAI[eCh];
	vActuatorBusySET(psAI);
	u32_t CurRpt = sAI[eCh].Rpt;
	sAI[eCh].Rpt = 0;
	// XXX: Add range checking to not wrap around any member
	psAI->tFI += (tFI * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tON += (tON * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tFO += (tFO * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	psAI->tOFF += (tOFF * configTICK_RATE_HZ) / MILLIS_IN_SECOND;
	sAI[eCh].Rpt = (CurRpt == 0xFFFFFFFF) ? CurRpt : CurRpt + Rpt;
	vActuatorBusyCLR(psAI);
}

void vActuatorAdjust(u8_t eCh, int Stage, int Adjust) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && !sAI[eCh].Blocked && INRANGE(0, Stage, actSTAGE_OFF));
	act_info_t * psAI = &sAI[eCh];
	Adjust = (Adjust * configTICK_RATE_HZ) / MILLIS_IN_SECOND; 	// convert adjustment to Ticks
	vActuatorBusySET(psAI);
	u32_t CurVal = psAI->tXXX[Stage]; 			// save the selected stage value
	u32_t NewVal = CurVal + Adjust;
	if (Adjust < 0) psAI->tXXX[Stage] = NewVal < CurVal ? NewVal : 0;
	else psAI->tXXX[Stage] = NewVal > CurVal ? NewVal : UINT32_MAX;
	vActuatorBusyCLR(psAI);
	IF_EXEC_2(debugTRACK && (ioB2GET(dbgActuate) & 2), vActuatorReportChan, NULL, eCh);
}

void xActuatorToggle(u8_t eCh) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && !sAI[eCh].Blocked);
	act_info_t * psAI = &sAI[eCh];
	vActuatorBusySET(psAI);
	SWAP(psAI->tFI, psAI->tFO, u32_t);
	SWAP(psAI->tON, psAI->tOFF, u32_t);
	vActuatorBusyCLR(psAI);
}

void vActuatorBreath(u8_t eCh) { vActuatorLoad(eCh, 0xFFFFFFFF, 750, 750, 750, 750); }

void vActuatorPanic(u8_t eCh) { vActuatorLoad(eCh, 0xFFFFFFFF, 150, 150, 150, 150); }

void vActuatorOn(u8_t eCh) { vActuatorLoad(eCh, 0xFFFFFFFF, 0, 0xFFFFFFFF, 0, 0); }

void vActuatorOff(u8_t eCh) { vActuatorLoad(eCh, 0xFFFFFFFF, 0, 0, 0, 0xFFFFFFFF); }

int xActuatorRunningCount(void) { return ActuatorsRunning; }

u64_t xActuatorGetRemainingTime(u8_t eCh) {
	IF_myASSERT(debugTRACK, eCh < halXXX_XXX_OUT);
	IF_myASSERT(debugTRACK, sAI[eCh].ConfigOK == 1);
	IF_myASSERT(debugTRACK, sAI[eCh].Blocked == 0);
	act_info_t * psAI = &sAI[eCh];
	IF_RETURN_X(psAI->Rpt == UINT32_MAX, UINT64_MAX);		// indefinite/unlimited repeat ?
	IF_RETURN_X(psAI->Rpt == 0, 0);
	// calculate remaining time for full repeats
	vActuatorBusySET(psAI);
	u64_t u64Value = (psAI->Rpt > 1) ? (psAI->tFI + psAI->tON + psAI->tFO + psAI->tOFF) * (psAI->Rpt - 1) : 0;
	u8_t Stage = psAI->StageNow;						// now add remaining time in current stage
	do {
		u64Value += (Stage == psAI->StageNow) ? psAI->tXXX[Stage] - psAI->tNOW : psAI->tXXX[Stage];
		++Stage;
		Stage %= actSTAGE_NUM;
	} while (Stage != psAI->StageBeg);

	// now add the time for the (optional) sequences
	for (int Idx = 0; psAI->Seq[Idx] < NO_MEM(sAS); ++Idx) {
		const act_seq_t * psAS = &sAS[psAI->Seq[Idx]];
		u64Value += psAS->Rpt * (psAS->tFI + psAS->tON + psAS->tFO + psAS->tOFF);
	}
	vActuatorBusyCLR(psAI);
	return u64Value;
}

/**
 * Determines the longest remaining running time, not the sum of ALL
 * @return Remaining maximum actuator running time in uSecs
 */
u64_t xActuatorGetMaxRemainingTime (void) {
	u64_t u64Now, u64Max = 0.0;
	for (int eCh = 0; eCh < halXXX_XXX_OUT; ++eCh) {
		u64Now = xActuatorGetRemainingTime(eCh);
		if (u64Now > u64Max) u64Max = u64Now;
	}
	return u64Max * MICROS_IN_MILLISEC;
}

/* ############################ Actuator alerting support functions ################################
 * Start (OFF -> FI/ON)
 * Stop (ON/FO -> OFF)
 * Stage (OFF -> FI -> ON -> FO -> OFF)
 * Event Blocked due to any reason, especially pending restart
 */

void xActuatorSetAlertStage(u8_t eCh, int OnOff) {
	act_info_t	* psAI = &sAI[eCh];
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && psAI->ConfigOK && !psAI->Blocked);
	psAI->alertStage = OnOff ? 1 : 0;
}

void xActuatorSetAlertDone(u8_t eCh, int OnOff) {
	act_info_t	* psAI = &sAI[eCh];
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && psAI->ConfigOK && !psAI->Blocked);
	psAI->alertDone = OnOff ? 1 : 0;
}

void xActuatorSetStartStage(u8_t eCh, int Stage) {
	act_info_t	* psAI = &sAI[eCh];
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && psAI->ConfigOK && !psAI->Blocked && INRANGE(actSTAGE_FI, Stage, actSTAGE_OFF));
	psAI->StageBeg = Stage;
}

void vActuatorSetMinMaxDC(u8_t eCh, int iMin, int iMax) {
	act_info_t	* psAI = &sAI[eCh];
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && psAI->ConfigOK && !psAI->Blocked);
	IF_myASSERT(debugTRACK, INRANGE(0, iMin, 100) && INRANGE(0, iMax, 100));
	if (iMin > iMax) SWAP(iMin, iMax, u8_t);
	IF_PT(debugDUTY_CYCLE, "[ACT] SetMMDC Ch=%d  Min=%d->%d  Max=%d->%d\r\n", eCh, iMin, psAI->MinDC, iMax, psAI->MaxDC);
	psAI->MinDC = iMin;
	psAI->MaxDC = iMax;
}

void vActuatorBlock(u8_t eCh) {
	IF_myASSERT(debugTRACK, eCh < halXXX_XXX_OUT);
	sAI[eCh].Blocked = 1;
}

void vActuatorUnBlock(u8_t eCh) {
	IF_myASSERT(debugTRACK, eCh < halXXX_XXX_OUT);
	sAI[eCh].Blocked = 0;
}

/**
 * @brief	Load up to a maximum of actMAX_SEQUENCE sequence numbers to the
 * 			sequence table, overwriting any existing (pending) sequences
 *			Expects a full array of actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param	eCh		Channel number
 * @param	paSeq		pointer to array of sequence numbers
 * @return
 */
void xActuatorLoadSequences(u8_t eCh, u8_t * paSeq) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && !sAI[eCh].Blocked);
	IF_myASSERT(debugTRACK, halCONFIG_inMEM(paSeq));
	vActuatorAddSequences(eCh, 0, paSeq);
}

/**
 * @brief	Append additional sequence numbers to the end of the sequence table
 *			Expects an array of up to actMAX_SEQUENCE size, unused elements to be 0xFF
 * @param	eCh	- Channel number
 * @param	paSeq - pointer to array of sequence numbers
 * @return	erFAILURE if none appended else number of sequences appended
 */
void vActuatorQueSequences(u8_t eCh, u8_t * paSeq) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && !sAI[eCh].Blocked);
	IF_myASSERT(debugTRACK, halCONFIG_inFLASH(paSeq));
	for (int Idx = 0; Idx < actMAX_SEQUENCE; ++Idx) {
		if (sAI[eCh].Seq[Idx] == 0xFF) { vActuatorAddSequences(eCh, Idx, paSeq); return; }
	}
}

void vActuatorStartSequence(u8_t eCh, int Seq) {
	IF_myASSERT(debugTRACK, (eCh < halXXX_XXX_OUT) && sAI[eCh].ConfigOK && !sAI[eCh].Blocked);
	IF_myASSERT(debugTRACK, INRANGE(0, Seq, actMAX_SEQUENCE-1));
	const act_seq_t * psAS = &sAS[Seq];
	vActuatorLoad(eCh, psAS->Rpt, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
}

// ############################## Private & Public reporting APIs ##################################

static void vActuatorReportChan(report_t * psR, u8_t eCh) {
	act_info_t * psAI = &sAI[eCh];
	printfx_lock(psR);
	#define HDR1 "%C Ch|Value|Stage| Repeat|  tFI  |  tON  |  tFO  |  tOFF |  tNOW | Div Cnt Mtch| Min  DC Max| Sequence%C\r\n"
	#define HDR2 " %s | %#'5d |%#'7d|%#'7d|%#'7d|%#'7d|%#'7d| %3d %3d %3d | %3d %3d %3d|"
	if (eCh == 0) wprintfx(psR, HDR1, colourFG_CYAN, attrRESET);
	wprintfx(psR, " %2d|",psAI->ChanNum);
	#if (halSOC_ANA_OUT > 0)
	if (ActInit[eCh].ioType == actSOC_ANA) wprintfx(psR, " %4hhu|", halGAO_ReadRAW(ActInit[eCh].ioNum));
	else
	#endif
	{
		bool bLevel = xActuateGetLevelDIG(eCh);
		wprintfx(psR, " %c%c%c |", CHR_0 + bLevel, psAI->Blocked ? CHR_B : CHR_SPACE, psAI->Busy ? CHR_b : CHR_SPACE);
	}
	wprintfx(psR, HDR2, StageNames[psAI->StageNow], psAI->Rpt, psAI->tFI, psAI->tON, psAI->tFO,
						psAI->tOFF, psAI->tNOW, psAI->Divisor, psAI->Count, psAI->Match,
						psAI->MinDC, psAI->CurDC, psAI->MaxDC);
	if (psAI->Blocked == 0 && psAI->Seq[0] != 0xFF) {
		for (int Idx = 0; Idx < actMAX_SEQUENCE && psAI->Seq[Idx] != 0xFF; ++Idx)
			wprintfx(psR, "%02x ", psAI->Seq[Idx]);
	}
	wprintfx(psR, strCRLF);
	printfx_unlock(psR);
}

static void vActuatorReportSeq(report_t * psR, u8_t Seq) {
	const act_seq_t * psAS = &sAS[Seq];
	printfx_lock(psR);
	#define HDR3 "%CSeq |Repeat|  tFI  |  tON  |  tFO  |  tOFF |%C\r\n"
	#define HDR4 " %2d | %#'5u|%#'7u|%#'7u|%#'7u|%#'7u|\r\n"
	if (Seq == 0) wprintfx(psR, HDR3, colourFG_CYAN, attrRESET);
	wprintfx(psR, HDR4, Seq, psAS->Rpt, psAS->tFI, psAS->tON, psAS->tFO, psAS->tOFF);
	printfx_unlock(NULL);
}

void vTaskActuatorReport(report_t * psR) {
	for (u8_t eCh = 0; eCh < halXXX_XXX_OUT;  vActuatorReportChan(psR, eCh++));
	for (u8_t Seq = 0; Seq < NO_MEM(sAS); vActuatorReportSeq(psR, Seq++));
	wprintfx(psR, "Running=%u  maxDelay=%!.R\r\n\n", xActuatorRunningCount(), xActuatorGetMaxRemainingTime());
}

// ############################## Rules interface to Actuator table ################################

double	dActuatorGetFieldValue(u8_t eCh, u8_t Field, v64_t * px64Var) {
	IF_myASSERT(debugTRACK, halCONFIG_inSRAM(px64Var));
	x64_t x64Value = { .f64 = 0.0 };
	if (xActuatorVerifyParameters(eCh, Field) != erFAILURE) {
		px64Var->def = SETDEF_CVAR(0, 0, vtVALUE, cvU32, 1, 0);
		act_info_t * psAI = &sAI[eCh];
		if (Field < selACT_T_REM) {							// all these are real tXXX fields/stages
			x64Value.f64 				= psAI->tXXX[Field-selACT_T_FI];
			px64Var->val.x64.x32[0].u32 = psAI->tXXX[Field-selACT_T_FI];
		} else {
			x64Value.f64 = (double) xActuatorGetRemainingTime(eCh);
			px64Var->val.x64.x32[0].u32 = (u32_t) x64Value.f64;
			IF_P(debugFUNC_RULES, "F64=%f", x64Value.f64);
		}
		IF_P(debugFUNC_RULES, "%s: C=%d  F=%d  I=%d  V=%'lu\r\n", __FUNCTION__, eCh, Field, Field-selACT_T_FI, px64Var->val.x64.x32[0].u32);
	}
	return x64Value.f64;
}

int	xActuatorSetFieldValue(u8_t eCh, u8_t Field, v64_t * px64Var) {
	IF_myASSERT(debugTRACK, halCONFIG_inSRAM(px64Var));
	if (xActuatorVerifyParameters(eCh, Field) != erFAILURE) {
		sAI[eCh].tXXX[Field-selACT_T_FI] = px64Var->val.x64.x32[0].u32;
		IF_P(debugFUNC_RULES, "F=%d  I=%d  V=%'lu\r\n", Field, Field-selACT_T_FI, sAI[eCh].tXXX[Field-selACT_T_FI]);
		return erSUCCESS;
	}
	return erFAILURE;
}

int	xActuatorUpdateFieldValue(u8_t eCh, u8_t Field, v64_t * px64Var) {
	IF_myASSERT(debugTRACK, halCONFIG_inSRAM(px64Var));
	if (xActuatorVerifyParameters(eCh, Field) != erFAILURE) {
		u32_t CurVal = sAI[eCh].tXXX[Field-selACT_T_FI];
		if ((px64Var->val.x64.x32[0].i32 < 0) && (CurVal >= abs(px64Var->val.x64.x32[0].i32))) {
			CurVal	+= px64Var->val.x64.x32[0].i32;
		} else {
			CurVal	= 0;
		}
		sAI[eCh].tXXX[Field-selACT_T_FI] = CurVal;
		IF_P(debugFUNC_RULES, "F=%d  I=%d  V=%'lu\r\n", Field, Field-selACT_T_FI, sAI[eCh].tXXX[Field-selACT_T_FI]);
		return erSUCCESS;
	}
	return erFAILURE;
}

// ############################# DUMMY stub for actuator MODE support ##############################

#define	SCALE		4
#define	tBASE		(3000 * SCALE)
#define	tSTEP		(500 * SCALE)

int xActuatorsConfigMode(rule_t * psR, int Xcur, int Xmax) {
	do {
		u32_t tXX = tBASE + (Xcur * tSTEP);
		vActuatorLoad(Xcur, 2, 0, tXX, 0, tXX);
		vActuatorReportChan(NULL, Xcur);
	} while (++Xcur < Xmax);
	return erSUCCESS;
}

// ##################################### functional tests ##########################################

void vActuatorTestReport(u8_t eCh, char * pcMess) {
	IF_myASSERT(debugPARAM, eCh < halXXX_XXX_OUT);
	act_info_t * psAI = &sAI[0];
	printfx("%s #%d Stage:%d Rpt:%d tFI:%d tON:%d tFO:%d tOFF:%d tNOW:%d ",
				pcMess, eCh, psAI->StageNow, psAI->Rpt,
				psAI->tFI, psAI->tON, psAI->tFO, psAI->tOFF, psAI->tNOW);
	switch(ActInit[eCh].ioType) {
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

		printfx("(%s) Div:%d Match:%d\r\n", ActTypeNames[ActInit[eCh].ioType], psAI->Divisor, psAI->Match);
		break;
#endif

	default:
		printfx("Invalid actuator type=%d\r\n", ActInit[eCh].ioType);
		break;
	}
}

void vActuatorTest(void) {
	// Test PHYSical level functioning
	#if	(debugPHYS || debugFUNC || debugUSER)
	if (i2cDevCount)
		xRtosWaitStatus(flagAPP_I2C, portMAX_DELAY);
	#endif

	#if	(debugPHYS)
	for(u8_t eCh = 0; eCh < halXXX_XXX_OUT; ++eCh) {
		vActuatorConfig(eCh);
		vActuateSetLevelDIG(eCh, 1);
		vTaskDelay(1000);
		vActuateSetLevelDIG(eCh, 0);
	}
	#endif

	#if	(debugFUNC)
	for(u8_t eCh = 0; eCh < halXXX_XXX_OUT; ++eCh) {
		vActuatorConfig(eCh);
		for(u32_t Freq = actFREQ_MIN;  Freq <= actFREQ_MAX; Freq *= 5) {
			vActuatorSetFrequency(eCh, Freq);
			vActuatorSetTiming(eCh, 0, 0, UINT32_MAX, 0);
			vActuatorStart(eCh, UINT32_MAX);
			for(int8_t CurDC = 0; CurDC <= 100;  CurDC = (CurDC == 0) ? 1 : CurDC * 2) {
				vActuatorSetDC(eCh, CurDC);
				SL_INFO("DIG: eCh=%d  Freq=%'u  Lev=%'u\r\n", eCh, Freq, CurDC);
				getchar();
			}
		}
	}
	#endif

	#if	(debugUSER)
	for(u8_t eCh = 0; eCh < halXXX_XXX_OUT; ++eCh) {
		vActuatorConfig(eCh);
		vActuatorSetFrequency(eCh, 1000);

		vActuatorLoad(eCh, 1, 1000, 1000, 1000, 1000);
		vActuatorTestReport(eCh, "1s On/Off each, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(4000));

		vActuatorLoad(eCh, 1, 2000, 2000, 2000, 2000);
		vActuatorTestReport(eCh, "2s per phase, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(8000));

		vActuatorLoad(eCh, 1, 5000, 5000, 5000, 5000);
		vActuatorTestReport(eCh, "5s per phase, 0->100%");
		vTaskDelay(pdMS_TO_TICKS(20000));
	}
	#endif
}
#endif
