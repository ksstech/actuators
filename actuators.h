/*
 * x_actuators.h
 */

#pragma once

#include	"hal_config.h"
#include	"FreeRTOS_Support.h"
#include	"complex_vars.h"			// struct_union x_time definitions time stdint

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Theory of operation:	DIGital outputs
 * ------------------------------------
 * Minimum interval for ANYTHING is 1mS (1 or more ticks)
 * Fixed number of levels = 100 to provide 0->100 %
 * Therefore, any stage being OFF -> FI -> ON -> FO must a multiple of 100mS
 * To control the intensity of an LED using a DIGital 0/1 pin...
 * we control the number of mSec (of this fixed 100mS period) that pin is 0/1
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
 * Theory of operation:	PWM outputs
 * --------------------------------
 *
 *
 */

// ####################################### MACROS ##################################################

#define	ACTUATE_PRIORITY			8
#define	ACTUATE_STACK_SIZE			(configMINIMAL_STACK_SIZE + 1024 + (flagSTACK * 256))
#define	ACTUATE_TASK_PERIOD			2

#define	actDIG_CLOCK_HZ				configTICK_RATE_HZ
#define	actDIG_MIN_COUNT			1		// steps to go 0 -> 100%
#define	actDIG_MIN_FREQ				1
#define	actDIG_MAX_FREQ				(actDIG_CLOCK_HZ / actDIG_MIN_COUNT)
/* default was 10, timing worked exactly but flashing is prominent
 * 25 reduces flashing, provides more steps in intensity, smoother FI & FO
 * 50 is very smooth, no/minimal flicker, but slightly abrupt at end of FO
 * 100 is smooth but FI/FO look abrupt at start & end
 */
#define	actDIG_DEF_FREQ				33

#define	actMIN_DUTYCYCLE			0
#define	actMAX_DUTYCYCLE			100

#define	actMAX_SEQUENCE				8
#define actNUM_SEQUENCES			10

// ######################################### Enumerations #########################################

enum {													// interface SOC/I2C/SPI & type DIG/PWM/ANA
	actSOC_DIG,											// All (DIGital + PWM + ANAlog) SoC integrated actuators
	actSOC_PWM,
	actSOC_ANA,
	actI2C_DIG,											// All (DIGital + PWM + ANAlog) I2C connected actuators
	actI2C_PWM,
	actI2C_ANA,
	actSPI_DIG,											// All (DIGital + PWM + ANAlog) SPI connected actuators
	actSPI_PWM,
	actSPI_ANA,
	actXXX_NUM,											// last item
} ;

enum { actSTAGE_FI, actSTAGE_ON, actSTAGE_FO, actSTAGE_OFF, actSTAGE_NUM, } ;// Actuator Stages

// ########################################## Structures ##########################################

typedef struct act_init_t {
	uint8_t		Type ;
	uint8_t		halGPIO ;
} act_init_t ;
DUMB_STATIC_ASSERT(sizeof(act_init_t) == 2) ;

typedef struct act_info_t {								// Actuator structure
	union {												// all values in TICKS not mSec
		struct { uint32_t	tFI, tON, tFO, tOFF, Rpt, tNOW ; } ;
		uint32_t tXXX[actSTAGE_NUM + 2] ;				// +2 for tNOW & Rpt
	} ;
	uint8_t		Seq[actMAX_SEQUENCE] ;					// number of queued sequences
	uint32_t	Divisor ;								// number of clocks in a cycle
	uint32_t	Match ;									// level for switching 0/1 output
	uint32_t	Count ;
	uint8_t		MinDC ;									// 0% = OFF
	uint8_t		MaxDC ;									// 100% = ON
	uint8_t		DelDC ;									// MaxDC - MinDC
	uint8_t		CurDC ;									// Current DutyCycle 0 -> 100%
	uint8_t		StageBeg ;								// Stage to start a cycle with, primarily for lead/trail OFF
	uint8_t		StageNow ;								// Current stage for actuator
	uint8_t		ChanNum ;
	union {
		struct {
			uint8_t	ConfigOK	: 1 ;
			uint8_t	alertStage	: 1 ;
			uint8_t	alertDone	: 1 ;
			uint8_t	alertStart	: 1 ;
			uint8_t	alertStop	: 1 ;
			uint8_t	Blocked		: 1 ;
			uint8_t	Busy		: 1 ;					// rudimentary lock between tasks/cores
			uint8_t	Spare		: 1 ;
		} ;
		uint8_t		flags ;
	} ;
} act_info_t ;
DUMB_STATIC_ASSERT(sizeof(act_info_t) == 52) ;

typedef struct act_seq_t {							// Sequence structure
	uint32_t	Rpt, tFI, tON, tFO, tOFF ;
} act_seq_t ;
DUMB_STATIC_ASSERT(sizeof(act_seq_t) == 20) ;

// ################################### Public Variables ############################################


// ################################ GLOBAL Functions Prototypes ####################################

void vTaskActuator(void * pvParameters) ;
void vTaskActuatorInit(void * pvAlertFunc) ;

void vActuatorUpdateTiming(act_info_t * pAI) ;

void vActuatorSetDC(uint8_t Chan, int8_t CurDC) ;
void vActuatorUpdateCurDC(act_info_t * pAI) ;

int32_t	xActuatorAlert(act_info_t * pAI, uint8_t Type, uint8_t Level) ;

// Hardware dependent functions
int32_t	xActuatorConfig(uint8_t Chan) ;
void	vActuatorsConfig(void) ;

int32_t	xActuatorSetAlertStage(uint8_t Chan, uint8_t State) ;
int32_t	xActuatorSetAlertDone(uint8_t Chan, uint8_t State) ;
int32_t	xActuatorSetFrequency(uint8_t Chan, uint32_t Frequency) ;

void	vActuateSetLevelDIG(uint8_t eChan, uint8_t NewState) ;
int32_t	xActuateGetLevelDIG(uint8_t eChan) ;
int32_t	xActuatorSetStartStage(uint8_t Chan, uint8_t Stage) ;
int32_t	vActuatorSetMinMaxDC(uint8_t Chan, int8_t iMin, int8_t iMax) ;
int32_t	xActuatorSetTiming(uint8_t Chan, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) ;

int32_t	xActuatorStart(uint8_t Chan, uint32_t Repeats) ;
int32_t	xActuatorStop(uint8_t Chan) ;

uint32_t xActuatorPause(uint8_t Chan) ;
int32_t	xActuatorUnPause(uint8_t Chan, uint32_t CurRpt) ;

int32_t	xActuatorBlock(uint8_t Chan) ;
int32_t	xActuatorUnBlock(uint8_t Chan) ;

int32_t	xActuatorLoad(uint8_t Chan, uint32_t Rpt, uint32_t tFI, uint32_t tON, uint32_t tFO, uint32_t tOFF) ;
int	xActuatorStartSequence(uint8_t Chan, uint8_t Seq) ;
int	xActuatorLoadSequences(uint8_t Chan, uint8_t * paSeq) ;
int	xActuatorQueSequences(uint8_t Chan, uint8_t * paSeq) ;
int	xActuatorUpdate(uint8_t Chan, int32_t Rpt, int32_t tFI, int32_t tON, int32_t tFO, int32_t tOFF) ;
int32_t	xActuatorAdjust(uint8_t Chan, uint32_t Stage, int32_t Adjust) ;

int32_t	xActuatorToggle(uint8_t Act) ;
int32_t	xActuatorBreath(uint8_t Chan) ;
int32_t	vActuatorPanic(uint8_t Chan) ;
int32_t	vActuatorOn(uint8_t Act) ;
int32_t	vActuatorOff(uint8_t Act) ;

// ############################## Rules interface to Actuator table ################################

uint64_t xActuatorGetRemainingTime(uint8_t Chan) ;
uint64_t xActuatorGetMaxRemainingTime (void) ;
uint32_t xActuatorRunningCount (void) ;

double	dActuatorGetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) ;
int32_t	xActuatorSetFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) ;
int32_t	xActuatorUpdateFieldValue(uint8_t Chan, uint8_t Field, v64_t * px64Var) ;

// ######################################## status reporting #######################################

void	vActuatorReportSeq(uint8_t Seq) ;
void	vActuatorReportChan(uint8_t Chan) ;
void	vTaskActuatorReport(void) ;

// ##################################### functional tests ##########################################

void	vActuatorTest(void)  ;
int xActuatorsConfigMode(rule_t * psR);

#ifdef __cplusplus
}
#endif
