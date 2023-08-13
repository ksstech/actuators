/*
 * x_actuators.h
 */

#pragma once

#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

// ####################################### MACROS ##################################################

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
};

enum { actSTAGE_FI, actSTAGE_ON, actSTAGE_FO, actSTAGE_OFF, actSTAGE_NUM, };// Actuator Stages

// ########################################## Structures ##########################################

typedef struct act_init_t {
	u8_t ioType:4;					// Max = 8
	u8_t ioNum:4;					// Max = 16 (PCA9555)
} act_init_t;
DUMB_STATIC_ASSERT(sizeof(act_init_t) == 1);

typedef struct act_info_t {			// Actuator structure
	union {							// all values in TICKS not mSec
		struct { u32_t	tFI, tON, tFO, tOFF, Rpt, tNOW; };
		u32_t tXXX[actSTAGE_NUM + 2];	// +2 for tNOW & Rpt
	};
	u8_t	Seq[actMAX_SEQUENCE];	// number of queued sequences
	u32_t	Divisor;				// number of ticks in a cycle
	u32_t	Match;					// level for switching 0/1 output
	u32_t	Count;					// number of ticks into current stage
	u8_t	MinDC;					// 0% = OFF
	u8_t	MaxDC;					// 100% = ON
	u8_t	DelDC;					// MaxDC - MinDC
	u8_t	CurDC;					// Current DutyCycle 0 -> 100%
	u8_t	StageBeg;				// Stage to start a cycle with, primarily for lead/trail OFF
	u8_t	StageNow;				// Current stage for actuator
	u8_t	ChanNum;
	union {
		struct {
			volatile u8_t ConfigOK	: 1;
			volatile u8_t alertStage: 1;
			volatile u8_t alertDone	: 1;
			volatile u8_t alertStart: 1;
			volatile u8_t alertStop	: 1;
			volatile u8_t Blocked	: 1;
			volatile u8_t Spare	: 1;
			volatile u8_t Busy	: 1;	// rudimentary lock between tasks/cores
		};
		volatile u8_t flags;
	};
} act_info_t;
DUMB_STATIC_ASSERT(sizeof(act_info_t) == 52);

typedef struct act_seq_t {			// Sequence structure
	u32_t	Rpt, tFI, tON, tFO, tOFF;
} act_seq_t;
DUMB_STATIC_ASSERT(sizeof(act_seq_t) == 20);

// ################################### Public Variables ############################################

extern u8_t	NumActuator;

// ################################ GLOBAL Functions Prototypes ####################################

void vTaskActuatorInit(void);

int	xActuatorSetAlertStage(u8_t Chan, int State);
int	xActuatorSetAlertDone(u8_t Chan, int State);
int	xActuatorSetStartStage(u8_t Chan, int Stage);
int	vActuatorSetMinMaxDC(u8_t Chan, int iMin, int iMax);
void vActuatorBlock(u8_t Chan);
void vActuatorUnBlock(u8_t Chan);

void vActuatorLoad(u8_t Chan, u32_t Rpt, u32_t tFI, u32_t tON, u32_t tFO, u32_t tOFF);
void vActuatorStartSequence(u8_t Chan, int Seq);
void xActuatorLoadSequences(u8_t Chan, u8_t * paSeq);
void vActuatorQueSequences(u8_t Chan, u8_t * paSeq);
void vActuatorUpdate(u8_t Chan, int Rpt, int tFI, int tON, int tFO, int tOFF);
void vActuatorAdjust(u8_t Chan, int Stage, int Adjust);

void vActuatorToggle(u8_t Act);
void vActuatorBreath(u8_t Chan);
void vActuatorPanic(u8_t Chan);
void vActuatorOn(u8_t Act);
void vActuatorOff(u8_t Act);

// ############################## Rules interface to Actuator table ################################

u64_t xActuatorGetRemainingTime(u8_t Chan);
u64_t xActuatorGetMaxRemainingTime (void);
int xActuatorRunningCount (void);

struct v64_t;
double dActuatorGetFieldValue(u8_t Chan, u8_t Field, struct v64_t * px64Var);
int	xActuatorSetFieldValue(u8_t Chan, u8_t Field, struct v64_t * px64Var);
int	xActuatorUpdateFieldValue(u8_t Chan, u8_t Field, struct v64_t * px64Var);

// ######################################## status reporting #######################################

void vTaskActuatorReport(void);

// ##################################### functional tests ##########################################

struct rule_t;
int xActuatorsConfigMode(struct rule_t * psR, int Xcur, int Xmax);
void vActuatorTest(void);

#ifdef __cplusplus
}
#endif
