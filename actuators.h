// actuators.h

#pragma once

#include "struct_union.h"
#include "alerts_id.h"							// alertTYPE_ACT_* / alertLEVEL_*, canonical values

#ifdef __cplusplus
extern "C" {
#endif

// ####################################### MACROS ##################################################

#define	actMAX_SEQUENCE				8
#define actNUM_SEQUENCES			10

#define actMAKE_DEF(T,B,N)	{ .ioType = T, .ioBus = B, .ioNum = N, }

// ######################################### Enumerations #########################################

enum { actBUS_SOC, actBUS_I2C, actBUS_SPI, actBUS_NUM };

// actTYPE_FUN: an actuator with no pin, filling the last value of the 2-bit ioType field. Scheduled
// like any other (Rpt / tON / tOFF / sequences); ON/OFF edges go to a module registered via
// xActuatorRegisterFUN(), which switches on ioNum. Guarded by HAL_XFO throughout.
enum {actTYPE_DIG, actTYPE_PWM, actTYPE_ANA, actTYPE_FUN, actTYPE_NUM };

enum {													// interface SOC/I2C/SPI & type DIG/PWM/ANA
	actSOC_DIG,											// All (DIGital + PWM + ANAlog) SoC integrated actuators
	actSOC_PWM,
	actSOC_ANA,
	actSOC_NA,
	actI2C_DIG,											// All (DIGital + PWM + ANAlog) I2C connected actuators
	actI2C_PWM,
	actI2C_ANA,
	actI2C_NA,
	actSPI_DIG,											// All (DIGital + PWM + ANAlog) SPI connected actuators
	actSPI_PWM,
	actSPI_ANA,
	actSPI_NA,
	actXXX_NUM,											// last item
};

enum { actSTAGE_FI, actSTAGE_ON, actSTAGE_FO, actSTAGE_OFF, actSTAGE_NUM };// Actuator Stages

// ########################################## Structures ##########################################

typedef union __attribute__((packed)) {
	struct __attribute__((packed)) {
		u8_t ioType:2; 				// DIGital / PWM / ANAlog
		u8_t ioBus:2; 				// SoC / I2C / SPI
		u8_t spare1:4;
		u8_t spare2:3;
		u8_t ioNum:5;				// logical channel (max 32)
	};
	u16_t u16Val;
} act_init_t;
DUMB_STATIC_ASSERT(sizeof(act_init_t) == 2);

typedef struct __attribute__((packed)) {				// Actuator structure
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
	u8_t	ChanNum;				// logical actuator number
	union {
		struct __attribute__((packed)) {
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

typedef struct act_seq_t {								// Sequence structure
	u32_t	Rpt, tFI, tON, tFO, tOFF;
} act_seq_t;
DUMB_STATIC_ASSERT(sizeof(act_seq_t) == 20);

// ################################### Public Variables ############################################

extern u8_t	NumActuator;

// ################################ GLOBAL Functions Prototypes ####################################

void vTaskActuatorInit(void);

/**
 * @brief	Optional completion hook: called from the actuate task when a channel's repeat count
 *			reaches zero, before any queued sequence starts or the channel is stopped. NULL = none.
 * @note	shActMux is NOT held, but the channel is still Busy: calling vActuatorLoad/Update/
 *			Adjust/Toggle/GetRemainingTime on THAT channel from the callback spins forever.
 */
typedef void (*act_done_cb_t)(u8_t eCh);
void vActuatorSetDoneHook(act_done_cb_t pfDone);

#if (HAL_XFO > 0)
// Stage/done alert hook (jig timing): xActuatorAlert calls it on the actuate task with the channel,
// alert type (alertTYPE_ACT_*) and the just-completed stage. Return immediately - no blocking.
typedef void (*act_alert_cb_t)(u8_t ch, u8_t type, u8_t stage);
void vActuatorSetAlertHook(act_alert_cb_t pfAlert);
#endif

/**
 * @brief	Handlers for actTYPE_FUN channels. One set serves all of them, told apart by ioNum;
 *			the module owns any private state (act_info_t size is asserted).
 * @note	SetLevel() runs on the actuator task every actuateTASK_PERIOD with the channel Busy.
 *			MUST return immediately - no I2C, no blocking, no logging - or it stalls every actuator.
 * @note	Level 1 = entering actSTAGE_ON, 0 = actSTAGE_OFF. Start/Stop also drive 0, so treat 0
 *			as "clear", not an event.
 */
typedef struct {
	int  (*Config)(u8_t ioNum);						// optional: erSUCCESS accepts the channel
	void (*SetLevel)(u8_t ioNum, u8_t Level);		// required
	int  (*GetLevel)(u8_t ioNum);					// optional: reporting only
} act_fun_ops_t;

/**
 * @brief	Register the handlers for actTYPE_FUN channels.
 * @note	MUST precede vTaskActuatorInit(): vActuatorConfig() runs inside the task and rejects a
 *			FUN channel with no handler, leaving ConfigOK clear so the task skips it.
 * @return	erSUCCESS, or erFAILURE once the actuator task has started.
 */
int xActuatorRegisterFUN(const act_fun_ops_t * psOps);

/**
 * @brief
 * @param[in]
 * @return
 */
u8_t xActuatorGetBus(u8_t eCh);

/**
 * @brief
 * @param[in]
 * @return
 */
u8_t xActuatorGetType(u8_t eCh);

/**
 * @brief		Return the number of actuators of the specific type present in platform
 * @param[in]	ioType being ANAlog, DIGital or PWM
 * @return		number of actuators available
 */
u8_t xActuatorGetNumber(u8_t ioType);

void vActuatorLoad(u8_t eCh, u32_t Rpt, u32_t tFI, u32_t tON, u32_t tFO, u32_t tOFF);
void vActuatorUpdate(u8_t eCh, int Rpt, int tFI, int tON, int tFO, int tOFF);
void vActuatorAdjust(u8_t eCh, int Stage, int Adjust);

void vActuatorToggle(u8_t Act);
void vActuatorBreath(u8_t eCh);
void vActuatorPanic(u8_t eCh);
void vActuatorOn(u8_t eCh);
void vActuatorOff(u8_t eCh);

void xActuatorSetAlertStage(u8_t eCh, int State);
void xActuatorSetAlertDone(u8_t eCh, int State);
void xActuatorSetStartStage(u8_t eCh, int Stage);

// ############################ Actuator alerting support functions ################################

void vActuatorSetMinMaxDC(u8_t eCh, int iMin, int iMax);
void vActuatorBlock(u8_t eCh);
void vActuatorUnBlock(u8_t eCh);

void xActuatorLoadSequences(u8_t eCh, u8_t * paSeq);
void vActuatorQueSequences(u8_t eCh, u8_t * paSeq);
void vActuatorStartSequence(u8_t eCh, int Seq);

// ############################## Rules interface to Actuator table ################################

u64_t xActuatorGetRemainingTime(u8_t eCh);
u64_t xActuatorGetMaxRemainingTime (void);

/**
 * @brief	check all actuators, if repeat count = unlimited/forever, set it to 1
 */
void vActuatorsWinddown(void);

/**
 * @brief	get the number of actuators currently running/active
 * @return	number of running actuators
 */
int xActuatorRunningCount(void);

struct v64_t;
double dActuatorGetFieldValue(u8_t eCh, u8_t Field, struct v64_t * px64Var);
int	xActuatorSetFieldValue(u8_t eCh, u8_t Field, struct v64_t * px64Var);
int	xActuatorUpdateFieldValue(u8_t eCh, u8_t Field, struct v64_t * px64Var);

// ######################################## status reporting #######################################

struct report_t;
int xActuatorReportChan(struct report_t * psR, u8_t eCh);
int xTaskActuatorReport(struct report_t * psR);

// ##################################### functional tests ##########################################

struct rule_t;
int xActuatorsConfigMode(struct rule_t * psR, int Xcur, int Xmax);
int xActuatorTest(void);

#ifdef __cplusplus
}
#endif
