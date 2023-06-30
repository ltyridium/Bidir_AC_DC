#ifndef BOARDHAL_H_
#define BOARDHAL_H_

#include "main.h"
#include "stdint.h"
#include "gpio.h"
#include "tim.h"

#include "../utilities/printf.h"
#include "../APP/PFC_user_setting.h"


#define PFC_PHASE_A_TIMER LL_HRTIM_TIMER_F
#define PFC_PHASE_B_TIMER LL_HRTIM_TIMER_E
#define PFC_PHASE_C_TIMER LL_HRTIM_TIMER_A
#define PFC_PHASE_N_TIMER LL_HRTIM_TIMER_B

#define TRACE_IN    LL_GPIO_SetOutputPin(TRACE_GPIO_Port,TRACE_Pin)
#define TRACE_OUT   LL_GPIO_ResetOutputPin(TRACE_GPIO_Port,TRACE_Pin)
#define TRACE_TOGGLE LL_GPIO_TogglePin(TRACE_GPIO_Port,TRACE_Pin)


typedef union {
    enum {
        systemState_idle = 0,
        systemState_waitForACVoltage = 1,
        systemState_state1 = 2,     //close phase relay and wait for bus cap charge
        systemState_state2 = 3,     //close inrush relay
        systemState_normalOperation = 4,  //start up bus voltage with PFC
        systemState_Fault = 6,
        systemState_CalOffset
    } enum_systemState;
//    uint32_t pad;
} PFC_systemState_t;

typedef struct {
    uint8_t busOverVoltage;
    uint8_t busOverCurrent;
    uint8_t gateFault;
    uint8_t phaseOverCurrent;
    uint8_t globalFault;
    uint8_t girdUnderVoltageFault;
} PFC_struct_boardFaultFlags_t;


extern __IO PFC_systemState_t PFC_systemState;
extern __IO PFC_struct_boardFaultFlags_t PFC_boardFaultFlags;

extern __IO uint32_t uwTick;

extern void PFC_HAL_setupDevice(void);

extern void PFC_HAL_setupGPIO(void);

extern void PFC_HAL_setupADC(void);

extern void PFC_HAL_setupDAC(void);

extern void PFC_HAL_setupFAN(void);

extern void PFC_HAL_setupLED(void);

extern void PFC_HAL_setupUART(void);

extern void PFC_HAL_setupHRTIM(void);

extern void PFC_HAL_setupProtect(void);

extern void PFC_HAL_PWMTest(void);

extern void PFC_HAL_setupOLED(void);

extern void PFC_HAL_FaultCallback(void);

extern void PFC_HAL_LEDTask(void);

extern void PFC_HAL_debugWrite(uint8_t *data, uint32_t len);

extern float PFC_HAL_getTemperature(void);

extern void PFC_HAL_calADCOffset(void);





static inline void PFC_HAL_debugWriteDoneCallback()
{
  extern __IO uint32_t uart_busy;
  uart_busy = RESET;
}

static inline void PFC_HAL_Delay(uint32_t delay)
{
  LL_mDelay(delay);
}

static inline void PFC_HAL_SetFanSpeed(float speed)
{
  LL_TIM_OC_SetCompareCH2(TIM5, (uint16_t) (speed * FANTIMERARR));
}

static inline void PFC_HAL_EnableGatePWM()
{
  LL_GPIO_ResetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
}

static inline void PFC_HAL_DisableGatePWM()
{
  LL_GPIO_SetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
}


#error //Please confirm the actual circuit to select the correct PWM polarity
//use ((1 - duty) * period)) or (duty * period) depends on the actual circuit connection
static inline void PFC_HAL_SetDuty_A(float duty)
{
  uint32_t period;
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, PFC_PHASE_A_TIMER);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, PFC_PHASE_A_TIMER, (uint32_t) ((1 - duty) * period));
}

static inline void PFC_HAL_SetDuty_B(float duty)
{
  uint32_t period;
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, PFC_PHASE_B_TIMER);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, PFC_PHASE_B_TIMER, (uint32_t) ((1 - duty) * period));
}

static inline void PFC_HAL_SetDuty_C(float duty)
{
  uint32_t period;
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, PFC_PHASE_C_TIMER);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, PFC_PHASE_C_TIMER, (uint32_t) ((1 - duty) * period));
}

static inline void PFC_HAL_SetDuty_N(float duty)
{
  uint32_t period;
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, PFC_PHASE_N_TIMER);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, PFC_PHASE_N_TIMER, (uint32_t) ((1 - duty) * period));
}

static inline void PFC_HAL_updatePWMDuty(float dutyA,
                                         float dutyB,
                                         float dutyC)
{
  PFC_HAL_SetDuty_A((dutyA + 1) / 2);
  PFC_HAL_SetDuty_B((dutyB + 1) / 2);
  PFC_HAL_SetDuty_C((dutyC + 1) / 2);
//  PFC_HAL_SetDuty_N(0.5F);
};

static inline void PFC_HAL_updatePWMDeadBand(uint32_t deadTime)
{
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, deadTime);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, deadTime);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_B, deadTime);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_B, deadTime);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_E, deadTime);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_E, deadTime);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_F, deadTime);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_F, deadTime);
}

static inline void PFC_HAL_closePhaseRelay(void)
{
//  LL_GPIO_SetOutputPin(RLY_A_GPIO_Port,RLY_A_Pin);
//  LL_GPIO_SetOutputPin(RLY_B_GPIO_Port,RLY_B_Pin);
//  LL_GPIO_SetOutputPin(RLY_C_GPIO_Port,RLY_C_Pin);
  LL_GPIO_SetOutputPin(RLY_A_GPIO_Port, RLY_A_Pin | RLY_B_Pin | RLY_C_Pin);
}

static inline void PFC_HAL_openPhaseRelay(void)
{
//  LL_GPIO_ResetOutputPin(RLY_A_GPIO_Port,RLY_A_Pin);
//  LL_GPIO_ResetOutputPin(RLY_B_GPIO_Port,RLY_B_Pin);
//  LL_GPIO_ResetOutputPin(RLY_C_GPIO_Port,RLY_C_Pin);
  LL_GPIO_ResetOutputPin(RLY_A_GPIO_Port, RLY_A_Pin | RLY_B_Pin | RLY_C_Pin);
}

static inline void PFC_HAL_closeInrushRelay(void)
{
  LL_GPIO_SetOutputPin(INRUSH_RELAY_GPIO_Port, INRUSH_RELAY_Pin);
}

static inline void PFC_HAL_openInrushRelay(void)
{
  LL_GPIO_ResetOutputPin(INRUSH_RELAY_GPIO_Port, INRUSH_RELAY_Pin);
}


void PFC_HAL_while(void);






/**
 *
 * FAULT
 *
 */



//  OC_DC --> FAULT 4
//
//  OV_DC --> FAULT 3
//
//  GATE_F --> FAULT 2
//
//  OC_AC-->  FAULT 1

//#define PFC_FLT_SRC_OC_AC   LL_HRTIM_FAULT_1
//#define PFC_FLT_SRC_GATE_F  LL_HRTIM_FAULT_2
//#define PFC_FLT_SRC_OV_DC   LL_HRTIM_FAULT_3
//#define PFC_FLT_SRC_OC_DC   LL_HRTIM_FAULT_4


static inline void PFC_HAL_faultEnable_OCAC()
{
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_1);
}

static inline void PFC_HAL_faultEnable_GATE_F()
{
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_2);
}

static inline void PFC_HAL_faultEnable_OVDC()
{
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_3);
}

static inline void PFC_HAL_faultEnable_OCDC()
{
  LL_HRTIM_FLT_Enable(HRTIM1, LL_HRTIM_FAULT_4);
}

static inline void PFC_HAL_faultDisable_OCAC()
{
  LL_HRTIM_FLT_Disable(HRTIM1, LL_HRTIM_FAULT_1);
}

static inline void PFC_HAL_faultDisable_GATE_F()
{
  LL_HRTIM_FLT_Disable(HRTIM1, LL_HRTIM_FAULT_2);
}

static inline void PFC_HAL_faultDisable_OVDC()
{
  LL_HRTIM_FLT_Disable(HRTIM1, LL_HRTIM_FAULT_3);
}

static inline void PFC_HAL_faultDisable_OCDC()
{
  LL_HRTIM_FLT_Disable(HRTIM1, LL_HRTIM_FAULT_4);
}



static inline void PFC_HAL_faultEnableIT_OCAC()
{
  LL_HRTIM_EnableIT_FLT1(HRTIM1);
}

static inline void PFC_HAL_faultEnableIT_GATE_F()
{
  LL_HRTIM_EnableIT_FLT2(HRTIM1);
}

static inline void PFC_HAL_faultEnableIT_OVDC()
{
  LL_HRTIM_EnableIT_FLT3(HRTIM1);
}

static inline void PFC_HAL_faultEnableIT_OCDC()
{
  LL_HRTIM_EnableIT_FLT4(HRTIM1);
}


static inline void PFC_HAL_faultDisableIT_OCAC()
{
  LL_HRTIM_DisableIT_FLT1(HRTIM1);
}

static inline void PFC_HAL_faultDisableIT_GATE_F()
{
  LL_HRTIM_DisableIT_FLT2(HRTIM1);
}

static inline void PFC_HAL_faultDisableIT_OVDC()
{
  LL_HRTIM_DisableIT_FLT3(HRTIM1);
}

static inline void PFC_HAL_faultDisableIT_OCDC()
{
  LL_HRTIM_DisableIT_FLT4(HRTIM1);
}


static inline uint32_t PFC_HAL_faultGet_OCAC()
{
  return LL_HRTIM_IsActiveFlag_FLT1(HRTIM1);
}

static inline uint32_t PFC_HAL_faultGet_GATE_F()
{
  return LL_HRTIM_IsActiveFlag_FLT2(HRTIM1);
}

static inline uint32_t PFC_HAL_faultGet_OVDC()
{
  return LL_HRTIM_IsActiveFlag_FLT3(HRTIM1);
}

static inline uint32_t PFC_HAL_faultGet_OCDC()
{
  return LL_HRTIM_IsActiveFlag_FLT4(HRTIM1);
}

static inline void PFC_HAL_LEDGreenON()
{
  LL_GPIO_ResetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}

static inline void PFC_HAL_LEDGreenOFF()
{
  LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
}

static inline void PFC_HAL_LEDBlueON()
{
  LL_GPIO_ResetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
}

static inline void PFC_HAL_LEDBlueOFF()
{
  LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
}


static inline void PFC_HAL_IncTick()
{
  uwTick += 1;
}

static inline uint32_t PFC_HAL_GetTick()
{
  return uwTick;
}


#endif // BOARDHAL_H_