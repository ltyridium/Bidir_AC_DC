#include "../BoardHAL/BoardHAL.h"
#include "../../Drivers/OLED/oled.h"
#include "../APP/PFC.h"
#include "../BoardHAL/State_machine.h"

__IO PFC_systemState_t
        PFC_systemState;
__IO PFC_struct_boardFaultFlags_t
        PFC_boardFaultFlags;

__IO uint32_t
        uwTick;

//for adc regular data dma
__IO uint32_t
        iBusADCData;
__IO uint32_t
        vBusADCData[2];


void PFC_HAL_setupOLED();

void PFC_HAL_setupDevice(void)
{
  PFC_HAL_setupGPIO();
  PFC_HAL_setupLED();
  PFC_HAL_setupFAN();
  PFC_HAL_setupUART();
  PFC_HAL_setupADC();
  PFC_HAL_setupDAC();


  PFC_HAL_Delay(100);

#ifdef OLED_ENABLE
  PFC_HAL_setupOLED();
#endif

  PFC_systemState.enum_systemState = systemState_CalOffset;

  PFC_boardFaultFlags.busOverCurrent = RESET;
  PFC_boardFaultFlags.busOverVoltage = RESET;
  PFC_boardFaultFlags.gateFault = RESET;
  PFC_boardFaultFlags.globalFault = RESET;
  PFC_boardFaultFlags.phaseOverCurrent = RESET;
  PFC_boardFaultFlags.busOverVoltage = RESET;

  //wait for voltage stable
  PFC_HAL_Delay(1000);
  PFC_control_init();
  PFC_HAL_setupHRTIM();


  while (PFC_systemState.enum_systemState == systemState_CalOffset)
  {
    //wait for adc cal offset
  }

  //set background task
  //LED,display,uart data log
  alphaTaskPtr = PFC_HAL_while;

  aTaskPtr = PFC_fsmATask;

  bTaskPtr = PFC_fsmBTask;

  PFC_HAL_setupProtect();

  // if no use auto start ,
#ifdef COLSE_PHASE_RELAY_DEFAULT
  PFC_HAL_closePhaseRelay();
#endif //COLSE_PHASE_RELAY_DEFAULT


#ifdef CLOSE_INRUSH_RELAY_DEFAULT
  PFC_HAL_Delay(1000);
  PFC_HAL_closeInrushRelay();
#endif //CLOSE_INRUSH_RELAY_DEFAULT

#ifdef ENABLE_GATE_DEFAULT
  PFC_HAL_EnableGatePWM();
#else
  PFC_HAL_DisableGatePWM();
#endif

  //use for system tick and FSM
  LL_SYSTICK_EnableIT();

  alphaTaskPtr();
}

void PFC_HAL_calADCOffset(void)
{
  static uint32_t sampleNum = ADC_OFFSST_SAMP_NUM;
  if (sampleNum > 0)
  {

    PFC_iGrid_sensed_pu[0] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
    PFC_iGrid_sensed_pu[1] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
    PFC_iGrid_sensed_pu[2] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
#if ADC_OFFSST_VGRID == 1
    PFC_vGrid_sensed_pu[0] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
    PFC_vGrid_sensed_pu[1] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
    PFC_vGrid_sensed_pu[2] =
            (float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3) /
            __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B);
#endif


    for (
            int i = 0; i < 3; ++i)
    {
      PFC_iGrid_sensedOffset_pu[i] += PFC_iGrid_sensed_pu[i];
    }
#if ADC_OFFSST_VGRID == 1
    for (
            int i = 0; i < 3; ++i)
    {
      PFC_vGrid_sensedOffset_pu[i] += PFC_vGrid_sensed_pu[i];
    }
#endif


    sampleNum--;
  }
  else
  {
    for (
            int i = 0; i < 3; ++i)
    {
      PFC_iGrid_sensedOffset_pu[i] = PFC_iGrid_sensedOffset_pu[i] / ((float) ADC_OFFSST_SAMP_NUM);
    }
#if ADC_OFFSST_VGRID == 1
    for (
            int i = 0; i < 3; ++i)
    {
      PFC_vGrid_sensedOffset_pu[i] = PFC_vGrid_sensedOffset_pu[i] / ((float) ADC_OFFSST_SAMP_NUM);
    }
#endif

    PFC_systemState.enum_systemState = systemState_idle;
  }
}


void PFC_HAL_setupOLED()
{
  PFC_HAL_Delay(50);
  OLED_Init();
}

void PFC_HAL_setupGPIO(void)
{


}


__IO uint32_t
        adctest;

void PFC_HAL_setupADC(void)
{
  PFC_HAL_LEDBlueON();

  //phase and bus current
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
  {

  }
  //grid and bus voltage
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0)
  {

  }
  //load current adc
  LL_ADC_StartCalibration(ADC3, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC3) != 0)
  {

  }
  //temperature and ref adc
  LL_ADC_StartCalibration(ADC4, LL_ADC_SINGLE_ENDED);
  /* Poll for ADC effectively calibrated */
  while (LL_ADC_IsCalibrationOnGoing(ADC4) != 0)
  {

  }

  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
  {
  }

  LL_ADC_Enable(ADC2);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0)
  {
  }

  LL_ADC_Enable(ADC3);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC3) == 0)
  {
  }

  LL_ADC_Enable(ADC4);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC4) == 0)
  {
  }

  //Hrtim 2 TimerB trig out ADC
  LL_ADC_INJ_StartConversion(ADC1);
  LL_ADC_INJ_StartConversion(ADC2);
  LL_ADC_INJ_StartConversion(ADC3);
  LL_ADC_INJ_StartConversion(ADC4);


  //enable phase current complete interrupt

  //Hrtim 3 TimerB trig out ADC


  //bus current adc
  LL_ADC_REG_StartConversion(ADC1);
  //bus voltage adc
  LL_ADC_REG_StartConversion(ADC2);



//  LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,2);
//  LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1,LL_ADC_DMA_REG_REGULAR_DATA));
//  LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_1,(uint32_t)&AD_DMA);
//  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);



  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
                          LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &iBusADCData);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2,
                          LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)
          vBusADCData);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);




  //enable phase current isr
  LL_ADC_EnableIT_JEOS(ADC1);
//  LL_ADC_EnableIT_JEOC(ADC3);
//  LL_ADC_EnableIT_JEOC(ADC4);
  PFC_HAL_LEDBlueOFF();
}

void PFC_HAL_setupDAC(void)
{
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 2048);
}

void PFC_HAL_setupFAN(void)
{
  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);

  PFC_HAL_SetFanSpeed(0.1f);

  LL_TIM_EnableAllOutputs(TIM5);
}

void PFC_HAL_setupHRTIM(void)
{

  //1 us
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, 170);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, 170);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_B, 170);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_B, 170);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_E, 170);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_E, 170);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_F, 170);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_F, 170);

  uint32_t period;
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, period / 2);
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, period / 2);
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_E);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_E, period / 2);
  period = LL_HRTIM_TIM_GetPeriod(HRTIM1, LL_HRTIM_TIMER_F);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_F, period / 2);

  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1
                                | LL_HRTIM_OUTPUT_TA2
                                | LL_HRTIM_OUTPUT_TB1
                                | LL_HRTIM_OUTPUT_TB2
                                | LL_HRTIM_OUTPUT_TE1
                                | LL_HRTIM_OUTPUT_TE2
                                | LL_HRTIM_OUTPUT_TF1
                                | LL_HRTIM_OUTPUT_TF2);


  //adc trig 0xE700

  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_B, 0x9200);

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_E | LL_HRTIM_TIMER_F);
}


void PFC_HAL_setupLED(void)
{
  PFC_HAL_LEDBlueOFF();
  PFC_HAL_LEDGreenOFF();

  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  LL_TIM_SetPrescaler(TIM1, 17000 - 1);
  LL_TIM_SetAutoReload(TIM1, 2999);
  LL_TIM_OC_SetCompareCH4(TIM1, 2990);
}


void PFC_HAL_setupUART(void)
{

  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_5,
                          LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_USART_EnableDMAReq_TX(UART4);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_5);

  PFC_HAL_debugWrite((uint8_t *) "PFC\r\n", 5);
}


__IO uint32_t
        uart_busy = RESET;

static uint32_t PFC_HAL_isDebugUARTBusy()
{
  return uart_busy == SET;
}

void PFC_HAL_debugWrite(uint8_t *data, uint32_t len)
{
  while (PFC_HAL_isDebugUARTBusy())
  {

  }
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_5, (uint32_t)
          data);
  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, len);
  uart_busy = SET;
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
}


void PFC_HAL_PWMTest()
{
  PFC_HAL_SetDuty_A(0.4F);
  PFC_HAL_SetDuty_B(0.5F);
  PFC_HAL_SetDuty_C(0.6F);
  PFC_HAL_SetDuty_N(0.5F);

  PFC_HAL_EnableGatePWM();
}


void PFC_HAL_setupProtect(void)
{
  LL_HRTIM_FLT_DisableBlanking(HRTIM1, LL_HRTIM_FAULT_1);
  LL_HRTIM_FLT_DisableBlanking(HRTIM1, LL_HRTIM_FAULT_2);
  LL_HRTIM_FLT_DisableBlanking(HRTIM1, LL_HRTIM_FAULT_3);
  LL_HRTIM_FLT_DisableBlanking(HRTIM1, LL_HRTIM_FAULT_4);

  //  OC_AC   -->   FAULT 1
  //  GATE_F  -->   FAULT 2
  //  OV_DC   -->   FAULT 3
  //  OC_DC   -->   FAULT 4

#ifdef BOARD_PROTECTION_PHASEOC
  PFC_HAL_faultEnable_OCAC();
  PFC_HAL_faultEnableIT_OCAC();
#endif

#ifdef BOARD_PROTECTION_GATEFAULT
  PFC_HAL_faultEnable_GATE_F();
  PFC_HAL_faultEnableIT_GATE_F();
#endif

#ifdef BOARD_PROTECTION_BUSOV
  PFC_HAL_faultEnable_OVDC();
  PFC_HAL_faultEnableIT_OVDC();
#endif

#ifdef BOARD_PROTECTION_BUSOC
  PFC_HAL_faultEnable_OCDC();
  PFC_HAL_faultEnableIT_OCDC();
#endif

}

void PFC_HAL_FaultCallback()
{

//	LL_HRTIM_ClearFlag_FLT1(HRTIM1);
//	LL_HRTIM_ClearFlag_FLT2(HRTIM1);
//	LL_HRTIM_ClearFlag_FLT3(HRTIM1);
//	LL_HRTIM_ClearFlag_FLT4(HRTIM1);
  PFC_systemState.enum_systemState = systemState_Fault;
  if (PFC_HAL_faultGet_OCAC() == SET)
  {
    //
    PFC_boardFaultFlags.phaseOverCurrent = SET;
    PFC_HAL_faultDisableIT_OCAC();
  }
  if (PFC_HAL_faultGet_GATE_F() == SET)
  {
    //
    PFC_boardFaultFlags.gateFault = SET;
    PFC_HAL_faultDisableIT_GATE_F();
  }
  if (PFC_HAL_faultGet_OCDC() == SET)
  {
    //
    PFC_boardFaultFlags.busOverCurrent = SET;
    PFC_HAL_faultDisableIT_OCDC();
  }
  if (PFC_HAL_faultGet_OVDC() == SET)
  {
    //
    PFC_boardFaultFlags.busOverVoltage = SET;
    PFC_HAL_faultDisableIT_OVDC();
  }
  PFC_HAL_openPhaseRelay();
  PFC_HAL_openInrushRelay();
}


uint32_t LED_State;
enum
{
    F1,
    F2,
    F3,
    F4,
    no_fault
};

enum
{
    led_color_yellow,
    led_color_red,
    led_color_green,
    led_off
};


void PFC_HAL_LEDSet(uint32_t color)
{
  LL_TIM_SetPrescaler(TIM1, 17 - 1);
  LL_TIM_SetAutoReload(TIM1, 999);
  switch (color)
  {
    case led_color_yellow:
    {
      LL_TIM_OC_SetCompareCH4(TIM1, 200);   //pin = 0.2 YELLOW
      LL_TIM_EnableAllOutputs(TIM1);
      break;
    }
    case led_color_red:
    {
      LL_TIM_OC_SetCompareCH4(TIM1, 0);     //pin = 0 RED
      LL_TIM_EnableAllOutputs(TIM1);
      break;
    }
    case led_color_green:
    {
      LL_TIM_OC_SetCompareCH4(TIM1, LL_TIM_GetAutoReload(TIM1)); //GREEN
      LL_TIM_EnableAllOutputs(TIM1);
      break;
    }
    case led_off:
    {
      LL_TIM_DisableAllOutputs(TIM1);
      break;
    }
    default:
    {
      break;
    }
  }
}


void PFC_HAL_LEDTask()
{
  if (PFC_systemState.enum_systemState == systemState_Fault)
  {
    //fault occurred
    if (PFC_boardFaultFlags.phaseOverCurrent == SET)
    {
      LED_State = F1;
    }
    else if (PFC_boardFaultFlags.gateFault == SET)
    {
      LED_State = F2;
    }
    else if (PFC_boardFaultFlags.busOverVoltage == SET)
    {
      LED_State = F3;
    }
    else if (PFC_boardFaultFlags.busOverCurrent == SET)
    {
      LED_State = F4;
    }
  }
  else
  {
    //no fault
    LED_State = no_fault;
    if ((PFC_HAL_GetTick() % 1000) > 500)
    {
      PFC_HAL_LEDBlueON();
    }
    else
    {
      PFC_HAL_LEDBlueOFF();
    }
  }

  //todo 重构屎山代码
  uint32_t cnt = PFC_HAL_GetTick() % 4500;
  switch (LED_State)
  {
    case F1:
    {
      if (cnt < 500)
      {
        // led_color_yellow 500ms
        PFC_HAL_LEDSet(led_color_yellow);
      }
      else
      {
        if (cnt < 1000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else
        {
          PFC_HAL_LEDSet(led_off);
        }
      }
      break;
    }
    case F2:
    {
      if (cnt < 500)
      {
        // led_color_yellow 500ms
        PFC_HAL_LEDSet(led_color_yellow);
      }
      else
      {
        if (cnt < 1000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 1500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 2000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else
        {
          PFC_HAL_LEDSet(led_off);
        }
      }
      break;
    }
    case F3:
    {
      if (cnt < 500)
      {
        // led_color_yellow 500ms
        PFC_HAL_LEDSet(led_color_yellow);
      }
      else
      {
        if (cnt < 1000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 1500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 2000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 2500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 3000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else
        {
          PFC_HAL_LEDSet(led_off);
        }
      }
      break;
    }
    case F4:
    {
      if (cnt < 500)
      {
        // led_color_yellow 500ms
        PFC_HAL_LEDSet(led_color_yellow);
      }
      else
      {
        if (cnt < 1000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 1500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 2000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 2500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 3000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else if (cnt < 3500)
        {
          PFC_HAL_LEDSet(led_off);
        }
        else if (cnt < 4000)
        {
          PFC_HAL_LEDSet(led_color_red);
        }
        else
        {
          PFC_HAL_LEDSet(led_off);
        }
      }
      break;
    }
    default:
    {
      PFC_HAL_LEDSet(led_off);
      break;
    }
  }
}


#include "math.h"

//NTC
float const NTC_Vref = 2.9f;     // [V]
float const NTC_Rt = 10000;    // Resistor t [ohm]
float const NTC_R0 = 13000;    // value of rct in T0 [ohm]
float const NTC_T0 = 298.15;   // use T0 in Kelvin [K]
float const NTC_beta = 3435.0f;

float PFC_HAL_getTemperature(void)
{
  float voltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(NTC_Vref, LL_ADC_INJ_ReadConversionData12(ADC4, LL_ADC_INJ_RANK_1),
                                                LL_ADC_RESOLUTION_12B);
  float R_NTC = NTC_Rt * voltage / (NTC_Vref - voltage);

  float Rinf = NTC_R0 * expf(-NTC_beta / NTC_T0);

  float TempK = (NTC_beta / logf(R_NTC / Rinf)); // calc for temperature

  return TempK - 273.15f;

}


// for uart data log
__IO float data[10];
__IO uint8_t
        tail[4] = {
        0x00, 0x00, 0x80, 0x7f};

void PFC_HAL_while(void)
{
  while (1)
  {
    PFC_HAL_LEDTask();

    //fan speed control
    float temp = PFC_HAL_getTemperature();
    if (temp > FAN_START_THRESHODE)
    {
      float duty = (temp - FAN_START_THRESHODE) / 100.0f;
      PFC_HAL_SetFanSpeed(0.08f + duty);
    }
    else if (temp < FAN_START_THRESHODE - 5)
    {
      PFC_HAL_SetFanSpeed(0);
    }
    LL_ADC_INJ_StartConversion(ADC4);
#ifdef OLED_ENABLE
    OLED_ShowNum(0, 0, (uint32_t) temp, 3, 16);
#endif //OLED_ENABLE

#ifdef UART_DEBUG_ENABLE

//    data[0] = PFC_vGrid_sensed_pu[0];
//    data[1] = PFC_iGrid_sensed_pu[0];
//    data[2] = PFC_iInv_dq0_pos.d;
//    data[3] = PFC_idRef_pu;

    data[0] = PFC_idRef_pu;
    data[1] = PFC_iInv_dq0_pos.d;
		data[2] = PFC_spll_3ph.fo;
////    data[3] = PFC_iInv_dq0_pos.d;

//    data[0] = PFC_vGrid_sensed_pu[0];
//    data[1] = PFC_vGrid_sensed_pu[1];
//    data[2] = PFC_vGrid_sensed_pu[2];
//    data[3] = PFC_iInv_dq0_pos.q;
//    data[4] = PFC_vBus_sensed_pu;



//    data[3] = PFC_vGrid_sensedOffset_pu[0];
//
//    data[4] = PFC_vGrid_sensedOffset_pu[1];
//
//    data[5] = PFC_vGrid_sensedOffset_pu[2];

//
//    data[3] = __LL_ADC_CALC_DATA_TO_VOLTAGE(2900, LL_ADC_REG_ReadConversionData12(ADC1) >> 4,
//                                            LL_ADC_RESOLUTION_12B);

//    data[7] = PFC_spll_3ph.fo;

    PFC_HAL_debugWrite((uint8_t *) data, 4 * 3);
    PFC_HAL_debugWrite((uint8_t *) tail, 4);

#endif //UART_DEBUG_ENABLE

  }
}



