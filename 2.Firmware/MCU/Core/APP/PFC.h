#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus

extern "C"
{
#endif

#include "../lib/dq0_abc.h"
#include "../lib/abc_dq0_pos.h"
#include "../lib/abc_dq0_neg.h"
#include "../lib/spll_3ph_srf.h"
#include "../lib/spll_3ph_ddsrf.h"
#include "../lib/DCL.h"
#include "PFC_parameter.h"
#include "../BoardHAL/BoardHAL.h"

// Multiplier = （1 - 2*pi*Fcuttoff/Fsampling）
#define CAL_M(Fcuttoff, Fsampling) (1 - (2 * 3.1415926f * Fcuttoff / Fsampling))
#define PFC_EMAVG_MACRO(in, out, multiplier) out = ((out - in) * multiplier) + in;



extern ABC_DQ0_POS PFC_vGrid_dq0_pos;
extern ABC_DQ0_NEG PFC_vGrid_dq0_neg;

extern ABC_DQ0_POS PFC_internalRef_dq0_pos;
extern ABC_DQ0_NEG PFC_internalRef_dq0_neg;

extern ABC_DQ0_POS PFC_iInv_dq0_pos;

extern DQ0_ABC PFC_vInv_dq0;

extern DCL_PI PFC_gi_id;
extern DCL_PI PFC_gi_iq;
extern DCL_PI PFC_gv_vBus;

extern float32_t PFC_vdInv_pu;
extern float32_t PFC_vqInv_pu;
extern float32_t PFC_vzInv_pu;
extern float32_t PFC_duty_A_pu;
extern float32_t PFC_duty_B_pu;
extern float32_t PFC_duty_C_pu;

extern volatile float32_t PFC_vdInvRef_pu;
extern volatile float32_t PFC_vqInvRef_pu;
extern volatile float32_t PFC_idRef_pu;
extern volatile float32_t PFC_iqRef_pu;
extern volatile float32_t PFC_vBusRef_pu;
extern volatile float32_t PFC_vBusRefSlewed_pu;

//internal reference
extern volatile float32_t PFC_angle_radians;
extern volatile float32_t PFC_angleFixed_radians;
extern float32_t PFC_angleSPLL_radians;
extern float32_t PFC_sine_A;
extern float32_t PFC_sine_B;
extern float32_t PFC_sine_C;
extern float32_t PFC_cosine_A;
extern float32_t PFC_cosine_B;
extern float32_t PFC_cosine_C;
extern float32_t PFC_sine;
extern float32_t PFC_cosine;

extern float32_t PFC_gi_id_out;
extern float32_t PFC_gi_iq_out;
extern float32_t PFC_gv_vBus_out;
extern SPLL_3PH_DDSRF PFC_spll_3ph_1;
extern SPLL_3PH_SRF PFC_spll_3ph;



// input

//for adc regular data dma
extern __IO uint32_t
        iBusADCData;
extern __IO uint32_t
        vBusADCData[2];

extern float32_t PFC_vGrid_A_sensed_prev_pu;
extern volatile float32_t PFC_vGrid_sensed_pu[3];
extern volatile float32_t PFC_vGrid_sensedOffset_pu[3];
extern volatile float32_t PFC_iGrid_sensed_pu[3];
extern volatile float32_t PFC_iGrid_sensedOffset_pu[3];



extern volatile float32_t PFC_vBus_sensed_pu;
extern volatile float32_t PFC_vBusUp_sensed_pu;
extern volatile float32_t PFC_vBusDown_sensed_pu;
extern volatile float32_t PFC_vBusUp_sensedOffset_pu;
extern volatile float32_t PFC_vBusDown_sensedOffset_pu;

extern volatile float32_t PFC_iBus_sensed_pu;
extern volatile float32_t PFC_iBus_sensedOffset_pu;

extern float32_t PFC_vBus_sensed_Filtered_pu;
extern float32_t PFC_vBus_sensed_FilteredAndClamped_pu;

// SI unit
extern float32_t PFC_vBus_sensed_Volts;
extern float32_t PFC_vGridRms_A_sensed_Volts;
extern float32_t PFC_vGridRms_B_sensed_Volts;
extern float32_t PFC_vGridRms_C_sensed_Volts;

// state
extern volatile uint32_t PFC_closeGiLoop;
extern volatile uint32_t PFC_closeGvLoop;
extern volatile uint32_t PFC_enablePWM;
extern volatile uint32_t PFC_startPowerStage;
extern volatile uint32_t PFC_rlyConnect;
extern volatile uint32_t PFC_firstTimeGvLoop;
extern volatile uint32_t PFC_deadBandSlewUpdate;
extern volatile uint32_t PFC_deadBand;
extern volatile uint32_t PFC_enableHarmComp;

void PFC_control_init(void);

void PFC_autoStartPFC(void);


static inline void PFC_readCurrentAndVoltageSignals(void)
{
  PFC_iGrid_sensed_pu[0] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_iGrid_sensedOffset_pu[0]) * 2.0f;
  PFC_iGrid_sensed_pu[1] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_iGrid_sensedOffset_pu[1]) * 2.0f;
  PFC_iGrid_sensed_pu[2] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_iGrid_sensedOffset_pu[2]) * 2.0f;
					
  PFC_vGrid_A_sensed_prev_pu = PFC_vGrid_sensed_pu[0];

  PFC_vGrid_sensed_pu[0] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_vGrid_sensedOffset_pu[0]) * 2.0f;
  PFC_vGrid_sensed_pu[1] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_vGrid_sensedOffset_pu[1]) * 2.0f;
  PFC_vGrid_sensed_pu[2] =
          ((float) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3) /
           __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) - PFC_vGrid_sensedOffset_pu[2]) * 2.0f;

  PFC_vBusUp_sensed_pu = ((float32_t) vBusADCData[0] *
                          PFC_ADC_PU_SCALE_FACTOR) - PFC_vBusUp_sensedOffset_pu;
  PFC_vBusDown_sensed_pu = ((float32_t) vBusADCData[1] *
                            PFC_ADC_PU_SCALE_FACTOR) - PFC_vBusDown_sensedOffset_pu;
  PFC_vBus_sensed_pu = PFC_vBusUp_sensed_pu + PFC_vBusDown_sensed_pu;

  PFC_iBus_sensed_pu = (((float32_t) iBusADCData *
                         PFC_ADC_PU_SCALE_FACTOR) - PFC_iBus_sensedOffset_pu) * 2.0f;;
}

static inline void PFC_filterAndCheckForBusOverVoltage(void)
{
  PFC_EMAVG_MACRO(PFC_vBus_sensed_pu, PFC_vBus_sensed_Filtered_pu, 0.9875);

  PFC_vBus_sensed_Volts = PFC_vBus_sensed_Filtered_pu *
                          PFC_VBUS_MAX_SENSE_VOLTS;

  PFC_vBus_sensed_FilteredAndClamped_pu =
          (PFC_vBus_sensed_Filtered_pu < PFC_VBUS_CLAMP_MIN_PU) ?
          PFC_VBUS_CLAMP_MIN_PU : PFC_vBus_sensed_Filtered_pu;

  if (PFC_vBus_sensed_Volts > PFC_VBUS_OVERVOLT_LIMIT)
  {
    PFC_HAL_DisableGatePWM();
    PFC_systemState.enum_systemState = systemState_Fault;
    PFC_boardFaultFlags.busOverVoltage = SET;
  }
}


static inline void PFC_runTransformOnSensedSignals()
{
  //
  // Run ABC DQ0 on the inverter current
  // with the locked PLL angle
  //
  ABC_DQ0_POS_run(&PFC_iInv_dq0_pos,
                  PFC_iGrid_sensed_pu[0],
                  PFC_iGrid_sensed_pu[1],
                  PFC_iGrid_sensed_pu[2],
                  PFC_sine, PFC_cosine);

  //
  // Run ABC DQ0 POS on the grid voltage
  // with the locked PLL angle
  //
  ABC_DQ0_POS_run(&PFC_vGrid_dq0_pos,
                  PFC_vGrid_sensed_pu[0],
                  PFC_vGrid_sensed_pu[1],
                  PFC_vGrid_sensed_pu[2],
                  PFC_sine, PFC_cosine);

  // //
  // // Run ABC DQ0 NEG on the grid voltage
  // // with the locked PLL angle
  // //
  ABC_DQ0_NEG_run(&PFC_vGrid_dq0_neg,
                  PFC_vGrid_sensed_pu[0],
                  PFC_vGrid_sensed_pu[1],
                  PFC_vGrid_sensed_pu[2],
                  PFC_sine, PFC_cosine);
}


//Generate an internal reference for non grid connected case

static inline void PFC_generateInternalReference(void)
{

  //
  // Compute PFC_angle and sine and cosine
  //
  static float step = (1 / PFC_ISR1_FREQUENCY_HZ) * PFC_AC_FREQ_HZ;
  static float theta = 0;
  theta += step;
  if (theta > 1)
  {
    theta -= 1;
  }

  PFC_angle_radians = theta * PFC_PI * 2.0f;

  //
  // Use the PFC_angle value to compute the sine value
  //
  PFC_sine_A = sinf(PFC_angle_radians);
  PFC_cosine_A = cosf(PFC_angle_radians);

  PFC_sine_B = sinf(PFC_angle_radians - PFC_PI * 2.0f / 3.0f);
  PFC_cosine_B = cosf(PFC_angle_radians - PFC_PI * 2.0f / 3.0f);

  PFC_sine_C = sinf(PFC_angle_radians - PFC_PI * 4.0f / 3.0f);
  PFC_cosine_C = cosf(PFC_angle_radians - PFC_PI * 4.0f / 3.0f);

  ABC_DQ0_POS_run(&PFC_internalRef_dq0_pos,
                  PFC_sine_A, PFC_sine_B, PFC_sine_C,
                  PFC_sine, PFC_cosine);
  ABC_DQ0_NEG_run(&PFC_internalRef_dq0_neg,
                  PFC_sine_A, PFC_sine_B, PFC_sine_C,
                  PFC_sine, PFC_cosine);
}

static inline void PFC_runCurrentLoop(void)
{
  if (PFC_closeGiLoop == 1)
  {
    PFC_gi_id_out = PFC_GI_RUN(&PFC_gi_id,
                               PFC_idRef_pu,
                               PFC_iInv_dq0_pos.d);
    PFC_gi_iq_out = PFC_GI_RUN(&PFC_gi_iq, PFC_iqRef_pu,
                               PFC_iInv_dq0_pos.q);
    //
    // Feed forward & decoupling for the control loop
    //

    PFC_vdInv_pu = (PFC_gi_id_out +
                    (PFC_vGrid_dq0_pos.d *
                     (PFC_VGRID_MAX_SENSE_VOLTS / PFC_VBUS_MAX_SENSE_VOLTS)) -
                    (PFC_iInv_dq0_pos.q * PFC_DECOUPLING_CONST * PFC_AC_FREQ_HZ)) /
                   (PFC_vBus_sensed_FilteredAndClamped_pu * 0.5f);

    PFC_vqInv_pu = (PFC_gi_iq_out +
                    (PFC_vGrid_dq0_pos.q *
                     (PFC_VGRID_MAX_SENSE_VOLTS / PFC_VBUS_MAX_SENSE_VOLTS)) +
                    (PFC_iInv_dq0_pos.d * PFC_DECOUPLING_CONST * PFC_AC_FREQ_HZ)) /
                   (PFC_vBus_sensed_FilteredAndClamped_pu * 0.5f);

    PFC_vdInv_pu = (PFC_vdInv_pu > 1.0f) ? 1.0f : PFC_vdInv_pu;
    PFC_vdInv_pu = (PFC_vdInv_pu < -1.0f) ? -1.0f : PFC_vdInv_pu;

    PFC_vqInv_pu = (PFC_vqInv_pu > 1.0f) ? 1.0f : PFC_vqInv_pu;
    PFC_vqInv_pu = (PFC_vqInv_pu < -1.0f) ? -1.0f : PFC_vqInv_pu;
  }
}

//static inline void PFC_runVoltageLoop(void)
//{
//  if (PFC_closeGvLoop == 1)
//  {
//    //Slew ref
//    {
//      PFC_vBusRefSlewed_pu = PFC_vBusRef_pu;
//    }
//    {

//      PFC_gv_vBus_out = PFC_GV_RUN(&PFC_gv_vBus,
//                                   PFC_vBusRefSlewed_pu,
//                                   PFC_vBus_sensed_pu);
//    }

//    //
//    // the multiply by one below is to accomodate the sign of idRef
//    // which is positive when the current is sourced i.e. inverter mode
//    // and negative when the current is sinked i.e. PFC mode
//    //
//    PFC_idRef_pu = -1.0f * PFC_gv_vBus_out;

//  }
//  else
//  {


//  }
//}

static inline void PFC_runSPLL(float32_t v_p_d, float32_t v_n_d,
                               float32_t v_p_q, float32_t v_n_q)
{
#if PFC_SPLL_TYPE == PFC_SPLL_SRF
  SPLL_3PH_SRF_run(v_p_q, &PFC_spll_3ph);

  PFC_angleSPLL_radians = PFC_spll_3ph.theta[1];
#else
  SPLL_3PH_DDSRF_run(&PFC_spll_3ph_1,
                     v_p_d, v_n_d,
                     v_p_q, v_n_q);

  PFC_angleSPLL_radians = PFC_spll_3ph_1.theta[1];
#endif

}


static inline void ctrlIsr1_Lab1(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();
  PFC_generateInternalReference();
  //
  // ABC-> DQ0
  //
  PFC_vdInv_pu = PFC_vdInvRef_pu;
  PFC_vqInv_pu = PFC_vqInvRef_pu;
  PFC_vzInv_pu = 0;
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);


  PFC_runSPLL(PFC_internalRef_dq0_pos.d,
              PFC_internalRef_dq0_neg.d,
              PFC_internalRef_dq0_pos.q,
              PFC_internalRef_dq0_neg.q);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uint32_t) ((PFC_duty_A_pu + 1.0f) / 2 * 1.0f * 4095));
  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);
  PFC_filterAndCheckForBusOverVoltage();
}

static inline void ctrlIsr1_Lab2(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();
//  PFC_generateInternalReference();
  //
  // ABC-> DQ0
  //
  PFC_vdInv_pu = PFC_vdInvRef_pu;
  PFC_vqInv_pu = PFC_vqInvRef_pu;
  PFC_vzInv_pu = 0;
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);


  PFC_runSPLL(PFC_vGrid_dq0_pos.d,
              PFC_vGrid_dq0_neg.d,
              PFC_vGrid_dq0_pos.q,
              PFC_vGrid_dq0_neg.q);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,
                                   (uint32_t) ((PFC_angleSPLL_radians / 2 / PFC_PI) * 1.0f * 4095));
  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);
  PFC_filterAndCheckForBusOverVoltage();
}

static inline void ctrlIsr1_Lab3(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();
  PFC_generateInternalReference();
  //
  // ABC-> DQ0
  //
  PFC_vdInv_pu = PFC_vdInvRef_pu;
  PFC_vqInv_pu = PFC_vqInvRef_pu;
  PFC_vzInv_pu = 0;
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);


  PFC_runSPLL(PFC_internalRef_dq0_pos.d,
              PFC_internalRef_dq0_neg.d,
              PFC_internalRef_dq0_pos.q,
              PFC_internalRef_dq0_neg.q);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,
                                   (uint32_t) ((PFC_iGrid_sensed_pu[0] + 1) * 0.5f * 4095));

  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);
  PFC_filterAndCheckForBusOverVoltage();

}

static inline void ctrlIsr1_Lab4(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();
  PFC_generateInternalReference();
  //
  // ABC-> DQ0
  //

  PFC_runCurrentLoop();

  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);


  PFC_runSPLL(PFC_internalRef_dq0_pos.d,
              PFC_internalRef_dq0_neg.d,
              PFC_internalRef_dq0_pos.q,
              PFC_internalRef_dq0_neg.q);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uint32_t) ((PFC_duty_A_pu + 1.0f) / 2 * 1.0f * 4095));
  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);
  PFC_filterAndCheckForBusOverVoltage();
}

static inline void ctrlIsr1_Lab5(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();

  //
  // ABC-> DQ0
  //
  PFC_vdInv_pu = PFC_vdInvRef_pu;
  PFC_vqInv_pu = PFC_vqInvRef_pu;
  PFC_vzInv_pu = 0;
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ? PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ? -PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);


  PFC_runSPLL(PFC_internalRef_dq0_pos.d,
              PFC_internalRef_dq0_neg.d,
              PFC_internalRef_dq0_pos.q,
              PFC_internalRef_dq0_neg.q);

  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uint32_t) ((PFC_duty_A_pu + 1.0f) / 2 * 1.0f * 4095));
  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);

  PFC_filterAndCheckForBusOverVoltage();
}

static inline void ctrlIsr1_Lab6(void)
{
  PFC_readCurrentAndVoltageSignals();
  PFC_runTransformOnSensedSignals();

  if (PFC_enablePWM == SET)
  {
    PFC_enablePWM = RESET;
    PFC_HAL_EnableGatePWM();
  }

  //
  // If command is issued to start the power stage, wait for zero crossing
  // on phase A, and then set the variables to clear PWM Trip and
  // close the current loop
  //
  if (PFC_startPowerStage == SET)
  {
    if (PFC_vGrid_sensed_pu[0] > 0.0f && PFC_vGrid_A_sensed_prev_pu < 0.0f)
    {
      PFC_startPowerStage = RESET;
      PFC_closeGiLoop = SET;
      PFC_enablePWM = SET;
    }
  }
  PFC_runCurrentLoop();

  //
  // ABC-> DQ0
  //
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  //
  // update duty values
  //
  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_C_pu;

  //
  // update PWM with new duty
  //

  if (PFC_closeGiLoop == SET)
  {

    PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);
    PFC_HAL_updatePWMDeadBand(PFC_deadBand);
    PFC_deadBandSlewUpdate++;


    if (PFC_deadBand > PFC_DEAD_TIME_MIN && PFC_deadBandSlewUpdate == 1)
    {
      PFC_deadBand = PFC_deadBand - 1;
      PFC_deadBandSlewUpdate = RESET;
    }
  }
  else
  {
    //dead time soft start
    PFC_deadBand = PFC_DEAD_TIME_MAX;
    PFC_HAL_updatePWMDuty(0.0f, 0.0f, 0.0f);
    PFC_HAL_updatePWMDeadBand(PFC_deadBand);
    PFC_deadBandSlewUpdate = RESET;
  }

  PFC_runSPLL(PFC_vGrid_dq0_pos.d,
              PFC_vGrid_dq0_neg.d,
              PFC_vGrid_dq0_pos.q,
              PFC_vGrid_dq0_neg.q);

  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);

  PFC_filterAndCheckForBusOverVoltage();
}

static inline void ctrlIsr1_Lab7(void)
{

  PFC_readCurrentAndVoltageSignals();

  PFC_runTransformOnSensedSignals();

  if (PFC_enablePWM == SET)
  {
    PFC_enablePWM = RESET;
    PFC_HAL_EnableGatePWM();
  }

  PFC_runCurrentLoop();

  //
  // ABC-> DQ0
  //
  DQ0_ABC_run(&PFC_vInv_dq0,
              PFC_vdInv_pu, PFC_vqInv_pu, PFC_vzInv_pu,
              PFC_sine, PFC_cosine);

  PFC_duty_A_pu = PFC_vInv_dq0.a;
  PFC_duty_B_pu = PFC_vInv_dq0.b;
  PFC_duty_C_pu = PFC_vInv_dq0.c;

  //
  // Clamp duty cycles to -1..1
  //
  PFC_duty_A_pu = (PFC_duty_A_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_A_pu = (PFC_duty_A_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_A_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_B_pu = (PFC_duty_B_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_B_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu > PFC_DUTY_CLAMP) ?
                  PFC_DUTY_CLAMP : PFC_duty_C_pu;
  PFC_duty_C_pu = (PFC_duty_C_pu < -PFC_DUTY_CLAMP) ?
                  -PFC_DUTY_CLAMP : PFC_duty_C_pu;

  //
  // update PWM with new duty
  //

  if (PFC_closeGiLoop == SET)
  {

    PFC_HAL_updatePWMDuty(PFC_duty_A_pu, PFC_duty_B_pu, PFC_duty_C_pu);
    PFC_HAL_updatePWMDeadBand(PFC_deadBand);
    PFC_deadBandSlewUpdate++;


    if (PFC_deadBand > PFC_DEAD_TIME_MIN && PFC_deadBandSlewUpdate == 1)
    {
      PFC_deadBand = PFC_deadBand - 1;
      PFC_deadBandSlewUpdate = 0;
    }
  }
  else
  {
    //dead time soft start
    PFC_deadBand = PFC_DEAD_TIME_MAX;
    PFC_HAL_updatePWMDuty(0.0f, 0.0f, 0.0f);
    PFC_HAL_updatePWMDeadBand(PFC_deadBand);
    PFC_deadBandSlewUpdate = 0;
  }

  PFC_runSPLL(PFC_vGrid_dq0_pos.d,
              PFC_vGrid_dq0_neg.d,
              PFC_vGrid_dq0_pos.q,
              PFC_vGrid_dq0_neg.q);

  PFC_sine = sinf(PFC_angleSPLL_radians);
  PFC_cosine = cosf(PFC_angleSPLL_radians);

  //
  // Actions to take when power stage needs to be started
  //

  if (PFC_startPowerStage == SET)
  {

    if (PFC_vGrid_sensed_pu[0] > 0.0f && PFC_vGrid_A_sensed_prev_pu < 0.0f)
    {
      PFC_startPowerStage = RESET;
      if (PFC_firstTimeGvLoop == SET)
      {
        PFC_vBusRefSlewed_pu = PFC_vBus_sensed_pu;
        PFC_firstTimeGvLoop = RESET;
      }

      PFC_closeGvLoop = SET;
      PFC_closeGiLoop = SET;
      PFC_enablePWM = SET;
    }
  }

  //
  // Voltage Loop
  //
  if (PFC_closeGvLoop == SET)
  {
    //
    // Slew vBusRef
    //
    if ((PFC_vBusRef_pu - PFC_vBusRefSlewed_pu) >
        (2.0f * PFC_VOLTS_PER_SECOND_SLEW /
         PFC_VBUS_MAX_SENSE_VOLTS) *
        (1.0f / (float32_t) PFC_ISR1_FREQUENCY_HZ))
    {
      PFC_vBusRefSlewed_pu = PFC_vBusRefSlewed_pu +
                             (PFC_VOLTS_PER_SECOND_SLEW / PFC_VBUS_MAX_SENSE_VOLTS) *
                             (1.0f / (float32_t) PFC_ISR1_FREQUENCY_HZ);
    } else if ((PFC_vBusRef_pu - PFC_vBusRefSlewed_pu) <
             -(2.0f * PFC_VOLTS_PER_SECOND_SLEW /
               PFC_VBUS_MAX_SENSE_VOLTS) *
             (1.0f / (float32_t) PFC_ISR1_FREQUENCY_HZ))
    {
      PFC_vBusRefSlewed_pu = PFC_vBusRefSlewed_pu -
                             (PFC_VOLTS_PER_SECOND_SLEW / PFC_VBUS_MAX_SENSE_VOLTS) *
                             (1.0f / (float32_t) PFC_ISR1_FREQUENCY_HZ);
    } else
    {
      PFC_vBusRefSlewed_pu = PFC_vBusRef_pu;
    }


    PFC_gv_vBus_out = PFC_GV_RUN(&PFC_gv_vBus,
                                 PFC_vBusRefSlewed_pu,
                                 PFC_vBus_sensed_pu);


    //
    // the multiply by one below is to accomodate the sign of idRef
    // which is positive when the current is sourced i.e. inverter mode
    // and negative when the current is sinked i.e. PFC mode
    //
    PFC_idRef_pu = -1.0f * PFC_gv_vBus_out;
//
//
  } else
  {
    PFC_idRef_pu = -1.0f * PFC_IREF_DEFAULT;
    PFC_firstTimeGvLoop = SET;
  }

  //
  // Connect variables needed for debug to the datalogger
  //
//	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,
//                                   (uint32_t) ((PFC_angleSPLL_radians / 2 / PFC_PI) * 1.0f * 4095));
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,
                                   (uint32_t) ((PFC_iGrid_sensed_pu[0] + 1) * 0.5f * 4095));

  PFC_filterAndCheckForBusOverVoltage();

}



#ifdef __cplusplus
}
#endif

#endif // CONTROL_H