#include "PFC.h"
#include "PFC_user_setting.h"

ABC_DQ0_POS PFC_vGrid_dq0_pos;
ABC_DQ0_NEG PFC_vGrid_dq0_neg;

ABC_DQ0_POS PFC_internalRef_dq0_pos;
ABC_DQ0_NEG PFC_internalRef_dq0_neg;

ABC_DQ0_POS PFC_iInv_dq0_pos;


DQ0_ABC PFC_vInv_dq0;

DCL_PI PFC_gi_id = PI_DEFAULTS;
DCL_PI PFC_gi_iq = PI_DEFAULTS;
DCL_PI PFC_gv_vBus = PI_DEFAULTS;

float32_t PFC_vdInv_pu;
float32_t PFC_vqInv_pu;
float32_t PFC_vzInv_pu;

float32_t PFC_duty_A_pu;
float32_t PFC_duty_B_pu;
float32_t PFC_duty_C_pu;

volatile float32_t PFC_vdInvRef_pu;
volatile float32_t PFC_vqInvRef_pu;

volatile float32_t PFC_idRef_pu;
volatile float32_t PFC_iqRef_pu;
volatile float32_t PFC_vBusRef_pu;
volatile float32_t PFC_vBusRefSlewed_pu;

//internal reference
volatile float32_t PFC_angle_radians;
volatile float32_t PFC_angleFixed_radians;
float32_t PFC_angleSPLL_radians;
float32_t PFC_sine_A;
float32_t PFC_sine_B;
float32_t PFC_sine_C;
float32_t PFC_cosine_A;
float32_t PFC_cosine_B;
float32_t PFC_cosine_C;
float32_t PFC_sine;
float32_t PFC_cosine;


float32_t PFC_gi_id_out;
float32_t PFC_gi_iq_out;
float32_t PFC_gv_vBus_out;


SPLL_3PH_SRF PFC_spll_3ph;
SPLL_3PH_DDSRF PFC_spll_3ph_1;

float32_t PFC_angleSPLL_radians;

float32_t PFC_sine;
float32_t PFC_cosine;

float32_t PFC_vGrid_A_sensed_prev_pu;
// input
volatile float32_t PFC_vGrid_sensed_pu[3];
volatile float32_t PFC_vGrid_sensedOffset_pu[3];

volatile float32_t PFC_iGrid_sensed_pu[3];
volatile float32_t PFC_iGrid_sensedOffset_pu[3];

volatile float32_t PFC_vBus_sensed_pu;

volatile float32_t PFC_vBusUp_sensed_pu;
volatile float32_t PFC_vBusDown_sensed_pu;

volatile float32_t PFC_vBusUp_sensedOffset_pu;
volatile float32_t PFC_vBusDown_sensedOffset_pu;

volatile float32_t PFC_iBus_sensed_pu;
volatile float32_t PFC_iBus_sensedOffset_pu;


float32_t PFC_vBus_sensed_Filtered_pu;
float32_t PFC_vBus_sensed_FilteredAndClamped_pu;


// SI unit
float32_t PFC_vBus_sensed_Volts;
float32_t PFC_vGridRms_A_sensed_Volts;
float32_t PFC_vGridRms_B_sensed_Volts;
float32_t PFC_vGridRms_C_sensed_Volts;


// use for state machine
volatile uint32_t PFC_enableHarmComp;
volatile uint32_t PFC_closeGiLoop;
volatile uint32_t PFC_closeGvLoop;
volatile uint32_t PFC_enablePWM = RESET;
volatile uint32_t PFC_startPowerStage;
volatile uint32_t PFC_rlyConnect;
volatile uint32_t PFC_firstTimeGvLoop;
volatile uint32_t PFC_deadBandSlewUpdate = 0;
volatile uint32_t PFC_deadBand = PFC_DEAD_TIME_MAX;


void PFC_control_init(void)
{
  ABC_DQ0_POS_reset(&PFC_vGrid_dq0_pos);
  ABC_DQ0_POS_reset(&PFC_internalRef_dq0_pos);
  ABC_DQ0_POS_reset(&PFC_iInv_dq0_pos);

  DQ0_ABC_reset(&PFC_vInv_dq0);

  SPLL_3PH_SRF_init(PFC_AC_FREQ_HZ,
                    (float32_t) (1.0 / PFC_ISR1_FREQUENCY_HZ),
                    &PFC_spll_3ph);

  PFC_spll_3ph.lpf_coeff.b0 = 66.6747378f;
  PFC_spll_3ph.lpf_coeff.b1 = -66.62144421;


  SPLL_3PH_DDSRF_init(PFC_AC_FREQ_HZ,
                      (float32_t) (1.0 / PFC_ISR1_FREQUENCY_HZ),
                      0.00933678,
                      -0.9813264,
                      &PFC_spll_3ph_1);

  PFC_spll_3ph_1.lpf_coeff.b0 = 66.6747378;
  PFC_spll_3ph_1.lpf_coeff.b1 = -66.62144421;



  // PID
  PFC_gi_id.Kp = PFC_GI_PI_KP;
  PFC_gi_id.Ki = PFC_GI_PI_KI;
  PFC_gi_id.Umax = PFC_GI_PI_MAX;
  PFC_gi_id.Umin = PFC_GI_PI_MIN;

  PFC_gi_iq.Kp = PFC_GI_PI_KP;
  PFC_gi_iq.Ki = PFC_GI_PI_KI;
  PFC_gi_iq.Umax = PFC_GI_PI_MAX;
  PFC_gi_iq.Umin = PFC_GI_PI_MIN;

  PFC_gv_vBus.Kp = PFC_GV_PI_KP;
  PFC_gv_vBus.Ki = PFC_GV_PI_KI;
  PFC_gv_vBus.Umax = PFC_GV_PI_MAX;
  PFC_gv_vBus.Umin = PFC_GV_PI_MIN;

  PFC_vdInvRef_pu = PFC_VREF_DEFAULT;
  PFC_vqInvRef_pu = 0.0f;

  PFC_vBusRef_pu = PFC_VBUS_NOMINAL_VOLTS / PFC_VBUS_MAX_SENSE_VOLTS;

  PFC_vGrid_sensedOffset_pu[0] = PFC_VGRIDA_OFFSET_PU;
  PFC_vGrid_sensedOffset_pu[1] = PFC_VGRIDB_OFFSET_PU;
  PFC_vGrid_sensedOffset_pu[2] = PFC_VGRIDC_OFFSET_PU;


  PFC_vBusUp_sensedOffset_pu = PFC_VBUS_UP_OFFSET_PU;
  PFC_vBusDown_sensedOffset_pu = PFC_VBUS_DOWN_OFFSET_PU;

  PFC_iBus_sensedOffset_pu = PFC_IBUS_OFFSET_PU;

#if PFC_POWERFLOW_MODE == PFC_INVERTER_MODE
  PFC_idRef_pu = PFC_IREF_DEFAULT;
  PFC_iqRef_pu = 0.0f;
#else
  PFC_idRef_pu = -PFC_IREF_DEFAULT;
  PFC_iqRef_pu = 0.0f;
#endif
#ifdef PFC_CLOSE_I_LOOP
  PFC_closeGiLoop = 1;
#endif

#ifdef PFC_CLOSE_V_LOOP
  PFC_closeGvLoop = 1;
#endif

//  PFC_vBus_sensed_FilteredAndClamped_pu = 250.0f / PFC_VBUS_MAX_SENSE_VOLTS;

}


//200Hz
__attribute__((section("ccmram")))
void PFC_autoStartPFC(void)
{
  static uint32_t starTick;
  switch (PFC_systemState.enum_systemState)
  {
    case systemState_CalOffset:
    {
      // 上电默认先校准ADC
      break;
    }
    case systemState_idle:
    {
      // 设备已经初始化完毕
      PFC_systemState.enum_systemState = systemState_waitForACVoltage;
      break;
    }
    case systemState_waitForACVoltage :
    {
//      if (PFC_vGridRms_A_sensed_Volts > PFC_UNIVERSAL_GRID_MIN_VRMS &&
//          PFC_vGridRms_B_sensed_Volts > PFC_UNIVERSAL_GRID_MIN_VRMS &&
//          PFC_vGridRms_C_sensed_Volts > PFC_UNIVERSAL_GRID_MIN_VRMS)
      if (PFC_vGrid_dq0_pos.d > 0.10f)
      {
				if (PFC_spll_3ph.fo > 65.0f || PFC_spll_3ph.fo < 45.0f)
				{
					PFC_boardFaultFlags.globalFault = SET;
					PFC_systemState.enum_systemState = systemState_Fault;
				}
        PFC_systemState.enum_systemState = systemState_state1;
      }
      break;
    }

    case systemState_state1:
    {
      //mask the phase over current fault for a while
      PFC_HAL_faultDisable_OCAC();
      PFC_HAL_closePhaseRelay();
      PFC_rlyConnect = SET;
      starTick = PFC_HAL_GetTick();
      PFC_systemState.enum_systemState = systemState_state2;
      break;
    }
    case systemState_state2:
    {
      if (PFC_rlyConnect == SET)
      {
        if (PFC_HAL_GetTick() - starTick > INRUSH_RELAY_DELAY_MS)
        {
          PFC_HAL_closeInrushRelay();
          if (PFC_HAL_GetTick() - starTick > 2 * INRUSH_RELAY_DELAY_MS)
          {
            //close inrush relay
            PFC_HAL_faultEnable_OCAC();
            if (PFC_HAL_faultGet_OCAC() == SET)
            {
              // if over current continue system go to fault
              PFC_systemState.enum_systemState = systemState_Fault;
            }
            else
            {
              PFC_startPowerStage = SET;
              PFC_systemState.enum_systemState = systemState_normalOperation;
            }
          }
        }
      }
      break;
    }

    case systemState_normalOperation:
    {
      //
      // Check for under voltage
      //
      if (PFC_vGridRms_A_sensed_Volts < PFC_UNIVERSAL_GRID_MIN_VRMS ||
          PFC_vGridRms_B_sensed_Volts < PFC_UNIVERSAL_GRID_MIN_VRMS ||
          PFC_vGridRms_C_sensed_Volts < PFC_UNIVERSAL_GRID_MIN_VRMS)
      {
        PFC_systemState.enum_systemState = systemState_Fault;
        PFC_boardFaultFlags.girdUnderVoltageFault = SET;
      }
      if (PFC_vBus_sensed_Volts > PFC_VBUS_OVERVOLT_LIMIT)
      {
        PFC_systemState.enum_systemState = systemState_Fault;
        PFC_boardFaultFlags.busOverVoltage = SET;
      }
			if (PFC_spll_3ph.fo > 65.0f || PFC_spll_3ph.fo < 45.0f)
			{
				PFC_boardFaultFlags.globalFault = SET;
				PFC_systemState.enum_systemState = systemState_Fault;
			}

      break;
    }
    case systemState_Fault:
    {
      PFC_rlyConnect = RESET;
      PFC_HAL_DisableGatePWM();
      PFC_HAL_openPhaseRelay();
      PFC_HAL_openInrushRelay();
      break;
    }
  }
}





