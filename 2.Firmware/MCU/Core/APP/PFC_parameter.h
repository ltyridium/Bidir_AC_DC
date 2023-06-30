

#define PFC_PI ((float32_t)3.141592653589)
#define PFC_ADC_PU_SCALE_FACTOR  ((float32_t)1/(1<<12))  //12bit


#warning //Modify parameters here to suit your system

#define PWM_SWITCHING_FREQ_HZ ((float32_t)41525)

#define PFC_UNIVERSAL_GRID_MIN_VRMS ((float32_t)25.0)
#define PFC_UNIVERSAL_GRID_MAX_VRMS ((float32_t)125)

#define PFC_VBUS_OVERVOLT_LIMIT (390)


#define CNTRL_ISR_FREQ_RATIO 1
#define PFC_ISR1_FREQUENCY_HZ ((float32_t)PWM_SWITCHING_FREQ_HZ / CNTRL_ISR_FREQ_RATIO)

#define PFC_DEAD_TIME_MAX  (511)
#define PFC_DEAD_TIME_MIN  (45)

#define PFC_AC_FREQ_HZ ((float32_t)50.0)

#define PFC_VOLTS_PER_SECOND_SLEW ((float32_t) 100)
#define PFC_VBUS_NOMINAL_VOLTS ((float32_t)350)



#define PFC_LI_INDUCTOR_VALUE  ((float32_t)0.5*0.001)
#define PFC_LG_INDUCTOR_VALUE  ((float32_t)0.0068*0.001)
#define PFC_VGRID_MAX_SENSE_VOLTS ((float32_t)269.5)
#define PFC_VINV_MAX_SENSE_VOLTS PFC_VGRID_MAX_SENSE_VOLTS

#define PFC_VBUS_UP_MAX_SENSE_VOLTS ((float32_t)149.07)
#define PFC_VBUS_UP_OFFSET_PU       ((float32_t)0.1641)
#define PFC_VBUS_DOWN_MAX_SENSE_VOLTS ((float32_t)207)
#define PFC_VBUS_DOWN_OFFSET_PU       ((float32_t) -0.1641)

#define PFC_VBUS_CLAMP_MIN_PU (0.1f)


#define PFC_IBUS_MAX_SENSE_AMPS ((float32_t) 10.74)
#define PFC_IBUS_OFFSET_PU       ((float32_t) 0.4937)


#define PFC_VBUS_MAX_SENSE_VOLTS (PFC_VBUS_UP_MAX_SENSE_VOLTS + PFC_VBUS_DOWN_MAX_SENSE_VOLTS)

#define PFC_IINV_MAX_SENSE_AMPS ((float32_t)5.894308943)
#define PFC_IINV_TRIP_LIMIT_AMPS ((float32_t)5)
//#define PFC_IGRID_MAX_SENSE_AMPS PFC_IINV_MAX_SENSE_AMPS
//#define PFC_IGRID_TRIP_LIMIT_AMPS PFC_IINV_TRIP_LIMIT_AMPS

#define PFC_VGRIDA_OFFSET_PU ((float32_t) 0.58432)
#define PFC_VGRIDB_OFFSET_PU ((float32_t) 0.58432)
#define PFC_VGRIDC_OFFSET_PU ((float32_t) 0.5830)

#define PFC_DECOUPLING_CONST ((float32_t) 2.0f * PFC_PI * PFC_LI_INDUCTOR_VALUE * PFC_IINV_MAX_SENSE_AMPS / PFC_VBUS_MAX_SENSE_VOLTS )

//
// Control Loop Design
//

#define PFC_GI_RUN DCL_runPI_C2
#define PFC_GV_RUN DCL_runPI_C3

#define PFC_DUTY_CLAMP ((float32_t)0.98)

#define PFC_VREF_DEFAULT  ((float32_t)0.85)
#define PFC_IREF_DEFAULT  ((float32_t)0.1)

#define PFC_GI_PI_KP ((float32_t)0.1693591151)
#define PFC_GI_PI_KI ((float32_t)0.0075683544)

#define PFC_GV_PI_KP ((float32_t) 1.0540138247)
#define PFC_GV_PI_KI ((float32_t) 0.0051723506)

#define PFC_GI_PI_MAX ((float32_t)0.9)
#define PFC_GI_PI_MIN ((float32_t)-0.9)

#define PFC_GV_PI_MAX ((float32_t)0.9)
#define PFC_GV_PI_MIN ((float32_t)-0.2)





