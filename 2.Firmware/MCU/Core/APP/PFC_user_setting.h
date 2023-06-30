//
// Created by ltyridium on 2023/4/7.
//

#ifndef APF_PFC_USER_SETTING_H
#define APF_PFC_USER_SETTING_H


#define PFC_INVERTER_MODE 0
#define PFC_RECTIFIER_MODE 1
#define PFC_SPLL_SRF  0
#define PFC_SPLL_DDSRF  1
#define PFC_SPLL_TYPE PFC_SPLL_SRF

//
// LAB
// Lab 1 -> 4 , Inverter Mode
// 1 -> Open loop check for PWM drivers no relay close
// 2 -> Open loop check for PWM and check for pll no relay close
// 3 -> Open loop check for PWM drivers with protection, ADC sensing,
//      DC bus connected with a voltage source
//      Resistive load connected in a star as load
// 4-> Closed current loop with Resistive Load
//      DC bus connected with a voltage source
//      Resistive load connected in a star as load
// Lab 5 - 7 , PFC Mode
// 5 -> Open Loop CHeck with Grid Connected
//      with relay close , Resistive load connected at DC bus
// 6 -> Closed current loop with Grid Connected
//      Resistive load connected at DC bus
// 7 -> Closed current and voltage loop with Grid Connected
//      Resistive load connected at DC bus


#warning //When building this system for the first time, it is strongly recommended to test the relevant waveforms step by step according to the LAB process to ensure safety

#define LAB 7   //Uncomment the code to start starting


//
//Do not modify macro definitions unless you understand the options
//

#if LAB == 1
#define PFC_POWERFLOW_MODE PFC_INVERTER_MODE
#define PROTECTION PROTECTION_ENABLED
#define ENABLE_GATE_DEFAULT
#endif

#if LAB == 2
#define PFC_POWERFLOW_MODE PFC_INVERTER_MODE
#define PROTECTION PROTECTION_ENABLED
#define COLSE_PHASE_RELAY_DEFAULT
//#define CLOSE_INRUSH_RELAY_DEFAULT
#endif

#if LAB == 3
#define PFC_POWERFLOW_MODE PFC_INVERTER_MODE
#define PROTECTION PROTECTION_ENABLED
#define ENABLE_GATE_DEFAULT
#define COLSE_PHASE_RELAY_DEFAULT
#define CLOSE_INRUSH_RELAY_DEFAULT
#endif

#if LAB == 4
#define PFC_POWERFLOW_MODE PFC_INVERTER_MODE
#define PROTECTION PROTECTION_ENABLED
#define ENABLE_GATE_DEFAULT
#define COLSE_PHASE_RELAY_DEFAULT
#define CLOSE_INRUSH_RELAY_DEFAULT
#define PFC_CLOSE_I_LOOP
#endif

#if LAB == 5
#define PFC_POWERFLOW_MODE PFC_RECTIFIER_MODE
#define PROTECTION PROTECTION_ENABLED
#endif

#if LAB == 6
#define PFC_POWERFLOW_MODE PFC_RECTIFIER_MODE
#define PROTECTION PROTECTION_ENABLED
#endif


#if LAB == 7
#define PFC_POWERFLOW_MODE PFC_RECTIFIER_MODE
#define PROTECTION PROTECTION_ENABLED
#endif




// PROTECTION  ,
// 0 -> DISABLED
// 1 -> ENABLED

#define PROTECTION_ENABLED 1
#define PROTECTION_DISABLED 0


//#define OLED_ENABLE
#define UART_DEBUG_ENABLE

#define FAN_START_THRESHODE     (35)

#define INRUSH_RELAY_DELAY_MS   (100U)

#define USE_TIMEOUT 0
#define ADC_CALIBRATION_TIMEOUT_MS        ( 100U)
#define LED_BLINK_ERROR                   ( 500u)
#define ADC_OFFSST_SAMP_NUM               ( 828u)

#define ADC_OFFSST_VGRID 1



#if PROTECTION == PROTECTION_ENABLED
#define BOARD_PROTECTION_BUSOV 1
#define BOARD_PROTECTION_BUSOC 1
#define BOARD_PROTECTION_GATEFAULT 1
#define BOARD_PROTECTION_PHASEOC 1
#define BOARD_PROTECTION_GLOBALF 1
#else
//#define BOARD_PROTECTION_BUSOV 0
//#define BOARD_PROTECTION_BUSOC 0
//#define BOARD_PROTECTION_GATEFAULT 0
//#define BOARD_PROTECTION_PHASEOC 0
//#define BOARD_PROTECTION_GLOBALF 0
#endif


#if PROTECTION == PROTECTION_DISABLED
#warning protect is disable
#endif

#ifdef ENABLE_GATE_DEFAULT
#warning Gate drive signal is now enabled by default
#endif

#endif //APF_PFC_USER_SETTING_H
