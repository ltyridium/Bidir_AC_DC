//#############################################################################
//
//  FILE:   splll_3ph_srf.h
//
//  TITLE:  Software Phase Lock Loop for Three Phase Grid Tied Systems
//
//#############################################################################
// $TI Release: Software Phase Lock Loop Library v1.03.00.00 $
// $Release Date: Fri Dec 16 18:40:19 CST 2022 $
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef SPLL_3PH_SRF_H
#define SPLL_3PH_SRF_H

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \addtogroup SPLL_3PH_SRF
//! @{
//
//*****************************************************************************

//
// Included Files
//
#include <stdint.h>
#ifndef __TMS320C28XX_CLA__
#include <math.h>
#else
#include <CLAmath.h>
#endif

//#############################################################################
//
// Macro Definitions
//
//#############################################################################
#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES
#ifdef __TI_EABI__
typedef float         float32_t;
typedef double        float64_t;
#else // TI COFF
typedef float         float32_t;
typedef long double   float64_t;
#endif // __TI_EABI__
#endif // C2000_IEEE754_TYPES

//
// Typedefs
//

//! \brief          Defines the coefficients for a loop filter
//!
//! \details        Loop filter coefficients
//!
typedef struct{
    float32_t b1;
    float32_t b0;
} SPLL_3PH_SRF_LPF_COEFF;

//! \brief          Defines the SPLL_3PH_SRF structure
//!
//! \details        This software module implements a software phase lock loop
//!                 based on synchronous reference frame for
//!                 grid connection to three phase grid
//!
typedef struct{
    float32_t v_q[2];     //!< Rotating reference frame Q-axis value
    float32_t ylf[2];     //!< Data buffer for loop filter output
    float32_t fo;         //!< Output frequency of PLL
    float32_t fn;         //!< Nominal frequency
    float32_t theta[2];   //!< Grid phase angle
    float32_t delta_t;    //!< Inverse of the ISR rate at which module is called
    SPLL_3PH_SRF_LPF_COEFF lpf_coeff;  //!< Loop filter coefficients
} SPLL_3PH_SRF;

//! \brief              Initialize SPLL_3PH_SRF module
//! \param grid_freq    The grid frequency
//! \param delta_t      1/Frequency of calling the PLL routine
//! \param *spll_obj     The SPLL_3PH_SRF structure
//! \return None
//!
static inline void SPLL_3PH_SRF_init(float32_t grid_freq, float32_t delta_t,
                                     SPLL_3PH_SRF *spll_obj)
{
    spll_obj->v_q[0] = (float32_t)(0.0);
    spll_obj->v_q[1] = (float32_t)(0.0);

    spll_obj->ylf[0] = (float32_t)(0.0);
    spll_obj->ylf[1] = (float32_t)(0.0);

    spll_obj->fo = (float32_t)(0.0);
    spll_obj->fn = (float32_t)(grid_freq);

    spll_obj->theta[0] = (float32_t)(0.0);
    spll_obj->theta[1] = (float32_t)(0.0);

    spll_obj->delta_t = (float32_t)delta_t;
}

//! \brief              Reset SPLL_3PH_SRF module
//! \param *spll_obj     The SPLL_3PH_SRF structure
//! \return None
//!
static inline void SPLL_3PH_SRF_reset(SPLL_3PH_SRF *spll_obj)
{
    spll_obj->v_q[0] = (float32_t)(0.0);
    spll_obj->v_q[1] = (float32_t)(0.0);

    spll_obj->ylf[0] = (float32_t)(0.0);
    spll_obj->ylf[1] = (float32_t)(0.0);

    spll_obj->fo = (float32_t)(0.0);

    spll_obj->theta[0] = (float32_t)(0.0);
    spll_obj->theta[1] = (float32_t)(0.0);

}

//! \brief              Run SPLL_3PH_SRF module
//! \param v_q          Q component of the grid voltage
//! \param *spll_obj     The SPLL_3PH_SRF structure
//!
static inline void SPLL_3PH_SRF_run(float32_t v_q, SPLL_3PH_SRF *spll_obj)
{
    //
    // Update the spll_obj->v_q[0] with the grid value
    //
    spll_obj->v_q[0] = v_q;

    //
    // Loop Filter
    //
    spll_obj->ylf[0] =  spll_obj->ylf[1]
                     + (spll_obj->lpf_coeff.b0 * spll_obj->v_q[0])
                     + (spll_obj->lpf_coeff.b1 * spll_obj->v_q[1]);
    spll_obj->ylf[1] = spll_obj->ylf[0];
    spll_obj->v_q[1] = spll_obj->v_q[0];

    spll_obj->ylf[0] = (spll_obj->ylf[0] > (float32_t)(200.0)) ?
                                (float32_t)(200.0) : spll_obj->ylf[0];

    //
    // VCO
    //
    spll_obj->fo = spll_obj->fn + spll_obj->ylf[0];

    spll_obj->theta[0] = spll_obj->theta[1] +
                         ((spll_obj->fo * spll_obj->delta_t) *
                          (float32_t)(2.0 * 3.1415926));
    if(spll_obj->theta[0] > (float32_t)(2.0 * 3.1415926))
    {
        spll_obj->theta[0] = spll_obj->theta[0] - (float32_t)(2.0 * 3.1415926);
    }

    spll_obj->theta[1] = spll_obj->theta[0];
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of  _SPLL_3PH_SRF_H_ definition

//
// End of File
//
