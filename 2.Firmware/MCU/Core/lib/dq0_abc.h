//#############################################################################
//
//  FILE:   dq0_abc.h
//
//  TITLE:  DQ0 to ABC Transform Module
//
//#############################################################################
// $TI Release: C2000Ware DigitalPower SDK v4.03.00.00 $
// $Release Date: Fri Dec 16 18:13:09 CST 2022 $
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef DQ0_ABC_H
#define DQ0_ABC_H

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \addtogroup DQ0_ABC
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

//! \brief          Defines the DQ0_ABC transform structure
//!
//! \details        This block stores variables used to transform
//!                 DQ0 domain to ABC domain
//!
typedef struct{
    float32_t a;    //!< Output: A phase in 3PH AC Signal Component (alpha beta -> a,b,c)
    float32_t b;    //!< Output: B phase in 3PH AC Signal Component (alpha beta -> a,b,c)
    float32_t c;    //!< Output: C phase in 3PH AC Signal Component (alpha beta -> a,b,c)
    float32_t alpha;//!< Output: Alpha component (abc-> alpha beta)
    float32_t beta; //!< Output: Beta component  (abc-> alpha beta)
}DQ0_ABC;

//! \brief       Resets internal data to zero
//! \param *v    The DQ0_ABC structure pointer
//! \return None
//!
static inline void  DQ0_ABC_reset(DQ0_ABC *v)
{
    v->a = 0;
    v->b = 0;
    v->c = 0;
    v->alpha = 0;
    v->beta = 0;
}

//! \brief       Run DQ0_ABC module
//! \param *v    The DQ0_ABC structure pointer
//! \param d     d ref value
//! \param q     q ref value
//! \param z     z ref value
//! \param sine_val    sine value of the grid angle
//! \param cosine_val  cosine value of the grid angle
//! \return None
//!
static inline void DQ0_ABC_run(DQ0_ABC *v,
                               float32_t d, float32_t q, float32_t z,
                               float32_t sine_val, float32_t cosine_val)
{
    v->alpha = d * cosine_val  - q * sine_val;
    v->beta  = d * sine_val    + q * cosine_val;

    v->a     =  v->alpha     + 0.5f * z;
    v->b     = -0.5f * v->alpha + 0.8660254f * v->beta + 0.5f * z;
    v->c     = -0.5f * v->alpha - 0.8660254f * v->beta + 0.5f * z;
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


#endif // end of  _DQ0_ABC_H_ definition

//
// End of File
//
