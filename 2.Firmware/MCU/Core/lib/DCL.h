#ifndef DCL_H__
#define DCL_H__


typedef struct dcl_pi {
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t i10;      //!< I storage
    float32_t Umax;     //!< Upper control saturation limit
    float32_t Umin;     //!< Lower control saturation limit
    float32_t i6;       //!< Saturation storage
    float32_t i11;      //!< I storage
    float32_t Imax;     //!< Upper integrator saturation limit
    float32_t Imin;     //!< Lower integrator saturation limit
    // DCL_PI_SPS *sps;    //!< Pointer to the shadow parameter set
    // DCL_CSS *css;       //!< Pointer to the common support structure
} DCL_PI;

//! \brief  Defines default values to initialize the PI structure
//!
#define PI_DEFAULTS { 1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, -1.0f, \
                      }


//! \brief          Executes a parallel form PI controller on the FPU32
//!                 Implemented as inline C function
//! \param[in] p    Pointer to the DCL_PI structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \return         The control effort
//!
static inline float32_t DCL_runPI_C3(DCL_PI *p, float32_t rk, float32_t yk)
{
    float32_t v1, v2, v4, v5, v9;

    v1 = rk - yk;
    v2 = p->Kp * v1;
    v4 = (v1 * p->Ki * p->i6) + p->i10;
    p->i10 = v4;
    v5 = v2 + v4;
    v9 = (v5 > p->Umax) ? p->Umax : v5;
    v9 = (v9 < p->Umin) ? p->Umin : v9;
    p->i6 = (v5 == v9) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    p->css->tpt = v5;
#endif

    return(v9);
}

static inline float32_t DCL_runPI_C2(DCL_PI *p, float32_t rk, float32_t yk)
{
    float32_t v2, v4, v5, v9;

    v2 = p->Kp * (rk - yk);
    v4 = p->i10 + (p->Ki * p->i6 * v2);
    v5 = v2 + v4;
    v9 = (v5 > p->Umax) ? p->Umax : v5;
    v9 = (v9 < p->Umin) ? p->Umin : v9;
    p->i10 = v4;
    p->i6 = (v5 == v9) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    p->css->tpt = v5;
#endif

    return(v9);
}

#endif //DCL_H__