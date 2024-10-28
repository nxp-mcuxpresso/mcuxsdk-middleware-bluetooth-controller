/**
 * @file lcl_hadm_hartt.c
 *
 * This file implements HARTT services for HADM
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* === Includes ============================================================ */
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"

/* === Types =============================================================== */

/* === Macros ============================================================== */
#define XOR3(A,B,C) (((A) ^ (B)) | ((B) ^ (C)))

#define P_DELTA_FP_FACT (1 << 9U)  /* Fixed-point quantification factor for p_delta: 2^9 */
#define SIGMA_FP_FACT   (1 << 20U) /* Fixed-point quantification factor for sigma matrix: 2^20. */
                                   /* Allows to keep k_coeff_fp on 32bits integers without loosing accuracy */
                                   /* (c_coeff requires 5bits, c_coeff^2 requires 10bits, 1bit for sign) */
/* === Globals ============================================================= */

#ifdef HARTT_ENABLE_FLOAT
const float sigma_1Mbps[6][4] = 
{
{ -0.00134209796949550  , 0.110145960558979    ,  -0.0248243797218084     , -0.671259234963662  },
{ 0.00322332545985516   , -0.00132674903483410 ,  0.0107652363631390      , 0.831944160011834   },
{ 0.000668087129714857  , 0.105887694139227    ,  -0.00631875679989436    ,-0.280619465195777   },
{ 3.28197400698171e-05  , -0.00119742985221536 ,  0.000863768332161733    ,0.0161286478820798   },
{ -5.24656456091608e-05 , 0.00171932817894839  ,  -0.000415106197518885   ,-0.0291225723402052  },
{ 1.77937505944535e-05  , 0.00328528242596985  ,  -0.000867562806380772   ,-0.00861216560743896 }
};

const float sigma_2Mbps[6][4] = 
{
{ -0.00257163622455905	, 0.135756463849064	, -0.0314897209128025	, -0.421652633196281     },
{ 0.00353970455987343	, -0.0392308769887525	, 0.0241947818216294	, 0.708136664403345      },
{ -0.000335207016826424	, 0.124561915952980	, -0.0138652107868654	, -0.263125212672375     },
{ 4.09087712587445e-05	, -0.00130129573031991	, 0.000743261187370301	, 0.00683212421233378    },
{ -7.17413469543641e-05	, 0.00303711773915562	, -0.000916254610883207	, -0.0250148638530733    },
{ 4.22266382204039e-05	, 0.00304994707909277	, -0.000768771158258999	, 0.00163548250807529    }
};
#endif

const int32_t sigma_1Mbps_fp[6][4] =    /* = sigma_1Mbps * 2^20 */
{
{ -1407 ,  115496,  -26030 , -703866 },
{ 3380  ,  -1391 ,   11288 , 872357  },
{ 701   , 111031 ,  -6626  , -294251 },
{ 34    , -1256  ,   906   , 16912   },
{ -55   , 1803   ,  -435   , -30537  },
{ 19    , 3445   ,  -910   , -9031   }
};

const int32_t sigma_2Mbps_fp[6][4] =    /* = sigma_2Mbps * 2^20 */
{
{-2697  ,142351 ,-33019 ,-442135    },
{3712   ,-41137 ,25370  ,742535     },
{-351   ,130613 ,-14539 ,-275907    },
{43	,-1365  ,779	,7164       },
{-75	,3185   ,-961	,-26230     },
{44	,3198	,-806	,1715       }
};

/* === Externals =========================================================== */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/* Count number of set bits in 32 bits word */
static uint32_t lcl_hadm_hartt_popcount(uint32_t n) 
{
    n = (n & 0x55555555u) + ((n >> 1) & 0x55555555u);
    n = (n & 0x33333333u) + ((n >> 2) & 0x33333333u);
    n = (n & 0x0f0f0f0fu) + ((n >> 4) & 0x0f0f0f0fu);
    n = (n & 0x00ff00ffu) + ((n >> 8) & 0x00ff00ffu);
    n = (n & 0x0000ffffu) + ((n >>16) & 0x0000ffffu);
    return n;
}

/*
* This function computes bit pattern counts for the given PN sequence
* Output array is: cnt_010, cnt_011, cnt_111
* Bit patterns are computed by performing logical operations on aa, aa>>1 and aa<<1,
* allowing efficient neighbor bits comparison in a single binary operation.
* Assumption is that reference PN sequence has boundary bits inversed compared to last bit preamble/first bit of PDU.
*/
static void lcl_hadm_hartt_compute_c_coeff(uint32_t pn_seq, uint32_t *c_coeff)
{
    uint32_t N111, N010;
    uint32_t aaL, aaR;

    /* Compute left bits with rightmost added bit being PN sequence inverted */
    aaL = pn_seq << 1U;
    aaL = (aaL & ~BIT0) | ((~pn_seq) & BIT0);

    /* Compute right bits with leftmost added bit being PN sequence inverted */
    aaR = pn_seq >> 1U;
    aaR = (aaR & ~BIT31) | ((~pn_seq) & BIT31);

    /* Compute logical operations for cnt_111 and cnt_010 */
    N111 = ~XOR3(aaL, pn_seq, aaR); /* All the bits being equal */
    N010 = (~((aaL)^(aaR)))&(pn_seq^(aaL)); /* left and right bits being equal and different from central bit */

    c_coeff[0U] = lcl_hadm_hartt_popcount(N010);  // cnt_010
    c_coeff[3U] = c_coeff[0U]*c_coeff[0U];        // cnt_010 ^2
    c_coeff[2U] = lcl_hadm_hartt_popcount(N111);  // cnt_111
    c_coeff[5U] = c_coeff[2U]*c_coeff[2U];        // cnt_111 ^2
    c_coeff[1U] = 32U - c_coeff[0U] - c_coeff[2U];// cnt_011
    c_coeff[4U] = c_coeff[1U]*c_coeff[1U];        // cnt_011 ^2
}

#ifdef HARTT_ENABLE_FLOAT
static uint8_t enable_float = 0;

/* for debug purpose, to compare with fixed-point results */
void lcl_hadm_hartt_enable_float(uint8_t en)
{
    enable_float = en;
}
#endif

/* 
*  This function computes fractional delay based on HARTT HW block correlation output and PN sequence correlation properties (cnt_xxx)
*  int_adj and frac_delay are relative to the radio clock used, hence unit is 1/4MHz or 1/8MHz for BLE 1Mbps or 2Mbps respectively
*  output is the fractional delay + integer adjustment in nanoseconds
*/
int32_t lcl_hadm_hartt_compute_fractional_delay(const uint32_t data_rate, const uint32_t pn_seq, int16_t  p_delta, const int32_t int_adj)
{
    uint32_t c_coeff[6];
    uint32_t k;
    int32_t  frac;
    uint32_t  Ts; // (1 / Fs) in ns
    const int32_t (*sigma_fp_p)[6][4];
#ifdef HARTT_ENABLE_FLOAT
    const float (*sigma_p)[6][4];
#endif

    if (data_rate == 0)
    {
        Ts =  250U; /* Ts = 1/4e6 * 1e9 (Fs = 4MHz @1Mbps) */
#ifdef HARTT_ENABLE_FLOAT
        sigma_p = &sigma_1Mbps;
#endif
        sigma_fp_p = &sigma_1Mbps_fp;
    }
    else
    {
        Ts =  125U; /* Ts = 1/8e6 * 1e9 (Fs = 8MHz @2Mbps) */
#ifdef HARTT_ENABLE_FLOAT
        sigma_p = &sigma_2Mbps; 
#endif
        sigma_fp_p = &sigma_2Mbps_fp;
    }

    /* Compute p_delta: p_delta format is sfix10_En9 aka Q9, hence 1 sign bit plus 9 fractional bits */
    p_delta <<= 6U; /* Align sign bit on MSB */
    p_delta /= (1 << 6U); /* and divide by 2^6 */
    
    /* Compute c coefficients corresponding to the given PN sequence */
    lcl_hadm_hartt_compute_c_coeff(pn_seq, c_coeff);

#ifdef HARTT_ENABLE_FLOAT
    if (!enable_float)
#endif
    {
        /* fixed-point version */
        /* quantization used targets 1ns accuracy compared to floatting point, which is the granularity of ToA-ToD defined in HCI spec */
        int32_t  k_coeff_fp[4U];
        int64_t  frac_fp; /* 32 bits are not enough given the ^3 operation */
        int64_t  p_delta_fp;

        p_delta_fp = (int64_t)p_delta; /* enforce 64bits operations on p_delta */

        /* Compute k coefficients  */
        /* [k0, k1, k2, k3] = [c010, c011, c111, c010^2, c011^2, c111^2] * SIGMA */
        for (k=0; k<4U; k++)
        {
            k_coeff_fp[k] = c_coeff[0U]*(*sigma_fp_p)[0U][k] + c_coeff[1U]*(*sigma_fp_p)[1U][k] + c_coeff[2U]*(*sigma_fp_p)[2U][k]
                          + c_coeff[3U]*(*sigma_fp_p)[3U][k] + c_coeff[4U]*(*sigma_fp_p)[4U][k] + c_coeff[5U]*(*sigma_fp_p)[5U][k];
        }
        
        /* Compute fractional delay corresponding to P_DELTA computed by HW */
        /* frac = [k0, k1, k2, k3] * [1, p_delta, p_delta^2, p_delta^3] */
        frac_fp  =   k_coeff_fp[0U] * P_DELTA_FP_FACT;
        frac_fp += ((k_coeff_fp[1U] * p_delta_fp));
        p_delta_fp *= p_delta; // p_delta^2
        frac_fp += ((k_coeff_fp[2U] * p_delta_fp) / (P_DELTA_FP_FACT));
        p_delta_fp *= p_delta; // p_delta^3
        frac_fp += ((k_coeff_fp[3U] * p_delta_fp) / (P_DELTA_FP_FACT * P_DELTA_FP_FACT));
        
        /* Convert to ns */
        frac = (int32_t)((frac_fp * Ts) / (SIGMA_FP_FACT * P_DELTA_FP_FACT));
    }
#ifdef HARTT_ENABLE_FLOAT
    else
    {
        /* floatting-point version */
        float    k_coeff[4U];
        float    frac_f;
        float    p_delta_f;
        
        p_delta_f = ((float)p_delta) / P_DELTA_FP_FACT;

        /* Compute k coefficients  */
        /* [k0, k1, k2, k3] = [c010, c011, c111, c010^2, c011^2, c111^2] * SIGMA */
        for (k=0; k<4U; k++)
        {
            k_coeff[k] = c_coeff[0U]*(*sigma_p)[0U][k] + c_coeff[1U]*(*sigma_p)[1U][k] + c_coeff[2U]*(*sigma_p)[2U][k]
                       + c_coeff[3U]*(*sigma_p)[3U][k] + c_coeff[4U]*(*sigma_p)[4U][k] + c_coeff[5U]*(*sigma_p)[5U][k];
        }
        
        /* Compute fractional delay corresponding to P_DELTA computed by HW */
        /* frac = [k0, k1, k2, k3] * [1, p_delta, p_delta^2, p_delta^3] */
        frac_f = k_coeff[0U] + (k_coeff[1U] * p_delta_f) + (k_coeff[2U] * p_delta_f * p_delta_f) + (k_coeff[3U] * p_delta_f * p_delta_f * p_delta_f);
        
        /* Convert to ns */
        frac = (int32_t)(frac_f * Ts);
    }
#endif
    
    /* Add integer adjustment. int_adj fomat is ufix2En0 and possible values are -1, 0 or 1 */
    if (int_adj == 1)
    {
        frac += Ts;
    }
    else if (int_adj == 3)
    {
        frac -= Ts;
    }

    return frac;
}
/* EOF */
