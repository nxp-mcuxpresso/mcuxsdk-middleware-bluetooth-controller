/**
 * @file main.c
 *
 * This file implements HARTT unit test main function
 */
/*
 * Copyright 2021-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "fsl_os_abstraction.h"
#include "EmbeddedTypes.h"
#include "lcl_hadm_hartt.h"

/*Latency = 2*40*4/Fs + 2*3.6e-6 + 2*12*4/Fs + 1.859e-6 = 0.000113059s */
#define TOF_LATENCY_1MBPS (113059) /* ns */
#define TOF_LATENCY_2MBPS (113059) /* TBD */

#define MAX_REASONABLE_FRAC 250 /* 2 ticks radio freq */

typedef struct
{
    bool rtt_found;
    uint8_t rtt_int_adj;
    uint8_t rtt_dist_sat;
    uint16_t rtt_p_delta;
    uint16_t rtt_cfo;
} hartt_unpacked_t;

/* stepno,t1,t2,hartt_stat,stat0,hpmcal,AA */
typedef struct
{
    uint8_t  stepno;
    uint16_t t1;
    uint16_t t2;
    uint32_t hartt_stat;
    uint32_t stat0;
    uint32_t hpmcal;
    uint32_t aa;
} hartt_test_vect_t;

typedef struct
{
    uint8_t  num_steps; // num steps in vect_p array
    uint8_t  data_rate; // 0: 1Mbps,  1:2Mbps
    uint8_t  true_dist; // in m
    uint8_t  dummy;
    hartt_test_vect_t *vect_I_p;
    hartt_test_vect_t *vect_R_p;
    char     *str;      // string description of TC entry
} hartt_test_case_t;

/* -------------------------------------- */

#define NUM_TC   6

#include "hartt_vect.h"

hartt_test_case_t hartt_tc[] = 
{
  { 18, 0, 3, 0, &test_vect_I_3m_24_22[0], &test_vect_R_3m_24_22[0], "08042021_1Mbps/3m/24_22" },
  { 18, 0, 3, 0, &test_vect_I_3m_24_24[0], &test_vect_R_3m_24_24[0], "08042021_1Mbps/3m/24_24" },
  { 18, 0, 3, 0, &test_vect_I_3m_24_25[0], &test_vect_R_3m_24_25[0], "08042021_1Mbps/3m/24_25" },
  { 18, 0, 9, 0, &test_vect_I_9m_18_46[0], &test_vect_R_9m_18_46[0], "08042021_1Mbps/9m/18_46" },
  { 18, 0, 9, 0, &test_vect_I_9m_18_47[0], &test_vect_R_9m_18_47[0], "08042021_1Mbps/9m/18_47" },
  { 18, 0, 9, 0, &test_vect_I_9m_18_50[0], &test_vect_R_9m_18_50[0], "08042021_1Mbps/9m/18_50" },
};

/* -------------------------------------- */

void unpackRttResult(uint32_t rtt_raw, hartt_unpacked_t *rtt_unpacked)
{
    /* Unpack the RTT raw results into separate entries */
    rtt_unpacked->rtt_cfo =       rtt_raw & 0x0000FFFF;
    rtt_unpacked->rtt_found =    (rtt_raw & 0x40000000) >> 30;
    rtt_unpacked->rtt_int_adj =  (rtt_raw & 0x30000000) >> 28;
    rtt_unpacked->rtt_dist_sat = (rtt_raw & 0x0C000000) >> 26;
    rtt_unpacked->rtt_p_delta =  (rtt_raw & 0x03FF0000) >> 16;
}

/* -------------------------------------- */

int main()
{
  int t,i, num_fail = 0; 
  
  for (t = 0; t < NUM_TC; t++)
  {
      uint8_t data_rate = hartt_tc[t].data_rate;
      printf("------------------------\n");
      printf("Reading Captures/%s true dist = %dm data rate = %d\n", hartt_tc[t].str, hartt_tc[t].true_dist, data_rate);

      for (i = 0; i < hartt_tc[t].num_steps; i++)
      {
          hartt_test_vect_t *vect_I_p, *vect_R_p;
          int32_t fracI, fracR, ToF;
          int32_t fracI_f, fracR_f, ToF_f, diff; // float results
          hartt_unpacked_t rtt_unpacked_I, rtt_unpacked_R;
          uint16_t RTT_I, RTT_R;
          
          vect_I_p = &hartt_tc[t].vect_I_p[i];
          vect_R_p = &hartt_tc[t].vect_R_p[i];

          /* Unpack HARTT results */
          unpackRttResult(vect_I_p->hartt_stat, &rtt_unpacked_I);
          unpackRttResult(vect_R_p->hartt_stat, &rtt_unpacked_R);
          
          /* Compute individual ToA-ToD */
          RTT_I = vect_I_p->t2 - vect_I_p->t1; /* handles wraps */
          RTT_R = vect_R_p->t2 - vect_R_p->t1;
          
          /* Compute frac delay in fixed-point*/
          lcl_hadm_hartt_enable_float(0);
          fracI = lcl_hadm_hartt_compute_fractional_delay(data_rate, vect_I_p->aa, rtt_unpacked_I.rtt_p_delta, rtt_unpacked_I.rtt_int_adj);
          fracR = lcl_hadm_hartt_compute_fractional_delay(data_rate, vect_R_p->aa, rtt_unpacked_R.rtt_p_delta, rtt_unpacked_R.rtt_int_adj);
          assert(fracI < MAX_REASONABLE_FRAC);
          assert(fracR < MAX_REASONABLE_FRAC);
          
          /* Compute frac delay in floatting-point*/
          lcl_hadm_hartt_enable_float(1);
          fracI_f = lcl_hadm_hartt_compute_fractional_delay(data_rate, vect_I_p->aa, rtt_unpacked_I.rtt_p_delta, rtt_unpacked_I.rtt_int_adj);
          fracR_f = lcl_hadm_hartt_compute_fractional_delay(data_rate, vect_R_p->aa, rtt_unpacked_R.rtt_p_delta, rtt_unpacked_R.rtt_int_adj);
          assert(fracI_f < MAX_REASONABLE_FRAC);
          assert(fracR_f < MAX_REASONABLE_FRAC);
          
          /* Compute ToF in ns */
          ToF = ((RTT_I * 125) >> 2) + fracI - ((RTT_R * 125) >> 2) + fracR;
          ToF_f = ((RTT_I * 125) >> 2) + fracI_f - ((RTT_R * 125) >> 2) + fracR_f;
          
          /* remove latency to align with Matlab model */
          if (data_rate == 0)
          {
              ToF -= TOF_LATENCY_1MBPS;
              ToF_f -= TOF_LATENCY_1MBPS;
          }
          else
          {
              ToF -= TOF_LATENCY_2MBPS;
              ToF_f -= TOF_LATENCY_2MBPS;
          }
          
          /* Print fixed-point results */
          printf("step:%02d fracI=%dns fracR=%dns ToF=%dns dist=%03.2fm\n", vect_I_p->stepno, fracI, fracR, ToF, ((float)ToF/2)*0.3);
          
          /* Compare fixed-point & floatting point results */
          if (ToF > ToF_f)
              diff = ToF - ToF_f;
          else
              diff = ToF_f - ToF;
          if (diff > 1) // 1 ns = 0.3m tolerance
          {
              printf("step:%02d verdict FAIL (ToF diff =%dns ToF float =%dns\n", vect_I_p->stepno, diff, ToF_f);
              num_fail++;
          }
      }
  }
  
  if (num_fail > 0)
      printf("\nFP vs float verdict FAIL");
  else
      printf("\nFP vs float verdict PASS. Please check against Matlab");
  
  return 0;
}

/* EOF */
