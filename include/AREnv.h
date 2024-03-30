/*
   Eris - Dynamic Stochastic Synthesizer
   Spare Knobs 2020-2022

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

// AR envelope generator - attack / release triggered

#pragma once

#include "Arduino.h"
#include "AudioStream.h"
#include "utility/dspinst.h"

#define ENV_PEAK_MAX 32000
#define GAIN_STEP_PER_FRAME 0.001

// Mult to Const, non-destructive vers
// signed multiply 32x16 gets:
// U( 0 + 0 + 1 , 15 + 15 ) = U(1,30) = 31 + sign = 32 bits >> 16 = 16 bits
inline void MulC( int16_t *dataOut, const int16_t *dataIn, int32_t mult )
{
    uint32_t *src = (uint32_t *)dataIn;
    uint32_t *dst = (uint32_t *)dataOut;
    const uint32_t *end = (uint32_t *)(dataOut + AUDIO_BLOCK_SAMPLES);
    
    do {
        uint32_t tmp32 = *src++; // read 2 samples from *data
        int32_t val1 = signed_multiply_32x16b(mult, tmp32);
        int32_t val2 = signed_multiply_32x16t(mult, tmp32);
        val1 = signed_saturate_rshift(val1, 16, 0);
        val2 = signed_saturate_rshift(val2, 16, 0);
        *dst++ = pack_16b_16b(val2, val1);
    } while (dst < end);
}

// In-Place Mult to Const 
// signed multiply 32x16 gets:
// U( 0 + 0 + 1 , 15 + 15 ) = U(1,30) = 31 + sign = 32 bits >> 16 = 16 bits
inline void MulC( int16_t *data, int32_t mult )
{
    uint32_t *p = (uint32_t *)data;
    const uint32_t *end = (uint32_t *)(data + AUDIO_BLOCK_SAMPLES);
    
    do {
        uint32_t tmp32 = *p; // read 2 samples from *data

        int32_t val1 = signed_multiply_32x16b(mult, tmp32);
        int32_t val2 = signed_multiply_32x16t(mult, tmp32);
        
        val1 = signed_saturate_rshift(val1, 16, 0);
        val2 = signed_saturate_rshift(val2, 16, 0);
        
        *p++ = pack_16b_16b(val2, val1);
        
    } while (p < end);
}

enum EnvState
{
  EnvState_Off=0,
  EnvState_Attack,
  EnvState_Hold,
  EnvState_Release
};

class AREnv
{

public:

    AREnv(){}
    ~AREnv(){}
   void Process ( audio_block_t *apOut, int acSamples );
   void TriggerAttack();
   void TriggerRelease();
     
   void SetAttackMs( float acValue ){
      float vnsamps = acValue * 0.001f * AUDIO_SAMPLE_RATE_EXACT;
      mAttack_req = (long)vnsamps + 1; // at least 1 sample
   }
    
   void SetReleaseMs( float acValue ){
      float vnsamps = acValue * 0.001f * AUDIO_SAMPLE_RATE_EXACT;
      mRelease_req = (long)vnsamps + 1; // at least 1 samp 
   }
   
   void SetGain( float acValue ){
      mPeak_req = acValue;
   }

   void SetBiasGain( float acValue ){
      mBias_req = acValue;
      //mCurBias = mBias_req; // temp
   }

   bool Done(){ return mState==EnvState_Off; }
 
private:
   void UpdateGain();

    // Params
    volatile long mAttack_req{1};  // in samps
    volatile long mRelease_req{1};  // in samps
    volatile float  mPeak_req{0.f};
    volatile float  mBias_req{0.f}; // bias gain
    volatile EnvState mState{EnvState_Off};
    
    long mAttack {1}; // in samps
    long mRelease {1}; // in samps
    float mCurPeak {0.f};
    float mCurBias {0.f};
    float mStep {0.f};
};

