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

#include <Arduino.h>
#include "AREnv.h"

void AREnv::Process ( audio_block_t *apOut, int acSamples ) {

    int16_t* vpIn = apOut->data;

    UpdateGain();
     
    switch (mState)
    {
      case EnvState_Off:
      {
        int32_t vgain = (int32_t)( mCurBias * 65536.f );
        MulC( vpIn, vgain );
      }
        break;

      case EnvState_Attack:
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          int32_t vgain = (int32_t)( (mCurBias + mCurPeak) * 65536.f );
          vgain = min(vgain,65536);
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
          mCurPeak += mStep;
          mCurPeak = min(mCurPeak,1.f);
          if ( ( mStep>0.f && mCurPeak>=mPeak_req) || ( mStep<0.f && mCurPeak<=mPeak_req) ) {
              mState = EnvState_Hold;
          }
        }
      }
        break;
  
      case EnvState_Hold:
      {
        int32_t vgain = (int32_t)( (mCurBias + mCurPeak) * 65536.f );
        MulC( vpIn, vgain );  
      }
        break;
  
      case EnvState_Release:  
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          int32_t vgain = (int32_t)( (mCurBias + mCurPeak) * 65536.f );
          vgain = min(vgain,65536);
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
          mCurPeak += mStep;
          mCurPeak = max(mCurPeak,0.f);
          if ( mCurPeak <= 0.f ) {
            mState = EnvState_Off;
          } 
        }
      }
        break;
    }
}

void AREnv::TriggerAttack(){
    mAttack = mAttack_req;
    float vdelta = mPeak_req - mCurPeak;
    mStep = vdelta / (float)mAttack;
    mState = EnvState_Attack;
    Serial.println("TriggerAttack");
    Serial.println(mStep);
  }

void AREnv::TriggerRelease(){
  mRelease = mRelease_req;
  float vdelta = mCurPeak;
  mStep = -vdelta / (float)mRelease;
  mState = EnvState_Release;
  Serial.println("TriggerRelease: step = ");
  Serial.println(mStep);
}

// :TODO: refact optim
void AREnv::UpdateGain(){
  if ( mCurBias < mBias_req ){
        mCurBias += GAIN_STEP_PER_FRAME;
        if ( mCurBias > mBias_req )  mCurBias = mBias_req; 
      }
      else if ( mCurBias > mBias_req ){
        mCurBias -= GAIN_STEP_PER_FRAME;
        if ( mCurBias < mBias_req )  mCurBias = mBias_req; 
      }
}