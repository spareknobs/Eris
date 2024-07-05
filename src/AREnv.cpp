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

#include "AREnv.h"

void AREnv::Process ( audio_block_t *apOut, int acSamples ) {

    int16_t* vpIn = apOut->data;

    switch (mState)
    {
      case EnvState_Off:
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          UpdateGain();
          int32_t vgain = min(mCurBias,65536);
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
        }
      }
        break;

      case EnvState_Attack:
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          UpdateGain();
          int32_t vgain = min( mCurBias + mCurPeak, 65536 );
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
          mCurPeak += mStep;
          mCurPeak = min(mCurPeak, 65536);
          if ( ( mStep>0 && mCurPeak>=mPeak_req) || ( mStep<0 && mCurPeak<=mPeak_req) ) {
              mState = EnvState_Hold;
          }
        }
      }
        break;
  
      case EnvState_Hold:
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          UpdateGain();
          int32_t vgain = min(mCurBias + mCurPeak, 65536);
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
        }
      }
        break;
  
      case EnvState_Release:  
      {
        for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
          UpdateGain();
          int32_t vgain = min(mCurBias + mCurPeak, 65536);
          vpIn[n] = ( vgain * vpIn[n] ) >> 16;
          mCurPeak += mStep;
          mCurPeak = max(mCurPeak,0);
          if ( mCurPeak <= 0 ) {
            mState = EnvState_Off;
          } 
        }
      }
        break;
    }
}

void AREnv::TriggerAttack(){
    mAttack = mAttack_req;
    int32_t vdelta = mPeak_req - mCurPeak;
    mStep = vdelta / mAttack;
    mState = EnvState_Attack;
    Serial.println("TriggerAttack");
    Serial.println(mStep);
  }

void AREnv::TriggerRelease(){
  mRelease = mRelease_req;
  int32_t vdelta = mCurPeak;
  mStep = -vdelta / mRelease;
  mState = EnvState_Release;
  Serial.println("TriggerRelease: step = ");
  Serial.println(mStep);
}

void AREnv::UpdateGain(){
  if ( mCurBias < mBias_req ){
        mCurBias += GAIN_STEP_PER_SAMPLE;
        if ( mCurBias > mBias_req )  mCurBias = mBias_req; 
      }
      else if ( mCurBias > mBias_req ){
        mCurBias -= GAIN_STEP_PER_SAMPLE;
        if ( mCurBias < mBias_req )  mCurBias = mBias_req; 
      }
}