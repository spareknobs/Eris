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
#include "Eris.h"

Eris::Eris() : AudioStream( 0, NULL ){
   mGen1.SetSamplerate(SR_DEF);
   mGen2.SetFreqRange(0);
   mGen1.SetFreqNorm(0.25f);
   mGen1.SetScale(0.5f);
   mGen1.SetParam(0.5f);
   mGen2.SetSamplerate(SR_DEF);
   mGen2.SetFreqRange(0);
   mGen2.SetFreqNorm(0.5f);
   mGen2.SetParam(1.f);
   mGen2.SetScale(1.f);
   mGen2.SetDist(0.f);
   mVcf.SetCutoff(5000.f);
   mVcf.SetResonance(1.f);
   mVcf.octaveControl(0.f);
   mAREnv.SetGain(1.f);
   mAREnv.SetAttackMs(100.f);
   mAREnv.SetReleaseMs(200.f);
}
     
 void Eris::update(void){

   float blockGen1[AUDIO_BLOCK_SAMPLES];
   float blockGen2[AUDIO_BLOCK_SAMPLES];
   float blockLfo[AUDIO_BLOCK_SAMPLES];

   audio_block_t *blockout;
   blockout = allocate();
   if (!blockout) return; 

   memset(blockGen1, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));
   memset(blockGen2, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));
   memset(blockLfo, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));

   // Process Gen2 - todo: use prev gen1 buf as ctrl sig?
   mGen2.Process( blockGen2, blockGen2, 0.f, AUDIO_BLOCK_SAMPLES );   

   // Gen2 --> Lfo - make unipolar & expand
   for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
      float val = 0.5f * blockGen2[n] + 0.5f;
      blockLfo[n] = val * val * mGen2Gain;
   }
         
   // Update Gen2 Gain
   float *op2 = (float *)blockGen2;
   const float *oend2 = (float *)(blockGen2 + AUDIO_BLOCK_SAMPLES);
    
   do {
         if ( mGen2Gain < mGen2Gain_req ){
            mGen2Gain += GAIN_RAMP_STEP;
            if ( mGen2Gain > mGen2Gain_req )  mGen2Gain = mGen2Gain_req; 
         }
         else if ( mGen2Gain > mGen2Gain_req ){
            mGen2Gain -= GAIN_RAMP_STEP;
            if ( mGen2Gain < mGen2Gain_req )  mGen2Gain = mGen2Gain_req; 
         }

        float tmp = *op2;
        float val = mGen2Gain * tmp;
        *op2++ = val;
        
   } while (op2 < oend2);

   // Modulate Gen1 Rate with Lfo according to RateMod switch
   float vRateMod = (float)mRateMod;
   mGen1.Process( blockGen1, blockLfo, vRateMod, AUDIO_BLOCK_SAMPLES );

   // Update Gen1 Gain @ sr
   float *op = (float *)blockGen1;
   const float *oend = (float *)(blockGen1 + AUDIO_BLOCK_SAMPLES);
    
   do {
         if ( mGen1Gain < mGen1Gain_req ){
            mGen1Gain += GAIN_RAMP_STEP;
            if ( mGen1Gain > mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }
         else if ( mGen1Gain > mGen1Gain_req ){
            mGen1Gain -= GAIN_RAMP_STEP;
            if ( mGen1Gain < mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }

        float tmp = *op;
        float val1 = mGen1Gain * tmp;
        *op++ = val1;
        
   } while (op < oend);
   
   // save a value for controlling LEDs
   mLastGen1Val = blockGen1[0] * blockGen1[0];
   mLastGen2Val = blockGen2[0] * blockGen2[0];
   
   if (mGen2ToOut){
      // mix gens --> out
      for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
         blockGen1[n] += blockGen2[n];
      }
   }

   // Process Filter 
   mVcf.Process( blockGen1, blockGen1, blockLfo );

   // Write to fixed point output
   for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
      float val = blockGen1[n];
      blockout->data[n] = saturate16( val * 32767.0 );
   }

   // Process AR
   mAREnv.Process( blockout, AUDIO_BLOCK_SAMPLES );
   if (mAREnv.Done()) mMidiFreq_req=0;

   // (double mono for now)
   transmit( blockout,0 );
   transmit( blockout,1 );   
   release( blockout );
 }

void Eris::TriggerMidiNote( byte acNote, byte acVel ){
      __disable_irq();
      mMidiFreq_req = gcNoteFreqs[acNote];
      mGen1.SetFreq(mMidiFreq_req);
      float mult = round( 1.f + mGen2Rate_req * 4.f );
      float vval = Clip( mult * mGen1.FreqNorm(), 0.f, 1.f );
      mGen2.SetFreqNorm( vval );   
      mAREnv.SetGain( (float)acVel * DIV127 );
      mAREnv.TriggerAttack();
      __enable_irq();
    }

void Eris::TriggerRelease(){
      __disable_irq();
      mAREnv.TriggerRelease();
      __enable_irq();
   }

void Eris::SetGen1Rate( float acValue ){
      // if midi note active, bypass
   if (mMidiFreq_req > 0) return;
   __disable_irq();
   mGen1Rate_req = acValue;
   mGen1.SetFreqNorm(mGen1Rate_req);
   if ( mSyncGens==true || mMidiFreq_req>0 ){
      int ind = (int)round( mGen2Rate_req * (NPARTIALRATIOS-1) );
      float vpartialRatio = gcPartialsRatios[ind];
      float vval = Clip( vpartialRatio * mGen1.FreqNorm(), 0.f, 1.f );
      mGen2.SetFreqNorm( vval );
   }      
   __enable_irq();
}

void Eris::SetGen2Rate( float acValue ){      
   __disable_irq();
   mGen2Rate_req = acValue;
   // if midi note active, sync 
   // this way gen2 rate will control the harmonic #
   if ( mSyncGens==true || mMidiFreq_req>0 ){
      int ind = (int)round( mGen2Rate_req * (NPARTIALRATIOS-1) );
      float vpartialRatio = gcPartialsRatios[ind];
      float vval = Clip( vpartialRatio * mGen1.FreqNorm(), 0.f, 1.f );
      mGen2.SetFreqNorm( vval );
   }
   else{
      mGen2.SetFreqNorm( mGen2Rate_req );
   }
   __enable_irq();
}