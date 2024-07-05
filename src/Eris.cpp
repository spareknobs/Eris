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
   msine.set_freq(400);
}

// test update
  /* void Eris::update(void){

   float blockGen1[AUDIO_BLOCK_SAMPLES];
   int16_t blockGen1t[AUDIO_BLOCK_SAMPLES];
   int16_t blockGen2t[AUDIO_BLOCK_SAMPLES];
   audio_block_t *blockout;
   blockout = allocate();
   if (!blockout) return; 

   memset(blockGen1, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));
   memset(blockGen1t, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
   memset(blockGen2t, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
   
   msine.Process(blockGen1, AUDIO_BLOCK_SAMPLES);

   //mGen1.Process( blockGen1, blockLfo, 0, AUDIO_BLOCK_SAMPLES );

   f2fix(blockGen1, blockGen1t, AUDIO_BLOCK_SAMPLES);

   memcpy( blockGen2t, blockGen1t, AUDIO_BLOCK_SAMPLES * sizeof(int16_t) );

   // Gen1 Gain @ sr
   int16_t *opt = (int16_t*)blockGen1t; 
   const int16_t* oendt = (int16_t*)(blockGen1t + AUDIO_BLOCK_SAMPLES);
   do {
         if ( mGen1Gain < mGen1Gain_req ){
            mGen1Gain += GAIN_RAMP_STEP;
            if ( mGen1Gain > mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }
         else if ( mGen1Gain > mGen1Gain_req ){
            mGen1Gain -= GAIN_RAMP_STEP;
            if ( mGen1Gain < mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }
        int16_t tmp = *opt;
        *opt++ = ( mGen1Gain * tmp ) >> 16;
   } while (opt < oendt);

   for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
      int16_t vmix = signed_add_16_and_16(blockGen1t[n],blockGen2t[n]);
      blockout->data[n] = vmix;
   }

   //memcpy( blockout->data, blockGen1t, AUDIO_BLOCK_SAMPLES * sizeof(int16_t) );

   // Process AR
   //mAREnv.Process( blockout, AUDIO_BLOCK_SAMPLES );

   // (double mono for now)
   transmit( blockout,0 );
   transmit( blockout,1 );   
   release( blockout );
 }*/

 void Eris::update(void){

   float blockGen1[AUDIO_BLOCK_SAMPLES];
   int16_t blockGen1t[AUDIO_BLOCK_SAMPLES];
   float blockGen2[AUDIO_BLOCK_SAMPLES];
   int16_t blockGen2t[AUDIO_BLOCK_SAMPLES];
   float blockLfo[AUDIO_BLOCK_SAMPLES];

   audio_block_t *blockout;
   blockout = allocate();
   if (!blockout) return; 

   memset(blockGen1, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));
   memset(blockGen2, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));
   memset(blockGen1t, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
   memset(blockGen2t, 0, AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
   memset(blockLfo, 0, AUDIO_BLOCK_SAMPLES * sizeof(float));

   mGen2.Process( blockGen2, blockGen2, 0.f, AUDIO_BLOCK_SAMPLES );   
         
   f2fix(blockGen2, blockGen2t, AUDIO_BLOCK_SAMPLES);

   // Update Gen2 Gain
   int16_t *op2 = (int16_t*)blockGen2t; 
   const int16_t* oend2 = (int16_t*)(blockGen2t + AUDIO_BLOCK_SAMPLES);
   do {
         if ( mGen2Gain < mGen2Gain_req ){
            mGen2Gain += GAIN_RAMP_STEP;
            if ( mGen2Gain > mGen2Gain_req )  mGen2Gain = mGen2Gain_req; 
         }
         else if ( mGen2Gain > mGen2Gain_req ){
            mGen2Gain -= GAIN_RAMP_STEP;
            if ( mGen2Gain < mGen2Gain_req )  mGen2Gain = mGen2Gain_req; 
         }
        int16_t tmp = *op2;
        *op2++ = ( mGen2Gain * tmp ) >> 16;
   } while (op2 < oend2);

   // Gen2 --> Lfo - make unipolar & expand
   for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
      float val =  (float)blockGen2t[n] * Q_DIV_16;
      val = 0.5 * val + 0.5; 
      blockLfo[n] = val * val * 2.0; // scale up 
   }

   // Modulate Gen1 Rate with Lfo
   float vRateMod = (float)mRateMod;
   mGen1.Process( blockGen1, blockLfo, vRateMod, AUDIO_BLOCK_SAMPLES );
   f2fix(blockGen1, blockGen1t, AUDIO_BLOCK_SAMPLES);

   // Gen1 Gain @ sr
   int16_t *op1 = (int16_t*)blockGen1t; 
   const int16_t* oend1 = (int16_t*)(blockGen1t + AUDIO_BLOCK_SAMPLES);
   do {
         if ( mGen1Gain < mGen1Gain_req ){
            mGen1Gain += GAIN_RAMP_STEP;
            if ( mGen1Gain > mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }
         else if ( mGen1Gain > mGen1Gain_req ){
            mGen1Gain -= GAIN_RAMP_STEP;
            if ( mGen1Gain < mGen1Gain_req )  mGen1Gain = mGen1Gain_req; 
         }
        int16_t tmp = *op1;
        *op1++ = ( mGen1Gain * tmp ) >> 16;
   } while (op1 < oend1);
   
   // save a value for controlling LEDs
   mLastGen1Val = blockGen1[0] * blockGen1[0];
   mLastGen2Val = blockGen2[0] * blockGen2[0];
   
   if (mGen2ToOut){
      for ( int n=0; n < AUDIO_BLOCK_SAMPLES; ++n ) {
         int16_t vmix = signed_add_16_and_16(blockGen1t[n],blockGen2t[n]);
         blockGen1t[n] = vmix;
      }
   }

   // temp
   fix2f(blockGen1t,blockGen1,AUDIO_BLOCK_SAMPLES);

   // Process Filter 
   mVcf.Process( blockGen1, blockGen1, blockLfo );

   // temp
   f2fix(blockGen1, blockout->data, AUDIO_BLOCK_SAMPLES);

   //memcpy( blockout->data, blockGen1t, AUDIO_BLOCK_SAMPLES * sizeof(int16_t) );

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