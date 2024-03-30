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

#pragma once

#include "Arduino.h"
#include "AudioStream.h"
#include "utility/dspinst.h"
#include "AREnv.h"
#include "GenDyn.h"
#include "VCFloat.h"
#include "MIDI.h"

#define Q_SCALER_16 32767.0
#define Q_SCALER_32 64536.0
#define NUM_MEMORY_BLOCKS 50
#define GAIN_RAMP_STEP 0.00001f
#define SR_DEF 44100.0
#define MAX_OSC_GAIN 0.6f
#define MAX_LFO_GAIN 1.f
#define VCF_OCTAVERANGE 7.f
#define DIV127 1.f/127.f

#define  NPARTIALRATIOS 9
static const float gcPartialsRatios[NPARTIALRATIOS] = { 0.5f, 1.f, 2.f, 2.9986f, 4.033f, 5.9997f, 8.01f, 10.093f, 11.330f };


class Eris : public AudioStream
{
    
public:
    
    Eris();
    virtual void update(void);

   // Gen1 Params
   void SetGen1Rate( float acValue );
   
   void SetGen1Scale( float acValue ){
      __disable_irq();
      mGen1.SetScale(acValue);
      __enable_irq();
   }

    void SetGen1Dist( float acValue ){
      __disable_irq();
      mGen1.SetDist(acValue);
      __enable_irq();
   }

   void SetGen1Param( float acValue ){
      __disable_irq();
      mGen1.SetParam(acValue);
      __enable_irq();
   }

    void SetGen1Gain( float acValue ){
      mGen1Gain_req = acValue;
   }
   
   // Gen2 Params

   void SetGen2Rate( float acValue );

   void SetGen2Dist( float acValue ){
      __disable_irq();
      mGen2.SetDist(acValue);
      __enable_irq();
   }

   void SetGen2Param( float acValue ){
      __disable_irq();
      mGen2.SetParam(acValue);
      __enable_irq();
   }

   void SetGen2Scale( float acValue ){
      __disable_irq();
      mGen2.SetScale(acValue);
      __enable_irq();
   }
   
   void SetGen2Gain( float acValue ){
      mGen2Gain_req = acValue;
   }
   
   void SetGen2ToOut( bool acValue ){
      mGen2ToOut = acValue;
   }
   
   void SetGen2Range( const bool acValue ){
      mGen2.SetFreqRange(acValue);
   }

   void SyncGens( const bool acValue ){
      mSyncGens=acValue;
   }

    void SetRateMod( bool acValue ){
      __disable_irq();
      mRateMod = acValue;
      __enable_irq();
   }

   void SetGainMod( bool acValue ){
      __disable_irq();
      mGainMod = acValue;
      __enable_irq();
   }

   // VCF
   void SetCutoff( float acValue ){
      __disable_irq();
      mVcf.SetCutoff( acValue );
      __enable_irq();
   }

   void SetResonance( float acValue ){
      __disable_irq();
      mVcf.SetResonance( acValue );
      __enable_irq();
   }

   void SetCutoffMod( bool acValue ){
      __disable_irq();
      mCutMod = acValue;

      if (acValue==true){
        mVcf.octaveControl( VCF_OCTAVERANGE );
      }
      else{
        mVcf.octaveControl( 0.f );
      }  
      __enable_irq();
   }

   // AREnv
   
   void SetAttackMs( float acValue ){
      __disable_irq();
      mAREnv.SetAttackMs(acValue);
      __enable_irq();
   }
    
   void SetReleaseMs( float acValue ){
      __disable_irq();
      mAREnv.SetReleaseMs(acValue);
      __enable_irq();
   }
   
   // Initial Gain (BIAS)
   void SetVCABiasGain( float acValue ){
       __disable_irq();
      mVCABiasGain_req = acValue;
      mAREnv.SetBiasGain(mVCABiasGain_req);
      __enable_irq();
   }

   // Master (output) Gain
   void SetMasterGain( float acValue ){
       __disable_irq();
      mMasterGain_req = (int32_t)( acValue * 65536 );
      __enable_irq();
   }

   // MIDI
   void TriggerMidiNote( byte acNote, byte acVel );
   void TriggerRelease();
         
   inline float GetGen1Val(){ return mLastGen1Val; }
   inline float GetGen2Val(){ return mLastGen2Val; }

   void Init(const float* apSeeds){ 
      mGen1.Init(apSeeds); 
      mGen2.Init(apSeeds); 
   }
   
private:

   // params
   volatile float mVCABiasGain_req{0.f};
   volatile int32_t mMasterGain_req{0};

   volatile float mGen1Gain_req{0.f};
   volatile float mGen2Gain_req{0.f};

   volatile float mGen1Rate_req{0.f};
   volatile float mGen2Rate_req{0.f};

   volatile float mMidiFreq_req{0.f};

    volatile bool  mGainMod{false};
    volatile bool  mCutMod{false};
    volatile bool  mRateMod{false};
    volatile bool  mGen2ToOut{false};
    volatile bool  mSyncGens{false};

    GenDyn mGen1;
    GenDyn mGen2;
    VCFloat mVcf;
    AREnv mAREnv;
    
    float mLastGen1Val{0.f};
    float mLastGen2Val{0.f};
    float mGen1Gain{0.f};
    float mGen2Gain{0.f};
    float mVCABiasGain{0.f};
    float mMasterGain{0.f};
};
