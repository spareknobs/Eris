/*
   Eris - Dynamic Stochastic Synthesizer
   Spare Knobs 2020-2022

   GenDyn Oscillator

    The GENDYN algorithm (Iannis Xenakis) implemented here is inspired by 
    the Supercollider version by Nick Collins, see:
    https://github.com/supercollider/supercollider/blob/develop/server/plugins/GendynUGens.cpp

    + in this version, only the amplitudes are modulated by the stochastic 
    process, not the durations.

    + The 5 distributions are interpolated by mean of the "Dist" parameter


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

#define FREQ_RAMP_STEP_FRACT 0.1f
#define FREQ_MIN 20
#define FREQ_MAX 4500
#define LFO_FREQ_MIN 0.1f
#define LFO_FREQ_MAX 10.f
#define FREQ_MAX_FM 4500
#define PARAM_MIN 0.00001f
#define PARAM_MAX 1.f
#define SCALE_MIN 0.025f
#define SCALE_MAX 1.f
#define NUM_CONTROL_PTS_MAX 20
#define NUM_CONTROL_PTS_MIN 3

// we have 5 distributions
// divide the range in 4 interp sectors
// 0 = lin   1 = lfo
#define DIST_PARAM_RANGE 0.25
#define DIST_PARAM_THRES_1 0.25
#define DIST_PARAM_THRES_2 0.5
#define DIST_PARAM_THRES_3 0.75
#define DIST_PARAM_THRES_4 1.0 // to catch the lfo

static const double cLehmerCoef1 = 1.17f; 
static const double cLehmerCoef2 = 0.31f;     
    
inline const float Clip(float acValue,const float min, const float max){
    float vout = acValue < min ? min : acValue;
    vout = acValue > max ? max : acValue;
    return vout;
}

 enum DistId
 {
   Linear=0,
   Exponential=1,
   Cauchy=2,
   Hyperbcos=3, 
   Lfo=4,

   cNumDist
 };

class GenDyn
{

public:
    GenDyn(){}
    ~GenDyn(){}
   void Init(const float* apSeeds);
   void Process ( float *apOut, float *apCtl, float acFMAmount, int acSamples );
   void SetFreq( float acValue ); 
   void SetFreqNorm( float acValue ); // normalized in range min-max
   void SetFreqRange( int acValue ); // 0=hi 1=lo
   void SetDist( float acValue );
   void SetScale( float acValue );
   void SetParam( float acValue );
   void SetSamplerate( float acValue );
    float FreqNorm(){ return mFreqNorm; }

private:
    float ComputeInterpDist( float acParam2, float acLfo );
    float ComputeDist(  DistId acDist,
                         float acParam1, 
                        float acParam2, 
                        float acLfo );

    float mirroring( float in, const float acLimit );
    float LagrangeInterp( float x,float y0,float y1,float y2 );
    float LinearInterp( float x,float y0,float y1 );

private:
  
    // Params
    volatile float mSR{AUDIO_SAMPLE_RATE_EXACT};
    volatile float mFreq{440.f};
    volatile float mFreqNorm{0.f};
    volatile float mDistParam{0.f}; 
    volatile float mParam{1.f};
    volatile float mScale{0.5f};
    volatile int mFreqRange{0}; // 0=hi, 1=lo 
    
    // Runtime Vars
    DistId mDist1{Linear};
    DistId mDist2{Linear};
    float my0{0.f}; 
    float my1{0.f}; 
    float my2{0.f}; 
    float mdx{0.f};
    float mdxmin{0.f};
    float mdxmax{0.f};
    float mx{1.f};
    int16_t mIndex{0}; 
    float mY[NUM_CONTROL_PTS_MAX];
    float mdY[NUM_CONTROL_PTS_MAX];
    int mNumKP{8};
    float mPrevVal{0.f};
    float mPrevOut{0.f};
    float mFreqMin{FREQ_MIN};
    float mFreqMax{FREQ_MAX};
};
