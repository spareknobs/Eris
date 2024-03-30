/*
   Eris - Dynamic Stochastic Synthesizer
   Spare Knobs 2020-2022

  State Variable Filter (Chamberlin) with 2X oversampling

  A floating point version of the implementation by Paul Stoffregen, see:
  https://github.com/PaulStoffregen/Audio

/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "Arduino.h"
#include "AudioStream.h"

#define CUTOFF_MIN 40.f
#define CUTOFF_MAX 8000.f

class VCFloat
{
  
public:

    VCFloat() {
    SetCutoff(1000);
    octaveControl(1.0); 
    SetResonance(0.707);
    state_inputprev = 0;
    state_lowpass = 0;
    state_bandpass = 0;
  }
  
  void SetCutoff( float freq ) 
  {
    mCutoffRadians = freq * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT;
  }
  
  void SetResonance( float q ) 
  {
    if (q < 0.5) q = 0.5;
    else if (q > 5.0) q = 5.0;
    // TODO: allow lower Q when frequency is lower
    mDamp = (1.0 / q);
  }
  
  void octaveControl(float acValue) 
  {
    // filter's corner frequency is Fcenter * 2^(control * N)
    // where "control" ranges from -1.0 to +1.0
    // and "N" allows the frequency to change from 0 to 7 octaves
    if ( acValue < 0.f ) acValue = 0.f;
    else if ( acValue > 6.9999f ) acValue = 6.9999f;
    mOctavemult = acValue;
  }
  
  void Process( float *apIn, float *apOut, float *apCtl );
  
private:
  
  // params
  volatile float mCutoffRadians; // in radians, 0...pi/2
  volatile float mOctavemult;
  volatile float mDamp;

  // internals

  float state_inputprev;
  float state_lowpass;
  float state_bandpass;
  float mRadiansMin{ CUTOFF_MIN * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT}; 
  float mRadiansMax{ CUTOFF_MAX * 3.141592654f / AUDIO_SAMPLE_RATE_EXACT}; 
  
};
