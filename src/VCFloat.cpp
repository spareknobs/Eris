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

#include <Arduino.h>
#include "VCFloat.h"
#include "utility/dspinst.h"

void VCFloat::Process( float *apIn, float *apOut, float *apCtl )
{

  float *in = apIn;
  const float *end = in + AUDIO_BLOCK_SAMPLES;
  float *ctl = apCtl;
  float *out = apOut;
  float input, inputprev, control;
  float lowpass, bandpass, highpass;
  float lowpasstmp;// bandpasstmp, highpasstmp;
  float fmult, damp, octavemult;

  octavemult = mOctavemult;
  damp = mDamp;
  inputprev = state_inputprev;
  lowpass = state_lowpass;
  bandpass = state_bandpass;

 do 
  {
     // compute fmult using control input, fcenter and octavemult
    // filter's corner frequency is Fcenter * 2^(control * N)
    // where control ranges from 0 to +1.0
    // and "N" allows the frequency to change from 0 to 7 octaves
    
    control = *ctl++; 
    float vradians = mCutoffRadians * powf(2.f, control * octavemult );
    if ( vradians < mRadiansMin ) vradians = mRadiansMin;
    if ( vradians > mRadiansMax ) vradians = mRadiansMax;
    fmult = 0.5f * sinf( vradians );

    //---------------------------------------------------------------
    // now do the state variable filter as normal, using fmult
    input = *in++;
    
    // 2x oversamp: first pass
    // in + inprev / 2, simple interp for getting the 'ghost prev sample'
    lowpass = lowpass + fmult * bandpass;
    highpass = ( (input + inputprev) * 0.5f ) - lowpass - damp * bandpass;
    inputprev = input;
    bandpass = bandpass + fmult * highpass;
    lowpasstmp = lowpass;
    
    // second pass
    lowpass = lowpass + fmult * bandpass;
    highpass = input - lowpass - damp *bandpass;
    bandpass = bandpass + fmult * highpass;

    lowpasstmp = ( lowpass + lowpasstmp ) * 0.5f;
    *out++ = lowpasstmp;
    
  } while (in < end);
  
  state_inputprev = inputprev;
  state_lowpass = lowpass;
  state_bandpass = bandpass;
       
  return;
}
