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

#include "GenDyn.h"

void GenDyn::Init(const float *apSeeds)
{
    // freq range is [FREQ_MIN FREQ_MAX] thus mdx range is 
    float vNumKPMin = (int)floorf(0.25f / mFreqMax * mSR);
    float vNumKPMax = (int)floorf(0.25f / mFreqMin * mSR);
    mdxmin = mFreqMin / mSR * vNumKPMax;
    mdxmax = mFreqMax / mSR * vNumKPMin;
    mNumKP = 10;

    // init the state according to current knobs positions
    for( int i=0; i < NUM_CONTROL_PTS_MAX; ++i ) 
    {
        mY[i] = 0.f; //2.f * apSeeds[i] - 1.f;
        mdY[i] = 0.f; //2.f * apSeeds[i] - 1.f;
    }
}

void GenDyn::Process ( float *apOut, float *apCtl, float acFMAmount, int acSamples )
{      
    float* out = apOut;
    float* ctl = apCtl;
    int n = acSamples;

    while (n--)
    {
        if ( mx >= 1.f ) 
        {
            mx -= 1.f;
            mIndex = ++mIndex % mNumKP;

            float vrand = fabs( fmod( ( my0 * cLehmerCoef1 ) + cLehmerCoef2, 1.f ) );
            
            // y0,y1 = prev k-points values
            // y2 = target point value
            my0 = my1;
            my1 = my2;
         
            float vdy = mdY[mIndex] + ComputeInterpDist( vrand, *ctl );
            vdy = mirroring( vdy, 1.f );
            mdY[mIndex] = vdy;

            // update current point
            my2 = mY[mIndex] + ( mScale * vdy );
            my2 = mirroring( my2, 0.6f );
            mY[mIndex] = my2;
        }

        // interp btw prev point and current(target) point
        float val = LagrangeInterp( mx, my0, my1, my2 );

        // FM with ctl signal 
        float vfrange = min(FREQ_MAX-mFreq, mFreq-FREQ_MIN);
        float vf = mFreq + acFMAmount * vfrange * (*ctl);

        // mod numKP with freq in order to achieve higher freq range
        // vf*KP must be < nyquist, thus KP < 20KHz / vf
        mNumKP = (int)floorf( 20000.f / vf );
        if (mNumKP > NUM_CONTROL_PTS_MAX ) mNumKP = NUM_CONTROL_PTS_MAX;
        if (mNumKP < NUM_CONTROL_PTS_MIN ) mNumKP = NUM_CONTROL_PTS_MIN;
        mdx = vf / mSR * mNumKP;
        if (mdx >= 0.99f) mdx = 0.99f;
        mx += mdx;
    
         // dc blocking filter
        // R=0.995; % for 44100 SR
        float vout = val - mPrevVal + 0.9999f * mPrevOut;
        mPrevOut = vout;
        mPrevVal = val; 
        *out++ = vout;

        ctl++;
    }
}

float GenDyn::ComputeInterpDist( float acParam2, float acLfo ) {
    // compute Dist1
    float d1 = ComputeDist(mDist1,mParam,acParam2,acLfo);

    // compute Dist2
    float d2 = ComputeDist(mDist2,mParam,acParam2,acLfo);

    // interp btw the two
    float vd = d1 + (d2-d1) * mDistParam;

    return vd;
}

float GenDyn::ComputeDist(  DistId acDist, 
                            float acParam1,   
                            float acParam2, 
                            float acLfo ) 
{
    switch (acDist) 
    {
        case Linear:
        {
            // FIXME: using param1 here biases the dist,
            // replace with something like?
            // float vval = acParam1*(2.f * (acParam2) - 1.f);
            float vval = 2.f * (acParam1*acParam2) - 1.f;
            return vval;
        }
        break;

        case Cauchy:
        {
            float argmax = 1.5f;
            float argmin = 0.7f; //0.75f;
            float scale = acParam1*(argmax-argmin) + argmin;
            float arg = ( 2.f*acParam2 - 1.f ) *  scale;
            float vval = tanf( arg );
            float normfactor = tanf(scale);
            vval = vval / normfactor;

            return vval;
        }
        break;

        case Hyperbcos: 
        {
            // stretch param1 towards 1 using a log
            // [0,1] --> [-3,3]
            // exp(-3) = 0.0498
            // exp(3) = 20.0855
            float p1 = (logf( acParam1 * 20.0357f + 0.0498f ) + 3.f) * 0.1667f;
            float argmax = 1.4 * p1; // was 1.5
            float arg = acParam2 * argmax;
            float logargmin = 0.0005f; // was 0.001
            float vval = logf( tanf(arg) / tanf(argmax) * (1.f-logargmin) + logargmin );
            float normfact = 1.f / logf(logargmin);
            vval = vval * normfact;
            
            return vval;
        }
        break;

        case Exponential:
        {
            // X original -(log(1-z))/a  [0,1]-> [1,0]-> [0,-inf]->[0,inf]
            float c = logf( 1.f - ( 0.999f * acParam1 ) );
            float vval = logf( 1.f - ( acParam2 * 0.999f * acParam1) ) / c;
            
            return 2.f * vval - 1.f;
        }
        break;

        case Lfo:
        {
            float vval = acParam1 * acLfo;
            vval = 2.f * vval - 1.f;
            
            return vval;
        }
        break;
        
        default:
            return 0.f;
            break;
    }

    return 0.f;
}

// mirroring for bounds - new vers
// bounce until in bounds
 float GenDyn::mirroring(float in, const float limit )
 { 
    // compute distance from edge
    float dist = fabs(in) - limit;
    
    while ( dist > 0.f )
    {
        if ( in > limit )
        {
            in = in - (2.f * dist);
        }
        else
        {
            in = in + (2.f * dist);
        }
        
        dist = fabs(in) - limit;   
    }
    
    return in;   
}

void GenDyn::SetDist( float acValue )
{
    if ( acValue < 0.f ) acValue = 0.f;
    else if ( acValue > DIST_PARAM_THRES_4 ) acValue = DIST_PARAM_THRES_4;

    if ( acValue <= DIST_PARAM_THRES_1)
    {
        mDist1 = Linear;
        mDist2 = Exponential;
        mDistParam = acValue/DIST_PARAM_RANGE;
    }    
    else if ( acValue > DIST_PARAM_THRES_1 && acValue <= DIST_PARAM_THRES_2)
    {
        mDist1 = Exponential;
        mDist2 = Cauchy;
        mDistParam = ( acValue - DIST_PARAM_THRES_1 ) / DIST_PARAM_RANGE;
    }
    else if ( acValue > DIST_PARAM_THRES_2 && acValue <= DIST_PARAM_THRES_3)
    {
        mDist1 = Cauchy;
        mDist2 = Hyperbcos;
        mDistParam = ( acValue - DIST_PARAM_THRES_2 ) / DIST_PARAM_RANGE;
    }
    else if ( acValue > DIST_PARAM_THRES_3 && acValue <= DIST_PARAM_THRES_4)
    {
        mDist1 = Hyperbcos;
        mDist2 = Lfo;
        mDistParam = ( acValue - DIST_PARAM_THRES_3 ) / DIST_PARAM_RANGE;
    }
}
 
void GenDyn::SetScale( float acValue )
{
    float a = acValue;
    if( a > SCALE_MAX ) a = SCALE_MAX;
    if( a < SCALE_MIN ) a = SCALE_MIN;  
    mScale = a;
}

void GenDyn::SetParam( float acValue )
{
    float a = acValue;
    if( a > PARAM_MAX ) a = PARAM_MAX;       
    if( a < PARAM_MIN ) a = PARAM_MIN; 
    mParam = a;
}

void GenDyn::SetSamplerate( float acValue )
{
    mSR = acValue;
    mdx = mFreq / mSR * mNumKP;
    mdxmin = mFreqMin / mSR * mNumKP;
    mdxmax = mFreqMax / mSR * mNumKP;
}

// note: overwritten if you then call SetRange()
void GenDyn::SetFreq( float acValue ){
    mFreq = Clip(acValue,mFreqMin,mFreqMax);
    mFreqNorm = (mFreq - mFreqMin) / (mFreqMax-mFreqMin);
}

void GenDyn::SetFreqNorm( float acValue ){
    mFreqNorm = Clip(acValue,0.f,1.f);
    mFreq = mFreqMin + acValue * (mFreqMax-mFreqMin);
}

void GenDyn::SetFreqRange(int acRange){
    mFreqRange = acRange;
    mFreqMin = mFreqRange > 0 ? LFO_FREQ_MIN : FREQ_MIN;
    mFreqMax = mFreqRange > 0 ? LFO_FREQ_MAX : FREQ_MAX;
    mdxmin = mFreqMin / mSR * mNumKP;
    mdxmax = mFreqMax / mSR * mNumKP;
    mFreq = mFreqMin + mFreqNorm * (mFreqMax-mFreqMin);
}

// 3pts interpolator
float GenDyn::LagrangeInterp( float x, float y0, float y1, float y2 )
{
    float lamb1 = (x-1.f) * (x-2.f) * 0.5f;
    float lamb2 = x *( 2.f - x );
    float lamb3 = x * (x-1.f) * 0.5f;
    float y = y0*lamb1 + y1*lamb2 + y2*lamb3;

    return y;
}

// 2pts interpolator
float GenDyn::LinearInterp( float x, float y0, float y1 )
{
    float y = ( ( 1.f - x ) * y0 ) + ( x * y1 );

    return y;
}
