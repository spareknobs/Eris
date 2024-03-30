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


#include "Arduino.h"
#include "Audio.h"
#include "Eris.h"
#include <Wire.h>
#include <SPI.h>
#include <Smoothed.h>

//-------------------------------------------------------------------
// Global Defs
//#define CPU_TEST

// uncommet to enable MIDI capabilities (work in progress!)
//#define ENABLE_MIDI

#define LOOP_TIME 5  // control rate (ms) 
#define KNOB_FILTER_LENGTH 20

#define KS_BIASGAIN 25 // keyswitch to set AR bias gain with Master knob
bool masterKnobSetsBiasGain = true;

//-------------------------------------------------------------------
// Digital Pins
#define SYNC      2
#define RANGE     5
#define RATEMOD   3
#define GEN2OUT   4
#define CUTMOD    9
#define LED1     23
#define LED2     22

//-------------------------------------------------------------------
// DSP Modules
Eris   module;
AudioOutputI2S           output;
AudioConnection          patchCord1( module, 0, output, 0 );
AudioConnection          patchCord2( module, 1, output, 1 );

//-------------------------------------------------------------------
// Multiplexer 
static const int cSIG_pin = A0;
static const int cControlPin[] = {12,11,10}; // s0,s1,s2 
static const int cNumMuxChannels = 8;

static const int cMuxChannel[cNumMuxChannels][3]={
    {0,0,0}, //channel 0
    {1,0,0}, //channel 1
    {0,1,0}, //channel 2
    {1,1,0}, //channel 3
    {0,0,1}, //channel 4
    {1,0,1}, //channel 5
    {0,1,1}, //channel 6
    {1,1,1}, //channel 7
  };

int readMux( int channel ){
  for( int i = 0; i < 3; i++ ){
    digitalWrite( cControlPin[i], cMuxChannel[channel][i]);
  }

  delayMicroseconds(200);
  int val = analogRead(cSIG_pin);

  return val;
}

//-------------------------------------------------------------------
// KNOBS MAPPING 
// note: A8 is noisy - removed 
enum AnalogMap
{
    CUT=0,   // Mux A0 
    GAIN1,   // Mux A1 
    RATE1,   // Mux A2 
    RES,     // Mux A3 
    DIST1,   // Mux A4 
    RATE2,   // Mux A5 
    SCALE1,  // Mux A6 
    PAR1,    // Mux A7 
    
    GAIN2,    // Teensy A1 
    PAR2,     // Teensy A2 
    SCALE2,   // Teensy A3 
    DIST2,    // Teensy A4
    MASTER,   // Teensy A5

    cNumKnobs
};

Smoothed <float> knob[cNumKnobs]; 

//--------------------------------------------------------------------
void ReadKnobs(){

      // Read from Mux (MA0..MA7)
      for (int i = 0; i < 8; i++){
          knob[i].add( readMux(i) );
      }

      // Read Teensy Analogs (A1...A5)
      knob[GAIN2].add( analogRead(A1) );   
      knob[PAR2].add( analogRead(A2) );
      knob[SCALE2].add( analogRead(A3) );
      knob[DIST2].add( analogRead(A4) );
      knob[MASTER].add( analogRead(A5) );

      // Filter & Dispatch
     for (int k=0; k < cNumKnobs; k++ ){

        float val = knob[k].get();
        float vval = (float)val / 1023.f;

        switch (k)
        {
            case CUT:
                vval *= vval;
                vval = map( vval, 0.f, 1.f, 100.f, 10000.f );
                module.SetCutoff(vval);
            break;

            case RES:
                vval = map( vval, 0.f, 1.f, 0.7, 5.0 );
                module.SetResonance(vval);
            break;
            
            case GAIN2:
                vval *= vval;
                module.SetGen2Gain( vval * MAX_OSC_GAIN );
            break;
            
            case MASTER:
                // This master knob sets the VCA Bias Gain - bypassed when Receiving MIDI
                vval *= vval;
                module.SetVCABiasGain(vval);
                //module.SetMasterGain( vval );
            break;
            
            case SCALE2:
                module.SetGen2Scale(vval);
            break;
            
            case PAR1:
                module.SetGen1Param(vval);
            break;
            
            case SCALE1:
                module.SetGen1Scale(vval);
            break;
            
            case PAR2:
                module.SetGen2Param(vval);
            break;

            case GAIN1:
                vval *= vval;
                module.SetGen1Gain( vval * MAX_OSC_GAIN );
            break;
            
            case DIST2:
                module.SetGen2Dist(vval);
            break;            
            
            case DIST1:
                module.SetGen1Dist(vval);
            break;

            case RATE2:
                vval *= vval;
                module.SetGen2Rate(vval);
            break;

            case RATE1:
                vval *= vval;
                module.SetGen1Rate(vval);
            break;

            default:
            break;
        } 
    }
}

void ReadSwitches(){
    module.SetRateMod( (bool)!digitalRead(RATEMOD) );
    module.SetCutoffMod( (bool)!digitalRead(CUTMOD) );
    module.SetGen2ToOut( (bool)!digitalRead(GEN2OUT) );
    module.SetGen2Range( !digitalRead(RANGE) ); // note: this up
    module.SyncGens( (bool)!digitalRead(SYNC) );
}

void ControlLed(){
    float val = module.GetGen2Val();
    int intensity = map( val, 0.0, MAX_OSC_GAIN, 0, 1023 );
    analogWrite(LED2,intensity);
    val = module.GetGen1Val();
    intensity = map( val, 0.0, MAX_OSC_GAIN, 0, 1023 );
    analogWrite(LED1,intensity);
}

//--------------------------------------------------------------------
// MIDI
byte mCurNote = 0;
byte mCurVelocity = 0;
static const byte gcKeysBufferSize = 32; // keys buffer
static byte mKeysBuffer[gcKeysBufferSize];
static byte mBufSize = 0;

void ManageKey( byte note, bool playNote ) {
  if ( playNote == true && ( mBufSize < gcKeysBufferSize ) ) {
    module.TriggerMidiNote(note,mCurVelocity);
    mKeysBuffer[mBufSize] = note;
    mBufSize++;
    return;
  }
  else if ( playNote == false ) {
    if ( mBufSize == 0 ){
      module.TriggerRelease();
      return;
    }
    for ( byte found = 0; found < mBufSize; found++ ) {
      if (mKeysBuffer[found] == note) {
        // shift back the notes in the queue
        for (byte gap = found; gap < (mBufSize - 1); gap++) {
          mKeysBuffer[gap] = mKeysBuffer[gap + 1];
        }
        mBufSize--;
        mKeysBuffer[mBufSize] = 255; // set to invalid
        
        if ( mBufSize != 0 ) {
          // retrigger prev key in buffer
          module.TriggerMidiNote( mKeysBuffer[mBufSize - 1], mCurVelocity );
          return;
        }
        else {
          module.TriggerRelease();
          return;
        }
      }
    }
  }
}

// some extra controls available via CCs
void ControlChange(byte channel, byte cc, byte val) {
  //Serial.println(cc);
  //Serial.println(val);
  switch (cc)
  {
    case 74:
      module.SetAttackMs(val * DIV127 * 3000.f);
      break;
    case 71:
      float ms = (float)val * DIV127 * 3000.f;
      module.SetReleaseMs(ms);
      break;
    case 73:
      //module.SetVCABiasGain( (float)val * DIV127);
      break;
    
    default:
      break;
  }
}

void NoteOn(byte channel, byte note, byte velocity) {
  // :TODO: MIDI activity disables Bias Gain Knob
  //masterKnobSetsBiasGain = false;
  mCurNote = note;
  mCurVelocity = velocity;
  ManageKey(note, true);
}

void NoteOff(byte channel, byte note, byte velocity){
  if ( note == KS_BIASGAIN) {
    masterKnobSetsBiasGain = false;
    return;
  }
  ManageKey(note, false);
}

//-------------------------------------------------------------------
void setup(){

    AudioMemory(NUM_MEMORY_BLOCKS);
    //AudioNoInterrupts();

    // Mux INIT
    pinMode(cControlPin[0], OUTPUT);
    pinMode(cControlPin[1], OUTPUT); 
    pinMode(cControlPin[2], OUTPUT); 

    digitalWrite(cControlPin[0], LOW);
    digitalWrite(cControlPin[1], LOW);
    digitalWrite(cControlPin[2], LOW);

    pinMode(RATEMOD, INPUT_PULLUP);
    pinMode(GEN2OUT, INPUT_PULLUP); 
    pinMode(CUTMOD, INPUT_PULLUP);
    pinMode(RANGE, INPUT_PULLUP);
    pinMode(SYNC, INPUT_PULLUP);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

#ifdef CPU_TEST
    AudioProcessorUsageMaxReset();
    AudioMemoryUsageMaxReset();
#endif

    // init knobs
    for (int k=0; k < cNumKnobs; k++ ){
        knob[k].begin(SMOOTHED_AVERAGE, KNOB_FILTER_LENGTH);
    }

    // Init GenDyns using status of the synth knobs
    float vpSeeds[NUM_CONTROL_PTS_MAX];
    int navailseeds = min(NUM_CONTROL_PTS_MAX,cNumKnobs);

    for ( int i=0; i < navailseeds; i++ ) {
        //vpSeeds[i] = 2.f * (float)knob[i].get() / 1023.f - 1.f;
        vpSeeds[i] = 0.f;
    }
    if ( navailseeds < NUM_CONTROL_PTS_MAX){
        for ( int i=navailseeds; i < NUM_CONTROL_PTS_MAX; i++ ) {
            //float vval = 2.f * (float)mKnobValues[leftovers--] / 1023.f - 1.f;
            //vpSeeds[i] = vval;
            vpSeeds[i] = 0.f;
        }
    }

    module.Init(vpSeeds);

    // init Params
    module.SetGen1Rate(0.15f);
    module.SetGen1Dist(1);
    module.SetGen1Gain( 0.5f );
    module.SetGen1Scale(0.3f);
    module.SetGen1Param(0.4f);
    module.SetGen2Rate(0.1f);
    module.SetGen2Dist(1);
    module.SetGen2Gain( 0.5f );
    module.SetGen2Scale(0.6f);
    module.SetGen2Param(0.4f);
    module.SetCutoff(8000.f);
    module.SetResonance(1.f);
    module.SetVCABiasGain(0.f);
    module.SetMasterGain(1.f);
    module.SetRateMod(0);
    module.SetCutoffMod(0);
    module.SetGen2ToOut(1);
    module.SetGen2Range(0);
    module.SyncGens(0);

    // init MIDI
    #ifdef ENABLE_MIDI
    usbMIDI.setHandleNoteOff(NoteOff);
    usbMIDI.setHandleNoteOn(NoteOn); 
    usbMIDI.setHandleControlChange(ControlChange);
    #endif

    //Serial.begin(9600);
    //AudioInterrupts();
}

//-------------------------------------------------------------------
void loop(){

#ifdef ENABLE_MIDI
    usbMIDI.read();
#endif

    ReadKnobs();
    ReadSwitches();
    ControlLed();

#ifdef CPU_TEST
    Serial.print("CPU CURRENT: "); Serial.print(AudioProcessorUsage());
    Serial.print(" CPU MAX: "); Serial.println(AudioProcessorUsageMax());
#endif

    delay(LOOP_TIME);
}

