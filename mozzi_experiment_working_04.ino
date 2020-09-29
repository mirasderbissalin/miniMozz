
#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/cos1024_int8.h>

#include <LowPassFilter.h>
#include <ADSR.h>
#include <IntMap.h>
#include <EventDelay.h>

#include <tables/sin512_int8.h>
#include <tables/saw_analogue512_int8.h>
#include <tables/square_analogue512_int8.h>
#include <tables/saw512_int8.h>

#include <tables/sin1024_int8.h>
#include <tables/saw1024_int8.h>
#include <tables/triangle1024_int8.h>

#include <tables/phasor256_int8.h>
#include <tables/uphasor256_uint8.h>
#include <tables/waveshape_chebyshev_3rd_256_int8.h>


#include <tables/sin2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <tables/triangle_hermes_2048_int8.h>
#include <tables/triangle_dist_cubed_2048_int8.h>
#include <tables/triangle_dist_squared_2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <512, AUDIO_RATE> aOscil;
Oscil <1024, AUDIO_RATE> bOscil;
Oscil <1024, AUDIO_RATE> cOscil;

Oscil<COS1024_NUM_CELLS, AUDIO_RATE> aVibrato(COS1024_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> kTremelo(SIN2048_DATA);

EventDelay FilterSweep;
EventDelay ADSRk;
EventDelay ADSRk2;
EventDelay Arpeg;
// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable
int gain = 255;
int gain2 = 255;
int gain3 = 255;

word vol1;
word vol2;
word vol3;

int place;

word volSus1;
int preADSR;

word bandRange = 100;
int bendCents = 0;

int bendFreq = 0;
int bendFreq2 = 0;
int bendFreq3 = 0;
int zeroSum = 0;

int detune2cents;
int detune3cents;
int detune2HZ;
int detune3HZ;
int subPos = 0;

/*int detuneGlobalCents;
  int detuneGlobalHZ;
  int detuneGlobalHZ2;
  int detuneGlobalHZ3;*/

byte intensity;
int tremolo;
int tremoloGain = 127;

int lpf_mode = 0;
int lpf_speed;
int cutOffFreq;
int Reson;
int LPFdirection = 1;
int LPFaccel;

byte Apot = 8;
byte Dpot = 9;
byte Spot = 10;
byte Rpot = 11;

byte LPSpot = 13;
byte LPFpot = 12;
byte LPRpot = 0;

byte ODpot = 1;
byte BRpot = 2;

byte V1pot = 5;
byte V2pot = 6;
byte V3pot = 7;

byte Det2pot = 3;
byte Det3pot = 4;

int regist = 0;
int regist1 = 0;
int regist2 = 0;
int regist3 = 0;

int osc1 = 0;
int osc2 = 0;
int osc3 = 0;

float A_1 = 55.00;
int notes[73];
bool butState [25];
bool butStateWas [25];
bool ArpegStop = 1;
int ArpegState = 0;
bool ArpegDir = 0;

int playedNote[6] = {0, 0, 0, 0, 0, 0};
int playedNoteIndex;
int playedNotePre[6] = {0, 0, 0, 0, 0, 0};
int butNow[6] = {0, 0, 0, 0, 0, 0};
int butWas[6] = {0, 0, 0, 0, 0, 0};

bool noteState [6] = {0, 0, 0, 0, 0, 0,};
bool newNote [6] = {0, 0, 0, 0, 0, 0,};
bool triggerOff[6] = {0, 0, 0, 0, 0, 0,};
bool triggerOn[6] = {0, 0, 0, 0, 0, 0,};

int presustain = 0;

int freq [6];

LowPassFilter lpf;
IntMap kMapS(0, 1023, 0, 255);
IntMap kMapQ(0, 32385, 0, 127);
IntMap kMapVibr(600, 1023, 0, 12);
IntMap kMapTrem(550, 0, 0, 12);
IntMap kMapDetune(0, 1023, -100, 100);
IntMap kMapVolPre(0, 1023, 0, 127);
IntMap kMapVol(0, 32385, 0, 255);
IntMap kMaptremConv(-127, 127, 0, 127);
IntMap kMapBandRange(0, 800, 1, 12);
IntMap kMapBendUp(508 + 10, 1023, 0, 101);
IntMap kMapBendDown(508 - 10, 0, 0, -100);
IntMap kMapLPFspeed(0 , 1023, 12, 0);
IntMap kMapLPFspeed2(0 , 1023, 50, 1);
IntMap kMapHzBytes(0 , 8191, 0, 255);
IntMap kMapArpeg (0, 1023, 500, 20);

ADSR <CONTROL_RATE, CONTROL_RATE> envelope;

void setTables() {
  if (osc1 == 0) {
    aOscil.setTable(SAW_ANALOGUE512_DATA);
  }
  else if (osc1 == 1) {
    aOscil.setTable(SIN512_DATA);
  }
  else {
    aOscil.setTable(SQUARE_ANALOGUE512_DATA);
  }

  if (osc2 == 0) {
    bOscil.setTable(SAW1024_DATA);
  }
  else if (osc2 == 1) {
    bOscil.setTable(SIN1024_DATA);
  }
  else {
    bOscil.setTable(TRIANGLE1024_DATA);
  }

  if (osc3 == 0) {
    cOscil.setTable(SAW1024_DATA);
  }
  else if (osc3 == 1) {
    cOscil.setTable(SIN1024_DATA);
  }
  else {
    cOscil.setTable(TRIANGLE1024_DATA);
  }
}


void setup() {
  Serial.begin (115200);
  startMozzi(CONTROL_RATE); // :)
  aOscil.setFreq(220); // set the frequency
  aVibrato.setFreq(2);
  envelope.setAttackLevel(127);
  for (int i = 3; i <= 73 + 3; i++) {
    notes[i - 3] = A_1 * pow(2 , (i / 12.00));
    //Serial.print(i - 3);
    //Serial.print("   ");
    //Serial.println(notes[i - 3]);
  }
  for (int x = 22; x <= 46; x++) {
    pinMode (x, OUTPUT);
    digitalWrite (x, HIGH);
  }
  for (int y = 10; y >= 2; y--) {
    pinMode (y, OUTPUT);
    digitalWrite (y, HIGH);
  }
  for (int z = 14; z <= 20; z++) {
    pinMode (z, OUTPUT);
    digitalWrite (z, HIGH);
  }
}

void updateControl() {
  noteState [0] = noteState [1] = noteState [2] = noteState [3] = noteState [4] = noteState [5] = 0;
  collectingNotes();
  adsrs();
  collectingRegisters();
  setTables();
  collectingAnalogs();

  //Serial.println((mozziAnalogRead (ODpot)));
  //Serial.println(kMapArpeg(mozziAnalogRead (ODpot)));
  if ((mozziAnalogRead (ODpot)) < 10) {
    playedNoteIndex = 0;
  }
  else {
    Serial.println (zeroSum);
    if (ArpegStop == 1 && zeroSum == 0) {
      envelope.noteOff();
      ArpegStop = 0;
      //Serial.println ("arp off");
    }
    else if (zeroSum != 0) {
      ArpegStop = 1;
      Arpeg.set(kMapArpeg(mozziAnalogRead (ODpot)));

      if (Arpeg.ready()) {

        //Serial.println("arpeg");
        envelope.noteOn();
        if (lpf_mode == -1) {
          cutOffFreq = kMapHzBytes(freq[1]) + kMapS(mozziAnalogRead(LPFpot));
        }
        if (cutOffFreq > 255) {
          cutOffFreq = 255;
        }
        Arpeg.start();
        if (ArpegDir == 0) {
          //Serial.println("D......................");
          playedNoteIndex++;
          if (playedNoteIndex >= subPos) {
            playedNoteIndex = subPos - 2;
            
            if (subPos < 1){
              
            playedNoteIndex = 0;
            }
            ArpegDir = 1;
          }
        }
        else if (ArpegDir == 1) {
          //Serial.println("U...............");
          playedNoteIndex--;
          if (playedNoteIndex < 0) {
            playedNoteIndex = 1;
            if (subPos < 1){
              
            playedNoteIndex = 0;
            }
            ArpegDir = 0;
          }
        }
        Serial.print("        ");Serial.println(playedNoteIndex);
      }
    }
  }

  if (playedNoteIndex <= 0) {
    playedNoteIndex = 0;
  }
  //Serial.println(playedNoteIndex);
  place = playedNote[playedNoteIndex] + regist * 12 + regist1 * 12;
  freq [0] = notes[place];
  //Serial.println(freq [0]);
  place = playedNote[playedNoteIndex] + regist * 12 + regist2 * 12;
  freq [1] = notes[place];
  //Serial.println(freq [1]);
  place = playedNote[playedNoteIndex] + regist * 12 + regist3 * 12;
  freq [2] = notes[place];

  aOscil.setFreq(freq [0] + bendFreq);
  bOscil.setFreq(freq [1] + bendFreq2 + detune2HZ);
  cOscil.setFreq(freq [2] + bendFreq3 + detune3HZ);

  envelope.update();

  preADSR = envelope.next();
  gain = kMapQ(preADSR * vol1);
  gain2 = kMapQ(preADSR * vol2);
  gain3 = kMapQ(preADSR * vol3);

}


int updateAudio() {
  //Q15n16 vibrato = (Q15n16) intensity * aVibrato.next();
  //int asig = lpf.next(((aOscil.phMod(vibrato) * gain) >> 8) + ((bOscil.phMod(vibrato) * gain2) >> 8) + ((cOscil.phMod(vibrato) * gain3) >> 8));
  int asig = lpf.next(((aOscil.next() * gain) >> 8) + ((bOscil.next() * gain2) >> 8) + ((cOscil.next() * gain3) >> 8));
  //int asig = lpf.next(((aOscil.phMod(vibrato) * gain) >> 8) + ((aOscil2.phMod(vibrato) * gain2) >> 8) + ((aOscil3.phMod(vibrato) * gain3) >> 8) + ((aOscil4.phMod(vibrato) * gain4) >> 8) + ((aOscil5.phMod(vibrato) * gain5) >> 8) + ((aOscil6.phMod(vibrato) * gain6) >> 8));
  //int asig = lpf.next( ((aOscil2.phMod(vibrato) * gain2) >> 8));
  return (int) asig;
}

void collectingNotes() {
  subPos = 0;
  zeroSum = 0;
  for (int o = 0; o < 6; o++) {
    butWas [o] = butNow [o];
    butNow [o] = 0;
  }
  for (int k = 22; k <= 46; k++) {


    if (digitalRead (k) == 0) {
      zeroSum++;
      noteState [subPos] = 1;
      playedNote [subPos] = 70 - k;
      butNow [subPos] = k;
      subPos++;
      if (subPos == 6) {
        break;
      }
    }
  }
}


void collectingAnalogs() {

  bandRange = kMapBandRange(mozziAnalogRead(BRpot));

  if (mozziAnalogRead(14) > 508 + 10) {
    bendCents = kMapBendUp (mozziAnalogRead(14)) * bandRange;
  }
  else if (mozziAnalogRead(14) < 508 - 10) {
    bendCents = kMapBendDown (mozziAnalogRead(14)) * bandRange;
  }
  else {
    bendCents = 0;
  }
  bendFreq = (freq [0] * pow (2.00, bendCents / 1200.00)) - freq [0];
  bendFreq2 = (freq [1] * pow (2.00, bendCents / 1200.00)) - freq [1];
  bendFreq3 = (freq [2] * pow (2.00, bendCents / 1200.00)) - freq [2];
  detune2cents = kMapDetune (mozziAnalogRead(Det2pot));
  detune3cents = kMapDetune (mozziAnalogRead(Det3pot));
  detune2HZ = (freq [1] * pow (2.00, detune2cents / 1200.00)) - freq [1];
  detune3HZ = (freq [2] * pow (2.00, detune3cents / 1200.00)) - freq [2];

  //detuneGlobalCents = kMapDetune (mozziAnalogRead(ODpot));;
  /*detuneGlobalHZ = (freq [0] * pow (2.00, detuneGlobalCents / 1200.00)) - freq [0];
    detuneGlobalHZ = (freq [1] * pow (2.00, detuneGlobalCents / 1200.00)) - freq [1];
    detuneGlobalHZ = (freq [2] * pow (2.00, detuneGlobalCents / 1200.00)) - freq [2];
  */

  /*intensity = mozziAnalogRead(15) + 500;
    //int vibrato = mozziAnalogRead(15);
    int vibrato = kMapVibr (mozziAnalogRead(15));
    if (vibrato < 0) {
    vibrato = 0;
    }
    aVibrato.setFreq(vibrato);

    tremolo = kMapTrem(mozziAnalogRead(15));
    if (tremolo < 0) {
    tremolo = 0;
    }
    kTremelo.setFreq(tremolo);
    tremoloGain = kMaptremConv(kTremelo.next());

    if (tremoloGain < 0) {
    tremoloGain = 0;
    }
    if (tremolo == 0) {
    tremoloGain = 127;
    }*/

  vol1 = kMapVol (tremoloGain * kMapVolPre (mozziAnalogRead(V1pot)));
  vol2 = kMapVol (tremoloGain * kMapVolPre (mozziAnalogRead(V2pot)));
  vol3 = kMapVol (tremoloGain * kMapVolPre (mozziAnalogRead(V3pot)));

  if (lpf_mode == 1) {
    FilterSweep.set(kMapLPFspeed(mozziAnalogRead(LPSpot)));
    if (kMapLPFspeed(mozziAnalogRead(LPSpot)) < 4) {
      LPFaccel = 4 - kMapLPFspeed(mozziAnalogRead(LPSpot));
    }
    else {
      LPFaccel = 0;
    }
    if (FilterSweep.ready()) {
      if (LPFdirection == 1) {
        cutOffFreq++;
        if (LPFaccel > 0) {
          cutOffFreq = cutOffFreq + LPFaccel;
        }
      }
      else {
        cutOffFreq--;
        if (LPFaccel > 0) {
          cutOffFreq = cutOffFreq - LPFaccel;
        }
      }
      if (cutOffFreq > 255) {
        cutOffFreq = 255;
        LPFdirection = -1;
      }
      if (cutOffFreq < kMapS(mozziAnalogRead(LPFpot))) {
        cutOffFreq = kMapS(mozziAnalogRead(LPFpot));
        LPFdirection = 1;
      }
      FilterSweep.start();
    }
  }
  else if (lpf_mode == -1) {
    FilterSweep.set(kMapLPFspeed2(mozziAnalogRead(LPSpot)));
    if (FilterSweep.ready()) {
      cutOffFreq = cutOffFreq + 10;
      FilterSweep.start();
    }
    if (cutOffFreq > 255) {
      cutOffFreq = 255;
    }
  }
  else {
    cutOffFreq = kMapS (mozziAnalogRead(LPFpot));
  }

  Reson = kMapS (mozziAnalogRead(LPRpot));
  lpf.setCutoffFreqAndResonance(cutOffFreq, Reson);

  volSus1 = kMapVolPre (mozziAnalogRead(Spot));

  ADSRk.set(1000);
  ADSRk2.set(1100);
  if (ADSRk.ready()) {
    Serial.println(mozziAnalogRead(Apot));
    Serial.println(mozziAnalogRead(Spot));
    ADSRk.start();
  }
  if (ADSRk2.ready()) {
    Serial.println(mozziAnalogRead(Dpot));
    Serial.println(mozziAnalogRead(Rpot));
    ADSRk2.start();
  }

  envelope.setDecayLevel(volSus1);
  //envelope.setSustainLevel(volSus1);
  //envelope.setReleaseLevel(volSus1);
  envelope.setTimes(mozziAnalogRead(Apot), mozziAnalogRead(Dpot), 65535, mozziAnalogRead(Rpot));

}

void adsrs() {

  if (butWas [0] != butNow [0] && !digitalRead (butNow [0])) {
    envelope.noteOn();
    if (lpf_mode == -1) {
      cutOffFreq = kMapHzBytes(freq[1]) + kMapS(mozziAnalogRead(LPFpot));
      if (cutOffFreq > 255) {
        cutOffFreq = 255;
      }
    }
    Serial.println("1 - on");
  }
  else if (butWas [0] != butNow [0]) {
    envelope.noteOff();
    Serial.println("1 - off");
  }

  for (int i = 0; i < 6; i++) {
    playedNotePre[i] = playedNote[i];
  }
}

void collectingRegisters() {

  regist = !digitalRead (2) - !digitalRead (14);
  regist1 = !digitalRead (18) - !digitalRead (17);
  regist2 = !digitalRead (4) - !digitalRead (3);
  regist3 = !digitalRead (7) - !digitalRead (8);

  lpf_mode = !digitalRead (20) - !digitalRead (19);
  osc1 = !digitalRead (16) - !digitalRead (15);
  osc2 = !digitalRead (5) - !digitalRead (6);
  osc3 = !digitalRead (9) - !digitalRead (10);

}

void loop() {
  audioHook();
}
