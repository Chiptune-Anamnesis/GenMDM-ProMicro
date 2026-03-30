// GenMDM Pro Micro - Arduino Pro Micro (5V/16MHz ATmega32U4) port of GenMDM
// Direct 5V connection to Genesis controller port - no level shifter needed.
//
// Pro Micro Pin -> Genesis DE-9 Controller Port (direct wire)
// A0 (18) -> Pin 1 (Up)    = Data bit 0, register bit 0
// A1 (19) -> Pin 2 (Down)  = Data bit 1, register bit 1
// A2 (20) -> Pin 3 (Left)  = Data bit 2, register bit 2
// A3 (21) -> Pin 4 (Right) = Data bit 3, register bit 3
// 4       -> Pin 6 (B/TL)  = NB (nibble select), register bit 4
// 5       -> Pin 9 (C/TR)  = AD (addr/data phase), register bit 5
// 6       -> Pin 7 (TH)    = WR (write strobe), register bit 6
// GND     -> Pin 8 (GND)
// (Pin 5 = +5V on Genesis - leave unconnected)

#include <Arduino.h>
#include <MIDIUSB.h>
#include <MIDI.h>
#include <math.h>
#include <avr/pgmspace.h>
#include "samples_hex.h"

// Serial MIDI on Pin 0 (Serial1 RX) at 31250 baud - TRS input via 6N137
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, serialMIDI);

// ==================== Pin Definitions ====================
static const uint8_t PIN_D0 = A0;  // Data bit 0 -> Genesis Pin 1
static const uint8_t PIN_D1 = A1;  // Data bit 1 -> Genesis Pin 2
static const uint8_t PIN_D2 = A2;  // Data bit 2 -> Genesis Pin 3
static const uint8_t PIN_D3 = A3;  // Data bit 3 -> Genesis Pin 4
static const uint8_t PIN_NB = 4;   // Nibble select -> Genesis Pin 6
static const uint8_t PIN_AD = 5;   // Addr/Data phase -> Genesis Pin 9
static const uint8_t PIN_WR = 6;   // Write strobe -> Genesis Pin 7

static const int dT = 20; // delay time in microseconds

// ==================== GPIO Helpers (direct drive, no inversion) ====================
inline void outputNibble(uint8_t n) {
  digitalWrite(PIN_D0, (n >> 0) & 1);
  digitalWrite(PIN_D1, (n >> 1) & 1);
  digitalWrite(PIN_D2, (n >> 2) & 1);
  digitalWrite(PIN_D3, (n >> 3) & 1);
}

inline void setNB(bool high) { digitalWrite(PIN_NB, high ? HIGH : LOW); }
inline void setAD(bool high) { digitalWrite(PIN_AD, high ? HIGH : LOW); }
inline void setWR(bool high) { digitalWrite(PIN_WR, high ? HIGH : LOW); }

inline void strobeWR() {
  setWR(true);
  delayMicroseconds(dT);
  setWR(false);
  delayMicroseconds(dT);
}

// ==================== Protocol Variables ====================
int bendAmount = 2;
byte page_number = 0;

// MIDI Values
byte velocity;
byte ccvalue2;
byte bendLSB;
byte bendMSB;

// Sample playback
int SPB_flag = 0;
int SPB_sound = 0;
int SPB_counter = 0;
int SPB_speed = 0;
int SPB_tick = 1;
int sample_on = 0;
byte overSamp = 1;
int SPB_max = 400;
byte save_speed = 1;
byte noise_flag = 0;
byte noise_data = 0;
byte noise_velocity = 0;
byte tri_flag = 1;

byte triangle[] = {
  0b00000001, 0b00000010, 0b00000100, 0b00001000,
  0b00010000, 0b00100000, 0b01000000, 0b10000000,
  0b01000000, 0b00100000, 0b00010000, 0b00001000,
  0b00000100, 0b00000010,
};

// Polyphonic Handler
byte polyFlag = 0;
byte polyBusy[6];
byte pitchTracking[6];

// Pitch Values
byte octDiv = 12;
byte pitchOffset = 64;
int pitchInt;
double pitchDouble;
double constantDouble = 6.711;
byte reg22 = 0;

byte bend[] = { 64, 64, 64, 64, 64, 64 };

// ==================== Register Shadow Arrays ====================
byte regB0[] = { 0, 0, 0, 0, 0, 0 };
byte regB4[] = { 0, 0, 0, 0, 0, 0 };
byte reg30[] = { 0, 0, 0, 0, 0, 0 };
byte reg34[] = { 0, 0, 0, 0, 0, 0 };
byte reg38[] = { 0, 0, 0, 0, 0, 0 };
byte reg3c[] = { 0, 0, 0, 0, 0, 0 };
byte reg50[] = { 0, 0, 0, 0, 0, 0 };
byte reg54[] = { 0, 0, 0, 0, 0, 0 };
byte reg58[] = { 0, 0, 0, 0, 0, 0 };
byte reg5c[] = { 0, 0, 0, 0, 0, 0 };
byte reg60[] = { 0, 0, 0, 0, 0, 0 };
byte reg64[] = { 0, 0, 0, 0, 0, 0 };
byte reg68[] = { 0, 0, 0, 0, 0, 0 };
byte reg6c[] = { 0, 0, 0, 0, 0, 0 };
byte reg80[] = { 0, 0, 0, 0, 0, 0 };
byte reg84[] = { 0, 0, 0, 0, 0, 0 };
byte reg88[] = { 0, 0, 0, 0, 0, 0 };
byte reg8c[] = { 0, 0, 0, 0, 0, 0 };

byte TL1[] = { 127, 127, 127, 127, 127, 127 };
byte TL2[] = { 127, 127, 127, 127, 127, 127 };
byte TL3[] = { 127, 127, 127, 127, 127, 127 };
byte TL4[] = { 127, 127, 127, 127, 127, 127 };

// ==================== FM Presets (16) ====================
const byte ALGO[16] PROGMEM =      { 37, 62, 127, 127, 35, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte FB[16] PROGMEM =        { 0, 65, 0, 0, 0, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte TLOP1[16] PROGMEM =     { 127, 100, 100, 120, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte TLOP2[16] PROGMEM =     { 127, 103, 103, 113, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte TLOP3[16] PROGMEM =     { 127, 108, 108, 107, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte TLOP4[16] PROGMEM =     { 127, 127, 127, 127, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte MULOP1[16] PROGMEM =    { 0, 0, 25, 29, 0, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte MULOP2[16] PROGMEM =    { 0, 0, 18, 55, 0, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte MULOP3[16] PROGMEM =    { 30, 10, 8, 36, 0, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte MULOP4[16] PROGMEM =    { 0, 0, 0, 0, 0, 0, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DETUNEOP1[16] PROGMEM = { 0, 0, 0, 0, 0, 27, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DETUNEOP2[16] PROGMEM = { 0, 0, 0, 0, 0, 9, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DETUNEOP3[16] PROGMEM = { 0, 0, 0, 0, 0, 18, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DETUNEOP4[16] PROGMEM = { 0, 0, 0, 0, 0, 70, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte ATTACKOP1[16] PROGMEM = { 127, 127, 127, 127, 122, 54, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte ATTACKOP2[16] PROGMEM = { 127, 127, 127, 127, 123, 49, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte ATTACKOP3[16] PROGMEM = { 127, 127, 127, 127, 120, 41, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte ATTACKOP4[16] PROGMEM = { 127, 127, 127, 127, 118, 89, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY1OP1[16] PROGMEM = { 0, 52, 0, 60, 58, 30, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY1OP2[16] PROGMEM = { 0, 0, 0, 60, 127, 30, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY1OP3[16] PROGMEM = { 0, 0, 0, 60, 122, 30, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY1OP4[16] PROGMEM = { 0, 0, 0, 60, 75, 30, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY2OP1[16] PROGMEM = { 127, 127, 127, 127, 87, 73, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY2OP2[16] PROGMEM = { 127, 127, 127, 127, 68, 51, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY2OP3[16] PROGMEM = { 127, 127, 127, 127, 88, 43, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte DECAY2OP4[16] PROGMEM = { 127, 127, 127, 127, 61, 65, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte AMP2OP1[16] PROGMEM =   { 127, 127, 127, 127, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte AMP2OP2[16] PROGMEM =   { 127, 127, 127, 127, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte AMP2OP3[16] PROGMEM =   { 127, 127, 127, 127, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte AMP2OP4[16] PROGMEM =   { 127, 127, 127, 127, 127, 127, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte RELOP1[16] PROGMEM =    { 127, 127, 60, 127, 127, 70, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte RELOP2[16] PROGMEM =    { 127, 127, 60, 127, 127, 70, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte RELOP3[16] PROGMEM =    { 127, 127, 60, 127, 127, 70, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };
const byte RELOP4[16] PROGMEM =    { 127, 127, 60, 127, 127, 70, 96, 112, 0, 16, 32, 48, 64, 80, 96, 112 };

// ==================== SN76489 Data ====================
byte working_byte = 0;
int pdatInt = 0;
byte pdat1 = 0;
byte pdat2 = 0;

const int pitchTable[] PROGMEM = {
  1008, 951, 898, 847, 800, 755, 713, 673, 635, 599, 566, 534, 504,
  475, 449, 424, 400, 378, 356, 336, 317, 300, 283, 267, 252, 238,
  224, 212, 200, 189, 178, 168, 159, 150, 141, 133, 126, 119, 112,
  106, 100, 94, 89, 84, 79, 75, 71, 67, 63, 59, 56, 53, 50, 47, 45,
  42, 40, 37, 35, 33, 31, 30, 28, 26, 25, 24, 22, 21, 20, 19, 18, 17,
  16, 15, 14, 13, 13, 12, 11, 11, 10, 9, 9,
  1017, 960, 906, 855, 807, 762, 719, 679, 641, 605, 571, 539, 508, 480,
  453, 428, 404, 381, 360, 339, 320, 302, 285, 269, 254, 240, 226, 214,
  202, 190, 180, 170, 160, 151, 143, 135, 127, 120, 113, 107, 101, 95,
  90, 85, 80, 76, 71, 67, 64, 60, 57, 53, 50, 48, 45, 42, 40, 38, 36,
  34, 32, 30, 28, 27, 25, 24, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13,
  13, 12, 11, 11, 10, 9, 9
};

int coarsePitch[] = { 64, 64, 64, 64 };
int pitchTableOffset = 0;

byte bend_data[] = { 64, 64, 64, 64 };
byte bend_MSB[] =  { 64, 64, 64, 64 };
byte pitchData[] = { 0, 0, 0, 0 };
byte velocityData[] = { 0, 0, 0, 0 };

const byte noiseLookup[] PROGMEM = {
  0b11100000, 0b11100000, 0b11100001, 0b11100001,
  0b11100010, 0b11100100, 0b11100101, 0b11100110,
  0b11100110, 0b11100011, 0b11100011, 0b11100111,
};

// ==================== Forward Declarations ====================
void doNote(byte channel, byte pitch, byte velocity);
void doNoteOff(byte channel, byte pitch, byte velocity);
void doSample();
void doCC(byte channel, byte ccnumber, byte ccvalue);
void doBend(byte channel, int bend_usb);
void writeMD(byte page, byte address, byte data);
void writeSN76489(byte data);
void writeAmplitude(byte velocity, byte channel);
void writeFrequency(byte pitch, byte channel);

// ==================== YM2612 Write Protocol ====================
void writeMD(byte page, byte address, byte data) {
  if (page_number != page) {
    page_number = page;
    if (page == 0) {
      outputNibble(0x0D);
      setNB(true); setAD(true);
      strobeWR();
    } else if (page == 1) {
      outputNibble(0x0E);
      setNB(true); setAD(true);
      strobeWR();
    }
  }

  outputNibble(data & 0x0F);
  setNB(false); setAD(false);
  strobeWR();

  outputNibble((data >> 4) & 0x0F);
  setNB(true); setAD(false);
  strobeWR();

  outputNibble(address & 0x0F);
  setNB(false); setAD(true);
  strobeWR();

  outputNibble((address >> 4) & 0x0F);
  setNB(true); setAD(true);
  strobeWR();
}

// ==================== SN76489 Write Protocol ====================
void writeSN76489(byte data) {
  outputNibble(data & 0x0F);
  setNB(false); setAD(false);
  strobeWR();

  outputNibble((data >> 4) & 0x0F);
  setNB(true); setAD(false);
  strobeWR();

  outputNibble(0x0C);
  setNB(true); setAD(true);
  strobeWR();

  delayMicroseconds(dT);
  delayMicroseconds(dT);
}

// ==================== SN76489 Helpers ====================
void writeAmplitude(byte velocity, byte channel) {
  channel = channel << 5;
  velocity = 15 - (velocity >> 3);
  working_byte = 0b10010000 + channel + velocity;
  writeSN76489(working_byte);
}

void writeFrequency(byte pitch, byte channel) {
  int coarsePitchVal = -12 + (coarsePitch[channel] / 5);
  pdatInt = (pgm_read_word(&pitchTable[pitch - 45 + pitchTableOffset + coarsePitchVal])) + (64 - bend_data[channel]);
  if (pdatInt < 0) pdatInt = 0;
  else if (pdatInt > 1023) pdatInt = 1023;
  pdat1 = pdatInt % 16;
  byte ch_shifted = channel << 5;
  working_byte = 0b10000000 + ch_shifted + pdat1;
  writeSN76489(working_byte);
  pdat2 = pdatInt >> 4;
  writeSN76489(pdat2);
}

// ==================== Note On ====================
void doNote(byte channel, byte pitch, byte velocity) {
  channel--;

  // Channel 6 sample mode (when DAC enabled via CC78)
  if (channel == 5 && sample_on == 1) {
    if (pitch >= 60) {
      SPB_sound = pitch % NUM_SAMPLES;
      if (velocity > 0) {
        SPB_speed = save_speed;
        noise_flag = 0;
        SPB_flag = 1;
        SPB_counter = 0;
        SPB_max = (int)pgm_read_word(&sample_length_list[SPB_sound]);
      } else {
        SPB_flag = 0;
      }
    }
    if (pitch < 60) {
      if (velocity > 0) {
        SPB_flag = 1;
        SPB_counter = 0;
        noise_flag = tri_flag;
        noise_velocity = 7 - (velocity >> 4);
        SPB_speed = 59 - pitch;
      } else {
        SPB_flag = 0;
      }
    }
  }

  else if (channel <= 5) {
    if (velocity > 0) {
      if (polyFlag == 1) {
        for (int i = 0; i < 6; i++) {
          if (polyBusy[i] == 0) {
            channel = i;
            polyBusy[i] = 1;
            i = 6;
          }
        }
      }
      pitch = pitch - 64 + pitchOffset;
      pitchTracking[channel] = pitch;
      pitchDouble = pow(2, ((pitch % octDiv) + (0.015625 * bendAmount * (bend[channel] - 64)) + constantDouble) / octDiv) * 440;
      pitchInt = (int)pitchDouble;
      pitchInt = ((pitch / octDiv) << 11) | pitchInt;

      writeMD(channel / 3, 0xa4 + (channel % 3), pitchInt >> 8);
      writeMD(channel / 3, 0xa0 + (channel % 3), pitchInt % 256);
      writeMD(channel / 3, 0x40 + (channel % 3), 127 - ((velocity * TL1[channel]) / 127));
      writeMD(channel / 3, 0x44 + (channel % 3), 127 - ((velocity * TL2[channel]) / 127));
      writeMD(channel / 3, 0x48 + (channel % 3), 127 - ((velocity * TL3[channel]) / 127));
      writeMD(channel / 3, 0x4C + (channel % 3), 127 - ((velocity * TL4[channel]) / 127));
      writeMD(0, 0x28, 0xf0 | ((channel / 3 << 2) | (channel % 3)));
    } else {
      if (polyFlag == 1) {
        for (int i = 0; i < 6; i++) {
          if (pitchTracking[i] == pitch) {
            channel = i;
            polyBusy[i] = 0;
            i = 6;
          }
        }
      }
      if (pitchTracking[channel] == pitch) {
        writeMD(0, 0x28, ((channel / 3 << 2) | (channel % 3)));
      }
    }
  }

  // SN76489 tone channels (MIDI channels 7-9)
  else if (channel >= 6 && channel <= 8 && pitch >= 45) {
    byte psgCh = channel - 6;
    if (velocity > 0) {
      pitchData[psgCh] = pitch;
      velocityData[psgCh] = velocity;
      writeFrequency(pitch, psgCh);
      writeAmplitude(velocity, psgCh);
    } else {
      velocityData[psgCh] = 0;
      writeAmplitude(0, psgCh);
    }
  }

  // SN76489 noise channel (MIDI channel 10)
  else if (channel == 9) {
    byte psgCh = 3;
    if (velocity > 0) {
      velocityData[psgCh] = velocity;
      writeSN76489(pgm_read_byte(&noiseLookup[pitch % 12]));
      writeAmplitude(velocity, psgCh);
    } else {
      velocityData[psgCh] = 0;
      writeAmplitude(0, psgCh);
    }
  }
}

// ==================== Note Off ====================
void doNoteOff(byte channel, byte pitch, byte velocity) {
  channel--;

  if (channel == 5 && sample_on == 1) {
    SPB_flag = 0;
  }

  if (channel <= 5) {
    if (polyFlag == 1) {
      for (int i = 0; i < 6; i++) {
        if (pitchTracking[i] == pitch) {
          channel = i;
          polyBusy[i] = 0;
          i = 6;
        }
      }
    }
    if (pitchTracking[channel] == pitch) {
      writeMD(0, 0x28, ((channel / 3 << 2) | (channel % 3)));
    }
  }

  else if (channel >= 6 && channel <= 8 && pitch >= 45) {
    byte psgCh = channel - 6;
    velocityData[psgCh] = 0;
    writeAmplitude(0, psgCh);
  }

  else if (channel == 9) {
    velocityData[3] = 0;
    writeAmplitude(0, 3);
  }
}

// ==================== Control Change ====================
void doCC(byte channel, byte ccnumber, byte ccvalue) {
  channel--;

  if (channel <= 5) {
    // Triangle waveform data (CC 100-113)
    if (ccnumber >= 100 && ccnumber <= 113) {
      triangle[ccnumber - 100] = ccvalue << 1;
    }

    switch (ccnumber) {
    case 79: // DAC value
      writeMD(0, 0x2a, ccvalue << 1);
      break;

    case 14: // Algorithm
      regB0[channel] = regB0[channel] | 0b00000111;
      ccvalue = (ccvalue >> 4) | 0b00111000;
      regB0[channel] = regB0[channel] & ccvalue;
      writeMD(channel / 3, 0xB0 + (channel % 3), regB0[channel]);
      break;

    case 15: // Feedback
      regB0[channel] = regB0[channel] | 0b00111000;
      ccvalue2 = ((ccvalue >> 4) << 3) | 0b00000111;
      regB0[channel] = regB0[channel] & ccvalue2;
      writeMD(channel / 3, 0xB0 + (channel % 3), regB0[channel]);
      break;

    case 16: TL1[channel] = ccvalue; writeMD(channel / 3, 0x40 + (channel % 3), 127 - ((velocity * TL1[channel]) / 127)); break;
    case 17: TL2[channel] = ccvalue; writeMD(channel / 3, 0x44 + (channel % 3), 127 - ((velocity * TL2[channel]) / 127)); break;
    case 18: TL3[channel] = ccvalue; writeMD(channel / 3, 0x48 + (channel % 3), 127 - ((velocity * TL3[channel]) / 127)); break;
    case 19: TL4[channel] = ccvalue; writeMD(channel / 3, 0x4C + (channel % 3), 127 - ((velocity * TL4[channel]) / 127)); break;

    case 20: reg30[channel] = (reg30[channel] | 0x0F) & ((ccvalue >> 3) | 0x70); writeMD(channel / 3, 0x30 + (channel % 3), reg30[channel]); break;
    case 21: reg34[channel] = (reg34[channel] | 0x0F) & ((ccvalue >> 3) | 0x70); writeMD(channel / 3, 0x34 + (channel % 3), reg34[channel]); break;
    case 22: reg38[channel] = (reg38[channel] | 0x0F) & ((ccvalue >> 3) | 0x70); writeMD(channel / 3, 0x38 + (channel % 3), reg38[channel]); break;
    case 23: reg3c[channel] = (reg3c[channel] | 0x0F) & ((ccvalue >> 3) | 0x70); writeMD(channel / 3, 0x3c + (channel % 3), reg3c[channel]); break;

    case 24: reg30[channel] = (reg30[channel] | 0x70) & (((ccvalue >> 4) << 4) | 0x0F); writeMD(channel / 3, 0x30 + (channel % 3), reg30[channel]); break;
    case 25: reg34[channel] = (reg34[channel] | 0x70) & (((ccvalue >> 4) << 4) | 0x0F); writeMD(channel / 3, 0x34 + (channel % 3), reg34[channel]); break;
    case 26: reg38[channel] = (reg38[channel] | 0x70) & (((ccvalue >> 4) << 4) | 0x0F); writeMD(channel / 3, 0x38 + (channel % 3), reg38[channel]); break;
    case 27: reg3c[channel] = (reg3c[channel] | 0x70) & (((ccvalue >> 4) << 4) | 0x0F); writeMD(channel / 3, 0x3c + (channel % 3), reg3c[channel]); break;

    case 39: reg50[channel] = (reg50[channel] | 0xC0) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x50 + (channel % 3), reg50[channel]); break;
    case 40: reg54[channel] = (reg54[channel] | 0xC0) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x54 + (channel % 3), reg54[channel]); break;
    case 41: reg58[channel] = (reg58[channel] | 0xC0) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x58 + (channel % 3), reg58[channel]); break;
    case 42: reg5c[channel] = (reg5c[channel] | 0xC0) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x5c + (channel % 3), reg5c[channel]); break;

    case 43: reg50[channel] = (reg50[channel] | 0x1F) & ((ccvalue >> 2) | 0xC0); writeMD(channel / 3, 0x50 + (channel % 3), reg50[channel]); break;
    case 44: reg54[channel] = (reg54[channel] | 0x1F) & ((ccvalue >> 2) | 0xC0); writeMD(channel / 3, 0x54 + (channel % 3), reg54[channel]); break;
    case 45: reg58[channel] = (reg58[channel] | 0x1F) & ((ccvalue >> 2) | 0xC0); writeMD(channel / 3, 0x58 + (channel % 3), reg58[channel]); break;
    case 46: reg5c[channel] = (reg5c[channel] | 0x1F) & ((ccvalue >> 2) | 0xC0); writeMD(channel / 3, 0x5c + (channel % 3), reg5c[channel]); break;

    case 70: reg60[channel] = (reg60[channel] | 0x80) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x60 + (channel % 3), reg60[channel]); break;
    case 71: reg64[channel] = (reg64[channel] | 0x80) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x64 + (channel % 3), reg64[channel]); break;
    case 72: reg68[channel] = (reg68[channel] | 0x80) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x68 + (channel % 3), reg68[channel]); break;
    case 73: reg6c[channel] = (reg6c[channel] | 0x80) & ((ccvalue << 1) | 0x1F); writeMD(channel / 3, 0x6c + (channel % 3), reg6c[channel]); break;

    case 47: reg60[channel] = (reg60[channel] | 0x1F) & ((ccvalue >> 2) | 0x80); writeMD(channel / 3, 0x60 + (channel % 3), reg60[channel]); break;
    case 48: reg64[channel] = (reg64[channel] | 0x1F) & ((ccvalue >> 2) | 0x80); writeMD(channel / 3, 0x64 + (channel % 3), reg64[channel]); break;
    case 49: reg68[channel] = (reg68[channel] | 0x1F) & ((ccvalue >> 2) | 0x80); writeMD(channel / 3, 0x68 + (channel % 3), reg68[channel]); break;
    case 50: reg6c[channel] = (reg6c[channel] | 0x1F) & ((ccvalue >> 2) | 0x80); writeMD(channel / 3, 0x6c + (channel % 3), reg6c[channel]); break;

    case 51: writeMD(channel / 3, 0x70 + (channel % 3), ccvalue >> 3); break;
    case 52: writeMD(channel / 3, 0x74 + (channel % 3), ccvalue >> 3); break;
    case 53: writeMD(channel / 3, 0x78 + (channel % 3), ccvalue >> 3); break;
    case 54: writeMD(channel / 3, 0x7c + (channel % 3), ccvalue >> 3); break;

    case 55: reg80[channel] = (reg80[channel] | 0xF0) & ((ccvalue << 1) | 0x0F); writeMD(channel / 3, 0x80 + (channel % 3), reg80[channel]); break;
    case 56: reg84[channel] = (reg84[channel] | 0xF0) & ((ccvalue << 1) | 0x0F); writeMD(channel / 3, 0x84 + (channel % 3), reg84[channel]); break;
    case 57: reg88[channel] = (reg88[channel] | 0xF0) & ((ccvalue << 1) | 0x0F); writeMD(channel / 3, 0x88 + (channel % 3), reg88[channel]); break;
    case 58: reg8c[channel] = (reg8c[channel] | 0xF0) & ((ccvalue << 1) | 0x0F); writeMD(channel / 3, 0x8c + (channel % 3), reg8c[channel]); break;

    case 59: reg80[channel] = (reg80[channel] | 0x0F) & ((ccvalue >> 3) | 0xF0); writeMD(channel / 3, 0x80 + (channel % 3), reg80[channel]); break;
    case 60: reg84[channel] = (reg84[channel] | 0x0F) & ((ccvalue >> 3) | 0xF0); writeMD(channel / 3, 0x84 + (channel % 3), reg84[channel]); break;
    case 61: reg88[channel] = (reg88[channel] | 0x0F) & ((ccvalue >> 3) | 0xF0); writeMD(channel / 3, 0x88 + (channel % 3), reg88[channel]); break;
    case 62: reg8c[channel] = (reg8c[channel] | 0x0F) & ((ccvalue >> 3) | 0xF0); writeMD(channel / 3, 0x8c + (channel % 3), reg8c[channel]); break;

    case 1: // LFO frequency
      reg22 = (reg22 | 0x07) & ((ccvalue >> 4) | 0x08);
      writeMD(0, 0x22, reg22);
      break;

    case 74: // LFO enable
      reg22 = (reg22 | 0x08) & (((ccvalue >> 6) << 3) | 0x07);
      writeMD(0, 0x22, reg22);
      break;

    case 75: // PMS
      regB4[channel] = (regB4[channel] | 0x07) & ((ccvalue >> 4) | 0xF8);
      writeMD(channel / 3, 0xB4 + (channel % 3), regB4[channel]);
      break;

    case 76: // AMS
      regB4[channel] = (regB4[channel] | 0x30) & (((ccvalue >> 4) << 3) | 0xC7);
      writeMD(channel / 3, 0xB4 + (channel % 3), regB4[channel]);
      break;

    case 77: // L/R output enable
      regB4[channel] = (regB4[channel] | 0xC0) & (((ccvalue >> 5) << 6) | 0x3F);
      writeMD(channel / 3, 0xB4 + (channel % 3), regB4[channel]);
      break;

    case 78: // DAC enable
      writeMD(0, 0x2B, ccvalue << 1);
      sample_on = ccvalue >> 6;
      break;

    case 81: bendAmount = ccvalue / 18; break;

    case 83: // PAL/NTSC tuning
      constantDouble = (ccvalue > 63) ? 6.41 : 6.711;
      break;

    case 84: octDiv = ccvalue + 1; break;
    case 85: pitchOffset = ccvalue; break;

    case 86: SPB_speed = ccvalue; save_speed = SPB_speed; break; // Sample speed
    case 87: polyFlag = ccvalue >> 6; break;
    case 88: overSamp = (ccvalue >> 3) + 1; break; // Oversampling
    case 89: tri_flag = (ccvalue >> 6) + 1; break; // Triangle wave

    case 9: { // Preset select
      byte idx = ccvalue / 8;
      doCC(channel + 1, 14, pgm_read_byte(&ALGO[idx]));
      doCC(channel + 1, 15, pgm_read_byte(&FB[idx]));
      doCC(channel + 1, 16, pgm_read_byte(&TLOP1[idx]));
      doCC(channel + 1, 17, pgm_read_byte(&TLOP2[idx]));
      doCC(channel + 1, 18, pgm_read_byte(&TLOP3[idx]));
      doCC(channel + 1, 19, pgm_read_byte(&TLOP4[idx]));
      doCC(channel + 1, 20, pgm_read_byte(&MULOP1[idx]));
      doCC(channel + 1, 21, pgm_read_byte(&MULOP2[idx]));
      doCC(channel + 1, 22, pgm_read_byte(&MULOP3[idx]));
      doCC(channel + 1, 23, pgm_read_byte(&MULOP4[idx]));
      doCC(channel + 1, 24, pgm_read_byte(&DETUNEOP1[idx]));
      doCC(channel + 1, 25, pgm_read_byte(&DETUNEOP2[idx]));
      doCC(channel + 1, 26, pgm_read_byte(&DETUNEOP3[idx]));
      doCC(channel + 1, 27, pgm_read_byte(&DETUNEOP4[idx]));
      doCC(channel + 1, 43, pgm_read_byte(&ATTACKOP1[idx]));
      doCC(channel + 1, 44, pgm_read_byte(&ATTACKOP2[idx]));
      doCC(channel + 1, 45, pgm_read_byte(&ATTACKOP3[idx]));
      doCC(channel + 1, 46, pgm_read_byte(&ATTACKOP4[idx]));
      doCC(channel + 1, 47, pgm_read_byte(&DECAY1OP1[idx]));
      doCC(channel + 1, 48, pgm_read_byte(&DECAY1OP2[idx]));
      doCC(channel + 1, 49, pgm_read_byte(&DECAY1OP3[idx]));
      doCC(channel + 1, 50, pgm_read_byte(&DECAY1OP4[idx]));
      doCC(channel + 1, 51, pgm_read_byte(&DECAY2OP1[idx]));
      doCC(channel + 1, 52, pgm_read_byte(&DECAY2OP2[idx]));
      doCC(channel + 1, 53, pgm_read_byte(&DECAY2OP3[idx]));
      doCC(channel + 1, 54, pgm_read_byte(&DECAY2OP4[idx]));
      doCC(channel + 1, 55, pgm_read_byte(&AMP2OP1[idx]));
      doCC(channel + 1, 56, pgm_read_byte(&AMP2OP2[idx]));
      doCC(channel + 1, 57, pgm_read_byte(&AMP2OP3[idx]));
      doCC(channel + 1, 58, pgm_read_byte(&AMP2OP4[idx]));
      doCC(channel + 1, 59, pgm_read_byte(&RELOP1[idx]));
      doCC(channel + 1, 60, pgm_read_byte(&RELOP2[idx]));
      doCC(channel + 1, 61, pgm_read_byte(&RELOP3[idx]));
      doCC(channel + 1, 62, pgm_read_byte(&RELOP4[idx]));
      break;
    }
    }
  }

  // SN76489 CC messages (channels 7-9)
  else if (channel >= 6 && channel <= 8) {
    byte psgCh = channel - 6;
    if (ccnumber == 83 && psgCh == 0) {
      pitchTableOffset = (ccvalue > 63) ? 84 : 0;
    }
    else if (ccnumber == 42) {
      coarsePitch[psgCh] = ccvalue;
      writeFrequency(pitchData[psgCh], psgCh);
    }
    else if (ccnumber == 11) {
      velocityData[psgCh] = ccvalue;
      writeAmplitude(ccvalue, psgCh);
    }
  }
}

// ==================== Pitch Bend ====================
void doBend(byte channel, int bend_usb) {
  channel--;
  // MIDIUSB pitch bend is 0-16383 (14-bit), extract MSB
  bendMSB = (bend_usb >> 7) & 0x7F;
  bendLSB = bend_usb & 0x7F;

  if (channel <= 5) {
    bend[channel] = bendMSB;
    pitchDouble = pow(2, ((pitchTracking[channel] % octDiv) + (0.015625 * bendAmount * (bend[channel] - 64)) + constantDouble) / octDiv) * 440;
    pitchInt = (int)pitchDouble;
    pitchInt = ((pitchTracking[channel] / octDiv) << 11) | pitchInt;
    writeMD(channel / 3, 0xa4 + (channel % 3), pitchInt >> 8);
    writeMD(channel / 3, 0xa0 + (channel % 3), pitchInt % 256);
  }

  else if (channel >= 6 && channel <= 8) {
    byte psgCh = channel - 6;
    bend_MSB[psgCh] = bendMSB;
    bend_data[psgCh] = bend_MSB[psgCh];
    writeFrequency(pitchData[psgCh], psgCh);
  }
}

// ==================== Serial MIDI Callbacks (TRS input) ====================
void handleSerialNoteOn(byte ch, byte note, byte vel) {
  if (vel == 0) doNoteOff(ch, note, 0);
  else doNote(ch, note, vel);
}
void handleSerialNoteOff(byte ch, byte note, byte vel) {
  doNoteOff(ch, note, vel);
}
void handleSerialCC(byte ch, byte num, byte val) {
  doCC(ch, num, val);
}
void handleSerialPitchBend(byte ch, int bendVal) {
  // FortySevenEffects sends -8192..+8191, convert to 0..16383
  doBend(ch, bendVal + 8192);
}

// ==================== USB MIDI Polling ====================
void processUsbMidi() {
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header == 0) break;

    byte status = rx.byte1 & 0xF0;
    byte ch = (rx.byte1 & 0x0F) + 1;

    switch (status) {
    case 0x90:
      if (rx.byte3 > 0) doNote(ch, rx.byte2, rx.byte3);
      else doNoteOff(ch, rx.byte2, 0);
      break;
    case 0x80:
      doNoteOff(ch, rx.byte2, rx.byte3);
      break;
    case 0xB0:
      doCC(ch, rx.byte2, rx.byte3);
      break;
    case 0xE0: {
      int bendVal = (rx.byte2 | (rx.byte3 << 7));
      doBend(ch, bendVal);
      break;
    }
    }
  } while (1);
}

// ==================== Sample Playback ====================
void doSample() {
  if (SPB_flag == 1 && sample_on == 1) {
    if (SPB_counter >= SPB_max && noise_flag == 0) {
      SPB_flag = 0;
      SPB_counter = 0;
    }

    if (SPB_tick >= SPB_speed && noise_flag == 1) {
      SPB_counter = SPB_counter + overSamp;
      SPB_tick = 0;
      noise_data = byte(random(256)) >> noise_velocity;
      writeMD(0, 0x2a, noise_data);
    }

    if (SPB_tick >= SPB_speed && noise_flag == 2) {
      SPB_counter = SPB_counter + overSamp;
      SPB_tick = 0;
      noise_data = triangle[SPB_counter % 14] >> noise_velocity;
      writeMD(0, 0x2a, noise_data);
    }

    else if (SPB_tick >= SPB_speed && noise_flag == 0) {
      SPB_tick = 0;
      if (SPB_sound < NUM_SAMPLES) {
        const uint8_t* smp = (const uint8_t*)pgm_read_ptr(&sample_ptrs[SPB_sound]);
        writeMD(0, 0x2a, pgm_read_byte(smp + SPB_counter));
      }
      SPB_counter = SPB_counter + overSamp;
      SPB_tick = 0;
    }

    if (SPB_tick < SPB_speed) {
      SPB_tick = SPB_tick + 1;
    }
  }
}

// ==================== Setup ====================
void setup() {
  pinMode(PIN_D0, OUTPUT);
  pinMode(PIN_D1, OUTPUT);
  pinMode(PIN_D2, OUTPUT);
  pinMode(PIN_D3, OUTPUT);
  pinMode(PIN_NB, OUTPUT);
  pinMode(PIN_AD, OUTPUT);
  pinMode(PIN_WR, OUTPUT);

  // Idle state: WR=LOW (Genesis reads when WR=HIGH)
  setWR(false);
  setNB(false);
  setAD(false);
  outputNibble(0x00);

  // Serial MIDI setup (TRS input on Pin 0 via 6N137)
  serialMIDI.begin(MIDI_CHANNEL_OMNI);
  serialMIDI.setHandleNoteOn(handleSerialNoteOn);
  serialMIDI.setHandleNoteOff(handleSerialNoteOff);
  serialMIDI.setHandleControlChange(handleSerialCC);
  serialMIDI.setHandlePitchBend(handleSerialPitchBend);

  // Wait for cart ROM to finish YM2612 init
  delay(550);

  // Enable L/R output on all 6 FM channels
  doCC(1, 77, 127); delay(1);
  doCC(2, 77, 127); delay(1);
  doCC(3, 77, 127); delay(1);
  doCC(4, 77, 127); delay(1);
  doCC(5, 77, 127); delay(1);
  doCC(6, 77, 127); delay(1);

  // Load initial presets
  doCC(1, 9, 40); delay(1);
  doCC(2, 9, 20); delay(1);
  doCC(3, 9, 10); delay(1);
  doCC(4, 9, 20); delay(1);
  doCC(5, 9, 60); delay(1);
  doCC(6, 9, 20); delay(1);

  // Startup melody (confirms all 6 voices)
  delay(100);
  doNote(1, 60, 50); delay(120);  // C4
  doNote(1, 60, 0);  delay(30);
  doNote(2, 64, 50); delay(120);  // E4
  doNote(2, 64, 0);  delay(30);
  doNote(3, 67, 50); delay(120);  // G4
  doNote(3, 67, 0);  delay(30);
  doNote(4, 72, 50); delay(120);  // C5
  doNote(4, 72, 0);  delay(30);
  doNote(5, 76, 45); delay(120);  // E5
  doNote(5, 76, 0);  delay(30);
  doNote(6, 79, 40); delay(300);  // G5 (linger)
  doNote(6, 79, 0);

  randomSeed(analogRead(A4));
}

// ==================== Main Loop ====================
void loop() {
  doSample();
  serialMIDI.read();  // TRS MIDI input
  processUsbMidi();   // USB MIDI input
}
