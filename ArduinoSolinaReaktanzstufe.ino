/* Arduino Synth from
  https://janostman.wordpress.com/2016/01/15/how-to-build-your-very-own-string-synth/
*/

/* Original code by Jan Ostman,  Modified by Dave Field of Take The Moon
 *  
        https://moroccodave.com/
        https://takethemoon.com/
    
    Modifications are licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. 
    To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
        
    MODS:
    *****

    1. Keyboard port scanning removed - this version is MIDI-only
    2. Potentiometer inputs assigned to lower analog pins for ATMega328 compatibility
    3. DIP switch on digital pins 6, 7, 8, 9 added to allow MIDI channel selection
    4. MIDI handler re-written to use stock MIDI library and to be channel-aware
    5. Any resulting redundant code and variables removed

    Also modified by LAURI'S DIY CHANNEL TV: There's a control for clock rate / pitch and lfo rate


    MODS AND FIXED ISSUES BY REAKTANZSTUFE2000
    0. commented the whole code to make it maintainable
    1. use else if where appropriate
    2. pot readout from lauri fixed 
    3. vca cleaned
    4. dco arrays shortened (8 instead of 16 since unused)
    5. new implementation for note detection retaining the "sticky chord" behaviour
    6. the whole gating behaviour when release is minimal made things quite messy and i have no musical use for it so i removed it
    7. replaced magical voice count number with N_VOICES. won't work above 4 since tone generator interrupt is already quite busy
    8. switch on pop fixed
    9. when turning detune down all to the left the phase relationship between the two osc per voice sometimes is a bit unlucky.
       now all the way left is single osc, together with low sample rate gives a nice bass

    ISSUES TODO
    - output sum > 255 --> distortion?
    - detune without division
    - dco aliasing
    - not happy with the non-existent naming convention
    - lfo and detune are one direction only so they lead to overall detune
    - usable fx parameter range (detune already works quite nice)
    - stereo chorus effect (output ready) might not be possible since tone generation interrupt is already too busy
*/

#include <MIDI.h>
#include <util/atomic.h>

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define N_VOICES 4  // maximum is 4, but can be reduced (for example singl voice for bass)
#define STEREO      // output styles: MONO, STEREO

// prescaler registers
const unsigned char PS_2 = (1 << ADPS0);
const unsigned char PS_4 = (1 << ADPS1);
const unsigned char PS_8 = (1 << ADPS1) | (1 << ADPS0);
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

uint32_t NOTES[12] = { 208065 >> 2, 220472 >> 2, 233516 >> 2, 247514 >> 2, 262149 >> 2, 277738 >> 2,
                       294281 >> 2, 311779 >> 2, 330390 >> 2, 349956 >> 2, 370794 >> 2, 392746 >> 2 };

const uint8_t ATTrates[32] = {
  1, 2, 3, 4, 5, 8, 12, 20, 32, 37, 43, 51, 64, 85, 128, 255, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const uint8_t RELrates[32] = {
  1, 2, 3, 4, 5, 8, 12, 20, 32, 37, 43, 51, 64, 85, 128, 255, 255, 128, 85, 64, 51, 43, 37, 32,
  20, 12, 8, 5, 4, 3, 2, 1
};

const uint8_t sinetable[256] PROGMEM = {
  127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 178,
  181, 184, 187, 190, 192, 195, 198, 200, 203, 205, 208, 210, 212, 215, 217, 219, 221,
  223, 225, 227, 229, 231, 233, 234, 236, 238, 239, 240,
  242, 243, 244, 245, 247, 248, 249, 249, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254,
  254, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 249, 249, 248, 247, 245, 244,
  243, 242, 240, 239, 238, 236, 234, 233, 231, 229, 227, 225, 223,
  221, 219, 217, 215, 212, 210, 208, 205, 203, 200, 198, 195, 192, 190, 187, 184, 181, 178,
  176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127,
  124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78,
  76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21,
  20, 18, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
  2, 2, 3, 4, 5, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31,
  33, 35, 37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 78, 81, 84, 87, 90, 93,
  96, 99, 102, 105, 108, 111, 115, 118, 121, 124
};

const uint8_t crushmask[8] = { 0xff, 0b11111000, 0b11110000, 0b11100000, 0b01111111, 0b00111111, 0b00011111, 0b00011000 };
const uint16_t samplerates[8] = { 758, 1516, 3032, 6064, 12128, 1516, 3032, 6064 };
const uint8_t noteshift[8] = { 0, 1, 2, 3, 4, 0, 0, 0 };


volatile uint8_t LFOPH;
volatile uint8_t LFOPH2;
volatile uint16_t LFOVAL;
volatile uint16_t LFOVAL2;
volatile uint8_t OSCNOTES[N_VOICES];  // Oscillator midi notes 0...127; >127 are used as control messages
int16_t ENVELOPE = 0;

//-------- Synth parameters --------------
uint32_t FREQ[2 * N_VOICES] = { 0 };   // DCO pitch
volatile uint32_t DETUNE = 0;          // Osc spread or detune
volatile uint32_t TUNE_POT = 0;        // Osc fine tune
volatile uint16_t LFOINC = 20;         // Lfo increment 0-255
volatile uint8_t VCA = 255;            // VCA level 0-255
volatile uint8_t ATTACK = 1;           // ENV Attack rate 0-255
volatile uint8_t RELEASE = 1;          // ENV Release rate 0-255
volatile uint8_t ENVSHAPE = 0;         // ENV Shape
volatile uint8_t TRIG = 0;             // Trigger 0 0 0 0 Ch3 Ch2 Ch1 Ch0
volatile int16_t MOD;                  // MODwheel
volatile int8_t CRUSH;                 // crusher setting
volatile int8_t SAMPLE;                // samplerate setting
volatile uint16_t SAMPLERATE_POT = 0;  // samplerate potentiometer
volatile uint16_t SAMPLERATE = 758;    // samplerate
uint32_t DCOPH[2 * N_VOICES];
uint8_t integrators[2 * N_VOICES];
uint8_t delayline[256];
volatile uint8_t writepointer;
volatile uint8_t PHASERMIX;
uint8_t DCO;

/* --------- MIDI ----------- */
/* DIP Switch Setting = Channel (1=ON, 0=OFF)
    0000 = 1   0001 = 2   0010 = 3   0011 = 4
    0100 = 5   0101 = 6   0110 = 7   0111 = 8
    1000 = 9   1001 = 10  1010 = 11  1011 = 12
    1100 = 13  1101 = 14  1110 = 15  1111 = 16

DIP Switch Pins */
#define DIP_SW1 5
#define DIP_SW2 6
#define DIP_SW3 7
#define DIP_SW4 8
MIDI_CREATE_DEFAULT_INSTANCE();
byte MIDI_CHANNEL = 1;


// setup
void setup() {

  //PWM and GATE outputs
  pinMode(11, OUTPUT);           // OCR2A out
  pinMode(3, OUTPUT);            // OCR2B out --> experimental stereo chorus
  pinMode(10, OUTPUT);           // gate
  pinMode(LED_BUILTIN, OUTPUT);  // for debugging purposes

  // Set up Timer 1 to send a sample every interrupt.
  cli();
  // Set CTC mode
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  // No prescaler
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with interrupts disabled to be safe.
  OCR1A = 758;  //F_CPU / SAMPLE_RATE;
  // Enable interrupt when TCNT1 == OCR1A
  TIMSK1 |= _BV(OCIE1A);
  //set timer0 interrupt at 61Hz
  TCCR0A = 0;  // set entire TCCR0A register to 0
  TCCR0B = 0;  // same for TCCR0B
  TCNT0 = 0;   //initialize counter value to 0
  // set compare match register for 62hz increments
  OCR0A = 255;  // = 61Hz
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for prescaler 1024
  TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);  //1024 prescaler
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();
  // Set baud rate to 31,250. Requires modification if clock speed is not 16MHz.
  UBRR0H = ((F_CPU / 16 + 31250 / 2) / 31250 - 1) >> 8;
  UBRR0L = ((F_CPU / 16 + 31250 / 2) / 31250 - 1);
  // Set frame format to 8 data bits, no parity, 1 stop bit
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  // enable rx
  UCSR0B |= _BV(RXEN0);
  // USART RX interrupt enable bit on
  UCSR0B |= _BV(RXCIE0);
  // Set up Timer 2 to do pulse width modulation on the speaker pin.
  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  // Set fast PWM mode (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);
  // Do non-inverting PWM on pin OC2A (p.155)
  // On the Arduino this is pin 11.
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);  // experimental stereo chorus
  //TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0)); // original
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  // Set initial pulse width to the first sample.
  OCR2A = 0;  // must be 0 to avoid pop since tone generation is asymmetrical
  OCR2B = 0;  // experimental stereo chorus
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above. PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_128;
  ADMUX = 69;
  sbi(ADCSRA, ADSC);

  // MIDI channel DIP switches
  pinMode(DIP_SW1, INPUT_PULLUP);
  pinMode(DIP_SW2, INPUT_PULLUP);
  pinMode(DIP_SW3, INPUT_PULLUP);
  pinMode(DIP_SW4, INPUT_PULLUP);

  MIDI_CHANNEL = ReadDipSwitch();

  // Initialise MIDI
  MIDI.begin(MIDI_CHANNEL);
  //MIDI.setThruFilterMode(midi::Thru::Off); // throws an error

  // Initialise MIDI message handlers
  MIDI.setHandleNoteOff(midiNoteOffHandler);
  MIDI.setHandleNoteOn(midiNoteOnHandler);
}

// main loop
void loop() {
  int8_t MUX = 0;
  uint32_t freq;

  while (1) {

    digitalWrite(10, TRIG);  // output the trigger signal
    digitalWrite(LED_BUILTIN, LOW);

    //--------------- ADC block -------------------------------------
    while (bit_is_set(ADCSRA, ADSC))
      ;  //Wait for ADC EOC

    switch (MUX) {
      case 0:  // sample rate / octave
        SAMPLERATE_POT = ((ADCL + (ADCH << 8)) >> 7);
        //SAMPLERATE = samplerates[SAMPLERATE_POT];
        break;
      case 1:  // tune TODO
        TUNE_POT = (ADCL + (ADCH << 8));
        break;
      case 2:  // ensemble
        DETUNE = ((ADCL + (ADCH << 8)) >> 3);
        break;
      case 3:  // phaser amt
        PHASERMIX = ((ADCL + (ADCH << 8)) >> 2);
        break;
      case 4:  // lfo freq
        LFOINC = ((ADCL + (ADCH << 8)) >> 5) + 1;
        break;
      case 5:  // lfo amt
        MOD = ((ADCL + (ADCH << 8)) >> 2);
        break;
      case 6:  // envelope
        ENVSHAPE = ((ADCL + (ADCH << 8)) >> 5);
        ATTACK = ATTrates[ENVSHAPE];
        RELEASE = RELrates[ENVSHAPE];
        break;
      case 7:  // bitcrush
        CRUSH = ((ADCL + (ADCH << 8)) >> 7);
        break;
    }

    // here we assign the frequencies to the oscillators
    for (uint8_t i = 0; i < N_VOICES; i++) {
      if ((OSCNOTES[i] < 255) && TRIG) {        // only update base osc when note change and trigger set
        FREQ[i << 1] = MIDI2FREQ(OSCNOTES[i]);  // calculate the frequency of the note and assign to the base osc
        //if (!(TUNE_POT >> 5)) FREQ[i << 1] = (FREQ[i << 1] * (TUNE_POT + 3584)) >> 12;
      }
      //FREQ[i << 1] = FREQ[i << 1] + vibrato lfo
      if (DETUNE) FREQ[(i << 1) | 1] = FREQ[i << 1] + (((FREQ[i << 1] / 50) >> 0) * DETUNE / 127);
      else FREQ[(i << 1) | 1] = 0;  // turns off secondary osc to avoid unpleasant sounds
      //FREQ[(i << 1) | 1] = FREQ[i << 1] + (((FREQ[i << 1] >> 10) * DETUNE) >> 7);  // version without division
    }

    MUX++;
    if (MUX > 7) MUX = 0;
    ADMUX = 64 | MUX;   //Select MUX
    sbi(ADCSRA, ADSC);  //start next conversation
    //--------------------------------------------------------------------
  }
}


// audio interrupt
ISR(TIMER1_COMPA_vect) {

  uint8_t wet;

  // poll midi
  MIDI.read();

  //-------------------- 8 DCO block ------------------------------------------
  // that doesn't produce a saw but rather a spike thing like |\___ where the spike always has the same length (28/fsample)
  // the weird behaviour above a certain frequency is plain and simple aliasing
  // the resulting wavefront is still a good approximation of how string machines work
  // the "integrator" is actually a differentiator and even more actually a high pass and a diode giving you the impulse response of the rising edge only
  DCO = 0;

  for (uint8_t i = 0; i < 2 * N_VOICES; i++) {
    if (integrators[i]) integrators[i]--;  //Decrement integrators only when not zero
    DCOPH[i] += FREQ[i];                   //Add freq to phaseacc's
    if (DCOPH[i] & 0x800000) {             //Check for integrator reset
      DCOPH[i] &= 0x7FFFFF;                //Trim NCO
      integrators[i] = 28;                 //Reset integrator
    }
    DCO += integrators[i];
  }

  // --------------- Chorus effect ----------------------------------
  // -> might lead to distortion sometimes
  // with three lfos and another pwm output a stereo chorus should be possible
  writepointer++;
  delayline[writepointer] = DCO;
  wet = (delayline[(writepointer - LFOVAL2) & 255] * PHASERMIX) >> 8;

  //------------------ VCA block ------------------------------------
  // there is a chance i didn't get the intention behind the original vca so i replaced it with a simple multiplication
  // envelope and dco are both 0...255 and result must be the same since OCR2A is 8 bit

#ifdef MONO
  DCO += wet;
  OCR2A = (ENVELOPE * (int16_t)DCO) >> 8;
#elif defined STEREO
  OCR2A = ((ENVELOPE * (int16_t)DCO) >> 8) & crushmask[CRUSH];
  OCR2B = ((ENVELOPE * (int16_t)wet) >> 8) & crushmask[CRUSH];
#endif

  //-------------- Calc Sample freq ---------------------------------
  OCR1A = SAMPLERATE;
}


// slow interrupt for envelope and LFO
ISR(TIMER0_COMPA_vect) {
  uint16_t sr_temp;

  //------------------------------ LFO Block -----------------------
  LFOPH += LFOINC;
  LFOVAL = pgm_read_byte_near(sinetable + LFOPH);        //LFO for pitch
  LFOVAL2 = pgm_read_byte_near(sinetable + (LFOPH2++));  //LFO for the Phaser
  sr_temp = samplerates[SAMPLERATE_POT] + ((MOD * LFOVAL) >> 8);
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    SAMPLERATE = sr_temp;  // needs protection otherwise makes some blop blop
  }

  //--------------------- ENV block ---------------------------------
  // that is a linear envelope so not your typical choice for vca but sounds nice in the attack phase
  // also the quick release drop helps with quantization noise, also simple
  if ((TRIG) && (ENVELOPE < 255)) {
    ENVELOPE += ATTACK;
    if (ENVELOPE > 255) ENVELOPE = 255;
  } else if ((!TRIG) && (ENVELOPE > 0)) {
    ENVELOPE -= RELEASE;
    if (ENVELOPE < 0) ENVELOPE = 0;
  }
}


/****************
  MIDI HANDLERS
*****************/
void midiNoteOnHandler(byte channel, byte note, byte velocity) {
  handleMIDINOTE(0x90, note, 127);
  //digitalWrite(LED_BUILTIN, HIGH);
}

void midiNoteOffHandler(byte channel, byte note, byte velocity) {
  handleMIDINOTE(0x80, note, 0);
  //digitalWrite(LED_BUILTIN, LOW);
}

// FUNCTIONS
//---------------- Get the base frequency for the MIDI note ---------------
uint32_t MIDI2FREQ(uint8_t note) {
  uint8_t key = note % 12;
  uint32_t out = 0;
  if (note > 127) out = 0;  // using these as control messages since midi notes are <128
  else if (note < 36) out = (NOTES[key] >> (1 + (35 - note) / 12));
  else if (note > 47) out = (NOTES[key] << ((note - 36) / 12));
  else out = NOTES[key];
  return out << noteshift[SAMPLERATE_POT];
}

//---------------- Handle Notes---------------------------------------
void handleMIDINOTE(uint8_t message, uint8_t note, uint8_t vel) {
  uint8_t i;

  // note on with velocity 0 is note off --> some controllers do such weird things, for example some arduino example scripts
  if ((!vel) && (message == 0x90)) message = 0x80;

  // note off
  if (message == 0x80) {              // if note off command arrives
    for (i = 0; i < N_VOICES; i++) {  // check all oscillators if that note is playing
      if (OSCNOTES[i] == note) {      // when we arrive at the oscillator playing that note
        OSCNOTES[i] = 255;            // free that osc. 255 will not be updatet in frequency calculation for the sticky chord behaviour
        TRIG &= ~(1 << i);            // clear channel trigger bit
      }
    }
    return;  // we can stop cycling through now since we found an oscillator playing the note
  }

  // note on
  if (message == 0x90) {                                 // when note on arrives
    if (!TRIG) {                                         // if all previous notes have been released before a new note is pressed free all osc first
      for (i = 0; i < N_VOICES; i++) OSCNOTES[i] = 254;  // the frequency calculation recognizes that as to be updatet but MIDI2FREQ returns 0
    }
    for (i = 0; i < N_VOICES; i++) {  // check all oscillators if busy
      if (OSCNOTES[i] > 127) {        // found an oscillator thats not busy? make it busy!
        OSCNOTES[i] = note;           // assign new note to the oscillator
        TRIG |= (1 << i);             // set channel trigger bit
        return;                       // we can stop cycling through now since we found an oscillator for our new note
      }
    }
  }
}

/*
 * Get MIDI channel from dipswitches
 */
byte ReadDipSwitch() {
  byte value = 0;
  if (digitalRead(DIP_SW4) == LOW) value += 1;
  if (digitalRead(DIP_SW3) == LOW) value += 2;
  if (digitalRead(DIP_SW2) == LOW) value += 4;
  if (digitalRead(DIP_SW1) == LOW) value += 8;
  return (value + 1);
}