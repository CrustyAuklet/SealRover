#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
#include "SBUS.h"

PWMServo panServo;

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=146,292
AudioOutputI2S           i2s1;           //xy=383,291
AudioConnection          patchCord1(playSdWav1, 0, i2s1, 0);
AudioConnection          patchCord2(playSdWav1, 1, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=152,360
// GUItool: end automatically generated code

// Use these with the audio adaptor board
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

// PIN DEFINES
#define SBUS_INPUT 1
//#define UNUSED_DIN1 2
//#define UNUSED_DIN2 8
#define PAN_SERVO 3
#define TILT_SERVO 4
//#define UNUSED_PWM2 5
#define DIP_PIN_1 21
#define DIP_PIN_2 20
#define DIP_PIN_3 17
#define DIP_PIN_4 16

/*** GLOBAL VARIABLES ***/
SBUS x8r(SBUS_INPUT);     // SBUS object, which is on HWSERIAL 1
uint16_t channels[16];    // RX channels read from sbus 11 bit unsigned values (0-2047)
uint8_t failSafe;         // failsafe flag
uint16_t lostFrames = 0;  // counter for lost frames
char tmpStr[30];          // temporary string for holding serial output
float vol = .8;           // Volume setting ( range is 0 to 1, distortion when > .8 )
short soundNum = 0;       // The number of the sound to play, set in ISR by dip-switches
unsigned int panVal = 90; // tracking value for pan servo

void setup() {
    Serial.begin(9600);
    // begin the SBUS communication
    Serial.println("Starting SBUS input...");
    x8r.begin();

    // setup the pan servo
    panServo.attach(PAN_SERVO);
    panServo.write(90);
  
    // Setup the audio
    Serial.println("Starting Audio Controls...");
    AudioMemory(8);
    sgtl5000_1.enable();
    sgtl5000_1.volume(vol);
  
    // Make sure that SD card is inserted
    SPI.setMOSI(SDCARD_MOSI_PIN);
    SPI.setSCK(SDCARD_SCK_PIN);
    if (!(SD.begin(SDCARD_CS_PIN))) {
        // stop here, but print a message repetitively
        while (1) {
            Serial.println("Unable to access the SD card!");
            delay(500);
        }
    }
    Serial.println("SD Card detected...");
    
    // Setup the interrupts for DIP switches
    pinMode(DIP_PIN_1, INPUT_PULLUP);
    pinMode(DIP_PIN_2, INPUT_PULLUP);
    pinMode(DIP_PIN_3, INPUT_PULLUP);
    pinMode(DIP_PIN_4, INPUT_PULLUP);    
    attachInterrupt(DIP_PIN_1, isrDipSwitches, CHANGE);  // Bit 1 - LSB
    attachInterrupt(DIP_PIN_2, isrDipSwitches, CHANGE);  // Bit 2
    attachInterrupt(DIP_PIN_3, isrDipSwitches, CHANGE);  // Bit 3
    attachInterrupt(DIP_PIN_4, isrDipSwitches, CHANGE);  // Bit 4 - MSB
    isrDipSwitches();
  
} // END OF SETUP

void loop() {
    // look for a good SBUS packet from the receiver
    if(x8r.read(&channels[0], &failSafe, &lostFrames)){

        // move left till hit 180 degrees
        if(channels[3] > 1050 && panVal < 180) {
          panServo.write(panVal++);
        }
        // move right when less, till hit 0 degrees
        else if(channels[3] < 950 && panVal > 0) {
          panServo.write(panVal--);
        }

        if(channels[5] > 1000 & !playSdWav1.isPlaying()) {
          playFile(soundNum);
        }
    
        // Set the volume of the headphone output from 0.0 to 0.8
        // Based on analog input on pin 15 (pot from audio shield)
        setVolume();
    
        printInputs();
    }
}

// Playfile function from Teensy Example
// plays numerically names wave files from SD card
void playFile(const short fileNum)
{
  sprintf(tmpStr, "%02d.WAV", fileNum);
  Serial.print("Playing file: ");
  Serial.println(tmpStr);

  // sketch continues to run while the file plays.
  playSdWav1.play(tmpStr);

  // A brief delay for the library read WAV info
  delay(5);
}

void setVolume() {
    vol = (analogRead(15) / 1024.0) * .8;
    sgtl5000_1.volume(vol);
}

void printInputs() {
    Serial.print("DATA: ");
    for(int i = 0; i < 16; i++){
      sprintf(tmpStr, " CH%d: %4d", i, channels[i]);
      Serial.print(tmpStr);
    }
    //sprintf(tmpStr, " VOL: %02d SND: %d", vol, soundNum);
    Serial.print(" VOL: ");
    Serial.print(vol);
    Serial.print(" DIP: ");
    Serial.print(soundNum);
    Serial.print(" PAN_VAL: ");
    Serial.print(panVal);
    Serial.println();
}

// ISR for updating the file name when DIP switches change
void isrDipSwitches() {
    soundNum  = !digitalRead(DIP_PIN_1);
    soundNum |= (!digitalRead(DIP_PIN_2))<<1;
    soundNum |= (!digitalRead(DIP_PIN_3))<<2;
    soundNum |= (!digitalRead(DIP_PIN_4))<<3;
}

