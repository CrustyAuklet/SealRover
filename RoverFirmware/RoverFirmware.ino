#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
#include "SBUS.h"

#define DEBUG_OUTPUT 1

PWMServo throttle;
PWMServo steering;
PWMServo panServo;
PWMServo tiltServo;
PWMServo pwm_ch5;
PWMServo pwm_ch6;

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=146,292
AudioOutputI2S           i2s1;           //xy=383,291
AudioConnection          patchCord1(playSdWav1, 0, i2s1, 0);
AudioConnection          patchCord2(playSdWav1, 1, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=152,360
// GUItool: end automatically generated code

// defines for general use and eliminating magic numbers
#define NUM_CHANNELS      (8)

// Use these with the audio adaptor board
#define SDCARD_CS_PIN     (10)
#define SDCARD_MOSI_PIN   (7)
#define SDCARD_SCK_PIN    (14)

// Define Inputs
#define SBUS_INPUT        (1)
#define DIP_PIN_1         (15)
#define DIP_PIN_2         (16)
#define DIP_PIN_3         (17)
#define BATT_SENSE        (10)
#define TEMP_SENSE        (11)

// Define outputs
#define CENTER_VAL      (90)
#define THROTTLE_STOP   (70)
#define THROTTLE        (3)
#define STEERING        (4)
#define PAN_SERVO       (5)
#define TILT_SERVO      (6)
#define PWM_5           (21)
#define PWM_6           (20)

// Define SBUS channels
#define THROTTLE_IN         (0)
#define STEERING_IN         (1)
#define CAM_TILT_IN         (2)
#define CAM_PAN_IN          (3)
#define DUAL_SWITCH_IN      (4)
#define SOUND_PLAY_IN       (5)
#define SCALING_IN          (6)
#define JOYSTICK_BTN_IN     (7)

// Define speed scaling values
#define SLOW_SCALE          (0.15)
#define MEDIUM_SCALE        (0.25)
#define FULL_SCALE          (1.00)
#define VOLUME_SETTING      (0.80)

/*** GLOBAL VARIABLES ***/
SBUS x8r(SBUS_INPUT);                   // SBUS object, which is on HWSERIAL 1
uint16_t channels[16];                  // RX channels read from sbus 11 bit unsigned values (0-2047)
char tmpStr[30];                        // temporary string for holding serial output
uint8_t  failSafe    = 1;               // failsafe flag
uint16_t lostFrames  = 0;               // counter for lost frames
float vol            = VOLUME_SETTING;  // Volume setting ( range is 0 to 1, distortion when > .8 )
short soundNum       = 0;               // The number of the sound to play, set in ISR by dip-switches
unsigned int panVal  = CENTER_VAL;      // tracking value for pan servo
unsigned int tiltVal = CENTER_VAL;      // tracking value for tilt servo
unsigned int steer   = CENTER_VAL;      // tracking value for steering
int throt            = THROTTLE_STOP;   // tracking value for throttle
int battVal          = 0;               // stores the battery value of the car, could be used for lipo protection
float scaling        = 1.0;             // Value to scale the throttle by

void setup() {
    Serial.begin(9600);
    // begin the SBUS communication
    Serial.println("Starting SBUS input...");
    x8r.begin();

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

    // create PWM channels for motor outputs and set starting values
    throttle.attach(THROTTLE, 1000, 2000);
    throttle.write(THROTTLE_STOP);
    steering.attach(STEERING, 1000, 2000);
    steering.write(CENTER_VAL);
    panServo.attach(PAN_SERVO, 500, 2500);
    panServo.write(CENTER_VAL);
    tiltServo.attach(TILT_SERVO, 800, 2200);
    panServo.write(CENTER_VAL);
    pwm_ch5.attach(PWM_5, 1000, 2000);
    pwm_ch5.write(CENTER_VAL);
    pwm_ch6.attach(PWM_6, 1000, 2000);
    pwm_ch6.write(CENTER_VAL);

    // Setup the interrupts for DIP switches
    pinMode(DIP_PIN_1, INPUT_PULLUP);
    pinMode(DIP_PIN_2, INPUT_PULLUP);
    pinMode(DIP_PIN_3, INPUT_PULLUP);
    attachInterrupt(DIP_PIN_1, isrDipSwitches, CHANGE);  // Bit 1 - LSB
    attachInterrupt(DIP_PIN_2, isrDipSwitches, CHANGE);  // Bit 2
    attachInterrupt(DIP_PIN_3, isrDipSwitches, CHANGE);  // Bit 3
    isrDipSwitches();

    // center the pan and tilt servo on reset
    panServo.write(panVal);
    tiltServo.write(tiltVal);

} // END OF SETUP

void loop() {
    battVal = analogRead(BATT_SENSE);

    // look for a good SBUS packet from the receiver
    if(x8r.read(&channels[0], &failSafe, &lostFrames)){
        if(failSafe) {
            throttle.write(THROTTLE_STOP);
            steering.write(CENTER_VAL);
        }
        else {

            // Camera panning, with limits and static values
            if(channels[CAM_PAN_IN] > 1050 && panVal < 180) {
              panServo.write(panVal++);
            }
            else if(channels[CAM_PAN_IN] < 950 && panVal > 0) {
              panServo.write(panVal--);
            }

            // Camera Tilting, with limits and static values.
            // mechanical limitations limits range to between 50 and 110 degrees
            if(channels[CAM_TILT_IN] > 1050 && tiltVal < 110) {
              tiltServo.write(tiltVal++);
            }
            else if(channels[CAM_TILT_IN] < 950 && tiltVal > 50) {
              tiltServo.write(tiltVal--);
            }

            // update scaling factor
            if(channels[SCALING_IN] < 600) {
                // creeping speed
                scaling = SLOW_SCALE;
            }
            else if(channels[SCALING_IN] < 1500) {
                // moderate speed
                scaling = MEDIUM_SCALE;
            }
            else {
                // fastest - no scaling
                scaling = FULL_SCALE;
            }

            // Map values for steering and throttle and then send them
            throt = map(channels[THROTTLE_IN], 500, 1800, 0, 180);
            steer = map(channels[STEERING_IN], 180, 1800, 180, 0);

            if(throt > 71){
                throt -= THROTTLE_STOP;
                throt *= scaling;
                throt += THROTTLE_STOP;
            }
            else if(throt < 69) {
                throt = THROTTLE_STOP - throt;
                throt *= scaling;
                throt = THROTTLE_STOP - throt;
            }

            throttle.write(throt);
            steering.write(steer);

            if(channels[SOUND_PLAY_IN] > 1000 & !playSdWav1.isPlaying()) {
              playFile(soundNum);
            }
        }
    }
    #if DEBUG_OUTPUT
    printInputs();
    #endif

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
    for(int i = 0; i < NUM_CHANNELS; i++){
      sprintf(tmpStr, " CH%d: %4d", i, channels[i]);
      Serial.print(tmpStr);
    }
    //sprintf(tmpStr, " VOL: %02d SND: %d", vol, soundNum);
    Serial.print(" STEERING: ");
    Serial.print(steer);
    Serial.print(" THROTTLE: ");
    Serial.print(throt);
    Serial.print(" VOL: ");
    Serial.print(vol);
    Serial.print(" DIP: ");
    Serial.print(soundNum);
    Serial.print(" PAN_VAL: ");
    Serial.print(panVal);
    Serial.print(" TILT_VAL: ");
    Serial.print(tiltVal);
    Serial.print(" BAT: ");
    Serial.print(battVal);
    Serial.print(" FAILSAFE: ");
    Serial.print(failSafe);
    Serial.print(" LOST: ");
    Serial.print(lostFrames);
    Serial.println();
}

// ISR for updating the file name when DIP switches change
void isrDipSwitches() {
    soundNum  = !digitalRead(DIP_PIN_1);
    soundNum |= (!digitalRead(DIP_PIN_2))<<1;
    soundNum |= (!digitalRead(DIP_PIN_3))<<2;
}

