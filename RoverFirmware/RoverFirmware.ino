#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <PWMServo.h>
#include "SBUS.h"

#define DEBUG_OUTPUT 1

// defines for general use and eliminating magic numbers
#define NUM_CHANNELS        (8)

// Audio board defines
#define VS1053_RESET        (-1)  // VS1053 reset pin is not used
#define VS1053_CS           (3)   // VS1053 chip select pin (output)
#define VS1053_DCS          (10)  // VS1053 Data/command select pin (output)
#define CARDCS              (8)   // SD card select pin
#define VS1053_DREQ         (4)   // VS1053 Data request, ideally an Interrupt pin

// Define outputs
#define CENTER_VAL          (90)
#define THROTTLE_STOP       (70)
#define THROTTLE            (5)
#define STEERING            (6)
#define PAN_SERVO           (9)
#define TILT_SERVO          (20)

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
#define SLOW_SCALE          (0.25)
#define MEDIUM_SCALE        (0.50)
#define FULL_SCALE          (1.00)
#define VOLUME_SETTING      (0.80)

/*** GLOBAL VARIABLES ***/
// music-player object for playing sound files
Adafruit_VS1053_FilePlayer musicPlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS); // sound player
PWMServo throttle;                      // PWM output for the car ESC
PWMServo steering;                      // PWM output for the steering servo
PWMServo panServo;                      // PWM for pan servo on camera mount
PWMServo tiltServo;                     // PWM for pan servo on camera mount
SBUS     x8r(Serial1);                  // SBUS object, which is on HWSERIAL 1
uint16_t channels[16];                  // RX channels read from sbus 11 bit unsigned values (0-2047)
char     tmpStr[30];                    // temporary string for holding serial output
bool     failSafe    = true;            // failsafe flag
bool     lostFrames  = false;           // counter for lost frames
unsigned int panVal  = CENTER_VAL;      // tracking value for pan servo
unsigned int tiltVal = CENTER_VAL;      // tracking value for tilt servo
unsigned int steer   = CENTER_VAL;      // tracking value for steering
int      throt       = THROTTLE_STOP;   // tracking value for throttle
int      battVal     = 0;               // stores the battery value of the car, could be used for lipo protection
float    scaling     = 1.0;             // Value to scale the throttle by

void setup() {
    Serial.begin(9600);
    Serial.println("\n\nCosta Lab Seal Rover: v0.1.0");

    // begin the SBUS communication
    Serial.println("Starting SBUS input...");
    x8r.begin();

    // initialise the music player, print error message if not found and DON'T stop
    if (! musicPlayer.begin()) {
        Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
        musicPlayer.setVolume(10,10);
        while(1) {
            musicPlayer.sineTest(0x44, 500);
        }
    }
    else {
        Serial.println(F("VS1053 found"));
    }

    // see if SD card exists, if not print error but DO NOT stop
    if (!SD.begin(CARDCS)) {
        Serial.println(F("SD failed, or not present"));
        musicPlayer.setVolume(10,10);
        while(1) {
            musicPlayer.sineTest(0x44, 500);
        }
    }
    else {
        // print positive message and then list file for debug
        Serial.println("SD OK!");
        printDirectory(SD.open("/"), 0);
    }

    // Set volume for left, right channels. lower numbers == louder volume!
    musicPlayer.setVolume(1,1);

    // DREQ is on an interrupt pin so we can do background audio playing
    musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);

    // create PWM channels for motor outputs and set starting values
    throttle.attach(THROTTLE, 1000, 2000);
    throttle.write(THROTTLE_STOP);

    steering.attach(STEERING, 1000, 2000);
    steering.write(CENTER_VAL);

    panServo.attach(PAN_SERVO, 500, 2500);
    panServo.write(CENTER_VAL);

    tiltServo.attach(TILT_SERVO, 800, 2200);
    tiltServo.write(CENTER_VAL);

} // END OF SETUP

void loop() {
//    battVal = analogRead(BATT_SENSE);

    // look for a good SBUS packet from the receiver
    if(x8r.read(&channels[0], &failSafe, &lostFrames)){
        if(failSafe) {
            throttle.write(THROTTLE_STOP);
            steering.write(CENTER_VAL);
        }
        else {

            if(channels[JOYSTICK_BTN_IN] < 1500) {
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
            }
            else {
                panVal  = CENTER_VAL;
                tiltVal = CENTER_VAL;
                panServo.write(panVal);
                tiltServo.write(tiltVal);
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

            // saftey check, throttle values should lie between 400 and 1800
            // often sudden signal loss can result in out of range values
            if(channels[THROTTLE_IN] > 400 && channels[THROTTLE_IN] < 1800) {
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
            }
            else {
                throt = THROTTLE_STOP;
                steer = CENTER_VAL;
            }

            throttle.write(throt);
            steering.write(steer);

            if( channels[SOUND_PLAY_IN] > 1000 & !musicPlayer.playingMusic ) {
                if(!musicPlayer.startPlayingFile("04.wav")) {
                    Serial.println("CAN NOT FIND FILE!");
                }
            }
        }
    }
    else if(failSafe){
        throttle.write(THROTTLE_STOP);
        steering.write(CENTER_VAL);
    }

    #if DEBUG_OUTPUT
    printInputs();
    #endif

}

/// function to print debug output for the inputs
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

/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {

     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

