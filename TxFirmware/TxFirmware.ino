/*
 * PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 */

//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 8            // set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  // set the default servo value
#define FRAME_LENGTH 22500          // set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300            // set the pulse length
#define onState 0                   // set polarity of the pulses: 1 is positive, 0 is negative

/***** DIGITAL OUTPUT *****/
#define sigPin 10                   // set PPM signal output pin on the arduino
#define redLedPin 11                // The pin for red output LED
#define greenLedPin 12              // The pin for green output LED

/***** DIGITAL INPUT - ACTIVE LOW *****/
#define cameraBtn 6                 // down press on camera joystick
#define engSwPin 5                  // 2-way switch for car power
#define bigRedBtnPin 4              // Big red momentary button
#define cameraPwrUp 3               // 3-way camera switch in upper position
#define cameraPwrDwn 2              // 3-way camera switch in lower position

/***** ANALOG INPUT *****/
#define throttlePin A0              // controller throttle (trigger)
#define steeringPin A1              // controller steering (wheel)
#define cameraXpin A2               // Camera pan-tilt X-axis
#define cameraYpin A3               // Camera pan-tilt Y-axis

/***** PPM SETTINGS *****/
#define MAX_CH_VALUE 2000           // Maximum value to send over PPM
#define MIN_CH_VALUE 1000           // Minimum value to send over PPM
#define STEERING_OFFSET 450         // Center offset for seeting pot
#define THROTTLE_OFFSET 100         // Center Offset for throttle pot
#define DEAD_ZONE_LOW 1450          // Set up lower limit for channel dead zones
#define DEAD_ZONE_HIGH 1550         // Set up upper limit for channel dead zones
//////////////////////////////////////////////////////////////////



/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];

int  minSteering, ctrSteering, maxSteering, minThrottle, ctrThrottle, maxThrottle;
int  steering = 1500;
int  throttle = 1500;
int  cameraX  = 1500;
int  cameraY  = 1500;
int  cameraSw = 0;
int  motorSw  = 0;

void setup(){
  // Get the center values of inputs at powerup
  ctrThrottle = analogRead(throttlePin);
  ctrSteering = analogRead(steeringPin);
  minThrottle = ctrThrottle - THROTTLE_OFFSET;
  maxThrottle = ctrThrottle + THROTTLE_OFFSET;
  minSteering = ctrSteering - STEERING_OFFSET;
  maxSteering = ctrSteering + STEERING_OFFSET;
  
  Serial.begin(115200);
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
    ppm[i]= CHANNEL_DEFAULT_VALUE;
  }

  // Setup the output LEDs
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);

  // Setup Digital Inputs
  pinMode(cameraBtn, INPUT_PULLUP);
  pinMode(engSwPin, INPUT_PULLUP);
  pinMode(bigRedBtnPin, INPUT_PULLUP);
  pinMode(cameraPwrUp, INPUT_PULLUP);
  pinMode(cameraPwrDwn, INPUT_PULLUP);

  // Setup the PPM oputput pin
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

  digitalWrite(redLedPin, HIGH);
  delay(100);
  digitalWrite(redLedPin, LOW);
  delay(100);
  digitalWrite(redLedPin, HIGH);
  delay(100);
  digitalWrite(redLedPin, LOW);
  delay(100);
  digitalWrite(redLedPin, HIGH);
  delay(100);
  digitalWrite(redLedPin, LOW);
}



void loop(){
  // Throttle (CH1)
  throttle = map(analogRead(throttlePin), minThrottle, maxThrottle, MIN_CH_VALUE, MAX_CH_VALUE);
  if(throttle > DEAD_ZONE_LOW && throttle < DEAD_ZONE_HIGH) {  // Create deadZone for the throttle
    throttle = 1500;
  }
  ppm[0] = throttle;
  
  // Steering (CH2)
  steering = map(analogRead(steeringPin), minSteering, maxSteering, MIN_CH_VALUE, MAX_CH_VALUE);
  ppm[1] = steering;

  // Camera X-Axis (CH3)
  cameraX = map(analogRead(cameraXpin), 0, 1024, MAX_CH_VALUE, MIN_CH_VALUE);
  if(cameraX > DEAD_ZONE_LOW && cameraX < DEAD_ZONE_HIGH) {  // Create deadZone for the Camera X-Axis
    cameraX = 1500;
  }
  ppm[2] = cameraX;
  
  // Camera Y-Axis (CH4)
  cameraY = map(analogRead(cameraYpin), 0, 1024, MAX_CH_VALUE, MIN_CH_VALUE);
  if(cameraY > DEAD_ZONE_LOW && cameraY < DEAD_ZONE_HIGH) {  // Create deadZone for the Camera Y-Axis
    cameraY = 1500;
  }
  ppm[3] = cameraY;

  // Motor Kill Switch - ACTIVE FORWARD (CH5)
  if(digitalRead(engSwPin)){
    ppm[4] = 2000;
  }
  else {
    ppm[4] = 1000;
  }
  
  // Play Sound Switch - ACTIVE LOW (CH6)
  if(digitalRead(bigRedBtnPin)){
    ppm[5] = 1000;
  }
  else {
    ppm[5] = 2000;
  }

  // Camera Power Switch (3-way) (CH7)
  if(digitalRead(cameraPwrUp)){
    if(digitalRead(cameraPwrDwn)) {
      // Switch is in middle position
      ppm[6] = 1500;
    }
    else {
      // Switch is in lower position
      ppm[6] = 1000;
    }
  }
  else {
    // Switch is in upper position
    ppm[6] = 2000;
  }

  digitalWrite(redLedPin, !digitalRead(cameraBtn));
  digitalWrite(greenLedPin, digitalRead(engSwPin));
  
  Serial.print("THROTTLE: ");
  Serial.print(ppm[0]);
  Serial.print("    STEERING: ");
  Serial.print(ppm[1]);
  Serial.print("    CAMERA-X: ");
  Serial.print(ppm[2]);
  Serial.print("    CAMERA-Y: ");
  Serial.print(ppm[3]);
  Serial.print("    MOTOR: ");
  Serial.print(ppm[4]);
  Serial.print("    SOUND: ");
  Serial.print(ppm[5]);
  Serial.print("    CAM-PWR: ");
  Serial.print(ppm[6]);
  Serial.print("    UNASSIGNED: ");
  Serial.println(ppm[7]);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
