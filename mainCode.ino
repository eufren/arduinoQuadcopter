#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <PID_v1.h> // Open source PID lib
#include <digitalWriteFast.h> // Open source pin register manipulation library
#include <NewPing.h> // Non-blocking ultrasonic library

// Pins currently in use
// Using 'const uint8_t' instead of just int for memory saving reasons
const uint8_t pThrottleIn = 5;
const uint8_t pThrottleOut = 6;
const uint8_t pRelaySwitch = 4;
const uint8_t pTrigger = 9;
const uint8_t pEcho = 8;
const uint8_t pDropServo = 3;
const uint8_t pSD = 10;
const uint8_t pRadioSwitch = 14; // 14 is A0

// Mode setting
const bool manual = false; // For only recording throttle response data on manual flight
const bool onlyDrop = false; // Manual control, drop beanbag at correct height
const bool onlyHover = false; // Automatic throttle, hover at correct height
const bool hoverAndDrop = true; // Automatic thqrottle, hover at correct height, drop beanbag at correct height

// Autopilot variables
double Setpoint = 100;
double Input, Output;
double preKp=0.55, preKi=0.7, preKd=0.5; // Values before drop 
double postKp=0.55, postKi=0.7, postKd=1; // Values after drop 
PID myPID(&Input, &Output, &Setpoint, preKp, preKi, preKd, P_ON_M, DIRECT);

// Global variables
Servo dropServo;
Servo naze32;
NewPing sonar(pTrigger, pEcho, 500);
File logfile;
char logname[15];
unsigned long timer; // Timer variable (needs to be in startup and loop)
float currentAlt;
float lastSignal;
bool dropped = false;
bool pidSet = false;
bool justRun = false;
int cycles = 0;
float dropTimer;
float lastAlt;

// Custom functions
 
float ultrasonicRead() { // Computes smoothed output from ultrasonic sensor
  float instAlt = sonar.convert_cm(sonar.ping_median(2));
  if(fabs(instAlt - lastAlt) > 20){
    return lastAlt;
  }
  float alpha = 0.8;
  float smoothedAlt = alpha*instAlt + (1-alpha)*lastAlt;
  lastAlt = smoothedAlt;
  return smoothedAlt;
}

void checkAndSet() {
    // If the radio switch has changed more than 100 from the last time
  float currentSignal = pulseIn(A0, HIGH);
  if(fabs(currentSignal-lastSignal) > 100){ 
    if(pulseIn(A0, HIGH) < 1500){ // then check what the value is, if it's low, turn off relay and set PID mode to manual, set flag saying pid not running
        digitalWriteFast(pRelaySwitch, LOW);
        myPID.SetMode(MANUAL);
        pidSet = false;
      }
      else{ // if it's high, turn on relay and set PID mode to auto, set flag saying PID is running
        myPID.SetMode(AUTOMATIC);
          if(dropped==true) // If we're in post-drop state, set post drop gains
          {
            myPID.SetTunings(postKp, postKi, postKd);
            Input = currentAlt;
            myPID.Compute();
            naze32.write(Output);
            pidSet = true;
            justRun = true;
            digitalWriteFast(pRelaySwitch, HIGH);
          }
          else // If we're in pre-drop state, set pre drop gains
          {
            myPID.SetTunings(preKp, preKi, preKd);
            Input = currentAlt;
            myPID.Compute();
            naze32.write(Output);
            pidSet = true;
            justRun = true;
            digitalWriteFast(pRelaySwitch, HIGH);
          }
      }
  }
  lastSignal = currentSignal;
}

void doThrottle() {
  if(pidSet==true && justRun==false){   // check if pid flag is set, and that the pid wasnt just calculated because the swtich changed, and if so, update the pid
    Input = currentAlt;
    myPID.Compute();
    naze32.write(Output);
  }
  else if(pidSet==true && justRun==true){ // if the PID was already calculated because the switch changed, then just reset the flag for the next loop
    justRun = false;
  }else{   // if neither, then use the radio throttle
    Output = pulseIn(5, HIGH);
  }
}

void checkDrop() {
    if(fabs(currentAlt-Setpoint) < 2.5 && dropped==false){
      dropServo.write(0);
      logfile = SD.open(logname, FILE_WRITE);
      char servoNotification[30];
      dropTimer = millis();
      sprintf(servoNotification, "Servo opened at %d", dropTimer);
      logfile.println(servoNotification);
      logfile.close();
      dropped = true;
    };
    if(dropped==true && (millis()-dropTimer > 1000)){
     dropServo.write(30);
    }
}

void writeSD() {
  // Write estimate altitude, throttle signal, and millis() to SD card
  unsigned long timeElapsed = (millis()-timer);
  String writeOut = "";
  writeOut.concat(timeElapsed);
  writeOut.concat(",");  
  writeOut.concat(Output);
  writeOut.concat(",");
  writeOut.concat(String(currentAlt));
  logfile = SD.open(logname, FILE_WRITE);
  logfile.println(writeOut);
  logfile.close();
}

// Standard functions
 
void setup()  {
  Serial.begin(9600);
  
  // Servo setup
  dropServo.attach(pDropServo);
  naze32.attach(pThrottleOut);

  // SD card setup
  pinModeFast(pSD, OUTPUT); // Using pin 10 for SD card but could change it according to https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial?view=all
  float lognum; // Increment variable, lets us make a new log for each flight, MAKE SURE YOU HAVE SET THIS AS 0 OR SOME OTHER FLOAT FIRST, USING ANOTHER SKETCH
  EEPROM.get(0, lognum); // EEPROM address for storing increment variable
  if(isnan(lognum)){EEPROM.put(0, 0);} // If the EEPROM has nothing in then start at 0
  int intLognum = round(lognum)+1; // Increment it
  EEPROM.put(0, float(intLognum));
  sprintf(logname, "Log_%d.csv", intLognum); // Name of the log files
  SD.begin(pSD);
  logfile = SD.open(logname, FILE_WRITE); // Create the log file and prepare it for writing to
  logfile.flush();

  // Ultrasonic setup
  pinModeFast(pTrigger, OUTPUT); // Sets the trigger pin, output
  pinModeFast(pEcho, INPUT); // Sets the echo pin, input
 
  // Throttle logging setup
  pinModeFast(pRadioSwitch, INPUT);
  pinModeFast(pThrottleIn, INPUT);
  pinModeFast(pRelaySwitch, OUTPUT);
  timer = millis(); // Start timer for logging
  dropServo.write(30); // Move servo arm to ready position

  // PID Setup
  myPID.SetOutputLimits(1098, 1900);
  myPID.SetSampleTime(150);

  // Poll the radio switch
  lastSignal = pulseIn(A0, HIGH);

  // Check which mode the program is in, and run
  if((manual&&onlyDrop)||(manual&&onlyHover)||(manual&&hoverAndDrop)||(onlyDrop&&onlyHover)||(onlyDrop&&hoverAndDrop)||(onlyHover&&hoverAndDrop)) { // 4 input one-and-only-one high XNOR
    while(1) { // Locks out the drone if the mode is configured incorrectly
      digitalWriteFast(pRelaySwitch, HIGH);
      naze32.write(1200);
      delay(500);
      naze32.write(1000);
      delay(500);
    }
  }
  else if(manual == true) {
    while(1) { // For simple data gathering
      currentAlt = ultrasonicRead();
      doThrottle();
      writeSD();
    }
  }  
  else if(onlyDrop == true) {
    while(1) { // For objective 1 - drop the beanbag when at correct height 
      currentAlt = ultrasonicRead();
      doThrottle();
      checkDrop();
      writeSD();
    }
  }
  else if(onlyHover == true) {
    while(1) { // For objective 2 - hover at correct height
      currentAlt = ultrasonicRead();
      checkAndSet();
      doThrottle();
      writeSD();
    }
  }
  else if(hoverAndDrop == true) {
    while(1){ // For objective 3 - hover at correct height, drop beanbag, continue to hover
      currentAlt = ultrasonicRead();
      checkAndSet();
      doThrottle();
      checkDrop();
      writeSD();
    }
  }
}
 
void loop() {
} 
