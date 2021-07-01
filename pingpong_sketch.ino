/*Ping Pong PID Balance
 * March 2020
 * P Ferrell
 * Inspired by: http://www.electronoobs.com/eng_arduino_tut100.php
 * */
 
#include <SharpIR.h>  //https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/
#include <Servo.h>
#include <Adafruit_NeoPixel.h>  //https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use

#define irPin A0  // IR sensor attached on pin A0
#define irModel 1080  // Sharp IR sensor model GP2Y0A21YK0F
SharpIR SharpIR(irPin,irModel);

#define servoPin 9  // servo motor attached on pin 9
Servo myServo;
int servoDefault = 110; // nominal default position for servo to hold balance rails level
int servoLowerLimit = 20;
int servoUpperLimit = 150;
int servoCorrection = 30;  // value to offset servo if needed

#define NeoPixelPin 6 // NeoPixel control wire attached on pin 6
#define NeoPixelCount 11 // number of NeoPixels on strand
Adafruit_NeoPixel strip(NeoPixelCount, NeoPixelPin, NEO_GRB + NEO_KHZ800);
uint32_t defaultPixelColor = strip.Color(127,127,127); // default color for indicator pixel

// Connections for manual controls
#define manualOveridePin 10 // switch for manual overide
#define kpOnPin 11 // switch to turn on manual Kp
#define kiOnPin 12 // switch to turn on manual Ki
#define kdOnPin 13 // switch to turn on manual Kd
#define setpointInputPin A1 // analog input for setpoint potentiometer
#define kpInputPin A2 // analog input for Kp potentiometer
#define kiInputPin A3 // analog input for Ki potentiometer
#define kdInputPin A4 // analog input for Kd potentiometer
int manualOverideState = 0;
int kpOnState = 0;
int kiOnState = 0;
int kdOnState = 0;


// Physical Constants - **ALL UNITS IN CENTIMETERS (cm)**
float ballDiameter = 0; // (40mm is standard for ping pong balls) **not currently used in code**

// manual setpoint parameters
int midpointIndex = 5; // 6th pixel in strand out of 11
float discreteSetpoints[NeoPixelCount] = {11,7,11,15,19,24,29,33,37,42,46}; //distances in integer cm; note that the first setpoint [0] is larger due to non-linearity of senor; these values will be determined experimentally
int newSetpointIndex = 100; //out of bounds value for initialization

// Default PID Constants  //
float default_Kp = 8;
float default_Ki = 0.2;
float default_Kd = 3500;
float default_setpoint = discreteSetpoints[midpointIndex];
int period = 50;  // refresh rate period of the loop in ms - a guess, and perhaps slightly different for 'manual' vs. 'auto' modes

// PID variables
float Kp; // proportional 
float Ki; // integral 
float Kd; // derivative 
float setpoint = discreteSetpoints[midpointIndex];
float dist; // distance from sensor to ball surface **not ceter of ball**
float distanceError = 0;  // distance from setpoint
float distanceErrorPrevious = 0;
float PID_iWindow = 14; // distanceError must be within +/- this window in cm for PID_i to engage
float PID_p, PID_i, PID_d, PID_total;  // PID correction factors



void setup() {
  pinMode (irPin, INPUT);
  pinMode (setpointInputPin, INPUT);
  pinMode (kpInputPin, INPUT);
  pinMode (kiInputPin, INPUT);
  pinMode (kdInputPin, INPUT);
  pinMode (manualOveridePin, INPUT);
  pinMode (kpOnPin, INPUT);
  pinMode (kiOnPin, INPUT);
  pinMode (kdOnPin, INPUT);

  strip.begin();
  strip.show(); // initialize all NeoPixels to 'off'
  
  myServo.attach(servoPin);
  myServo.write(servoDefault);  // put servo in default position, which should make balance level

}

void loop() {

manualOverideState = digitalRead(manualOveridePin);
if (manualOverideState == HIGH) // 'manual mode' - setpoint, and Kp, Ki, Kd all adjusted from control panel
{
  strip.clear(); 
  int setpointValue = analogRead(setpointInputPin);
  int setpointIndex = map(setpointValue,0,1023,0,10);
  setpoint = discreteSetpoints[setpointIndex];
  strip.setPixelColor(setpointIndex,defaultPixelColor);
  if (newSetpointIndex != setpointIndex) {strip.clear();}
  strip.show();
  newSetpointIndex = setpointIndex;

// set manual Kp
  kpOnState = digitalRead(kpOnPin);
  if (kpOnState == HIGH)
  {
    int KpMultiplier = analogRead(kpInputPin);
    KpMultiplier = map(KpMultiplier, 0,1023,2000,0);
    Kp = default_Kp*KpMultiplier/1000;
  }
  else
  {
    Kp = 0;
  }

// set manual Ki
  kiOnState = digitalRead(kiOnPin);
  if (kiOnState == HIGH)
  {
    int KiMultiplier = analogRead(kiInputPin);
    KiMultiplier = map(KiMultiplier, 0,1023,2000,0);
    Ki = default_Ki*KiMultiplier/1000;
  }
  else
  {
    Ki = 0;
  }
  
// set manual Kd
  kdOnState = digitalRead(kdOnPin);
  if (kdOnState == HIGH)
  {
    int KdMultiplier = analogRead(kdInputPin);
    KdMultiplier = map(KdMultiplier, 0,1023,2000,0);
    Kd = default_Kd*KdMultiplier/1000;
  }
  else
  {
    Kd = 0;
  }
}
else  // manual switch is off and software operating in default auto mode
{
  strip.clear();
  setpoint = default_setpoint;
  Kp = default_Kp;
  Ki = default_Ki;
  Kd = default_Kd;
  strip.setPixelColor(midpointIndex,defaultPixelColor);
  strip.show();
}

  dist = GetDistance(5);  //get distance from IR sensor in cm - input parameter is number of samples to take in average
  distanceError = setpoint - dist;
  PID_p = Kp * distanceError;
  if((-1*PID_iWindow)<distanceError && distanceError<PID_iWindow)
  {
    PID_i = PID_i + (Ki*distanceError);
  }
  else
  {
    PID_i = 0;
  }
  PID_d = Kd*(distanceError-distanceErrorPrevious)/period;
  PID_total = PID_p + PID_i + PID_d;
  distanceErrorPrevious = distanceError;
  ApplyCorrection(PID_total);
}

void ApplyCorrection(float PID_total)  // I pulled this code from electronoobs, but don't fully understand it yet
{
  PID_total = map(PID_total,-150,150,0,160);
  if(PID_total < servoLowerLimit) {PID_total = servoLowerLimit;}
  if(PID_total > servoUpperLimit) {PID_total = servoUpperLimit;}
  myServo.write(PID_total+servoCorrection);
}

float GetDistance(int sampleSize)
// returns distance in integer cm - this is the resolution of the SharpIR sensor
{
  long sum=0;
  for(int i=0;i<sampleSize;i++)
  {
    sum = sum+SharpIR.distance();
  }
  float dist = sum/sampleSize;
  return(dist);
}
