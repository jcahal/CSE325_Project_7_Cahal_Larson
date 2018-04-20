#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object DO NOT USE Serial0
DFR_Key keypad;                               // define keypad object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object

#define GPSECHO  false                        // echo GPS Sentence 
#define Threshold 5                           // Threshold for Obstacle avoidance (number of obstacles)
#define SLAVE_ADDR 8

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that change across functions
float heading = 0;          // Heading in degree
imu::Vector<3> euler;                         // Vector of IMU
float bearing = 0;          // initialize bearing
int sa = 90;        // servo initial angle (range is 0:180)
int carSpeed = 10;
boolean usingInterrupt = false;
int carSpeedPin = 2;              // pin for DC motor (PWM for motor driver). don't use other pins....
float errorHeadingRef = 0;        // error
long int lat;                     // GPS latitude in degree decimal * 100000   |     we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
long int lon;                     // GPS latitude in degree decimal * 100000   |     0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
long int latDestination = 33.423716 * 100000; //33.425891 * 100000;       // define an initial reference Latitude of destination
long int lonDestination =  -111.939204 * 100000; //-111.940458 * 100000;    // define an initial reference Longitude of destination
int localkey = 0;                                   // var
int state = 0;

void setup() {
  myservo.attach(44);     // servo is connected to pin 44     (All pins are used by LCD except 2. Pin 2 is used for DC motor)
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)

  Wire.begin();
  Serial.begin(9600);     // serial for monitoring
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // Use your CALIBRATION DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);


  // GPS 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // ask GPS to send RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate, don't use higher rates
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna is detected
  useInterrupt(true);                           // use interrupt for reading chars of GPS sentences (From Serial Port)

  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);
  }

  GPSRead();
  latDestination = lat;     // saving the destiantion point
  lonDestination = lon;     // saving the destiantion point
  localkey = 0;

  // Steering and Actuate interupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt
  interrupts();
}

SIGNAL(TIMER0_COMPA_vect) {       // don't change this !!
  char c = GPS.read();            // interrupt for reading GPS sentences char by char....
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {    // enable inttrupt for GPS, don't change this!!
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis() - we'll just interrupt somewhere
    TIMSK0 |= _BV(OCIE0A);      // in the middle by Output Compare Register A (OCR0A) and "TIMER0_COMPA_vect" function is called
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A); // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS DATA
  sei();        //   reset interrupt flag
  TCNT4  = 336; //   re-initialize timer value
  GPSRead();    //   read GPS data
  Serial.println();
  Serial.print("Read GPS");
  Serial.print(" Runtime: ");
  Serial.println(millis() / 1000);
}

void GPSRead() {
  // read GPS data
    // read from GPS module and update the current position
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  lat = GPS.latitudeDegrees * 100000;
  lon = GPS.longitudeDegrees * 100000;
}


void ReadHeading()
{
  // calculate HEADING
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  heading = euler.x() + 10.37;
}

void CalculateBearing() {
  bearing = heading; //go straight
  // Calculate Bearing base on GPS Destination
  /*
  bearing = 90 - atan2((latDestination - lat),(lonDestination - lon)) * (180 / PI);
  if(bearing < 0) {
    bearing += 360;
  }
  */
}

void CalculateSteer() {
  // Calculate Steer angle based on GPS data and IMU
  switch(state) {
    case 0: {
      float dr;
      float dl;
      
      if (heading > 360)
        heading -= 360;
  
      if(heading > bearing) {
        dl = heading - bearing;
        dr = (360 - heading) + bearing; 
      } else {
        dr = bearing - heading;
        dl = (360 - bearing) + heading;
      }
    
      if (dl < dr) { 
        sa = 60; // turn left all the way, it's closer...
        
        if(dl < 45)
         sa = 70;
    
        if(dl < 30)
         sa = 80;
    
        if(dl < 15)
         sa = 85; 
      } else {
        sa = 125; // turn right, it's closer...
    
        if(dr < 45)
         sa = 115;
    
        if(dr < 30)
         sa = 105;
    
        if(dr < 15)
         sa = 100;
      }
    
      if(heading >= bearing - 2 && heading <= bearing + 2) {
        sa = 93; // go straight
      } 
      break;
    }
    case 1: {
      sa = 100; 
      break;
    }
    case 2: {
      sa = 105;
      break;
    }
    case 3: {
      sa = 115;
      break;
    }
    case 4: {
      sa = 70;
      break;
    }
    case 5: {
      sa = 80;
      break;
    }
    case 6: {
      sa = 84;
      break;
    }
    case default: {
      break;
    }
  }
}

void SetCarDirection() {    // Input: Lidar data
  // Set Steering angle,
  // If any obstacle is detected by Lidar, Ignore steering angle and turn left or right based on observation
  // Filter Objs
  
}

void SetCarSpeed() {  // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
  analogWrite(carSpeedPin, carSpeed); //change to carSpeed for production
  myservo.write(sa);
  
}

void ReadLidar() { 
  // Get state 
  Wire.requestFrom(SLAVE_ADDR, 1);
  state = Wire.read();
}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                  // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  ReadLidar();
  CalculateBearing();
  CalculateSteer();
  //SetCarDirection(); //nothing in it
  SetCarSpeed();
}


void printHeadingOnLCD() {

}

void printLocationOnLCD() {

}

void printDistanceOnLCD() {
  
}

void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear lcd
  // Pring data to LCD in order to debug your program!
  printHeadingOnLCD();
  printObstacleOnLCD();
  delay(100);
}
