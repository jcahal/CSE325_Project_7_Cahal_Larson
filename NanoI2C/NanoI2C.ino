#include <Wire.h>
#include <RPLidar.h>
#include <SoftwareSerial.h>

RPLidar lidar;          // define lidar as RPLIDAR Object
#define RPLIDAR_MOTOR 3 // motor pin for lidar speed control

int left = 0;                     // variable for detected points on left hand side
int right = 0;                    // variable for detected points on right hand side
unsigned long time = millis();    // time variable for reseting variables

int c1;                           // variable for received integer

float minDist = 100000;       // minimum distance an object has to be to trigger response
float aMinDist = 0;         // angle of the object when at minimum distance

float ld = -1.00;
float la = -1.00;
float rd = -1.00;
float ra = -1.00;
int state = 0;

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
}

// This is when the master is writing to salve
void receiveEvent(int bytes)
{
  
  // read the received byte as integer. This indicates what data to send back when master is requesting data
  state = Wire.read();
          
}

// This is when the master wants the slave to write something
void requestEvent() 
{
  
   // receive message byte as a character
   // if master's request is right side data, ("1"), send back the right side data
   // if master's request is left side data, ("2"), send back the left side data
   String rdStr = String(rd, 2);
   String raStr = String(ra, 2);
   String ldStr = String(ld, 2);
   String laStr = String(la, 2);

   switch(state) {
    case 1: {
      Wire.write(rdStr.c_str());
      break;
    }
    case 2: {
      Wire.write(raStr.c_str());
      break;
    }
    case 3: {
      Wire.write(ldStr.c_str());
      break;
    }
    case 4: {
      Wire.write(laStr.c_str());
      break;
    }
   }

   rd = -1.00;
   ra = -1.00;
   ld = -1.00;
   la = -1.00;
}

void loop() 
{
  if (IS_OK(lidar.waitPoint())) { // if lidar is working properly (waiting time less than timeout)
    // read angle and distance of the obstacle
    // filter data (keep only the data in desired range and with desired angle) 
    // COUNT the number of obstacles on LEF and RIGHT side
    // reset obstacle variables every 1 second

    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    
    if (lidar.getCurrentPoint().startBit) {
      // a new scan, display the previous data...
       minDist = 100000;
       aMinDist = 0;
    } else {
       if ( distance > 0 &&  distance < minDist) {
          minDist = distance;
          aMinDist = angle;

          //what side of the car is it
          if(angle > 270 && angle < 350) { // left side
            ld = distance;
            la = angle;
          } else if(angle > 10 && angle < 90) { // right side
            rd = distance;
            ra = angle;
          }
       }
    }
    
  } else {                                                  // if lidar is not responding           // Dont change this......
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor                // Dont change this......
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR...             // Dont change this......
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
      lidar.startScan();                                    // start scan                           // Dont change this......
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed    // Dont change this......
      delay(1000);                                                                                  // Dont change this......
    }
  }
}
