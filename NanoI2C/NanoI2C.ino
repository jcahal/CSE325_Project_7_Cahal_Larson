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

float d = -1.00;
float a = -1.00;
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
void requestEvent() {
   String dStr = String(d, 2);
   String aStr = String(a, 2);

   switch(state) {
    case 1: { // send distance
      Wire.write(dStr.c_str());
      break;
    }
    case 2: { // send angle
      Wire.write(aStr.c_str());
      break;
    }
    case 3: { //master done with dist. and angle... reset
      d = -1.00;
      a = -1.00;  
    }
   }
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
       minDist = 3000;
    } else {
       if ( distance > 500) {
          d = distance;
          a = angle;
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
