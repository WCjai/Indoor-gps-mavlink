// currently tag is module labeled #5
// This code calculates the (X,Y,Z) position in meters of a UWB tag, based on the known locations
// of four UWB anchors, labeled 1 to 4
// S. James Remington 1/2022

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>     // Calls inverse, determinant, LU decomp., etc.
#include "Mersenne.h"  //random number generator
#include <PixhawkArduinoMAVLink.h>
#include <HardwareSerial.h>
#include <RTClib.h> // Include the RTC library
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

//pixhawk serial def
HardwareSerial &hs = Serial2;
PixhawkArduinoMAVLink mav(hs);
//GPS variable init
double ORIGIN_LATITUDE = 12.9730249;    // Latitude of the origin
double ORIGIN_LONGITUDE = 77.6462344;   // Longitude of the origin
RTC_DS3231 rtc; // Create an RTC_DS3231 object
float rot = 90;
const float latpm = 9.00000e-06;
const float lonpm = 0.919e-05; // Longitude per meter;
const uint32_t GPS_EPOCH_UNIX = 315964800; // Unix timestamp for GPS epoch (January 6, 1980)
const uint32_t SECONDS_IN_WEEK = 604800;   // Number of seconds in a week

// Debug
//#define DEBUG_TRILAT   //debug output in trilateration code
#define DEBUG_COMPARE_CURRENT_AVG   //debug output in trilateration code
//#define DEBUG_GPS
#define DEBUG_EEPROM


//DW1000 spi
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384

// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";
float current_tag_position[3] = {0}; //tag current position (meters with respect to origin anchor)
float Avg_tag_position[3] = {0}; //tag current position (meters with respect to origin anchor)

// variables for position determination
#define N_ANCHORS 4   //THIS VERSION WORKS ONLY WITH 4 ANCHORS. May be generalized to 5 or more.
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)


// anchor matrix defination
float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates
  {0.0, -1.22, 0.82},
  {2.92, 0.0, 0.84},
  {2.93, 6.80, 0.73},
  {2.78, 0.0, 0.05}
};



float x1 = anchor_matrix[0][0], Y11 = anchor_matrix[0][1], z1 = anchor_matrix[0][2];
float x2 = anchor_matrix[1][0], y2 = anchor_matrix[1][1], z2 =  anchor_matrix[1][2];
float x3 = anchor_matrix[2][0], y3 = anchor_matrix[2][1], z3 =  anchor_matrix[2][2];
float x4 = anchor_matrix[3][0], y4 = anchor_matrix[3][1], z4 =  anchor_matrix[3][2];
 

//matrix init
MatrixXf A(3,3);    // A
MatrixXf b(3,1);    // B
MatrixXf Ainv(3,3); // Ainv
MatrixXf P(1,3);    // resulting position
MatrixXf Pavg(1,3);    // resulting position
MatrixXf rc(1,3);
MatrixXf rcavg(1,3);    // resulting position
// averaging number
int nk = 10;
MTRand Random;  //required object for MTRand
float alpha = 0.8; //exponential filter weight

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports



void setup()
{
  Serial.begin(115200);
  delay(1000);
    //check for RTC
  while (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    delay(1000);
  }

  //check for FC
  while(!mav.begin()){
    Serial.println("Not Connected!");
    Serial.println(hs);
    delay(1000);
  }

  mav.Stream();
  delay(2000);

  //initialize configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

  //determine the Ainv matrix here itself 
  A << x2 - x1, y2 - Y11, z2 - z1,
       x3 - x1, y3 - Y11, z3 - z1,
       x4 - x1, y4 - Y11, z4 - z1;

  Ainv = (A.transpose() * A).inverse() * A.transpose();
  Pavg << 0, 0, 0;

}

void loop()
{
  DW1000Ranging.loop();

}

// collect distance data from anchors, presently configured for 4 anchors
// solve for position if all four beacons are current

void newRange()
{
  int i;

  //index of this anchor, expecting values 1 to 4
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07; //expect devices 1 to 7
  if (index > 0 && index < 5) {
    last_anchor_update[index - 1] = millis();  //(-1) => array index
    float range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance[index-1] = range;
    if (range < 0.0 || range > 30.0)     last_anchor_update[index - 1] = 0;  //sanity check, ignore this measurement
  }

  //check for four measurements within the last interval
  int detected = 0;  //count anchors recently seen

  for (i = 0; i < N_ANCHORS; i++) {

    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }
  if ( (detected == N_ANCHORS)) { //four recent measurements

    trilat3D_4A();
  

      // Serial.print("P= ");  //result current tag pos
      // Serial.print(current_tag_position[0]);
      // Serial.write(',');
      // Serial.print(current_tag_position[1]);
      // Serial.write(',');
      // Serial.print(current_tag_position[2]);
    double rotateLatitude = ORIGIN_LATITUDE - Avg_tag_position[0] * cos(radians(rot)) * latpm + (Avg_tag_position[1] +0.055) * sin(radians(rot)) * latpm;
    double rotateLongitude = ORIGIN_LONGITUDE + Avg_tag_position[0] * sin(radians(rot)) * lonpm + (Avg_tag_position[1] + 0.055) * cos(radians(rot)) * lonpm;
    DateTime now = rtc.now();
    uint32_t gps_time_since_epoch = now.unixtime() - GPS_EPOCH_UNIX;
    uint16_t time_week = gps_time_since_epoch / SECONDS_IN_WEEK;
    uint32_t start_of_gps_week_unix = GPS_EPOCH_UNIX + (time_week * SECONDS_IN_WEEK);
    uint32_t time_week_ms = ((now.unixtime() - start_of_gps_week_unix) % SECONDS_IN_WEEK) * 1000;
    unsigned long GPScurrentMillis = millis();
    float alt  = Avg_tag_position[2]+980.2365;
    mav.FakeGPS(rotateLatitude, rotateLongitude, Avg_tag_position[2], time_week, time_week_ms);
    Serial.print(" Pavg= ");  //average tag pos
    Serial.print(Avg_tag_position[0]);
    Serial.write(',');
    Serial.print(Avg_tag_position[1]);
    Serial.write(',');
    Serial.println(Avg_tag_position[2]);

  }
}  //end newRange

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

float Gauss (int n) {
    int i;
    float t=0.0;
    for(i=0; i<n; i++) {
    t += genRand(&Random)-0.5;;
    }
    return t/n;
}

int trilat3D_4A(void) {

  int i;
  int k1;
  float d[N_ANCHORS];
  float dn;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];
  #ifdef DEBUG_TRILAT
     char line[60];
     snprintf(line, sizeof line, "d1 = %6.2f,d2 = %6.2f,d3 = %6.2f,d4 = %6.2f", d[0], d[1], d[2], d[3]);
     Serial.println(line);
  #endif

    //b matrix
  b << (x2 * x2 + y2 * y2 + z2 * z2 - d[1] * d[1]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
       (x3 * x3 + y3 * y3 + z3 * z3 - d[2] * d[2]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
       (x4 * x4 + y4 * y4 + z4 * z4 - d[3] * d[3]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]);
  

  P = Ainv * b /2.0;
  current_tag_position[0] = P(0,0);
  current_tag_position[1] = P(1,0);
  current_tag_position[2] = P(2,0);
  rc << 0, 0, 0;
  rcavg << 0, 0, 0;
  Pavg << 0, 0, 0;

    for(k1=0; k1<nk; k1++) {//average several position estimates

      for (int i = 0; i < N_ANCHORS; i++) {
        //  double dx = current_tag_position[0] - anchor_matrix[i][0];
        //  double dy = current_tag_position[1] - anchor_matrix[i][1];
        //  double dz = current_tag_position[2] - anchor_matrix[i][2];
         double dx = anchor_matrix[i][0] - current_tag_position[0];
         double dy = anchor_matrix[i][1] - current_tag_position[1];
         double dz = anchor_matrix[i][2] - current_tag_position[2];
         double dn = Gauss(7);  // Generate +/- 0.1 rms noise
         d[i] = sqrt(dx * dx + dy * dy + dz * dz) + dn;
      } 
      
      b << (x2 * x2 + y2 * y2 + z2 * z2 - d[1] * d[1]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x3 * x3 + y3 * y3 + z3 * z3 - d[2] * d[2]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x4 * x4 + y4 * y4 + z4 * z4 - d[3] * d[3]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]);
      rc = Ainv * b * 0.5;

      if (k1==0) {
        rcavg = rc;
      }
      else {
        rcavg += alpha*(rc - rcavg);
      }

    }

    Avg_tag_position[0] = rcavg(0,0);
    Avg_tag_position[1] = rcavg(1,0);
    Avg_tag_position[2] = rcavg(2,0);

  return 1;
}  //end trilat3D_4A