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
// Origin latitude, longitude, and altitude (geodetic coordinates)
const double ORIGIN_LATITUDE = 12.9730249;
const double ORIGIN_LONGITUDE = 77.6462344;
const double ORIGIN_ALTITUDE = 921.0062; // Assuming origin altitude at MSL
// Rotation angles in degrees around the x, y, and z axes
double rotation_angle_x = 13; // Example rotation angle around x-axis
double rotation_angle_y = 0; // Example rotation angle around y-axis
double rotation_angle_z = -12.354; // Example rotation angle around z-axis

const double a = 6378137.0;  // Semi-major axis in meters
const double f = 1.0 / 298.257223563;  // Flattening
const double e2 = f * (2 - f);  // Square of first eccentricity
const double b = a * (1 - f);  // Semi-minor axis in meters



RTC_DS3231 rtc; // Create an RTC_DS3231 object
uint64_t time_usec;
const float latpm = 9.010063270126722e-06;
const float lonpm = 1.130896616413607e-05;
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
MatrixXf c(3,1);    // B
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


// Convert geodetic to ECEF coordinates
void geodetic_to_ecef(double lat, double lon, double h, double& X, double& Y, double& Z) {
    double lat_rad = radians(lat);
    double lon_rad = radians(lon);

    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

    X = (N + h) * cos(lat_rad) * cos(lon_rad);
    Y = (N + h) * cos(lat_rad) * sin(lon_rad);
    Z = (N * (1 - e2) + h) * sin(lat_rad);
}

// Convert ECEF to geodetic coordinates using an iterative method for increased accuracy
void ecef_to_geodetic(double X, double Y, double Z, double& lat, double& lon, double& h) {
    lon = atan2(Y, X);
    
    double p = sqrt(X * X + Y * Y);
    lat = atan2(Z, p * (1 - e2)); // Initial latitude approximation
    
    double lat_prev = 0;
    h = 0;
    
    while (abs(lat - lat_prev) > 1e-12) { // Convergence threshold
        lat_prev = lat;
        double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
        h = p / cos(lat) - N;
        lat = atan2(Z + e2 * N * sin(lat), p);
    }
    
    lat = degrees(lat);
    lon = degrees(lon);
}

// Define the rotation matrix for a given angle (in degrees) around the x-axis
void rotation_matrix_x(double angle_deg, double R[3][3]) {
    double angle_rad = radians(angle_deg);
    R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
    R[1][0] = 0; R[1][1] = cos(angle_rad); R[1][2] = -sin(angle_rad);
    R[2][0] = 0; R[2][1] = sin(angle_rad); R[2][2] = cos(angle_rad);
}

// Define the rotation matrix for a given angle (in degrees) around the y-axis
void rotation_matrix_y(double angle_deg, double R[3][3]) {
    double angle_rad = radians(angle_deg);
    R[0][0] = cos(angle_rad); R[0][1] = 0; R[0][2] = sin(angle_rad);
    R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
    R[2][0] = -sin(angle_rad); R[2][1] = 0; R[2][2] = cos(angle_rad);
}

// Define the rotation matrix for a given angle (in degrees) around the z-axis
void rotation_matrix_z(double angle_deg, double R[3][3]) {
    double angle_rad = radians(angle_deg);
    R[0][0] = cos(angle_rad); R[0][1] = -sin(angle_rad); R[0][2] = 0;
    R[1][0] = sin(angle_rad); R[1][1] = cos(angle_rad); R[1][2] = 0;
    R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;
}

// Apply rotation to the local coordinates around all three axes
void rotate_coordinates(double x, double y, double z, double angle_x, double angle_y, double angle_z, double& rx, double& ry, double& rz) {
    double coordinates[3] = {x, y, z};
    double R_x[3][3], R_y[3][3], R_z[3][3], R[3][3];

    rotation_matrix_x(angle_x, R_x);
    rotation_matrix_y(angle_y, R_y);
    rotation_matrix_z(angle_z, R_z);

    // Multiply rotation matrices: R = R_z * R_y * R_x
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                R[i][j] += R_z[i][k] * R_y[k][j];
            }
        }
    }

    double temp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += R[i][k] * R_x[k][j];
            }
        }
    }

    // Apply the rotation
    rx = temp[0][0] * coordinates[0] + temp[0][1] * coordinates[1] + temp[0][2] * coordinates[2];
    ry = temp[1][0] * coordinates[0] + temp[1][1] * coordinates[1] + temp[1][2] * coordinates[2];
    rz = temp[2][0] * coordinates[0] + temp[2][1] * coordinates[1] + temp[2][2] * coordinates[2];
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
    //check for RTC
  while (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    delay(1000);
  }

  if (rtc.lostPower()) {
        // If the RTC has lost power, set the time to a specific date & time
        Serial.println("RTC lost power, setting the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
        // rtc.adjust(DateTime(2024, 6, 8, 12, 0, 0)); // Set to specific date & time
    }

    DateTime now = rtc.now();
    time_usec = now.unixtime() * 1000000ULL; // Convert to microseconds

    Serial.print("Timestamp: ");
    Serial.println(time_usec);

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
    double enu_x = -Avg_tag_position[0];
    double enu_y = Avg_tag_position[2];
    double enu_z = Avg_tag_position[1];
    double rotated_x, rotated_y, rotated_z;
    rotate_coordinates(enu_x, enu_y, enu_z, rotation_angle_x, rotation_angle_y, rotation_angle_z, rotated_x, rotated_y, rotated_z);
    // Convert origin to ECEF coordinates
    double origin_X, origin_Y, origin_Z;
    geodetic_to_ecef(ORIGIN_LATITUDE, ORIGIN_LONGITUDE, ORIGIN_ALTITUDE, origin_X, origin_Y, origin_Z);

    // Translate rotated local ENU coordinates to ECEF coordinates
    double ecef_X = origin_X + rotated_x;
    double ecef_Y = origin_Y + rotated_y;
    double ecef_Z = origin_Z + rotated_z;

    // Convert translated ECEF coordinates to geodetic coordinates
    double latitude, longitude, altitude;
    ecef_to_geodetic(ecef_X, ecef_Y, ecef_Z, latitude, longitude, altitude);

    DateTime now = rtc.now();
    time_usec = now.unixtime() * 1000000ULL + now.secondstime() % 1 * 1000000;
    uint32_t gps_time_since_epoch = now.unixtime() - GPS_EPOCH_UNIX;
    uint16_t time_week = gps_time_since_epoch / SECONDS_IN_WEEK;
    uint32_t start_of_gps_week_unix = GPS_EPOCH_UNIX + (time_week * SECONDS_IN_WEEK);
    uint32_t time_week_ms = ((now.unixtime() - start_of_gps_week_unix) % SECONDS_IN_WEEK) * 1000;
    unsigned long GPScurrentMillis = millis();
    mav.FakeGPS(latitude, longitude, altitude , time_week, time_week_ms, time_usec);




    Serial.print("GPS= ");  //average tag pos
    Serial.print(latitude, 8);
    Serial.write(',');
    Serial.print(longitude, 8);
    Serial.write(',');
    Serial.println(altitude, 8);
    Serial.print(" Pavg= ");  //average tag pos
    Serial.print(Avg_tag_position[0], 8);
    Serial.write(',');
    Serial.print(Avg_tag_position[1], 8);
    Serial.write(',');
    Serial.println(Avg_tag_position[2], 8);

    Serial.print(current_tag_position[0], 8);
    Serial.write(',');
    Serial.print(current_tag_position[1], 8);
    Serial.write(',');
    Serial.println(current_tag_position[2], 8);

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
  c << (x2 * x2 + y2 * y2 + z2 * z2 - d[1] * d[1]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
       (x3 * x3 + y3 * y3 + z3 * z3 - d[2] * d[2]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
       (x4 * x4 + y4 * y4 + z4 * z4 - d[3] * d[3]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]);
  

  P = Ainv * c /2.0;
  current_tag_position[0] = P(0,0);
  current_tag_position[1] = P(1,0);
  current_tag_position[2] = P(2,0);
  rc << 0, 0, 0;
  rcavg << 0, 0, 0;
  Pavg << 0, 0, 0;

    for(k1=0; k1<nk; k1++) {//average several position estimates

      for (int i = 0; i < N_ANCHORS; i++) {

         double dx = anchor_matrix[i][0] - current_tag_position[0];
         double dy = anchor_matrix[i][1] - current_tag_position[1];
         double dz = anchor_matrix[i][2] - current_tag_position[2];
         double dn = Gauss(7);  // Generate +/- 0.1 rms noise
         d[i] = sqrt(dx * dx + dy * dy + dz * dz) + dn;
      } 
      
      c << (x2 * x2 + y2 * y2 + z2 * z2 - d[1] * d[1]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x3 * x3 + y3 * y3 + z3 * z3 - d[2] * d[2]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x4 * x4 + y4 * y4 + z4 * z4 - d[3] * d[3]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]);
      rc = Ainv * c * 0.5;

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