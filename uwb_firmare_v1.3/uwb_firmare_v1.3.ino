// currently tag is module labeled #5
// This code calculates the (X,Y,Z) position in meters of a UWB tag, based on the known locations
// of four UWB anchors, labeled 1 to 4
// S. James Remington 1/2022

// This code does not average position measurements!

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>     // Calls inverse, determinant, LU decomp., etc.
#include <PixhawkArduinoMAVLink.h>
#include <HardwareSerial.h>
#include <RTClib.h> // Include the RTC library
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <EEPROM.h>
using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

float version = 1.3;

//wifi init
const char* ssid = "BUMBELBEE-TAG";
const char* password = "12345678";
WebServer server(80);

//pixhawk serial def
HardwareSerial &hs = Serial2;
PixhawkArduinoMAVLink mav(hs);

//GPS variable init
double ORIGIN_LATITUDE = 38.897580137902104;    // Latitude of the origin
double ORIGIN_LONGITUDE = -77.0365303701208;   // Longitude of the origin
RTC_DS3231 rtc; // Create an RTC_DS3231 object
float rot = 180;
const float latpm = 9.010063270126722e-06;
const float lonpm = 1.130896616413607e-05;
const uint32_t GPS_EPOCH_UNIX = 315964800; // Unix timestamp for GPS epoch (January 6, 1980)
const uint32_t SECONDS_IN_WEEK = 604800;   // Number of seconds in a week

// Debug
//#define DEBUG_TRILAT   //debug output in trilateration code
#define DEBUG_COMPARE_CURRENT_AVG   //debug output in trilateration code
#define DEBUG_GPS
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

// EEPROM INTI
#define EEPROM_FLOAT_SIZE sizeof(float)
#define EEPROM_DOUBLE_SIZE sizeof(double)
#define EEPROM_ANCHOR_START_ADDRESS 0
#define EEPROM_ANCHOR_SIZE (EEPROM_FLOAT_SIZE * 3)
#define EEPROM_ORIGIN_LATITUDE_ADDRESS (EEPROM_ANCHOR_START_ADDRESS + EEPROM_ANCHOR_SIZE * N_ANCHORS)
#define EEPROM_ORIGIN_LONGITUDE_ADDRESS (EEPROM_ORIGIN_LATITUDE_ADDRESS + EEPROM_DOUBLE_SIZE)
#define EEPROM_ROT_ADDRESS (EEPROM_ORIGIN_LONGITUDE_ADDRESS + EEPROM_DOUBLE_SIZE)

// anchor matrix defination
float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates
  {-1.30, 0.0, 1.21},
  {-1.30, 1.13, 1.21},
  {1.56, 1.47, 1.29},
  {-1.26, 0.0, 0.255}
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
MatrixXf Ptem(1,3);

// averaging number
int nk = 10;

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports


void saveAnchorPositionsToEEPROM() {
  int address = EEPROM_ANCHOR_START_ADDRESS;
  for (int i = 0; i < N_ANCHORS; i++) {
    for (int j = 0; j < 3; j++) {
      EEPROM.put(address, anchor_matrix[i][j]);
      address += EEPROM_FLOAT_SIZE;
    }
  }
  EEPROM.put(EEPROM_ORIGIN_LATITUDE_ADDRESS, ORIGIN_LATITUDE);
  EEPROM.put(EEPROM_ORIGIN_LONGITUDE_ADDRESS, ORIGIN_LONGITUDE);
  EEPROM.put(EEPROM_ROT_ADDRESS, rot);
  EEPROM.commit();
}

// Function to load anchor positions from EEPROM
void loadAnchorPositionsFromEEPROM() {
  int address = EEPROM_ANCHOR_START_ADDRESS;
  for (int i = 0; i < N_ANCHORS; i++) {
    for (int j = 0; j < 3; j++) {
      EEPROM.get(address, anchor_matrix[i][j]);
      address += EEPROM_FLOAT_SIZE;
    }
  }
    EEPROM.get(EEPROM_ORIGIN_LATITUDE_ADDRESS, ORIGIN_LATITUDE);
    EEPROM.get(EEPROM_ORIGIN_LONGITUDE_ADDRESS, ORIGIN_LONGITUDE);
    EEPROM.get(EEPROM_ROT_ADDRESS, rot);
}

void handleRoot() {
  String page = "<form action='/update' method='POST'>";
  page += "<h1>GPS Configuration </h1><br>";
  page += "<h3>Firmware version: v" + String(version) + "</h3><br>";
  page += "Latitude: <input type='text' name='latitude' value='" + String(ORIGIN_LATITUDE, 8) + "'><br>";
  page += "Longitude: <input type='text' name='longitude' value='" + String(ORIGIN_LONGITUDE, 8) + "'><br>";
  page += "Rotation: <input type='text' name='rotation' value='" + String(rot, 2) + "'><br>";
  // Input fields for each anchor's coordinates
  for (int i = 0; i < N_ANCHORS; i++) {
    page += "Anchor " + String(i+1) + ":<br>";
    page += "x: <input type='text' name='anchor_x_" + String(i) + "' value='" + String(anchor_matrix[i][0]) + "'><br>";
    page += "y: <input type='text' name='anchor_y_" + String(i) + "' value='" + String(anchor_matrix[i][1]) + "'><br>";
    page += "z: <input type='text' name='anchor_z_" + String(i) + "' value='" + String(anchor_matrix[i][2]) + "'><br>";
  }
  page += "<input type='submit' value='Update'>";
  page += "</form>";
  server.send(200, "text/html", page);
}



 void handleUpdate() {
   // Check if latitude, longitude, rotation, and anchor position values are provided in the request
   if (server.hasArg("latitude") && server.hasArg("longitude") && server.hasArg("rotation")) {
     // Retrieve latitude, longitude, and rotation values from the request
     double newLatitude = server.arg("latitude").toDouble();
     double newLongitude = server.arg("longitude").toDouble();
     double newRotation = server.arg("rotation").toDouble();
    
     // Update latitude, longitude, and rotation values
     ORIGIN_LATITUDE = newLatitude;
     ORIGIN_LONGITUDE = newLongitude;
     rot = newRotation;

     // Update anchor positions if provided in the request
     for (int i = 0; i < N_ANCHORS; i++) {
       String anchorXParam = "anchor_x_" + String(i);
       String anchorYParam = "anchor_y_" + String(i);
       String anchorZParam = "anchor_z_" + String(i);
      
       if (server.hasArg(anchorXParam) && server.hasArg(anchorYParam) && server.hasArg(anchorZParam)) {
         float newX = server.arg(anchorXParam).toFloat();
         float newY = server.arg(anchorYParam).toFloat();
         float newZ = server.arg(anchorZParam).toFloat();
        
         // Update anchor position in the anchor_matrix
         anchor_matrix[i][0] = newX;
         anchor_matrix[i][1] = newY;
         anchor_matrix[i][2] = newZ;
       }
     }
 
     // Save anchor positions to EEPROM
     saveAnchorPositionsToEEPROM();
    
     // Send a success response
     server.send(200, "text/plain", "Variables updated");
   } else {
     // If any of the required parameters is missing, send an error response
     server.send(400, "text/plain", "Invalid request");
   }
 }



void setup()
{
  Serial.begin(115200);
  delay(1000);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.begin();
  
  // check for EEPROM
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialize EEPROM");
    while (1); // Halt program execution
  }
  
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

  // Load anchor positions from EEPROM
  loadAnchorPositionsFromEEPROM();
#ifdef DEBUG_EEPROM
    Serial.print("### IN EEPROM ###");
    for (int i = 0; i < N_ANCHORS; i++) {
        Serial.print("(x");
        Serial.print(i + 1);
        Serial.print(", y");
        Serial.print(i + 1);
        Serial.print(", z");
        Serial.print(i + 1);
        Serial.print(") = (");
        Serial.print(anchor_matrix[i][0], 2); // print with 2 decimal places
        Serial.print(", ");
        Serial.print(anchor_matrix[i][1], 2);
        Serial.print(", ");
        Serial.print(anchor_matrix[i][2], 2);
        Serial.println(")");
    }
        Serial.print("origin_gps ");  //result current tag pos
        Serial.print(ORIGIN_LATITUDE, 8);
        Serial.write(',');
        Serial.print(ORIGIN_LONGITUDE, 8);
        Serial.write(',');
        Serial.println(rot, 2);
#endif

  //initialize configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

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
  server.handleClient();
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
    double rotateLatitude = ORIGIN_LATITUDE - Avg_tag_position[0] * cos(radians(rot)) * latpm + (Avg_tag_position[1] +0.055) * sin(radians(rot)) * latpm;
    double rotateLongitude = ORIGIN_LONGITUDE + Avg_tag_position[0] * sin(radians(rot)) * lonpm + (Avg_tag_position[1] + 0.055) * cos(radians(rot)) * lonpm;
    DateTime now = rtc.now();
    uint32_t gps_time_since_epoch = now.unixtime() - GPS_EPOCH_UNIX;
    uint16_t time_week = gps_time_since_epoch / SECONDS_IN_WEEK;
    uint32_t start_of_gps_week_unix = GPS_EPOCH_UNIX + (time_week * SECONDS_IN_WEEK);
    uint32_t time_week_ms = ((now.unixtime() - start_of_gps_week_unix) % SECONDS_IN_WEEK) * 1000; 
    mav.FakeGPS(rotateLatitude, rotateLongitude, Avg_tag_position[2], time_week, time_week_ms);
    #ifdef DEBUG_GPS
      Serial.print("GPS: ");  //result current tag pos
      Serial.print(rotateLatitude, 6);
      Serial.write(',');
      Serial.println(rotateLongitude, 6);
    #endif


    #ifdef DEBUG_COMPARE_CURRENT_AVG
      Serial.print("P= ");  //result current tag pos
      Serial.print(current_tag_position[0]);
      Serial.write(',');
      Serial.print(current_tag_position[1]);
      Serial.write(',');
      Serial.println(current_tag_position[2]);
      Serial.print("Pavg= ");  //average tag pos
      Serial.print(Avg_tag_position[0]);
      Serial.write(',');
      Serial.print(Avg_tag_position[1]);
      Serial.write(',');
      Serial.println(Avg_tag_position[2]);
      Serial.print("Diff= ");  //Difference betweem avg and current tag
      Serial.print(abs(Avg_tag_position[0] - current_tag_position[0]));
      Serial.write(',');
      Serial.print(abs(Avg_tag_position[1] - current_tag_position[1]));
      Serial.write(',');
      Serial.println(abs(Avg_tag_position[2] - current_tag_position[2]));
    #endif

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

float Gauss(int n) {
    float t = 0.0;
    for (int i = 0; i < n; i++) {
        t += (float)rand() / RAND_MAX; // Generate random number between 0 and 1
    }
    return t / n - 0.5; // Normalize and shift mean to 0
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
  Pavg << 0, 0, 0;

    for(k1=0; k1<nk; k1++) {//average several position estimates

      dn = Gauss(7);
      d[0] = sqrt(pow((current_tag_position[0] - x1), 2) + pow((current_tag_position[1] - Y11), 2) + pow((current_tag_position[2] - z1), 2)) + dn;
      dn = Gauss(7);
      d[1] = sqrt(pow((current_tag_position[0] - x2), 2) + pow((current_tag_position[1] - y2), 2) + pow((current_tag_position[2] - z2), 2)) + dn;
      dn = Gauss(7);
      d[2] = sqrt(pow((current_tag_position[0] - x3), 2) + pow((current_tag_position[1] - y3), 2) + pow((current_tag_position[2] - z3), 2)) + dn;
      dn = Gauss(7);
      d[3] = sqrt(pow((current_tag_position[0] - x4), 2) + pow((current_tag_position[1] - y4), 2) + pow((current_tag_position[2] - z4), 2)) + dn;
     
      b << (x2 * x2 + y2 * y2 + z2 * z2 - d[1] * d[1]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x3 * x3 + y3 * y3 + z3 * z3 - d[2] * d[2]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]),
           (x4 * x4 + y4 * y4 + z4 * z4 - d[3] * d[3]) - (x1 * x1 + Y11 * Y11 + z1 * z1 - d[0] * d[0]);
      Ptem = Ainv * b /2.0;
      Pavg += Ptem;
    }

    Avg_tag_position[0] = Pavg(0,0)/nk;
    Avg_tag_position[1] = Pavg(1,0)/nk;
    Avg_tag_position[2] = Pavg(2,0)/nk;

  return 1;
}  //end trilat3D_4A