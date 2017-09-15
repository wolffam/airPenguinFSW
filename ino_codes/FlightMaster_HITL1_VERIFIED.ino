///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// VERSION HISTORY //
// YYYY-MM-DD/HH:MM - Version - Description of changes (Author)
// 2016-05-18/13:31 - V1.0 - Initial Upload of Flight Master (Alex Wolff)
// 2016-05-18/17:17 - V1.1 - Changed Pyro1/MOXV1 timings; Added Main loop details
// 2016-05-19/17:49 - V1.2 - Added SoftAbort, OpenFVV, CloseFVV
// 2016-05-19/23:52 - V1.3 - Changed equality sign, added comments
// 2016-05-20/04:28 - V1.4 - Added SetupSD, WriteSD and incorporated into master loop
// 2016-05-21/14:49 - V1.5 - Added POST HITL V1.0, PRE HITL V2.0
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////////////////////////////
// SD CARD LIBRARY //
#include <SD.h>
// SPI LIBRARY //
#include <SPI.h>  // TODO: IS THIS NECESSARY?

// PRESSURE SENSOR LIBRARY //
#include <Adafruit_BMP085.h>

//IMU LIBRARIES //
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// GENERAL LIBRARIES //
#include <Wire.h>
#include <math.h>
/////////////////////////////


// STATE DEFINITIONS //
// PRELAUNCH 0 //
// LAUNCH    1 //
// FLIGHT_1  2 //
// COAST     3 //
// FLIGHT_2  4 //
// RECOVERY  5 //
// SAFE      6 //
///////////////////////
int STATE = 0; // INITIALIZE TO PRELAUNCH STATE
///////////////////////

// FVV COUNTERS //
////////////////////
int FVVOpenNum  = 0;
int FVVCloseNum = 0;
////////////////////

// DEFINE PINS FOR ACTUATIONS //
#define MOXV01_PIN 2
#define MOXV02_PIN 3
#define MOXV03_PIN 4
#define SMOXV_PIN  5
#define PYRO_1_PIN 6
#define PYRO_2_PIN 7
#define FVV_PIN    8
#define QDSEP_PIN  9
#define RD_LAUNCH  10
#define RD_ABORT   13
#define RD_ABORT_SOFT 12

#define RD_FVV        25
#define WR_ERROR      26
  
// DEFINE ACTIVE LOW RELAY CONFIG //
#define RELAY_ON  0
#define RELAY_OFF 1

// PRESSURE SENSOR DEFINITIONS //
Adafruit_BMP085 bmp;
int trueSeaLevelPress;
const int chipSelect = 4;
////////////////////////////////

// IMU DEFINITIONS //
Adafruit_BNO055 bno = Adafruit_BNO055();

// GLOBAL VARIABLES //
int initMillis = millis();  
int currentMillis = millis();

void setup() {

  // SET INITIAL STATE TO CLOSE FOR ALL RELAYS //
  pinMode(MOXV01_PIN, OUTPUT);
  digitalWrite(MOXV01_PIN,RELAY_OFF);
  pinMode(MOXV02_PIN, OUTPUT);
  digitalWrite(MOXV02_PIN,RELAY_OFF);
  pinMode(MOXV03_PIN, OUTPUT);
  digitalWrite(MOXV03_PIN,RELAY_OFF);
  pinMode(SMOXV_PIN, OUTPUT);
  digitalWrite(SMOXV_PIN,RELAY_OFF);
  pinMode(PYRO_1_PIN, OUTPUT);
  digitalWrite(PYRO_1_PIN,RELAY_OFF);
  pinMode(PYRO_2_PIN, OUTPUT);
  digitalWrite(PYRO_2_PIN,RELAY_OFF);
  pinMode(FVV_PIN, OUTPUT);
  digitalWrite(FVV_PIN,RELAY_OFF);
  pinMode(QDSEP_PIN, OUTPUT);
  digitalWrite(QDSEP_PIN,RELAY_OFF);
  pinMode(WR_ERROR, OUTPUT);
  digitalWrite(WR_ERROR,RELAY_OFF);

  // INITIALIZE READ PINS AS INPUTS //
  pinMode(RD_LAUNCH, INPUT);
  pinMode(RD_ABORT,  INPUT);
  pinMode(RD_ABORT_SOFT, INPUT);
  pinMode(RD_FVV,  INPUT);

  int MasterInitMillis = millis(); // CURRENTLY NOT USED

  // INITIALIZE BMP,BNO, AND SD DEVICES
  //SetupSD(); 

}

void loop() {
  Serial.print("STATE = ");
  Serial.println(STATE);
  int LoopMillis = millis();
  OpenFVV();  // CHECKS FOR OPEN FVV COMMAND
  CloseFVV(); // CHECKS FOR CLOSE FVV COMMAND

  //WriteSD(); // WRITE TELEMETRY TO SD CARD
  
  STATE = HardAbortCheck(); // CHECKS FOR HARD ABORT SIGNAL
  STATE = SoftAbortCheck(); // CHECKS FOR SOFT ABORT SIGNAL
  //IF ABORT IS NOT IN EFFECT, CONFIRM THAT LAUNCH PIN HIGH LASTS A FULL SECOND //  
  if (digitalRead(RD_LAUNCH) == HIGH && digitalRead(RD_ABORT) == LOW && STATE != 6){

    int currLaunchMillis = millis();
    int falseLaunchPinRead = 0;
    
    while ((currLaunchMillis - LoopMillis) < 100) {

      if (digitalRead(RD_LAUNCH) == LOW) {

        falseLaunchPinRead = 1;
        break;
      }
      currLaunchMillis = millis();
    }
    
    if ((falseLaunchPinRead == 0) && (digitalRead(RD_ABORT)) == LOW) {
      STATE = 1; // SEND STATE TO LAUNCH

      STATE = LaunchSeq();
      Serial.print("STATE = ");
      Serial.println(STATE);
    }
  }

  // END LAUNCH PIN HIGH CONTINUITY CHECK //
  // POSSIBLE OUTPUTS: (1) RETURN TO PREVIOUS STATE //
  //                   (2) ENTER LAUNCH STATE // 

  if (STATE == 2) {
    STATE = Flight1Seq();
    Serial.print("STATE = ");
      Serial.println(STATE);
  }

  if (STATE == 3) {
    STATE = CoastSeq();
    Serial.print("STATE = ");
      Serial.println(STATE);
  }

  if (STATE == 4) {
    STATE = Flight2Seq();
    Serial.print("STATE = ");
      Serial.println(STATE);
  }
  
  if (STATE == 5) {
    RecoverySeq();
    Serial.print("STATE = ");
      Serial.println(STATE);
  }

  if (STATE == 6) {
    SafeSeq();
    Serial.print("STATE = ");
      Serial.println(STATE);
  }

  

}


//// LAUNCH SEQUENCING FUCTION ////
int LaunchSeq(){

  int LaunchSeqEnd      = 0;
  int startSeqCounter   = 0;
  int pyro1OpenCounter  = 0;
  int pyro1CloseCounter = 0;
  int moxv1OpenCounter  = 0;
  int qdsepOffCounter   = 0;
  initMillis = millis();

  while (LaunchSeqEnd == 0) {

    Serial.print("LAUNCH STATE = ");
    Serial.println(STATE);

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    // AFTER *30 SECONDS*, ACTUAL QUICK DISCONNECT AND OPEN SAFETY VALVE //
    STATE = HardAbortCheck();
    STATE = SoftAbortCheck();
    if (STATE != 1){
      break;
    }
    if ((currentMillis - initMillis) > 30000 && startSeqCounter == 0 && STATE != 6) {
      digitalWrite(QDSEP_PIN,RELAY_ON);
      digitalWrite(SMOXV_PIN,RELAY_ON);  
      startSeqCounter = 1;
    }

    // AFTER *32 SECONDS*, CLOSE QDSEP RELAY //
    STATE = HardAbortCheck();
    STATE = SoftAbortCheck();
    if ((currentMillis - initMillis) > 32000 && qdsepOffCounter == 0 && STATE != 6) {
      digitalWrite(QDSEP_PIN,RELAY_OFF);
      qdsepOffCounter = 1;
    }

    // AFTER *60 SECONDS*, FEED CURRENT TO PYRO 1 //
    STATE = HardAbortCheck();
    STATE = SoftAbortCheck();
    // NOTE: CANNOT ABORT OUT OF LAUNCH AT THIS POINT //
    if ((currentMillis - initMillis) > 60000 && pyro1OpenCounter == 0 && STATE != 6){
      digitalWrite(PYRO_1_PIN,RELAY_ON);
      pyro1OpenCounter = 1;  
    }

    // SHUT OFF CURRENT TO PYRO_1 AFTER *60.25 SECONDS* //
    if ((currentMillis - initMillis) > 60250 && pyro1CloseCounter == 0){
      digitalWrite(PYRO_1_PIN,RELAY_OFF);
      pyro1CloseCounter = 1;
    }

    // AFTER *60.3 SECONDS*, OPEN MAIN OX VALVE (1) //
    if ((currentMillis - initMillis) > 60300 && moxv1OpenCounter == 0){
      digitalWrite(MOXV01_PIN,RELAY_ON);
      moxv1OpenCounter = 1;
      STATE = 2; // SET STATE TO FLIGHT_1
      LaunchSeqEnd = 1; // END WHILE LOOP
    }

  }

  return STATE;
}
///////////////////////////////////
///////////////////////////////////

// FLIGHT1 SEQUENCING FUNCTION //
int Flight1Seq(){

  initMillis = millis();
  int Flight1SeqEnd = 0;
  int moxv1OffCounter = 0;
  int Burn1Counter = 0;

  while (Flight1SeqEnd == 0){
    Serial.print("FLIGHT1 STATE = ");
    Serial.println(STATE);

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    if ((currentMillis - initMillis) > 1000 && moxv1OffCounter == 0){
      digitalWrite(MOXV01_PIN,RELAY_OFF);
      moxv1OffCounter = 1;
    }

    if ((currentMillis - initMillis) > 7000 && Burn1Counter == 0){
      digitalWrite(MOXV02_PIN,RELAY_ON);
      Burn1Counter = 1;
      STATE = 3; // SET STATE TO COAST
      Flight1SeqEnd = 1;
    }
  }
  return STATE;
}
/////////////////////////////////
/////////////////////////////////



// COAST SEQUENCING FUNCTION //
int CoastSeq(){

  initMillis = millis();
  int CoastSeqEnd = 0;
  int moxv2OffCounter = 0;
  int CoastCounter = 0;

  while (CoastSeqEnd == 0){

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    if ((currentMillis - initMillis) > 1000 && moxv2OffCounter == 0){
      digitalWrite(MOXV02_PIN,RELAY_OFF);
      moxv2OffCounter = 1;
    }

    if ((currentMillis - initMillis) > 7000 && CoastCounter == 0){
      CoastCounter = 1;
      STATE = 4; // SET STATE TO FLIGHT_2
      CoastSeqEnd = 1;
    }
  }
  return STATE;
}
///////////////////////////////
///////////////////////////////



// FLIGHT_2 SEQUENCING FUNCTION //
int Flight2Seq(){

  initMillis = millis();
  int Flight2SeqEnd = 0;
  int moxv3OpenCounter = 0;
  int moxv3OffCounter = 0;
  int Burn2Counter = 0;
  int pyro2OpenCounter = 0;
  int pyro2CloseCounter = 0;

  while (Flight2SeqEnd == 0){

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    if (pyro2OpenCounter == 0){
      digitalWrite(PYRO_2_PIN,RELAY_ON);
      pyro2OpenCounter = 1;  
    }

    if ((currentMillis - initMillis) > 250 && pyro2CloseCounter == 0){
      digitalWrite(PYRO_2_PIN,RELAY_OFF);
      pyro2CloseCounter = 1;
    }

    if ((currentMillis - initMillis) > 300 && moxv3OpenCounter == 0){
      digitalWrite(MOXV03_PIN,RELAY_ON);
      moxv3OpenCounter = 1;
    }

    if ((currentMillis - initMillis) > 1000 && moxv3OffCounter == 0){
      digitalWrite(MOXV03_PIN,RELAY_OFF);
      moxv3OffCounter = 1;
    }

    if ((currentMillis - initMillis) > 10000 && Burn2Counter == 0){
      Burn2Counter = 1;
      STATE = 5; // SET STATE TO RECOVERY
      Flight2SeqEnd = 2;
    }
  }
  return STATE;
}
//////////////////////////////
//////////////////////////////



// RECOVERY SEQUENCING FUNCTION //
void RecoverySeq(){

  initMillis = millis();
  int RecoverySeqEnd = 0;

  while (RecoverySeqEnd == 0){

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    if ((currentMillis - initMillis) > 1000 && FVVOpenNum == FVVCloseNum){
      digitalWrite(FVV_PIN,RELAY_ON);
      FVVOpenNum += 1;
    }

    if ((currentMillis - initMillis) > 120000 && FVVOpenNum != FVVCloseNum){
      digitalWrite(FVV_PIN,RELAY_OFF);
      FVVCloseNum = +1;
    }
  }
}
////////////////////////////////
////////////////////////////////


// SAFE SEQUENCING FUNCTION //
void SafeSeq(){

  initMillis = millis();
  int SafeSeqEnd = 0;

  while (SafeSeqEnd == 0){

    Serial.println("*******SAFE MODE*********");

    currentMillis = millis();
    //WriteSD(); // WRITE TELEMETRY TO SD CARD

    if (FVVOpenNum == FVVCloseNum){

      digitalWrite(SMOXV_PIN,RELAY_OFF);
      digitalWrite(FVV_PIN,RELAY_ON);
      FVVOpenNum += 1;
      SafeSeqEnd = 1;
    }
  }
}
//////////////////////////////
//////////////////////////////


//////////////////////////////
//////////////////////////////
//////////////////////////////
//////////////////////////////
// COMMANDS //

// HARD ABORT CHECK COMMAND //
int HardAbortCheck(){
  if (digitalRead(RD_ABORT) == HIGH){

    int falseAbortPinRead = 0;
    int initAbortMillis = millis();
    int currAbortMillis = millis();
    while (currAbortMillis - initAbortMillis < 100){

      if (digitalRead(RD_ABORT) == LOW){
        falseAbortPinRead = 1;
        break;
      }
      currAbortMillis = millis();
    }
    if (falseAbortPinRead == 0){

      STATE = 6; // SEND STATE TO SAFE
    }
  }
  return STATE;
}
//////////////////////////////
//////////////////////////////

// SOFT ABORT CHECK COMMAND //
int SoftAbortCheck(){
  
  if (digitalRead(RD_ABORT_SOFT) == HIGH){
    int falseAbortSoftPinRead = 0;
    int initAbortMillis = millis();
    int currAbortMillis = millis();
    while (currAbortMillis - initAbortMillis < 250){

      if (digitalRead(RD_ABORT_SOFT) == LOW){
        falseAbortSoftPinRead = 1;
        break;
      }
      currAbortMillis = millis();
    }
    if (falseAbortSoftPinRead == 0){

      digitalWrite(SMOXV_PIN,RELAY_OFF);
      STATE = 0; // SEND STATE TO PRELAUNCH
    }
  }
  return STATE;
}

//////////////////////////////
//////////////////////////////

// OPEN FVV COMMAND //
void OpenFVV(){
  if (digitalRead(RD_FVV) == HIGH){
    int falseOpenFVVPinRead = 0;
    int initFVVMillis = millis();
    int currFVVMillis = millis();
    while (currFVVMillis - initFVVMillis < 500){

      if (digitalRead(RD_FVV) == LOW){
        falseOpenFVVPinRead = 1;
        break;
      }
      currFVVMillis = millis();
    }
    if (falseOpenFVVPinRead == 0){

      digitalWrite(FVV_PIN,RELAY_ON);
      FVVOpenNum += 1;
    }
  }
}


//////////////////////////////
//////////////////////////////

// CLOSE FVV COMMAND //
void CloseFVV(){
  
  if (digitalRead(RD_FVV) == LOW && FVVCloseNum != FVVOpenNum){
      digitalWrite(FVV_PIN,RELAY_OFF);
      FVVCloseNum += 1;
  }
}

//////////////////////////////
//////////////////////////////

// SETUP SD COMMAND //
void SetupSD(){
  // CHECK FOR PRESSURE SENSOR //
  initMillis = millis();
  if (!bmp.begin()) {
    currentMillis = millis();
    digitalWrite(WR_ERROR,RELAY_ON);
    while (!bmp.begin() || (currentMillis - initMillis) > 500) { // IF DEVICE NOT FOUND, KEEP CHECKING FOR 0.5 SECONDS
    }
  }
  // CHECK FOR IMU DEVICE //
  initMillis = millis();
  if (!bno.begin()) {
    currentMillis = millis();
    digitalWrite(WR_ERROR,RELAY_ON);
    while (!bno.begin() || (currentMillis - initMillis) > 500) {
    }
  }
  // CHECK FOR SD CARD //
  initMillis = millis();
  if (!SD.begin(chipSelect)) {
    currentMillis = millis();
    digitalWrite(WR_ERROR,RELAY_ON);
    while (!SD.begin() || (currentMillis - initMillis) > 500) {
    }
  }
  // REMOVE OLD FILE //
  if (SD.exists("guinlog.txt")) {
    SD.remove("guinlog.txt");
  }
  
}


//////////////////////////////
//////////////////////////////
void WriteSD(){
  // PRINT VARIABLES TO SD CARD //
  
  // OPEN FILE TO WRITE DATA LOG //
  // THIS CREATES A FILE IF NONE CURRENTLY EXISTS //
  File guinLog = SD.open("guinlog.txt", FILE_WRITE);
  
  // MAKE A STRING FOR ASSEMBLING THE DATA TO THE LOG //
  // FORMAT: ALTITUDE (m), PRESSURE (Pa), TEMPERATURE_BMP (C), TEMPERATURE_IMU (C), ACCELERATION_x,y,z (m/s^2), EULER_x,y,z (deg), GYRO_x,y,z (rad/s), MAG_x,y,z (uT), LIN_ACCEL_x,y,z (m/s^2), GRAVITY_x,y,z, QUAT_w_x_y_z //
  String dataString = "";

  // CONCATENATE VALUES INTO STRING //
  dataString += bmp.readAltitude();
  dataString += ",";

  dataString += bmp.readPressure();
  dataString += ",";

  dataString += bmp.readTemperature();
  dataString += ",";

  dataString += bno.getTemp();
  dataString += ",";
/*
  dataString += accel.x();
  dataString += ",";
  dataString += accel.y();
  dataString += ",";
  dataString += accel.z();
  dataString += ",";  

  dataString += euler.x();
  dataString += ",";
  dataString += euler.y();
  dataString += ",";
  dataString += euler.z();
  dataString += ",";

  dataString += gyro.x();
  dataString += ",";
  dataString += gyro.y();
  dataString += ",";
  dataString += gyro.z();
  dataString += ",";  

  dataString += mag.x();
  dataString += ",";
  dataString += mag.y();
  dataString += ",";
  dataString += mag.z();
  dataString += ",";    

  dataString += linaccel.x();
  dataString += ",";
  dataString += linaccel.y();
  dataString += ",";
  dataString += linaccel.z();
  dataString += ",";    

  dataString += grav.x();
  dataString += ",";
  dataString += grav.y();
  dataString += ",";
  dataString += grav.z();
  dataString += ",";

  // Quaternion data
  dataString += quat.w();
  dataString += ",";
  dataString += quat.y();
  dataString += ",";
  dataString += quat.x();
  dataString += ",";
  dataString += quat.z();  
  */
  ///////////////////////////////////////////

// PRINT FINAL STRING ONTO SD CARD //
// CLOSE OPENED FILE //
if (guinLog) {
    guinLog.println(dataString);
    guinLog.close();
}
}

//////////////////////////////
//////////////////////////////


