///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// VERSION HISTORY //
// YYYY-MM-DD/HH:MM - Description of changes (Author)
// 2016-05-18/13:31 - Initial Upload of Flight Master (Alex Wolff)
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////


// STATE DEFINITIONS //
// PRELAUNCH 0 //
// LAUNCH    1 //
// FLIGHT_1  2 //
// COAST     3 //
// FLIGHT_2  4 //
// RECOVERY  5 //
///////////////////////
int STATE = 0; // INITIALIZE TO PRELAUNCH STATE
///////////////////////

// READY COUNTER //
////////////////////
int LaunchReadyCounter = 0;
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
#define RD_ABORT   11
#define RD_PLOAD   12
  
// DEFINE ACTIVE LOW RELAY CONFIG //
#define RELAY_ON  0
#define RELAY_OFF 1

// GLOBAL VARIABLES //
int initMillis = millis();  
int currentMillis = millis();

void setup() {

  // INITIALIZE READ PINS AS INPUTS //
  pinMode(RD_LAUNCH, INPUT);
  pinMode(RD_ABORT,  INPUT);
  pinMode(RD_PLOAD,  INPUT);

  int MasterInitMillis = millis();
  

}

void loop() {

  int LoopMillis = millis();
  

  if (digitalRead(RD_LAUNCH) == HIGH && digitalRead(RD_ABORT) == LOW){
    // DELAY FOR SHORT TIME TO ENSURE TRUE SIGNAL //
    
  }

  

}


//// LAUNCH SEQUENCING FUCTION ////
int LaunchSeq(){

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

  int LaunchSeqEnd    = 0;
  int startSeqCounter = 0;
  int pyro1Counter    = 0;
  int mainOx1Counter  = 0;
  initMillis = millis();

  while (LaunchSeqEnd == 0) {

    currentMillis = millis();

    // AFTER *30 SECONDS*, ACTUAL QUICK DISCONNECT AND OPEN SAFETY VALVE //
    if ((currentMillis - initMillis) > 30000 && startSeqCounter == 0) {
      digitalWrite(QDSEP_PIN,RELAY_ON);
      int QDSEP_time = millis();
      digitalWrite(SMOXV_PIN,RELAY_ON);  
      startSeqCounter = 1;
    }

    // AFTER *32 SECONDS*, CLOSE QDSEP RELAY //
    if ((currentMillis - initMillis) > 32000 && startSeqCounter == 1) {
      digitalWrite(QDSEP_PIN,RELAY_OFF);
    }

    // AFTER *60 SECONDS*, FEED CURRENT TO PYRO 1 //
    if ((currentMillis - initMillis) > 60000 && pyro1Counter == 0){
      digitalWrite(PYRO_1_PIN,RELAY_ON);
      pyro1Counter == 1;  
    }

    // SHUT OFF CURRENT TO PYRO_1 AFTER *61 SECONDS* //
    if ((currentMillis - initMillis) > 61000 && pyro1Counter == 1){
      digitalWrite(PYRO_1_PIN,RELAY_OFF);
    }

    // AFTER *62 SECONDS*, OPEN MAIN OX VALVE (1) //
    if ((currentMillis - initMillis) > 62000 && mainOx1Counter == 0){
      digitalWrite(MOXV01_PIN,RELAY_ON);
      mainOx1Counter = 1;
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

  initMillis - millis();
  int Flight1SeqEnd = 0;
  int moxv1CloseCounter = 0;
  int Burn1Counter = 0;

  while (Flight1SeqEnd == 0){

    currentMillis = millis();

    if ((currentMillis - initMillis) > 1000 && moxv1CloseCounter == 0){
      digitalWrite(MOXV01_PIN,RELAY_OFF);
      moxv1CloseCounter = 1;
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

  initMillis - millis();
  int CoastSeqEnd = 0;
  int moxv2CloseCounter = 0;
  int CoastCounter = 0;

  while (CoastSeqEnd == 0){

    currentMillis = millis();

    if ((currentMillis - initMillis) > 1000 && moxv2CloseCounter == 0){
      digitalWrite(MOXV02_PIN,RELAY_OFF);
      moxv2CloseCounter = 1;
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

  initMillis - millis();
  int Flight2SeqEnd = 0;
  int moxv3OpenCounter = 0;
  int moxv3CloseCounter = 0;
  int Burn2Counter = 0;

  while (Flight2SeqEnd == 0){

    currentMillis = millis();

    if (moxv3OpenCounter == 0){
      digitalWrite(MOXV03_PIN,RELAY_ON);
      moxv3OpenCounter = 1;
    }

    if ((currentMillis - initMillis) > 1000 && moxv3CloseCounter == 0){
      digitalWrite(MOXV03_PIN,RELAY_OFF);
      moxv3CloseCounter = 1;
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

  initMillis - millis();
  int RecoverySeqEnd = 0;
  int FVVOpenCounter = 0;
  int FVVCloseCounter = 0;

  while (RecoverySeqEnd == 0){

    currentMillis = millis();

    if ((currentMillis - initMillis) > 1000 && FVVOpenCounter == 0){
      digitalWrite(FVV_PIN,RELAY_ON);
      FVVOpenCounter = 1;
    }

    if ((currentMillis - initMillis) > 120000 && FVVCloseCounter == 0){
      digitalWrite(FVV_PIN,RELAY_OFF);
      FVVCloseCounter = 1;
    }
  }
}
////////////////////////////////
////////////////////////////////


// SAFE SEQUENCING FUNCTION //
void SafeSeq(){

  initMillis - millis();
  int SafeSeqEnd = 0;
  int FVVOpenCounter = 0;

  while (SafeSeqEnd == 0){

    currentMillis = millis();

    if (FVVOpenCounter == 0){

      digitalWrite(SMOXV_PIN,RELAY_OFF);
      digitalWrite(FVV_PIN,RELAY_ON);
      FVVOpenCounter = 1;
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

    int falsePinRead = 0;
    int initAbortMillis = millis();
    int currAbortMillis = millis();
    while (currAbortMillis - initAbortMillis < 500){

      if (digitalRead(RD_ABORT) == LOW){
        falsePinRead = 1;
        break;
      }
      currAbortMillis = millis();
    }
    if (falsePinRead == 0){

      STATE = 6; // SEND STATE TO SAFE
    }
    else {
      STATE = STATE;
    }
    return STATE;
  }
}
//////////////////////////////
//////////////////////////////
//////////////////////////////
//////////////////////////////


