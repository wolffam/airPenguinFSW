#include <Average.h>
Average<float> aveState(1000);
Average<float> avePress1(1000);
Average<float> avePress2(1000);
Average<float> avePress3(1000);

#include "HX711.h"
float calibration_factor = -5200.0;

// Define digital pins
#define SVV 2
#define VSOX 3
#define NPV 4
#define LAUNCH 5
#define HARDABORT 6
#define SOFTABORT 7
#define FVV 8

// Define analog pins
#define STATE A0
#define CLK  A1    // For load cells
#define DOUT  A2   // For load cells
#define PRESSURE1 A3
#define PRESSURE2 A4
#define PRESSURE3 A5

// Unused valves
#define VALVE4 5
#define VALVE5 6
#define VALVE6 7
#define VALVE8 9

  int incomingByte = 0;   // for incoming serial data
  int counter = 0;
  int val = 0;
  float pressure = 0.0;
  int x = 0;
  int pressureducer1 = 0;
  int cycle = 0;
  int State = 0;
  float Thrust = 1;
  float Press1 = 0;
  float Press2 = 0;
  float Press3 = 0;

  HX711 scale(DOUT, CLK);
  
void setup() {
  // put your setup code here, to run once:

  // initialize serial communication:
  Serial.begin(9600);
  delay(2000);

  // Set up scale
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0

  // Initialize pin modes
  pinMode(SVV,OUTPUT);
  pinMode(VSOX,OUTPUT);
  pinMode(NPV,OUTPUT);
  pinMode(VALVE4,OUTPUT);
  pinMode(VALVE5,OUTPUT);
  pinMode(VALVE6,OUTPUT);
  pinMode(FVV,OUTPUT);
  pinMode(VALVE8,OUTPUT);
  pinMode(LAUNCH,OUTPUT);
  pinMode(SOFTABORT,OUTPUT);
  pinMode(HARDABORT,OUTPUT);

  // Relays for valves are open when high, so intitialize them to be open
  digitalWrite(SVV,1);
  digitalWrite(VSOX,1);
  digitalWrite(NPV,1);
  //digitalWrite(VALVE4,1);
  //digitalWrite(VALVE5,1);
  //digitalWrite(VALVE6,1);
  digitalWrite(FVV,0);
  //digitalWrite(VALVE8,1);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Read in the state
  for (int i = 0; i < 1000; i++) {
      aveState.push(analogRead(STATE));
      avePress1.push(analogRead(PRESSURE1));
      avePress2.push(analogRead(PRESSURE2));
      avePress3.push(analogRead(PRESSURE3));
  }
  State = int((aveState.mean())/170.5 + 0.5); 
  Press1 = avePress1.mean();
  Press2 = avePress2.mean();
  Press3 = avePress3.mean();

  // Read in load cells
  Thrust = scale.get_units();

  // The ordering here is VERY IMPORTANT, never change these
  Serial.println(Press1);
  Serial.println(Press2);
  Serial.println(Press3);
  Serial.println(Thrust);
  Serial.println(State);
  delay(500);

  /*
  // State sent over PWM Test  
  //Print State 5
  analogWrite(STATEOUT,212.5);
      // Display the current data set
    for (int i = 0; i < 1000; i++) {
        ave.push(analogRead(STATEIN));
    }

    // And show some interesting results.
    Serial.println(int((ave.mean())/170.5 + 0.5)); 
    delay(500); 
  //Print State 6
  analogWrite(STATEOUT,255);
      // Display the current data set
    for (int i = 0; i < 1000; i++) {
        ave.push(analogRead(STATEIN));
    }

    // And show some interesting results.
    Serial.println(int((ave.mean())/170.5 + 0.5)); 
    delay(500); 
  */



  // Logic for opening and closing Valves
  if (Serial.available() > 0) {
           
      // read the incoming byte:
      incomingByte = Serial.read();

      // Valve 1: open
      if (incomingByte == 'a') {
        digit
        alWrite(SVV,0);
      }

      // Valve 1: closed
      if (incomingByte == 'b') {
        digitalWrite(SVV,1);
      }

      // Valve 2: open
      if (incomingByte == 'c') {
        digitalWrite(VSOX,0);
      }

      // Valve 2: closed
      if (incomingByte == 'd') {
        digitalWrite(VSOX,1);
      }

      // Valve 3: open
      if (incomingByte == 'e') {
        digitalWrite(NPV,0);
      }

      // Valve 3: closed
      if (incomingByte == 'f') {
        digitalWrite(NPV,1);
      }

      // Valve 4: open
      if (incomingByte == 'g') {
        digitalWrite(VALVE4,0);
      }

      // Valve 4: closed
      if (incomingByte == 'h') {
        digitalWrite(VALVE4,1);
      }

      // Valve 5: open
      if (incomingByte == 'i') {
        digitalWrite(VALVE5,0);
      }

      // Valve 5: closed
      if (incomingByte == 'j') {
        digitalWrite(VALVE5,1);
      }

      // Valve 6: open
      if (incomingByte == 'k') {
        digitalWrite(VALVE6,0);
      }

      // Valve 6: closed
      if (incomingByte == 'l') {
        digitalWrite(VALVE6,1);
      }

      // Valve 7: open
      if (incomingByte == 'm') {
        digitalWrite(FVV,1);
      }

      // Valve 7: closed
      if (incomingByte == 'n') {
        digitalWrite(FVV,0);
      }

      // Valve 8: open
      if (incomingByte == 'o') {
        digitalWrite(VALVE8,0);
      }

      // Valve 8: closed
      if (incomingByte == 'p') {
        digitalWrite(VALVE8,1);
      }

      if (incomingByte == 'q') {
        x = 0;
        while (x < 100) {
          x = x+1;
          Serial.println(analogRead(0));
          Serial.println(1);
          delay(100);
        }
      }

      // Soft Abort
      if (incomingByte == 's') {
        digitalWrite(SOFTABORT,1);
        delay(2000);
        digitalWrite(SOFTABORT,0);
      }

      // Hard Abort
      if (incomingByte == 'r') {
        digitalWrite(HARDABORT,1);
        delay(2000);
        digitalWrite(HARDABORT,0);        
      }

      // Launch
      if (incomingByte == 't') {
        // Close VSOX first, or shit gets real
        digitalWrite(VSOX,1);
        // Now that shit ain't real, launch
        digitalWrite(LAUNCH,1);
        delay(2000);
        digitalWrite(LAUNCH,0);        
      }

      if (incomingByte == 'u') {  
          scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
      }
  } 
  
}
