#include <Average.h>

int analogPin = 1;


Average<float> ave(1000);

void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Add a new random value to the bucket
    
Serial.println("test");
    // Display the current data set
    for (int i = 0; i < 1000; i++) {
        ave.push(analogRead(analogPin)*(5.0/1023.0));
    }

    // And show some interesting results.
    Serial.println(ave.mean());
    

}

