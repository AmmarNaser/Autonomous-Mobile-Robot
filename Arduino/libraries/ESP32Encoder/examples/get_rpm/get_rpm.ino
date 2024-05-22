#include <amrEncoder.h>

AmrEncoder encoder1(5, 6, 420);

void setup() {
    Serial.begin(115200);

}

void loop() {
    
    Serial.println("RPM = " + String((int32_t)encoder1.getRPM()) + "\n");

}