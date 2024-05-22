#ifndef AMR_ENCODER_H_
#define AMR_ENCODER_H_

#include <Arduino.h>
#include <ESP32Encoder.h>

class AmrEncoder {
public:
    AmrEncoder(uint8_t chA, uint8_t chB);
    // int getDirectionalRPM(int counts_per_rev);
    int getRPM();

private:
    uint8_t chA_;
    uint8_t chB_;
    int counts_per_rev_ = 850;
    // Variables to keep track of previous values
    unsigned long prev_update_time_ = 0;
    long prev_encoder_ticks_ = 0;
    int prev_direction_ = 1; // Initial direction: clockwise
    ESP32Encoder *encoder_; // Pointer to ESP32Encoder object
};

#endif // AMR_ENCODER_H_
