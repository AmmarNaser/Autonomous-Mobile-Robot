#include "AmrEncoder.h"

// Constructor definition
AmrEncoder::AmrEncoder(uint8_t chA, uint8_t chB) : chA_(chA), chB_(chB) {
    // Create an instance of the ESP32Encoder class
    encoder_ = new ESP32Encoder();
    // Attach the encoder to the specified pins
    encoder_->attachHalfQuad(chA_, chB_);
}


int AmrEncoder::getRPM() {
    long encoder_ticks = encoder_->getCount();
    
    // Get current time
    unsigned long current_time = millis();
    
    // Calculate delta time since last update
    unsigned long dt = current_time - prev_update_time_;
    
    // Convert time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    
    // Calculate delta ticks since last update
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;
    
    // Calculate wheel's speed (RPM)
    
    // Update previous values for next calculation
    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;
    
		return (delta_ticks / counts_per_rev_) / dtm;
    }

