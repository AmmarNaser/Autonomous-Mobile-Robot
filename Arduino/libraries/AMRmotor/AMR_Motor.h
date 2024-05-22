#ifndef AMR_MOTOR_H
#define AMR_MOTOR_H

#include <Arduino.h>


class Controller
{
    public:
        Controller(int pwm_pin, int motor_pinA, int motor_channel, int freq, int res);
        void spin(int pwm);

    private:
        int pwm_pin_;
        int motor_pinA_;
        int motor_channel_;
        int freq_;
        int res_;
};

#endif
