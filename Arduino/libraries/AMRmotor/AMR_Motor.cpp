#include "AMR_Motor.h"

Controller::Controller( int pwm_pin, int motor_pinA, int motor_channel, int freq, int res):
    pwm_pin_(pwm_pin),
    motor_pinA_(motor_pinA),
    motor_channel_(motor_channel),
    freq_(freq),
    res_(res)
{
    pinMode(pwm_pin_, OUTPUT);
    pinMode(motor_pinA_, OUTPUT);
    ledcSetup(motor_channel_ ,freq_ , res_);

    ledcAttachPin(pwm_pin_,motor_channel_);
    ledcWrite(motor_channel_, abs(0));  
}

void Controller::spin(int pwm)
{
   
     if(pwm > 0)
     {
         digitalWrite(motor_pinA_, HIGH);
     }
     else if(pwm < 0)
     {
         digitalWrite(motor_pinA_, LOW);
     }

    //digitalWrite(motor_pinA_,pwm  < 0);
    //digitalWrite(motor_pinA_,pwm  > 0);
    ledcWrite(motor_channel_, abs(pwm));  

    
}
