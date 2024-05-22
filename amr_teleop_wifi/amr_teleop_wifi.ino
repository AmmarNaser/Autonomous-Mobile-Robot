#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "geometry_msgs/Pose.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "control_msgs/PidState.h"
//header file for imu
#include "sensor_msgs/Imu.h"

#include "amr_base_config.h"
#include "AMR_Motor.h"
#include "Kinematics.h"
#include "PID.h"
#include "AmrEncoder.h"

#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

AmrEncoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B );
AmrEncoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B );
AmrEncoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B );
AmrEncoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B );

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(Kinematics::AMR_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

IPAddress server(192, 168, 8, 103);
uint16_t serverPort = 11411;
const char*  ssid = "Nathan";
const char*  password = "neso0ofares2001";

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const control_msgs::PidState& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<control_msgs::PidState> pid_sub("pid", PIDCallback);

void setup()
{
    Serial.begin(115200);
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);

    nh.initNode();
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("AMR CONNECTED");
    delay(2000);
}


void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_debug_time = 0;


        //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

        //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
//    if(DEBUG)
//    {
//        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
//        {
//           printDebug();
//            prev_debug_time = millis();
//        }
//    }

    nh.spinOnce();

}

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}

void PIDCallback(const control_msgs::PidState& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
    motor2_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
    motor3_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
    motor4_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM(COUNTS_PER_REV);
    int current_rpm2 = motor2_encoder.getRPM(COUNTS_PER_REV);
    int current_rpm3 = motor3_encoder.getRPM(COUNTS_PER_REV);
    int current_rpm4 = motor4_encoder.getRPM(COUNTS_PER_REV);

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));  
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));    

    Kinematics::velocities current_vel;

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);

    

}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}


//void printDebug()
//{
//    char buffer[50];
//
//    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.getCount());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.getCount());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.getCount());
//    nh.loginfo(buffer);
//    sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.getCount());
//    nh.loginfo(buffer);
//}
