//#define ROSSERIAL_ARDUINO_TCP
//#include "WiFi.h"
#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "geometry_msgs/Pose.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "control_msgs/PidState.h"
//header file for i,m.kjliumu
#include "sensor_msgs/Imu.h"

//#include "amr_base_config.h"
#include "AMR_Motor.h"
#include "Kinematics.h"
// #include "PID.h"
#include "AmrEncoder.h"
#include "mpu9250.h"

//#include "PID_v1.h"

#define AMR_BASE SKID_STEER

#define DEBUG 1

//=================BIGGER ROBOT SPEC (MDDA10)=============================
#define K_P 0.348607064985295  // P constant
#define K_I 0.25750443968910   // I constant
#define K_D 0.0   // D constant

// define your robot' specs here
#define MAX_RPM 83              // motor's maximum RPM
#define COUNTS_PER_REV 844      // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.125      // wheel's diameter in meters
#define PWM_BITS 8               // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.45  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.215  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN

#define MOTOR1_ENCODER_A 5
#define MOTOR1_ENCODER_B 23

#define MOTOR2_ENCODER_A 12
#define MOTOR2_ENCODER_B 13 

#define MOTOR3_ENCODER_A 16
#define MOTOR3_ENCODER_B 17 

#define MOTOR4_ENCODER_A 19
#define MOTOR4_ENCODER_B 18
  
#define MOTOR1_PWM 27 // Changed to a valid PWM pin
#define MOTOR1_IN_A 33
#define MOTOR1_CHANNEL 0

#define MOTOR2_PWM 26 // Changed to a valid PWM pin
#define MOTOR2_IN_A 32
#define MOTOR2_CHANNEL 1

#define MOTOR3_PWM 14 // Changed to a valid PWM pin
#define MOTOR3_IN_A 4
#define MOTOR3_CHANNEL 6

#define MOTOR4_PWM 25 // Changed to a valid PWM pin
#define MOTOR4_IN_A 15
#define MOTOR4_CHANNEL 7

#define PWM_MAX (pow(2, PWM_BITS) - 1)
#define PWM_MIN (-PWM_MAX)


#define FREQ 5000
#define RES  8



bfs::Mpu9250 imu;


#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

AmrEncoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B );
AmrEncoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B );
AmrEncoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B );
AmrEncoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B );

Controller motor1_controller(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_CHANNEL, FREQ, RES);
Controller motor2_controller(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_CHANNEL, FREQ, RES); 
Controller motor3_controller(MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_CHANNEL, FREQ, RES);
Controller motor4_controller(MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_CHANNEL, FREQ, RES);

// PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE, PWM_BITS);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

//IPAddress server(192, 168, 8, 103);
//uint16_t serverPort = 11411;
//const char*  ssid = "Nathan";
//const char*  password = "neso0ofares2001";

void commandCallback(const geometry_msgs::Twist& cmd_msg);
//void PIDCallback(const control_msgs::PidState& pid);


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//ros::Subscriber<control_msgs::PidState> pid_sub("pid", PIDCallback);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::Pose raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);


void setup() {

//    Serial.begin(115200);
//    setupWiFi();

//    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
//    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(imu_pub);
    nh.loginfo("AMR CONNECTED");

    Serial.begin(115200);
     while(!Serial) {}
//    /* Start the I2C bus */
    Wire.begin();
    Wire.setClock(400000);
    /* I2C bus,  0x68 address */
    imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
    /* Initialize and configure IMU */
    if (!imu.Begin()) {
      Serial.println("Error initializing communication with IMU");
      while(1) {}
    }
    /* Set the sample rate divider */
    if (!imu.ConfigSrd(19)) {
      Serial.println("Error configured SRD");
      while(1) {}
    }
    delay(5000);



}

void loop() {


    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

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

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
//        if (!imu_is_initialized)
//        {
//            imu_is_initialized = setupIMU();
//
//
//            if(imu_is_initialized)
//                nh.loginfo("IMU Initialized");
//            else
//                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
//        }
//        else
//        {
            publishIMU();
//        }
        prev_imu_time = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}

//void setupWiFi()
//{  
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
//   Serial.print("SSID: ");
//   Serial.println(WiFi.SSID());
//   Serial.print("IP:   ");
//   Serial.println(WiFi.localIP());
//
//}

//void PIDCallback(const control_msgs::PidState& pid)
//{
//    //callback function every time PID constants are received from lino_pid for tuning
//    //this callback receives pid object where P,I, and D constants are stored
//    motor1_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
//    motor2_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
//    motor3_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
//    motor4_pid.updateConstants(pid.p_term, pid.i_term, pid.d_term);
//}


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
    Kinematics::output req_pwm = kinematics.getPWM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    //get the current speed of each motor
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();
    int current_rpm3 = motor3_encoder.getRPM();
    int current_rpm4 = motor4_encoder.getRPM();

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(req_pwm.motor1);
    motor2_controller.spin(req_pwm.motor2);
    motor3_controller.spin(req_pwm.motor3);  
    motor4_controller.spin(req_pwm.motor4);    

//    motor1_controller.spin(MAX_RPM);
//    motor2_controller.spin(MAX_RPM);
//    motor3_controller.spin(MAX_RPM);  
//    motor4_controller.spin(MAX_RPM);    

    Kinematics::velocities current_vel;

    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);

    //pass velocities to publisher object
    raw_vel_msg.position.x = current_vel.linear_x;
    raw_vel_msg.position.y = current_vel.linear_y;
    raw_vel_msg.orientation.z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);    

}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    if( imu.Read())
      {
      // imu.new_imu_data();
      // imu.new_mag_data();
      //pass accelerometer data to imu object
      imu_msg.linear_acceleration.x = imu.accel_x_mps2();
      imu_msg.linear_acceleration.y = imu.accel_y_mps2();
      imu_msg.linear_acceleration.z = imu.accel_z_mps2();
  
      //pass gyroscope data to imu object
      imu_msg.angular_velocity.x = imu.gyro_x_radps();
      imu_msg.angular_velocity.y = imu.gyro_y_radps();
      imu_msg.angular_velocity.z = imu.gyro_z_radps();
  
      //pass accelerometer data to imu object
      imu_msg.orientation.x = imu.mag_x_ut();
      imu_msg.orientation.y = imu.mag_y_ut();
      imu_msg.orientation.z = imu.mag_z_ut();
  
      //publish raw_imu_msg
      imu_pub.publish(&imu_msg);
      }
  }


void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.getRPM());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.getRPM());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.getRPM());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.getRPM());
    nh.loginfo(buffer);
}
