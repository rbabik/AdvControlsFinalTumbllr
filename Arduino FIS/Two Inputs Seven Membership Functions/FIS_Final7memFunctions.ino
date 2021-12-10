/*
 * INCLUDES A PSI_D WEIGHT THAT IS SMALL, WHICH ALLOWS IT TO HANDLE LIGHT IMPULSES
 */


/* Include necessary header files */
#include <BasicLinearAlgebra.h>
#include "PinChangeInt.h"
#include "MsTimer2.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "KalmanFilter.h"


/* TB6612FNG motor driver signal pins */
#define IN1_L 7
#define IN1_R 12
#define PWM_L 5
#define PWM_R 6
#define STBY 8

//#define PI 3.14159265

/* Encoder count signal pins */
#define PinA_left 2
#define PinA_right 4

/* Sensor Reading Variables */
MPU6050 mpu; //Instantiate a MPU6050 object with the object name MPU.
int16_t ax, ay, az, gx, gy, gz; // Variables for IMU readings
volatile long right_encoder = 0;
volatile long left_encoder = 0;
/* 
 The volatile long type is used to ensure that the value is valid when the external 
 interrupt pulse count value is used in other functions 
 */
float num_of_encoder_counts_per_rev = 780.0;
float thousand_by_num_of_encoder_counts_per_rev = 1000.0/num_of_encoder_counts_per_rev;

/* IMU Callibration Variables */ 

//long gyroXCalli = -380, gyroYCalli = 65, gyroZCalli = -229; // Obtained by callibrating gyroscope values
long gyroXCalli = -415, gyroYCalli = 36, gyroZCalli = -292; // Obtained by callibrating gyroscope values
long accelXCalli = 739, accelYCalli = -73, accelZCalli = 15748; // Obtained by callibrating accelerometer values

/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 5; // in milliseconds -- really this controller requires a 10ms rate to be able to keep up
//but many loop runs can take somewhere between 5 and 10ms

/* Encoder to speed measurement variables */

int encoder_count_max = 20;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

/* Robot parameters */

float wheel_rad = 0.0335; // radius of the wheel in metres
float lat_dist = 0.194; // distance between ends of the wheels in metres
/********************Initialization settings********************/
void setup() {
  
  /* TB6612FNGN Motor Driver module control signal initialization */
  pinMode(IN1_L, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(IN1_R, OUTPUT); //Control the direction of motor 1, 1 is forward, 0 is reverse
  pinMode(PWM_L, OUTPUT); //PWM of left motor
  pinMode(PWM_R, OUTPUT); //PWM of right motor
  pinMode(STBY, OUTPUT); //enable TB6612FNG
  
  /* Initializing motor drive module */
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN1_R, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  
  /* Initialize I2C bus */
  Wire.begin(); //Add I2C bus sequence
  Serial.begin(57600); //Open the serial port and set the baud rate
  delay(1500);
  mpu.initialize(); //Initialization MPU6050
  delay(2);
  Serial.print("MPU SETUP COMPLETE");

  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
  //Serial.println("Started Calibration");
  //callibrateGyroValues();
  //Serial.println("Finished Gyro Calli, starting Accel");
  //callibrateAccelValues();
  
  time = millis();
  startTime_left = millis();
  startTime_right = millis();
  
  /* Interrupt function to count the encoder pulses */
  attachInterrupt(digitalPinToInterrupt(PinA_left), encoder_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, encoder_right, CHANGE);
  
  /* 
  Timing interrupt settings, using MsTimer2. Because PWM uses a timer to control the 
  duty cycle, it is important to look at the pin port corresponding to the timer when 
  using timer.
  */
  MsTimer2::set(sampling_rate, mainfunc);
  MsTimer2::start();
  
  Serial.print("Setup Done!\n");

  /* Set prevAngle for angle calc to angle computed by accelerometer
   * assuming robot is at rest to start, after that we use 
   * complimentary filter with alpha = 0.97 to compute angle
   */
  //startAngle();
}

/***************************************************************************************/
/*
 This section contains functions that you can use to build your controllers
*/
/***************************************************************************************/

/* 
encoder_left() counts encoder pulses of the left wheel motor and stores it in a 
global variable 'left_encoder'
*/
void encoder_left(){left_encoder++;}
  
/* 
encoder_right() counts encoder pulses of the right wheel motor and stores it in a 
global variable 'right_encoder'
*/
void encoder_right(){right_encoder++;}


/* 
SetLeftWheelSpeed() takes one input which will be set as the PWM input to the left motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_left'
*/
void SetLeftWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_left = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_L, 1);
    analogWrite(PWM_L, -speed_val);
  }
  else
  {
    digitalWrite(IN1_L, 0);
    analogWrite(PWM_L, speed_val);
  }
}

/* 
SetRightWheelSpeed() takes one input which will be set as the PWM input to the right motor.
If the value is outside the range (-255,255), then the input will be saturated and the 
motor PWM will be set. The value is also written to the global variable 'motor_right'
*/
void SetRightWheelSpeed(double speed_val)
{
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) { speed_val = 255; }
  else if (speed_val < -255) { speed_val = -255; }
  
  motor_right = speed_val;
  
  if (speed_val < 0)
  {
    digitalWrite(IN1_R, 1);
    analogWrite(PWM_R, -speed_val);
  }
  else
  {
    digitalWrite(IN1_R, 0);
    analogWrite(PWM_R, speed_val);
  }
}

/*
readIMU() creates an MPU6050 class object and calls the function to read the six axis IMU.
The values are stored in the global variables ax,ay,az,gx,gy,gz where ax,ay,az are the 
accelerometer readings and gx,gy,gz are the gyroscope readings. 
*/
void readIMU()
{
  MPU6050 mpu_obj;
  mpu_obj.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
}

/*
readEncoder() takes the encoder pulse counts and calculates the angular velocities of the
wheels and stores it in the global variables 'motor_left_ang_vel' and 'motor_right_ang_vel'
*/
void readEncoder()
{ 
  // Encoder Calculations
  // angular velocity = (encoder_reading/num_of_counts_per_rotation)*(2*pi/sampling_time)
  
  motor_left_ang_vel = (float) 2 * 3.1415 * left_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_left);  
  if (motor_left < 0){
    motor_left_ang_vel = -motor_left_ang_vel;}
  startTime_left = time;
  left_encoder = 0;  
  
  motor_right_ang_vel = (float) 2 * 3.1415 * right_encoder * (float)thousand_by_num_of_encoder_counts_per_rev / (float)(time - startTime_right);  
  if (motor_right < 0){
    motor_right_ang_vel = -motor_right_ang_vel;}
  startTime_right = time;
  right_encoder = 0;  
}

/* 
printIMU() prints the IMU readings to the serial monitor in the following format:
ax,ay,az,gx,gy,gz
*/
void printIMU()
{
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print('\n');
}

/* 
printEncoder() prints the encoder readings to the serial monitor in the following format:
motor_left_ang_vel, motor_right_ang_vel
*/
void printEncoder()
{
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print('\n');
}

/* 
printAllData() prints the IMU readings, encoder readings and PWM inputs to the motor to
the serial monitor in the following format:
ax,ay,az,gx,gy,gz,motor_left_ang_vel,motor_right_ang_vel,motor_left,motor_right
*/
void printAllData()
{
  Serial.print(time);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print(',');
  Serial.print(motor_left);
  Serial.print(',');
  Serial.print(motor_right);
  Serial.print('\n');
}

/*
callibrateGyroValues() gets the gyroscope readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
gyroXCalli,gyroYCalli,gyroZCalli
*/
double gx_callibration;
void callibrateGyroValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      gyroXCalli = gyroXCalli + gx;
      gyroYCalli = gyroYCalli + gy;
      gyroZCalli = gyroZCalli + gz;
    }
    gyroXCalli = gyroXCalli/n;
    gyroYCalli = gyroYCalli/n;
    gyroZCalli = gyroZCalli/n;
    Serial.print(gyroXCalli);
    Serial.print(',');
    Serial.print(gyroYCalli);
    Serial.print(',');
    Serial.print(gyroZCalli);
    Serial.print('\n');
}

/*
callibrateAccelValues() gets the accelerometer readings n times and calculates the average of
the values to find the sensor bias. The callibration values are printed as:
accelXCalli,accelYCalli,accelZCalli
*/
void callibrateAccelValues() 
{
    int n = 10000;
    for (int i=0; i < n; i++) 
    {
      readIMU();
      accelXCalli = accelXCalli + ax;
      accelYCalli = accelYCalli + ay;
      accelZCalli = accelZCalli + az;
    }
    accelXCalli = accelXCalli/n;
    accelYCalli = accelYCalli/n;
    accelZCalli = accelZCalli/n;
    Serial.print(accelXCalli);
    Serial.print(',');
    Serial.print(accelYCalli);
    Serial.print(',');
    Serial.print(accelZCalli);
    Serial.print('\n');
}

/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/

void deadZoneCharacterization(void)
{
    // deadzone left is PWM < 16
//  // deadzone right is PWM < 18
//  static float testTime;
//  static float deadZoneCharacterization = 10;
//  if (millis() - testTime > 100) {
//    SetLeftWheelSpeed(deadZoneCharacterization);
//    //SetLeftWheelSpeed(deadZoneCharacterization);
//    deadZoneCharacterization += 0.1;
//    testTime = millis();
//  }
////  Serial.print(testTime);
////  Serial.print("\t");
////  Serial.print(millis());
////  Serial.print("\t");
//  Serial.print(deadZoneCharacterization);
//  Serial.print("\t");
//  Serial.println(motor_left_ang_vel);
}
/* Added macros */
float alpha = 0.97;
float prevAngle;
float currAngle;
float Gyro_x;
float curr_angular_rate;
float prev_angular_rate;
void startAngle(void)
{
  prevAngle = atan2(ay,az)*57.3;
}


//void getAngle(void)
//{
//  // 1 deg/s = 131 raw gyro output -- converting to degrees per second here
//  Gyro_x = (gx - gyroXCalli) / 131; //(gx - 128.1) / 131;
//  
//  Gyro_x = Gyro_x * PI/180.;
//
//  // computes integrated degree angle change from last timestep
//  float gyroPart = Gyro_x * sampling_rate / 1000;
//
//  // 57.3 converts rad to degrees
//  float accPart = atan2(ay , az) * 57.3;
//  
//  // performs complimentary filter to compute angle with alpha = 0.97
//  currAngle = alpha * (prevAngle - gyroPart) + (1 - alpha) * accPart;
//
//  /* Tried using low pass deriv to get angular rate data -- not as good as gyro */
////  curr_angular_rate = 30*currAngle - 30*prevAngle + 0.8607*prev_angular_rate;
////  prev_angular_rate = curr_angular_rate;
//  
//  prevAngle = currAngle;
//}

/* Error and Ref variables */
float e_p, e_d;
float e_p_prev = 0;
float e_d_prev = 0;
float refAngle = 0, refAngularRate = 0;
/* PD Balance Control Gains */
//double kp_balance = 29, kd_balance = 3.5;
/* Bedillion Given Gains */
//double kp_balance = 90.8, kd_balance = 1.22;
double balanceCmd;

KalmanFilter kalmanfilter;
double kp_balance = 50, kd_balance = .7;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
float kalmanfilter_angle;
float kalmanfilter_angle_prev = 0;
float kalmanfilter_angle_dot;

///////////////////////////////////////////////////////////////////////////////////////////////////

//Parameters
const int aisPin  = A0;
const int numReadings  = 5;
float readings [numReadings];
int readIndex  = 0;
float total  = 0;

//Variables
int aisVal  = 0;

float smooth(float val) { /* function smooth */
  ////Perform average on sensor readings
  float average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  readings[readIndex] = val;
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

  return average;
}

/////////////////////////////////////////////////////////////////////////////////////

void getAngle(void)
{
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  kalmanfilter_angle_dot = (kalmanfilter_angle - kalmanfilter_angle_prev) / dt;
  kalmanfilter_angle_dot = smooth(kalmanfilter_angle_dot);
  kalmanfilter_angle_prev = kalmanfilter_angle;
  
}

float motorCmd;

// variable to account for motor offset
double deadZoneOffset = 18.3;


///////////////////////////////////////////////////////////////////////////
/////////////////////////////FUZZY LOGIC CONTROL START/////////////////////
///////////////////////////////////////////////////////////////////////////

//start with helper functions for curves to define gaussian and sigmoid

//helper function for sigmoid point evaulation
float sigmoid(float x, float c, float a)
{
  float f = 1 / (1 + (exp(-a*(x-c))));
  return f;
}

//helper function for gaussian point evaulation
float gaussian(float x, float c, float sd)
{
  float f = pow(2.718, ((-0.5) * sq((x-c)/sd)));
  return f;
}

//output characteristics
int NB = 255; 
int NM = 120; //-150; 
int NS = 40; //-40; 
int Z = 0;
int PS = -40; //40; 
int PM = -120; //150; 
int PB = -255; 

//control rule table for inputs of theta and theta dot
BLA::Matrix<7, 7> control_rule = {NB, NB, NB, NM, NS, NS, Z,
    NB, NB, NM, NM, NS, Z, PS,
    NB, NB, NM, NS, Z, PS, PM,
    NB, NM, NS, Z, PS, PM, PB,
    NM, NS, Z, PS, PM, PB, PB,
    NS, Z, PS, PM, PM, PB, PB,
    Z, PS, PS, PM, PB, PB, PB};
    
//define motor contraints
int vMax = 7; int vMin = 7;

// define the fuzzification constants for the input curves

//CONSTANTS FOR PSI ERROR
int c_NB_psi = -20; int a_NB_psi = -1;   //sigmoid
int c_NM_psi = -12; int sd_NM_psi= 3;  //gaussian
int c_NS_psi= -3; int sd_NS_psi = 2;  //gaussian
int c_Z_psi = 0; int sd_Z_psi = 1;   //gaussian
int c_PS_psi = 3; int sd_PS_psi = 2;   //gaussian
int c_PM_psi = 12; int sd_PM_psi = 3;  //gaussian
int c_PB_psi = 20; int a_PB_psi = 1;    //sigmoid

//CONSTANTS FOR PSI DOT ERROR
int c_NB_psid = -2; int a_NB_psid = -1;   //sigmoid
int c_NM_psid = -.75; int sd_NM_psid = .25;  //gaussian
int c_NS_psid = -.25; int sd_NS_psid = .25;  //gaussian
int c_Z_psid = 0; int sd_Z_psid = .25;   //gaussian
int c_PS_psid = .25; int sd_PS_psid = .25;   //gaussian
int c_PM_psid = .75; int sd_PM_psid = .25;  //gaussian
int c_PB_psid = 2; int a_PB_psid = 1;    //sigmoid


void balanceFIS(void)
{
  //get error in psi -- reading in degrees!
  float e_psi = (-2.0 - kalmanfilter_angle);
  //get error in psi dot -- convert reading from degrees to radians per second for ang vel table
  float kalmanfilter_angle_dot_rads = kalmanfilter_angle_dot * 3.14159 / 180.0;
  float e_psi_d = (0 - kalmanfilter_angle_dot_rads);
  
  //input curve evaluation
  //psi error
  float u_NB_psi = sigmoid(e_psi,c_NB_psi,a_NB_psi); 
  float u_NM_psi = gaussian(e_psi,c_NM_psi,sd_NM_psi); 
  float u_NS_psi = gaussian(e_psi,c_NS_psi,sd_NS_psi); 
  float u_Z_psi = gaussian(e_psi,c_Z_psi,sd_Z_psi); 
  float u_PS_psi = gaussian(e_psi,c_PS_psi,sd_PS_psi); 
  float u_PM_psi = gaussian(e_psi,c_PM_psi,sd_PM_psi);
  float u_PB_psi = gaussian(e_psi,c_PB_psi,a_PB_psi);   

  
  //array of psi errors
  float u_psi[7] = {u_NB_psi, u_NM_psi, u_NS_psi, u_Z_psi, u_PS_psi, u_PM_psi, u_PB_psi};

  //psi dot error
  float u_NB_psid = sigmoid(e_psi_d,c_NB_psid,a_NB_psid); 
  float u_NM_psid = gaussian(e_psi_d,c_NM_psid,sd_NM_psid); 
  float u_NS_psid = gaussian(e_psi_d,c_NS_psid,sd_NS_psid); 
  float u_Z_psid = gaussian(e_psi_d,c_Z_psid,sd_Z_psid); 
  float u_PS_psid = gaussian(e_psi_d,c_PS_psid,sd_PS_psid); 
  float u_PM_psid = gaussian(e_psi_d,c_PM_psid,sd_PM_psid);
  float u_PB_psid = gaussian(e_psi_d,c_PB_psid,a_PB_psid); 
  
  //array of psi dot errors
  float u_psid[7] = {u_NB_psid, u_NM_psid, u_NS_psid, u_Z_psid, u_PS_psid, u_PM_psid, u_PB_psid};

  //create matrix to store rule evaluation
  BLA::Matrix<7, 7> ruleEvals;
  //fill matrix with zeros initially
  ruleEvals.Fill(0);

  //rule evaluation from current states
  for (int i=0; i<7; i++)
  {
    //using TS system so that have continuity
    for (int j=0; j<7; j++) ruleEvals(j,i) = u_psi[i] * u_psid[j];
  }

  //defuzzification
  //define key variables
  float weightedSum = 0.0;
  float ruleEvalSum = 0.0;
  
  BLA::Matrix<7, 7> defuzzMatrix;
  for (int i=0; i<7; i++)
  {
    for (int j=0; j<7; j++)
    {
      //add to rule eval sum
      ruleEvalSum += ruleEvals(i,j);
      //add to weighted sum
      weightedSum += ruleEvals(i,j) * control_rule(i,j);      
    }
  }
  
  //get the final defuzzified output -- given in PWM!
  float defuzzOutput = weightedSum / ruleEvalSum;

  //account for deadzone and scaling factor
  if (motorCmd >= 0) motorCmd = .75*defuzzOutput + deadZoneOffset;
  else if (motorCmd < 0) motorCmd = .75*defuzzOutput - deadZoneOffset;

}



///////////////////////////////////////////////////////////////////////////
/////////////////////////////FUZZY LOGIC CONTROL END///////////////////////
///////////////////////////////////////////////////////////////////////////


float currX;
float prevX;
float currXdot;

void getX(void)
{
  if (motorCmd > 0) {
    currX = currX + ((float)right_encoder / (float)num_of_encoder_counts_per_rev) * ((float)2 * PI * wheel_rad);
  }
  if (motorCmd < 0) {
    currX = currX - ((float)right_encoder / (float)num_of_encoder_counts_per_rev) * ((float)2 * PI * wheel_rad);
  }
  currXdot = (currX - prevX)/0.005;
  prevX = currX;
}

float kp_position = 100, kd_position = 100;
float positionCmd;
void positionPID(void) 
{
  float ep_pos = 0 - currX;
  float ed_pos = 0 - currXdot;

  positionCmd = kp_position * ep_pos + kd_position * ed_pos;
  positionCmd = -positionCmd;
}

/*
mainfunc() is the function that is called at your specified sampling rate. The default 
sampling rate is 5ms. This function will be called at every sampling instance.
*/





void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 

  //have to bump down sampling rate from 5ms to 10ms for 7 membership function use!!
  static int prevTime = 0;
  Serial.println(time - prevTime);
  prevTime = time;
  
  if (time - prev_time_encoder >= encoder_count_max)
  {
    readEncoder();
    prev_time_encoder = time;
  }
  readIMU();
  /* Do not modify ends*/

  /*Write your code below*/
    /***********************/
  getAngle();

  balanceFIS();

  
  SetRightWheelSpeed(motorCmd);
  SetLeftWheelSpeed(motorCmd);
}

void loop()
{
  
}
