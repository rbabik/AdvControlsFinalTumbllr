/* Include necessary header files */
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
long gyroXCalli = -374, gyroYCalli = 45, gyroZCalli = -251; // Obtained by callibrating gyroscope values
long accelXCalli = 739, accelYCalli = -73, accelZCalli = 15748; // Obtained by callibrating accelerometer values

///* IMU Callibration Variables */ //11_27 Kenny Calibration
//long gyroXCalli = -392, gyroYCalli = 62, gyroZCalli = -243; // Obtained by callibrating gyroscope values
//long accelXCalli = -210, accelYCalli = -94, accelZCalli = 15779; // Obtained by callibrating accelerometer values

/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 25; // in milliseconds

/* Encoder to speed measurement variables */

int encoder_count_max = 20;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

/* Robot parameters */

double wheel_rad = 0.0335; // radius of the wheel in metres
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

  /* 
   Uncomment the next two lines only once and store the callibration values in the 
   global variables defined for callibration
  */
//  Serial.println("Started Calibration");
//  callibrateGyroValues();
//  Serial.println("Finished Gyro Calli, starting Accel");
//  callibrateAccelValues();
  
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
  
//  Serial.print("Setup Done!\n");

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


/* Error and Ref variables */
float e_p = 1, e_d;
float e_p_prev = 0;
float e_d_prev = 0;
float refAngle = 0, refAngularRate = 0;
double balanceCmd;

KalmanFilter kalmanfilter;
double kp_balance = 60, kd_balance = 0.8  ;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
float kalmanfilter_angle, desired_Angle = 0;
float kalmanfilter_angle_prev = 0;
float kalmanfilter_angle_dot;

void getAngle(void)
{
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  kalmanfilter_angle_dot = (kalmanfilter_angle - kalmanfilter_angle_prev) / dt;
  kalmanfilter_angle_prev = kalmanfilter_angle;
}

//Parameters
const int aisPin  = A0;
const int numReadings  = 10;
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

void balancePID(void)
{
  e_p = desired_Angle - kalmanfilter_angle; //refAngle - currAngle;
  e_d = 0 - smooth(kalmanfilter_angle_dot);
  
  balanceCmd = kp_balance * e_p + kd_balance * e_d;
  balanceCmd = -balanceCmd;
}

float currX;
float prevX;
float currXdot;
float motorCmd;

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

// variables initialization for motor offset & matlab communication
double deadZoneOffset = 18.3;
float matlabData;
double kt = 0.11;
double R = 10;


const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator
byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered
float data_MATLAB = 0.0;

float getdatafromMATLAB() 
{

    // Function credits: https://forum.arduino.cc/index.php?topic=236162.0

    // this function takes the characters from the serial input and converts them
    // to a single floating point value using the function "atof()"
     
    // a similar approach can be used to read an integer value if "atoi()" is used

    // first read severalChars into the array inputSeveral
    inputSeveral[0] = 0; 
    byte charCount = 0;  
    byte ndx = 0;
    
    if (Serial.available() > 0) {
      long time = micros();
      while (Serial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = Serial.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0;

       // and then convert the string into a floating point number
     
      data_MATLAB = atof(inputSeveral); // atof gives 0.0 if the characters are not a valid number
      //Serial.print("Data from MATLAB -- ");
      //Serial.println(data_MATLAB, 3); // the number specifies how many decimal places
    }
}


void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 
  if (time - prev_time_encoder >= encoder_count_max) {
    readEncoder();
    prev_time_encoder = time;
  }
  readIMU();
  /* Do not modify ends*/

  /*Write your code below*/
    /***********************/
  getAngle();
  getX();
  

  if (abs(kalmanfilter_angle) > 22) {
    motorCmd = 0;
  }
  else if (kalmanfilter_angle <= 0.2 && kalmanfilter_angle >= -0.2) {
    motorCmd = 0; 
  }
  else {
//   Send states to matlab
    Serial.print(currX);
    Serial.print("\t");
    Serial.print(currXdot);
    Serial.print("\t");
    Serial.print(kalmanfilter_angle);
    Serial.print("\t");
    Serial.print(kalmanfilter_angle_dot);
    Serial.print("\t");
    Serial.print(motorCmd);
    Serial.print("\t");
    Serial.print(matlabData);
    Serial.println("\t");
  
    // Receive serial data (Uopt) from Matlab
    getdatafromMATLAB();
    motorCmd = data_MATLAB * (255./12.);
//    if(Serial.available()>0) // if there is data to read
//    {
//      matlabData = Serial.parseFloat(); // read data
//      motorCmd = matlabData * (255./12.); //change voltage to PWM
//    //    motorCmd = matlabData;
//    }
  
    // Inner successive loop
//    while((e_p/desired_Angle) > 0.1) {
//        desired_Angle = data_MATLAB;
//        balancePID();
//        motorCmd = balanceCmd;
//        SetRightWheelSpeed(motorCmd); // This works
//        SetLeftWheelSpeed(motorCmd);
//    }
  }
  SetRightWheelSpeed(motorCmd);
  SetLeftWheelSpeed(motorCmd);
}

void loop()
{

}
