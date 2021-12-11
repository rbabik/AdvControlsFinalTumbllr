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

//long gyroXCalli = -380, gyroYCalli = 65, gyroZCalli = -229; // Obtained by callibrating gyroscope values
  // Original Robot
long gyroXCalli = -415, gyroYCalli = 36, gyroZCalli = -292; // Obtained by callibrating gyroscope values
  // New Robot
//long gyroXCalli = 5189, gyroYCalli = 614, gyroZCalli = 129; // Obtained by callibrating gyroscope values
long accelXCalli = 739, accelYCalli = -73, accelZCalli = 15748; // Obtained by callibrating accelerometer values

/* Motor PWM Input Values */
long motor_left = 0;
long motor_right = 0;

/* Time variables */
unsigned long time;
unsigned long prev_time_encoder = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
int sampling_rate = 5; // in milliseconds

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
double kp_balance =50, kd_balance = .7;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
float kalmanfilter_angle;
float kalmanfilter_angle_prev = 0;
float kalmanfilter_angle_dot;

void getAngle(void)
{
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  kalmanfilter_angle_dot = (kalmanfilter_angle - kalmanfilter_angle_prev) / dt;
  kalmanfilter_angle_prev = kalmanfilter_angle;
  
}

float motorCmd;


//Parameters
const int aisPin  = A0;
const int numReadings  = 10;
float readings [numReadings];
int readIndex  = 0;
float total  = 0;

//Variables
int aisVal  = 0;

float last_average = 0;
float last_val;
bool first_smooth_call = true;

float smooth(float val) { /* function smooth */
  ////Perform average on sensor readings
  float throwout_margin = 2;
  /*if(abs(last_val-val) > throwout_margin && !first_smooth_call){
    return last_average;
  }*/
  last_val = val;
  first_smooth_call = false;
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
  last_average = average;
  return average;
}

void balancePID(void)
{
  float offset = 3.84;
  e_p = -.95 - kalmanfilter_angle; //refAngle - currAngle;
  e_d = 0 - smooth(kalmanfilter_angle_dot);
  //e_d = refAngularRate - Gyro_x;
  //e_d = 0.8*((e_p - e_p_prev)/((float)sampling_rate / 1000.0)) + 0.2*e_d_prev;
  //e_p_prev = e_p;
  //e_d_prev = e_d;
  Serial.print(smooth(kalmanfilter_angle_dot));
  Serial.print("\t");
  Serial.print(kalmanfilter_angle_dot);
  Serial.print("\t");
  Serial.println(kalmanfilter_angle);
  
  balanceCmd = kp_balance * e_p + kd_balance * e_d;
  balanceCmd = -balanceCmd;
}

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

// variable to account for motor offset
double deadZoneOffset = 18.3;

//float K_dlqr[4] = { -0.0439, -.148, 2.5226, .015 };
float K_dlqr[4] = { -0.0439, -.148, 2.5226, .02 };

//float K_dlqr[4] = { -0.0118, -.8416, 51.831, 4.907 };
//float K_dlqr[4] = { -0.0439, -.048, 2.5226, .01 };
//float K_dlqr[4] = { 0, -.06, 3, .01 };

// [x_e, x_dot_e, psi_e, psi_dot_e]
float lqr_error[4];
float pwmSignal;
void balanceLQR(void)
{
  float offset = 3.84;
  lqr_error[0] = 0 - currX;
  lqr_error[1] = 0 - currXdot;
  lqr_error[2] = -0.95 - kalmanfilter_angle;
  lqr_error[3] = 0 - smooth(kalmanfilter_angle_dot);

  float motorVoltage = 0;
  for (int i=0; i<4; i++) { motorVoltage += -K_dlqr[i]*lqr_error[i];}
  //Serial.println(motorVoltage);

//  Serial.print(lqr_error[0]);
//  Serial.print("\t");
//  Serial.print(lqr_error[1]);
//  Serial.print("\t");
//  Serial.print(lqr_error[2]);
//  Serial.print("\t");
//  Serial.println(lqr_error[3]);

  pwmSignal = map(motorVoltage, -8, 8, -255, 255);
  if (pwmSignal > 255) {pwmSignal = 255;}
  if (pwmSignal < -255) {pwmSignal = -255;}
}

void mainfunc()
{
  /* Do not modify begins*/
  sei();
  time = millis(); 
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
  Serial.print(kalmanfilter_angle_dot);
  Serial.print("\t");
  Serial.println(kalmanfilter_angle);
  
  //Serial.print("\t");
  /* Apply PD control to balance robot */
  
  //balancePID();
  float margin = .1;
  if(abs(kalmanfilter_angle) <= margin){
    balanceCmd = 0;
  }
  getX();
  //positionPID();
  balanceLQR();
  motorCmd = pwmSignal;//balanceCmd; //+ positionCmd;
//  if (balanceCmd > 0) {
//    motorCmd = balanceCmd + deadZoneOffset;
//  }
//  if (balanceCmd < 0) {
//    motorCmd = balanceCmd - deadZoneOffset;
//  }
  if (abs(kalmanfilter_angle) > 22) {
    motorCmd = 0;
  }
  /*
  Serial.print("\t");
  Serial.print(currX);
  Serial.print("\t");
  Serial.print(balanceCmd);
  Serial.print("\t");
  Serial.print(positionCmd);
  Serial.print("\t");
  Serial.println(motorCmd);*/
  
  SetRightWheelSpeed(motorCmd);
  SetLeftWheelSpeed(motorCmd);
}

void loop()
{

}
